#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tip_run.py —— 命令行入口：大拇指指尖笛卡尔直线轨迹规划 + 打印报告 +（可选）执行

用法：
  python3 tip_run.py --offline           # 纯数学自检，不连 ROS
  python3 tip_run.py                     # 连 ROS，只规划（读 /joint_states）
  python3 tip_run.py --rviz              # 只规划，并把直线/路径点发到 RViz
  python3 tip_run.py --execute --rviz    # 真正执行，并把实际轨迹画在 RViz
  python3 tip_run.py --from-initial      # 起点用指尖初始位置(q=0)

参数：
  --end X Y Z             直线终点(m)，默认 = 预弯曲起点沿 -z 走 25mm
  --preflex a b c d e     预弯曲构型(rad)，默认 0.30 0.30 0.40 0.40 0.30
  --steps N               直线插值点数(默认 60)   --duration S 总时长(默认 4)
  --vmax V                关节速度上限(默认 0.5 rad/s)   --tol IK 容差
  --hold S                RViz 保持显示秒数(默认 10)
  --actual-csv PATH       执行结束后把实际关节数据(/joint_states 采样)导出成 CSV

算法都在 tip_core.py（纯数学），可视化在 tip_viz.py（RViz Marker），
本文件只负责命令行、ROS 收发和打印报告。
"""
import argparse
import math
import sys
import time

import numpy as np

from tip_core import (JOINTS, QLOW, QHIGH, Q_PRE, TIP_INITIAL, fk_tip, fk_pose,
                      build_plan)

ACTION = '/damuzhi_controller/follow_joint_trajectory'

ACK_TIMEOUT = 10.0        # 等控制器"确认接收目标"的最长秒数（超时多半是没连上控制器）
RESULT_EXTRA = 15.0       # 轨迹跑完后额外等待结果的秒数


def spin_future(node, fut, timeout):
    """带超时地 spin，返回 future 是否已完成（避免无限等待，只能靠 Ctrl-C）"""
    import rclpy                      # 本文件里 rclpy 一律函数内导入
    deadline = time.time() + timeout
    while rclpy.ok() and not fut.done() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    return fut.done()


def _rpy_deg(M):
    """旋转矩阵 -> (roll, pitch, yaw) 度，只为打印好看"""
    sy = math.hypot(M[0, 0], M[1, 0])
    if sy > 1e-9:
        r = math.atan2(M[2, 1], M[2, 2])
        p = math.atan2(-M[2, 0], sy)
        y = math.atan2(M[1, 0], M[0, 0])
    else:
        r, p, y = math.atan2(-M[1, 2], M[1, 1]), math.atan2(-M[2, 0], sy), 0.0
    return tuple(math.degrees(a) for a in (r, p, y))


# ==================== ROS：读状态 ====================
def read_current_joints(timeout=15.0):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    rclpy.init()
    node = Node('tip_run_read')
    box = {'q': None}

    def cb(msg):
        d = dict(zip(msg.name, msg.position))
        if all(j in d for j in JOINTS):
            box['q'] = np.array([d[j] for j in JOINTS])

    node.create_subscription(JointState, '/joint_states', cb, 10)
    t0 = time.time()
    while rclpy.ok() and box['q'] is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    return box['q']


# ==================== ROS：执行 ====================
def send_trajectory(res, rviz=False, hold=10.0, actual_csv=None):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from control_msgs.action import FollowJointTrajectory
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    from tip_viz import TipTraceNode, spin_for
    from tip_core import trace_deviation_report

    rclpy.init()
    node = Node('tip_run')
    tracer = TipTraceNode(node, res.p_start, res.p_end, res.pts[res.n_line0:], publish=rviz)
    client = ActionClient(node, FollowJointTrajectory, ACTION)
    if not client.wait_for_server(timeout_sec=5.0):
        print('!! 找不到 %s —— 确认 demo.launch.py(ros2_control) 在运行' % ACTION)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    traj = JointTrajectory()
    traj.joint_names = JOINTS
    for k in range(len(res.q)):
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in res.q[k]]
        pt.velocities = [float(x) for x in res.v[k]]
        sec = int(res.t[k])
        pt.time_from_start = Duration(sec=sec, nanosec=int((res.t[k] - sec) * 1e9))
        traj.points.append(pt)

    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    print('>>> 发送轨迹: %d 点, 总时长 %.2f s' % (len(traj.points), res.t[-1]))
    fut = client.send_goal_async(goal)
    try:
        got_ack = spin_future(node, fut, ACK_TIMEOUT)
    except KeyboardInterrupt:
        print('\n!! 手动中断（Ctrl-C）：目标已经发出去了，控制器可能正在执行。')
        print('   要停下来的话：再跑一次 --execute 覆盖，或重启 launch 终端。')
        node.destroy_node()
        rclpy.shutdown()
        return 130
    if not got_ack:
        print('!! 等了 %.0f s 也没等到控制器确认接收目标 —— 这不是轨迹的问题，是没接上控制器：' % ACK_TIMEOUT)
        print('   1) 控制器 active 吗？   ros2 control list_controllers')
        print('   2) action 在吗？        ros2 action list | grep damuzhi')
        print('   3) 上一条轨迹还在跑吗？ 等它跑完再试')
        print('   4) 食指(shizhi)的 demo 是不是也在跑？同时跑会抢 /controller_manager 和 /joint_states')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    gh = fut.result()
    if gh is None or not gh.accepted:
        print('!! 控制器拒绝了这条轨迹')
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print('    控制器已接收，开始执行 ...')
    rf = gh.get_result_async()
    try:
        done = spin_future(node, rf, float(res.t[-1]) + RESULT_EXTRA)
    except KeyboardInterrupt:
        print('\n!! 手动中断（Ctrl-C）：正在取消控制器里还没跑完的轨迹 ...')
        try:
            gh.cancel_goal_async()
            spin_for(node, 1.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        return 130
    if not done:
        print('!! 轨迹超时未结束（%.1f s）—— 控制器可能掉线了，检查 ros2 control list_controllers'
              % (float(res.t[-1]) + RESULT_EXTRA))
        node.destroy_node()
        rclpy.shutdown()
        return 1
    code = rf.result().result.error_code
    print('执行结束: error_code=%d %s' % (code, rf.result().result.error_string))
    trace_deviation_report(tracer.trace, res.p_start, res.p_end)
    if actual_csv:
        tracer.write_joint_csv(actual_csv)
    if rviz and hold > 0:
        print('保持 %.1f s 方便在 RViz 里看 ...' % hold)
        spin_for(node, hold)
    node.destroy_node()
    rclpy.shutdown()
    return 0 if code == 0 else 1


def write_csv(path, res):
    """把规划出的轨迹导出成 CSV：t, 5 个关节角, 指尖 xyz, IK 残差, 到理想直线垂距, 段标签

    头部两行注释带着 P0/P1，方便 tip_plot.py 直接画理想直线。"""
    from tip_core import line_deviation
    with open(path, 'w') as f:
        f.write('# P0_m=%.6f,%.6f,%.6f\n' % tuple(res.p_start))
        f.write('# P1_m=%.6f,%.6f,%.6f\n' % tuple(res.p_end))
        f.write('t,q0,q1,q2,q3,q4,x_m,y_m,z_m,resid_mm,dev_mm,segment\n')
        for k in range(len(res.q)):
            f.write('%.4f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.5f,%.5f,%s\n'
                    % (res.t[k], *res.q[k], *res.pts[k], res.resid[k] * 1000.0,
                       line_deviation(res.pts[k], res.p_start, res.p_end) * 1000.0, res.tags[k]))
    print('已导出 CSV: %s   (可用 tip_plot.py --csv %s 画图，或 PlotJuggler 里 Load data file)'
          % (path, path))


def show_plan_in_rviz(res, hold=10.0):
    import rclpy
    from rclpy.node import Node
    from tip_viz import TipTraceNode, spin_for

    rclpy.init()
    node = Node('tip_run_plan')
    TipTraceNode(node, res.p_start, res.p_end, res.pts[res.n_line0:], publish=True)
    print('已把规划结果发到 RViz，保持 %.0f s ...' % hold)
    spin_for(node, hold)
    node.destroy_node()
    rclpy.shutdown()


# ==================== 打印报告 ====================
def print_report(res, q_now=None):
    print('%-70s' % ('-' * 70))
    print('起点位姿(笛卡尔段开始): 位置 = [%.3f, %.3f, %.3f] mm, rpy = (%.2f, %.2f, %.2f) deg'
          % (*(res.pose_start[:3, 3] * 1000), *_rpy_deg(res.pose_start)))
    print('终点位姿(笛卡尔段结束): 位置 = [%.3f, %.3f, %.3f] mm, rpy = (%.2f, %.2f, %.2f) deg'
          % (*(res.pose_end[:3, 3] * 1000), *_rpy_deg(res.pose_end)))
    print('段B 笛卡尔直线: 完成度 fraction = %.4f  (MoveIt 的 fraction 就是这个含义)' % res.fraction)
    if res.fraction < 0.999:
        print('!! 直线没走完，已中途截断：终点超出可达范围或受关节限位阻挡')
        print('   解决：把 --end 拉近；或用 tip_scan_directions.py 找可行方向')
    print('IK 残差: max = %.4f mm, mean = %.4f mm' % (res.resid.max() * 1000, res.resid.mean() * 1000))
    print('时间参数化: %d 点, 总时长 %.2f s, 最大关节速度 %.3f rad/s' % (len(res.q), res.t[-1], res.max_vel))
    print('直线度(规划): 指尖到理想直线最大垂距 = %.4f mm' % (res.max_line_dev * 1000))
    print('%-70s' % ('-' * 70))
    print('指尖坐标随轨迹变化 (base_link, mm):')
    print('  %6s %10s %10s %10s  %-8s %s' % ('t[s]', 'x', 'y', 'z', '段', 'IK残差[mm]'))
    step = max(1, len(res.q) // 14)
    for k in range(0, len(res.q), step):
        p = res.pts[k] * 1000.0
        print('  %6.2f %10.3f %10.3f %10.3f  %-8s %.4f'
              % (res.t[k], p[0], p[1], p[2], res.tags[k], res.resid[k] * 1000))
    pL = res.pts[-1] * 1000.0
    print('  %6.2f %10.3f %10.3f %10.3f  %-8s %.4f  <- 结束'
          % (res.t[-1], pL[0], pL[1], pL[2], res.tags[-1], res.resid[-1] * 1000))
    print('终点实际到达 = [%.3f, %.3f, %.3f] mm   目标 = [%.3f, %.3f, %.3f] mm   误差 = %.4f mm'
          % (*pL, *(res.p_end * 1000), res.end_err * 1000))


# ==================== 入口 ====================
def main():
    ap = argparse.ArgumentParser(description='大拇指指尖笛卡尔直线轨迹（core/run/viz 拆分版）')
    ap.add_argument('--offline', action='store_true', help='纯数学自检，不连 ROS')
    ap.add_argument('--execute', action='store_true', help='真的把轨迹发给控制器')
    ap.add_argument('--rviz', action='store_true', help='发 Marker 到 RViz')
    ap.add_argument('--from-initial', action='store_true', help='起点用指尖初始位置(q=0)')
    ap.add_argument('--preflex', type=float, nargs=5, default=list(Q_PRE),
                    metavar=('A', 'B', 'C', 'D', 'E'), help='预弯曲构型(rad)')
    ap.add_argument('--end', type=float, nargs=3, default=None, metavar=('X', 'Y', 'Z'),
                    help='直线终点(m)，默认=预弯曲起点沿 -z 走 25mm')
    ap.add_argument('--steps', type=int, default=60, help='直线插值点数(默认 60)')
    ap.add_argument('--duration', type=float, default=4.0, help='总时长(s，默认 4)')
    ap.add_argument('--vmax', type=float, default=0.5, help='关节速度上限(rad/s)')
    ap.add_argument('--tol', type=float, default=1e-5, help='IK 残差容差(m)')
    ap.add_argument('--hold', type=float, default=10.0, help='RViz 保持显示(s)')
    ap.add_argument('--csv', type=str, default=None, help='把规划轨迹导出成 CSV（t + 5 关节角 + 指尖 xyz + 垂距）')
    ap.add_argument('--actual-csv', type=str, default=None, metavar='PATH',
                    help='执行结束后把实际关节数据（/joint_states 采样）导出成 CSV')
    ap.add_argument('--ik', type=str, default='closed', choices=['closed', 'hybrid', 'numeric'],
                    help='逆解方式: closed=解析闭式(默认, 最快最准) / hybrid=闭式+精修 / numeric=原数值迭代')
    args = ap.parse_args()

    q_pre = np.array(args.preflex, dtype=float)
    p_pre = fk_tip(q_pre)
    p_start = TIP_INITIAL.copy() if args.from_initial else p_pre
    p_end = (p_start + np.array([0.0, 0.0, -0.025])) if args.end is None else np.array(args.end, float)

    print('=' * 74)
    print('大拇指指尖笛卡尔直线轨迹  (core/run/viz 拆分版，不经过 MoveIt)')
    print('=' * 74)
    print('关节顺序            : %s' % ' / '.join(JOINTS))
    print('指尖初始位置(q=0)   : [%.5f, %.5f, %.5f] m' % tuple(TIP_INITIAL))
    print('预弯曲构型 q_pre    : [%s]' % ', '.join('%.2f' % x for x in q_pre))
    print('预弯曲后指尖位置    : [%.5f, %.5f, %.5f] m' % tuple(p_pre))
    print('直线起点 P0         : [%.5f, %.5f, %.5f] m' % tuple(p_start))
    print('直线终点 P1         : [%.5f, %.5f, %.5f] m' % tuple(p_end))
    print('直线长度            : %.2f mm' % (np.linalg.norm(p_end - p_start) * 1000))
    print('-' * 74)

    if args.offline:
        q_now = np.zeros(5)
        print('[offline] 用 q=0 作为当前状态')
    else:
        q_now = read_current_joints()
        if q_now is None:
            print('!! 收不到 /joint_states：先 ros2 launch moveit_config demo.launch.py')
            print('   或先用 --offline 做纯数学自检')
            return 1
        print('当前关节角(rad)     : [%s]' % ', '.join('%.4f' % x for x in q_now))
        print('当前指尖位置        : [%.5f, %.5f, %.5f] m' % tuple(fk_tip(q_now)))

    ik_func = None
    if args.ik != 'numeric':
        try:
            from tip_ik import make_ik
            ik_func = make_ik(args.ik)
            print('逆解方式            : %s (解析闭式)' % args.ik if args.ik == 'closed' else '逆解方式            : %s' % args.ik)
        except Exception as e:
            print('!! 解析逆解不可用(%s)，回退数值迭代' % e)

    res = build_plan(q_now, p_start, p_end, q_pre=q_pre, steps=args.steps,
                     duration=args.duration, vmax=args.vmax, tol=args.tol,
                     from_initial=args.from_initial, ik_func=ik_func)
    print_report(res, q_now=q_now)
    if args.csv:
        write_csv(args.csv, res)

    print('-' * 74)
    if args.offline:
        print('[offline] 数学自检完成，未发指令。去掉 --offline 即可连 ROS。')
    elif args.execute:
        if res.fraction < 0.999:
            print('完成度不足 100%%，为安全起见不执行。')
            return 1
        return send_trajectory(res, rviz=args.rviz, hold=args.hold, actual_csv=args.actual_csv)
    elif args.rviz:
        show_plan_in_rviz(res, hold=args.hold)
    else:
        print('(只规划未执行；加 --execute 执行，加 --rviz 在 RViz 里看)')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\n已手动中断（Ctrl-C）。若此时拇指还在动，用 --execute 再跑一次即可覆盖，或 Ctrl-C 掉 launch 终端。')
        sys.exit(130)
