#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
jt_run.py —— 命令行入口：关节空间点到点轨迹 + 校验报告 +（可选）写文件

只干一件事：给定同一组关节的初末角度，生成一条关节空间轨迹。
不做笛卡尔、不做 IK、不连 MoveIt、不 import ROS —— 全程离线可跑。

用法：
  python3 jt_run.py --group arm  --start 0 0 0 0 0 0 0 \
                              --goal 0.119 -0.109 0.119 1.026 0.730 0.287 0.00
  python3 jt_run.py --group hand --start-deg 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 \
                              --goal-deg  ...
  python3 jt_run.py --group both --start <28个> --goal <28个> --sync

参数：
  --group    arm | hand | both（both = 前 7 个给 arm，后 21 个给 hand）
  --start/--goal        初末关节角(rad)；--start-deg/--goal-deg 则按度输入
  --steps N              插值点数（默认 60，轨迹共 61 个点）
  --duration S           期望总时长(默认 1.0 s)；速度不够会自动放长
  --vmax V               速度上限，缺省取该组最严的 URDF 限幅
  --sync                 both 时让两组用同一总时长（同起同落）
  --outdir DIR           输出目录（默认 项目根/traj）
  --csv                  除 JSON 外再写一份 CSV
  --no-write             只打印报告，不写文件

输出文件：
  <outdir>/<group>_traj.json             JointTrajectory 结构：joint_names + points
  <outdir>/<group>_traj.json.meta.json   本次用的初末角度/限幅/峰值记录
  <outdir>/arm_hand_combined.json        both + --sync 时的 28 关节合并文件

算法都在 jt_core.py（纯 numpy），本文件只负责命令行、打印报告和写文件。
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

from jt_core import (AMAX, COSINE_A, COSINE_V, GROUPS, LIMITS, JOINTS_ARM, JOINTS_HAND,
                     build_plan, limits_of)

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUTDIR = os.path.join(os.path.dirname(HERE), 'traj')


# ==================== 写文件 ====================
def traj_to_dict(res):
    """转成 trajectory_msgs/JointTrajectory 的结构（外加 group 便于自查）"""
    return {
        'group': res.group,
        'joint_names': list(res.joints),
        'profile': 'cosine',
        'duration': float(res.t[-1]),
        'points': [{'time_from_start': float(res.t[k]),
                    'positions': [float(x) for x in res.q[k]],
                    'velocities': [float(x) for x in res.v[k]]}
                   for k in range(len(res.t))],
    }


def write_traj(res, outdir, also_csv=False):
    os.makedirs(outdir, exist_ok=True)
    path = os.path.join(outdir, '%s_traj.json' % res.group)
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(traj_to_dict(res), f, indent=1)

    qlow, qhigh, vlim, alim = limits_of(res.group)
    meta = {
        'generated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
        'command': ' '.join(sys.argv),
        'group': res.group,
        'joint_names': list(res.joints),
        'start_positions': [float(x) for x in res.start],
        'goal_positions': [float(x) for x in res.goal],
        'delta': [float(x) for x in (res.goal - res.start)],
        'profile': 'cosine  (s = 0.5*(1-cos(pi*f))，按关节空间累积弧长分配时间)',
        'steps': len(res.t) - 1,
        'num_points': len(res.t),
        'requested_duration_s': float(res.requested_duration),
        'duration_s': float(res.t[-1]),
        'vmax_used': float(res.vmax_used),
        'max_vel_measured': float(res.max_vel),
        'limits': {j: {'lower': float(qlow[i]), 'upper': float(qhigh[i]),
                       'velocity': float(vlim[i]), 'acceleration': float(alim[i])}
                   for i, j in enumerate(res.joints)},
        'peak_v': {j: float(res.peak_v[i]) for i, j in enumerate(res.joints)},
        'peak_a': {j: float(res.peak_a[i]) for i, j in enumerate(res.joints)},
        'limits_source': 'chapttan6_ws/src/shoubi_description/urdf/arm.urdf.xacro, '
                         'hand_l20.urdf.xacro',
        'output_file': os.path.basename(path),
    }
    with open(path + '.meta.json', 'w', encoding='utf-8') as f:
        json.dump(meta, f, indent=1, ensure_ascii=False)
    print('已写出: %s  (%d 点, %.4f s)  + .meta.json' % (path, len(res.t), res.t[-1]))

    if also_csv:
        cpath = os.path.join(outdir, '%s_traj.csv' % res.group)
        with open(cpath, 'w', encoding='utf-8') as f:
            f.write('time,' + ','.join(res.joints)
                    + ',' + ','.join(j + '_vel' for j in res.joints) + '\n')
            for k in range(len(res.t)):
                f.write('%.6f,' % res.t[k]
                        + ','.join('%.6f' % x for x in res.q[k])
                        + ',' + ','.join('%.6f' % x for x in res.v[k]) + '\n')
        print('已写出: %s' % cpath)
    return path


def write_combined(ra, rh, outdir):
    """两组时间栅格一致时，合并成一个 28 关节文件"""
    path = os.path.join(outdir, 'arm_hand_combined.json')
    names = list(ra.joints) + list(rh.joints)
    pts = [{'time_from_start': float(ra.t[k]),
            'positions': [float(x) for x in ra.q[k]] + [float(x) for x in rh.q[k]],
            'velocities': [float(x) for x in ra.v[k]] + [float(x) for x in rh.v[k]]}
           for k in range(len(ra.t))]
    with open(path, 'w', encoding='utf-8') as f:
        json.dump({'group': 'arm+hand', 'joint_names': names, 'profile': 'cosine',
                   'duration': float(ra.t[-1]), 'points': pts}, f, indent=1)
    meta = {
        'generated_at': time.strftime('%Y-%m-%d %H:%M:%S'),
        'note': 'arm 与 hand 合并（--sync 使两组时间栅格一致）',
        'command': ' '.join(sys.argv),
        'joint_names': names,
        'start_positions': [float(x) for x in ra.start] + [float(x) for x in rh.start],
        'goal_positions': [float(x) for x in ra.goal] + [float(x) for x in rh.goal],
        'source_files': ['arm_traj.json', 'hand_traj.json'],
        'profile': 'cosine',
        'duration_s': float(ra.t[-1]),
        'num_points': len(pts),
        'output_file': 'arm_hand_combined.json',
    }
    with open(path + '.meta.json', 'w', encoding='utf-8') as f:
        json.dump(meta, f, indent=1, ensure_ascii=False)
    print('已写出: %s  (%d 关节, %d 点)  + .meta.json' % (path, len(names), len(pts)))
    return path


# ==================== 打印报告 ====================
def print_report(res):
    qlow, qhigh, vlim, alim = limits_of(res.group)
    n = len(res.joints)
    T = float(res.t[-1])
    print('%-70s' % ('-' * 70))
    print('组 %-5s 关节数 %2d   %d 个点   总时长 %.4f s   时间律 余弦律(起末速度为零)'
          % (res.group, n, len(res.t), T))
    if res.requested_duration <= 0:
        print('           (未指定时长，已按速度限幅取最短)')
    elif abs(T - res.requested_duration) > 1e-6:
        print('           (要求 %.4f s，受速度限幅自动放长到 %.4f s)' % (res.requested_duration, T))
    print('速度上限 %.3f rad/s（该组最严的 URDF 限幅）   实测最大关节速度 %.4f rad/s'
          % (res.vmax_used, res.max_vel))
    if res.violations:
        print('!! 起点/目标超限，共 %d 处：' % len(res.violations))
        for v in res.violations:
            print('   ' + v)
    else:
        print('起点/目标限幅检查: 通过')
    print('%-70s' % ('-' * 70))
    print('逐关节校验（设计峰值 vs 限幅；设计峰值按余弦律解析算，与实测吻合到 %.1e 量级）'
          % 1e-3)
    print('  %-18s %9s %8s %7s %8s %7s %s'
          % ('joint', 'd(rad)', 'v峰值', 'v限幅', 'a峰值', 'a限幅', '余量'))
    ok_all = True
    for i, j in enumerate(res.joints):
        d = abs(res.goal[i] - res.start[i])
        ok = res.peak_v[i] <= vlim[i] + 1e-6 and res.peak_a[i] <= alim[i] + 1e-6
        ok_all = ok_all and ok
        mv_ = min(vlim[i] / res.peak_v[i] if res.peak_v[i] > 1e-9 else float('inf'),
                  alim[i] / res.peak_a[i] if res.peak_a[i] > 1e-9 else float('inf'))
        print('  %-18s %9.4f %8.4f %7.2f %8.3f %7.1f %s'
              % (j, d, res.peak_v[i], vlim[i], res.peak_a[i], alim[i],
                 ('%.1fx' % mv_) if mv_ != float('inf') else '-'))
    print('  结论: %s' % ('全部满足限幅' if ok_all else '!! 有限幅被突破'))
    err = float(np.max(np.abs(res.q[-1] - res.goal)))
    print('末端与目标最大偏差: %.3e rad   起末速度: |v0|=%.2e, |vT|=%.2e rad/s'
          % (err, float(np.max(np.abs(res.v[0]))), float(np.max(np.abs(res.v[-1])))))
    d_all = np.linalg.norm(res.goal - res.start)
    print('关节空间位移(欧氏): %.4f rad   峰值速度解析/实测: %.4f / %.4f'
          % (d_all, float(np.max(res.peak_v)), res.max_vel))
    return ok_all


# ==================== 入口 ====================
def pick_values(rad, deg, n, label):
    v = rad if rad is not None else deg
    if v is None:
        print('错误: 缺少 --%s（弧度）或 --%s-deg（度）' % (label, label))
        sys.exit(2)
    if len(v) != n:
        print('错误: --%s 需要 %d 个数，收到 %d 个' % (label, n, len(v)))
        sys.exit(2)
    return np.array([math.radians(x) for x in v] if (rad is None and deg is not None) else v,
                    dtype=float)


def main():
    ap = argparse.ArgumentParser(
        description='关节空间点到点轨迹（core/run 拆分版，不经过 MoveIt）')
    ap.add_argument('--group', choices=['arm', 'hand', 'both'], required=True)
    ap.add_argument('--start', type=float, nargs='+', help='起点关节角(rad)')
    ap.add_argument('--goal', type=float, nargs='+', help='目标关节角(rad)')
    ap.add_argument('--start-deg', type=float, nargs='+', help='起点关节角(度)')
    ap.add_argument('--goal-deg', type=float, nargs='+', help='目标关节角(度)')
    ap.add_argument('--steps', type=int, default=60, help='插值点数(默认 60，轨迹共 61 点)')
    ap.add_argument('--duration', type=float, default=1.0,
                    help='期望总时长(s，默认 1.0；给 0 表示只求速度允许的最短时长)')
    ap.add_argument('--vmax', type=float, default=None, help='速度上限(rad/s)，缺省取最严限幅')
    ap.add_argument('--sync', action='store_true', help='both 时两组用同一总时长')
    ap.add_argument('--outdir', default=DEFAULT_OUTDIR, help='输出目录(默认 项目根/traj)')
    ap.add_argument('--csv', action='store_true', help='除 JSON 外再写一份 CSV')
    ap.add_argument('--no-write', action='store_true', help='只打印报告，不写文件')
    args = ap.parse_args()

    both = (args.group == 'both')
    n_total = len(JOINTS_ARM) + len(JOINTS_HAND) if both else len(GROUPS[args.group])
    q0 = pick_values(args.start, args.start_deg, n_total, 'start')
    q1 = pick_values(args.goal, args.goal_deg, n_total, 'goal')

    print('=' * 70)
    print('关节空间点到点轨迹  (core/run 拆分版，不经过 MoveIt)')
    print('=' * 70)
    print('组            : %s' % args.group)
    print('插值点数      : %d (轨迹共 %d 点)   期望时长: %.2f s'
          % (args.steps, args.steps + 1, args.duration))
    print('-' * 70)

    if both:
        qa0, qa1 = q0[:len(JOINTS_ARM)], q1[:len(JOINTS_ARM)]
        qh0, qh1 = q0[len(JOINTS_ARM):], q1[len(JOINTS_ARM):]
        print('arm  起点->目标 (rad):')
        for j, a, b in zip(JOINTS_ARM, qa0, qa1):
            print('   %-18s %8.4f -> %8.4f   d = %+.4f' % (j, a, b, b - a))
        print('hand 起点->目标 (rad):')
        for j, a, b in zip(JOINTS_HAND, qh0, qh1):
            print('   %-18s %8.4f -> %8.4f   d = %+.4f' % (j, a, b, b - a))

        seed = 1e-3 if args.duration <= 0 else args.duration
        ra = build_plan('arm', qa0, qa1, steps=args.steps,
                        duration=seed, vmax=args.vmax)
        rh = build_plan('hand', qh0, qh1, steps=args.steps,
                        duration=seed, vmax=args.vmax)
        if args.sync:
            Ts = max(float(ra.t[-1]), float(rh.t[-1]))
            print('--sync: 两组统一到 %.4f s（取两者中较长的）' % Ts)
            ra = build_plan('arm', qa0, qa1, steps=args.steps, duration=Ts, vmax=args.vmax)
            rh = build_plan('hand', qh0, qh1, steps=args.steps, duration=Ts, vmax=args.vmax)

        ok_a, ok_h = print_report(ra), print_report(rh)
        bad = (ra.violations + rh.violations)
        if not args.no_write and not bad:
            write_traj(ra, args.outdir, also_csv=args.csv)
            write_traj(rh, args.outdir, also_csv=args.csv)
            if args.sync:
                write_combined(ra, rh, args.outdir)
            else:
                print('(未加 --sync，两组时长不同，没生成 28 关节合并文件)')
        print('=' * 70)
        return 0 if (ok_a and ok_h and not bad) else 1

    # ---- 单组 ----
    r = build_plan(args.group, q0, q1, steps=args.steps,
                   duration=1e-3 if args.duration <= 0 else args.duration, vmax=args.vmax)
    print('%s 起点->目标 (rad):' % args.group)
    for j, a, b in zip(r.joints, r.start, r.goal):
        print('   %-18s %8.4f -> %8.4f   d = %+.4f' % (j, a, b, b - a))
    ok = print_report(r)
    if r.violations:
        print('目标或起点超限，未写文件。')
        print('=' * 70)
        return 1
    if not args.no_write:
        write_traj(r, args.outdir, also_csv=args.csv)
    print('=' * 70)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())