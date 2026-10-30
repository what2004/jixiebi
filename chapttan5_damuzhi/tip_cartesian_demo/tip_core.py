#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tip_core.py —— 大拇指指尖笛卡尔轨迹的数学核心（纯 numpy，不 import ROS）

职责：正运动学 FK、位置优先 IK、关节空间插值、笛卡尔直线插值、
      时间参数化、直线度/实际轨迹偏差评估，以及串起全流程的 build_plan()。

关节顺序：thumb_cmc_yaw / thumb_cmc_roll / thumb_cmc_pitch / thumb_mcp / thumb_dip
          （与 ros2_controllers.yaml 里 damuzhi_controller 的 joints 顺序一致）
坐标系  ：base_link（与 world 重合，URDF 里 fixed_base 把 base_link 焊在 world 上）
几何来源：damuzhi_description/urdf/hand.urdf.xacro（关节 origin/rpy/axis 逐字抄自 URDF）
指尖    ：thumb_tip（thumb_distal 系内偏移 (-13.946, 0, 31.885) mm）
"""
import math
from dataclasses import dataclass, field
from typing import List

import numpy as np

# ==================== 常量（与 URDF 一致，单位 m / rad） ====================
JOINTS = ['thumb_cmc_yaw', 'thumb_cmc_roll', 'thumb_cmc_pitch', 'thumb_mcp', 'thumb_dip']

# 每个关节的 origin(xyz, rpy)，轴的转向都是子坐标系里的 +y
_ORIGINS = [
    ((0.0039847,             0.0050818,            0.057392),            (3.1416,            0.0,                 1.5708)),
    ((0.000784740002368223,  0.0196077603627024,   0.000308721957079858), (1.18264333118745, 0.0,                 1.57079632679503)),
    ((-0.000109690908352875, -0.00396440679180957, 0.0307945044045898),  (0.0,               0.388152995607441,  -1.57079632624948)),
    ((0.00857856380171737,   0.0,                  0.0555715948082196),  (0.0,               0.0,                 0.0)),
    ((0.0066653832125308,    0.0,                  0.0342561774516276),  (0.0,               0.0,                 0.0)),
]
TIP = np.array([-0.013946, 0.0, 0.031885])        # thumb_distal -> thumb_tip（含指尖小球）

QLOW = np.array([0.0, 0.0, 0.0, 0.0, 0.0])        # 来自 URDF 的 limit lower
QHIGH = np.array([1.4, 1.22, 0.79, 1.05, 1.22])   # 来自 URDF 的 limit upper

Q_PRE = np.array([0.30, 0.30, 0.40, 0.40, 0.30])  # 预弯曲构型（关节空间，避开全 0 的限位角点）


# ==================== 正运动学 ====================
def _T(v):
    M = np.eye(4)
    M[:3, 3] = v
    return M


def _Rx(a):
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    M[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
    return M


def _Ry(a):
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    M[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return M


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    M = np.eye(4)
    M[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return M


def _rpy(r, p, y):
    return _Rz(y) @ _Ry(p) @ _Rx(r)


def fk_pose(q):
    """5 个关节角 -> base_link 到 thumb_tip 的 4x4 位姿"""
    q = np.asarray(q, dtype=float)
    M = np.eye(4)
    for i, (xyz, rpy) in enumerate(_ORIGINS):
        M = M @ _T(np.array(xyz)) @ _rpy(*rpy) @ _Ry(q[i])
    return M @ _T(TIP)


def fk_tip(q):
    """5 个关节角 -> 指尖位置(3,)，单位 m"""
    return fk_pose(q)[:3, 3]


TIP_INITIAL = fk_tip(np.zeros(5))                 # q=0 时的指尖位置（四个关节都卡在下限）


def jac_pos(q, eps=1e-7):
    """指尖位置对 5 个关节角的雅可比(3x5)，数值差分"""
    J = np.zeros((3, 5))
    for i in range(5):
        dq = np.zeros(5)
        dq[i] = eps
        J[:, i] = (fk_tip(q + dq) - fk_tip(q - dq)) / (2 * eps)
    return J


# ==================== 逆运动学（位置优先） ====================
def ik_position(target, seed, tol=1e-5, max_iter=400, lam=3e-3, step_clip=0.2):
    """阻尼最小二乘 + 限位裁剪 + seed 决定落在哪一组解（保证轨迹连续）

    拇指 5 自由度只约束 3 维位置：多余的自由度由 seed（上一点）决定，
    所以轨迹天然连续、不会在解之间跳。"""
    q = np.clip(np.asarray(seed, dtype=float).copy(), QLOW, QHIGH)
    best, best_err = q.copy(), float(np.linalg.norm(target - fk_tip(q)))
    for _ in range(max_iter):
        e = target - fk_tip(q)
        err = float(np.linalg.norm(e))
        if err < best_err:
            best, best_err = q.copy(), err
        if err < tol:
            break
        J = jac_pos(q)
        dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(3), e)
        q = np.clip(q + np.clip(dq, -step_clip, step_clip), QLOW, QHIGH)
    return best, best_err


# ==================== 路径生成 ====================
def plan_joint_segment(q_from, q_to, steps):
    """关节空间线性插值（移动过程中指尖走自由路径）"""
    q_from = np.asarray(q_from, float)
    q_to = np.asarray(q_to, float)
    return np.array([q_from + (q_to - q_from) * (k / steps) for k in range(1, steps + 1)])


def plan_cartesian_segment(p_from, p_to, q_seed, steps, tol=1e-5, max_iter=400, ik_func=None):
    """笛卡尔直线插值 + 逐点 IK；解不出来的点就截断
    返回 (q轨迹, 残差, 指尖位置, 完成度 fraction)"""
    p_from = np.asarray(p_from, float)
    p_to = np.asarray(p_to, float)
    q = np.asarray(q_seed, float).copy()
    q_list, resid, pts = [], [], []
    ok = 0
    for k in range(1, steps + 1):
        p = p_from + (p_to - p_from) * (k / steps)
        q, _ = (ik_func or ik_position)(p, q, tol=tol)
        p_fk = fk_tip(q)                       # 残差统一用正解回读算，跟 IK 实现无关
        err = float(np.linalg.norm(p_fk - p))
        q_list.append(q.copy())
        resid.append(err)
        pts.append(p_fk)
        if err <= 1e-3:
            ok += 1
        else:
            break
    return np.array(q_list), np.array(resid), np.array(pts), ok / steps


def time_parameterize(q_traj, duration, vmax):
    """按关节空间累积弧长分配时间：起末速度为零，并保证最大关节速度 <= vmax"""
    q_traj = np.asarray(q_traj, float)
    n = len(q_traj)
    if n < 2:
        return np.zeros(n), np.zeros_like(q_traj), 0.0
    d = np.linalg.norm(np.diff(q_traj, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(d)])
    if s[-1] < 1e-12:
        s = np.linspace(0.0, 1.0, n)
    s = s / s[-1]
    # s = 0.5*(1-cos(pi*f)) 的反解 -> 两端 ds/df = 0，即起末速度为零
    frac = np.arccos(1.0 - 2.0 * s) / np.pi
    while True:
        t = duration * frac
        v = np.gradient(q_traj, t, axis=0)
        mv = float(np.max(np.abs(v)))
        if mv <= vmax or duration > 120.0:
            return t, v, mv
        duration *= min(4.0, 1.05 * mv / vmax)


# ==================== 评估 ====================
def line_deviation(p, a, b):
    """点 p 到直线段 a->b 的垂距(m)"""
    ab = np.asarray(b, float) - np.asarray(a, float)
    L = np.linalg.norm(ab)
    if L < 1e-12:
        return float(np.linalg.norm(np.asarray(p, float) - np.asarray(a, float)))
    return float(np.linalg.norm(np.cross(ab, np.asarray(p, float) - np.asarray(a, float))) / L)


def trace_deviation_report(trace, p0, p1, verbose=True):
    """用实际采到的指尖轨迹，算它到理想直线 P0->P1 的垂距：返回 dict"""
    out = {'ok': False}
    if trace is None or len(trace) < 2:
        if verbose:
            print('实际轨迹: 没采到点')
        return out
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    line = p1 - p0
    L = float(np.linalg.norm(line))
    if L < 1e-9:
        return out
    u = line / L
    start_idx = None
    for i, pt in enumerate(trace):
        if np.linalg.norm(np.asarray(pt, float) - p0) < 5e-4:
            start_idx = i
            break
    if start_idx is None:
        if verbose:
            print('实际轨迹: 没检测到指尖到达直线起点，无法评估直线度')
        return out
    seg = [np.asarray(t, float) for t in trace[start_idx:]]
    devs = []
    for pt in seg:
        v = pt - p0
        devs.append(float(np.linalg.norm(v - np.dot(v, u) * u)))
    out = {'ok': True, 'progress': float(np.dot(seg[-1] - p0, u)), 'target': L,
           'dev_max': max(devs), 'dev_mean': sum(devs) / len(devs), 'n': len(seg)}
    if verbose:
        print('实际执行轨迹: 直线段走过 %.2f mm / 目标 %.2f mm' % (out['progress'] * 1000, L * 1000))
        print('              到理想直线垂距: max = %.4f mm, mean = %.4f mm (共 %d 个采样点)'
              % (out['dev_max'] * 1000, out['dev_mean'] * 1000, out['n']))
    return out


# ==================== 把上面串起来 ====================
@dataclass
class PlanResult:
    q: np.ndarray            # (N,5) 关节轨迹
    t: np.ndarray            # (N,)  时间戳
    v: np.ndarray            # (N,5) 关节速度
    pts: np.ndarray          # (N,3) 指尖位置（正解回读）
    resid: np.ndarray        # (N,)  IK 残差
    tags: List[str] = field(default_factory=list)
    p_start: np.ndarray = None       # 直线段起点
    p_end: np.ndarray = None         # 直线段终点
    pose_start: np.ndarray = None    # 起点位姿 4x4
    pose_end: np.ndarray = None      # 终点位姿 4x4
    fraction: float = 0.0            # 直线段完成度
    n_line0: int = 0                 # 直线段在轨迹里的起始下标
    max_vel: float = 0.0
    max_line_dev: float = 0.0
    end_err: float = 0.0


def build_plan(q_now, p_start, p_end, q_pre=Q_PRE, steps=60, duration=4.0,
               vmax=0.5, tol=1e-5, from_initial=False, ik_func=None):
    """完整规划：当前构型 -> (预弯曲) -> 笛卡尔直线 -> 时间参数化"""
    q_now = np.asarray(q_now, float)
    p_start = np.asarray(p_start, float)
    p_end = np.asarray(p_end, float)

    q_traj = [q_now.copy()]
    resid = [0.0]
    pts = [fk_tip(q_now)]
    tags = ['start']

    q_seed = q_now.copy()
    if not from_initial and np.linalg.norm(np.asarray(q_pre, float) - q_now) > 1e-3:
        nA = max(10, int(np.linalg.norm(np.asarray(q_pre, float) - q_now) / 0.03))
        qA = plan_joint_segment(q_now, q_pre, nA)
        q_traj += list(qA)
        resid += [0.0] * len(qA)
        pts += [fk_tip(q) for q in qA]
        tags += ['preflex'] * len(qA)
        q_seed = qA[-1]

    if from_initial:
        q0, e0 = (ik_func or ik_position)(p_start, q_seed, tol=tol)
        q_traj.append(q0)
        resid.append(e0)
        pts.append(fk_tip(q0))
        tags.append('line')
        q_seed = q0

    n_line0 = len(q_traj)
    qB, rB, pB, fraction = plan_cartesian_segment(p_start, p_end, q_seed, steps, tol, ik_func=ik_func)
    q_traj += list(qB)
    resid += list(rB)
    pts += list(pB)
    tags += ['line'] * len(qB)

    q_all = np.array(q_traj)
    pts_all = np.array(pts)
    resid_all = np.array(resid)
    t, v, mv = time_parameterize(q_all, duration, vmax)

    devs = [line_deviation(p, p_start, p_end) for p in pts_all[n_line0:]] if len(pts_all) > n_line0 else [0.0]
    q_line_start = q_all[n_line0 - 1] if n_line0 > 0 else q_all[0]
    return PlanResult(q=q_all, t=t, v=v, pts=pts_all, resid=resid_all, tags=tags,
                      p_start=p_start, p_end=p_end,
                      pose_start=fk_pose(q_line_start), pose_end=fk_pose(q_all[-1]),
                      fraction=fraction, n_line0=n_line0,
                      max_vel=mv, max_line_dev=max(devs),
                      end_err=float(np.linalg.norm(pts_all[-1] - p_end)))
