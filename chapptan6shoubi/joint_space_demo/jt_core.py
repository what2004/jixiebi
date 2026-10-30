#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
jt_core.py —— 关节空间点到点轨迹的数学核心（纯 numpy，不 import ROS，不经过 MoveIt）

职责：关节顺序与限幅、关节空间线性插值、时间参数化（余弦律 + 累积弧长 + 速度限幅）、
      限幅校验，以及把全流程串起来的 build_plan()。

与 chapttan5/tip_cartesian_demo/tip_core.py 同构：plan_joint_segment / time_parameterize
沿用那边的写法；唯一区别是本文件没有笛卡尔直线与 IK —— 只做"已知初末关节角"的规划。

关节顺序与限幅抄自 chapttan6_ws/src/shoubi_description/urdf/arm.urdf.xacro 与
hand_l20.urdf.xacro（arm 7 关节 / hand 21 关节）。改了 URDF 限幅请同步下面的 LIMITS。
"""
import math
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np

# ==================== 关节顺序（与 URDF / SRDF 一致） ====================
JOINTS_ARM = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']

JOINTS_HAND = [
    'index_mcp_roll', 'index_mcp_pitch', 'index_pip', 'index_dip',
    'middle_mcp_roll', 'middle_mcp_pitch', 'middle_pip', 'middle_dip',
    'ring_mcp_roll', 'ring_mcp_pitch', 'ring_pip', 'ring_dip',
    'pinky_mcp_roll', 'pinky_mcp_pitch', 'pinky_pip', 'pinky_dip',
    'thumb_cmc_yaw', 'thumb_cmc_roll', 'thumb_cmc_pitch', 'thumb_mcp', 'thumb_dip',
]

GROUPS = {'arm': JOINTS_ARM, 'hand': JOINTS_HAND}

# ==================== 限幅（来自 URDF，单位 rad / rad·s^-1） ====================
# joint -> (lower, upper, max_velocity)
LIMITS: Dict[str, Tuple[float, float, float]] = {
    # ---------- arm ----------
    'joint1': (-3.1416, 3.1416, 2.0),
    'joint2': (-1.8326, 1.8326, 2.0),
    'joint3': (-3.1416, 3.1416, 2.0),
    'joint4': (0.0, 2.9671, 2.0),          # 只能正向弯
    'joint5': (-3.1416, 3.1416, 2.0),
    'joint6': (-1.8326, 1.8326, 2.0),
    'joint7': (-1.5708, 1.5708, 2.0),
    # ---------- hand ----------
    'index_mcp_roll': (-0.17, 0.17, 1.0),
    'index_mcp_pitch': (0.0, 1.4, 1.0),
    'index_pip': (0.0, 1.57, 1.0),
    'index_dip': (0.0, 1.4, 1.0),
    'middle_mcp_roll': (-0.17, 0.17, 1.0),
    'middle_mcp_pitch': (0.0, 1.4, 1.0),
    'middle_pip': (0.0, 1.57, 1.0),
    'middle_dip': (0.0, 1.4, 1.0),
    'ring_mcp_roll': (-0.17, 0.17, 1.0),
    'ring_mcp_pitch': (0.0, 1.4, 1.0),
    'ring_pip': (0.0, 1.57, 1.0),
    'ring_dip': (0.0, 1.4, 1.0),
    'pinky_mcp_roll': (-0.17, 0.17, 1.0),
    'pinky_mcp_pitch': (0.0, 1.4, 1.0),
    'pinky_pip': (0.0, 1.57, 1.0),
    'pinky_dip': (0.0, 1.4, 1.0),
    'thumb_cmc_yaw': (0.0, 1.4, 1.0),
    'thumb_cmc_roll': (0.0, 1.22, 1.0),
    'thumb_cmc_pitch': (0.0, 0.79, 1.0),
    'thumb_mcp': (0.0, 1.05, 1.0),
    'thumb_dip': (0.0, 1.22, 1.0),
}

# 加速度限幅 URDF 里没有：这是工程取值，只用于报告里的校验，不参与时间分配
AMAX = {'arm': 10.0, 'hand': 20.0}

# 余弦律 s(f) = 0.5*(1-cos(pi*f)) 的峰值系数（解析值，用来算设计峰值）
COSINE_V = math.pi / 2.0          # |ds/df|max
COSINE_A = math.pi ** 2 / 2.0     # |d2s/df2|max


# ==================== 关节与限幅查询 ====================
def joints_of(group: str) -> List[str]:
    return GROUPS[group]


def limits_of(group: str):
    """返回 (qlow, qhigh, vmax, amax)，四个都是长度 n 的 numpy 数组"""
    js = GROUPS[group]
    qlow = np.array([LIMITS[j][0] for j in js])
    qhigh = np.array([LIMITS[j][1] for j in js])
    vmax = np.array([LIMITS[j][2] for j in js])
    amax = np.full(len(js), AMAX[group])
    return qlow, qhigh, vmax, amax


def check_limits(group: str, q_start, q_goal) -> List[str]:
    """起点/目标是否在 URDF 限幅内；返回违规描述列表（空列表 = 全通过）"""
    js = GROUPS[group]
    qlow, qhigh, _, _ = limits_of(group)
    bad = []
    for i, j in enumerate(js):
        for tag, q in (('起点', float(q_start[i])), ('目标', float(q_goal[i]))):
            if q < qlow[i] - 1e-9 or q > qhigh[i] + 1e-9:
                bad.append('%s %s = %.4f 超出 [%.4f, %.4f]' % (tag, j, q, qlow[i], qhigh[i]))
    return bad


# ==================== 路径生成 ====================
def plan_joint_segment(q_from, q_to, steps):
    """关节空间线性插值：返回 steps 个点，不含起点（与 tip_core 的约定一致）"""
    q_from = np.asarray(q_from, float)
    q_to = np.asarray(q_to, float)
    return np.array([q_from + (q_to - q_from) * (k / steps) for k in range(1, steps + 1)])


# ==================== 时间参数化 ====================
def time_parameterize(q_traj, duration, vmax):
    """按关节空间累积弧长分配时间：起末速度为零，并保证最大关节速度 <= vmax

    时间律用余弦律 s = 0.5*(1-cos(pi*f))：它在 f=0 和 f=1 处 ds/df = 0，
    也就是起末速度为零；中间峰值 ds/df = pi/2。duration 是最短时长的下限，
    速度不够时按实测速度迭代放大（与 chapttan5 的实现一致）。
    """
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
        # 余弦律在两端 ds/df = 0，速度解析为 0；np.gradient 端点只能用单边差分，
        # 看不到这个零，会把起末速度估成峰值的 ~13%。这里显式置零。
        v[0] = 0.0
        v[-1] = 0.0
        mv = float(np.max(np.abs(v)))
        if mv <= vmax or duration > 120.0:
            return t, v, mv
        duration *= min(4.0, 1.05 * mv / vmax)


# ==================== 组装结果 ====================
@dataclass
class PlanResult:
    group: str
    joints: List[str]
    q: np.ndarray                     # (N,n) 关节位置轨迹
    t: np.ndarray                     # (N,)  时间戳(s)
    v: np.ndarray                     # (N,n) 关节速度
    start: np.ndarray                 # (n,)  起点
    goal: np.ndarray                  # (n,)  目标
    peak_v: np.ndarray = None         # (n,)  设计峰值速度（解析）
    peak_a: np.ndarray = None         # (n,)  设计峰值加速度（解析）
    max_vel: float = 0.0              # 采样实测的最大关节速度
    vmax_used: float = 0.0            # 实际用的速度上限
    requested_duration: float = 0.0   # 调用方要的时长
    violations: List[str] = field(default_factory=list)   # 起点/目标超限描述


def build_plan(group: str, q_from, q_to, steps: int = 60, duration: float = 1.0,
               vmax: float = None):
    """串起全流程：限幅校验 -> 关节空间线性插值 -> 时间参数化

    vmax 缺省取该组最严的 URDF 速度限幅。限幅违规不抛异常，写进 result.violations，
    由调用方（jt_run.py）决定是否中止。
    """
    js = GROUPS[group]
    n = len(js)
    q_from = np.asarray(q_from, float).reshape(-1)
    q_to = np.asarray(q_to, float).reshape(-1)
    if len(q_from) != n or len(q_to) != n:
        raise ValueError('%s 组需要 %d 个关节角，收到 %d / %d 个'
                         % (group, n, len(q_from), len(q_to)))

    qlow, qhigh, vlim, alim = limits_of(group)
    vmax_used = float(vmax) if vmax is not None else float(np.min(vlim))

    q_traj = np.vstack([q_from, plan_joint_segment(q_from, q_to, steps)])
    t, v, mv = time_parameterize(q_traj, duration, vmax_used)

    delta = np.abs(q_to - q_from)
    T = max(float(t[-1]), 1e-9)
    peak_v = delta * COSINE_V / T
    peak_a = delta * COSINE_A / (T ** 2)

    return PlanResult(group=group, joints=js, q=q_traj, t=t, v=v,
                      start=q_from, goal=q_to, peak_v=peak_v, peak_a=peak_a,
                      max_vel=mv, vmax_used=vmax_used, requested_duration=duration,
                      violations=check_limits(group, q_from, q_to))
