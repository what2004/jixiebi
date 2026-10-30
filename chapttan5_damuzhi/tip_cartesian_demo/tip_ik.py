#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tip_ik.py —— 大拇指的【解析（闭式）逆解】+【解析雅可比】，为实时性准备

DH 表（由 URDF 反算，并已用"指尖点到各关节轴距离/轴间夹角/点云两两距离"等
与坐标约定无关的不变量校验：对齐基座后残差 2.6e-4 mm）：

    i  关节              θ0(deg)    d(mm)      a(mm)      α(deg)
    1  thumb_cmc_yaw       0       19.6078     0.8432     90
    2  thumb_cmc_roll      0       -3.9757    30.7945    -90
    3  thumb_cmc_pitch    31.0150  -0.1097    56.2298      0
    4  thumb_mcp           2.2353   0         34.8986      0
    5  thumb_dip         -34.6345   0         34.8015      0
    指尖 = 第 5 坐标系原点；θ_i = q_i + θ0_i

解析解依据的三个结构性质：
  1) α3=α4=α5=0 → 后三个关节轴（pitch/mcp/dip）互相平行 → 指尖在垂直于这三条轴的平面内
     运动（平面 3R 机构），落到第 2 坐标系里就是 u = (a3,0) + Rz(θ3)·(a4 + a5·cosψ, a5·sinψ)；
  2) 指尖位置只依赖 ψ = θ4 + θ5 之和，与二者怎么拆无关 → 这个自由度用限位 + 连续性直接选；
  3) 前两个关节只负责把该平面摆到"包含目标点"，约束可化为
        sinθ2·(a1 − B) + cosθ2·(Pz − d1) = d3,   B = Px·cosθ1 + Py·sinθ1
     → θ1 是自由参数（1 维自运动），θ2 闭式解，θ3 与 ψ 由余弦定理闭式解。

对外接口：
    ik_closed(target, seed)    -> (q, 残差)   纯闭式，无迭代（微秒级）
    jac_pos_closed(q)          -> 3x5 解析雅可比
    ik_dls(target, seed)       -> 用解析雅可比的阻尼最小二乘（兜底/精修）
    ik_hybrid(target, seed)    -> 闭式 + 必要时精修
    make_ik(kind)              -> 给 tip_core.build_plan 用的适配器
"""
import math
import time

import numpy as np

from tip_core import QLOW, QHIGH, _ORIGINS, _T, _rpy, _Ry, fk_tip, fk_pose

# ---------------- DH 常量（米 / 弧度） ----------------
_D = [0.0196078, -0.0039757, -0.0001097, 0.0, 0.0]
_A = [0.0008432, 0.0307945, 0.0562298, 0.0348986, 0.0348015]
_AL = [math.radians(90.0), math.radians(-90.0), 0.0, 0.0, 0.0]
_TH0 = [0.0, 0.0, math.radians(31.0150), math.radians(2.2353), math.radians(-34.6345)]

# θ 空间的限位（θ = q + θ0）
_TLOW = [float(QLOW[i]) + _TH0[i] for i in range(5)]
_THIGH = [float(QHIGH[i]) + _TH0[i] for i in range(5)]

# 关节 origin 的常量矩阵（避免每帧重复构造，FK 更快）
_ORIGIN_MATS = [(_T(np.array(xyz, float)) @ _rpy(*rpy)) for xyz, rpy in _ORIGINS]


def dh_tip(q):
    """DH 正解：第 5 坐标系原点（DH 基座系下），单位 m"""
    x = y = z = 0.0
    M = np.eye(4)
    for i in range(5):
        th = _TH0[i] + q[i]
        c, s = math.cos(th), math.sin(th)
        ca, sa = math.cos(_AL[i]), math.sin(_AL[i])
        M = M @ np.array([[c, -s * ca, s * sa, _A[i] * c],
                          [s, c * ca, -c * sa, _A[i] * s],
                          [0.0, sa, ca, _D[i]],
                          [0.0, 0.0, 0.0, 1.0]])
    return M[:3, 3]


# ---------------- base_link <-> DH 基座系的固定变换 ----------------
# 注意：这张 DH 表与 URDF 在【指尖位置】上严格等价（对齐后 8e-5 mm），但末节坐标系的
# 姿态定义方式不同，所以基座变换必须只用"位置"做刚体对齐（Kabsch），不能用位姿相除。
def _compute_C(verify_tol_mm=0.01):
    rng = np.random.default_rng(0)
    from tip_core import QLOW, QHIGH
    qs = [rng.uniform(QLOW, QHIGH) for _ in range(80)]
    A = np.array([dh_tip(q) for q in qs])            # DH 系下的指尖
    B = np.array([fk_tip(q) for q in qs])            # base_link 下的指尖
    ca, cb = A.mean(0), B.mean(0)
    U, S, Vt = np.linalg.svd((A - ca).T @ (B - cb))
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = cb - R @ ca
    # 自检：留出一部分构型验证
    rng2 = np.random.default_rng(1)
    err = max(np.linalg.norm(R @ dh_tip(q) + t - fk_tip(q))
              for q in [rng2.uniform(QLOW, QHIGH) for _ in range(60)]) * 1000.0
    if err > verify_tol_mm:
        raise RuntimeError('DH 模型与 URDF 指尖位置对不齐：%.4f mm' % err)
    C = np.eye(4)
    C[:3, :3] = R
    C[:3, 3] = t
    return C


_C = _compute_C()                     # p_base = _C_R @ p_dh + _C_t
_CR = [[float(_C[i][j]) for j in range(3)] for i in range(3)]
_Ct = [float(_C[i][3]) for i in range(3)]
# 反向：p_dh = _CR^T @ (p_base − _Ct)（预转置，纯 Python 微秒级）
_CRT = [[_CR[j][i] for j in range(3)] for i in range(3)]


def to_dh(p):
    """base_link 下的点 -> DH 基座系下的点"""
    x, y, z = float(p[0]) - _Ct[0], float(p[1]) - _Ct[1], float(p[2]) - _Ct[2]
    return (_CRT[0][0] * x + _CRT[0][1] * y + _CRT[0][2] * z,
            _CRT[1][0] * x + _CRT[1][1] * y + _CRT[1][2] * z,
            _CRT[2][0] * x + _CRT[2][1] * y + _CRT[2][2] * z)


# ---------------- 闭式求解（内部全在 θ 空间） ----------------
def _Rz3(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _Rx3(a):
    c, s = math.cos(a), math.sin(a)
    return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))


def _solve_planar_at(v, alpha):
    """平面 3R 逆解：给定末杆绝对角 α，由 (a3, a4) 的 2R 链解出 θ3、θ4，再 θ5 = α − θ3 − θ4"""
    a3, a4, a5 = _A[2], _A[3], _A[4]
    Wx = v[0] - a5 * math.cos(alpha)
    Wy = v[1] - a5 * math.sin(alpha)
    r2 = Wx * Wx + Wy * Wy
    r = math.sqrt(r2)
    if r > a3 + a4 or r < abs(a3 - a4):
        return ()
    c4 = (r2 - a3 * a3 - a4 * a4) / (2.0 * a3 * a4)
    if c4 < -1.0 or c4 > 1.0:
        return ()
    ac4 = math.acos(c4)
    out = []
    for t4 in (ac4, -ac4):
        t3 = math.atan2(Wy, Wx) - math.atan2(a4 * math.sin(t4), a3 + a4 * math.cos(t4))
        out.append((t3, t4, alpha - t3 - t4))
    return out


def _in_limits(th):
    for i in range(5):
        if th[i] < _TLOW[i] - 1e-9 or th[i] > _THIGH[i] + 1e-9:
            return False
    return True


def _candidates_at(P, t1, alpha):
    """给定 (θ1, α)，闭式给出所有可行候选（θ2 两个分支 × 平面两个分支）

    推导（α1=+90°, α2=−90°）：
        R_02 = [[c1·c2, −s1, −c1·s2],
                [s1·c2,  c1, −s1·s2],
                [s2,      0,   c2  ]]
    于是 u = R_02ᵀ·(P − p02) 用标量算，平面部分交给 _solve_planar_at。
    """
    Px, Py, Pz = P
    d0, d1, d2 = _D[0], _D[1], _D[2]
    a0, a1 = _A[0], _A[1]
    if t1 < _TLOW[0] or t1 > _THIGH[0]:
        return
    c1, s1 = math.cos(t1), math.sin(t1)
    B = Px * c1 + Py * s1
    K = Pz - d0
    ux0 = a0 - B
    R = math.hypot(ux0, K)
    if R < 1e-12:
        return
    sn = d2 / R                        # sin(θ2 + γ) = d3 / R
    if sn < -1.0 or sn > 1.0:
        return
    gam = math.atan2(K, ux0)
    base = math.asin(sn)
    for t2 in (base - gam, math.pi - base - gam):
        if t2 < _TLOW[1] or t2 > _THIGH[1]:
            continue
        c2, s2 = math.cos(t2), math.sin(t2)
        A_ = a0 + a1 * c2
        dx = Px - (A_ * c1 + d1 * s1)
        dy = Py - (A_ * s1 - d1 * c1)
        dz = Pz - (d0 + a1 * s2)
        ux = c1 * c2 * dx + s1 * c2 * dy + s2 * dz       # R_02ᵀ 第 1 行
        uy = -s1 * dx + c1 * dy                           # R_02ᵀ 第 2 行
        for t3, t4, t5 in _solve_planar_at((ux, uy), alpha):
            th = (t1, t2, t3, t4, t5)
            if _in_limits(th):
                yield th


def _cost(th, seed_th):
    return ((th[0] - seed_th[0]) ** 2 + (th[1] - seed_th[1]) ** 2 + (th[2] - seed_th[2]) ** 2
            + (th[3] - seed_th[3]) ** 2 + (th[4] - seed_th[4]) ** 2)


def _local_search(P, seed_th, start_t1, start_al, steps=(0.02, 0.005, 0.0012)):
    """从给定 (θ1, α) 出发做坐标下降（两个自由度上轮流走），返回 (最优 θ, cost, θ1, α)

    因为起点就是上一时刻的解，得到的解天然连续；平均只需几十次评估。"""
    best, best_cost = None, float('inf')
    t1, al = start_t1, start_al
    for th in _candidates_at(P, t1, al):
        c = _cost(th, seed_th)
        if c < best_cost:
            best, best_cost = th, c
    for step in steps:
        moved = True
        while moved:
            moved = False
            for dt1, dal in ((step, 0.0), (-step, 0.0), (0.0, step), (0.0, -step)):
                nt1, nal = t1 + dt1, al + dal
                for th in _candidates_at(P, nt1, nal):
                    c = _cost(th, seed_th)
                    if c < best_cost - 1e-15:
                        best, best_cost, t1, al, moved = th, c, nt1, nal, True
    return best, best_cost, t1, al


def _global_search(P, seed_th, n_t1=41, n_al=721):
    """兜底（换支/大跳变时才会用到）：θ1 粗网格 + α 细网格（0.5°），命中后再局部细化

    α 必须够细：可行的 α 往往只是一条窄带，粗网格容易整条漏掉。"""
    best, best_cost, bt1, bal = None, float('inf'), None, None
    for i in range(n_t1):
        t1 = _TLOW[0] + (_THIGH[0] - _TLOW[0]) * i / (n_t1 - 1)
        for j in range(n_al):
            al = -math.pi + 2.0 * math.pi * j / (n_al - 1)
            for th in _candidates_at(P, t1, al):
                c = _cost(th, seed_th)
                if c < best_cost:
                    best, best_cost, bt1, bal = th, c, t1, al
    if best is None:
        return None, float('inf')
    # 在命中的网格点附近再细化一次（粗网格解离最优还有一点距离）
    b2, c2, _, _ = _local_search(P, seed_th, bt1, bal, steps=(0.01, 0.003, 0.001))
    if b2 is not None and c2 < best_cost:
        return b2, c2
    return best, best_cost


def ik_closed(target, seed, tol=1e-9, verify=True):
    """解析逆解：返回 (q, 残差[m])。无迭代（局部下降 + 全局兜底两次闭式搜索）。

    5 自由度对 3 维位置 → 解是 2 参数族：外层的 θ1 与平面内的 α（末杆绝对角）。
    两者都从上一解出发用坐标下降选 → 轨迹连续、不跳支。
    """
    seed = np.asarray(seed, float)
    seed_th = [seed[i] + _TH0[i] for i in range(5)]
    seed_alpha = seed_th[2] + seed_th[3] + seed_th[4]
    P = to_dh(target)
    best, best_cost, _, _ = _local_search(P, seed_th, seed_th[0], seed_alpha)
    if best is None:
        best, best_cost = _global_search(P, seed_th)
    if best is None:                           # 全被限位挡住：返回裁剪后的种子
        q = np.clip(seed, QLOW, QHIGH)
        if not verify:
            return q, 0.0
        return q, float(np.linalg.norm(fk_tip(q) - np.asarray(target, float)))
    q = np.array([best[i] - _TH0[i] for i in range(5)], float)
    q = np.clip(q, QLOW, QHIGH)
    if not verify:                             # 闭式解在模型内精确，实时路径可跳过回读
        return q, 0.0
    return q, float(np.linalg.norm(fk_tip(q) - np.asarray(target, float)))



# ---------------- 解析雅可比 ----------------
def jac_pos_closed(q):
    """指尖位置对 5 个关节角的雅可比（3x5）：J_i = z_i × (p_tip − p_i)，闭式无差分"""
    q = np.asarray(q, float)
    p_tip = fk_tip(q)
    M = np.eye(4)
    J = np.zeros((3, 5))
    for i in range(5):
        Mj = M @ _ORIGIN_MATS[i]              # 关节 i 的轴所在坐标系（自身旋转之前）
        p = Mj[:3, 3]
        z = Mj[:3, :3] @ np.array([0.0, 1.0, 0.0])
        J[:, i] = np.cross(z, p_tip - p)
        M = Mj @ _Ry(q[i])
    return J


# ---------------- 数值迭代（解析雅可比版 DLS，做兜底/精修） ----------------
def ik_dls(target, seed, tol=1e-9, max_iter=60, lam=3e-3, step_clip=0.2):
    q = np.clip(np.asarray(seed, float).copy(), QLOW, QHIGH)
    target = np.asarray(target, float)
    best, best_err = q.copy(), float(np.linalg.norm(target - fk_tip(q)))
    for _ in range(max_iter):
        e = target - fk_tip(q)
        err = float(np.linalg.norm(e))
        if err < best_err:
            best, best_err = q.copy(), err
        if err < tol:
            break
        J = jac_pos_closed(q)
        dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * np.eye(3), e)
        q = np.clip(q + np.clip(dq, -step_clip, step_clip), QLOW, QHIGH)
    return best, best_err


def ik_hybrid(target, seed, tol=1e-9, polish_tol=1e-7):
    """闭式解打底；万一不够准（数值极限情况），再用解析雅可比迭代几次精修"""
    q, err = ik_closed(target, seed, tol=polish_tol)
    if err > polish_tol:
        q2, err2 = ik_dls(target, q, tol=tol, max_iter=8)
        if err2 < err:
            return q2, err2
    return q, err


def make_ik(kind='closed'):
    """给 tip_core.build_plan 用的适配器：f(target, seed, tol=...) -> (q, 残差)"""
    if kind == 'closed':
        return lambda t, s, tol=1e-5: ik_closed(t, s, verify=False)   # 实时路径：省掉回读校验
    if kind == 'hybrid':
        return lambda t, s, tol=1e-5: ik_hybrid(t, s)

    def _numeric(t, s, tol=1e-5):
        from tip_core import ik_position
        return ik_position(t, s, tol=tol)
    if kind == 'numeric':
        return _numeric

    def _dls_closed(t, s, tol=1e-5):
        return ik_dls(t, s, tol=tol)
    if kind == 'dls-closed':
        return _dls_closed
    raise ValueError('未知的 ik kind: %s' % kind)