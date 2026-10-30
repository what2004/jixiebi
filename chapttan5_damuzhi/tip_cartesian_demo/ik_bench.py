#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ik_bench.py —— 解析逆解 vs 数值逆解：精度 / 速度 / 连续性 对比

跑法：  python3 ik_bench.py
"""
import time
import numpy as np

import tip_core as C
import tip_ik as IK

rng = np.random.default_rng(0)
N = 200


def stat(x):
    x = np.asarray(x, float)
    return x.max(), x.mean()


print('=' * 78)
print('① 精度：随机取 200 个限位内构型 -> 目标点 -> 反解，比较位置残差')
print('=' * 78)
err_c, err_h, err_n, ok_c = [], [], [], 0
for _ in range(N):
    q_true = rng.uniform(C.QLOW, C.QHIGH)
    P = C.fk_tip(q_true)
    seed = np.clip(q_true + rng.normal(0, 0.1, 5), C.QLOW, C.QHIGH)
    qc, ec = IK.ik_closed(P, seed)
    qh, eh = IK.ik_hybrid(P, seed)
    qn, en = C.ik_position(P, seed)
    err_c.append(ec); err_h.append(eh); err_n.append(en)
    if ec < 1e-6:
        ok_c += 1
mc, ac = stat(np.array(err_c) * 1000)
mh, ah = stat(np.array(err_h) * 1000)
mn, an = stat(np.array(err_n) * 1000)
print('  解析(闭式)  : 残差 max = %.6f mm, mean = %.6f mm   (成功 %d/%d, <1e-3 mm 记成功)' % (mc, ac, ok_c, N))
print('  解析+精修   : 残差 max = %.6f mm, mean = %.6f mm' % (mh, ah))
print('  数值(DLS)   : 残差 max = %.6f mm, mean = %.6f mm' % (mn, an))

print()
print('=' * 78)
print('② 速度：单点 IK 平均耗时')
print('=' * 78)
P = C.fk_tip(C.Q_PRE)
seed = C.Q_PRE.copy()


def timeit(fn, n=200):
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) / n * 1e6      # us


tc = timeit(lambda: IK.ik_closed(P, seed), 2000)              # 含回读校验
tcf = timeit(lambda: IK.ik_closed(P, seed, verify=False), 5000)  # 实时路径（不做回读）
th = timeit(lambda: IK.ik_hybrid(P, seed), 2000)
td = timeit(lambda: IK.ik_dls(P, seed), 200)
tn = timeit(lambda: C.ik_position(P, seed), 20)
tj_num = timeit(lambda: C.jac_pos(seed), 2000)
tj_cls = timeit(lambda: IK.jac_pos_closed(seed), 2000)
print('  解析(闭式, 含回读校验)        : %8.1f us/点' % tc)
print('  解析(闭式, 纯实时路径)        : %8.1f us/点' % tcf)
print('  解析 + 精修(hybrid)          : %8.1f us/点' % th)
print('  DLS + 解析雅可比              : %8.1f us/点' % td)
print('  数值 DLS + 数值雅可比(原版)   : %8.1f us/点' % tn)
print('  雅可比单次: 数值差分 %.1f us  vs 解析 %.1f us  (快 %.1f 倍)' % (tj_num, tj_cls, tj_num / max(tj_cls, 1e-9)))
print('  -> 相对原版加速: 解析(实时) %.0f 倍' % (tn / tcf))

print()
print('=' * 78)
print('③ 整条轨迹：25 mm 直线（60 段）用不同 IK 规划')
print('=' * 78)
p0 = C.fk_tip(C.Q_PRE)
p1 = p0 + np.array([0.0, 0.0, -0.025])
for name, kind in (('解析(闭式)', 'closed'), ('解析+精修', 'hybrid'), ('数值 DLS(原版)', 'numeric')):
    t0 = time.perf_counter()
    res = C.build_plan(np.zeros(5), p0, p1, ik_func=IK.make_ik(kind))
    dt = (time.perf_counter() - t0) * 1000
    print('  %-14s 规划耗时 %8.1f ms | fraction = %.4f | IK 残差 max = %.5f mm | 直线度 = %.5f mm'
          % (name, dt, res.fraction, res.resid.max() * 1000, res.max_line_dev * 1000))

print()
print('=' * 78)
print('④ 连续性：直线段相邻两点的最大关节跳变（越小越平滑）')
print('=' * 78)
for name, kind in (('解析(闭式)', 'closed'), ('数值 DLS(原版)', 'numeric')):
    res = C.build_plan(np.zeros(5), p0, p1, ik_func=IK.make_ik(kind))
    q_line = res.q[res.n_line0:]
    jump = np.linalg.norm(np.diff(q_line, axis=0), axis=1)
    print('  %-14s 相邻点最大关节跳变 = %.5f rad, 平均 %.5f rad' % (name, jump.max(), jump.mean()))
