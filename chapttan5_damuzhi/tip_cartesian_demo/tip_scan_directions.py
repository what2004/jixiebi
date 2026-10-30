#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tip_scan_directions.py —— 用真正的 IK 扫描：从起点出发哪个方向能走多长的【笛卡尔直线】

为什么需要它：
  * "可达"不等于"能走直线"——可达集是一张弯曲的曲面带，直线很快会离开它；
  * 再叠加关节限位（拇指 5 个关节下限全为 0），有些方向根本走不动。
  本脚本对每个候选方向，沿直线均匀取点跑 IK，二分出"还能保持直线"的最大长度。

用法：
  python3 tip_scan_directions.py                  # 从预弯曲构型的指尖位置扫（推荐）
  python3 tip_scan_directions.py --from-initial   # 从指尖初始位置(q=0)扫
  python3 tip_scan_directions.py --sphere         # 更密的方向球（慢）
"""
import argparse
import time
import numpy as np

from tip_core import (TIP_INITIAL, Q_PRE, fk_tip, ik_position,
                      plan_cartesian_segment)

MAX_ITER = 150          # 扫描时把 IK 迭代上限调小，够判断"解得出来/解不出来"即可


def axes_dirs():
    out = []
    for d in [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]:
        out.append(np.array(d, dtype=float))
    s = 1.0 / np.sqrt(2.0)
    for a in (1, -1):
        for b in (1, -1):
            out.append(np.array([a * s, 0.0, b * s]))
            out.append(np.array([0.0, a * s, b * s]))
            out.append(np.array([a * s, b * s, 0.0]))
    return out


def sphere_dirs(n_theta=7, n_phi=12):
    out = []
    for th in np.linspace(0, np.pi, n_theta):
        for ph in np.linspace(0, 2 * np.pi, n_phi, endpoint=False):
            out.append(np.array([np.sin(th) * np.cos(ph),
                                 np.sin(th) * np.sin(ph),
                                 np.cos(th)]))
    return out


def max_straight_length(p_start, q_seed, direction, max_len=0.060,
                        step_size=0.0005, end_tol=5e-4, bisect=9):
    """
    二分找最长可行直线长度。用的是主程序同一个 plan_cartesian_segment，所以结论和主程序一致：
    要求 完成度 fraction >= 0.999 且 末端实际到达与目标差 < end_tol。
    """
    def ok(L):
        n = min(90, max(12, int(L / step_size)))
        qB, rB, pB, frac = plan_cartesian_segment(
            p_start, p_start + direction * L, q_seed, n, max_iter=MAX_ITER)
        if frac < 0.999 or len(pB) == 0:
            return False
        return float(np.linalg.norm(pB[-1] - (p_start + direction * L))) < end_tol

    if not ok(1e-3):
        return 0.0
    lo, hi = 1e-3, max_len
    if ok(hi):
        return hi
    for _ in range(bisect):
        mid = 0.5 * (lo + hi)
        if ok(mid):
            lo = mid
        else:
            hi = mid
    return lo


def main():
    p_pre = fk_tip(Q_PRE)
    ap = argparse.ArgumentParser()
    ap.add_argument('--start', type=float, nargs=3, default=None, help='起点(m)')
    ap.add_argument('--from-initial', action='store_true', help='用指尖初始位置(q=0)当起点')
    ap.add_argument('--sphere', action='store_true', help='扫描更密的方向球(慢)')
    ap.add_argument('--max-len', type=float, default=0.040, help='搜索的最大长度(m)')
    args = ap.parse_args()

    if args.start is not None:
        p_start = np.array(args.start, dtype=float)
        seed = Q_PRE.copy() if np.linalg.norm(p_start - p_pre) < 1e-9 else np.zeros(5)
        label = '自定义起点'
    elif args.from_initial:
        p_start, seed, label = TIP_INITIAL.copy(), np.zeros(5), '指尖初始位置 q=0（限位角点）'
    else:
        p_start, seed, label = p_pre, Q_PRE.copy(), '预弯曲构型'

    q_start, e0 = ik_position(p_start, seed)
    print('起点(%s) = [%.5f, %.5f, %.5f] m   IK 残差 = %.2e m' % (label, *p_start, e0))

    dirs = sphere_dirs() if args.sphere else axes_dirs()
    t0 = time.time()
    res = []
    for d in dirs:
        L = max_straight_length(p_start, q_start, d, args.max_len)
        res.append((L, d))
        print('  扫描 %-24s -> %7.2f mm' % (np.round(d, 3).tolist(), L * 1000))
    res.sort(key=lambda x: -x[0])

    print()
    print('=== 可行笛卡尔直线（IK 逐点验证, %.1fs）===' % (time.time() - t0))
    print('  %10s   %s' % ('最长长度', '方向单位向量 (x, y, z)'))
    shown = 0
    for L, d in res:
        if L < 3e-5 or shown >= 12:
            break
        print('  %7.2f mm   [%+.3f, %+.3f, %+.3f]' % (L * 1000, d[0], d[1], d[2]))
        shown += 1
    if shown == 0:
        print('  (任何方向都走不动 —— 起点卡在关节限位角点上)')

    best_L, best_d = res[0]
    if best_L > 1e-4:
        use = best_L * 0.8
        print()
        print('建议终点（留 20%% 余量）：--end %.6f %.6f %.6f   (沿最优方向 %.1f mm)'
              % (*(p_start + best_d * use), use * 1000))


if __name__ == '__main__':
    main()
