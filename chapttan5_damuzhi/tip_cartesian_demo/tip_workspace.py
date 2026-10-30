#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tip_workspace.py —— 大拇指指尖可达范围自检（纯数学，不需要 ROS）

用途：跑笛卡尔直线之前，先确认"起点/终点"都落在指尖够得着的范围内。
      手指很小，可达范围只有几厘米，随便给坐标必然 IK 失败。

用法： python3 tip_workspace.py
"""
import numpy as np

from tip_core import (QLOW, QHIGH, Q_PRE, TIP_INITIAL, _ORIGINS, TIP, fk_tip)


def fk_tip_vec(q):
    """批量正解：q.shape=(N,5) -> 指尖位置 (N,3)

    p = o0 + M0*(o1 + M1*(o2 + M2*(o3 + M3*(o4 + M4*tip))))，其中 Mi = R(rpy_i)·Ry(q_i)
    """
    q = np.asarray(q, dtype=float)
    N = q.shape[0]
    c, s = np.cos(q), np.sin(q)
    # 每个关节的常量旋转 R(rpy_i)
    Rc = []
    for xyz, rpy in _ORIGINS:
        r, p, y = rpy
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        Rc.append(Rz @ Ry @ Rx)
    # Mi = Rc_i @ Ry(q_i)，批量算
    M = []
    for i in range(5):
        Ry = np.zeros((N, 3, 3))
        Ry[:, 0, 0], Ry[:, 0, 2] = c[:, i], s[:, i]
        Ry[:, 1, 1] = 1.0
        Ry[:, 2, 0], Ry[:, 2, 2] = -s[:, i], c[:, i]
        M.append(np.einsum('ij,njk->nik', Rc[i], Ry))

    def app(A, v):                      # v 是常量向量 (3,)
        return np.einsum('nij,j->ni', A, v)

    def appn(A, v):                     # v 是每个样本各一个 (N,3)
        return np.einsum('nij,nj->ni', A, v)

    o = [np.array(xyz, dtype=float) for xyz, _ in _ORIGINS]
    p = o[4] + app(M[4], TIP)
    for i in (3, 2, 1, 0):
        p = o[i] + appn(M[i], p)
    return p


def main():
    rng = np.random.default_rng(0)
    n = 300000
    q = rng.uniform(QLOW, QHIGH, size=(n, 5))
    P = fk_tip_vec(q) * 1000.0  # mm

    p0 = fk_tip(np.zeros(5)) * 1000.0
    print('=== 大拇指指尖可达范围 (base_link, mm) ===')
    print('  q=0 时指尖      : [%.3f, %.3f, %.3f]' % tuple(p0))
    print('  包裹盒尺寸      : x=%.1f  y=%.1f  z=%.1f' % tuple(P.max(0) - P.min(0)))
    print('  x 范围          : %8.2f ~ %8.2f' % (P[:, 0].min(), P[:, 0].max()))
    print('  y 范围          : %8.2f ~ %8.2f' % (P[:, 1].min(), P[:, 1].max()))
    print('  z 范围          : %8.2f ~ %8.2f' % (P[:, 2].min(), P[:, 2].max()))

    s = fk_tip(Q_PRE) * 1000.0
    print()
    print('=== 从预弯曲指尖 [%.3f, %.3f, %.3f] 出发，各方向能走多远 ===' % tuple(s))
    print('  (用 30 万随机采样点估，阈值 1mm)')
    for name, ax, sign in [('+x', 0, 1), ('-x', 0, -1), ('+y', 1, 1),
                           ('-y', 1, -1), ('+z', 2, 1), ('-z', 2, -1)]:
        d = (P[:, ax] - s[ax]) * sign
        cand = P[d > 0]
        reach = 0.0
        if cand.shape[0] > 0:
            # 只看"基本贴在同一条直线上"的点：其余两轴偏差 < 3mm
            other = [i for i in range(3) if i != ax]
            near = cand[(np.abs(cand[:, other[0]] - s[other[0]]) < 3.0) &
                        (np.abs(cand[:, other[1]] - s[other[1]]) < 3.0)]
            if near.shape[0] > 0:
                reach = float(((near[:, ax] - s[ax]) * sign).max())
        print('  %s 方向: %.1f mm' % (name, reach))

    print()
    print('提示：给 --end 时，让终点落在上面这些方向/距离之内，IK 才解得出来；')
    print('      更严格的"能不能走直线"请用 tip_scan_directions.py。')


if __name__ == '__main__':
    main()
