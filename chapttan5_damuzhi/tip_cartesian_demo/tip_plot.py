#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tip_plot.py —— 把一次规划的指尖轨迹画成图（matplotlib，本机没装请看 README）

用法：
  python3 tip_plot.py --offline                       # 现场算一遍再画（默认参数）
  python3 tip_plot.py --offline --end 0.049209 0.070845 0.163871
  python3 tip_plot.py --csv tip_traj.csv              # 画 tip_run.py --csv 导出的数据
  python3 tip_plot.py --offline --save tip_plot.png   # 存成 PNG（不弹窗，适合写报告）

画 6 张子图：
  ① 三维：理想直线(绿) + 规划指尖轨迹(蓝，含预弯曲段)
  ② x / y / z 随时间
  ③ 指尖到理想直线的垂距随时间（直线度）
  ④ IK 残差随时间（逆解精度）
  ⑤ 五个关节角随时间
  ⑥ 直线段局部放大：y 随时间（看是否匀速、有没有拐点）
"""
import argparse
import sys

import numpy as np


def _setup_font():
    """让 matplotlib 能显示中文：从系统字体里找一个 CJK 字体注册进来"""
    import matplotlib
    from matplotlib import font_manager
    known = ['/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
             '/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc',
             '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',
             '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf']
    keys = ('notosanscjk', 'notoserifcjk', 'sourcehans', 'wqy', 'droidsansfallback', 'simhei', 'msyh')
    scan = [p for p in font_manager.findSystemFonts()
            if any(k in p.lower().replace('-', '').replace('_', '') for k in keys)]
    picked = None
    for path in known + scan:
        try:
            font_manager.fontManager.addfont(path)
            picked = font_manager.FontProperties(fname=path).get_name()
            break
        except Exception:
            continue
    if picked:
        # 中文字体放第一位，后面挂 DejaVu Sans 兜底（缺字时自动回退，不会出方框）
        matplotlib.rcParams['font.sans-serif'] = [picked, 'DejaVu Sans']
    else:
        print('!! 没找到中文字体，图里中文会显示成方框（可 apt install fonts-noto-cjk）')
    matplotlib.rcParams['axes.unicode_minus'] = False


def load_csv(path):
    """读 tip_run.py --csv 导出的文件，返回 (P0, P1, 字典形式的数据列)"""
    meta, rows = {}, []
    with open(path, 'r') as f:
        for line in f:
            if line.startswith('#'):
                if 'P0_m=' in line:
                    meta['P0'] = np.array([float(x) for x in line.split('P0_m=')[1].split(',')])
                elif 'P1_m=' in line:
                    meta['P1'] = np.array([float(x) for x in line.split('P1_m=')[1].split(',')])
                continue
            if line.startswith('t,'):
                header = line.strip().split(',')
                continue
            if line.strip():
                rows.append([float(x) for x in line.strip().split(',')])
    a = np.array(rows)
    d = {name: a[:, i] for i, name in enumerate(header)}
    return meta.get('P0'), meta.get('P1'), d


def load_offline(args):
    from tip_core import Q_PRE, TIP_INITIAL, build_plan, fk_tip, line_deviation
    q_pre = np.array(args.preflex, dtype=float)
    p_start = TIP_INITIAL.copy() if args.from_initial else fk_tip(q_pre)
    p_end = (p_start + np.array([0.0, 0.0, -0.025])) if args.end is None else np.array(args.end, float)
    res = build_plan(np.zeros(5), p_start, p_end, q_pre=q_pre, steps=args.steps)
    d = {'t': res.t, 'x_m': res.pts[:, 0], 'y_m': res.pts[:, 1], 'z_m': res.pts[:, 2],
         'resid_mm': res.resid * 1000.0,
         'dev_mm': np.array([line_deviation(p, p_start, p_end) for p in res.pts]) * 1000.0,
         'segment': np.array(res.tags),
         'q0': res.q[:, 0], 'q1': res.q[:, 1], 'q2': res.q[:, 2], 'q3': res.q[:, 3], 'q4': res.q[:, 4]}
    return p_start, p_end, d


def main():
    ap = argparse.ArgumentParser(description='把指尖笛卡尔轨迹画成图')
    ap.add_argument('--offline', action='store_true', help='现场算一遍再画（不连 ROS）')
    ap.add_argument('--csv', type=str, default=None, help='画 tip_run.py --csv 导出的文件')
    ap.add_argument('--end', type=float, nargs=3, default=None, metavar=('X', 'Y', 'Z'))
    ap.add_argument('--preflex', type=float, nargs=5, default=[0.30, 0.30, 0.40, 0.40, 0.30])
    ap.add_argument('--from-initial', action='store_true')
    ap.add_argument('--steps', type=int, default=60)
    ap.add_argument('--save', type=str, default=None, help='存成 PNG 的路径（不弹窗）')
    args = ap.parse_args()

    try:
        import matplotlib
        if args.save:
            matplotlib.use('Agg')
        _setup_font()
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    except ModuleNotFoundError:
        print('!! 没装 matplotlib。任选其一：')
        print('   1) python3 -m pip install --user matplotlib     # 装完再跑本脚本')
        print('   2) 不装也行：用 tip_run.py --csv 导出数据，在 PlotJuggler 里 Load data file 直接画')
        return 1

    if args.csv:
        P0, P1, d = load_csv(args.csv)
        if P0 is None or P1 is None:
            print('!! CSV 里没找到 P0/P1 注释行，无法画理想直线')
            return 1
    else:
        P0, P1, d = load_offline(args)

    t = d['t']
    xyz = np.vstack([d['x_m'], d['y_m'], d['z_m']]).T * 1000.0   # mm
    seg = np.array([str(s) for s in d['segment']])
    line_st = seg == 'line'
    t0 = t[line_st].min() if line_st.any() else t.min()

    fig = plt.figure(figsize=(15, 9))
    fig.suptitle('大拇指指尖笛卡尔直线轨迹  |  P0=[%.3f, %.3f, %.3f] mm  →  P1=[%.3f, %.3f, %.3f] mm'
                 % (*P0 * 1000, *P1 * 1000), fontsize=12)

    # ① 三维
    ax = fig.add_subplot(2, 3, 1, projection='3d')
    p0, p1 = np.asarray(P0) * 1000, np.asarray(P1) * 1000
    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], [p0[2], p1[2]], color='limegreen', lw=2, label='理想直线')
    ax.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], color='royalblue', lw=1.5, label='规划指尖轨迹')
    ax.scatter(*xyz[line_st].T, s=4, color='royalblue')
    ax.scatter(*p1, color='green', s=40, marker='o')
    ax.set_xlabel('x [mm]'); ax.set_ylabel('y [mm]'); ax.set_zlabel('z [mm]')
    ax.legend(fontsize=8); ax.set_title('① 三维对比')

    # ② x/y/z
    ax = fig.add_subplot(2, 3, 2)
    for i, c in enumerate('xyz'):
        ax.plot(t, xyz[:, i], label='指尖 %s' % c)
    ax.axvline(t0, color='gray', ls='--', lw=1)
    ax.text(t0, ax.get_ylim()[0], ' 直线段开始', fontsize=8, va='bottom', color='gray')
    ax.set_xlabel('t [s]'); ax.set_ylabel('位置 [mm]'); ax.legend(fontsize=8)
    ax.set_title('② 指尖 x / y / z')

    # ③ 直线度
    ax = fig.add_subplot(2, 3, 3)
    ax.plot(t, d['dev_mm'], color='crimson')
    ax.axvline(t0, color='gray', ls='--', lw=1)
    ax.set_xlabel('t [s]'); ax.set_ylabel('到理想直线垂距 [mm]')
    ax.set_title('③ 直线度（max = %.4f mm）' % d['dev_mm'].max())

    # ④ IK 残差
    ax = fig.add_subplot(2, 3, 4)
    ax.plot(t, d['resid_mm'], color='darkorange')
    ax.axvline(t0, color='gray', ls='--', lw=1)
    ax.set_xlabel('t [s]'); ax.set_ylabel('IK 残差 [mm]')
    ax.set_title('④ 逆解残差（max = %.4f mm）' % d['resid_mm'].max())

    # ⑤ 关节角
    ax = fig.add_subplot(2, 3, 5)
    for i, name in enumerate(['yaw', 'roll', 'pitch', 'mcp', 'dip']):
        ax.plot(t, d['q%d' % i], label=name)
    ax.axvline(t0, color='gray', ls='--', lw=1)
    ax.set_xlabel('t [s]'); ax.set_ylabel('关节角 [rad]'); ax.legend(fontsize=8, ncol=2)
    ax.set_title('⑤ 五个关节角')

    # ⑥ 直线段放大
    ax = fig.add_subplot(2, 3, 6)
    ax.plot(t[line_st], xyz[line_st, 1], 'o-', ms=3, color='royalblue', label='规划')
    ax.plot([p0[1], p1[1]], [t[line_st].min(), t[line_st].max()], alpha=0)  # 占位
    ax.set_xlabel('t [s]'); ax.set_ylabel('y [mm]')
    ax.set_title('⑥ 直线段局部放大（y 随时间）')
    ax.grid(alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    if args.save:
        fig.savefig(args.save, dpi=130)
        print('已保存: %s' % args.save)
    else:
        plt.show()
    return 0


if __name__ == '__main__':
    sys.exit(main())
