# tip_cartesian_demo —— 大拇指指尖笛卡尔轨迹（独立小工具集）

自己的**笛卡尔插值 + 逆解**，不经过 MoveIt，所以不会遇到"5 自由度手指解不出 6D 位姿"的
followed 0.000000% 问题。轨迹最终仍以标准关节轨迹发给
`/damuzhi_controller/follow_joint_trajectory`，仿真里的 FakeSystem 会直接执行。

这是食指版（`~/chapttan5/tip_cartesian_demo`）的大拇指移植版，用法、输出格式完全一致，
只是关节从 4 个变成 5 个：**thumb_cmc_yaw / thumb_cmc_roll / thumb_cmc_pitch / thumb_mcp / thumb_dip**
（顺序与 `ros2_controllers.yaml` 里 `damuzhi_controller` 的 joints 一致），指尖是 `thumb_tip`。

## 文件说明

| 文件 | 行数 | 作用 |
|---|---|---|
| **tip_core.py** | 287 | **数学核心（纯 numpy，不 import ROS）**：FK、位置优先 IK、关节空间插值、笛卡尔直线插值、时间参数化、直线度评估、串起全流程的 build_plan() |
| **tip_run.py** | 239 | **命令行入口**：参数解析、读 /joint_states、发 FollowJointTrajectory、打印报告（含起末位姿） |
| **tip_viz.py** | 120 | **RViz 可视化**：Marker 发布（绿=理想直线/蓝=规划点/红=实际轨迹）+ 实际轨迹采集 + 实际关节数据采集(--actual-csv) |
| **tip_ik.py** | 319 | **解析（闭式）逆解 + 解析雅可比**：不迭代，微秒级，精度比数值解高两个数量级（详见下节） |
| **tip_plot.py** | 150 | **画图（matplotlib）**：三维直线对比、x/y/z-时间、直线度、IK 残差、五关节角，可存 PNG |
| ik_bench.py | 95 | **逆解性能对比**：解析 vs 数值的精度 / 单点耗时 / 整条轨迹耗时 / 连续性 |
| tip_scan_directions.py | 128 | 从起点出发，**哪个方向能走多长直线**（逐点 IK 二分） |
| tip_workspace.py | 95 | 指尖可达范围（纯几何，30 万随机采样，不需要 ROS） |

改算法只看 tip_core.py；改界面/打印看 tip_run.py；改 RViz 显示看 tip_viz.py。

几何全部抄自 `damuzhi_description/urdf/hand.urdf.xacro`（关节 origin/rpy/axis、限位、
指尖偏移 `(-13.946, 0, 31.885) mm`），单位 m / rad；基座是 `base_link`（URDF 里被 fixed_base 焊在 world 上）。

## 逆解：解析闭式解（实时性）

默认逆解是**解析（闭式）解**，不迭代。依据是 DH 表的三个结构性质：

1. `α = [90, −90, 0, 0, 0]` → 后三个关节（pitch/mcp/dip）轴线互相平行，三根连杆
   `a3,a4,a5` 构成一个**平面 3R** 机构，指尖在该平面内运动；
2. 前两个关节（yaw/roll）只负责把该平面摆到"包含目标点"，约束可化为
   `sinθ2·(a1 − B) + cosθ2·(Pz − d1) = d3`（`B = Px·cosθ1 + Py·sinθ1`）→ **θ2 有闭式解**；
3. 平面内用余弦定理由腕点解出 θ3、θ4，θ5 由末杆绝对角 α 回代 → **全程闭式**。

5 自由度对 3 维位置 → 解是 **2 参数族**（外层 θ1 与平面内 α）。两个自由参数都从上一解出发
用坐标下降选，所以轨迹连续、不跳支；万一局部下降失败（大跳变/换支），再用全局网格兜底。

实测（同机同参数，25 mm 直线 60 段，见 `ik_bench.py`）：

| 指标 | 解析闭式 | 数值 DLS（原版） |
|---|---|---|
| IK 残差 max | **0.000076 mm** | 0.0099 mm |
| 整条轨迹规划耗时 | **50 ms** | 1859 ms（**37×**） |
| 相邻点最大关节跳变 | 0.0213 rad | 0.0214 rad（连续性一致） |
| 单点耗时 | 0.24 ms（纯实时路径） | 0.51 ms |

用法（默认就是解析解）：

    python3 tip_run.py --execute --rviz            # 默认 --ik closed
    python3 tip_run.py --offline --ik numeric      # 想对比原数值迭代时用
    python3 ik_bench.py                            # 打印上面这张对比表

## 重要结论：为什么要先"预弯曲"

指尖初始位置对应 q = [0,0,0,0,0]，也就是 **5 个关节全部卡在各自的下限 0** 上 ——
关节空间的**角点**。实测：

| 起点 | 结果 |
|---|---|
| 指尖初始位置（q=0） | `--from-initial` 跑 -z 25 mm：**fraction = 0.0333**（只能走约 0.8 mm 就被限位挡住） |
| 预弯曲构型 q=[0.30, 0.30, 0.40, 0.40, 0.30] | 同一方向 -z 25 mm：**fraction = 1.0000**，IK 残差 0.0098 mm，直线度 0.0068 mm |

所以主程序默认流程是：**先在关节空间把手指摆到预弯曲构型，再从那里走笛卡尔直线**。

想亲眼看到这个限制：

    python3 tip_run.py --offline --from-initial

## 快速上手

    cd ~/jixiebi/chapttan5_damuzhi/tip_cartesian_demo

    # 1) 纯数学自检，不需要启动任何东西
    python3 tip_run.py --offline

    # 2) 看指尖可达范围 / 找合适的终点方向
    python3 tip_workspace.py
    python3 tip_scan_directions.py

    # 3) 连 ROS（另开终端先跑 ros2 launch moveit_config demo.launch.py），只规划不执行
    python3 tip_run.py
    python3 tip_run.py --rviz          # 顺便把规划结果显示到 RViz

    # 4) 真正让拇指动（--rviz 会把实际轨迹实时画出来）
    python3 tip_run.py --execute --rviz

常用参数：

    --end X Y Z               直线终点(m)，默认 = 预弯曲起点沿 -z 走 25mm
    --preflex a b c d e       预弯曲构型(rad)，默认 0.30 0.30 0.40 0.40 0.30
    --steps N                 直线插值点数（默认 60，越大越贴合直线）
    --duration S              总时长（默认 4s，会自动拉长以保证关节速度不超限）
    --vmax V                  关节速度上限 rad/s（默认 0.5）
    --csv PATH                导出规划轨迹 CSV（t + 5 关节角 + 指尖 xyz + 垂距）
    --actual-csv PATH         执行结束后导出实际关节数据 CSV（/joint_states 采样）
    --execute                 真的执行

## 自检输出（offline 实测）

    IK 求解: 直线段完成度 fraction = 1.0000
    IK 残差: max = 0.0098 mm, mean = 0.0051 mm
    时间参数化: 86 点, 总时长 4.00 s, 最大关节速度 0.459 rad/s
    直线度: 指尖到理想直线的最大垂距 = 0.0068 mm
    终点实际到达 = [49.209, 98.839, 138.877] mm, 目标 = [49.209, 98.845, 138.871] mm, 误差 = 0.0085 mm

长一点的斜向目标也能走（留余量后 32 mm）：

    python3 tip_run.py --offline --end 0.049209 0.066845 0.163871
    -> fraction = 1.0000, IK 残差 max = 0.0079 mm, 终点误差 = 0.0035 mm

## 方向扫描实测（预弯曲起点 [49.209, 98.845, 163.871] mm）

tip_scan_directions.py（真 IK 逐点验证，上限 40 mm）排前几名：

| 方向 | 最长直线 |
|---|---|
| -y | ≥ 40 mm |
| (+x, -z) / (+y, -z) / (+x, -y) / (-y, +z) | ≥ 40 mm |
| +x | 38.2 mm |
| -z | 36.9 mm |
| +z | 15.0 mm |
| +y | 16.8 mm |

tip_workspace.py（只看可达性，不保证直线）给的是：+x 38.4 / -x 25.7 / +y 17.5 / -y 176.9 /
+z 13.6 / -z 32.8 mm —— **"可达"≠"能走直线"**，两者差很多时以 tip_scan_directions.py 为准。

## 画图：matplotlib 脚本 + PlotJuggler

### 1) 用 Python 画（tip_plot.py）

需要先装一次 matplotlib（只需一次）：

    python3 -m pip install --user matplotlib

然后：

    # 现场算一遍再画（弹窗）
    python3 tip_plot.py --offline
    python3 tip_plot.py --offline --end 0.049209 0.070845 0.163871

    # 存成 PNG（不弹窗，适合写报告；中文字体会自动找系统的 Noto CJK）
    python3 tip_plot.py --offline --end 0.049209 0.070845 0.163871 --save tip_plot.png

    # 画之前那次执行导出的数据
    python3 tip_run.py --execute --csv tip_traj.csv     # 执行时顺便导出规划轨迹 CSV（含关节角）

    # 执行结束后把"实际跑的关节数据"也导出来（t + q0..q4 + 指尖 xyz + 垂距，来源 /joint_states）
    python3 tip_run.py --execute --rviz --actual-csv tip_exec_joints.csv
    python3 tip_plot.py --csv tip_traj.csv --save tip_plot.png

图里有 6 个子图：① 三维（理想直线 vs 规划指尖轨迹）② 指尖 x/y/z-时间 ③ 到理想直线垂距（直线度）
④ IK 残差 ⑤ 五个关节角 ⑥ 直线段局部放大。

### 2) 用 PlotJuggler 画（实时曲线）

`tip_run.py` 里已经发了两个 PlotJuggler 能直接画的话题（MarkerArray 那个它画不了）：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/tip_cartesian_demo/tip_point` | PointStamped | 实际指尖位置（PJ 里选 point/x、point/y、point/z） |
| `/tip_cartesian_demo/tip_metric` | Float64MultiArray | data/0 = 沿直线走过的距离 mm；data/1 = 到理想直线垂距 mm；data/2 = 目标长度 mm |

用法：先跑 `python3 tip_run.py --execute --rviz`，再打开 PlotJuggler →
**Streaming → ROS2 Topic Subscriber** → 勾上面两个话题即可实时看到曲线。
也可以 **Load data file** 直接打开 `--csv` 导出的 CSV，或者录包后再看：

    ros2 bag record /joint_states /tip_cartesian_demo/tip_point /tip_cartesian_demo/tip_metric

## 运行模式：什么时候 RViz 里能看到东西

| 命令 | 连 ROS | 拇指动 | RViz 里能看到 |
|---|---|---|---|
| --offline | 不连 | 不动 | **什么都看不到**（纯数学演算，只打印数字） |
| 直接运行（无 --execute） | 连（读 /joint_states） | 不动 | 看不到运动；加 --rviz 就能看到"规划出的直线" |
| --rviz（无 --execute） | 连 | 不动 | **绿线=理想直线, 蓝点=规划路径点**（保持 --hold 秒） |
| --execute | 连 | **动**（FakeSystem 假硬件） | 拇指连杆跟着动 + 指尖红球/坐标系移动 |
| --execute --rviz | 连 | **动** | 以上全部 + **红线=实际指尖轨迹（实时画出来）** |

RViz 里加显示：Add -> By topic -> `/tip_cartesian_demo/trace` -> MarkerArray。
颜色：绿=理想直线，蓝=规划路径点，红=实际指尖轨迹，绿球=终点。
再加一个 TF 显示并勾上 `thumb_tip`，就能看到指尖坐标系跟着走。

    # 只想看规划出的直线，不让拇指动
    python3 tip_run.py --rviz

    # 真的动，并在 RViz 里把实际走过的轨迹画出来
    python3 tip_run.py --execute --rviz

    # 终端实时读指尖坐标（执行时数字会跟着变）
    ros2 run tf2_ros tf2_echo base_link thumb_tip

## 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| fraction < 1.0，中途截断 | 终点超出可达范围或撞限位：把 --end 拉近，或先用 tip_scan_directions.py 挑方向 |
| 收不到 /joint_states | 没启动 `ros2 launch moveit_config demo.launch.py` |
| 找不到 /damuzhi_controller/follow_joint_trajectory | ros2_control 没起来，或控制器 spawn 失败 |
| 拇指不动但 error_code=0 | 确认用的是 FakeSystem 假硬件；或 --duration 太大看起来"没动" |
| 想让直线更长 | 用斜向，例如 `--end 0.049209 0.066845 0.163871`（32 mm）；再远要重新扫方向 |

## 与 MoveIt 方案的区别

| | 本程序 | MoveIt compute_cartesian_path |
|---|---|---|
| IK | **自己的数值 IK（位置优先）** | KDL，默认按 6D 完整位姿求解 |
| 5 自由度手指 | 直接可用（只约束位置，冗余自由度由 seed 决定） | 需要 position_only_ik: true 才可能成功 |
| 起点 | 可任意指定（默认预弯曲构型） | 只能是机器人**当前**状态 |
| 冗余自由度/解的选择 | 由"上一点作种子"决定，轨迹连续 | 取决于插件内部实现 |
