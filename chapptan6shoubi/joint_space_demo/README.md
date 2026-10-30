# 点到点动画

`ptp_target.json` 保存截图中的全部 28 个目标关节角（弧度）。起点为肩部固定、直臂向下的零位。

启动独立 RViz 动画：

```bash
source /opt/ros/humble/setup.bash
source ~/jixiebi/chapptan6shoubi/chapttan6_ws/install/setup.bash
ROS_DOMAIN_ID=76 ros2 launch ~/jixiebi/chapptan6shoubi/joint_space_demo/ptp_preview.launch.py
```

等待模型订阅并初始化后，机械臂与手指同时用 5 秒从零位平滑运动到目标，停留 3 秒，再用 5 秒返回，停留 2 秒后循环。修改 JSON 可调整角度和单程时长，修改后重新启动。关闭终端中的启动命令（Ctrl+C）可停止整组节点。

此动画用于模型可视化，没有接入动力学、接触或碰撞规划。每次启动自动生成独立的 `ptp_` 命名空间，隔离关节状态、机器人描述和 TF；多个动画窗口和滑块窗口可同时运行。

`traj/ptp_target/`（项目根目录下）保存此次生成的 241 点轨迹、CSV 和参数记录。

## 三指握笔预览

默认加载 `pen_grasp.json` 中的三指握笔候选姿态及笔的位姿。食指舒缓弯曲，拇指从侧面夹持，中指用侧面托笔，无名指和小指收拢；笔尾朝向虎口。与最初的钩状姿态相比，食指末节从约 80° 改为约 25°、中指末节改为约 7°，并把握持位置移到靠近笔尖的一段。只在握笔预设中调整了腕部关节，使笔尖向下约 60°；原始 `ptp_target.json` 不变。下面命令同时打开关节滑块，拖动后的角度不会自动写回文件。

```bash
source /opt/ros/humble/setup.bash
source ~/jixiebi/chapptan6shoubi/chapttan6_ws/install/setup.bash
ROS_DOMAIN_ID=76 ros2 launch ~/jixiebi/chapptan6shoubi/joint_space_demo/pen_preview.launch.py gui:=true
```

默认是手部近景。追加 `view:=full` 查看整条机械臂；设为 `gui:=false` 隐藏滑块。每个窗口使用独立命名空间，不会与此前的动画窗口争用关节状态或 TF。

蓝色圆柱笔杆长 160 mm、直径 12 mm，深灰笔尖用 40 层薄圆柱近似圆锥，每层厚 0.5 mm，笔尖长 20 mm，总长 180 mm。笔的尺寸没有改变。模型定义在 `chapttan6_ws/src/shoubi_description/urdf/pen.urdf.xacro`，笔尖坐标系为 `pen_tip`。

握笔预设中的 `goal` 保存 28 个关节角（弧度），`pen_xyz` 和 `pen_rpy` 保存笔相对于手掌的位置（米）和朝向（弧度）。可以直接编辑 JSON 并重启，也可用同名 launch 参数临时覆盖笔位姿。普通模型和 PTP 动画默认不加载笔；组合模型使用 `with_pen:=true` 加载。

### 验证与预览图

使用 numpy 和 Pillow，从实际手部 STL 生成四视图并检查笔与手部表面采样的间隙：

```bash
cd ~/jixiebi/chapptan6shoubi
python3 joint_space_demo/inspect_pen_grasp.py --render
```

结果保存在 `traj/pen_grasp/pen_grasp_views.png` 和 `geometry_check.json`。采样包含全部网格顶点、三角形中心和边中点；当前预设没有采样点落入笔的包络，三指指端最小采样间隙约 0.16–0.40 mm。参考的是三指握笔的形态，视觉自然程度仍以实际预览反馈为准。

这是静态可视化握笔：笔固定跟随手掌，没有验证摩擦、抓取力或完整自碰撞，也没有增加画圆运动。

## 画圆运动学验证

保持 `pen_grasp.json` 中的手指角度和笔尖朝向，离线计算水平圆周的连续逆运动学：

```bash
source /opt/ros/humble/setup.bash
source ~/jixiebi/chapptan6shoubi/chapttan6_ws/install/setup.bash
cd ~/jixiebi/chapptan6shoubi
python3 joint_space_demo/circle_plan.py --radius 0.03 --duration 8
```

默认从当前笔尖位置出发，圆心位于其世界 X 方向负侧 3 cm，纸面高度约 0.8733 m。一圈 361 个路点，笔的倾斜方向保持不变，使用五次时间律在圆周起末停稳。程序直接从 xacro 读取模型和固定笔尖偏移，通过雅可比阻尼逆解生成关节角，并检查笔尖误差、关节限位、相邻点连续性和速度。

纸面检查发现，直接使用静态握笔的腕部姿态时，小指部分网格会低于纸面约 1.5 mm。规划器因此保持手指角度不变，先将整只手绕笔轴旋转约 41.5°，使手部最低点距纸面约 5 mm；笔尖位置和笔的倾角不变。准备点位于纸面上方 3 cm，后续动画应从这个准备姿态开始，再落笔。静态握笔预设不会被覆盖。

结果写入 `traj/circle_plan/circle_trajectory.json` 与 `verification.json`。这一步不启动节点、不播放动画，也没有完整碰撞检查。完整动画的启动方法见下一节。

## 握笔画圆动画

```bash
source /opt/ros/humble/setup.bash
source ~/jixiebi/chapptan6shoubi/chapttan6_ws/install/setup.bash
ROS_DOMAIN_ID=76 ros2 launch ~/jixiebi/chapptan6shoubi/joint_space_demo/circle_preview.launch.py
```

默认打开纸面与手部近景：准备 1.5 秒、落笔 2 秒、停稳 0.35 秒、画圆 8 秒、完成停留 0.8 秒、抬笔 2 秒、展示 2.5 秒后循环。笔迹只在画圆阶段逐步出现，在空中准备下一圈时清除。手指保持已接受的握笔姿态；循环首尾关节角一致。

参数可追加在命令末尾：`view:=full` 显示完整机械臂与桌子，`loop:=false` 画一次后停在抬笔姿态，`radius:=0.03` 设置半径（米），`duration:=8` 设置画圆秒数。每次启动自动隔离关节状态、TF 和场景话题。

`circle_preview.py --check-only` 可离线验证完整动画。启动时也会先检查关节限位、速度、插值后笔尖误差和落笔的水平偏移；报告与整段轨迹写入 `traj/circle_animation/`（launch 启动的各实例位于各自命名空间子目录）。动画只发布模型状态和 RViz 场景，不连接控制器；没有动力学或完整碰撞规划。
