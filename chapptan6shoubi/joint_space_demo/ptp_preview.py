#!/usr/bin/env python3
"""仅发布模型关节状态的 RViz PTP 动画；不向控制器发送指令。"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from sensor_msgs.msg import JointState

from jt_core import JOINTS_ARM, JOINTS_HAND, build_plan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', type=Path, default=Path(__file__).with_name('ptp_target.json'))
    parser.add_argument('--loop', action='store_true', help='到达后停留，再返回起点，循环播放')
    args, ros_args = parser.parse_known_args()
    target = json.loads(args.target.read_text())
    names = JOINTS_ARM + JOINTS_HAND
    if set(target['goal']) != set(names):
        raise ValueError('目标必须恰好包含机械臂 7 个、手指 21 个关节')
    goal = np.array([target['goal'][name] for name in names], dtype=float)
    duration = float(target['duration_s'])
    if not np.all(np.isfinite(goal)) or not math.isfinite(duration) or duration <= 0:
        raise ValueError('目标角必须为有限值，时长必须为正数')
    for group, values in [('arm', goal[:7]), ('hand', goal[7:])]:
        plan = build_plan(group, np.zeros(len(values)), values, duration=duration, steps=240)
        if plan.violations or np.any(plan.peak_v > 2.0 if group == 'arm' else plan.peak_v > 1.0):
            raise ValueError(str(plan.violations) + ' 请检查目标或增大时长')
        if np.any(plan.peak_a > (10.0 if group == 'arm' else 20.0)):
            raise ValueError('加速度超限，请增大时长')

    rclpy.init(args=ros_args)
    node = rclpy.create_node('ptp_preview')
    pub = node.create_publisher(JointState, '/joint_states', 10)
    msg = JointState()
    msg.name = names
    start = None
    last_phase = None

    def tick():
        nonlocal start, last_phase
        # 等待模型订阅后再留出三秒供 RViz 初始化。
        if start is None and pub.get_subscription_count() > 0:
            start = time.monotonic() + 3.0
        elapsed = max(0., time.monotonic() - start) if start is not None else 0.
        if args.loop:
            cycle = 2 * duration + 5.0
            elapsed %= cycle
        if elapsed < duration:
            fraction = elapsed / duration
            phase = '零位 → 目标'
        elif not args.loop or elapsed < duration + 3.0:
            fraction = 1.0
            phase = '保持截图目标姿态'
        elif elapsed < 2 * duration + 3.0:
            fraction = 1.0 - (elapsed - duration - 3.0) / duration
            phase = '目标 → 零位'
        else:
            fraction = 0.0
            phase = '保持初始零位'
        blend = 0.5 * (1.0 - math.cos(math.pi * fraction))
        msg.position = (goal * blend).tolist()
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        if phase != last_phase:
            node.get_logger().info(phase)
            last_phase = phase

    node.create_timer(1.0 / 60.0, tick)
    node.get_logger().info(f'28 关节同步 PTP，单程 {duration:.1f}s；目标 {args.target}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
