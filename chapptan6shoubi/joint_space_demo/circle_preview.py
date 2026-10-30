#!/usr/bin/env python3
"""RViz 握笔画圆：准备、落笔、画圆、抬笔，保持手指姿态。"""
import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import numpy as np

from circle_plan import circle_plan, JOINTS_ARM, JOINTS_HAND, LIMITS

HERE = Path(__file__).resolve().parent
LABELS = {'ready': 'Ready', 'lower': 'Lowering pen', 'contact': 'Pen on paper',
          'draw': 'Drawing circle', 'complete': 'Circle complete',
          'lift': 'Lifting pen', 'hold': 'Finished'}


def smooth(t):
    return 10 * t**3 - 15 * t**4 + 6 * t**5


def interpolate(times, path, t):
    return np.array([np.interp(t, times, path[:, i]) for i in range(path.shape[1])])


@dataclass
class Sample:
    arm: np.ndarray
    phase: str
    local_time: float


class CircleAnimation:
    def __init__(self, preset, radius=.03, duration=8.):
        self.model, self.summary, self.draw_times, self.draw_q, _, _ = circle_plan(preset, radius, duration)
        if not self.summary['joint_limits_ok'] or not self.summary['joint_velocity_limits_ok']:
            raise ValueError('圆周轨迹超出关节角或速度限制')
        # 闭环末端解与起点仅相差数微弧度，统一到同一关节姿态以连续抬笔。
        if np.max(np.abs(self.draw_q[-1] - self.draw_q[0])) > 1e-4:
            raise ValueError('圆周末端关节姿态不连续')
        self.draw_q[-1] = self.draw_q[0]
        self.hand = np.array([preset['goal'][n] for n in JOINTS_HAND])
        self.center = np.array(self.summary['circle_center_world_m'])
        self.radius = radius
        self.duration = duration
        self.tip_start, _ = self.model.forward(self.draw_q[0])
        self.plane = float(self.center[2])
        self.stroke = np.array([self.model.forward(q)[0][:3, 3] for q in self.draw_q])
        self.move_times = np.linspace(0, 2., 121)
        self.lift_height = .03
        up = [self.draw_q[0]]
        for t in self.move_times[1:]:
            position = self.tip_start[:3, 3] + [0, 0, self.lift_height * smooth(t / 2.)]
            up.append(self.model.inverse(position, self.tip_start[:3, :3], up[-1], self.draw_q[0]))
        self.up_q = np.array(up)
        self.lower_q = self.up_q[::-1].copy()
        self.phases = [('ready', 1.5), ('lower', 2.), ('contact', .35), ('draw', duration),
                       ('complete', .8), ('lift', 2.), ('hold', 2.5)]
        self.bounds = np.r_[0., np.cumsum([length for _, length in self.phases])]
        self.cycle_time = float(self.bounds[-1])
        self.draw_begin = float(self.bounds[3])
        self.draw_end = float(self.bounds[4])

    def sample(self, elapsed, loop=True):
        t = elapsed % self.cycle_time if loop else min(elapsed, self.cycle_time - 1e-9)
        index = min(int(np.searchsorted(self.bounds, t, side='right') - 1), len(self.phases) - 1)
        phase = self.phases[index][0]
        local = t - self.bounds[index]
        if phase in ['ready', 'hold']:
            q = self.up_q[-1]
        elif phase == 'lower':
            q = interpolate(self.move_times, self.lower_q, local)
        elif phase == 'draw':
            q = interpolate(self.draw_times, self.draw_q, local)
        elif phase == 'lift':
            q = interpolate(self.move_times, self.up_q, local)
        else:
            q = self.draw_q[0]
        return Sample(q.copy(), phase, local)

    def trace(self, sample, tip):
        if sample.phase in ['ready', 'lower', 'contact']:
            return np.empty((0, 3))
        if sample.phase == 'draw':
            index = int(np.searchsorted(self.draw_times, sample.local_time, side='right'))
            points = np.vstack([self.stroke[:index], tip])
        else:
            points = self.stroke.copy()
        points[:, 2] = self.plane + .0006
        return points

    def verify(self, output):
        output.mkdir(parents=True, exist_ok=True)
        times = np.linspace(0, self.cycle_time, math.ceil(self.cycle_time * 120) + 1)
        samples = [self.sample(t, loop=False) for t in times]
        q = np.array([s.arm for s in samples])
        velocity = np.gradient(q, times, axis=0)
        acceleration = np.gradient(velocity, times, axis=0)
        linear_errors, rotation_errors = [], []
        lower_xy_errors = []
        draw_min_height = math.inf
        draw_max_height = -math.inf
        for sample in samples:
            tip, _ = self.model.forward(sample.arm)
            if sample.phase == 'draw':
                angle = 2 * math.pi * smooth(sample.local_time / self.duration)
                desired = self.center + [self.radius * math.cos(angle), self.radius * math.sin(angle), 0]
                linear_errors.append(float(np.linalg.norm(tip[:3, 3] - desired)))
                rotation_errors.append(math.acos(float(np.clip((np.trace(self.tip_start[:3, :3] @ tip[:3, :3].T) - 1) / 2, -1, 1))))
                draw_min_height = min(draw_min_height, float(tip[2, 3]))
                draw_max_height = max(draw_max_height, float(tip[2, 3]))
            elif sample.phase in ['lower', 'lift']:
                lower_xy_errors.append(float(np.linalg.norm(tip[:2, 3] - self.tip_start[:2, 3])))
        report = {
            'radius_m': self.radius, 'draw_duration_s': self.duration, 'cycle_duration_s': self.cycle_time,
            'paper_height_m': self.plane, 'circle_center_world_m': self.center.tolist(),
            'phase_order': [name for name, _ in self.phases], 'validation_samples': len(times),
            'joint_limits_ok': bool(np.all(q >= self.model.lower) and np.all(q <= self.model.upper)),
            'joint_velocity_limits_ok': bool(np.all(np.abs(velocity) <= [LIMITS[n][2] for n in JOINTS_ARM])),
            'peak_joint_velocity_rad_s': float(np.max(np.abs(velocity))),
            'peak_joint_acceleration_rad_s2': float(np.max(np.abs(acceleration))),
            'max_interpolated_circle_position_error_m': max(linear_errors),
            'max_interpolated_circle_orientation_error_rad': max(rotation_errors),
            'max_lower_lift_horizontal_error_m': max(lower_xy_errors),
            'draw_tip_height_range_m': [draw_min_height, draw_max_height],
            'hand_clearance_nominal_m': self.summary['drawing_hand_clearance_above_paper_m'],
            'loop_joint_closure_error_rad': float(np.max(np.abs(self.sample(0).arm - self.sample(self.cycle_time, loop=False).arm))),
            'hand_pose_fixed': True, 'full_collision_checked': False,
        }
        (output / 'animation_verification.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        (output / 'motion_plan.json').write_text(json.dumps({
            'joint_names': JOINTS_ARM + JOINTS_HAND, 'summary': self.summary,
            'points': [{'time_from_start': float(t), 'phase': s.phase,
                        'positions': np.r_[s.arm, self.hand].tolist()}
                       for t, s in zip(times, samples)]}, ensure_ascii=False, indent=2) + '\n')
        if not report['joint_limits_ok'] or not report['joint_velocity_limits_ok']:
            raise ValueError('完整动画超出关节限位或速度限制')
        if max(linear_errors) > .0001 or max(lower_xy_errors) > .0001:
            raise ValueError('轨迹插值的笔尖误差超出 0.1 mm')
        return report


def run(animation, loop):
    import rclpy
    from geometry_msgs.msg import Point, TransformStamped
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    node = rclpy.create_node('circle_preview')
    joint_pub = node.create_publisher(JointState, 'joint_states', 10)
    scene_pub = node.create_publisher(MarkerArray, 'drawing_scene',
                                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    phase_pub = node.create_publisher(String, 'draw_phase', 10)
    frame = TransformStamped()
    frame.header.frame_id = 'world'
    frame.child_frame_id = 'drawing_frame'
    frame.header.stamp = node.get_clock().now().to_msg()
    frame.transform.translation.x, frame.transform.translation.y = map(float, animation.center[:2])
    frame.transform.translation.z = animation.plane
    frame.transform.rotation.w = 1.
    broadcaster = StaticTransformBroadcaster(node)
    broadcaster.sendTransform(frame)

    def marker(ident, kind, color):
        item = Marker()
        item.header.frame_id = 'world'
        item.ns = 'drawing'
        item.id = ident
        item.type = kind
        item.action = Marker.ADD
        item.pose.orientation.w = 1.
        item.color.r, item.color.g, item.color.b, item.color.a = color
        return item

    def box(ident, xyz, size, color):
        item = marker(ident, Marker.CUBE, color)
        item.pose.position.x, item.pose.position.y, item.pose.position.z = map(float, xyz)
        item.scale.x, item.scale.y, item.scale.z = size
        return item

    x, y = map(float, animation.center[:2])
    z = animation.plane
    paper_x = max(.22, 2 * animation.radius + .12)
    paper_y = max(.18, 2 * animation.radius + .10)
    table_x, table_y = paper_x + .16, paper_y + .16
    static_scene = [box(0, [x, y, z - .0135], [table_x, table_y, .025], (.52, .33, .18, 1.)),
                    box(1, [x, y, z - .0005], [paper_x, paper_y, .001], (.97, .97, .94, 1.))]
    leg_height = z - .026
    for index, (dx, dy) in enumerate([(-1, -1), (-1, 1), (1, -1), (1, 1)]):
        static_scene.append(box(10 + index, [x + dx * (table_x / 2 - .035), y + dy * (table_y / 2 - .035), leg_height / 2],
                                [.025, .025, leg_height], (.26, .28, .31, 1.)))
    line = marker(2, Marker.LINE_STRIP, (.04, .20, .66, 1.))
    line.scale.x = .0014
    status = marker(3, Marker.TEXT_VIEW_FACING, (.12, .19, .25, 1.))
    status.scale.z = .012
    status.pose.position.x, status.pose.position.y, status.pose.position.z = x, y - paper_y / 2 - .025, z + .025
    joints = JointState()
    joints.name = JOINTS_ARM + JOINTS_HAND
    start = None
    previous_phase = None

    def tick():
        nonlocal start, previous_phase
        # 等待模型订阅并留三秒让 RViz 加载网格；循环在空中清除笔迹。
        if start is None and joint_pub.get_subscription_count() > 0:
            start = time.monotonic() + 3.
        elapsed = max(0., time.monotonic() - start) if start is not None else 0.
        sample = animation.sample(elapsed, loop=loop)
        now = node.get_clock().now().to_msg()
        joints.header.stamp = now
        joints.position = np.r_[sample.arm, animation.hand].tolist()
        joint_pub.publish(joints)
        tip, _ = animation.model.forward(sample.arm)
        trace = animation.trace(sample, tip[:3, 3])
        line.action = Marker.ADD if len(trace) >= 2 else Marker.DELETE
        line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in trace]
        status.text = LABELS[sample.phase]
        for item in static_scene + [line, status]:
            item.header.stamp = now
        scene_pub.publish(MarkerArray(markers=static_scene + [line, status]))
        phase_pub.publish(String(data=sample.phase))
        if sample.phase != previous_phase:
            node.get_logger().info(LABELS[sample.phase])
            previous_phase = sample.phase

    node.get_logger().info(f'半径 {animation.radius * 100:.1f} cm，画圆 {animation.duration:.1f}s；手指固定；准备/落笔/画圆/抬笔')
    node.create_timer(1 / 60., tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--radius', type=float, default=.03)
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--loop', choices=['true', 'false'], default='true')
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--outdir', type=Path, default=HERE.parent / 'traj/circle_animation')
    args, ros_args = parser.parse_known_args()
    preset = json.loads((HERE / 'pen_grasp.json').read_text())
    animation = CircleAnimation(preset, args.radius, args.duration)
    report = animation.verify(args.outdir)
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    if args.check_only:
        return
    import rclpy
    rclpy.init(args=ros_args)
    run(animation, args.loop == 'true')


if __name__ == '__main__':
    main()
