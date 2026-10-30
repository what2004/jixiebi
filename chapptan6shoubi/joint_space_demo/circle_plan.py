#!/usr/bin/env python3
"""离线验证固定握笔姿态的笛卡尔圆周轨迹；不启动 ROS 节点或发送运动指令。

source ROS 和项目环境后运行。只验证运动学、关节限位和轨迹速度，未做碰撞规划。
"""
import argparse
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory

from jt_core import JOINTS_ARM, JOINTS_HAND, LIMITS

HERE = Path(__file__).resolve().parent


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


def transform(origin):
    result = np.eye(4)
    if origin is not None:
        r, p, y = map(float, origin.get('rpy', '0 0 0').split())
        result[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
        result[:3, 3] = list(map(float, origin.get('xyz', '0 0 0').split()))
    return result


class PenKinematics:
    def __init__(self, preset):
        package = Path(get_package_share_directory('temp_description'))
        mappings = {'with_pen': 'true', **{name: ' '.join(map(str, preset[name]))
                    for name in ['pen_xyz', 'pen_rpy']}}
        self.description = xacro.process_file(str(package / 'urdf/arm_with_hand.urdf.xacro'),
                                              mappings=mappings).toxml()
        robot = ET.fromstring(self.description)
        self.robot = robot
        self.joints = []
        for joint in robot.findall('joint'):
            axis = joint.find('axis')
            self.joints.append((joint.get('name'), joint.find('parent').get('link'),
                                joint.find('child').get('link'), transform(joint.find('origin')),
                                np.array(list(map(float, axis.get('xyz').split()))) if axis is not None else None))
        self.fixed_hand = {name: preset['goal'][name] for name in JOINTS_HAND}
        self.lower = np.array([LIMITS[name][0] for name in JOINTS_ARM])
        self.upper = np.array([LIMITS[name][1] for name in JOINTS_ARM])

    def hand_points_in_tip(self, q):
        """实际手部网格顶点相对笔尖的坐标，用于检查水平纸面净空。"""
        tip, frames = self.forward(q)
        children = {'hand_base_link'}
        for _ in self.joints:
            for _, parent, child, _, _ in self.joints:
                if parent in children and child not in ['pen_link', 'pen_tip']:
                    children.add(child)
        clouds = {}
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        for link in self.robot.findall('link'):
            name = link.get('name')
            if name not in children:
                continue
            parts = []
            for visual in link.findall('visual'):
                mesh = visual.find('geometry/mesh')
                if mesh is None:
                    continue
                package, relative = mesh.get('filename').removeprefix('package://').split('/', 1)
                data = (Path(get_package_share_directory(package)) / relative).read_bytes()
                count = struct.unpack_from('<I', data, 80)[0]
                if len(data) != 84 + 50 * count:
                    raise ValueError('纸面净空检查需要二进制 STL')
                vertices = np.unique(np.frombuffer(data, dtype=dtype, offset=84, count=count)
                                     ['vertices'].reshape(-1, 3), axis=0).astype(float)
                vertices *= np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
                pose = frames[name] @ transform(visual.find('origin'))
                world_relative = vertices @ pose[:3, :3].T + pose[:3, 3] - tip[:3, 3]
                parts.append(world_relative @ tip[:3, :3])
            if parts:
                clouds[name] = np.vstack(parts)
        return clouds

    def forward(self, q, jacobian=False):
        values = {**self.fixed_hand, **dict(zip(JOINTS_ARM, q))}
        frames = {'world': np.eye(4)}
        axes = {}
        pending = list(self.joints)
        while pending:
            progressed = False
            for index in range(len(pending) - 1, -1, -1):
                name, parent, child, origin, axis = pending[index]
                if parent not in frames:
                    continue
                before = frames[parent] @ origin
                motion = np.eye(4)
                if axis is not None:
                    motion[:3, :3] = rotation(axis, values.get(name, 0.))
                    if name in JOINTS_ARM:
                        axes[name] = (before[:3, 3], before[:3, :3] @ axis)
                frames[child] = before @ motion
                pending.pop(index)
                progressed = True
            if not progressed:
                raise ValueError('URDF 的关节树不连通')
        tip = frames['pen_tip']
        if not jacobian:
            return tip, frames
        jac = np.zeros((6, 7))
        for i, name in enumerate(JOINTS_ARM):
            pivot, direction = axes[name]
            jac[:3, i] = np.cross(direction, tip[:3, 3] - pivot)
            jac[3:, i] = direction
        return tip, jac

    def inverse(self, position, orientation, seed, reference):
        q = np.array(seed, dtype=float)
        for _ in range(100):
            tip, jac = self.forward(q, jacobian=True)
            relative = orientation @ tip[:3, :3].T
            angular = .5 * np.array([relative[2, 1] - relative[1, 2],
                                     relative[0, 2] - relative[2, 0],
                                     relative[1, 0] - relative[0, 1]])
            linear = position - tip[:3, 3]
            angle = math.acos(float(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
            if np.linalg.norm(linear) < 2e-6 and angle < 2e-5:
                return q
            # 旋转误差换算为等效长度；七轴冗余方向轻微回拉到初始姿态。
            error = np.r_[linear, .15 * angular]
            jac[3:] *= .15
            pseudo = jac.T @ np.linalg.solve(jac @ jac.T + 1e-8 * np.eye(6), np.eye(6))
            step = pseudo @ error + (np.eye(7) - pseudo @ jac) @ (.12 * (reference - q))
            step *= min(1., .08 / max(float(np.max(np.abs(step))), 1e-12))
            q = np.clip(q + step, self.lower + 1e-7, self.upper - 1e-7)
        raise ValueError(f'IK 未收敛，目标 {position.tolist()}')


def circle_plan(preset, radius=.03, duration=8., samples=360):
    if not math.isfinite(radius) or radius <= 0 or not math.isfinite(duration) or duration <= 0 or samples < 60:
        raise ValueError('半径和时长必须为正数，采样点数至少 60')
    model = PenKinematics(preset)
    initial = np.array([preset['goal'][name] for name in JOINTS_ARM])
    start, _ = model.forward(initial)
    orientation = start[:3, :3]
    hand_points = model.hand_points_in_tip(initial)
    def paper_clearance(orientation):
        return min(float(np.min(points @ orientation[2, :])) for points in hand_points.values())
    initial_clearance = paper_clearance(orientation)
    roll = 0.
    # 圆柱笔绕自身轴旋转不改变笔尖或倾角，可据此抬高小指侧。
    if initial_clearance < .005:
        for degrees in sorted([i * .5 for i in range(-90, 91)], key=abs):
            candidate = orientation @ rotation([0, 0, 1], math.radians(degrees))
            if paper_clearance(candidate) >= .005:
                initial = model.inverse(start[:3, 3], candidate, initial, initial)
                start, _ = model.forward(initial)
                orientation = start[:3, :3]
                roll = degrees
                break
        else:
            raise ValueError('保持握笔形状时，无法在允许的绕笔轴旋转范围内获得纸面净空')
    final_clearance = paper_clearance(orientation)
    above = model.inverse(start[:3, 3] + np.array([0, 0, .03]), orientation, initial, initial)
    center = start[:3, 3] - np.array([radius, 0, 0])
    # 周期五次时间律使落笔后的起末速度和加速度均为零。
    times = np.linspace(0, duration, samples + 1)
    tau = times / duration
    progress = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    angles = 2 * math.pi * progress
    positions = center + np.column_stack([radius * np.cos(angles), radius * np.sin(angles), np.zeros_like(angles)])
    path = [initial]
    for position in positions[1:]:
        path.append(model.inverse(position, orientation, path[-1], initial))
    path = np.array(path)
    actual = []
    angular_errors = []
    for q in path:
        tip, _ = model.forward(q)
        actual.append(tip[:3, 3])
        angular_errors.append(math.acos(float(np.clip((np.trace(orientation @ tip[:3, :3].T) - 1) / 2, -1, 1))))
    actual = np.array(actual)
    velocity = np.gradient(path, times, axis=0)
    acceleration = np.gradient(velocity, times, axis=0)
    summary = {
        'radius_m': radius, 'duration_s': duration, 'samples': len(path),
        'circle_center_world_m': center.tolist(), 'start_tip_world_m': start[:3, 3].tolist(),
        'paper_height_m': float(center[2]), 'pen_axis_world': (orientation @ np.array([0., 0., 1.])).tolist(),
        'max_tip_position_error_m': float(np.max(np.linalg.norm(actual - positions, axis=1))),
        'max_tip_orientation_error_rad': max(angular_errors),
        'max_radial_error_m': float(np.max(np.abs(np.linalg.norm(actual[:, :2] - center[:2], axis=1) - radius))),
        'max_height_error_m': float(np.max(np.abs(actual[:, 2] - center[2]))),
        'joint_loop_closure_error_rad': float(np.max(np.abs(path[-1] - path[0]))),
        'max_adjacent_joint_step_rad': float(np.max(np.abs(np.diff(path, axis=0)))),
        'peak_joint_velocity_rad_s': float(np.max(np.abs(velocity))),
        'peak_joint_acceleration_rad_s2': float(np.max(np.abs(acceleration))),
        'joint_limits_ok': bool(np.all(path >= model.lower) and np.all(path <= model.upper)),
        'joint_velocity_limits_ok': bool(np.all(np.abs(velocity) <= np.array([LIMITS[n][2] for n in JOINTS_ARM]))),
        'initial_hand_clearance_above_paper_m': initial_clearance,
        'drawing_hand_clearance_above_paper_m': final_clearance,
        'preparation_roll_about_pen_deg': roll,
        'preparation_above_paper_arm_positions': above.tolist(),
        'draw_start_arm_positions': initial.tolist(),
        'collision_checked': False,
        'notes': '运动学及水平纸面手部净空验证；未做完整碰撞、接触力或动力学检查。圆周起末停稳，手指保持已接受的握笔姿态。',
    }
    return model, summary, times, path, positions, actual


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--radius', type=float, default=.03)
    parser.add_argument('--duration', type=float, default=8.)
    parser.add_argument('--samples', type=int, default=360)
    parser.add_argument('--outdir', type=Path, default=HERE.parent / 'traj/circle_plan')
    args = parser.parse_args()
    preset = json.loads((HERE / 'pen_grasp.json').read_text())
    _, summary, times, path, positions, actual = circle_plan(preset, args.radius, args.duration, args.samples)
    args.outdir.mkdir(parents=True, exist_ok=True)
    combined = np.column_stack([path, np.tile([preset['goal'][n] for n in JOINTS_HAND], (len(path), 1))])
    output = {'joint_names': JOINTS_ARM + JOINTS_HAND, 'summary': summary,
              'points': [{'time_from_start': float(t), 'positions': q.tolist(),
                          'target_tip_world_m': goal.tolist(), 'actual_tip_world_m': tip.tolist()}
                         for t, q, goal, tip in zip(times, combined, positions, actual)]}
    (args.outdir / 'circle_trajectory.json').write_text(json.dumps(output, ensure_ascii=False, indent=2) + '\n')
    (args.outdir / 'verification.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if not summary['joint_limits_ok'] or not summary['joint_velocity_limits_ok']:
        raise SystemExit('轨迹超出关节角或速度限制')


if __name__ == '__main__':
    main()
