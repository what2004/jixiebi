"""独立的握笔画圆动画窗口。"""
import json
from pathlib import Path
from uuid import uuid4
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from circle_plan import PenKinematics


def nodes(context):
    here = Path(__file__).resolve().parent
    preset = json.loads((here / 'pen_grasp.json').read_text())
    description = PenKinematics(preset).description
    namespace = 'circle_' + uuid4().hex[:8]
    remappings = [('/tf', f'/{namespace}/tf'), ('/tf_static', f'/{namespace}/tf_static')]
    view = LaunchConfiguration('view').perform(context)
    config = here / ('circle_view.rviz' if view == 'hand' else 'circle_full_view.rviz')
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=namespace,
             parameters=[{'robot_description': description}], remappings=remappings, output='screen'),
        ExecuteProcess(cmd=['/usr/bin/python3', str(here / 'circle_preview.py'),
                            '--radius', LaunchConfiguration('radius'), '--duration', LaunchConfiguration('duration'),
                            '--loop', LaunchConfiguration('loop'),
                            '--outdir', str(here.parent / 'traj/circle_animation' / namespace),
                            '--ros-args', '-r', f'__ns:=/{namespace}',
                            '-r', f'/tf:=/{namespace}/tf', '-r', f'/tf_static:=/{namespace}/tf_static'], output='screen'),
        Node(package='rviz2', executable='rviz2', namespace=namespace,
             remappings=remappings + [('/robot_description', f'/{namespace}/robot_description'),
                                      ('/drawing_scene', f'/{namespace}/drawing_scene'),
                                      ('visualization_marker_array', f'/{namespace}/drawing_scene')],
             arguments=['-d', str(config)], condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('radius', default_value='0.03'),
        DeclareLaunchArgument('duration', default_value='8'),
        DeclareLaunchArgument('loop', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('view', default_value='hand', choices=['hand', 'full']),
        OpaqueFunction(function=nodes),
    ])
