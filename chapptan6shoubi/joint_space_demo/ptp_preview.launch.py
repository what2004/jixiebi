"""每个窗口独立发布关节状态和 TF 的 RViz 点到点动画。"""
from pathlib import Path
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    here = Path(__file__).resolve().parent
    package = Path(get_package_share_directory('temp_description'))
    description = xacro.process_file(str(package / 'urdf/arm_with_hand.urdf.xacro')).toxml()
    # 多次启动也不能共用关节状态或 TF，否则不同播放进度会使模型跳变。
    namespace = 'ptp_' + uuid4().hex[:8]
    remappings = [('/tf', f'/{namespace}/tf'),
                  ('/tf_static', f'/{namespace}/tf_static')]
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=namespace, remappings=remappings,
             parameters=[{'robot_description': description}], output='screen'),
        ExecuteProcess(cmd=['/usr/bin/python3', str(here / 'ptp_preview.py'), '--loop',
                            '--ros-args', '-r', f'__ns:=/{namespace}',
                            '-r', f'/joint_states:=/{namespace}/joint_states'],
                       output='screen'),
        Node(package='rviz2', executable='rviz2',
             namespace=namespace,
             remappings=remappings + [('/robot_description', f'/{namespace}/robot_description')],
             arguments=['-d', str(package / 'rviz/arm_hand_view.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz')), output='screen'),
    ])
