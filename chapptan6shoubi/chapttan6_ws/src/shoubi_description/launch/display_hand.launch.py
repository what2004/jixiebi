import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    pkg_share = get_package_share_directory('temp_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'arm_with_hand.urdf.xacro')

    # 灵巧手挂载位姿，可用 launch 参数现场调整（不用改文件）
    mappings = {
        'hand_xyz': context.launch_configurations['hand_xyz'],
        'hand_rpy': context.launch_configurations['hand_rpy'],
    }
    robot_description = {'robot_description': xacro.process_file(xacro_file, mappings=mappings).toxml()}
    rviz_config = os.path.join(pkg_share, 'rviz', 'arm_hand_view.rviz')

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen', parameters=[robot_description]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             name='joint_state_publisher_gui', output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', rviz_config], output='screen'),
    ]


def generate_launch_description():
    """机械臂 youbi_tc7_arm + LinkerHand L20 灵巧手（拼装后完整模型）+ RViz2

    调挂载姿态示例
      ros2 launch temp_description display_hand.launch.py hand_rpy:="0 0 1.5708"
      ros2 launch temp_description display_hand.launch.py hand_xyz:="0 0 -0.01" hand_rpy:="3.1416 0 0"
    """
    return LaunchDescription([
        DeclareLaunchArgument('hand_xyz', default_value='0 0 0',
                              description='灵巧手相对机械臂 hand_mount 的平移 (m)'),
        DeclareLaunchArgument('hand_rpy', default_value='0 0 0',
                              description='灵巧手相对机械臂 hand_mount 的姿态 (rad, 固定轴 XYZ)'),
        OpaqueFunction(function=_nodes),
    ])
