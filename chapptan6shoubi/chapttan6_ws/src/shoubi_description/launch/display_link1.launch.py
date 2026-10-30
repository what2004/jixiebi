import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """仅 base_link(LINK1) 最小排查模型

    xacro 是唯一真源：这里在启动时现场展开成 URDF 字符串并作为
    robot_description 参数下发，因此仓库里不再需要预先导出的 .urdf
    （避免出现"改了 xacro 但 .urdf 没重新生成"的过期模型问题）。
    """
    pkg_share = get_package_share_directory('temp_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'link1_only.urdf.xacro')
    robot_description = {'robot_description': xacro.process_file(xacro_file).toxml()}

    rviz_config = os.path.join(pkg_share, 'rviz', 'link1_view.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
