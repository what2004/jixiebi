import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """机械臂 youbi_tc7_arm + LinkerHand L20 的 MoveIt2 演示

      ros2 launch youbi_moveit_config demo.launch.py                 # 带 RViz
      ros2 launch youbi_moveit_config demo.launch.py use_rviz:=false  # 只起 move_group
    """
    pkg = get_package_share_directory('youbi_moveit_config')

    # 只启用 ompl 管线（默认会把 MoveIt 自带的 pilz 管线也加载，那还需要 pilz_cartesian_limits.yaml）
    moveit_config = (
        MoveItConfigsBuilder('youbi_tc7_arm_with_l20_hand', package_name='youbi_moveit_config')
        .planning_pipelines(pipelines=['ompl'])
        .to_moveit_configs()
    )

    rviz_config = os.path.join(pkg, 'rviz', 'moveit.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[moveit_config.robot_description]),

        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen'),

        Node(package='moveit_ros_move_group', executable='move_group',
             output='screen', parameters=[moveit_config.to_dict()]),

        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             arguments=['-d', rviz_config],
             parameters=[moveit_config.robot_description,
                         moveit_config.robot_description_semantic,
                         moveit_config.robot_description_kinematics,
                         moveit_config.planning_pipelines,
                         moveit_config.joint_limits]),
    ])
