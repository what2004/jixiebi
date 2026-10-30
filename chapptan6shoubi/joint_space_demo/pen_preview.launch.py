"""查看已保存的三指握笔姿态，可开启关节滑块。"""
import json
from pathlib import Path
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def preview_nodes(context):
    package = Path(get_package_share_directory('temp_description'))
    here = Path(__file__).resolve().parent
    preset = json.loads(Path(LaunchConfiguration('pose_file').perform(context)).read_text())
    target = preset['goal']
    def pen_argument(name):
        override = LaunchConfiguration(name).perform(context).strip()
        return override or ' '.join(str(v) for v in preset[name])
    mappings = {'with_pen': 'true',
                'pen_xyz': pen_argument('pen_xyz'),
                'pen_rpy': pen_argument('pen_rpy')}
    description = xacro.process_file(str(package / 'urdf/arm_with_hand.urdf.xacro'),
                                     mappings=mappings).toxml()
    namespace = 'pen_' + uuid4().hex[:8]
    remappings = [('/tf', f'/{namespace}/tf'), ('/tf_static', f'/{namespace}/tf_static')]
    joint_parameters = [{'robot_description': description,
                         **{f'zeros.{name}': float(value) for name, value in target.items()}}]
    view = LaunchConfiguration('view').perform(context)
    rviz_config = here / 'pen_grasp_view.rviz' if view == 'hand' else package / 'rviz/arm_hand_view.rviz'
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=namespace, remappings=remappings,
             parameters=[{'robot_description': description}], output='screen'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             namespace=namespace,
             condition=UnlessCondition(LaunchConfiguration('gui')),
             parameters=joint_parameters,
             output='screen'),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             namespace=namespace,
             condition=IfCondition(LaunchConfiguration('gui')),
             parameters=joint_parameters,
             output='screen'),
        Node(package='rviz2', executable='rviz2', namespace=namespace,
             condition=IfCondition(LaunchConfiguration('use_rviz')),
             remappings=remappings + [('/robot_description', f'/{namespace}/robot_description')],
             arguments=['-d', str(rviz_config)], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pose_file', default_value=str(Path(__file__).resolve().with_name('pen_grasp.json')),
                              description='握笔预设，包含关节角和笔相对手掌的位姿'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='打开关节滑块以调整机械臂和手指角度'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('view', default_value='hand', choices=['hand', 'full'],
                              description='手部近景或完整机械臂视角'),
        DeclareLaunchArgument('pen_xyz', default_value='', description='留空时使用握笔预设的笔位置'),
        DeclareLaunchArgument('pen_rpy', default_value='', description='留空时使用握笔预设的笔朝向'),
        OpaqueFunction(function=preview_nodes),
    ])
