"""흡착 패드를 포함한 URDF와 SRDF를 MoveIt에 함께 전달한다."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = get_package_share_directory('ur5e_pick_place')
    sim_share = get_package_share_directory('ur_simulation_gz')
    ur_moveit = get_package_share_directory('ur_moveit_config')
    description = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'ur5e_vacuum.urdf.xacro'),
        ' ur_type:=ur5e name:=ur simulation_controllers:=',
        os.path.join(sim_share, 'config', 'ur_controllers.yaml'),
    ]), value_type=str)
    semantic = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'ur5e_vacuum.srdf.xacro'),
    ]), value_type=str)
    config = (MoveItConfigsBuilder('ur', package_name='ur_moveit_config')
              .robot_description_semantic(os.path.join(share, 'urdf', 'ur5e_vacuum.srdf.xacro'))
              .to_moveit_configs())
    parameters = [config.to_dict(), {'robot_description': description,
                  'robot_description_semantic': semantic,
                  'use_sim_time': True, 'publish_robot_description_semantic': True}]
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        Node(package='moveit_ros_move_group', executable='move_group', output='log', parameters=parameters),
        Node(package='rviz2', executable='rviz2', name='rviz2_moveit',
             condition=IfCondition(LaunchConfiguration('launch_rviz')),
             arguments=['-d', os.path.join(ur_moveit, 'config', 'moveit.rviz')],
             parameters=parameters, output='log'),
    ])
