"""팀원 2 전용 RViz: 공유 MoveIt launch와 배포판 RViz는 변경하지 않는다."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = get_package_share_directory('ur5e_pick_place')
    ur_sim = get_package_share_directory('ur_simulation_gz')
    semantic = os.path.join(share, 'urdf', 'ur5e_vacuum.srdf.xacro')
    config = (MoveItConfigsBuilder('ur', package_name='ur_moveit_config')
              .robot_description_semantic(semantic).to_moveit_configs())
    # 로봇 형상·운동학은 기존 설정과 동일하게 전달하고 종료 순서만 전용 실행기로 바꾼다.
    params = [config.to_dict(), {
        'robot_description': ParameterValue(Command([
            'xacro ', LaunchConfiguration('description_file'),
            ' ur_type:=ur5e name:=ur simulation_controllers:=',
            os.path.join(ur_sim, 'config', 'ur_controllers.yaml')]), value_type=str),
        'robot_description_semantic': ParameterValue(Command(['xacro ', semantic]), value_type=str),
        'use_sim_time': True}]
    return LaunchDescription([
        DeclareLaunchArgument('description_file'),
        Node(package='team2_move_group', executable='team2_rviz', name='rviz2_moveit',
             arguments=['-d', os.path.join(share, 'config', 'team2_view.rviz')],
             parameters=params, output='log')])
