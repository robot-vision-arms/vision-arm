"""Gazebo(UR5e) + MoveIt2 + 픽앤플레이스 노드를 한 번에 띄운다 (확장 범위, docs/PROJECT.md 4장).

    ros2 launch ur5e_pick_place sim.launch.py                     # Gazebo 창 + RViz
    ros2 launch ur5e_pick_place sim.launch.py gazebo_gui:=false   # WSL2 렌더링이 느리면 RViz 만

비전 쪽(pipeline.launch.py)은 그대로 띄우고, robot_server 를 --publish-ros 로 실행해야 /robot/targets 가 들어온다.
ur_simulation_gz 의 ur_sim_moveit.launch.py 도 내부에서 아래 두 런치를 같이 띄우는 구조인데, world·로봇 파일을 바꿔 넣으려고
두 개를 직접 include 했다. 인자 이름은 1주차에 확인한다 (docs/SETUP.md Step 5):
    ros2 launch ur_simulation_gz ur_sim_control.launch.py --show-args
    ros2 launch ur_moveit_config ur_moveit.launch.py --show-args
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

UR_TYPE = 'ur5e'


def generate_launch_description():
    share = get_package_share_directory('ur5e_pick_place')
    params_file = os.path.join(share, 'config', 'sim.yaml')
    with open(params_file) as f:
        p = yaml.safe_load(f)['pick_place_node']['ros__parameters']

    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_simulation_gz'), 'launch', 'ur_sim_control.launch.py')),
        launch_arguments={
            'ur_type': UR_TYPE,
            'description_file': os.path.join(share, 'urdf', 'ur5e_vacuum.urdf.xacro'),
            'world_file': os.path.join(share, 'worlds', 'workcell.sdf'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'launch_rviz': 'false',   # RViz 는 MoveIt 쪽에서 MotionPlanning 패널과 함께 띄운다
        }.items())

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_moveit_config'), 'launch', 'ur_moveit.launch.py')),
        launch_arguments={'ur_type': UR_TYPE, 'use_sim_time': 'true', 'launch_rviz': 'true'}.items())

    # 흡착 on/off: ROS std_msgs/Empty → Gazebo gz.msgs.Empty (']' = ROS 에서 Gazebo 한 방향)
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[f'/vacuum/obj_{k}/{op}@std_msgs/msg/Empty]gz.msgs.Empty'
                   for k in range(p['max_objects']) for op in ('attach', 'detach')])

    # 격자 종이 좌표계를 RViz 에서 보이게 한다. 계산은 pick_place_node 가 같은 sim.yaml 값으로 직접 한다
    work_plane_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', str(p['work_plane_x']), '--y', str(p['work_plane_y']), '--z', str(p['work_plane_z']),
                   '--yaw', str(math.radians(p['work_plane_yaw_deg'])),
                   '--frame-id', 'base_link', '--child-frame-id', 'work_plane'])

    pick_place = Node(package='ur5e_pick_place', executable='pick_place_node', parameters=[params_file])

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true', description='false 면 Gazebo 창 없이 서버만'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        ur_sim,
        moveit,
        bridge,
        work_plane_tf,
        pick_place,
    ])
