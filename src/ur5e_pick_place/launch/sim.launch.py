"""Gazebo(UR5e) + MoveIt2 + 픽앤플레이스 노드를 한 번에 띄운다 (확장 범위, docs/PROJECT.md 4장).

    ros2 launch ur5e_pick_place sim.launch.py                     # Gazebo 창 + RViz
    ros2 launch ur5e_pick_place sim.launch.py gazebo_gui:=false   # WSL2 렌더링이 느리면 RViz 만

2주차 이동 확인은 /sim/move_to_target 서비스를 사용한다 (docs/WEEK2.md).
Gazebo 구동은 설치된 ur_sim_control 런치를 쓰고, MoveIt은 패드가 포함된 로컬 moveit.launch.py를 사용한다.
전체 파이프라인의 /robot/targets 연동과 집기 사이클은 후속 주차 구현 대상이다.
"""
import math
import os
import json
from pathlib import Path
import uuid

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, SetLaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

UR_TYPE = 'ur5e'


def configure_world(context):
    """Use one absolute SDF for Gazebo and MoveIt; identify this Gazebo session."""
    world = str(Path(LaunchConfiguration('world_file').perform(context)).resolve(strict=True))
    partition = LaunchConfiguration('gz_partition').perform(context)
    if not partition:
        raise ValueError('gz_partition must not be empty')
    session = Path('logs/workcell_session.json')
    session.parent.mkdir(parents=True, exist_ok=True)
    session.write_text(json.dumps(dict(world_file=world, gz_partition=partition,
                                      ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0')), indent=2)+'\n')
    return [SetEnvironmentVariable('GZ_PARTITION', partition),
            SetLaunchConfiguration('world_file', world),
            LogInfo(msg=f'Workcell SDF: {world}; GZ_PARTITION={partition}')]


def generate_launch_description():
    share = get_package_share_directory('ur5e_pick_place')
    params_file = os.path.join(share, 'config', 'sim.yaml')
    with open(params_file) as f:
        p = yaml.safe_load(f)['pick_place_node']['ros__parameters']

    # GroupAction(scoped=True) 로 감싸야 한다. IncludeLaunchDescription 에 넘긴 인자는 바깥 설정까지 덮어써서,
    # 감싸지 않으면 아래 launch_rviz:='false' 가 새어 나와 MoveIt 쪽 RViz 도 뜨지 않는다 (2026-11-16 확인).
    ur_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_simulation_gz'), 'launch', 'ur_sim_control.launch.py')),
        launch_arguments={
            'ur_type': UR_TYPE,
            'description_file': os.path.join(share, 'urdf', 'ur5e_vacuum.urdf.xacro'),
            'world_file': LaunchConfiguration('world_file'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'launch_rviz': 'false',   # RViz 는 MoveIt 쪽에서 MotionPlanning 패널과 함께 띄운다
        }.items())

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'moveit.launch.py')),
        launch_arguments={'launch_rviz': LaunchConfiguration('launch_rviz')}.items())

    # 흡착 on/off: ROS std_msgs/Empty → Gazebo gz.msgs.Empty (']' = ROS 에서 Gazebo 한 방향)
    # 가상 카메라: Gazebo → ROS ('[' = Gazebo 에서 ROS 한 방향). workcell.sdf 의 top_camera
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[f'/vacuum/obj_{k}/{op}@std_msgs/msg/Empty]gz.msgs.Empty'
                   for k in range(p['max_objects']) for op in ('attach', 'detach')]
        + ['/sim/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
           '/sim/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
           '/sim/overview/image@sensor_msgs/msg/Image[gz.msgs.Image'])   # 시연용 전경 (GUI ① 탭)

    # 작업 시트(work_plane) 좌표계를 RViz 에서 보이게 한다. 계산은 pick_place_node 가 같은 sim.yaml 값으로 직접 한다
    work_plane_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', str(p['work_plane_x']), '--y', str(p['work_plane_y']), '--z', str(p['work_plane_z']),
                   '--yaw', str(math.radians(p['work_plane_yaw_deg'])),
                   '--frame-id', 'base_link', '--child-frame-id', 'work_plane'])

    pick_place = Node(package='ur5e_pick_place', executable='pick_place_node',
                      parameters=[params_file, {'world_file': LaunchConfiguration('world_file')}])

    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true', description='false 면 Gazebo 창 없이 서버만'),
        DeclareLaunchArgument('launch_rviz', default_value='true', description='RViz 실행 여부'),
        DeclareLaunchArgument('world_file', default_value=p.get('world_file') or os.path.join(share, 'worlds', 'workcell.sdf')),
        DeclareLaunchArgument('gz_partition', default_value=os.environ.get(
            'GZ_PARTITION') or f'vision_arm_{uuid.uuid4().hex[:12]}',
            description='Gazebo server/GUI/bridge partition; fresh by default unless GZ_PARTITION is set'),
        OpaqueFunction(function=configure_world),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(share, 'models')),
        GroupAction([ur_sim], scoped=True),
        moveit,
        bridge,
        work_plane_tf,
        pick_place,
        Node(package='ur5e_pick_place', executable='team2_workcell_visuals',
             condition=IfCondition(LaunchConfiguration('launch_rviz')),
             parameters=[{'world_file': LaunchConfiguration('world_file')}]),
    ])
