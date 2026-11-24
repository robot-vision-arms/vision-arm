"""Team 2 TCP → existing team 3 spawner. Start sim.launch.py first.

Does not edit/start the camera or GUI, does not trigger spawning or robot motion.
"""
import os
from pathlib import Path
import sys
import socket

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from vision_pick_place.team2_session import load_session, check_gazebo, check_ros_link


def start_link(context):
    import yaml
    port = int(LaunchConfiguration('port').perform(context))
    if not 1 <= port <= 65535:
        raise ValueError('port must be in 1..65535')
    session_file = LaunchConfiguration('session_file').perform(context)
    config = str(Path(LaunchConfiguration('sim_params').perform(context)).resolve(strict=True))
    session = load_session(session_file, os.environ.get('ROS_DOMAIN_ID', '0'))
    params = yaml.safe_load(Path(config).read_text())['pick_place_node']['ros__parameters']
    if params['world_name'] != session['world_name']:
        raise ValueError('Spawner config world_name differs from running simulation')
    env = dict(os.environ, GZ_PARTITION=session['gz_partition'])
    # ROS 도메인만 같아도 Gazebo는 다른 서버일 수 있다. 기록된 partition의 실제 응답까지 확인한다.
    check_gazebo(session, env)
    check_ros_link(params)
    # 다른 ROS 도메인의 서버도 같은 TCP 포트를 쓸 수 있다. 기존 서버에 잘못 접속하기 전에 거부한다.
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            probe.bind(('127.0.0.1', port))
        except OSError as exc:
            raise RuntimeError(f'TCP port {port} is unavailable. Use a free port or stop its existing launch.') from exc
    asset_prefix = LaunchConfiguration('asset_prefix').perform(context)
    configured_models=params.get('team2_models_dir','')
    if configured_models and not asset_prefix:
        models=Path(configured_models).resolve(strict=True)
        if models.parts[-3:] != ('share','ur5e_pick_place','models'):
            raise ValueError('team2_models_dir must belong to a generated team2 asset prefix')
        # 프로필만 지정한 경우도 스폰과 집기 판정이 같은 모델을 읽게 한다.
        asset_prefix=str(models.parents[2])
    spawner_env = {}
    if asset_prefix:
        asset_prefix = str(Path(asset_prefix).resolve(strict=True))
        marker = Path(asset_prefix)/'share/ament_index/resource_index/packages/ur5e_pick_place'
        if not marker.is_file():
            raise ValueError('Invalid team2 asset overlay prefix')
        if configured_models and Path(configured_models).resolve() != Path(asset_prefix)/'share/ur5e_pick_place/models':
            raise ValueError('Spawner asset_prefix and pick node team2_models_dir differ')
        # 모델 탐색 경로만 스폰 프로세스에 주입한다. 팀원 3 원본 파일·알고리즘은 바꾸지 않는다.
        spawner_env['AMENT_PREFIX_PATH'] = asset_prefix + os.pathsep + os.environ.get('AMENT_PREFIX_PATH', '')
    share = get_package_share_directory('vision_pick_place')
    return [LogInfo(msg=f"Team 2 link: domain={session.get('ros_domain_id', '0')}, "
                    f"GZ_PARTITION={session['gz_partition']}; spawning remains service-triggered"),
        GroupAction([
            SetEnvironmentVariable('GZ_PARTITION', session['gz_partition']),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'tcp.launch.py')),
                launch_arguments={'port': LaunchConfiguration('port'),
                                  'log_dir': LaunchConfiguration('log_dir')}.items()),
            ExecuteProcess(cmd=[sys.executable, '-m', 'ur5e_pick_place.team2_spawner_adapter',
                '--ros-args', '-p', f'config_file:={config}'], output='screen', additional_env=spawner_env),
            # 최신 팀원 3 노드가 /sim/vacuum도 제공한다. 중복 노드나 별도 attach 우회 경로를 만들지 않는다.
        ], scoped=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('session_file', default_value='logs/workcell_session.json'),
        DeclareLaunchArgument('sim_params', default_value=os.path.join(
            get_package_share_directory('ur5e_pick_place'), 'config', 'sim.yaml')),
        DeclareLaunchArgument('port', default_value='9000'),
        DeclareLaunchArgument('asset_prefix', default_value=''),
        DeclareLaunchArgument('log_dir', default_value='logs'),
        OpaqueFunction(function=start_link),
    ])
