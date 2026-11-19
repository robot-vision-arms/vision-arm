"""Team 2 adapter for linking TCP to another team's unmodified spawner."""
import json
import math
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


# 이 값이 다르면 ROS 타깃은 같아도 로봇 TF와 Gazebo 부품 위치·흡착 컵 높이가 달라진다.
LINK_PARAMETERS = ('world_name', 'work_plane_x', 'work_plane_y', 'work_plane_z',
                   'work_plane_yaw_deg', 'tool_length_m', 'max_objects', 'part_types', 'use_sim_time')


def validate_link_parameters(config, running):
    """다른 담당의 원본은 유지하고 연결 경계에서 설정 불일치를 거부한다."""
    for name in LINK_PARAMETERS:
        if name not in config or name not in running:
            raise ValueError(f'Missing shared parameter: {name}')
        expected, actual = config[name], running[name]
        matches = (math.isfinite(actual) and math.isclose(expected, actual, rel_tol=0, abs_tol=1e-9)
                   if type(expected) is float and type(actual) in (float, int)
                   else expected == actual)
        if not matches:
            raise ValueError(f'Spawner/robot parameter mismatch: {name}: config={expected!r}, running={actual!r}. '
                             'Use the same sim_params as the running simulation.')


def check_ros_link(config):
    """실행 중인 로봇 설정을 조회하고, 기존 TCP·스폰 스택과 중복되지 않는지 확인한다."""
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import parameter_value_to_python
    from rcl_interfaces.srv import GetParameters
    from std_srvs.srv import Trigger
    context = Context()
    rclpy.init(args=[], context=context)
    node = rclpy.create_node(f'team2_link_probe_{os.getpid()}', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        client = node.create_client(GetParameters, '/pick_place_node/get_parameters')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Robot parameter service is missing. Start sim.launch.py first in the same ROS domain.')
        future = client.call_async(GetParameters.Request(names=list(LINK_PARAMETERS)))
        executor.spin_until_future_complete(future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError('Robot parameter query timed out; no link nodes were started.')
        running = {name: parameter_value_to_python(value)
                   for name, value in zip(LINK_PARAMETERS, future.result().values)}
        validate_link_parameters(config, running)
        # 새 검사 노드는 로봇 서비스보다 스폰 서비스를 늦게 발견할 수 있다. 한 번의 그래프 조회로 판단하지 않는다.
        spawn_client = node.create_client(Trigger, '/sim/spawn_objects')
        spawn_running = spawn_client.wait_for_service(timeout_sec=2)
        # 이름을 바꾼 스폰 노드도 서비스 이름으로 찾는다. 기존 노드는 종료하거나 수정하지 않는다.
        duplicates = sorted({name for name, _ in node.get_node_names_and_namespaces()
                             if name in ('robot_server', 'robot_bridge', 'object_spawner')})
        # 서비스 이름 목록에는 검사 클라이언트만 만든 이름도 나타난다. 실제 서버 발견 여부로 판단한다.
        if spawn_running or duplicates:
            raise RuntimeError(f'Link nodes/services already running ({duplicates}). '
                               'Use the existing link or stop its launch before starting another.')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown(context=context)


def load_session(path, ros_domain_id='0'):
    # 팀원 1의 TF/이동과 팀원 3의 스폰이 동일한 월드·ROS 도메인을 사용하도록 연결 경계를 검사한다.
    try:
        data = json.loads(Path(path).read_text())
    except FileNotFoundError as exc:
        raise ValueError('Gazebo session file is missing. Start project sim.launch.py first.') from exc
    partition = data.get('gz_partition')
    if not isinstance(partition, str) or not partition:
        raise ValueError('Missing Gazebo partition. Start project sim.launch.py first.')
    if str(data.get('ros_domain_id', '0')) != str(ros_domain_id):
        raise ValueError(f"ROS_DOMAIN_ID mismatch: simulator={data.get('ros_domain_id')}, terminal={ros_domain_id}. "
                         'Use the same domain in both terminals.')
    world_path = Path(data['world_file']).resolve(strict=True)
    world = ET.parse(world_path).getroot().find('world')
    if world is None:
        raise ValueError('Session SDF has no world')
    return dict(data, world_file=str(world_path), world_name=world.attrib['name'])


def check_gazebo(session, env):
    result = subprocess.run(['gz', 'service', '-s', f"/world/{session['world_name']}/scene/info",
        '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene', '--timeout', '4000', '--req', ''],
        capture_output=True, text=True, timeout=7, env=env)
    if result.returncode or not result.stdout.strip():
        raise RuntimeError('Recorded Gazebo session is not responding. Restart sim.launch.py; '
                           'a remaining RViz window does not prove Gazebo is running.')
    # CLI가 출력한 오류 문구를 정상 장면 응답으로 오인하지 않도록 protobuf도 검사한다.
    from google.protobuf import text_format
    from gz.msgs10.scene_pb2 import Scene
    try:
        scene = text_format.Parse(result.stdout, Scene())
    except text_format.ParseError as exc:
        raise RuntimeError('Gazebo returned an invalid scene response; restart sim.launch.py.') from exc
    # Harmonic의 정상 scene/info 응답은 월드 name을 생략할 수 있으므로 장면 내용도 허용한다.
    if not scene.name and not scene.model and not scene.light:
        raise RuntimeError('Gazebo returned an empty scene response; restart sim.launch.py.')
    if scene.name and scene.name != session['world_name']:
        raise RuntimeError(f'Gazebo world mismatch: expected {session["world_name"]}, got {scene.name!r}')
