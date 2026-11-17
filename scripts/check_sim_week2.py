"""실행 중인 프로젝트 Gazebo에서 2주차 이동·흡착 메커니즘 확인.

source env.sh 후 python scripts/check_sim_week2.py [--session logs/workcell_session.json]
obj_0이 없는 새 시뮬레이터에서 실행. 이 스크립트는 부품을 스폰하고 팔을 움직인다.
Gazebo partition 은 sim.launch.py 가 남긴 logs/workcell_session.json 에서 읽는다 (GZ_PARTITION 을 직접 export 할 필요 없음).
고정 관절의 연결·추종·해제 확인용이며 접촉 위치를 맞추는 집기 사이클은 4주차 대상이다.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Empty
from std_srvs.srv import Trigger
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node as GzNode


def use_session(path: str) -> None:
    """sim.launch.py 가 쓴 세션 파일의 GZ_PARTITION 을 이 프로세스(와 ros_gz_sim create)에 적용한다.

    gz-transport 는 Node 를 만들 때 GZ_PARTITION 을 읽으므로 GzNode() 보다 먼저 불러야 한다.
    """
    session_path = Path(path)
    if not session_path.exists():
        if 'GZ_PARTITION' not in os.environ:
            raise SystemExit(f'{session_path} 가 없습니다. 먼저 ros2 launch ur5e_pick_place sim.launch.py 로 시뮬레이터를 띄우세요.')
        return
    session = json.loads(session_path.read_text())
    os.environ['GZ_PARTITION'] = session['gz_partition']
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if str(session.get('ros_domain_id', domain)) != domain:
        raise SystemExit(f"ROS_DOMAIN_ID 가 다릅니다: 시뮬레이터 {session['ros_domain_id']}, 이 터미널 {domain}")
    print(f"GZ_PARTITION={session['gz_partition']} ({session_path})", flush=True)


def main():
    parser = argparse.ArgumentParser(description='2주차 이동·흡착 확인')
    parser.add_argument('--session', default='logs/workcell_session.json')
    use_session(parser.parse_args().session)
    rclpy.init()
    node = rclpy.create_node('week2_sim_check')
    gz_node = GzNode()
    poses, states = {}, []

    def on_poses(message):
        for pose in message.pose:
            poses[pose.name] = (pose.position.x, pose.position.y, pose.position.z)

    gz_node.subscribe(Pose_V, '/world/workcell/pose/info', on_poses)
    gz_node.subscribe(StringMsg, '/vacuum/obj_0/state', lambda message: states.append(message.data))
    attach = node.create_publisher(Empty, '/vacuum/obj_0/attach', 10)
    detach = node.create_publisher(Empty, '/vacuum/obj_0/detach', 10)
    move = node.create_client(Trigger, '/sim/move_to_target')
    params = node.create_client(SetParameters, '/pick_place_node/set_parameters')
    report = {}

    def wait_for(condition, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return
            rclpy.spin_once(node, timeout_sec=0.05)
        raise RuntimeError('상태 대기 시간 초과')

    def call(client, request, timeout=45):
        if not client.wait_for_service(timeout_sec=8):
            raise RuntimeError(f'서비스 없음: {client.srv_name}')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('서비스 응답 없음')
        return future.result()

    def set_target(height):
        request = SetParameters.Request(parameters=[
            Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value))
            for name, value in dict(target_x_mm=100., target_y_mm=100.,
                                    target_angle_deg=0., target_height_m=height).items()])
        response = call(params, request)
        if not all(result.successful for result in response.results):
            raise RuntimeError(str(response))

    def move_once():
        response = call(move, Trigger.Request())
        print(response, flush=True)
        if not response.success:
            raise RuntimeError(response.message)

    def command(publisher, expected):
        before = len(states)
        for _ in range(3):
            publisher.publish(Empty())
            rclpy.spin_once(node, timeout_sec=0.1)
        wait_for(lambda: expected in states[before:])

    try:
        # 이미 obj_0 이 있으면 스폰이 실패하거나 예전 상태가 섞인다 → 처음부터 분명하게 알린다
        deadline = time.monotonic() + 3
        while not poses and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not poses:
            raise RuntimeError('Gazebo 자세 정보를 못 받았습니다. 시뮬레이터가 떠 있는지, GZ_PARTITION 이 맞는지 확인하세요.')
        if 'obj_0' in poses:
            raise RuntimeError('obj_0 이 이미 있습니다. 이 확인은 obj_0 이 한 번도 없던 새 시뮬레이터에서 실행하세요.')
        set_target(0.25)
        move_once()
        report['designated_move'] = True
        model = Path(__file__).resolve().parents[1] / 'src/ur5e_pick_place/models/screw/model.sdf'
        subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'workcell',
                        '-file', str(model), '-name', 'obj_0', '-x', '0.4', '-y', '-0.05', '-z', '0.24'],
                       check=True, timeout=15)
        wait_for(lambda: 'attached' in states and 'obj_0' in poses)
        report['late_spawn_attached'] = True
        command(detach, 'detached')
        wait_for(lambda: poses['obj_0'][2] < 0.02)
        report['first_detach_fell'] = True
        before = poses['obj_0']
        command(attach, 'attached')
        set_target(0.35)
        move_once()
        wait_for(lambda: poses['obj_0'][2] > before[2] + 0.07)
        report['attached_lift_m'] = poses['obj_0'][2] - before[2]
        command(detach, 'detached')
        wait_for(lambda: poses['obj_0'][2] < 0.02)
        report['final_detach_fell'] = True
        report['states'] = states
        output = Path('logs/week2_sim_check.json')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        detach.publish(Empty())
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
