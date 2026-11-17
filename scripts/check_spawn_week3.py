"""실행 중인 프로젝트 Gazebo에서 3주차 스폰·흡착 확인 (팀원 3).

    터미널 1: ros2 launch ur5e_pick_place sim.launch.py            (gazebo_gui:=false 로 창 없이도 됨)
    터미널 2: source env.sh && source install/setup.bash && python scripts/check_spawn_week3.py

스폰 노드(object_spawner)가 떠 있지 않으면 이 스크립트가 직접 띄우고 끝나면 끈다.
보정된 형식의 합성 패킷을 /robot/targets 로 보내므로 실제 카메라·보정 정확도 확인은 아니다.
팔을 움직이고 부품을 남긴다. 결과는 logs/week3_spawn_check.json.

확인 순서:
    1. 스폰: 나사(30°)·너트·나사(135°)·unknown → 3개가 계산 위치에, unknown 은 건너뜀
    2. calib_ok=false 패킷은 거부
    3. 팔이 부품 위로 지나가도 부품이 따라오지 않음 (스폰 직후 detach 됐는지)
    4. 나사·너트를 바로 위까지 내려가 attach → 들어 올려 옮김 → detach → 목표 위치에 놓임
    5. 같은 이름으로 다시 스폰 (DetachableJoint 가 지웠다 다시 만든 모델을 다루는지)
자동 집기 사이클(/sim/run_cycle)이 아니라 /sim/move_to_target 목표를 바꿔 가며 단계별로 움직인다.
"""
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

ROOT = Path(__file__).resolve().parents[1]
PLANE = (0.30, -0.15)          # config/sim.yaml work_plane_x, work_plane_y (yaw 0)
POS_TOL_M = 0.005              # 놓인 위치 허용 오차
PACKET = {'seq': 1, 't_capture': 0.0, 'calib_ok': True, 'objects': [
    {'id': 0, 'part_type': 'screw', 'x_mm': 100.0, 'y_mm': 105.0, 'angle_deg': 30.0},
    {'id': 1, 'part_type': 'nut', 'x_mm': 200.0, 'y_mm': 150.0, 'angle_deg': None},
    {'id': 2, 'part_type': 'screw', 'x_mm': 230.0, 'y_mm': 50.0, 'angle_deg': 135.0},
    {'id': 3, 'part_type': 'unknown', 'x_mm': 20.0, 'y_mm': 20.0, 'angle_deg': None}]}
# (슬롯, 이름, 집는 곳 mm, 각도, 놓는 곳 mm, 패드 끝 하강 높이 m). 나사 높이 8 mm, 너트 6 mm
PICKS = [(0, '나사', (100.0, 105.0), 30.0, (160.0, 60.0), 0.012),
         (1, '너트', (200.0, 150.0), 0.0, (150.0, 160.0), 0.010)]


def to_base(x_mm, y_mm):
    return PLANE[0] + x_mm / 1000.0, PLANE[1] + y_mm / 1000.0


def main():
    rclpy.init()
    node = rclpy.create_node('week3_spawn_check')
    gz_node = GzNode()
    poses = {}

    def on_poses(message):
        for pose in message.pose:
            poses[pose.name] = (pose.position.x, pose.position.y, pose.position.z)

    gz_node.subscribe(Pose_V, '/world/workcell/pose/info', on_poses)
    targets = node.create_publisher(String, '/robot/targets', 10)
    attach = [node.create_publisher(Empty, f'/vacuum/obj_{k}/attach', 10) for k in range(3)]
    detach = [node.create_publisher(Empty, f'/vacuum/obj_{k}/detach', 10) for k in range(3)]
    params = node.create_client(SetParameters, '/pick_place_node/set_parameters')
    report = {}
    spawner = None

    def call(name, timeout=120):
        client = node.create_client(Trigger, name)
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError(f'서비스 없음: {name}')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f'서비스 응답 없음: {name}')
        result = future.result()
        print(f'  {name}: {"성공" if result.success else "실패"} {result.message}', flush=True)
        return result

    def publish(publisher, message, times=5):
        for _ in range(times):
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)

    def objects():
        time.sleep(1.0)
        poses.clear()
        time.sleep(0.5)
        return {k: v for k, v in poses.items() if k.startswith('obj_')}

    def move(x_mm, y_mm, angle, height, label):
        if not params.wait_for_service(timeout_sec=10):
            raise RuntimeError('pick_place_node 파라미터 서비스 없음')
        request = SetParameters.Request(parameters=[
            Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
            for name, value in dict(target_x_mm=x_mm, target_y_mm=y_mm, target_angle_deg=angle,
                                    target_height_m=height).items()])
        future = params.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        print(f'  팔: {label} (패드 끝 {height * 1000:.0f} mm)', flush=True)
        if not call('/sim/move_to_target').success:
            raise RuntimeError(f'이동 실패: {label}')

    def check_spawned(where):
        placed = objects()
        for k, obj in enumerate(o for o in PACKET['objects'] if o['part_type'] != 'unknown'):
            x, y = to_base(obj['x_mm'], obj['y_mm'])
            got = placed.get(f'obj_{k}')
            if got is None or math.dist(got[:2], (x, y)) > POS_TOL_M or abs(got[2]) > POS_TOL_M:
                raise RuntimeError(f'{where}: obj_{k} 위치 {got}, 기대 ({x:.3f}, {y:.3f}, 0)')
        return placed

    try:
        if not node.create_client(Trigger, '/sim/spawn_objects').wait_for_service(timeout_sec=2):
            print('스폰 노드를 띄웁니다', flush=True)
            spawner = subprocess.Popen([sys.executable, '-m', 'ur5e_pick_place.object_spawner'])
            time.sleep(3)

        print('[1] 스폰', flush=True)
        publish(targets, String(data=json.dumps(PACKET)))
        result = call('/sim/spawn_objects')
        if not result.success or 'unknown' not in result.message:
            raise RuntimeError('스폰 실패 또는 unknown 을 건너뛰지 않음')
        before = check_spawned('스폰')
        report['spawned'] = {k: [round(v, 4) for v in p] for k, p in sorted(before.items())}

        print('[2] calib_ok=false 거부', flush=True)
        publish(targets, String(data=json.dumps({**PACKET, 'seq': 2, 'calib_ok': False})))
        if call('/sim/spawn_objects').success:
            raise RuntimeError('미보정 패킷을 거부하지 않음')
        report['uncalibrated_rejected'] = True
        publish(targets, String(data=json.dumps(PACKET)))

        print('[3] 팔이 지나가도 부품은 그대로', flush=True)
        move(100.0, 105.0, 0.0, 0.15, '부품 위 150 mm')
        after = objects()
        moved = max(math.dist(before[k], after[k]) for k in before)
        report['moved_by_arm_m'] = round(moved, 4)
        if moved > POS_TOL_M:
            raise RuntimeError(f'팔을 따라 부품이 움직임 {moved:.3f} m (detach 안 됨)')

        report['picks'] = []
        for k, name, (x, y), angle, (to_x, to_y), down in PICKS:
            print(f'[4] {name} 집어 옮기기 ({x:.0f}, {y:.0f}) → ({to_x:.0f}, {to_y:.0f}) mm', flush=True)
            move(x, y, angle, 0.08, '접근')
            move(x, y, angle, down, '바로 위로 하강')
            publish(attach[k], Empty(), 3)
            time.sleep(1.0)
            move(x, y, angle, 0.10, '들어 올림')
            lifted = objects()[f'obj_{k}'][2]
            if lifted < 0.07:
                raise RuntimeError(f'{name} 이 따라 올라오지 않음 (z={lifted:.3f} m)')
            move(to_x, to_y, angle, 0.10, '옆으로 이동')
            move(to_x, to_y, angle, down + 0.01, '내려놓을 자리로 하강')
            publish(detach[k], Empty(), 3)
            time.sleep(1.0)
            placed = objects()[f'obj_{k}']
            goal = to_base(to_x, to_y)
            error = math.dist(placed[:2], goal)
            move(to_x, to_y, angle, 0.10, '위로 빠짐')
            report['picks'].append({'obj': f'obj_{k}', 'part': name, 'lifted_z_m': round(lifted, 4),
                                    'placed': [round(v, 4) for v in placed], 'goal': [round(v, 4) for v in goal],
                                    'place_error_m': round(error, 4)})
            print(f'  {name}: 들어 올림 {lifted * 1000:.0f} mm, 놓인 위치 오차 {error * 1000:.1f} mm', flush=True)
            if error > POS_TOL_M or abs(placed[2]) > POS_TOL_M:
                raise RuntimeError(f'{name} 이 목표 위치에 놓이지 않음 {placed}')

        print('[5] 같은 이름으로 다시 스폰', flush=True)
        if not call('/sim/spawn_objects').success:
            raise RuntimeError('재스폰 실패')
        check_spawned('재스폰')
        report['respawned'] = True
        report['passed'] = True
        print('통과', flush=True)
    except Exception as error:  # 결과 파일에 실패 이유를 남긴다
        report['passed'] = False
        report['error'] = str(error)
        print(f'실패: {error}', flush=True)
    finally:
        out = ROOT / 'logs' / 'week3_spawn_check.json'
        out.parent.mkdir(exist_ok=True)
        out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'결과: {out}', flush=True)
        if spawner is not None:
            spawner.send_signal(signal.SIGINT)
            spawner.wait(timeout=10)
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if report.get('passed') else 1


if __name__ == '__main__':
    sys.exit(main())
