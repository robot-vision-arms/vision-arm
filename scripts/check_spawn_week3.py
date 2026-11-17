"""실행 중인 프로젝트 Gazebo에서 3주차 스폰·흡착 확인 (팀원 3).

    터미널 1: ros2 launch ur5e_pick_place sim.launch.py            (gazebo_gui:=false 로 창 없이도 됨)
    터미널 2: source env.sh && source install/setup.bash && python scripts/check_spawn_week3.py
    (Gazebo partition 은 logs/workcell_session.json 에서 읽는다. 스폰 노드는 team2_link.launch.py 로 띄운 것도 그대로 쓴다)

스폰 노드(object_spawner)가 떠 있지 않으면 이 스크립트가 직접 띄우고 끝나면 끈다.
보정된 형식의 합성 패킷을 /robot/targets 로 보내므로 실제 카메라·보정 정확도 확인은 아니다.
팔을 움직이고 부품을 남긴다. 결과는 logs/week3_spawn_check.json.
스폰 노드가 이미 떠 있으면 그 노드를 쓰므로, TF 확인을 하려면 TF 기능이 들어간 최신 object_spawner 여야 한다.

확인 순서:
    1. 스폰: 나사(30°)·너트·나사(135°)·unknown → 3개가 계산 위치에, unknown 은 건너뜀
    2. calib_ok=false 패킷은 거부
    3. 팔이 부품 위로 지나가도 부품이 따라오지 않음 (스폰 직후 detach 됐는지).
       그 높이(150 mm)에서 진공을 켜면 '떨어져 있음' 으로 거부돼야 한다 (현실 조건)
    4. 너트: 큰 컵(5 mm)을 가운데(구멍 위)에 대고 진공을 켜면 '구멍 둘레 폭이 컵보다 좁음' 으로 거부, 너트는 그대로.
       이어서 링 컵(tool0 +X 12 mm)을 너트 가운데로 내려 흡착 (센터링 핀이 ±1.5 mm 어긋남까지 맞춤) → 옮겨 놓음
    5. 나사 2개: 큰 컵으로 바로 위까지 내려가 진공 ON(/sim/vacuum) → 들어 올려 옮김 → 진공 OFF →
       목표 위치에 놓임. 둥근 컵이라 툴 방향은 나사 각도와 상관없다 (135° 나사를 툴 0° 로 집음)
    6. 같은 이름으로 다시 스폰 (DetachableJoint 가 지웠다 다시 만든 모델을 다루는지), 7. 삭제 시 마커 제거
    TF:  스폰 직후 work_plane → obj_k 가 패킷의 mm 좌표·각도와 같은지, 들어 올리면 TF 도 따라가는지,
         /sim/object_markers 에 부품 마커가 나오는지 함께 확인한다
자동 집기 사이클(/sim/run_cycle)이 아니라 /sim/move_to_target 목표를 바꿔 가며 단계별로 움직인다.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.time import Time
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
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
# (슬롯, 이름, 집는 곳 mm, 툴 각도, 놓는 곳 mm, 컵 끝 하강 높이 m). 나사 윗면 8 mm → 컵 끝 8.5 mm (간격 0.5 mm)
# 팔이 멈추는 높이는 ±1~2 mm 흔들리므로 (Gazebo 창을 띄우면 더 큼), 실제 진공 그리퍼처럼 진공이 안 잡히면
# 1 mm 씩 더 내려가며 다시 켠다 (SEARCH_STEPS 번까지). 주름 컵은 3 mm 까지 눌려도 된다.
PICKS = [(0, '나사 30°', (100.0, 105.0), 30.0, (160.0, 60.0), 0.0085),
         (2, '나사 135°', (230.0, 50.0), 0.0, (60.0, 150.0), 0.0085)]
SEARCH_STEPS = 2
NUT = (1, (200.0, 150.0), 0.0065, (140.0, 150.0))   # 너트 윗면 6 mm → 컵 끝 6.5 mm, 놓는 곳
RING_OFFSET_M = 0.012              # object_spawner.SuctionLimits.ring_offset (링 컵 = tool0 +X 12 mm)


def to_base(x_mm, y_mm):
    return PLANE[0] + x_mm / 1000.0, PLANE[1] + y_mm / 1000.0


def use_session_partition():
    """sim.launch.py 는 실행마다 새 GZ_PARTITION 을 쓰고 logs/workcell_session.json 에 적는다 (팀원 2, 11-17).
    이 스크립트의 Gazebo 구독과 직접 띄우는 스폰 노드가 같은 Gazebo 에 붙도록 그 값을 쓴다."""
    session = ROOT / 'logs' / 'workcell_session.json'
    if os.environ.get('GZ_PARTITION') or not session.exists():
        return
    data = json.loads(session.read_text())
    if str(data.get('ros_domain_id', '0')) != os.environ.get('ROS_DOMAIN_ID', '0'):
        raise SystemExit(f'ROS_DOMAIN_ID 가 시뮬레이터({data.get("ros_domain_id")})와 다름')
    os.environ['GZ_PARTITION'] = data['gz_partition']
    print(f'GZ_PARTITION={data["gz_partition"]} (logs/workcell_session.json)', flush=True)


def main():
    use_session_partition()
    rclpy.init()
    node = rclpy.create_node('week3_spawn_check')
    gz_node = GzNode()
    poses = {}

    def on_poses(message):
        for pose in message.pose:
            poses[pose.name] = (pose.position.x, pose.position.y, pose.position.z)

    gz_node.subscribe(Pose_V, '/world/workcell/pose/info', on_poses)
    targets = node.create_publisher(String, '/robot/targets', 10)
    vacuum_client = node.create_client(SetBool, '/sim/vacuum')
    params = node.create_client(SetParameters, '/pick_place_node/set_parameters')
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    markers = {}

    def on_markers(message):
        for marker in message.markers:
            markers[marker.id] = marker.action

    node.create_subscription(MarkerArray, '/sim/object_markers', on_markers, 10)
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
        # 시뮬레이터를 막 띄운 직후 OMPL 이 서비스 안의 재시도(3회)까지 모두 실패하는 일이 있어 한 번 더 부른다
        if not (call('/sim/move_to_target').success or call('/sim/move_to_target').success):
            raise RuntimeError(f'이동 실패: {label}')

    def vacuum(on, expect=True, word=''):
        """진공 켜기/끄기. expect 와 결과가 다르거나 실패 이유에 word 가 없으면 멈춘다."""
        if not vacuum_client.wait_for_service(timeout_sec=10):
            raise RuntimeError('서비스 없음: /sim/vacuum (최신 object_spawner 인지 확인)')
        future = vacuum_client.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(node, future, timeout_sec=30)
        result = future.result()
        print(f'  /sim/vacuum {"ON" if on else "OFF"}: {"성공" if result.success else "실패"} {result.message}',
              flush=True)
        if (expect is not None and result.success != expect) or (word and word not in result.message):
            raise RuntimeError(f'진공 {"ON" if on else "OFF"} 결과가 예상과 다름: {result.message}')
        return result.message

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def tf_of(child, parent='work_plane'):
        """최신 TF (위치 m, yaw deg). TF 는 10 Hz 로 오므로 잠깐 받은 뒤 읽는다."""
        spin_for(1.0)
        t = tf_buffer.lookup_transform(parent, child, Time()).transform
        q = t.rotation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))
        return (t.translation.x, t.translation.y, t.translation.z), yaw

    def check_tf():
        for k, obj in enumerate(o for o in PACKET['objects'] if o['part_type'] != 'unknown'):
            (x, y, z), yaw = tf_of(f'obj_{k}')
            if math.dist((x * 1000, y * 1000), (obj['x_mm'], obj['y_mm'])) > POS_TOL_M * 1000:
                raise RuntimeError(f'TF work_plane→obj_{k} = ({x * 1000:.1f}, {y * 1000:.1f}) mm, '
                                   f'패킷 ({obj["x_mm"]}, {obj["y_mm"]})')
            if obj['angle_deg'] is not None:
                diff = abs((yaw - obj['angle_deg'] + 90) % 180 - 90)   # 장축은 180° 주기
                if diff > 1.0:
                    raise RuntimeError(f'TF obj_{k} 각도 {yaw:.1f}°, 패킷 {obj["angle_deg"]}°')
            print(f'  TF work_plane→obj_{k}: ({x * 1000:.1f}, {y * 1000:.1f}) mm, {yaw:.1f}°', flush=True)
        if any(markers.get(k) != Marker.ADD for k in range(3)):
            raise RuntimeError(f'/sim/object_markers 에 부품 마커 3개가 없음: {markers}')

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
        check_tf()
        report['tf_matches_packet'] = True

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
        report['far_vacuum'] = vacuum(True, expect=False, word='떨어져')

        def pick(k, name, x, y, angle, to_x, to_y, down, shift=(0.0, 0.0), cup='5 mm 컵'):
            """부품 중심 (x, y) mm 를 집어 (to_x, to_y) 에 놓는다. shift = 툴 목표를 부품 중심에서 옮기는 양 (mm)."""
            sx, sy = shift
            move(x + sx, y + sy, angle, 0.08, '접근')
            move(x + sx, y + sy, angle, down, '바로 위로 하강')
            for step in range(SEARCH_STEPS + 1):
                message = vacuum(True, expect=None)
                if message.startswith('진공 ON') or step == SEARCH_STEPS or '떨어져' not in message:
                    break
                move(x + sx, y + sy, angle, down - 0.001 * (step + 1), f'진공 안 잡힘 → {step + 1} mm 더 내려감')
            if f'obj_{k}' not in message or not message.startswith('진공 ON') or cup not in message:
                raise RuntimeError(f'{name} 흡착 실패 ({cup} 기대): {message}')
            report.setdefault('vacuum_on', []).append(message)
            time.sleep(0.5)
            move(x + sx, y + sy, angle, 0.10, '들어 올림')
            lifted = objects()[f'obj_{k}'][2]
            if lifted < 0.07:
                raise RuntimeError(f'{name} 이 따라 올라오지 않음 (z={lifted:.3f} m)')
            tf_z = tf_of(f'obj_{k}', 'base_link')[0][2]
            if abs(tf_z - lifted) > POS_TOL_M:
                raise RuntimeError(f'{name} TF 높이 {tf_z:.3f} m 가 Gazebo {lifted:.3f} m 와 다름')
            move(to_x + sx, to_y + sy, angle, 0.10, '옆으로 이동')
            move(to_x + sx, to_y + sy, angle, down + 0.01, '내려놓을 자리로 하강')
            vacuum(False, word=f'obj_{k}')
            time.sleep(1.0)
            placed = objects()[f'obj_{k}']
            goal = to_base(to_x, to_y)
            error = math.dist(placed[:2], goal)
            move(to_x + sx, to_y + sy, angle, 0.10, '위로 빠짐')
            report['picks'].append({'obj': f'obj_{k}', 'part': name, 'cup': cup, 'lifted_z_m': round(lifted, 4),
                                    'lifted_tf_z_m': round(tf_z, 4),
                                    'placed': [round(v, 4) for v in placed], 'goal': [round(v, 4) for v in goal],
                                    'place_error_m': round(error, 4)})
            print(f'  {name}: {cup}으로 들어 올림 {lifted * 1000:.0f} mm, 놓인 위치 오차 {error * 1000:.1f} mm',
                  flush=True)
            if error > POS_TOL_M or abs(placed[2]) > POS_TOL_M:
                raise RuntimeError(f'{name} 이 목표 위치에 놓이지 않음 {placed}')

        report['picks'] = []
        k, (x, y), down, (to_x, to_y) = NUT
        print('[4] 너트: 5 mm 컵은 구멍 때문에 거부, 링 컵을 가운데로 내려 흡착', flush=True)
        nut_before = objects()[f'obj_{k}']
        move(x, y, 0.0, 0.08, '너트 가운데(구멍 위) 접근')
        move(x, y, 0.0, down, '5 mm 컵을 너트 가운데로 하강')
        report['nut_big_cup'] = vacuum(True, expect=False, word='폭')
        move(x, y, 0.0, 0.08, '위로')
        if math.dist(objects()[f'obj_{k}'], nut_before) > POS_TOL_M:
            raise RuntimeError('너트가 움직임 (붙으면 안 되는데 붙었거나 밀림)')
        # 링 컵 중심이 너트 중심에 오도록 툴은 너트 중심에서 툴 -X 로 12 mm
        yaw = math.radians(tf_of('tool0', 'base_link')[1])
        back = RING_OFFSET_M * 1000
        pick(k, '너트', x, y, 0.0, to_x, to_y, down, (-back * math.cos(yaw), -back * math.sin(yaw)), '링 컵')

        for k, name, (x, y), angle, (to_x, to_y), down in PICKS:
            print(f'[5] {name} 집어 옮기기 ({x:.0f}, {y:.0f}) → ({to_x:.0f}, {to_y:.0f}) mm, 툴 {angle:.0f}°',
                  flush=True)
            pick(k, name, x, y, angle, to_x, to_y, down)

        print('[6] 같은 이름으로 다시 스폰', flush=True)
        if not call('/sim/spawn_objects').success:
            raise RuntimeError('재스폰 실패')
        check_spawned('재스폰')
        report['respawned'] = True
        print('[7] 삭제하면 마커도 사라짐', flush=True)
        call('/sim/clear_objects')
        markers.clear()
        spin_for(1.5)
        if any(markers.get(k) != Marker.DELETE for k in range(3)):
            raise RuntimeError(f'삭제 후에도 마커가 남음: {markers}')
        report['markers_deleted'] = True
        call('/sim/spawn_objects')   # 다음 확인을 위해 부품은 다시 남겨 둔다
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
    code = main()
    sys.stdout.flush()
    # gz-transport·tf 리스너의 백그라운드 스레드가 파이썬 종료 정리 중에 abort 를 일으켜(core dump)
    # 통과해도 종료 코드가 134 가 되므로, 정리를 건너뛰고 바로 끝낸다. 결과 파일은 이미 저장됨
    os._exit(code)
