"""Team 2 existing-object integration, reusing the unchanged motion implementation.

existing: bind camera detections to already spawned models; never respawn.
respawn: explicitly use the original photo-to-simulation replication cycle.
"""
from dataclasses import replace
import math
import json
import time
import signal

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.time import Time
from tf2_ros import TransformException, TransformListener
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.qos import QoSProfile, DurabilityPolicy

from ur5e_pick_place.object_spawner import marker_shape, model_file, plan_spawns, rotate
from ur5e_pick_place.workcell_scene import collision_boxes
from ur5e_pick_place.pick_place_node import PickPlaceNode, UR_JOINTS


def bind_existing_targets(targets, observed, tolerance=0.005):
    """Match by position first; reject wrong classification instead of replacing models."""
    bound, used = [], set()
    for target in targets:
        # 검출 id는 화면의 나열 순서이므로 Gazebo의 obj_k 번호와 같다고 가정하지 않는다.
        # 단위는 m이며, 5 mm 안에 후보가 하나뿐일 때만 기존 물체로 대응시킨다.
        candidates = [(name, obj) for name, obj in observed.items()
                      if math.hypot(target.x-obj['x'], target.y-obj['y']) <= tolerance]
        if len(candidates) != 1:
            raise ValueError(f'기존 물체 대응 실패: id={target.source_id}, 후보 {len(candidates)}개')
        name, obj = candidates[0]
        if name in used:
            raise ValueError(f'하나의 물체에 검출이 중복 대응됨: {name}')
        if obj['part_type'] != target.part_type:
            # 분류가 틀렸다고 모델을 교체하면 오류가 가려지므로 실제 집기 전에 거부한다.
            raise ValueError(f'기존 물체 종류 불일치: {name}, 검출={target.part_type}, 모델={obj["part_type"]}')
        used.add(name)
        # 검출 좌표는 유지하고, 흡착·TF 조회에 쓸 이름만 실제 Gazebo 이름으로 바꾼다.
        bound.append(replace(target, name=name))
    return bound


def ready_reached(actual, expected, tolerance=0.05):
    # 투입 성공과 복귀 성공을 구분한다. 6개 관절 모두 0.05 rad 이내여야 복귀로 본다.
    return (len(actual) == len(expected) == 6
            and all(math.isfinite(a) and abs(a-b) < tolerance for a,b in zip(actual, expected)))


def cylinder_extents(axis, radius, half_length):
    """원기둥 자세를 반영한 월드 XYZ 반폭. 중심만 보고 상자 안이라고 판정하지 않는다."""
    return [abs(v)*half_length + radius*math.sqrt(max(0.,1-v*v)) for v in axis]


class Team2PickPlaceNode(PickPlaceNode):
    def __init__(self):
        super().__init__()
        # 원본이 시작한 TF 전용 실행기를 끝내고 메인 4스레드 실행기에만 노드를 둔다.
        # 같은 action client를 두 실행기에서 기다리면 wait-set 인덱스 예외가 날 수 있다.
        previous_listener = self.tf_listener
        previous_listener.executor.shutdown()
        previous_listener.dedicated_listener_thread.join()
        del self.tf_listener
        del previous_listener
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.declare_parameter('cycle_scene_mode', 'existing')
        self.declare_parameter('target_max_age_s', 3.0)
        self.declare_parameter('bin_inward_drop_m', 0.035)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        self.target_received = None
        self.marker_received = None
        self.object_types = {}
        self._cycle_routing = False
        self._compact_return_active = False
        self.declare_parameter('compact_cycle_return', False)
        self.declare_parameter('team2_acceleration_scaling', 0.5)
        self._scene_counter = 0
        self.declare_parameter('scene_seed', -1)
        self.declare_parameter('scene_position_jitter_mm', 0.0)
        self.declare_parameter('team2_world_bin_check', False)
        self.scene_pub = None
        self.scene_ack = None
        self.scene_spawn_client = self.create_client(Trigger, '/team2/spawn_exact', callback_group=self.cb_group)
        self.scene_ack_sub = self.create_subscription(String, '/team2/scene_request_ack',
            self.on_scene_request_ack, 10, callback_group=ReentrantCallbackGroup())
        self.truth_pub = self.create_publisher(String, '/team2/scene_truth',
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scene_srv = self.create_service(Trigger, '/team2/spawn_scene', self.on_spawn_scene,
                                             callback_group=self.cb_group)
        self.shapes = {p: marker_shape(model_file(self.models_dir, p))
                       for p in self.get_parameter('part_types').value}
        # 집기 서비스가 이동 응답을 기다리는 동안에도 물체 마커 수신을 계속 처리한다.
        self.markers_sub = self.create_subscription(
            MarkerArray, '/sim/object_markers', self.on_markers, 10,
            callback_group=ReentrantCallbackGroup())

    def prepare_motion(self):
        acceleration = self.value('team2_acceleration_scaling')
        if not math.isfinite(acceleration) or not 0 < acceleration <= 1:
            raise ValueError('team2_acceleration_scaling은 (0, 1]이어야 합니다.')
        super().prepare_motion()
        # pymoveit2 기본값 0은 MoveIt에서 1로 대체된다. 실제 요청에 유효한 배율을 명시한다.
        self.moveit2.max_acceleration = acceleration

    def on_targets(self, msg):
        previous = self.latest
        super().on_targets(msg)
        if self.latest is not previous:
            # 원본 노드가 이동 중 입력을 무시한 경우에는 수신 시각도 갱신하지 않는다.
            self.target_received = time.monotonic()

    def world_bin_contains(self, target, bin_xy):
        boxes=collision_boxes(self.get_parameter('world_file').value)
        floors=[b for b in boxes if b.id==target.part_type+'_bin/floor']
        if len(floors)!=1:raise ValueError('통 바닥 충돌 모델을 찾을 수 없습니다.')
        floor=floors[0]
        walls=[b for b in boxes if b.id.startswith(target.part_type+'_bin/wall')]
        if not walls or any(abs(v)>1e-6 for v in floor.quat_xyzw[:3]):
            raise ValueError('월드 기반 통 판정에는 축 정렬된 바닥·벽이 필요합니다.')
        if math.hypot(floor.position[0]-bin_xy[0],floor.position[1]-bin_xy[1])>.001:
            raise ValueError('실제 통 중심과 설정 좌표가 다릅니다.')
        pose=self.tf_buffer.lookup_transform('base_link',target.name,Time()).transform
        q=(pose.rotation.x,pose.rotation.y,pose.rotation.z,pose.rotation.w)
        shape=marker_shape(model_file(self.models_dir,target.part_type))
        if shape is None or shape.shape!='cylinder':raise ValueError('지원하지 않는 부품 충돌 형상')
        offset=rotate(q,shape.xyz)
        center=[v+o for v,o in zip((pose.translation.x,pose.translation.y,pose.translation.z),offset)]
        axis=rotate(q,rotate(shape.quat,(0.,0.,1.)))
        extents=cylinder_extents(axis,shape.scale[0]/2,shape.scale[2]/2)
        thickness=min(min(w.size) for w in walls)
        inside=all(abs(center[k]-floor.position[k])+extents[k] <= floor.size[k]/2-thickness+.001 for k in [0,1])
        bottom=floor.position[2]+floor.size[2]/2
        top=max(w.position[2]+w.size[2]/2 for w in walls)
        return inside and center[2]-extents[2]>=bottom-.002 and center[2]+extents[2]<=top+.001

    def pick_one(self, target, bin_xy, seq, is_last):
        self._compact_return_active = (not is_last and self.get_parameter('compact_cycle_return').value)
        try:
            return self.pick_one_with_world_check(target,bin_xy,seq,is_last)
        finally:
            self._compact_return_active = False

    def pick_one_with_world_check(self, target, bin_xy, seq, is_last):
        started=time.monotonic()
        row=super().pick_one(target,bin_xy,seq,is_last)
        if not self.get_parameter('team2_world_bin_check').value or 'final_x' not in row:
            return row
        # 원본의 15 cm 통 고정 기준 대신 프로필 SDF와 실제 부품 자세로 독립 판정한다.
        # 다른 종류의 이동·흡착 실패를 성공으로 바꾸지는 않는다.
        legacy_outside='상자 밖에 떨어짐' in row['error']
        if row['result']=='ok' or legacy_outside:
            try:
                if not self.world_bin_contains(target,bin_xy):
                    row.update(result='failed',error='실제 통 경계 밖이거나 부품이 벽 위로 나왔습니다.')
                elif legacy_outside:
                    self.go_ready()
                    row.update(result='ok',error='')
            except (ValueError,RuntimeError,TransformException) as exc:
                row.update(result='failed',error=str(exc))
        row['plan_ms']=(time.monotonic()-started)*1000
        return row

    def on_scene_request_ack(self, msg):
        try:self.scene_ack=json.loads(msg.data)
        except ValueError:pass

    def on_spawn_scene(self, request, response):
        # 장면 초기화와 집기를 같은 잠금으로 묶어 이동 중 부품이 교체되지 않게 한다.
        if not self.motion_lock.acquire(blocking=False):
            response.success, response.message = False, '이동 중에는 장면을 초기화할 수 없습니다.'
            return response
        try:
            self.scene_ack = None
            self.scene_pub = self.create_publisher(String, '/team2/scene_request', 10)
            from ur5e_pick_place.team2_demo_profile import scene_packet
            fixed_seed = int(self.get_parameter('scene_seed').value)
            packet = scene_packet(int(self.get_parameter('max_objects').value),
                                  self._scene_counter if fixed_seed < 0 else fixed_seed,
                                  self.value('scene_position_jitter_mm'))
            self._scene_counter += 1
            packet.update(seq=time.time_ns(), t_capture=time.time())
            if not self.scene_spawn_client.wait_for_service(timeout_sec=10):
                raise RuntimeError('/team2/spawn_exact 서비스가 없습니다. team2_link를 확인하세요.')
            future = None
            started = time.monotonic()
            # 합성 패킷은 장면 배치용이다. 실제 검출 결과로 위장하지 않고 표식을 남긴다.
            # 스폰 응답에서 같은 seq를 확인한 뒤 발행을 끝내고 실제 카메라 입력을 기다린다.
            while time.monotonic()-started < 60:
                self.scene_pub.publish(String(data=json.dumps(packet)))
                # 검출/로봇 목표 토픽과 장면 초기화를 분리한다. 수신 ACK 번호가 맞아야 생성한다.
                ack=self.scene_ack
                if future is None and ack is not None and ack.get('seq')==packet['seq']:
                    if not ack.get('accepted'):raise RuntimeError(ack.get('error','장면 요청 거부'))
                    future = self.scene_spawn_client.call_async(Trigger.Request())
                if future is not None and future.done():
                    result = future.result()
                    if not result.success or f'seq={packet["seq"]} ' not in result.message:
                        raise RuntimeError('요청한 장면의 스폰을 확인하지 못함: '+result.message)
                    # 학습용 클래스는 검출 예측이 아니라 실제로 생성한 모델 종류에서 가져온다.
                    self.truth_pub.publish(String(data=json.dumps(packet)))
                    response.success, response.message = True, result.message
                    return response
                time.sleep(.02)
            raise RuntimeError('장면 초기화 시간 초과')
        except (ValueError, RuntimeError) as exc:
            response.success, response.message = False, str(exc)
            return response
        finally:
            if self.scene_pub is not None:
                self.destroy_publisher(self.scene_pub)
                self.scene_pub = None
            self.motion_lock.release()

    def nearby_ik(self, position, quat_xyzw):
        # pymoveit2의 외부 응답 대기 시간과 IK 서버의 계산 제한은 서로 다르다.
        # 서버 계산 시간을 명시하고, tool0 목표임을 고정해 짧은 기본 시간의 실패를 줄인다.
        current = self.current_joints()
        seeds = [current]
        if self.ready_joints is not None:
            # 초기 관절값에 따라 IK가 실패할 수 있어 대기 자세도 별도 초기값으로 시도한다.
            seeds.append(self.ready_joints)
        candidates = []
        for seed in seeds:
            request = GetPositionIK.Request()
            ik = request.ik_request
            ik.group_name, ik.ik_link_name = 'ur_manipulator', 'tool0'
            ik.avoid_collisions = True
            ik.timeout.sec = 1
            ik.pose_stamped.header.frame_id = 'base_link'
            ik.pose_stamped.header.stamp = self.get_clock().now().to_msg()
            p, q = ik.pose_stamped.pose.position, ik.pose_stamped.pose.orientation
            p.x, p.y, p.z = map(float, position)
            q.x, q.y, q.z, q.w = map(float, quat_xyzw)
            ik.robot_state.joint_state.name = UR_JOINTS
            ik.robot_state.joint_state.position = list(seed)
            result = self.call(self.ik_client, request)
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                continue
            values = dict(zip(result.solution.joint_state.name, result.solution.joint_state.position))
            joints = []
            for name, start in zip(UR_JOINTS, current):
                value = values[name]
                # 같은 자세의 ±360도 해 중 현재 각도에 가까운 값을 골라 불필요한 회전을 줄인다.
                near = value + 2*math.pi*round((start-value)/(2*math.pi))
                limit = math.pi if name == 'elbow_joint' else 2*math.pi
                joints.append(near if abs(near) <= limit else value)
            candidates.append(joints)
        if not candidates:
            return None
        # 첫 번째로 풀린 해가 가장 가까운 해라는 보장은 없다. 같은 목표의 후보 중
        # 가장 크게 움직이는 관절의 회전량을 먼저 줄여 긴 우회·손목 회전을 억제한다.
        return min(candidates, key=lambda joints: (
            max(abs(a-b) for a,b in zip(joints,current)),
            sum((a-b)**2 for a,b in zip(joints,current))))

    def _timed_motion(self, label, operation, *args):
        # 시스템 시계 보정에 영향받지 않는 실제 경과 시간으로 계획·실행을 함께 측정한다.
        # 예외가 발생해도 시간을 남기고, 원래 반환값과 예외는 호출자에게 그대로 전달한다.
        started = time.monotonic()
        status = '중단 또는 오류'
        self.get_logger().info(f'[구간 시작] {label}')
        try:
            result = operation(*args)
            status = '호출 완료'
            return result
        finally:
            self.get_logger().info(
                f'[구간 시간] {label}: {time.monotonic() - started:.3f}초 ({status})')

    def move_tip(self, x, y, tip_z, yaw_deg=0.0, straight=False):
        # 자동 사이클에만 경유점·놓기 보정을 적용해 수동 지정 좌표의 의미를 유지한다.
        # 작업면→좌우 끝 상자로 한 번에 이동하면, 현재 자세에 따라 IK가 실패할 수 있다.
        # 먼저 상자 안쪽의 높은 경유점으로 가서 같은 충돌 검사 아래 바깥쪽으로 접근한다.
        if (self._cycle_routing and not straight and abs(tip_z-self.value('bin_drop_z')) < 1e-6
                and any(abs(y-by) < 1e-6 for by in self.get_parameter('bin_y').value)):
            inward_y = math.copysign(min(abs(y)*0.65, 0.55), y)
            self.get_logger().info(f'상자 안쪽 경유점: (0.20, {inward_y:.3f}) m')
            self._timed_motion('상자 안쪽 경유점', super().move_tip, 0.20, inward_y,
                               self.value('work_plane_z') + self.value('travel_height_m'), yaw_deg)
            # 상자 자체는 ±0.85 m에 유지한다. 중심보다 35 mm 안쪽에 놓으면
            # 최대 도달 거리 근처의 IK 제약을 줄이며, 안착 판정은 원래 상자 중심으로 한다.
            inset = self.value('bin_inward_drop_m')
            if not 0 <= inset <= 0.04:
                raise ValueError('bin_inward_drop_m은 0..0.04 m여야 합니다.')
            y -= math.copysign(inset, y)
            self.get_logger().info(f'상자 내부 놓기 목표: ({x:.3f}, {y:.3f}) m')
        return self._timed_motion(
            f'목표 이동 x={x:.3f}, y={y:.3f}, z={tip_z:.3f}, straight={straight}',
            super().move_tip, x, y, tip_z, yaw_deg, straight)

    def go_ready(self):
        # 상자 끝에서 대기 관절값으로 바로 회전하면 빈 패드와 팔이 겹치는
        # 경로가 나올 수 있다. 안쪽·정면의 높은 지점을 거쳐 복귀한다.
        if self._cycle_routing and self.ready_joints is not None:
            try:
                pose = self.tf_buffer.lookup_transform('base_link', 'tool0', Time())
            except TransformException as exc:
                raise RuntimeError(f'복귀 전 공구 위치 확인 실패: {exc}') from exc
            y = pose.transform.translation.y
            if abs(y) > 0.65:
                height = self.value('work_plane_z') + self.value('travel_height_m')
                self.get_logger().info('상자→안쪽 높은 경유점→정면→대기 자세 복귀')
                self._timed_motion('상자에서 안쪽으로 복귀', super().move_tip,
                                   0.20, math.copysign(0.55, y), height)
                self._timed_motion('정면으로 복귀', super().move_tip, 0.40, 0.0, height)
                # 정면 경유점 생략은 실제 시험에서 더 오래 걸려 유지하지 않는다.
                # 중간 부품만 대기 관절 이동을 생략하고, 마지막은 최종 자세까지 복귀한다.
                if self._compact_return_active:
                    return
        return self._timed_motion('최종 대기 자세 복귀', super().go_ready)

    def on_markers(self, msg):
        # 카메라 분류 결과와 별개로 기존 모델의 형상·크기를 읽어 종류 대조에 사용한다.
        kinds = {'box': Marker.CUBE, 'cylinder': Marker.CYLINDER, 'sphere': Marker.SPHERE}
        current = {}
        for marker in msg.markers:
            if marker.action != Marker.ADD:
                continue
            matches = [p for p, shape in self.shapes.items() if shape is not None
                       and kinds.get(shape.shape) == marker.type
                       and all(abs(a-b) < 1e-6 for a,b in zip(
                           shape.scale, (marker.scale.x, marker.scale.y, marker.scale.z))) ]
            if len(matches) == 1:
                current[marker.header.frame_id] = matches[0]
        self.object_types = current
        self.marker_received = time.monotonic()

    def on_run_cycle(self, request, response):
        mode = self.get_parameter('cycle_scene_mode').value
        if mode == 'respawn':
            return super().on_run_cycle(request, response)
        if mode != 'existing':
            response.success, response.message = False, f'알 수 없는 cycle_scene_mode: {mode}'
            return response
        if not self.motion_lock.acquire(blocking=False):
            response.success, response.message = False, '이미 이동 중입니다.'
            return response
        self._cycle_routing = True
        try:
            age = float(self.get_parameter('target_max_age_s').value)
            if not math.isfinite(age) or age <= 0:
                raise ValueError('target_max_age_s는 양수여야 합니다.')
            if self.target_received is None or time.monotonic()-self.target_received > age:
                raise ValueError('최신 로봇 목표가 없습니다. 카메라·TCP 연결을 확인하세요.')
            # 한 사이클은 이 패킷을 기준으로 끝낸다. 이동 중 새 검출로 목표를 바꾸지 않는다.
            # 위의 age는 이 노드가 목표를 받아들인 뒤의 시간이며 촬영 프레임 나이는 아니다.
            packet = self.latest
            part_types = list(self.get_parameter('part_types').value)
            xs, ys = self.get_parameter('bin_x').value, self.get_parameter('bin_y').value
            if len(xs) != len(part_types) or len(ys) != len(part_types):
                raise ValueError('part_types, bin_x, bin_y 길이가 같아야 합니다.')
            bins = dict(zip(part_types, zip(xs, ys)))
            targets, skipped = plan_spawns(packet, self.plane(), part_types,
                                           int(self.get_parameter('max_objects').value))
            if not targets:
                raise ValueError('집을 물체가 없습니다')
            if skipped:
                raise ValueError('검출 대상 일부를 처리할 수 없습니다: ' + '; '.join(skipped))
            if self.marker_received is None or time.monotonic()-self.marker_received > age:
                raise ValueError('기존 물체 마커가 없습니다. 스폰 노드와 TF를 확인하세요.')
            observed = {}
            for name, part_type in self.object_types.copy().items():
                x,y,z = self.wait_object_xyz(name)
                # 먼저 작업면 부근 높이로 좁히고, 아래 수평 위치 대응에서 실제 대상을 고른다.
                # 상자 바닥도 낮으므로 높이 조건만으로 상자 속 부품이 제외되는 것은 아니다.
                if abs(z-self.value('work_plane_z')) < 0.025:
                    observed[name] = dict(x=x,y=y,z=z,part_type=part_type)
            targets = bind_existing_targets(targets, observed)
            # 재스폰을 호출하지 않으므로 인식 전에 있던 물체의 이름과 모델 ID가 유지된다.
            self.get_logger().info('기존 물체 대응: ' + ', '.join(
                f'id={t.source_id}→{t.name}({t.part_type})' for t in targets))
            self.prepare_motion()
            self.go_ready()
            rows = []
            for index, target in enumerate(targets):
                row = self.pick_one(target, bins[target.part_type], packet.get('seq'),
                                    is_last=index == len(targets)-1)
                rows.append(row)
                self.record(row)
                if row['result'] != 'ok' or row['error']:
                    break
            ok = sum(r['result'] == 'ok' and not r['error'] for r in rows)
            # 원본의 마지막 복귀 실패는 경고만 남길 수 있어, 통합 응답에서 별도로 확인한다.
            if ok == len(targets) and not ready_reached(self.current_joints(), self.ready_joints or []):
                raise RuntimeError('상자 투입 후 대기 자세 복귀가 확인되지 않았습니다.')
            response.success = ok == len(targets)
            response.message = (f'existing seq={packet.get("seq")}: {ok}/{len(targets)} 개 상자 투입. '
                                + ', '.join(f'{r["obj"]}={r["part_type"]}:{r["result"]}' for r in rows))
            if rows and rows[-1]['error']:
                response.message += ' / 실패: ' + rows[-1]['error']
        except (ValueError, RuntimeError) as exc:
            response.success, response.message = False, str(exc)
        finally:
            # 실패해도 실행 상태를 해제해 다음 요청이 계속 '이동 중'으로 막히지 않게 한다.
            self._cycle_routing = False
            self.motion_lock.release()
        self.get_logger().info(response.message)
        return response


def main(args=None):
    from rclpy.signals import SignalHandlerOptions
    # 기본 핸들러는 context를 먼저 닫는다. 클라이언트 정리까지 context를 유지한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    def request_stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    node = Team2PickPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    from rclpy.clock import Clock, ClockType
    # Gazebo 시계가 멈춰도 Python 신호 핸들러를 처리하도록 wall-time으로 wait-set을 깨운다.
    wake_timer = node.create_timer(.1, lambda: None, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Team2 shutdown: stop executor')
        executor.remove_node(node)
        # Jazzy의 기본 shutdown은 guard를 먼저 파괴하고 worker를 나중에 join한다.
        # 아직 끝나는 중인 action 콜백이 guard를 깨우다 InvalidHandle을 내지 않게 순서를 바꾼다.
        executor._executor.shutdown(wait=True)
        executor.shutdown()
        # 노드가 먼저 사라지면 action future 정리 중 Destroyable 예외가 발생한다.
        # 클라이언트 자원을 명시적으로 해제한 뒤 TF 구독·노드를 제거한다.
        node.moveit2.destroy()
        node.tf_listener.unregister()
        node.destroy_timer(wake_timer)
        node.get_logger().info('Team2 shutdown: clients released before node/context')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
