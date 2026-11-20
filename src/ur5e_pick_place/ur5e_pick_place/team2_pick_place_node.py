"""Team 2 existing-object integration, reusing the unchanged motion implementation.

existing: bind camera detections to already spawned models; never respawn.
respawn: explicitly use the original photo-to-simulation replication cycle.
"""
from dataclasses import replace
import math
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.time import Time
from tf2_ros import TransformException
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes

from ur5e_pick_place.object_spawner import marker_shape, model_file, plan_spawns
from ur5e_pick_place.pick_place_node import PickPlaceNode, UR_JOINTS


def bind_existing_targets(targets, observed, tolerance=0.005):
    """Match by position first; reject wrong classification instead of replacing models."""
    bound, used = [], set()
    for target in targets:
        candidates = [(name, obj) for name, obj in observed.items()
                      if math.hypot(target.x-obj['x'], target.y-obj['y']) <= tolerance]
        if len(candidates) != 1:
            raise ValueError(f'기존 물체 대응 실패: id={target.source_id}, 후보 {len(candidates)}개')
        name, obj = candidates[0]
        if name in used:
            raise ValueError(f'하나의 물체에 검출이 중복 대응됨: {name}')
        if obj['part_type'] != target.part_type:
            raise ValueError(f'기존 물체 종류 불일치: {name}, 검출={target.part_type}, 모델={obj["part_type"]}')
        used.add(name)
        bound.append(replace(target, name=name))
    return bound


def ready_reached(actual, expected, tolerance=0.05):
    return (len(actual) == len(expected) == 6
            and all(math.isfinite(a) and abs(a-b) < tolerance for a,b in zip(actual, expected)))


class Team2PickPlaceNode(PickPlaceNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('cycle_scene_mode', 'existing')
        self.declare_parameter('target_max_age_s', 3.0)
        self.declare_parameter('bin_inward_drop_m', 0.035)
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik', callback_group=self.cb_group)
        self.target_received = None
        self.marker_received = None
        self.object_types = {}
        self._cycle_routing = False
        self.shapes = {p: marker_shape(model_file(self.models_dir, p))
                       for p in self.get_parameter('part_types').value}
        # Marker callbacks must still run while a service waits for TF or motion.
        self.markers_sub = self.create_subscription(
            MarkerArray, '/sim/object_markers', self.on_markers, 10,
            callback_group=ReentrantCallbackGroup())

    def on_targets(self, msg):
        previous = self.latest
        super().on_targets(msg)
        if self.latest is not previous:
            self.target_received = time.monotonic()

    def nearby_ik(self, position, quat_xyzw):
        # pymoveit2의 외부 응답 대기 시간과 IK 서버의 계산 제한은 서로 다르다.
        # 서버 계산 시간을 명시하고, tool0 목표임을 고정해 짧은 기본 시간의 실패를 줄인다.
        current = self.current_joints()
        seeds = [current]
        if self.ready_joints is not None:
            seeds.append(self.ready_joints)
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
                near = value + 2*math.pi*round((start-value)/(2*math.pi))
                limit = math.pi if name == 'elbow_joint' else 2*math.pi
                joints.append(near if abs(near) <= limit else value)
            return joints
        return None

    def move_tip(self, x, y, tip_z, yaw_deg=0.0, straight=False):
        # 작업면→좌우 끝 상자로 한 번에 이동하면, 현재 자세에 따라 IK가 실패할 수 있다.
        # 먼저 상자 안쪽의 높은 경유점으로 가서 같은 충돌 검사 아래 바깥쪽으로 접근한다.
        if (self._cycle_routing and not straight and abs(tip_z-self.value('bin_drop_z')) < 1e-6
                and any(abs(y-by) < 1e-6 for by in self.get_parameter('bin_y').value)):
            inward_y = math.copysign(min(abs(y)*0.65, 0.55), y)
            self.get_logger().info(f'상자 안쪽 경유점: (0.20, {inward_y:.3f}) m')
            super().move_tip(0.20, inward_y,
                             self.value('work_plane_z') + self.value('travel_height_m'), yaw_deg)
            # 상자 자체는 ±0.85 m에 유지한다. 중심보다 35 mm 안쪽에 놓으면
            # 최대 도달 거리 근처의 IK 제약을 줄이며, 안착 판정은 원래 상자 중심으로 한다.
            inset = self.value('bin_inward_drop_m')
            if not 0 <= inset <= 0.04:
                raise ValueError('bin_inward_drop_m은 0..0.04 m여야 합니다.')
            y -= math.copysign(inset, y)
            self.get_logger().info(f'상자 내부 놓기 목표: ({x:.3f}, {y:.3f}) m')
        return super().move_tip(x, y, tip_z, yaw_deg, straight)

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
                super().move_tip(0.20, math.copysign(0.55, y), height)
                super().move_tip(0.40, 0.0, height)
        return super().go_ready()

    def on_markers(self, msg):
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
                # Only objects on the work plane participate, not past bin contents.
                if abs(z-self.value('work_plane_z')) < 0.025:
                    observed[name] = dict(x=x,y=y,z=z,part_type=part_type)
            targets = bind_existing_targets(targets, observed)
            # No /sim/spawn_objects call here: model names and identities persist.
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
            self._cycle_routing = False
            self.motion_lock.release()
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Team2PickPlaceNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
