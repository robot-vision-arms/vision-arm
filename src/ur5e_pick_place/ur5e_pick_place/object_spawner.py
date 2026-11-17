"""Gazebo 에 가상 나사/너트 스폰 (3주차, 팀원 3).

비전 결과(work_plane mm, 각도, part_type)를 grasp.work_to_base 로 base_link 좌표로 바꾼 뒤 같은 자리에 모델을 놓는다.
모델 이름은 obj_0 ~ obj_{max_objects-1} 로 고정한다. urdf/ur5e_vacuum.urdf.xacro 의 DetachableJoint 가
붙일 모델 이름(child_model)을 미리 알아야 하기 때문.

로봇 베이스가 Gazebo world 원점에 있으므로 base_link 좌표를 world 좌표로 그대로 쓴다.

DetachableJoint 동작 (gz-sim8 소스와 11-17 실기로 확인, docs/DECISIONS.md 4장):
- 시뮬레이터 시작 때 '붙이기 요청'이 켜져 있어, 슬롯마다 **처음** 나타난 obj_k 는 곧바로 패드(wrist_3_link)에 붙는다.
  그대로 두면 부품이 팔에 고정된 채 남으므로 "attached" 를 확인하면 곧바로 detach 한다.
- 한 번 detach 한 슬롯은 같은 이름 모델을 지웠다 다시 만들어도 **자동으로 붙지 않는다** (요청 플래그가 꺼져 있음).
  attach 를 보내면 지워진 옛 모델 대신 새 모델을 다시 찾아 붙인다.
- 붙어 있지 않을 때 detach 를 보내면 무시된다 (다음 attach 에 영향 없음). 그래서 안전하게 한 번 더 보낼 수 있다.

Gazebo 와는 gz-transport 로 직접 통신한다 (ros_gz_bridge 불필요). 스폰·삭제 요청만 `gz service` 명령으로 보낸다 (_request 참고):
    스폰  /world/<world>/create   gz.msgs.EntityFactory → Boolean
    삭제  /world/<world>/remove   gz.msgs.Entity        → Boolean
    상태  /vacuum/obj_k/state     gz.msgs.StringMsg ("attached" / "detached")
    떼기  /vacuum/obj_k/detach    gz.msgs.Empty

단독 실행 (sim.launch.py 를 띄운 뒤, 같은 환경을 source 한 터미널에서):
    python3 -m ur5e_pick_place.object_spawner
    ros2 service call /sim/spawn_objects std_srvs/srv/Trigger   # 최신 /robot/targets 패킷대로 스폰
    ros2 service call /sim/clear_objects std_srvs/srv/Trigger   # obj_0 ~ obj_{n-1} 삭제
작업 평면 위치·max_objects·part_types 는 config/sim.yaml 의 pick_place_node 값을 그대로 읽는다
(sim.launch.py 의 base_link→work_plane TF 와 같은 값을 쓰기 위함).

TF·RViz 표시 (노드가 하는 일):
    TF      base_link → obj_k   Gazebo 의 실제 부품 위치 (/world/<world>/pose/info) 를 10 Hz 로 옮긴다.
                                흡착해서 들면 TF 도 따라 움직이고, 부품을 지우면 그 슬롯 TF 발행을 멈춘다.
    마커    /sim/object_markers  visualization_msgs/MarkerArray. obj_k 프레임에 붙인 부품 모양(원기둥 등).
                                모양은 models/<part_type>/model.sdf 의 visual 에서 읽는다 (종류가 늘어도 코드 수정 없음).
    RViz 에서 TF 표시와 MarkerArray(/sim/object_markers) 를 추가하면 Gazebo 와 같은 자리에 부품이 보인다.
"""
from __future__ import annotations

import math
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

from ur5e_pick_place.grasp import PlanePose, work_to_base

SPAWN_LIFT_M = 0.001   # 바닥과 겹친 채 생기면 물리 엔진이 튕겨 내므로 1 mm 위에서 놓는다


def model_file(models_dir: Path, part_type: str) -> Path:
    """part_type 이름 = models/ 아래 폴더 이름. 공구 종류를 늘릴 때 폴더만 추가하면 된다."""
    return models_dir / part_type / 'model.sdf'


def slot_name(k: int) -> str:
    return f'obj_{k}'


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """고정축 roll→pitch→yaw (SDF <pose> 규칙) 를 쿼터니언 (x, y, z, w) 로."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


@dataclass(frozen=True)
class MarkerShape:
    """RViz 마커로 그릴 부품 모양. 위치·자세는 모델 원점(obj_k 프레임) 기준."""
    shape: str                                   # 'cylinder' / 'box' / 'sphere'
    scale: tuple[float, float, float]            # RViz Marker.scale (m). 원기둥은 (지름, 지름, 길이)
    xyz: tuple[float, float, float]
    quat: tuple[float, float, float, float]      # (x, y, z, w)
    rgba: tuple[float, float, float, float]


def marker_shape(sdf_path: Path) -> MarkerShape | None:
    """model.sdf 의 첫 link 와 첫 visual 에서 모양·크기·위치·색을 읽는다. 모르는 형상이면 None.

    링크 <pose> 와 visual <pose> 를 합친다 (둘 중 하나만 있어도 됨, 단순화를 위해 visual 쪽은 위치만 더한다).
    """
    def pose_of(elem):
        text = elem.findtext('pose') if elem is not None else None
        values = [float(v) for v in text.split()] if text else [0.0] * 6
        return values + [0.0] * (6 - len(values))

    link = ET.parse(sdf_path).getroot().find('./model/link')
    visual = link.find('visual') if link is not None else None
    geometry = visual.find('geometry') if visual is not None else None
    if geometry is None:
        return None
    lp, vp = pose_of(link), pose_of(visual)
    if (g := geometry.find('cylinder')) is not None:
        d, length = 2 * float(g.findtext('radius')), float(g.findtext('length'))
        shape, scale = 'cylinder', (d, d, length)
    elif (g := geometry.find('box')) is not None:
        shape, scale = 'box', tuple(float(v) for v in g.findtext('size').split())
    elif (g := geometry.find('sphere')) is not None:
        d = 2 * float(g.findtext('radius'))
        shape, scale = 'sphere', (d, d, d)
    else:
        return None
    color = visual.findtext('material/diffuse') or visual.findtext('material/ambient') or '0.7 0.7 0.7 1'
    rgba = tuple(float(v) for v in color.split())
    return MarkerShape(shape, scale, (lp[0] + vp[0], lp[1] + vp[1], lp[2] + vp[2]),
                       rpy_to_quaternion(*lp[3:6]), rgba)


@dataclass(frozen=True)
class SpawnRequest:
    """Gazebo 에 놓을 부품 하나. 좌표는 world(=base_link) 기준 m, yaw 는 rad."""
    name: str
    part_type: str
    x: float
    y: float
    z: float
    yaw_rad: float
    source_id: object = None   # 패킷의 물체 id (기록용)


def plan_spawns(packet: dict, plane: PlanePose, part_types: list[str],
                max_objects: int) -> tuple[list[SpawnRequest], list[str]]:
    """패킷 → 스폰 목록과 건너뛴 이유 목록.

    - calib_ok 가 false 면 mm 좌표가 없으므로 ValueError (미보정 좌표로 부품을 놓지 않는다).
    - part_types 에 없는 종류(unknown 포함)·mm 좌표가 null 인 물체는 건너뛴다.
    - 남은 물체를 패킷 순서대로 max_objects 개까지 obj_0, obj_1, … 슬롯에 배정한다.
    - angle_deg 가 null(너트 등 장축 없음)이면 yaw 0. 작업 평면이 돌아가 있으면 그 각도를 더한다.
    """
    if not packet.get('calib_ok'):
        raise ValueError('calib_ok 가 false 인 패킷은 스폰하지 않습니다 (mm 좌표 없음).')
    requests, skipped = [], []
    for obj in packet.get('objects') or []:
        oid = obj.get('id')
        part_type = obj.get('part_type')
        if part_type not in part_types:
            skipped.append(f'id={oid}: 분류 대상이 아님 ({part_type})')
            continue
        x_mm, y_mm = obj.get('x_mm'), obj.get('y_mm')
        if x_mm is None or y_mm is None or not (math.isfinite(x_mm) and math.isfinite(y_mm)):
            skipped.append(f'id={oid}: mm 좌표 없음')
            continue
        if len(requests) >= max_objects:
            skipped.append(f'id={oid}: 슬롯 부족 (max_objects={max_objects})')
            continue
        x, y, z = work_to_base(x_mm, y_mm, plane)
        angle = obj.get('angle_deg')
        yaw = math.radians((angle if angle is not None else 0.0) + plane.yaw_deg)
        requests.append(SpawnRequest(slot_name(len(requests)), part_type, x, y, z + SPAWN_LIFT_M, yaw, oid))
    return requests, skipped


def load_sim_params(config_file: Path) -> dict:
    """config/sim.yaml 에서 pick_place_node 의 ros__parameters 를 읽는다."""
    import yaml
    with open(config_file, encoding='utf-8') as f:
        return yaml.safe_load(f)['pick_place_node']['ros__parameters']


def plane_from_params(p: dict) -> PlanePose:
    return PlanePose(p['work_plane_x'], p['work_plane_y'], p['work_plane_z'], p['work_plane_yaw_deg'])


class GzSpawner:
    """gz-transport 로 부품을 스폰·삭제하고, 스폰 직후 DetachableJoint 를 떼어 둔다."""

    def __init__(self, models_dir: Path, world: str = 'workcell', max_objects: int = 3,
                 timeout_s: float = 5.0, attach_wait_s: float = 2.0):
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.stringmsg_pb2 import StringMsg
        from gz.transport13 import Node

        self.models_dir = Path(models_dir)
        self.world = world
        self.max_objects = max_objects
        self.timeout_s = timeout_s
        self.attach_wait_s = attach_wait_s   # 처음 쓰는 슬롯의 자동 attach 를 기다리는 시간 (실기에서는 1초 안에 옴)
        self._node = Node()
        self._cond = threading.Condition()
        self._states: dict[str, list[str]] = {slot_name(k): [] for k in range(max_objects)}
        self._detach = {}
        self._empty = Empty
        self._poses = {}   # obj_k → (받은 시각 monotonic, (x, y, z), (qx, qy, qz, qw))
        from gz.msgs10.pose_v_pb2 import Pose_V
        self._node.subscribe(Pose_V, f'/world/{world}/pose/info', self._on_poses)
        for k in range(max_objects):
            name = slot_name(k)
            self._node.subscribe(StringMsg, f'/vacuum/{name}/state',
                                 lambda msg, name=name: self._on_state(name, msg.data))
            self._detach[name] = self._node.advertise(f'/vacuum/{name}/detach', Empty)

    def _on_poses(self, msg) -> None:
        now = time.monotonic()
        for pose in msg.pose:
            if pose.name in self._states:   # obj_0 ~ obj_{n-1} 모델만
                p, q = pose.position, pose.orientation
                self._poses[pose.name] = (now, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def object_poses(self, max_age_s: float = 0.5) -> dict:
        """지금 Gazebo 에 있는 obj_k 의 world(=base_link) 위치·자세. 지워진 슬롯은 곧 목록에서 빠진다."""
        now = time.monotonic()
        return {name: (xyz, quat) for name, (t, xyz, quat) in list(self._poses.items()) if now - t <= max_age_s}

    def _on_state(self, name: str, state: str) -> None:
        with self._cond:
            self._states[name].append(state)
            self._cond.notify_all()

    def _wait_state(self, name: str, state: str, since: int, timeout_s: float) -> bool:
        with self._cond:
            return self._cond.wait_for(lambda: state in self._states[name][since:], timeout=timeout_s)

    def _request(self, service: str, request, reply_type) -> bool:
        """gz 서비스 요청. 응답 data 가 true 면 True.

        같은 프로세스에서 gz-transport 구독(위치·상태)과 Python request() 를 함께 쓰면 응답을 못 받고
        시간 초과되는 일이 들쭉날쭉 생겼다 (11-17, 같은 요청을 gz CLI 로는 매번 0.3초에 성공).
        그래서 요청만 `gz service` 별도 프로세스로 보낸다. 스폰·삭제는 드물어서 0.3초 정도 걸려도 괜찮다.
        """
        import subprocess
        from google.protobuf import text_format

        command = ['gz', 'service', '-s', service,
                   '--reqtype', request.DESCRIPTOR.full_name, '--reptype', reply_type.DESCRIPTOR.full_name,
                   '--timeout', str(int(self.timeout_s * 1000)),
                   '--req', text_format.MessageToString(request, as_one_line=True)]
        try:
            out = subprocess.run(command, capture_output=True, text=True, timeout=self.timeout_s + 5).stdout
        except subprocess.TimeoutExpired:
            return False
        return 'data: true' in out

    def spawn(self, req: SpawnRequest) -> None:
        """부품을 만들고 DetachableJoint 가 붙인 것을 떼어 작업 평면 위에 남긴다. 실패하면 RuntimeError."""
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_factory_pb2 import EntityFactory

        sdf = model_file(self.models_dir, req.part_type)
        if not sdf.is_file():
            raise RuntimeError(f'모델 파일 없음: {sdf}')
        msg = EntityFactory()
        msg.sdf_filename = str(sdf)
        msg.name = req.name
        msg.allow_renaming = False   # 이름이 바뀌면 DetachableJoint 가 못 찾는다
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = req.x, req.y, req.z
        msg.pose.orientation.z = math.sin(req.yaw_rad / 2.0)
        msg.pose.orientation.w = math.cos(req.yaw_rad / 2.0)

        with self._cond:
            since = len(self._states[req.name])
            used = 'detached' in self._states[req.name]
        if not self._request(f'/world/{self.world}/create', msg, Boolean):
            raise RuntimeError(f'{req.name} 스폰 요청 실패 (같은 이름이 남아 있는지 확인)')
        if used:
            return   # 이미 한 번 뗀 슬롯은 자동으로 붙지 않는다
        # 처음 쓰는 슬롯(또는 이 노드가 뜨기 전에 쓰인 슬롯)은 자동으로 붙는지 기다린다
        if self._wait_state(req.name, 'attached', since, self.attach_wait_s):
            self.release(req.name)
        else:
            self._detach[req.name].publish(self._empty())   # 혹시 늦게 붙었을 때를 대비. 안 붙어 있으면 무시됨
            with self._cond:
                self._states[req.name].append('detached')   # 다음부터 기다리지 않도록 '쓴 슬롯'으로 표시

    def release(self, name: str) -> None:
        """슬롯을 detach 하고 "detached" 를 확인한다. 메시지가 유실될 수 있어 확인될 때까지 몇 번 다시 보낸다."""
        with self._cond:
            since = len(self._states[name])
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            self._detach[name].publish(self._empty())
            with self._cond:
                if self._cond.wait_for(lambda: 'detached' in self._states[name][since:], timeout=0.2):
                    return
        raise RuntimeError(f'{name}: detach 확인 실패')

    def remove(self, name: str) -> bool:
        """모델을 지운다. Gazebo 는 없는 이름에도 true 를 돌려주므로 응답이 오면 True, 서비스 오류면 False."""
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.entity_pb2 import Entity

        with self._cond:
            attached = self._states[name][-1:] == ['attached']
        if attached:   # 붙은 채 지우면 플러그인이 '붙음' 상태로 남아 다음 attach 를 거부하므로 먼저 뗀다
            self.release(name)
        msg = Entity()
        msg.name = name
        msg.type = Entity.MODEL
        return any(self._request(f'/world/{self.world}/remove', msg, Boolean) for _ in range(2))   # 한 번 더 시도

    def clear(self) -> None:
        """obj_0 ~ obj_{n-1} 을 모두 지운다 (없는 슬롯도 오류 아님)."""
        failed = [slot_name(k) for k in range(self.max_objects) if not self.remove(slot_name(k))]
        if failed:
            raise RuntimeError(f'삭제 요청 실패: {", ".join(failed)}')

    def spawn_all(self, requests: list[SpawnRequest]) -> None:
        """이전 부품을 치우고 새 목록을 스폰한다. 지운 직후 같은 이름을 만들면 Gazebo 가 거부할 수 있어 잠깐 기다린다."""
        self.clear()
        time.sleep(0.5)
        for req in requests:
            self.spawn(req)


def main(args=None):
    import json

    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import TransformBroadcaster
    from visualization_msgs.msg import Marker, MarkerArray

    share = Path(get_package_share_directory('ur5e_pick_place'))

    class ObjectSpawnerNode(Node):
        def __init__(self):
            super().__init__('object_spawner')
            self.declare_parameter('config_file', str(share / 'config' / 'sim.yaml'))
            self.declare_parameter('targets_topic', '/robot/targets')
            self.declare_parameter('tf_rate_hz', 10.0)
            p = load_sim_params(Path(self.get_parameter('config_file').value))
            # pick_place_node·RViz 와 같은 시계를 쓴다 (시뮬레이터는 /clock). 다르면 RViz 가 TF 를 오래된 값으로 버린다
            self.set_parameters([Parameter('use_sim_time', value=bool(p.get('use_sim_time', False)))])
            self.plane = plane_from_params(p)
            self.part_types = list(p['part_types'])
            self.max_objects = int(p['max_objects'])
            self.spawner = GzSpawner(share / 'models', p['world_name'], self.max_objects)
            self.latest = None
            self.lock = threading.Lock()
            self.slot_types = {}   # obj_k → part_type (이 노드가 스폰한 것만 마커 모양을 안다)
            self.shapes = {t: marker_shape(model_file(share / 'models', t)) for t in self.part_types}
            self.tf = TransformBroadcaster(self)
            self.pub_markers = self.create_publisher(MarkerArray, '/sim/object_markers', 10)
            self.create_timer(1.0 / self.get_parameter('tf_rate_hz').value, self.publish_tf)
            topic = self.get_parameter('targets_topic').value
            self.create_subscription(String, topic, self.on_targets, 10)
            self.create_service(Trigger, '/sim/spawn_objects', self.on_spawn)
            self.create_service(Trigger, '/sim/clear_objects', self.on_clear)
            self.get_logger().info(f'{topic} 의 최신 패킷을 /sim/spawn_objects 호출 때 스폰합니다. '
                                   f'작업 평면 base_link ({self.plane.x}, {self.plane.y}, {self.plane.z}) m, '
                                   f'yaw {self.plane.yaw_deg}°')

        def on_targets(self, msg):
            try:
                self.latest = json.loads(msg.data)
            except json.JSONDecodeError as e:
                self.get_logger().warning(f'패킷 JSON 오류: {e}')

        def on_spawn(self, request, response):
            packet = self.latest
            if packet is None:
                response.success, response.message = False, '아직 받은 패킷이 없습니다.'
                return response
            with self.lock:
                try:
                    requests, skipped = plan_spawns(packet, self.plane, self.part_types, self.max_objects)
                    self.slot_types = {}
                    self.spawner.spawn_all(requests)
                    self.slot_types = {r.name: r.part_type for r in requests}
                except (ValueError, RuntimeError) as e:
                    response.success, response.message = False, f'seq={packet.get("seq")}: {e}'
                    return response
            placed = ', '.join(f'{r.name}={r.part_type}({r.x:.3f}, {r.y:.3f}, {math.degrees(r.yaw_rad):.1f}°)'
                               for r in requests) or '없음'
            response.success = True
            response.message = f'seq={packet.get("seq")} 스폰 {len(requests)}개: {placed}'
            if skipped:
                response.message += ' / 건너뜀: ' + '; '.join(skipped)
            self.get_logger().info(response.message)
            return response

        def on_clear(self, request, response):
            with self.lock:
                try:
                    self.slot_types = {}
                    self.spawner.clear()
                except RuntimeError as e:
                    response.success, response.message = False, str(e)
                    return response
            response.success, response.message = True, f'obj_0 ~ obj_{self.max_objects - 1} 삭제'
            return response

        def publish_tf(self):
            """Gazebo 의 obj_k 위치를 TF 와 마커로 내보낸다. 사라진 슬롯의 마커는 지운다."""
            stamp = self.get_clock().now().to_msg()
            poses = self.spawner.object_poses()
            markers = MarkerArray()
            for k in range(self.max_objects):
                name = slot_name(k)
                marker = Marker()
                marker.header.frame_id, marker.header.stamp = name, stamp
                marker.ns, marker.id = 'objects', k
                shape = self.shapes.get(self.slot_types.get(name))
                if name not in poses or shape is None:
                    marker.action = Marker.DELETE
                    markers.markers.append(marker)
                    if name not in poses:
                        continue
                else:
                    marker.type = {'cylinder': Marker.CYLINDER, 'box': Marker.CUBE,
                                   'sphere': Marker.SPHERE}[shape.shape]
                    marker.action = Marker.ADD
                    marker.frame_locked = True   # obj_k TF 가 움직이면 마커도 따라간다
                    p, o = marker.pose.position, marker.pose.orientation
                    p.x, p.y, p.z = shape.xyz
                    o.x, o.y, o.z, o.w = shape.quat
                    marker.scale.x, marker.scale.y, marker.scale.z = shape.scale
                    marker.color.r, marker.color.g, marker.color.b, marker.color.a = shape.rgba
                    markers.markers.append(marker)
                (x, y, z), (qx, qy, qz, qw) = poses[name]
                t = TransformStamped()
                t.header.stamp, t.header.frame_id, t.child_frame_id = stamp, 'base_link', name
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
                r = t.transform.rotation
                r.x, r.y, r.z, r.w = qx, qy, qz, qw
                self.tf.sendTransform(t)
            self.pub_markers.publish(markers)

    rclpy.init(args=args)
    node = ObjectSpawnerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
