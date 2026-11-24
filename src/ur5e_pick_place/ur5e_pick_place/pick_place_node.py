"""Gazebo UR5e 픽앤플레이스 노드 (2~4주차).

robot_server --publish-ros 가 TCP 로 받은 패킷을 /robot/targets 로 다시 내보내면, 이 노드는 최신 패킷만 들고 있다가
/sim/run_cycle 서비스가 호출될 때 한 사이클을 실행한다. 10 Hz 로 들어오는 패킷마다 움직이면 로봇이 쉬지 않으므로
'패킷 한 장 → 집기 한 사이클' 로 끊는다.

한 사이클 (팀원 2 의 상자 투입 검사 check_team2_bin_reach.py --drop 에서 확인한 순서를 그대로 옮김):
    1. 최신 패킷 확인 (calib_ok 가 false 면 거부), part_types 에 없는 종류(unknown 포함)는 빼고 max_objects 개까지
    2. 스폰 노드의 /sim/spawn_objects 로 같은 위치·각도에 가상 나사/너트를 놓고, TF(base_link→obj_k)로 놓인 것을 확인
    3. 처음 한 번: 작업 시트 옆(카메라 시야 밖) travel_height_m 로 가서 그 관절 각도를 '대기 자세' 로 저장 (노드가 켜져 있는 동안 재사용)
    4. 물체마다: 부품 위 높은 곳(travel_height_m) → 직선으로 접근(윗면 + approach_height_m) → 하강(컵 끝이 윗면 + grasp_gap_m) → /sim/vacuum ON
                 ('떨어져 있음' 이면 1 mm 씩 더 내려 최대 grasp_attempts 번) → 들어 올림 → 상자 위 bin_drop_z → 진공 OFF
                 → 상자 안에 멈췄는지 TF 로 확인 → 대기 자세로 복귀 (관절 각도 목표)
    큰 이동(상자 ↔ 작업 매트)은 관절 각도 목표로 한다. 좌표 목표는 역기구학 해가 여러 개라 OMPL 이 이 구간을
    자주 못 풀었다 (11-19 팀원 2 시연, 11-20 시험 3회). 관절 목표는 해가 하나라 계획이 안정적이다 (현업의 티칭 경유점과 같은 방식).
    5. 물체마다 결과를 /sim/pick_log 로 발행 (GUI ③) + logs/sim_pick_*.csv 기록 (KPI 의 Pick 성공률)

집기는 컵 두 개 헤드 (urdf/ur5e_vacuum.urdf.xacro, object_spawner.py 설명):
    나사 = tool0 바로 아래 5 mm 컵, 너트 = tool0 +X 로 12 mm 떨어진 링 컵. 컵이 둥글어 툴 yaw 는 0 으로 고정하고,
    너트는 링 컵이 너트 중심에 오도록 tool0 을 -X 로 12 mm 옮긴다 (grasp.cup_tool_xy).
    붙일지 말지는 스폰 노드의 /sim/vacuum 이 컵·부품 기하로 판정한다 (가까이 있다고 무조건 붙이지 않음).

구독:   /robot/targets  std_msgs/String   packet 형식 JSON
발행:   /sim/pick_log   std_msgs/String   {"seq", "obj", "part_type", "result", "plan_ms", "error"}
서비스: /sim/run_cycle       std_srvs/Trigger   위 한 사이클
        /sim/move_to_target  std_srvs/Trigger   2주차 지정 좌표 이동 (target_* 파라미터)
필요:   스폰 노드 (team2_link.launch.py 가 띄움): /sim/spawn_objects, /sim/vacuum, TF base_link→obj_k

    ros2 service call /sim/run_cycle std_srvs/srv/Trigger

MoveIt 은 pymoveit2 (move_group 의 액션·서비스를 부르는 Python 클라이언트) 로 쓴다.
move_group 은 sim.launch.py 가 ur_moveit_config 로 띄운다.
"""
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import MoveItErrorCodes
from pymoveit2 import MoveIt2, MoveIt2State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from ur5e_pick_place.grasp import PlanePose, cup_tool_xy, inside_bin, needs_lift, tool_down_quaternion, work_to_base
from ur5e_pick_place.object_spawner import (HOLE_RADIUS_M, SuctionLimits, marker_shape, model_file, part_top_m,
                                            plan_spawns)
from ur5e_pick_place.workcell_scene import collision_boxes

# ur_moveit_config 의 계획 그룹·관절 이름 (1주차에 RViz MotionPlanning 패널에서 그룹 이름 확인)
UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
GROUP_NAME = 'ur_manipulator'
BASE_LINK = 'base_link'
EE_LINK = 'tool0'   # MoveIt 은 흡착 패드를 모르므로 tool0 목표 = 패드 끝 목표 + tool_length_m

# 다시 계획해도 되는 실패 코드. OMPL 은 무작위로 길을 찾으므로 같은 목표라도 다시 하면 풀리는 일이 있다
# (특히 시뮬레이터를 막 띄운 직후, 2026-11-16 확인: "Unable to solve the planning problem").
# Jazzy 의 OMPL 은 계획 실패를 PLANNING_FAILED 가 아니라 범용 FAILURE(99999) 로 돌려준다 (move_group 로그로 확인).
# 다시 시도는 MoveIt 이 IDLE(실행 중 아님)일 때만 하고, 계획은 항상 현재 자세에서 새로 시작하므로 안전하다.
# INVALID_MOTION_PLAN: 찾은 경로를 촘촘히 다시 검사했더니 중간에 충돌(예: forearm_link ↔ vacuum_pad)이 있어
#   MoveIt 이 실행하지 않고 버린 경우. 로봇은 안 움직였고 다시 계획하면 다른 경로가 나온다 (11-16 확인).
# CONTROL_FAILED(컨트롤러 추종 실패)·NO_IK_SOLUTION(닿을 수 없는 목표)은 다시 해도 같으므로 넣지 않는다.
RETRYABLE_ERRORS = (MoveItErrorCodes.FAILURE, MoveItErrorCodes.PLANNING_FAILED, MoveItErrorCodes.TIMED_OUT,
                    MoveItErrorCodes.INVALID_MOTION_PLAN)

CSV_FIELDS = ['time', 'seq', 'obj', 'part_type', 'result', 'plan_ms', 'error', 'final_x', 'final_y', 'final_z']


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        self.declare_parameter('world_name', 'workcell')
        # 빈 문자열이면 이 패키지의 worlds/<world_name>.sdf. 여기 있는 고정 물체(상자·바닥)를 MoveIt 충돌 물체로 등록한다
        self.declare_parameter('world_file', '')
        self.declare_parameter('max_objects', 3)
        self.declare_parameter('work_plane_x', 0.30)
        self.declare_parameter('work_plane_y', -0.15)
        self.declare_parameter('work_plane_z', 0.0)
        self.declare_parameter('work_plane_yaw_deg', 0.0)
        self.declare_parameter('tool_length_m', 0.05)
        self.declare_parameter('grasp_gap_m', 0.0005)
        self.declare_parameter('grasp_attempts', 3)
        self.declare_parameter('approach_height_m', 0.10)
        self.declare_parameter('lift_height_m', 0.15)
        self.declare_parameter('part_types', ['screw', 'nut'])
        self.declare_parameter('bin_x', [0.0, 0.0])
        self.declare_parameter('bin_y', [0.45, -0.45])
        self.declare_parameter('bin_drop_z', 0.15)
        self.declare_parameter('ready_x_mm', 148.5)        # 대기 자세: 작업 시트 좌표 이 점 (mm, 카메라 시야 밖) 의
        self.declare_parameter('ready_y_mm', -80.0)
        self.declare_parameter('travel_height_m', 0.25)
        # 대기 자세 관절 각도 (rad, UR_JOINTS 순서). 6개면 처음부터 관절 목표로 간다. 시작 자세(팔을 곧게 세움)는
        # 특이 자세라 거기서 역기구학이 자주 실패했다 (11-20). 6개가 아니면 ready_x/y_mm 로 역기구학을 풀어 정한다
        self.declare_parameter('ready_joints', [0.0])    # 이 높이 (컵 끝, 작업 평면 위). 부품 사이 이동 높이도 같음
        self.declare_parameter('settle_s', 3.0)
        self.declare_parameter('max_velocity_scaling', 0.3)
        self.declare_parameter('target_x_mm', 100.0)
        self.declare_parameter('target_y_mm', 100.0)
        self.declare_parameter('target_angle_deg', 0.0)
        self.declare_parameter('target_height_m', 0.25)  # 2주차 이동 확인용 패드 끝 높이
        self.declare_parameter('motion_timeout_s', 30.0)        # 시도 1번당 제한 시간
        self.declare_parameter('planning_time_s', 5.0)          # OMPL 이 길을 찾는 데 쓸 최대 시간 (시도 1번당)
        self.declare_parameter('plan_attempts', 3)              # 계획 실패 시 최대 시도 횟수 (1 = 다시 시도 안 함)
        self.declare_parameter('plan_retry_delay_s', 2.0)       # 다시 시도하기 전 쉬는 시간
        self.declare_parameter('joint_state_wait_s', 10.0)      # 첫 이동 전에 관절 상태를 기다리는 최대 시간
        self.declare_parameter('service_timeout_s', 30.0)       # 스폰·진공 서비스 응답 대기
        self.declare_parameter('log_dir', 'logs')

        # pymoveit2 는 액션 응답을 같은 노드의 콜백으로 받으므로, 서비스 콜백 안에서 기다리려면 재진입 그룹 + 멀티스레드 실행기가 필요
        self.cb_group = ReentrantCallbackGroup()
        self.latest = None   # 마지막으로 받은 패킷 dict
        self.motion_lock = threading.Lock()
        self.motion_failed = False
        self.ready_joints = None   # 대기 자세 관절 각도 (UR_JOINTS 순서). 첫 사이클에 저장
        self.csv_path = None

        self.sub = self.create_subscription(String, '/robot/targets', self.on_targets, 10)
        self.pub_log = self.create_publisher(String, '/sim/pick_log', 10)
        self.srv = self.create_service(Trigger, '/sim/run_cycle', self.on_run_cycle, callback_group=self.cb_group)
        self.move_srv = self.create_service(Trigger, '/sim/move_to_target', self.on_move_to_target,
                                            callback_group=self.cb_group)
        self.ready_srv = self.create_service(Trigger, '/sim/go_ready', self.on_go_ready, callback_group=self.cb_group)
        self.spawn_client = self.create_client(Trigger, '/sim/spawn_objects', callback_group=self.cb_group)
        self.vacuum_client = self.create_client(SetBool, '/sim/vacuum', callback_group=self.cb_group)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.moveit2 = MoveIt2(node=self, joint_names=UR_JOINTS, base_link_name=BASE_LINK,
                               end_effector_name=EE_LINK, group_name=GROUP_NAME,
                               callback_group=self.cb_group, use_move_group_action=True,
                               ignore_new_calls_while_executing=True)
        self.moveit2.max_velocity = self.get_parameter('max_velocity_scaling').value

        share = Path(get_package_share_directory('ur5e_pick_place'))
        self.models_dir = share / 'models'
        # Gazebo 의 상자·바닥을 MoveIt 에도 알려 준다. 모르면 상자 벽을 뚫고 가는 경로를 계획할 수 있다.
        # /collision_object 는 저장되지 않는(volatile) 토픽이라 move_group 이 아직 안 떴을 때 보내면 사라진다.
        # 그래서 move_group 이 구독할 때까지 1초마다 확인하고, 구독이 보이면 두 번 보낸 뒤 멈춘다.
        world_file = self.get_parameter('world_file').value or str(
            share / 'worlds' / f"{self.get_parameter('world_name').value}.sdf")
        self.scene_boxes = collision_boxes(world_file)
        self.scene_sent = 0
        self.scene_timer = self.create_timer(1.0, self.publish_scene, callback_group=self.cb_group)

    # ---------- 공통 ----------

    def value(self, name: str) -> float:
        result = float(self.get_parameter(name).value)
        if not math.isfinite(result):
            raise ValueError(f'{name}은 유한한 값이어야 합니다.')
        return result

    def plane(self) -> PlanePose:
        return PlanePose(*(self.value(f'work_plane_{name}') for name in ('x', 'y', 'z', 'yaw_deg')))

    def publish_scene(self) -> bool:
        """충돌 박스를 MoveIt 에 등록. move_group 이 아직 구독하지 않으면 False. 같은 id 로 다시 보내면 덮어쓴다."""
        if self.count_subscribers('/collision_object') == 0:
            return False
        for box in self.scene_boxes:
            self.moveit2.add_collision_box(id=box.id, size=box.size, position=box.position,
                                           quat_xyzw=box.quat_xyzw, frame_id=BASE_LINK)
        self.scene_sent += 1
        if self.scene_sent == 1:
            self.get_logger().info(f'MoveIt 충돌 물체 {len(self.scene_boxes)}개 등록: '
                                   + ', '.join(box.id for box in self.scene_boxes))
        if self.scene_sent >= 2:
            self.scene_timer.cancel()
        return True

    def on_targets(self, msg: String):
        try:
            packet = json.loads(msg.data)
            if not isinstance(packet, dict) or not isinstance(packet.get('objects'), list):
                raise ValueError('objects 배열이 필요합니다.')
            if not self.motion_lock.locked():
                self.latest = packet
        except (ValueError, TypeError) as exc:
            self.get_logger().warning(f'잘못된 로봇 타깃 패킷: {exc}')

    def prepare_motion(self):
        """이동 전 공통 준비: 설정값 검사, 관절 상태·충돌 물체 확인, MoveIt 속도·계획 시간 설정."""
        if self.motion_failed:
            raise RuntimeError('이전 이동 실패 후 노드를 재시작해야 합니다.')
        timeout, speed, planning = (self.value(n) for n in ('motion_timeout_s', 'max_velocity_scaling', 'planning_time_s'))
        attempts, delay = int(self.get_parameter('plan_attempts').value), self.value('plan_retry_delay_s')
        if self.value('tool_length_m') < 0 or timeout <= 0 or planning <= 0 or not 0 < speed <= 1:
            raise ValueError('툴 길이는 0 이상, 타임아웃·계획 시간은 양수, 속도 비율은 (0, 1]이어야 합니다.')
        if attempts < 1 or delay < 0:
            raise ValueError('plan_attempts는 1 이상, plan_retry_delay_s는 0 이상이어야 합니다.')
        self.wait_for_joint_state(self.value('joint_state_wait_s'))
        if self.scene_sent == 0:
            # 시작 직후 바로 불린 경우: 충돌 물체 없이 계획하면 상자를 뚫을 수 있으므로 먼저 등록하고 반영될 시간을 준다
            if not self.publish_scene():
                raise RuntimeError('move_group 이 아직 /collision_object 를 구독하지 않습니다. 잠시 후 다시 부르세요.')
            time.sleep(1.0)
        self.moveit2.max_velocity = speed
        self.moveit2.allowed_planning_time = planning

    def move_tip(self, x: float, y: float, tip_z: float, yaw_deg: float = 0.0, straight: bool = False) -> int:
        """컵 끝(패드 끝)을 base_link (x, y, tip_z) 로, 툴은 아래를 보고 yaw_deg. 계획 실패면 plan_attempts 번까지 다시.

        straight=True (수직 하강·들어 올림): 먼저 직선(Cartesian) 경로로 간다. OMPL 은 무작위 탐색이라 몇 cm 수직 이동도
        관절이 감긴 자세에서는 10초 안에 못 찾는 일이 있었다 (11-20 실험: 흡착 후 들어 올리기 3회 연속 실패).
        직선 경로가 안 되면 아래 일반 계획으로 넘어간다.
        성공하면 시도 횟수, 실패하면 RuntimeError. prepare_motion() 뒤에 부른다.
        """
        position = (x, y, tip_z + self.value('tool_length_m'))
        timeout = self.value('motion_timeout_s')
        if straight:
            if self.move_once(position, tool_down_quaternion(yaw_deg), timeout, cartesian=True) is None:
                return 1
            if self.motion_failed:
                raise RuntimeError('직선 이동 중 멈추지 않음. 노드를 재시작하세요.')
            self.get_logger().warning('직선 경로 실패, 일반 계획으로 다시 시도')
        # 좌표 목표를 그대로 주면 move_group 이 역기구학 해를 무작위로 골라 관절이 감기거나 멀리 도는 해가 나오고
        # OMPL 이 자주 못 푼다 (11-20 시험: 상자 쪽 ↔ 작업 매트 이동 실패 4회). 현재 관절에서 시작해 역기구학을 풀어
        # 가장 가까운 해를 얻고 관절 목표로 간다. 역기구학이 실패할 때만 좌표 목표로 계획한다.
        joints = self.nearby_ik(position, tool_down_quaternion(yaw_deg))
        if joints is not None:
            return self.move_joints(joints)
        self.get_logger().warning('역기구학 실패, 좌표 목표로 계획')
        return self.retry(lambda: self.move_once(position, tool_down_quaternion(yaw_deg), timeout))

    def current_joints(self) -> list[float]:
        state = self.moveit2.joint_state
        values = dict(zip(state.name, state.position))
        return [values[name] for name in UR_JOINTS]

    def nearby_ik(self, position, quat_xyzw):
        """현재 관절 각도를 시작점으로 tool0 자세의 역기구학을 푼다. 실패하면 None.

        같은 자세라도 관절이 360° 다른 값이 여럿 있어서, 각 관절을 현재 값에 가장 가까운 쪽으로 옮긴다
        (UR5e 는 팔꿈치 ±180°, 나머지 ±360°). 그래야 쓸데없이 한 바퀴 도는 경로가 나오지 않는다.
        """
        seed = self.current_joints()
        result = self.moveit2.compute_ik(position=position, quat_xyzw=quat_xyzw, start_joint_state=seed,
                                         timeout_sec=5.0)
        if result is None:
            return None
        values = dict(zip(result.name, result.position))
        joints = []
        for name, start in zip(UR_JOINTS, seed):
            value = values[name]
            limit = math.pi if name == 'elbow_joint' else 2 * math.pi
            near = value + 2 * math.pi * round((start - value) / (2 * math.pi))
            joints.append(near if abs(near) <= limit else value)
        return joints

    def move_joints(self, positions) -> int:
        """관절 각도 목표 (UR_JOINTS 순서) 로 이동. 계획 실패면 plan_attempts 번까지 다시."""
        timeout = self.value('motion_timeout_s')

        def once():
            started = time.monotonic()
            self.moveit2.move_to_configuration(list(positions), joint_names=UR_JOINTS)
            return self.wait_executed(started, timeout)
        return self.retry(once)

    def retry(self, attempt_once) -> int:
        """attempt_once() (성공 None, 실패 이유) 를 다시 계획해도 되는 실패일 때만 plan_attempts 번까지. 성공하면 시도 횟수."""
        delay, attempts = self.value('plan_retry_delay_s'), int(self.get_parameter('plan_attempts').value)
        for attempt in range(1, attempts + 1):
            error = attempt_once()
            if error is None:
                return attempt
            code = self.moveit2.get_last_execution_error_code()
            retryable = (not self.motion_failed and code is not None and code.val in RETRYABLE_ERRORS)
            if not retryable or attempt == attempts:
                detail = f' [error_code={None if code is None else code.val}, {self.moveit2.last_failure()}]'
                raise RuntimeError(error + detail + (f' ({attempt}회 시도)' if attempt > 1 else ''))
            self.get_logger().warning(f'계획 실패(code {code.val}), {delay:.1f}초 후 다시 시도 {attempt + 1}/{attempts}')
            time.sleep(delay)

    def wait_for_joint_state(self, timeout_s: float):
        """시뮬레이터를 막 띄우면 관절 상태가 아직 안 들어와 있다. 시작 자세를 모르면 계획이 실패하므로 먼저 기다린다."""
        deadline = time.monotonic() + timeout_s
        while self.moveit2.joint_state is None:
            if time.monotonic() > deadline:
                raise RuntimeError(f'{timeout_s:.0f}초 동안 /joint_states 를 받지 못했습니다. 컨트롤러가 켜졌는지 확인하세요.')
            time.sleep(0.1)

    def move_once(self, position, quat_xyzw, timeout_s: float, cartesian: bool = False):
        """계획 + 실행 1번. 성공하면 None, 실패하면 이유 문자열. 실행 중 멈추지 않으면 motion_failed 를 켠다.

        cartesian=True 면 직선 경로이고, 경로의 95 % 이상을 못 이으면 실행하지 않는다.
        """
        started = time.monotonic()
        if not self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, frame_id=BASE_LINK,
                                         cartesian=cartesian, cartesian_fraction_threshold=0.95 if cartesian else 0.0,
                                         timeout_sec=timeout_s):
            return 'MoveIt 이동 요청 실패: move_group·현재 관절 상태를 확인하세요.'
        return self.wait_executed(started, timeout_s)

    def wait_executed(self, started: float, timeout_s: float):
        """보낸 이동이 끝날 때까지 기다린다. 성공 None, 실패 이유. 멈추지 않으면 motion_failed 를 켠다."""
        remaining = max(0.0, timeout_s - (time.monotonic() - started))
        if self.moveit2.wait_until_executed(timeout_sec=remaining, cancel_on_timeout=True):
            return None
        self.motion_failed = self.moveit2.query_state() != MoveIt2State.IDLE
        return ('이동 실패 또는 시간 초과. MoveIt 로그를 확인하세요.'
                + (' 취소 완료 확인 후 노드를 재시작하세요.' if self.motion_failed else ''))

    def call(self, client, request):
        """다른 노드의 서비스를 부르고 응답을 기다린다 (멀티스레드 실행기라 콜백 안에서 기다려도 된다)."""
        timeout = self.value('service_timeout_s')
        if not client.wait_for_service(timeout_sec=min(timeout, 5.0)):
            raise RuntimeError(f'{client.srv_name} 서비스가 없습니다. team2_link.launch.py (스폰 노드) 를 띄웠는지 확인하세요.')
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError(f'{client.srv_name} 응답 시간 초과')
            time.sleep(0.02)
        return future.result()

    def object_xyz(self, name: str, max_age_s: float = 1.0):
        """스폰 노드가 내보내는 TF base_link→obj_k 로 Gazebo 속 부품 위치. 없거나 오래됐으면 None."""
        try:
            t = self.tf_buffer.lookup_transform(BASE_LINK, name, Time())
        except TransformException:
            return None
        age = (self.get_clock().now() - Time.from_msg(t.header.stamp)).nanoseconds / 1e9
        if age > max_age_s:
            return None
        p = t.transform.translation
        return (p.x, p.y, p.z)

    def wait_object_xyz(self, name: str, near=None, tol_m: float = 0.005, timeout_s: float = 5.0):
        """부품 위치가 들어올 때까지 (near 를 주면 그 근처에 올 때까지) 기다린다."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            xyz = self.object_xyz(name)
            if xyz is not None and (near is None or math.dist(xyz[:2], near[:2]) <= tol_m):
                return xyz
            time.sleep(0.1)
        raise RuntimeError(f'{name} 위치를 확인하지 못했습니다'
                           + (f' (기대 위치 {near[0]:.3f}, {near[1]:.3f} 근처)' if near else ''))

    def record(self, row: dict):
        """한 물체 결과를 /sim/pick_log 로 발행하고 CSV 에 한 줄 추가한다."""
        self.pub_log.publish(String(data=json.dumps(row, ensure_ascii=False)))
        if self.csv_path is None:
            log_dir = Path(self.get_parameter('log_dir').value)
            log_dir.mkdir(parents=True, exist_ok=True)
            self.csv_path = log_dir / f'sim_pick_{datetime.now():%Y%m%d_%H%M%S}.csv'
            with open(self.csv_path, 'w', newline='') as f:
                csv.DictWriter(f, CSV_FIELDS).writeheader()
        with open(self.csv_path, 'a', newline='') as f:
            csv.DictWriter(f, CSV_FIELDS, extrasaction='ignore').writerow(dict(time=time.time(), **row))

    # ---------- 2주차: 지정 좌표 이동 ----------

    def on_move_to_target(self, request, response):
        """2주차: 명시적으로 호출했을 때만 지정 좌표로 한 번 이동한다."""
        if not self.motion_lock.acquire(blocking=False):
            response.success, response.message = False, '이미 이동 중입니다.'
            return response
        started = time.monotonic()
        try:
            plane = self.plane()
            x, y, z = work_to_base(self.value('target_x_mm'), self.value('target_y_mm'), plane)
            height = self.value('target_height_m')
            if height < 0:
                raise ValueError('target_height_m은 0 이상이어야 합니다.')
            yaw = plane.yaw_deg + self.value('target_angle_deg')
            self.prepare_motion()
            attempt = self.move_tip(x, y, z + height, yaw)
            response.success = True
            response.message = (f'tool0 base_link(m)={(x, y, z + height + self.value("tool_length_m"))}, '
                                f'yaw={yaw:.2f} deg, 시도 {attempt}회')
        except (ValueError, RuntimeError) as exc:
            response.success, response.message = False, str(exc)
        finally:
            self.motion_lock.release()
        self.pub_log.publish(String(data=json.dumps(dict(
            action='move_to_target', result='ok' if response.success else 'failed',
            plan_ms=(time.monotonic() - started) * 1000, error='' if response.success else response.message))))
        return response

    # ---------- 4주차: 집기 한 사이클 ----------

    def on_run_cycle(self, request, response):
        if not self.motion_lock.acquire(blocking=False):
            response.success, response.message = False, '이미 이동 중입니다.'
            return response
        rows = []
        try:
            packet = self.latest
            if packet is None:
                raise RuntimeError('아직 받은 패킷이 없습니다 (/robot/targets).')
            part_types = list(self.get_parameter('part_types').value)
            bins = dict(zip(part_types, zip(self.get_parameter('bin_x').value, self.get_parameter('bin_y').value)))
            if len(bins) != len(part_types):
                raise ValueError('part_types, bin_x, bin_y 길이가 같아야 합니다.')
            targets, skipped = plan_spawns(packet, self.plane(), part_types,
                                           int(self.get_parameter('max_objects').value))
            if not targets:
                raise RuntimeError('집을 물체가 없습니다' + (f' ({"; ".join(skipped)})' if skipped else ''))
            self.prepare_motion()
            spawned = self.call(self.spawn_client, Trigger.Request())
            if not spawned.success:
                # 스폰 노드는 이전 부품을 지우고 같은 이름으로 바로 다시 만드는데, Gazebo 가 가끔 거부한다
                # (11-20 무작위 시험 5회 중 1회 "같은 이름이 남아 있음"). 잠깐 기다렸다 한 번 더 한다
                self.get_logger().warning(f'스폰 실패, 1초 뒤 다시: {spawned.message}')
                time.sleep(1.0)
                spawned = self.call(self.spawn_client, Trigger.Request())
            if not spawned.success:
                raise RuntimeError(f'스폰 실패: {spawned.message}')
            # 스폰 노드는 자기 최신 패킷으로 스폰하므로, 이 노드가 고른 위치와 같은지 확인한다 (패킷이 그새 바뀌면 다름)
            for t in targets:
                self.wait_object_xyz(t.name, near=(t.x, t.y))
            self.go_ready()
            for i, t in enumerate(targets):
                row = self.pick_one(t, bins[t.part_type], packet.get('seq'), is_last=(i == len(targets) - 1))
                rows.append(row)
                self.record(row)
                if row['result'] != 'ok' or row['error']:   # 실패, 또는 넣었지만 다음 물체로 갈 경유점 이동 실패
                    break
            ok = sum(r['result'] == 'ok' for r in rows)
            response.success = ok == len(targets)
            response.message = (f'seq={packet.get("seq")}: {ok}/{len(targets)} 개 상자 투입. '
                                + ', '.join(f'{r["obj"]}={r["part_type"]}:{r["result"]}' for r in rows)
                                + (f' / 실패: {rows[-1]["error"]}' if rows and rows[-1]['result'] != 'ok' else '')
                                + (f' / 건너뜀: {"; ".join(skipped)}' if skipped else ''))
        except (ValueError, RuntimeError) as exc:
            response.success, response.message = False, str(exc)
        finally:
            self.motion_lock.release()
        self.get_logger().info(response.message)
        return response

    def on_go_ready(self, request, response):
        """집기가 실패해 멈춘 뒤 되돌리기 (GUI [처음부터 다시]): 진공 끄기 → 작업 매트 위면 곧게 들어 올림 → 대기 자세."""
        if not self.motion_lock.acquire(blocking=False):
            response.success, response.message = False, '이미 이동 중입니다.'
            return response
        try:
            self.prepare_motion()
            released = self.call(self.vacuum_client, SetBool.Request(data=False))
            if not released.success:
                raise RuntimeError(f'진공 끄기 실패: {released.message}')
            self.recover_ready()
            response.success, response.message = True, f'대기 자세로 복귀 ({released.message})'
        except (ValueError, RuntimeError, TransformException) as exc:
            response.success, response.message = False, f'대기 자세 복귀 실패: {exc}'
        finally:
            self.motion_lock.release()
        self.get_logger().info(response.message)
        return response

    def recover_ready(self):
        """부품 옆에서 바로 관절 이동을 하면 팔이 부품을 칠 수 있어, 낮은 곳이면 먼저 곧게 올린다 (사이클과 같은 툴 yaw 0)."""
        p = self.tf_buffer.lookup_transform(BASE_LINK, EE_LINK, Time()).transform.translation
        travel = self.value('work_plane_z') + self.value('travel_height_m')
        if needs_lift(p.y, p.z - self.value('tool_length_m'), travel):
            self.move_tip(p.x, p.y, travel, straight=True)
        self.go_ready()

    def go_ready(self):
        """대기 자세로. 처음에는 작업 시트 위 좌표 목표로 가서 그 관절 각도를 저장하고, 다음부터는 관절 목표로 간다."""
        if self.ready_joints is None:
            configured = [float(v) for v in self.get_parameter('ready_joints').value]
            if len(configured) == len(UR_JOINTS):
                self.ready_joints = configured
        if self.ready_joints is not None:
            self.move_joints(self.ready_joints)
            return
        x, y, z = work_to_base(self.value('ready_x_mm'), self.value('ready_y_mm'), self.plane())
        self.move_tip(x, y, z + self.value('travel_height_m'))
        self.ready_joints = self.current_joints()
        self.get_logger().info('대기 자세 저장 (rad): ' + ', '.join(f'{v:.3f}' for v in self.ready_joints))

    def pick_one(self, t, bin_xy, seq, is_last: bool) -> dict:
        """물체 하나: 접근 → 하강·흡착 → 들어 올림 → 상자 위 → 놓기 → 안착 확인. 결과 dict (CSV 한 줄)."""
        started = time.monotonic()
        row = dict(seq=seq, obj=t.name, part_type=t.part_type, result='failed', error='')
        holding = False
        step = '준비'
        try:
            shape = marker_shape(model_file(self.models_dir, t.part_type))
            if shape is None:
                raise RuntimeError(f'{t.part_type} 모델 모양을 읽지 못했습니다.')
            top = self.value('work_plane_z') + part_top_m(shape)
            # 구멍 있는 부품(너트)은 링 컵으로 집는다. 툴 yaw 0 에서 링 컵은 tool0 +X 쪽이라 툴을 반대로 옮긴다
            offset = SuctionLimits().ring_offset if HOLE_RADIUS_M.get(t.part_type, 0.0) > 0 else 0.0
            x, y = cup_tool_xy(t.x, t.y, offset)
            # 대기 자세(작업 매트 위)에서 부품 위 같은 높이로 짧게 옮긴 뒤 접근 높이까지 직선으로 내려간다
            step = '부품 위 높은 곳으로 이동'
            self.move_tip(x, y, self.value('work_plane_z') + self.value('travel_height_m'))
            step = '접근 높이로 하강'
            self.move_tip(x, y, top + self.value('approach_height_m'), straight=True)
            step = '흡착 높이로 하강·진공'
            result = None
            for attempt in range(int(self.get_parameter('grasp_attempts').value)):
                self.move_tip(x, y, top + self.value('grasp_gap_m') - 0.001 * attempt, straight=True)
                result = self.call(self.vacuum_client, SetBool.Request(data=True))
                if result.success or '떨어져' not in result.message:
                    break
            if result is None or not result.success:
                raise RuntimeError(f'흡착 실패: {result.message if result else "시도 안 함"}')
            holding = True
            step = '들어 올림'
            if t.name not in result.message:
                raise RuntimeError(f'다른 부품을 흡착함: {result.message}')
            self.move_tip(x, y, self.value('work_plane_z') + self.value('lift_height_m'), straight=True)
            lifted = self.wait_object_xyz(t.name)
            if lifted[2] < self.value('work_plane_z') + self.value('approach_height_m'):
                raise RuntimeError(f'들어 올렸는데 부품이 따라오지 않음 (z={lifted[2]:.3f} m)')
            bx, by = bin_xy
            step = '상자 위로 이동·놓기'
            self.move_tip(*cup_tool_xy(bx, by, offset), self.value('bin_drop_z'))
            released = self.call(self.vacuum_client, SetBool.Request(data=False))
            if not released.success:
                raise RuntimeError(f'놓기 실패: {released.message}')
            holding = False
            time.sleep(self.value('settle_s'))   # 떨어져서 멈출 때까지
            final = self.wait_object_xyz(t.name)
            row.update(final_x=final[0], final_y=final[1], final_z=final[2])
            if not inside_bin(final, bx, by):
                raise RuntimeError(f'상자 밖에 떨어짐 ({final[0]:.3f}, {final[1]:.3f}, {final[2]:.3f}) m')
            row['result'] = 'ok'
            step = '대기 자세로 복귀'
            try:
                self.go_ready()   # 관절 각도 목표: 상자 쪽 → 작업 매트 위의 큰 이동
            except RuntimeError as exc:
                if not is_last:
                    row['error'] = f'투입 후 대기 자세 복귀 실패 (부품은 상자 안): {exc}'
                    return row | dict(plan_ms=(time.monotonic() - started) * 1000)
                # 마지막 물체는 이미 상자에 들어갔으므로 복귀 실패는 경고만 남긴다 (11-19 기록: 가끔 계획 실패)
                self.get_logger().warning(f'투입 후 복귀 실패 (부품은 상자 안): {exc}')
        except (ValueError, RuntimeError) as exc:
            row['error'] = f'[{step}] {exc}'
            if holding:   # 붙인 채 멈추면 다음 사이클이 꼬이므로 진공을 끈다
                try:
                    self.call(self.vacuum_client, SetBool.Request(data=False))
                except RuntimeError:
                    pass
        row['plan_ms'] = (time.monotonic() - started) * 1000
        return row

    def destroy_node(self):
        self.moveit2.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    # 서비스 콜백이 MoveIt 액션·다른 노드 서비스 응답을 기다리는 동안 그 응답을 처리할 스레드가 더 필요하다
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
