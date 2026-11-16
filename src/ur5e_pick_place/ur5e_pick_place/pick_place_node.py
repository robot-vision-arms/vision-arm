"""Gazebo UR5e 픽앤플레이스 노드 (2~4주차).

robot_server --publish-ros 가 TCP 로 받은 패킷을 /robot/targets 로 다시 내보내면, 이 노드는 최신 패킷만 들고 있다가
/sim/run_cycle 서비스가 호출될 때 한 사이클을 실행한다. 10 Hz 로 들어오는 패킷마다 움직이면 로봇이 쉬지 않으므로
'패킷 한 장 → 집기 한 사이클' 로 끊는다.

한 사이클:
    1. 최신 패킷 확인 (calib_ok 가 false 면 거부), part_types 에 없는 종류(unknown 포함)는 빼고 max_objects 개까지 자르기
    2. object_spawner 로 같은 위치·각도에 가상 나사/너트 스폰 → 곧바로 각 슬롯 detach
       (DetachableJoint 는 child 모델이 나타나면 바로 붙이므로, 스폰 직후 떼어 둬야 물체가 제자리에 남는다)
    3. 물체마다: 접근(approach_height_m 위) → 하강(grasp_gap_m 까지) → attach → 상승
                 → part_type 의 상자(bin_x/bin_y 같은 칸) 위 → detach → 다음 물체
    4. 결과를 /sim/pick_log 로 발행 (GUI ③) + logs/sim_pick_*.csv 기록 (KPI 의 Pick 성공률)

구독:   /robot/targets  std_msgs/String   packet 형식 JSON
발행:   /sim/pick_log   std_msgs/String   {"seq", "obj", "part_type", "result", "plan_ms", "error"}
        /vacuum/obj_<k>/attach, /vacuum/obj_<k>/detach  std_msgs/Empty  (ros_gz_bridge 가 Gazebo 로 전달)
서비스: /sim/run_cycle  std_srvs/Trigger

    ros2 service call /sim/run_cycle std_srvs/srv/Trigger

MoveIt 은 pymoveit2 (move_group 의 액션·서비스를 부르는 Python 클라이언트) 로 쓴다.
move_group 은 sim.launch.py 가 ur_moveit_config 로 띄운다.
"""
import json
import math
import os
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import MoveItErrorCodes
from pymoveit2 import MoveIt2, MoveIt2State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from ur5e_pick_place.object_spawner import slot_name
from ur5e_pick_place.grasp import PlanePose, tool_down_quaternion, work_to_base
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
        self.declare_parameter('grasp_gap_m', 0.002)
        self.declare_parameter('approach_height_m', 0.10)
        self.declare_parameter('part_types', ['screw', 'nut'])
        self.declare_parameter('bin_x', [0.0, 0.0])
        self.declare_parameter('bin_y', [0.45, -0.45])
        self.declare_parameter('bin_drop_z', 0.15)
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

        # pymoveit2 는 액션 응답을 같은 노드의 콜백으로 받으므로, 서비스 콜백 안에서 기다리려면 재진입 그룹 + 멀티스레드 실행기가 필요
        self.cb_group = ReentrantCallbackGroup()
        self.latest = None   # 마지막으로 받은 패킷 dict
        self.motion_lock = threading.Lock()
        self.motion_failed = False

        self.sub = self.create_subscription(String, '/robot/targets', self.on_targets, 10)
        self.pub_log = self.create_publisher(String, '/sim/pick_log', 10)
        n = self.get_parameter('max_objects').value
        self.pub_attach = [self.create_publisher(Empty, f'/vacuum/{slot_name(k)}/attach', 10) for k in range(n)]
        self.pub_detach = [self.create_publisher(Empty, f'/vacuum/{slot_name(k)}/detach', 10) for k in range(n)]
        self.srv = self.create_service(Trigger, '/sim/run_cycle', self.on_run_cycle, callback_group=self.cb_group)
        self.move_srv = self.create_service(Trigger, '/sim/move_to_target', self.on_move_to_target,
                                            callback_group=self.cb_group)

        self.moveit2 = MoveIt2(node=self, joint_names=UR_JOINTS, base_link_name=BASE_LINK,
                               end_effector_name=EE_LINK, group_name=GROUP_NAME,
                               callback_group=self.cb_group, use_move_group_action=True,
                               ignore_new_calls_while_executing=True)
        self.moveit2.max_velocity = self.get_parameter('max_velocity_scaling').value

        # Gazebo 의 상자·바닥을 MoveIt 에도 알려 준다. 모르면 상자 벽을 뚫고 가는 경로를 계획할 수 있다.
        # /collision_object 는 저장되지 않는(volatile) 토픽이라 move_group 이 아직 안 떴을 때 보내면 사라진다.
        # 그래서 move_group 이 구독할 때까지 1초마다 확인하고, 구독이 보이면 두 번 보낸 뒤 멈춘다.
        world_file = self.get_parameter('world_file').value or os.path.join(
            get_package_share_directory('ur5e_pick_place'), 'worlds', f"{self.get_parameter('world_name').value}.sdf")
        self.scene_boxes = collision_boxes(world_file)
        self.scene_sent = 0
        self.scene_timer = self.create_timer(1.0, self.publish_scene, callback_group=self.cb_group)
        # TODO: logs/sim_pick_YYYYMMDD_HHMMSS.csv 열기

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

    def on_move_to_target(self, request, response):
        """2주차: 명시적으로 호출했을 때만 지정 좌표로 한 번 이동한다."""
        if not self.motion_lock.acquire(blocking=False):
            response.success, response.message = False, '이미 이동 중입니다.'
            return response
        started = time.monotonic()
        try:
            if self.motion_failed:
                raise RuntimeError('이전 이동 실패 후 노드를 재시작해야 합니다.')
            def value(name):
                result = float(self.get_parameter(name).value)
                if not math.isfinite(result):
                    raise ValueError(f'{name}은 유한한 값이어야 합니다.')
                return result
            plane = PlanePose(*(value(f'work_plane_{name}') for name in ('x', 'y', 'z', 'yaw_deg')))
            x, y, z = work_to_base(value('target_x_mm'), value('target_y_mm'), plane)
            height, tool_length = value('target_height_m'), value('tool_length_m')
            timeout, speed = value('motion_timeout_s'), value('max_velocity_scaling')
            if height < 0 or tool_length < 0 or timeout <= 0 or not 0 < speed <= 1:
                raise ValueError('높이·툴 길이는 0 이상, 타임아웃은 양수, 속도 비율은 (0, 1]이어야 합니다.')
            yaw = plane.yaw_deg + value('target_angle_deg')
            position = (x, y, z + height + tool_length)
            attempts, delay = int(self.get_parameter('plan_attempts').value), value('plan_retry_delay_s')
            if attempts < 1 or delay < 0:
                raise ValueError('plan_attempts는 1 이상, plan_retry_delay_s는 0 이상이어야 합니다.')
            self.wait_for_joint_state(value('joint_state_wait_s'))
            if self.scene_sent == 0:
                # 시작 직후 바로 불린 경우: 충돌 물체 없이 계획하면 상자를 뚫을 수 있으므로 먼저 등록하고 반영될 시간을 준다
                if not self.publish_scene():
                    raise RuntimeError('move_group 이 아직 /collision_object 를 구독하지 않습니다. 잠시 후 다시 부르세요.')
                time.sleep(1.0)
            planning_time = value('planning_time_s')
            if planning_time <= 0:
                raise ValueError('planning_time_s는 양수여야 합니다.')
            self.moveit2.max_velocity = speed
            self.moveit2.allowed_planning_time = planning_time
            for attempt in range(1, attempts + 1):
                error = self.move_once(position, tool_down_quaternion(yaw), timeout)
                if error is None:
                    break
                code = self.moveit2.get_last_execution_error_code()
                retryable = (not self.motion_failed and code is not None and code.val in RETRYABLE_ERRORS)
                if not retryable or attempt == attempts:
                    detail = f' [error_code={None if code is None else code.val}, {self.moveit2.last_failure()}]'
                    raise RuntimeError(error + detail + (f' ({attempt}회 시도)' if attempt > 1 else ''))
                self.get_logger().warning(f'계획 실패(code {code.val}), {delay:.1f}초 후 다시 시도 {attempt + 1}/{attempts}')
                time.sleep(delay)
            response.success = True
            response.message = f'tool0 base_link(m)={position}, yaw={yaw:.2f} deg, 시도 {attempt}회'
        except (ValueError, RuntimeError) as exc:
            response.success, response.message = False, str(exc)
        finally:
            self.motion_lock.release()
        self.pub_log.publish(String(data=json.dumps(dict(
            action='move_to_target', result='ok' if response.success else 'failed',
            plan_ms=(time.monotonic() - started) * 1000, error='' if response.success else response.message))))
        return response

    def wait_for_joint_state(self, timeout_s: float):
        """시뮬레이터를 막 띄우면 관절 상태가 아직 안 들어와 있다. 시작 자세를 모르면 계획이 실패하므로 먼저 기다린다."""
        deadline = time.monotonic() + timeout_s
        while self.moveit2.joint_state is None:
            if time.monotonic() > deadline:
                raise RuntimeError(f'{timeout_s:.0f}초 동안 /joint_states 를 받지 못했습니다. 컨트롤러가 켜졌는지 확인하세요.')
            time.sleep(0.1)

    def move_once(self, position, quat_xyzw, timeout_s: float):
        """계획 + 실행 1번. 성공하면 None, 실패하면 이유 문자열. 실행 중 멈추지 않으면 motion_failed 를 켠다."""
        started = time.monotonic()
        if not self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw,
                                         frame_id=BASE_LINK, timeout_sec=timeout_s):
            return 'MoveIt 이동 요청 실패: move_group·현재 관절 상태를 확인하세요.'
        remaining = max(0.0, timeout_s - (time.monotonic() - started))
        if self.moveit2.wait_until_executed(timeout_sec=remaining, cancel_on_timeout=True):
            return None
        self.motion_failed = self.moveit2.query_state() != MoveIt2State.IDLE
        return ('이동 실패 또는 시간 초과. MoveIt 로그를 확인하세요.'
                + (' 취소 완료 확인 후 노드를 재시작하세요.' if self.motion_failed else ''))

    def on_run_cycle(self, request, response):
        # TODO: 위 docstring 의 1~4 단계. 좌표·자세는 grasp.work_to_base / nearest_equivalent_yaw_deg / tool_down_quaternion
        #       장축이 없는 종류(너트 등, 검출 단계에서 angle_deg 가 None)는 yaw 를 현재 손목 yaw 그대로 둔다
        response.success = False
        response.message = '전체 집기 사이클은 4주차 구현 대상입니다. 2주차 이동 확인은 /sim/move_to_target을 사용하세요.'
        return response

    def destroy_node(self):
        self.moveit2.destroy()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    executor = MultiThreadedExecutor(num_threads=2)
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
