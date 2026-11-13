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
import threading
import time

import rclpy
from pymoveit2 import MoveIt2, MoveIt2State
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from ur5e_pick_place.object_spawner import slot_name
from ur5e_pick_place.grasp import PlanePose, tool_down_quaternion, work_to_base

# ur_moveit_config 의 계획 그룹·관절 이름 (1주차에 RViz MotionPlanning 패널에서 그룹 이름 확인)
UR_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
             'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
GROUP_NAME = 'ur_manipulator'
BASE_LINK = 'base_link'
EE_LINK = 'tool0'   # MoveIt 은 흡착 패드를 모르므로 tool0 목표 = 패드 끝 목표 + tool_length_m


class PickPlaceNode(Node):
    def __init__(self):
        super().__init__('pick_place_node')
        self.declare_parameter('world_name', 'workcell')
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
        self.declare_parameter('motion_timeout_s', 30.0)

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
        self.moveit2.allowed_planning_time = 5.0
        # TODO: 작업면·상자 벽을 충돌 물체로 등록 (moveit2.add_collision_box) → 계획이 상자 벽을 피해 간다
        # TODO: logs/sim_pick_YYYYMMDD_HHMMSS.csv 열기

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
            self.moveit2.max_velocity = speed
            submitted = self.moveit2.move_to_pose(position=position, quat_xyzw=tool_down_quaternion(yaw),
                                                 frame_id=BASE_LINK, timeout_sec=timeout)
            if not submitted:
                raise RuntimeError('MoveIt 이동 요청 실패: move_group·현재 관절 상태를 확인하세요.')
            remaining = max(0.0, timeout - (time.monotonic() - started))
            if not self.moveit2.wait_until_executed(timeout_sec=remaining, cancel_on_timeout=True):
                self.motion_failed = self.moveit2.query_state() != MoveIt2State.IDLE
                raise RuntimeError('이동 실패 또는 시간 초과. MoveIt 로그를 확인하세요.'
                                   + (' 취소 완료 확인 후 노드를 재시작하세요.' if self.motion_failed else ''))
            response.success = True
            response.message = f'tool0 base_link(m)={position}, yaw={yaw:.2f} deg'
        except (ValueError, RuntimeError) as exc:
            response.success, response.message = False, str(exc)
        finally:
            self.motion_lock.release()
        self.pub_log.publish(String(data=json.dumps(dict(
            action='move_to_target', result='ok' if response.success else 'failed',
            plan_ms=(time.monotonic() - started) * 1000, error='' if response.success else response.message))))
        return response

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
