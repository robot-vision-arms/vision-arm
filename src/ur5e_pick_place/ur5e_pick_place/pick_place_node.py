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
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger

from ur5e_pick_place.object_spawner import slot_name

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

        # pymoveit2 는 액션 응답을 같은 노드의 콜백으로 받으므로, 서비스 콜백 안에서 기다리려면 재진입 그룹 + 멀티스레드 실행기가 필요
        self.cb_group = ReentrantCallbackGroup()
        self.latest = None   # 마지막으로 받은 패킷 dict

        self.sub = self.create_subscription(String, '/robot/targets', self.on_targets, 10)
        self.pub_log = self.create_publisher(String, '/sim/pick_log', 10)
        n = self.get_parameter('max_objects').value
        self.pub_attach = [self.create_publisher(Empty, f'/vacuum/{slot_name(k)}/attach', 10) for k in range(n)]
        self.pub_detach = [self.create_publisher(Empty, f'/vacuum/{slot_name(k)}/detach', 10) for k in range(n)]
        self.srv = self.create_service(Trigger, '/sim/run_cycle', self.on_run_cycle, callback_group=self.cb_group)

        # TODO: self.moveit2 = pymoveit2.MoveIt2(node=self, joint_names=UR_JOINTS, base_link_name=BASE_LINK,
        #           end_effector_name=EE_LINK, group_name=GROUP_NAME, callback_group=self.cb_group)
        #       max_velocity_scaling 적용
        # TODO: 작업면·상자 벽을 충돌 물체로 등록 (moveit2.add_collision_box) → 계획이 상자 벽을 피해 간다
        # TODO: logs/sim_pick_YYYYMMDD_HHMMSS.csv 열기

    def on_targets(self, msg: String):
        # TODO: json.loads 해서 self.latest 에 저장 (사이클 실행 중이면 무시)
        pass

    def on_run_cycle(self, request, response):
        # TODO: 위 docstring 의 1~4 단계. 좌표·자세는 grasp.work_to_base / nearest_equivalent_yaw_deg / tool_down_quaternion
        #       장축이 없는 종류(너트 등, 검출 단계에서 angle_deg 가 None)는 yaw 를 현재 손목 yaw 그대로 둔다
        response.success = False
        response.message = 'not implemented'
        return response


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
