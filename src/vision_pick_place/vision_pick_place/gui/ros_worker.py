"""ROS2 구독을 Qt 메인 스레드 밖에서 돌리는 워커 (3주차).

rclpy.spin 은 블로킹이라 GUI 스레드에서 돌리면 화면이 멈춘다. QThread 안에서 spin_once 를 돌리고,
받은 데이터는 Qt Signal 로 메인 스레드에 넘긴다 (위젯은 메인 스레드에서만 건드려야 함).
GUI 프로세스 안의 다른 rclpy 사용과 섞이지 않도록 전용 Context 를 쓴다.

구독:   /vision/info (String JSON, 검출 방식), /vision/detections (String JSON), /vision/debug_image (Image), /robot/log (String JSON), /sim/pick_log (String JSON),
        /sim/overview/image (Image, Gazebo 전경 카메라), /sim/camera/image (Image, Gazebo 위 카메라, sim_top=True 일 때만)
서비스: SERVICES 의 Trigger 서비스 — 메인 스레드가 request_service(이름) 로 요청할 때만 부른다 (Gazebo 를 켰을 때)
"""
import json
import queue
import threading
import time

from PySide6.QtCore import QThread, Signal

SERVICES = ('/sim/run_cycle',       # 집기 한 사이클 (pick_place_node)
            '/sim/spawn_objects',   # 최신 /robot/targets 대로 Gazebo 에 부품 (실물 → Gazebo 반영)
            '/sim/clear_objects',   # Gazebo 부품 지우기 (실물에서 다 치웠을 때)
            '/sim/spawn_random',    # 시뮬 시연용 무작위 배치
            '/sim/go_ready')        # 진공 끄고 대기 자세로 ([처음부터 다시])


class RosWorker(QThread):
    detections = Signal(dict)     # /vision/detections JSON
    vision_info = Signal(dict)    # /vision/info JSON (검출 방식)
    frame = Signal(object)        # /vision/debug_image → numpy BGR
    overview = Signal(object)     # /sim/overview/image → numpy BGR (Gazebo 전경 카메라, 시연용)
    sim_top = Signal(object)      # /sim/camera/image → numpy BGR (Gazebo 위 카메라 원본, 실물 모드에서 Gazebo 매트를 위에서 보기)
    robot_log = Signal(dict)      # /robot/log JSON
    sim_log = Signal(dict)        # /sim/pick_log JSON (Gazebo 를 켰을 때만 들어옴)
    service_result = Signal(str, bool, str)   # (서비스 이름, 성공, 메시지)
    gazebo_ready = Signal(bool)        # /sim/run_cycle 서비스가 보이는지 (1초마다, 헤더의 Gazebo 상태 표시)

    def __init__(self, parent=None, sim_top: bool = False):
        super().__init__(parent)
        self.subscribe_sim_top = sim_top
        self._running = threading.Event()
        self._requests = queue.Queue()

    def request_service(self, name: str):
        """메인 스레드에서 부른다. 다음 루프에서 서비스를 비동기로 호출하고 결과를 service_result 로 알린다."""
        if name not in SERVICES:
            raise ValueError(f'모르는 서비스: {name}')
        self._requests.put(name)

    def run(self):
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import Image
        from std_msgs.msg import String
        from std_srvs.srv import Trigger

        context = Context()
        rclpy.init(context=context)
        node = rclpy.create_node('gui', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        bridge = CvBridge()

        def json_signal(signal):
            def callback(msg):
                try:
                    value = json.loads(msg.data)
                except json.JSONDecodeError:
                    return
                if isinstance(value, dict):
                    signal.emit(value)
            return callback

        def image_signal(signal):
            def callback(msg):
                try:
                    signal.emit(bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'))
                except Exception:   # 잘못된 인코딩 한 장 때문에 GUI 가 멈추지 않게
                    pass
            return callback

        node.create_subscription(String, '/vision/detections', json_signal(self.detections), 10)
        node.create_subscription(String, '/vision/info', json_signal(self.vision_info),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Image, '/vision/debug_image', image_signal(self.frame), 1)   # 영상은 최신 1장만
        node.create_subscription(Image, '/sim/overview/image', image_signal(self.overview), 1)
        if self.subscribe_sim_top:
            node.create_subscription(Image, '/sim/camera/image', image_signal(self.sim_top), 1)
        node.create_subscription(String, '/robot/log', json_signal(self.robot_log), 100)
        node.create_subscription(String, '/sim/pick_log', json_signal(self.sim_log), 100)
        clients = {name: node.create_client(Trigger, name) for name in SERVICES}
        pending = {}   # 이름 → future (같은 서비스는 한 번에 하나만)
        last_check = 0.0

        self._running.set()
        try:
            while self._running.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.05)
                now = time.monotonic()
                if now - last_check > 1.0:
                    last_check = now
                    self.gazebo_ready.emit(clients['/sim/run_cycle'].service_is_ready())
                while not self._requests.empty():
                    name = self._requests.get_nowait()
                    if name in pending:
                        continue
                    if clients[name].service_is_ready():
                        pending[name] = clients[name].call_async(Trigger.Request())
                    else:
                        self.service_result.emit(name, False, f'{name} 서비스가 없습니다 (sim.launch.py·team2_link 확인).')
                for name, future in list(pending.items()):
                    if future.done():
                        result = future.result()
                        self.service_result.emit(name, bool(result and result.success),
                                                 result.message if result else '응답 없음')
                        del pending[name]
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.try_shutdown(context=context)

    def stop(self):
        self._running.clear()
        self.wait(3000)
