"""ROS2 구독을 Qt 메인 스레드 밖에서 돌리는 워커 (3주차).

rclpy.spin 은 블로킹이라 GUI 스레드에서 돌리면 화면이 멈춘다. QThread 안에서 spin_once 를 돌리고,
받은 데이터는 Qt Signal 로 메인 스레드에 넘긴다 (위젯은 메인 스레드에서만 건드려야 함).
GUI 프로세스 안의 다른 rclpy 사용과 섞이지 않도록 전용 Context 를 쓴다.

구독:   /vision/detections (String JSON), /vision/debug_image (Image), /robot/log (String JSON), /sim/pick_log (String JSON),
        /sim/overview/image (Image, Gazebo 전경 카메라)
서비스: /sim/run_cycle, /sim/spawn_random (Trigger) — 버튼으로 요청할 때만 부른다 (Gazebo 를 켰을 때)
"""
import json
import threading
import time

from PySide6.QtCore import QThread, Signal


class RosWorker(QThread):
    detections = Signal(dict)     # /vision/detections JSON
    frame = Signal(object)        # /vision/debug_image → numpy BGR
    overview = Signal(object)     # /sim/overview/image → numpy BGR (Gazebo 전경 카메라, 시연용)
    robot_log = Signal(dict)      # /robot/log JSON
    sim_log = Signal(dict)        # /sim/pick_log JSON (Gazebo 를 켰을 때만 들어옴)
    cycle_result = Signal(bool, str)   # /sim/run_cycle 응답 (성공, 메시지)
    spawn_result = Signal(bool, str)   # /sim/spawn_random 응답 (성공, 메시지)
    gazebo_ready = Signal(bool)        # /sim/run_cycle 서비스가 보이는지 (1초마다, 헤더의 Gazebo 상태 표시)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = threading.Event()
        self._cycle_requested = threading.Event()
        self._spawn_requested = threading.Event()

    def request_cycle(self):
        """메인 스레드에서 부른다. 다음 루프에서 /sim/run_cycle 을 비동기로 호출한다."""
        self._cycle_requested.set()

    def request_spawn(self):
        """메인 스레드에서 부른다. 다음 루프에서 /sim/spawn_random 을 비동기로 호출한다."""
        self._spawn_requested.set()

    def run(self):
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
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
        node.create_subscription(Image, '/vision/debug_image', image_signal(self.frame), 1)   # 영상은 최신 1장만
        node.create_subscription(Image, '/sim/overview/image', image_signal(self.overview), 1)
        node.create_subscription(String, '/robot/log', json_signal(self.robot_log), 100)
        node.create_subscription(String, '/sim/pick_log', json_signal(self.sim_log), 100)
        cycle = node.create_client(Trigger, '/sim/run_cycle')
        spawn = node.create_client(Trigger, '/sim/spawn_random')
        pending = None
        pending_spawn = None
        last_check = 0.0

        self._running.set()
        try:
            while self._running.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.05)
                now = time.monotonic()
                if now - last_check > 1.0:
                    last_check = now
                    self.gazebo_ready.emit(cycle.service_is_ready())
                if self._cycle_requested.is_set() and pending is None:
                    self._cycle_requested.clear()
                    if cycle.service_is_ready():
                        pending = cycle.call_async(Trigger.Request())
                    else:
                        self.cycle_result.emit(False, '/sim/run_cycle 서비스가 없습니다 (sim.launch.py 를 띄웠는지 확인).')
                if self._spawn_requested.is_set() and pending_spawn is None:
                    self._spawn_requested.clear()
                    if spawn.service_is_ready():
                        pending_spawn = spawn.call_async(Trigger.Request())
                    else:
                        self.spawn_result.emit(False, '/sim/spawn_random 서비스가 없습니다 (스폰 노드·team2_link 확인).')
                if pending_spawn is not None and pending_spawn.done():
                    result = pending_spawn.result()
                    self.spawn_result.emit(bool(result and result.success), result.message if result else '응답 없음')
                    pending_spawn = None
                if pending is not None and pending.done():
                    result = pending.result()
                    self.cycle_result.emit(bool(result and result.success),
                                           result.message if result else '응답 없음')
                    pending = None
        finally:
            executor.shutdown()
            node.destroy_node()
            rclpy.try_shutdown(context=context)

    def stop(self):
        self._running.clear()
        self.wait(3000)
