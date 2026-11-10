"""ROS2 구독을 Qt 메인 스레드 밖에서 돌리는 워커 (3주차).

rclpy.spin 은 블로킹이라 GUI 스레드에서 돌리면 화면이 멈춘다. QThread 안에서 spin 하고,
받은 데이터는 Qt Signal 로 메인 스레드에 넘긴다 (위젯은 메인 스레드에서만 건드려야 함).
"""
from PySide6.QtCore import QThread, Signal


class RosWorker(QThread):
    detections = Signal(dict)     # /vision/detections JSON
    frame = Signal(object)        # /vision/debug_image → numpy BGR
    robot_log = Signal(dict)      # /robot/log JSON
    sim_log = Signal(dict)        # /sim/pick_log JSON (Gazebo 를 켰을 때만 들어옴)

    def run(self):
        # TODO: rclpy.init → Node('gui') 생성 → 네 토픽 구독(콜백에서 emit) → while 루프에서 spin_once
        raise NotImplementedError

    def stop(self):
        # TODO: 루프 종료 플래그 → rclpy.try_shutdown → self.wait()
        raise NotImplementedError
