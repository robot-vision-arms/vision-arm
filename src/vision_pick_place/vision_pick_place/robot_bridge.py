"""ROS2 → TCP 브리지 (3주차): /vision/detections 를 구독해 가상 로봇 TCP 서버로 보내고 ACK 를 받는다.

발행 토픽:
    /robot/log  std_msgs/String  송수신 기록 1건 = {"seq", "sent", "acked", "latency_ms", "error"} (GUI 표시용)
송수신 기록은 logs/comm_YYYYMMDD_HHMMSS.csv 에도 남겨 KPI(송신 성공률, P95 지연) 계산에 쓴다.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')
        self.declare_parameter('robot_host', '127.0.0.1')
        self.declare_parameter('robot_port', 9000)
        self.declare_parameter('ack_timeout_s', 1.0)

        self.sub = self.create_subscription(String, '/vision/detections', self.on_detections, 10)
        self.pub_log = self.create_publisher(String, '/robot/log', 10)
        # TODO: socket 연결 (끊기면 재연결), CSV 로그 파일 열기

    def on_detections(self, msg: String):
        # TODO: packet.encode 로 송신 → ACK 한 줄 수신(timeout) → latency = now - t_capture
        #       → /robot/log 발행 + CSV 기록
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
