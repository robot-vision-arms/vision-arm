"""/vision/detections → TCP ACK → /robot/log and communication CSV.

No automatic packet replay after a failed ACK. The next frame reconnects.
The one-frame subscription queue avoids replaying a backlog after a timeout.
"""
import csv
from datetime import datetime
import json
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from vision_pick_place.transport import PoseClient
from vision_pick_place.clock_measurement import ClockMeasurement


class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')
        for name, default in [('robot_host', '127.0.0.1'), ('robot_port', 9000),
                              ('ack_timeout_s', 1.), ('log_dir', 'logs')]:
            self.declare_parameter(name, default)
        self.client = PoseClient(self.get_parameter('robot_host').value,
                                 self.get_parameter('robot_port').value,
                                 self.get_parameter('ack_timeout_s').value)
        path = Path(self.get_parameter('log_dir').value)
        path.mkdir(parents=True, exist_ok=True)
        self.log_file = (path / f'comm_{datetime.now():%Y%m%d_%H%M%S_%f}.csv').open('w', newline='')
        self.clock = ClockMeasurement()
        fields = ['seq', 'sent', 'acked', 'latency_ms', 'error', 'attempted', 'validated',
                  'connected', 'send_completed', 'stage', 'tcp_roundtrip_ms',
                  'raw_latency_ms', 'latency_valid', 'clock_warning', 'clock_jump_ms',
                  'clock_sample_span_ms', 'clock_scope']
        self.writer = csv.DictWriter(self.log_file, fieldnames=fields)
        self.writer.writeheader()
        self.pub_log = self.create_publisher(String, '/robot/log', 10)
        # 대기열을 1개로 제한해 통신 지연 뒤 오래된 좌표가 연속으로 전달되는 것을 줄인다.
        self.sub = self.create_subscription(String, '/vision/detections', self.on_detections, 1)

    def on_detections(self, msg):
        result = dict(seq=None, sent=False, acked=False, latency_ms=None, error='',
                      attempted=True, validated=False, connected=False, send_completed=False,
                      stage='decoding', tcp_roundtrip_ms=None)
        packet = None
        token = self.clock.begin()
        self.client.last_transaction = {}
        try:
            packet = json.loads(msg.data)
            if isinstance(packet, dict):
                result['seq'] = packet.get('seq')
            # sent는 기존 규약상 '전송 시도'다. send_completed는 로컬 송신 완료, acked는 서버 수신 확인이다.
            result['sent'] = True
            self.client.exchange(packet)
            result['acked'] = True
        except (ValueError, TypeError, OSError) as exc:
            result['error'] = str(exc)
        result.update(self.client.last_transaction)
        capture = packet.get('t_capture') if isinstance(packet, dict) else None
        # WSL 시계 튐으로 계산된 지연을 정상 측정값으로 취급하지 않도록 유효성도 함께 기록한다.
        result.update(self.clock.finish(token, capture))
        if not result['acked']:
            result['latency_ms'] = None
            result['latency_valid'] = False
        # 같은 결과를 GUI 연결용 토픽과 사후 분석용 CSV에 남긴다. 실제 로봇 집기 성공 로그는 아니다.
        self.pub_log.publish(String(data=json.dumps(result, ensure_ascii=False)))
        self.writer.writerow(result)
        self.log_file.flush()

    def destroy_node(self):
        self.client.close()
        self.log_file.close()
        return super().destroy_node()


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
