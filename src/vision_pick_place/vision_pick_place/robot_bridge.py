"""/vision/detections → TCP ACK → /robot/log and communication CSV.

No automatic packet replay after a failed ACK. The next frame reconnects.
Bounded receive/worker queues separate ROS reception from blocking TCP ACK waits.
Expired and overflowed frames are explicitly recorded instead of silently lost.
"""
import csv
from datetime import datetime
import json
import queue
import threading
import time
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
                              ('ack_timeout_s', 1.), ('log_dir', 'logs'),
                              ('receive_queue_depth', 32), ('max_queue_age_s', 2.0)]:
            self.declare_parameter(name, default)
        depth = self.get_parameter('receive_queue_depth').value
        self.max_queue_age = self.get_parameter('max_queue_age_s').value
        if not 1 <= depth <= 1024 or not 0 < self.max_queue_age <= 10:
            raise ValueError('receive_queue_depth: 1..1024; max_queue_age_s: (0,10]')
        self.pending = queue.Queue(maxsize=depth)
        self.stopping = threading.Event()
        self.record_lock = threading.Lock()
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
                  'clock_sample_span_ms', 'clock_scope', 'queue_wait_ms', 'processing_ms']
        self.writer = csv.DictWriter(self.log_file, fieldnames=fields)
        self.writer.writeheader()
        self.pub_log = self.create_publisher(String, '/robot/log', 10)
        # ROS 수신 콜백에서 TCP ACK를 기다리지 않는다. 짧은 실행 지연은 제한된 큐로 흡수한다.
        self.sub = self.create_subscription(String, '/vision/detections', self.on_detections, depth)
        self.worker = threading.Thread(target=self.work, name='team2_tcp_sender', daemon=True)
        self.worker.start()

    def on_detections(self, msg):
        if self.stopping.is_set():
            return
        item = (msg, time.monotonic())
        try:
            self.pending.put_nowait(item)
        except queue.Full:
            try:
                old, received = self.pending.get_nowait()
            except queue.Empty:
                pass  # 워커가 방금 빈자리를 만든 경우
            else:
                self.drop(old, received, 'queue_overflow')
            self.pending.put_nowait(item)

    def work(self):
        while not self.stopping.is_set():
            try:
                msg, received = self.pending.get(timeout=0.05)
            except queue.Empty:
                continue
            if time.monotonic() - received > self.max_queue_age:
                self.drop(msg, received, 'queue_expired')
            elif not self.stopping.is_set():
                self.process(msg, received)

    def drop(self, msg, received, stage):
        try:
            packet = json.loads(msg.data)
            seq = packet.get('seq') if isinstance(packet, dict) else None
        except (ValueError, TypeError):
            seq = None
        self.record(dict(seq=seq, sent=False, attempted=False, acked=False,
                         validated=False, connected=False, send_completed=False,
                         stage=stage, error=stage, latency_ms=None, latency_valid=False,
                         queue_wait_ms=(time.monotonic()-received)*1000))

    def record(self, result):
        # 수신 측의 초과 보고와 송신 워커의 ACK 기록이 같은 CSV를 동시에 쓰지 않게 한다.
        with self.record_lock:
            self.pub_log.publish(String(data=json.dumps(result, ensure_ascii=False)))
            self.writer.writerow(result)
            self.log_file.flush()

    def process(self, msg, received):
        processing_started = time.monotonic()
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
        result['queue_wait_ms'] = (processing_started-received)*1000
        result['processing_ms'] = (time.monotonic()-processing_started)*1000
        self.record(result)

    def destroy_node(self):
        self.stopping.set()
        self.worker.join(timeout=float(self.get_parameter('ack_timeout_s').value) + 2.0)
        if self.worker.is_alive():
            raise RuntimeError('TCP worker did not stop; refusing to close its log while active')
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
