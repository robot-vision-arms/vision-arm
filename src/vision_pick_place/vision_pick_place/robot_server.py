"""TCP NDJSON server; ACK confirms validation, not robot execution.

ros2 run vision_pick_place robot_server --publish-ros
Default mode uses no ROS. Each received line is bounded to 1 MiB.
"""
import argparse
import csv
from datetime import datetime
import socket
import socketserver
from pathlib import Path
from threading import Lock
import time

from vision_pick_place.packet import decode, encode, make_ack
from vision_pick_place.transport import MAX_LINE, validate_pose


class PoseServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address, on_packet=None, log_dir='logs'):
        path = Path(log_dir)
        path.mkdir(parents=True, exist_ok=True)
        self.log_path = path / f'robot_rx_{datetime.now():%Y%m%d_%H%M%S_%f}.csv'
        self.log_file = self.log_path.open('w', newline='')
        self.writer = csv.DictWriter(self.log_file, fieldnames=['t_received', 'seq', 'status', 'error', 'packet'])
        self.writer.writeheader()
        self.lock = Lock()
        self.clients = set()
        self.stopping = False
        self.on_packet = on_packet
        try:
            super().__init__(address, PoseHandler)
        except Exception:
            self.log_file.close()
            raise

    def record(self, packet, status, error=''):
        with self.lock:
            if self.log_file.closed:
                return
            self.writer.writerow(dict(t_received=time.time(), seq=packet.get('seq') if isinstance(packet, dict) else None,
                                      status=status, error=error, packet=encode(packet).decode().strip()))
            self.log_file.flush()

    def server_close(self):
        # 서버 재시작 뒤 클라이언트가 옛 연결을 계속 사용하지 않도록 기존 접속도 종료한다.
        with self.lock:
            self.stopping = True
            clients = list(self.clients)
        for client in clients:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
        super().server_close()
        with self.lock:
            self.log_file.close()


class PoseHandler(socketserver.StreamRequestHandler):
    timeout = 5

    def setup(self):
        super().setup()
        with self.server.lock:
            if self.server.stopping:
                raise ConnectionAbortedError('Server stopping')
            self.server.clients.add(self.request)

    def finish(self):
        try:
            super().finish()
        finally:
            with self.server.lock:
                self.server.clients.discard(self.request)

    def handle(self):
        try:
            while True:
                # TCP 수신 경계는 패킷 경계가 아니다. 줄바꿈까지 모아 JSON 한 개로 읽고 크기도 제한한다.
                raw = self.rfile.readline(MAX_LINE + 1)
                if not raw:
                    return
                if len(raw) > MAX_LINE or not raw.endswith(b'\n'):
                    return
                packet = None
                try:
                    packet = decode(raw)
                    validate_pose(packet)
                except (ValueError, UnicodeError, TypeError) as exc:
                    self.wfile.write(encode(dict(ack=None, status='error', error=str(exc))))
                    self.server.record(packet, 'invalid', str(exc))
                    continue
                # ACK는 형식 검사를 통과했다는 뜻이다. ROS 전달·스폰·집기 완료를 의미하지 않는다.
                self.wfile.write(encode(make_ack(packet['seq'])))
                self.server.record(packet, 'ok')
                if self.server.on_packet is not None:
                    try:
                        self.server.on_packet(packet)
                        self.server.record(packet, 'forward_published')
                    except Exception as exc:
                        self.server.record(packet, 'forward_error', str(exc))
        except (OSError, ValueError):
            return


def serve(host, port, on_packet=None, log_dir='logs'):
    with PoseServer((host, port), on_packet, log_dir) as server:
        print(f'TCP listening on {host}:{server.server_address[1]}; log={server.log_path}', flush=True)
        server.serve_forever(poll_interval=.1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=9000)
    parser.add_argument('--log-dir', default='logs')
    parser.add_argument('--publish-ros', action='store_true')
    parser.add_argument('--target-wait-s', type=float, default=1.0,
                        help='Wait for ROS target subscriber discovery after TCP ACK')
    args, ros_args = parser.parse_known_args()
    if not 0 < args.target_wait_s <= 30:
        parser.error('--target-wait-s must be in (0, 30]')
    node = context = None
    try:
        publish = None
        if args.publish_ros:
            import rclpy
            from rclpy.context import Context
            from std_msgs.msg import String
            context = Context()
            rclpy.init(args=ros_args, context=context)
            node = rclpy.create_node('robot_server', context=context)
            publisher = node.create_publisher(String, '/robot/targets', 10)
            def publish(packet):
                # 서버 재시작 직후 첫 타깃이 사라지는 것을 줄이기 위해 구독 연결을 제한 시간 동안 기다린다.
                deadline = time.monotonic() + args.target_wait_s
                while publisher.get_subscription_count() == 0:
                    if not context.ok() or time.monotonic() >= deadline:
                        raise TimeoutError('No matched /robot/targets subscriber; target not published')
                    time.sleep(.01)
                publisher.publish(String(data=encode(packet).decode().strip()))
        serve(args.host, args.port, publish, args.log_dir)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if context is not None:
            rclpy.try_shutdown(context=context)


if __name__ == '__main__':
    main()
