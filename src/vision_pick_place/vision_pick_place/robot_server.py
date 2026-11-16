"""TCP NDJSON server; ACK confirms validation, not robot execution.

ros2 run vision_pick_place robot_server --publish-ros
Default mode uses no ROS. Each received line is bounded to 1 MiB.
"""
import argparse
import csv
from datetime import datetime
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
        self.on_packet = on_packet
        try:
            super().__init__(address, PoseHandler)
        except Exception:
            self.log_file.close()
            raise

    def record(self, packet, status, error=''):
        with self.lock:
            self.writer.writerow(dict(t_received=time.time(), seq=packet.get('seq') if isinstance(packet, dict) else None,
                                      status=status, error=error, packet=encode(packet).decode().strip()))
            self.log_file.flush()

    def server_close(self):
        super().server_close()
        with self.lock:
            self.log_file.close()


class PoseHandler(socketserver.StreamRequestHandler):
    timeout = 5

    def handle(self):
        try:
            while True:
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
                self.wfile.write(encode(make_ack(packet['seq'])))
                self.server.record(packet, 'ok')
                if self.server.on_packet is not None:
                    try:
                        self.server.on_packet(packet)
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
    args, ros_args = parser.parse_known_args()
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
            publish = lambda packet: publisher.publish(String(data=encode(packet).decode().strip()))
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
