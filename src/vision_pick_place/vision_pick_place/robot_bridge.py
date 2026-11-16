"""/vision/detections → TCP ACK → /robot/log and communication CSV.

No automatic packet replay after a failed ACK. The next frame reconnects.
The one-frame subscription queue avoids replaying a backlog after a timeout.
"""
import csv
from datetime import datetime
import json
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from vision_pick_place.transport import PoseClient


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
        self.writer = csv.DictWriter(self.log_file, fieldnames=['seq', 'sent', 'acked', 'latency_ms', 'error'])
        self.writer.writeheader()
        self.pub_log = self.create_publisher(String, '/robot/log', 10)
        self.sub = self.create_subscription(String, '/vision/detections', self.on_detections, 1)

    def on_detections(self, msg):
        result = dict(seq=None, sent=False, acked=False, latency_ms=None, error='')
        try:
            packet = json.loads(msg.data)
            if isinstance(packet, dict):
                result['seq'] = packet.get('seq')
            # sent denotes an attempted transaction; success is acked.
            result['sent'] = True
            self.client.exchange(packet)
            result['acked'] = True
            result['latency_ms'] = (time.time()-packet['t_capture'])*1000
        except (ValueError, TypeError, OSError) as exc:
            result['error'] = str(exc)
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
