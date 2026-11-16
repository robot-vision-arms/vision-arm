"""Check a running integration.launch.py without commanding robot motion.

Checks actual ROS → TCP → ACK → ROS targets.
Run in another terminal, using the same ROS_DOMAIN_ID as the launch.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=20)
    parser.add_argument('--out', default='logs/team_integration_check.json')
    args = parser.parse_args()
    rclpy.init(args=[])
    node = rclpy.create_node('team_integration_observer')
    packets, targets, logs, frames = {}, {}, [], []
    def packet_callback(msg, store):
        packet = json.loads(msg.data)
        store[packet['seq']] = packet
    subscriptions = [
        node.create_subscription(String, '/vision/detections', lambda msg: packet_callback(msg, packets), 10),
        node.create_subscription(String, '/robot/targets', lambda msg: packet_callback(msg, targets), 10),
        node.create_subscription(String, '/robot/log', lambda msg: logs.append(json.loads(msg.data)), 10),
        node.create_subscription(Image, '/vision/debug_image', lambda msg: frames.append(msg.width), 1),
    ]
    report = {}
    try:
        deadline = time.monotonic()+args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            matched = set(packets) & set(targets)
            acked = {entry['seq'] for entry in logs if entry['acked']}
            delivered = matched & acked
            if len(delivered) >= 5 and frames:
                assert all(packets[seq] == targets[seq] for seq in delivered)
                report = dict(ok=True, identical_packets=len(delivered),
                              image_received=True,
                              tcp_errors=[e for e in logs if e['error']],
                              calib_ok_values=sorted({targets[s]['calib_ok'] for s in delivered}),
                              robot_motion_commanded=False)
                break
        if not report:
            raise RuntimeError(f'Integration timeout: packets={len(packets)}, targets={len(targets)}, '
                               f'logs={len(logs)}, frames={len(frames)}')
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
