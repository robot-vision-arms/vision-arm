"""Measure an existing vision → ROS → TCP → ACK → ROS pipeline.

Default: 100 packets, no TCP failures, identical target packets and debug image.
Always overwrites the result (including failure); never commands robot motion.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import tempfile
import time

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from vision_pick_place.communication_check import CommunicationCheck


def save_report(path, report):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=path.parent, delete=False) as stream:
        temporary = Path(stream.name)
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    os.replace(temporary, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--count', type=int, default=100)
    parser.add_argument('--timeout', type=float, default=40)
    parser.add_argument('--skip-image', action='store_true', help='Transport-only test without image source')
    parser.add_argument('--out', default='logs/team_integration_check.json')
    args = parser.parse_args()
    if args.count < 1 or not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('count and timeout must be positive')
    started = time.monotonic()
    measured = CommunicationCheck(args.count, not args.skip_image)
    stamp = datetime.now(timezone.utc).isoformat()
    metadata = dict(started_at_utc=stamp, ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'))
    # A fresh run must never leave a previous PASS looking like its result.
    save_report(args.out, dict(ok=False, status='running', **metadata))
    node = None
    context_started = False
    interrupted = False
    try:
        rclpy.init(args=[])
        context_started = True
        node = rclpy.create_node('team_integration_observer')
        def receive(kind, msg):
            try:
                measured.receive(kind, json.loads(msg.data))
            except (ValueError, TypeError) as exc:
                measured.errors.append(f'{kind}: {exc}')
        subscriptions = [
            node.create_subscription(String, topic, lambda msg, kind=kind: receive(kind, msg), 100)
            for topic, kind in [('/vision/detections', 'packet'), ('/robot/targets', 'target'),
                                ('/robot/log', 'log')]]
        def image_received(msg):
            if msg.width > 0 and msg.height > 0 and msg.data:
                measured.image_received = True
        subscriptions.append(node.create_subscription(Image, '/vision/debug_image', image_received, 1))
        deadline = started+args.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            report = measured.report()
            if report['observed_packets'] == args.count:
                done = not report['missing_acks'] and not report['missing_or_changed_targets']
                if done and (measured.image_received or args.skip_image):
                    break
        else:
            measured.errors.append('Timed out before all required observations completed')
    except KeyboardInterrupt:
        interrupted = True
        measured.errors.append('Interrupted')
    except Exception as exc:
        measured.errors.append(f'{type(exc).__name__}: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if context_started:
            rclpy.try_shutdown()
        report = dict(measured.report(), **metadata, elapsed_s=time.monotonic()-started)
        report['status'] = 'passed' if report['ok'] else 'failed'
        save_report(args.out, report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 130 if interrupted else (0 if report['ok'] else 1)


if __name__ == '__main__':
    raise SystemExit(main())
