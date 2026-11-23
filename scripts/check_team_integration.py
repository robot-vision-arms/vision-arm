"""Measure an existing vision → ROS → TCP → ACK → ROS pipeline.

Default: 100 packets, no TCP failures, identical target packets and debug image.
Always overwrites the result (including failure); never commands robot motion.
--settle-s explicitly measures after all topic matches; default includes startup.
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
    parser.add_argument('--settle-s', type=float, default=0,
                        help='Explicitly exclude discovery startup; wait this many seconds after all topic matches')
    parser.add_argument('--out', default='logs/team_integration_check.json')
    args = parser.parse_args()
    if (args.count < 1 or not math.isfinite(args.timeout) or args.timeout <= 0
            or not math.isfinite(args.settle_s) or not 0 <= args.settle_s < args.timeout):
        parser.error('count and timeout must be positive; settle-s must be finite and in [0, timeout)')
    started = time.monotonic()
    measured = CommunicationCheck(args.count, not args.skip_image)
    stamp = datetime.now(timezone.utc).isoformat()
    metadata = dict(started_at_utc=stamp, ros_domain_id=os.environ.get('ROS_DOMAIN_ID', '0'),
                    settle_s=args.settle_s, startup_excluded=args.settle_s > 0,
                    warmup_messages=0, warmup_errors=[], boundary_excluded_source_packets=0)
    warmup_sequences = set()
    measuring = args.settle_s == 0
    # 새 검사가 실패·중단돼도 이전 성공 결과가 남지 않도록 시작 상태부터 저장한다.
    save_report(args.out, dict(ok=False, status='running', **metadata))
    node = None
    context_started = False
    interrupted = False
    try:
        rclpy.init(args=[])
        context_started = True
        node = rclpy.create_node('team_integration_observer')
        def receive(kind, msg):
            # 선택한 준비 구간은 측정에서 제외하되, 메시지 수와 오류는 보고서에 남긴다.
            if not measuring:
                metadata['warmup_messages'] += 1
                try:
                    value = json.loads(msg.data)
                    if isinstance(value, dict) and type(value.get('seq')) is int:
                        # 원본뿐 아니라 ACK·목표에서 먼저 본 번호도 준비 구간에 속한 것으로 묶는다.
                        warmup_sequences.add(value['seq'])
                    if kind == 'log' and (value.get('error') or value.get('acked') is not True):
                        metadata['warmup_errors'].append(value)
                except (ValueError, TypeError, AttributeError) as exc:
                    metadata['warmup_errors'].append(str(exc))
                return
            try:
                value = json.loads(msg.data)
                # 토픽별 콜백 순서는 같지 않다. 준비 구간에 ACK만 먼저 관찰한
                # 패킷의 원본이 늦게 도착해도 새 측정 패킷으로 세지 않는다.
                if (isinstance(value, dict) and type(value.get('seq')) is int
                        and value['seq'] in warmup_sequences):
                    if kind == 'packet':
                        metadata['boundary_excluded_source_packets'] += 1
                    return
                measured.receive(kind, value)
            except (ValueError, TypeError) as exc:
                measured.errors.append(f'{kind}: {exc}')
        subscriptions = [
            node.create_subscription(String, topic, lambda msg, kind=kind: receive(kind, msg), 100)
            for topic, kind in [('/vision/detections', 'packet'), ('/robot/targets', 'target'),
                                ('/robot/log', 'log')]]
        def image_received(msg):
            if measuring and msg.width > 0 and msg.height > 0 and msg.data:
                measured.image_received = True
        subscriptions.append(node.create_subscription(Image, '/vision/debug_image', image_received, 1))
        deadline = started+args.timeout
        matched_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if not measuring:
                required = subscriptions[:3] if args.skip_image else subscriptions
                # 토픽별 발견 시점이 다르므로 모든 구독 연결이 준비된 뒤부터 N개를 센다.
                if all(sub.get_publisher_count() > 0 for sub in required):
                    if matched_since is None:
                        matched_since = time.monotonic()
                    if time.monotonic() - matched_since >= args.settle_s:
                        measuring = True
                        metadata['measurement_started_after_s'] = time.monotonic() - started
                else:
                    matched_since = None
                continue
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
