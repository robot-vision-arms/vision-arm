"""Isolated team-2-only 100-packet ROS/TCP check plus actual server restart.

Synthetic calibrated poses test serialization, NOT real calibration accuracy.
Starts only bridge/server/test nodes; does not start or command any robot.
python scripts/check_team2_roundtrip.py --domain 92
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

from check_team_integration import save_report
from vision_pick_place.communication_check import CommunicationCheck


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain', type=int, default=192)
    parser.add_argument('--out', default='logs/team2_roundtrip_check.json')
    args = parser.parse_args()
    if not 1 <= args.domain <= 232:
        parser.error('Use an isolated ROS domain in 1..232')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    import rclpy
    from std_msgs.msg import String
    with socket.create_server(('127.0.0.1', 0)) as reserve:
        port = reserve.getsockname()[1]
    output = Path(args.out)
    log_dir = output.parent / f'team2_roundtrip_{datetime.now():%Y%m%d_%H%M%S_%f}'
    log_dir.mkdir(parents=True, exist_ok=True)
    children = []
    node = None
    measured = CommunicationCheck(100, require_image=False)
    targets, logs = {}, {}
    report = dict(ok=False, status='running', synthetic=True, ros_domain_id=args.domain,
                  robot_motion_commanded=False, log_dir=str(log_dir))
    save_report(output, report)
    process_log = (log_dir/'process.log').open('w')
    phase = 'setup'

    def start(module, *extra):
        child = subprocess.Popen([sys.executable, '-m', module, *extra],
                                 stdout=process_log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(child)
        return child

    def stop(child):
        if child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.wait()

    def start_server():
        return start('vision_pick_place.robot_server', '--port', str(port), '--publish-ros',
                     '--log-dir', str(log_dir))

    try:
        rclpy.init(args=[])
        node = rclpy.create_node('team2_roundtrip_probe')
        def receive(kind, msg):
            value = json.loads(msg.data)
            if kind == 'target':
                targets[value['seq']] = value
            else:
                logs[value['seq']] = value
            if phase == 'normal':
                measured.receive(kind, value)
        subscriptions = [node.create_subscription(String, topic,
            lambda msg, kind=kind: receive(kind, msg), 100)
            for topic, kind in [('/robot/targets', 'target'), ('/robot/log', 'log')]]
        publisher = node.create_publisher(String, '/vision/detections', 10)
        def wait_for(condition, timeout=10):
            deadline = time.monotonic()+timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                if condition():
                    return
            raise RuntimeError('Timed out waiting for expected ROS data; see process.log')
        # Do not inject synthetic targets into an existing robot domain.
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=.05)
        if any(info.node_name != node.get_name()
               for info in node.get_subscriptions_info_by_topic('/robot/targets')):
            raise RuntimeError('Domain has another robot target subscriber; choose a different --domain')
        server = start_server()
        bridge = start('vision_pick_place.robot_bridge', '--ros-args',
                       '-p', f'robot_port:={port}', '-p', 'ack_timeout_s:=0.5',
                       '-p', f'log_dir:={log_dir}')
        wait_for(lambda: publisher.get_subscription_count() == 1
                 and node.count_publishers('/robot/targets') == 1
                 and node.count_publishers('/robot/log') == 1)
        def send(seq, calibrated=True):
            value = dict(seq=seq, t_capture=time.time(), calib_ok=calibrated, objects=[
                dict(id=0, part_type='screw', x_mm=102.4 if calibrated else None,
                     y_mm=-55. if calibrated else None, angle_deg=179.5 if calibrated else None,
                     u_px=640.2, v_px=360.8),
                dict(id=1, part_type='nut', x_mm=10. if calibrated else None,
                     y_mm=20. if calibrated else None, angle_deg=None, u_px=100., v_px=200.)])
            if phase == 'normal':
                measured.receive('packet', value)
            publisher.publish(String(data=json.dumps(value)))
            return value
        phase = 'normal'
        for seq in range(100):
            send(seq, calibrated=(seq % 2 == 0))
            wait_for(lambda: seq in targets and seq in logs, timeout=3)
        normal = measured.report()
        if not normal['ok']:
            raise RuntimeError(f'100-packet run failed: {normal}')
        phase = 'restart'
        stop(server)
        send(100)
        wait_for(lambda: 100 in logs, timeout=3)
        if logs[100]['acked'] or not logs[100]['error']:
            raise RuntimeError('Server-down packet should have a failed ACK')
        server = start_server()
        wait_for(lambda: node.count_publishers('/robot/targets') == 1)
        value = send(101)
        wait_for(lambda: 101 in targets and 101 in logs)
        if not logs[101]['acked'] or targets[101] != value or 100 in targets:
            raise RuntimeError('Server restart recovery or no-replay check failed')
        report.update(ok=True, status='passed', normal=normal,
                      restart=dict(failure_recorded=True, failed_seq=100,
                                   recovered_seq=101, no_failed_packet_replay=True),
                      calibration_accuracy_verified=False)
    except (Exception, KeyboardInterrupt) as exc:
        report.update(ok=False, status='failed', error=f'{type(exc).__name__}: {exc}',
                      normal=measured.report())
    finally:
        for child in reversed(children):
            stop(child)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        process_log.close()
        report['finished_at_utc'] = datetime.now(timezone.utc).isoformat()
        save_report(output, report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
