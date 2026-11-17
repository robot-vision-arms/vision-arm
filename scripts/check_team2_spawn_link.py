"""Check TCP → unmodified team 3 spawner in a running project simulation.

Default: passively waits for a calibrated source packet, then spawns/clears it.
--demo: explicitly inject synthetic calibrated test poses in a dedicated domain.
No arm motion or attachment command is issued by this script.
Existing obj_k models are replaced by the spawner; run in a fresh test scene.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time

from check_team_integration import save_report
from vision_pick_place.team2_session import load_session, check_gazebo


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', default='logs/workcell_session.json')
    parser.add_argument('--demo', action='store_true')
    parser.add_argument('--out', default='logs/team2_spawn_link_check.json')
    args = parser.parse_args()
    report = dict(ok=False, synthetic=args.demo, arm_motion_commanded=False)
    save_report(args.out, report)
    node = None
    context_started = False
    try:
        session = load_session(args.session, os.environ.get('ROS_DOMAIN_ID', '0'))
        if args.demo and str(session.get('ros_domain_id', '0')) == '0':
            raise ValueError('--demo requires a dedicated nonzero ROS_DOMAIN_ID; do not use the live input domain')
        os.environ['GZ_PARTITION'] = session['gz_partition']
        check_gazebo(session, os.environ.copy())
        import rclpy
        from ament_index_python.packages import get_package_share_directory
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V
        from ur5e_pick_place.object_spawner import load_sim_params, plane_from_params, plan_spawns
        p = load_sim_params(Path(get_package_share_directory('ur5e_pick_place'))/'config/sim.yaml')
        poses = {}
        pose_times = {}
        def on_poses(msg):
            for pose in msg.pose:
                poses[pose.name] = (pose.position.x, pose.position.y, pose.position.z)
                pose_times[pose.name] = time.monotonic()
        gz = GzNode()
        if not gz.subscribe(Pose_V, f"/world/{session['world_name']}/pose/info", on_poses):
            raise RuntimeError('Cannot subscribe to Gazebo pose stream')
        rclpy.init(args=[])
        context_started = True
        node = rclpy.create_node('team2_spawn_link_observer')
        targets, logs = {}, {}
        def receive(store, msg):
            value = json.loads(msg.data)
            store[value['seq']] = value
        subscriptions = [node.create_subscription(String, topic,
            lambda msg, store=store: receive(store, msg), 100)
            for topic, store in [('/robot/targets', targets), ('/robot/log', logs)]]
        publisher = node.create_publisher(String, '/vision/detections', 10)
        spawn = node.create_client(Trigger, '/sim/spawn_objects')
        clear = node.create_client(Trigger, '/sim/clear_objects')
        def wait(condition, timeout=15):
            deadline = time.monotonic()+timeout
            while time.monotonic()<deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if condition(): return
            raise RuntimeError('Timed out waiting for team link. Check tcp.launch, object_spawner and matching domain/partition.')
        def call(client):
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('Missing service: '+client.srv_name)
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=25)
            if not future.done() or future.result() is None:
                raise RuntimeError('Service timed out: '+client.srv_name)
            return future.result()
        if not spawn.wait_for_service(timeout_sec=15):
            raise RuntimeError('/sim/spawn_objects absent. Start team2_link.launch.py after Gazebo.')
        if args.demo:
            wait(lambda: publisher.get_subscription_count()>0 and node.count_publishers('/robot/targets')>0)
            if node.count_publishers('/vision/detections') != 1:
                raise RuntimeError('Demo requires an isolated domain without another vision publisher')
            base = int(time.time()*1000)
            def transmit(seq, calibrated):
                packet = dict(seq=seq, t_capture=time.time(), calib_ok=calibrated, objects=[
                    dict(id=0, part_type='screw', x_mm=100. if calibrated else None,
                         y_mm=105. if calibrated else None, angle_deg=30. if calibrated else None,
                         u_px=100., v_px=105.),
                    dict(id=1, part_type='nut', x_mm=200. if calibrated else None,
                         y_mm=150. if calibrated else None, angle_deg=None, u_px=200., v_px=150.),
                    dict(id=2, part_type='unknown', x_mm=250. if calibrated else None,
                         y_mm=40. if calibrated else None, angle_deg=None, u_px=250., v_px=40.)])
                publisher.publish(String(data=json.dumps(packet)))
                wait(lambda: seq in targets and seq in logs)
                if targets[seq] != packet or not logs[seq]['acked']:
                    raise RuntimeError('TCP ACK/target mismatch')
                for _ in range(6): rclpy.spin_once(node, timeout_sec=.05)
                return packet
            transmit(base, False)
            rejected = call(spawn)
            if rejected.success:
                raise RuntimeError('Uncalibrated packet should be rejected')
            report['uncalibrated_rejected'] = rejected.message
            packet = transmit(base+1, True)
        else:
            wait(lambda: any(value.get('calib_ok') for value in targets.values()), timeout=20)
            packet = next(value for value in reversed(list(targets.values())) if value.get('calib_ok'))
            wait(lambda: packet['seq'] in logs)
            if not logs[packet['seq']]['acked']:
                raise RuntimeError('Latest calibrated packet has no successful ACK')
        requests, skipped = plan_spawns(packet, plane_from_params(p), p['part_types'], p['max_objects'])
        if not requests:
            raise RuntimeError('No supported calibrated objects to spawn')
        results = []
        for attempt in range(2):
            response = call(spawn)
            if not response.success:
                raise RuntimeError('Spawner rejected packet: '+response.message)
            import re
            match = re.search(r'seq=(\d+)', response.message)
            if match is None:
                raise RuntimeError('Spawner response has no seq: '+response.message)
            actual_seq = int(match.group(1))
            wait(lambda: actual_seq in targets and actual_seq in logs)
            if args.demo and actual_seq != packet['seq']:
                raise RuntimeError('Unexpected vision source replaced demo packet')
            packet = targets[actual_seq]
            if not packet.get('calib_ok') or not logs[actual_seq]['acked']:
                raise RuntimeError('Spawned packet has no valid calibration/ACK')
            requests, skipped = plan_spawns(packet, plane_from_params(p), p['part_types'], p['max_objects'])
            if not requests:
                raise RuntimeError('No supported objects in spawned packet')
            observed_after = time.monotonic()
            wait(lambda: all(req.name in poses and pose_times.get(req.name, 0) > observed_after
                             and math.dist(poses[req.name][:2], (req.x,req.y))<.015
                             and abs(poses[req.name][2]-p['work_plane_z'])<.03 for req in requests))
            results.append(dict(seq=actual_seq, message=response.message,
                                positions_m={req.name:list(poses[req.name]) for req in requests}))
        cleared = call(clear)
        if not cleared.success:
            raise RuntimeError(cleared.message)
        import subprocess
        from google.protobuf import text_format
        from gz.msgs10.scene_pb2 import Scene
        def remaining_objects():
            response = subprocess.run(['gz', 'service', '-s', f"/world/{session['world_name']}/scene/info",
                '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene', '--timeout', '2000', '--req', ''],
                capture_output=True, text=True, timeout=4)
            if not response.stdout.strip():
                raise RuntimeError('No scene response to verify object removal')
            scene = Scene(); text_format.Parse(response.stdout, scene)
            return [model.name for model in scene.model if model.name in [r.name for r in requests]]
        if remaining_objects():
            raise RuntimeError('Clear service succeeded but objects remain in Gazebo')
        report.update(ok=True, seq=packet['seq'], acked=True, targets_equal_source=True if args.demo else None,
                      expected_objects=len(requests), skipped=skipped, spawn_and_respawn=results,
                      clear_service_success=True, gz_partition=session['gz_partition'],
                      real_camera_calibration_verified=False)
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if node is not None: node.destroy_node()
        if context_started: rclpy.try_shutdown()
        save_report(args.out, report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
