"""Explicit simulation-only repeated MoveIt motion check using existing robot code.

Start project sim.launch.py first, then:
python scripts/check_team2_repeat_motion.py --execute --cycles 5
Moves to A=(100,100)mm then B=(130,100)mm and back, pad height 0.30m.
No spawning/attachment/bin-drop actions. Restores runtime target parameters.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

from check_team_integration import save_report
from vision_pick_place.team2_session import load_session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Explicitly permit simulated arm motion')
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--session', default='logs/workcell_session.json')
    parser.add_argument('--out', default='logs/team2_repeat_motion_check.json')
    args = parser.parse_args()
    if not args.execute:
        parser.error('This moves the simulated arm. Add --execute only in project Gazebo.')
    if not 1 <= args.cycles <= 20:
        parser.error('--cycles must be in 1..20')
    report = dict(ok=False, cycles_requested=args.cycles, steps=[], parameters_restored=False,
                  scope='Two preset empty-space poses only; not arbitrary RViz goals or bin placement')
    save_report(args.out, report)
    node = None
    initialized = False
    original = []
    try:
        session = load_session(args.session, os.environ.get('ROS_DOMAIN_ID', '0'))
        result = subprocess.run([sys.executable, str(Path(__file__).with_name('check_workcell_layout.py')),
                                 '--session', args.session], capture_output=True, text=True, timeout=30)
        if result.returncode:
            raise RuntimeError('Scene alignment check failed: '+result.stdout[-1500:])
        import rclpy
        from rcl_interfaces.srv import GetParameters, SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
        from std_srvs.srv import Trigger
        from tf2_ros import Buffer, TransformListener
        from rclpy.time import Time
        from ur5e_pick_place.grasp import PlanePose, work_to_base
        rclpy.init(args=[])
        initialized = True
        node = rclpy.create_node('team2_repeat_motion_observer')
        get = node.create_client(GetParameters, '/pick_place_node/get_parameters')
        set_client = node.create_client(SetParameters, '/pick_place_node/set_parameters')
        move = node.create_client(Trigger, '/sim/move_to_target')
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        def call(client, request, timeout=75):
            if not client.wait_for_service(timeout_sec=10):
                raise RuntimeError('Service missing: '+client.srv_name)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done() or future.result() is None:
                raise RuntimeError('Response timeout: '+client.srv_name+'; stop inspection and check controller state')
            return future.result()
        changed = ['target_x_mm', 'target_y_mm', 'target_height_m', 'target_angle_deg',
                   'max_velocity_scaling', 'motion_timeout_s']
        plane_names = ['work_plane_x', 'work_plane_y', 'work_plane_z', 'work_plane_yaw_deg', 'tool_length_m']
        values = call(get, GetParameters.Request(names=changed+plane_names)).values
        original = [Parameter(name=name, value=value) for name, value in zip(changed, values)]
        plane_values = [value.double_value for value in values[len(changed):]]
        plane = PlanePose(*plane_values[:4])
        tool = plane_values[4]
        def update(values):
            request = SetParameters.Request(parameters=[Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))) for name, value in values.items()])
            response = call(set_client, request)
            if not all(item.successful for item in response.results):
                raise RuntimeError('Runtime parameter update rejected')
        sequence = [('A_start', 100.)]
        for cycle in range(1, args.cycles+1):
            sequence.extend([(f'B_{cycle}', 130.), (f'A_{cycle}', 100.)])
        for name, x_mm in sequence:
            update(dict(target_x_mm=x_mm, target_y_mm=100., target_height_m=.30,
                        target_angle_deg=0., max_velocity_scaling=.10, motion_timeout_s=40.))
            print(f'Checking {name}: ({x_mm},100)mm, pad height 0.30m', flush=True)
            started = time.monotonic()
            response = call(move, Trigger.Request())
            record = dict(name=name, success=response.success, message=response.message,
                          elapsed_s=time.monotonic()-started)
            report['steps'].append(record)
            save_report(args.out, report)
            if not response.success:
                raise RuntimeError(f'{name}: {response.message}; no further motion requests issued')
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=.05)
            tf = buffer.lookup_transform('base_link', 'tool0', Time())
            actual = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
            x,y,z = work_to_base(x_mm, 100., plane)
            expected = [x,y,z+.30+tool]
            error = math.dist(actual, expected)
            record.update(actual_tool0_m=actual, expected_tool0_m=expected, position_error_m=error)
            print(f'{name}: success, TF error {error*1000:.2f}mm; {response.message}', flush=True)
            if error > .01:
                raise RuntimeError('TF position differs from target by more than 10mm')
        report.update(ok=True, completed_cycles=args.cycles,
                      gz_partition=session['gz_partition'], speed_scaling=.10)
    except Exception as exc:
        report['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if node is not None and original:
            try:
                restored = call(set_client, SetParameters.Request(parameters=original), timeout=5)
                report['parameters_restored'] = all(value.successful for value in restored.results)
                if not report['parameters_restored']:
                    report['ok'] = False
            except Exception as exc:
                report.update(ok=False, restore_error=str(exc))
        if node is not None:
            node.destroy_node()
        if initialized:
            rclpy.try_shutdown()
        save_report(args.out, report)
        print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
