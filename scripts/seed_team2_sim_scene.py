"""Initialize a synthetic Gazebo test scene; does not command arm motion.

Run only while the arm is idle. --replace-demo explicitly permits replacing
obj_k models. Seed packets go directly to /robot/targets, so this initialization
is not a vision/TCP accuracy test. The running camera supplies real detections
again when initialization finishes.
"""
import argparse
import json
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replace-demo', action='store_true', required=True)
    parser.parse_args()
    import rclpy
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rclpy.init()
    node = rclpy.create_node('team2_demo_scene_initializer')
    try:
        pub = node.create_publisher(String, '/robot/targets', 10)
        client = node.create_client(Trigger, '/sim/spawn_objects')
        if not client.wait_for_service(timeout_sec=10):
            raise RuntimeError('Start team2_link.launch.py first: spawn service unavailable')
        deadline = time.monotonic() + 10
        while pub.get_subscription_count() < 2 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if pub.get_subscription_count() < 2:
            raise RuntimeError('Expected both spawner and pick_place_node subscribers')
        packet = dict(seq=int(time.time()*1000), t_capture=time.time(), calib_ok=True,
                      synthetic=True, source='team2_demo_scene_initializer', objects=[
                          dict(part_type='screw', x_mm=90.0, y_mm=80.0, angle_deg=45.0),
                          dict(part_type='nut', x_mm=180.0, y_mm=140.0, angle_deg=None)])
        # Keep the explicitly synthetic seed available until the spawn callback
        # snapshots it; normal camera packets may be streaming concurrently.
        future = None
        started = time.monotonic()
        while time.monotonic()-started < 35:
            pub.publish(String(data=json.dumps(packet)))
            rclpy.spin_once(node, timeout_sec=0.02)
            if future is None and time.monotonic()-started > 0.5:
                future = client.call_async(Trigger.Request())
            if future is not None and future.done():
                reply = future.result()
                print(reply.message)
                if not reply.success or f'seq={packet["seq"]} ' not in reply.message:
                    raise RuntimeError('Scene initialization not confirmed; do not run cycle')
                print('Demo scene ready. Wait for camera detections before calling run_cycle.')
                return
        raise RuntimeError('Spawn response timeout; check simulator before retrying')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
