"""Compare live Gazebo and MoveIt with the SDF selected by sim.launch.py.

python scripts/check_workcell_layout.py
Reads logs/workcell_session.json from the latest project launch. No model moves.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', default='logs/workcell_session.json')
    parser.add_argument('--out', default='logs/workcell_layout_check.json')
    args = parser.parse_args()
    result = dict(ok=False, bins={}, errors=[])
    node = context = None
    try:
        from google.protobuf import text_format
        from gz.msgs10.scene_pb2 import Scene
        import rclpy
        from rclpy.context import Context
        from moveit_msgs.srv import GetPlanningScene
        from moveit_msgs.msg import PlanningSceneComponents
        from ur5e_pick_place.workcell_scene import collision_boxes
        session = json.loads(Path(args.session).read_text())
        world_path = Path(session['world_file'])
        root = ET.parse(world_path).getroot().find('world')
        world_name = root.attrib['name']
        env = dict(os.environ, GZ_PARTITION=session['gz_partition'])
        result.update(world_file=str(world_path), gz_partition=session['gz_partition'])
        rc = subprocess.run(['gz', 'service', '-s', f'/world/{world_name}/scene/info',
              '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene', '--timeout', '6000', '--req', ''],
              env=env, capture_output=True, text=True, timeout=10)
        if rc.returncode or not rc.stdout.strip():
            raise RuntimeError('No Gazebo scene response from recorded session; restart sim.launch.py')
        scene = Scene()
        text_format.Parse(rc.stdout, scene)
        models = {model.name: model for model in scene.model}
        # ROS domain recorded by the same launch, not an unrelated default graph.
        os.environ['ROS_DOMAIN_ID'] = session['ros_domain_id']
        context = Context()
        rclpy.init(args=[], context=context)
        node = rclpy.create_node('workcell_layout_observer', context=context)
        client = node.create_client(GetPlanningScene, '/get_planning_scene')
        if not client.wait_for_service(timeout_sec=8):
            raise RuntimeError('MoveIt /get_planning_scene unavailable in recorded ROS domain')
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = client.call_async(request)
        executor = rclpy.executors.SingleThreadedExecutor(context=context)
        executor.add_node(node)
        try:
            executor.spin_until_future_complete(future, timeout_sec=8)
        finally:
            executor.shutdown()
        if not future.done() or future.result() is None:
            raise RuntimeError('MoveIt planning scene request timed out')
        objects = {obj.id: obj for obj in future.result().scene.world.collision_objects}
        expected_boxes = {box.id: box for box in collision_boxes(str(world_path))}
        def close(a, b):
            return len(a) == len(b) and all(abs(x-y) < 1e-5 for x, y in zip(a, b))
        for name in ('screw_bin', 'nut_bin'):
            expected = list(map(float, root.find(f"model[@name='{name}']/pose").text.split()))[:3]
            model = models[name]
            actual = [model.pose.position.x, model.pose.position.y, model.pose.position.z]
            matching = close(expected, actual)
            walls_ok = True
            for ident, box in expected_boxes.items():
                if not ident.startswith(name+'/'):
                    continue
                obj = objects.get(ident)
                if obj is None or not obj.primitive_poses or not obj.primitives:
                    walls_ok = False
                    continue
                pose = obj.primitive_poses[0]
                # This project publishes box poses in base_link, which coincides with world.
                if obj.header.frame_id not in ('base_link', 'world'):
                    walls_ok = False
                obj_pose = obj.pose
                # CollisionObject has an object pose as well as per-primitive pose.
                actual_pos = [obj_pose.position.x+pose.position.x,
                              obj_pose.position.y+pose.position.y,
                              obj_pose.position.z+pose.position.z]
                walls_ok &= close(box.position, actual_pos) and close(box.size, obj.primitives[0].dimensions)
            result['bins'][name] = dict(sdf_center_m=expected, gazebo_center_m=actual,
                                        gazebo_matches=matching, moveit_collision_matches=bool(walls_ok))
            if not matching or not walls_ok:
                result['errors'].append(f'{name}: SDF/Gazebo/MoveIt mismatch')
        result['ok'] = not result['errors']
    except Exception as exc:
        result['errors'].append(f'{type(exc).__name__}: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if context is not None:
            import rclpy
            rclpy.try_shutdown(context=context)
        out = Path(args.out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result, indent=2))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
