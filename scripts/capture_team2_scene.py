"""Save an image and ground truth from actual Gazebo TF/model geometry (not predictions)."""
import argparse
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from tf2_ros import Buffer,TransformListener
from ur5e_pick_place.team2_scene_labels import visible_vertices,quaternion_matrix,project_world


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--profile',required=True);p.add_argument('--out',required=True);args=p.parse_args()
    profile=json.loads(Path(args.profile).read_text());out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=rclpy.create_node('team2_scene_ground_truth');buffer=Buffer();listener=TransformListener(buffer,node)
    truth=[];frames=[]
    subs=[node.create_subscription(String,'/team2/scene_truth',lambda m:truth.append(json.loads(m.data)),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
          node.create_subscription(Image,'/sim/camera/image',lambda m:frames.append(m) if len(frames)<4 else None,qos_profile_sensor_data)]
    try:
        end=time.monotonic()+15
        while (not truth or len(frames)<4) and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
        if not truth or len(frames)<4:raise RuntimeError('Scene truth or camera image unavailable')
        image=CvBridge().imgmsg_to_cv2(frames[-1],'bgr8');overlay=image.copy()
        with np.load(Path(profile['calibration_dir'])/'intrinsics.npz') as data:K=data['camera_matrix']
        world=ET.parse(profile['world_file']).getroot();camera=list(map(float,world.findtext(".//model[@name='top_camera']/pose").split()))
        labels=[]
        for slot,obj in enumerate(truth[-1]['objects']):
            name=f'obj_{slot}'
            end=time.monotonic()+5
            while not buffer.can_transform('base_link',name,Time()) and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
            t=buffer.lookup_transform('base_link',name,Time()).transform
            pos=np.array([t.translation.x,t.translation.y,t.translation.z]);q=(t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w)
            R=quaternion_matrix(q);kind=obj['part_type']
            vertices=visible_vertices(Path(profile['asset_prefix'])/f'share/ur5e_pick_place/models/{kind}/model.sdf')
            pixels=project_world((R@vertices.T).T+pos,camera,K).astype(np.float32)
            box=cv2.boxPoints(cv2.minAreaRect(pixels))
            labels.append(dict(model=name,part_type=kind,position_world_m=pos.tolist(),quaternion_xyzw=q,
                               yaw_deg=float(np.degrees(np.arctan2(R[1,0],R[0,0]))%180) if kind=='screw' else None,
                               obb_px=box.tolist(),bbox_xyxy_px=[*pixels.min(axis=0).tolist(),*pixels.max(axis=0).tolist()]))
            cv2.polylines(overlay,[box.astype(np.int32)],True,(0,255,255),1)
            cv2.putText(overlay,name+':'+kind,tuple(box[0].astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,255,255),1)
        cv2.imwrite(str(out/'image.png'),image);cv2.imwrite(str(out/'overlay.png'),overlay)
        (out/'labels.json').write_text(json.dumps(dict(source='Gazebo TF + known model geometry; no detection predictions',
                 scene_seq=truth[-1]['seq'],synthetic_scene=True,frame_stamp=dict(sec=frames[-1].header.stamp.sec,nanosec=frames[-1].header.stamp.nanosec),
                 profile=profile,labels=labels,temporal_alignment='settled static fixture; not motion-synchronized'),indent=2))
        print('Saved',len(labels),'independent ground-truth objects to',out)
    finally:node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
