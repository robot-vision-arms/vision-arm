"""길이 포함 전용 노드가 실제 ROS 토픽으로 안정화된 길이와 영상을 보내는지 확인한다."""
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String

from vision_pick_place.team2_sized_vision import SizedVisionNode


def test_sized_vision_publishes_stable_length_and_image(tmp_path):
    images=tmp_path/'images';images.mkdir()
    frame=np.full((120,160,3),255,dtype=np.uint8)
    cv2.rectangle(frame,(40,50),(120,70),(0,0,0),-1)
    cv2.imwrite(str(images/'frame.png'),frame)
    intrinsics,homography=tmp_path/'i.npz',tmp_path/'h.npz'
    np.savez(intrinsics,camera_matrix=np.array([[300.,0,80],[0,300.,60],[0,0,1]]),
             dist_coeffs=np.zeros(5),rms=0.)
    np.savez(homography,H=np.diag([.25,.25,1.]),reproj_err_mm=0.)
    rclpy.init(args=['--ros-args','-p',f'image_dir:={images}',
                    '-p',f'intrinsics_path:={intrinsics}','-p',f'homography_path:={homography}',
                    '-p','sheet_roi:=false'])
    executor=SingleThreadedExecutor();node=probe=None
    try:
        node=SizedVisionNode();probe=rclpy.create_node('team2_length_probe')
        packets=[];debug=[]
        probe.create_subscription(String,'/vision/detections',lambda m:packets.append(json.loads(m.data)),10)
        probe.create_subscription(Image,'/vision/debug_image',debug.append,10)
        executor.add_node(node);executor.add_node(probe)
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.1)
            if debug and any(o.get('length_status')=='stable' for p in packets for o in p['objects']):break
        stable=[o for p in packets for o in p['objects'] if o.get('length_status')=='stable']
        assert stable and debug
        assert stable[-1]['part_type']=='screw'
        assert stable[-1]['length_mm']==20.
        assert stable[-1]['x_mm']==20.
        assert debug[-1].encoding=='bgr8'
    finally:
        executor.shutdown()
        if node is not None:node.destroy_node()
        if probe is not None:probe.destroy_node()
        rclpy.try_shutdown()
