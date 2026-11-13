"""실제 ROS 퍼블리셔/구독자 통합 확인. ROS를 source한 환경에서 실행한다."""
import json
import time

import cv2
import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('cv_bridge')
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from vision_pick_place.vision_node import VisionNode


@pytest.mark.parametrize('calibrated', [False, True])
def test_publishes_pose_and_debug_image_with_real_ros(tmp_path, calibrated):
    image_dir = tmp_path / 'images'
    image_dir.mkdir()
    frame = np.full((120, 160, 3), 255, dtype=np.uint8)
    cv2.rectangle(frame, (40, 50), (120, 70), (0, 0, 0), -1)
    cv2.imwrite(str(image_dir / 'frame.png'), frame)
    intrinsics, homography = tmp_path / 'i.npz', tmp_path / 'h.npz'
    if calibrated:
        np.savez(intrinsics, camera_matrix=np.array([[300., 0, 80], [0, 300., 60], [0, 0, 1]]),
                 dist_coeffs=np.zeros(5), rms=0.)
        np.savez(homography, H=np.diag([2., 2., 1.]), reproj_err_mm=0.)
    rclpy.init(args=['--ros-args', '-p', f'image_dir:={image_dir}',
                    '-p', f'intrinsics_path:={intrinsics}', '-p', f'homography_path:={homography}'])
    executor = SingleThreadedExecutor()
    node = probe = None
    try:
        node = VisionNode()
        probe = Node('week2_vision_probe')
        packets, images = [], []
        sub_packet = probe.create_subscription(String, '/vision/detections', lambda m: packets.append(json.loads(m.data)), 10)
        sub_image = probe.create_subscription(Image, '/vision/debug_image', images.append, 10)
        executor.add_node(node)
        executor.add_node(probe)
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and (not packets or not images):
            executor.spin_once(timeout_sec=0.1)
        assert packets and images, '비전 토픽을 수신하지 못했습니다.'
        packet = packets[-1]
        assert packet['calib_ok'] is calibrated
        obj = packet['objects'][0]
        assert obj['part_type'] == 'screw'
        if calibrated:
            assert obj['x_mm'] == pytest.approx(160, abs=1)
            assert obj['y_mm'] == pytest.approx(120, abs=1)
            assert obj['angle_deg'] == pytest.approx(0, abs=1)
        else:
            assert obj['x_mm'] is obj['y_mm'] is obj['angle_deg'] is None
        assert images[-1].encoding == 'bgr8'
        assert (images[-1].width, images[-1].height) == (160, 120)
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if probe is not None:
            probe.destroy_node()
        rclpy.try_shutdown()
