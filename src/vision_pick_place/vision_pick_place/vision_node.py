"""ROS2 비전 노드 (2주차): 프레임 → 검출 → mm 변환 → 토픽 발행.

발행 토픽:
    /vision/detections   std_msgs/String   packet.make_pose_packet() 결과를 JSON 문자열로
    /vision/debug_image  sensor_msgs/Image ROI 박스·장축 화살표를 그린 영상 (GUI 표시용)
"""
import math
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from vision_pick_place.calibrator import Calibrator
from vision_pick_place.camera_stream import FrameSource
from vision_pick_place.packet import encode
from vision_pick_place.vision_pipeline import process_frame


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('camera_url', '')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_dir', '')
        self.declare_parameter('intrinsics_path', 'data/calib/intrinsics.npz')
        self.declare_parameter('homography_path', 'data/calib/homography.npz')
        self.declare_parameter('blur_ksize', 5)
        self.declare_parameter('threshold', -1)
        self.declare_parameter('min_area_px', 500)
        self.declare_parameter('nut_max_aspect', 1.5)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.pub_det = self.create_publisher(String, '/vision/detections', 10)
        self.pub_image = self.create_publisher(Image, '/vision/debug_image', 10)
        self.cv_bridge = CvBridge()
        self.calibrator = Calibrator(self.get_parameter('intrinsics_path').value,
                                     self.get_parameter('homography_path').value)

        rate = self.get_parameter('publish_rate_hz').value
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz는 양수여야 합니다.')
        self.source = FrameSource(camera_index=self.get_parameter('camera_index').value,
                                  camera_url=self.get_parameter('camera_url').value,
                                  image_dir=self.get_parameter('image_dir').value)
        if not self.calibrator.status.ready:
            self.get_logger().warning('미보정: px만 발행합니다. mm 좌표와 각도는 null입니다.')
        self.seq = 0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    def on_timer(self):
        frame = self.source.read()
        t_capture = time.time()  # 프로젝트 정의: 프레임 읽기 직후부터 ACK까지
        if frame is None:
            self.get_logger().warning('프레임을 읽지 못했습니다.', throttle_duration_sec=5.0)
            return
        options = {name: self.get_parameter(name).value for name in
                   ('blur_ksize', 'threshold', 'min_area_px', 'nut_max_aspect')}
        try:
            packet, debug = process_frame(frame, self.calibrator, self.seq, t_capture, **options)
        except (ValueError, RuntimeError, cv2.error) as exc:
            self.get_logger().error(f'프레임 처리 실패: {exc}', throttle_duration_sec=5.0)
            return
        self.pub_det.publish(String(data=encode(packet).decode('utf-8').rstrip('\n')))
        image = self.cv_bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'camera'
        self.pub_image.publish(image)
        self.seq += 1

    def destroy_node(self):
        self.source.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VisionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
