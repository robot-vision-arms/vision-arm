"""ROS2 비전 노드 (2주차): 프레임 → 검출 → mm 변환 → 토픽 발행.

발행 토픽:
    /vision/detections   std_msgs/String   packet.make_pose_packet() 결과를 JSON 문자열로
    /vision/debug_image  sensor_msgs/Image ROI 박스·장축 화살표를 그린 영상 (GUI 표시용)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
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
        # TODO: /vision/debug_image 퍼블리셔 (cv_bridge.CvBridge().cv2_to_imgmsg)
        # TODO: FrameSource, Calibrator 생성

        rate = self.get_parameter('publish_rate_hz').value
        self.seq = 0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    def on_timer(self):
        # TODO: t_capture = time.time() → read → preprocess → find_objects → classify_part → px_to_mm → 각도 계산
        #       → make_pose_packet → String 으로 발행, 디버그 영상 발행
        pass


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
