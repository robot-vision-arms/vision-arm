"""ROS2 비전 노드 (2주차): 프레임 → 검출 → mm 변환 → 토픽 발행.

입력 (하나만):
    image_topic   ROS 영상 토픽 (예: Gazebo 가상 카메라 /sim/camera/image, config/params_sim.yaml)
    image_dir     사진 폴더 (MVTec·촬영본 재생)
    camera_url    스마트폰 IP 카메라 주소 (실물, config/params.yaml)
    camera_index  USB 카메라 번호

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
from vision_pick_place.part_check import RANGE_PARAMS, limits_from_params
from vision_pick_place.vision_pipeline import process_frame
from vision_pick_place.workspace_roi import sheet_roi_mask
from vision_pick_place.yolo_detector import detector_from_params


class TopicFrameSource:
    """ROS 영상 토픽을 FrameSource 와 같은 read()/release() 로 감싼다. 가장 최근 1장만 들고 있다가
    아직 돌려주지 않은 새 프레임이 있을 때만 준다 (FrameSource 의 카메라 입력과 같은 규칙)."""

    def __init__(self, node: Node, topic: str, bridge: CvBridge):
        self._node, self._bridge = node, bridge
        self._latest = None
        self._sub = node.create_subscription(Image, topic, self._on_image, 1)

    def _on_image(self, msg: Image):
        self._latest = msg

    def read(self):
        msg, self._latest = self._latest, None
        return None if msg is None else self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def release(self):
        self._node.destroy_subscription(self._sub)


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.declare_parameter('image_topic', '')
        self.declare_parameter('camera_url', '')
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('image_dir', '')
        self.declare_parameter('camera_rotate_deg', 0)
        self.declare_parameter('intrinsics_path', 'data/calib/intrinsics.npz')
        self.declare_parameter('homography_path', 'data/calib/homography.npz')
        self.declare_parameter('blur_ksize', 5)
        self.declare_parameter('threshold', -1)
        self.declare_parameter('segmentation_mode', 'gray')
        self.declare_parameter('hsv_max_saturation', 70)
        self.declare_parameter('hsv_max_dark_value', 70)
        self.declare_parameter('close_ksize', 0)
        self.declare_parameter('open_ksize', 0)
        self.declare_parameter('min_area_px', 500)
        self.declare_parameter('nut_max_aspect', 1.5)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('sheet_roi', True)
        self.declare_parameter('sheet_edge_margin_mm', 3.0)
        self.declare_parameter('sheet_marker_margin_mm', 5.0)
        # 시차 보정: 종류별 윤곽 중심 높이 (mm, 같은 순서의 병렬 배열). 빈 배열이면 보정 안 함 (calibrator.px_to_mm 설명)
        self.declare_parameter('parallax_part_types', [''])
        self.declare_parameter('parallax_heights_mm', [0.0])
        # 나사·너트가 아닌 물체를 unknown 으로 (part_check). [0, 0] 이면 그 항목은 안 본다
        self.declare_parameter('reject_unknown', False)
        self.declare_parameter('nut_min_hole_ratio', 0.0)
        for name in RANGE_PARAMS:
            self.declare_parameter(name, [0.0, 0.0])

        self.part_limits = limits_from_params({name: self.get_parameter(name).value for name in
                                               ('reject_unknown', 'nut_min_hole_ratio', *RANGE_PARAMS)})
        if self.part_limits is not None:
            self.get_logger().info(f'unknown 거르기 켬: {self.part_limits}')
        # 부품 찾는 방법: rule(기본, 전처리·윤곽·길이 비) / yolo(재학습 모델, config/params_yolo.yaml)
        self.declare_parameter('detector', 'rule')
        self.declare_parameter('yolo_model_path', 'data/models/yolo_green.onnx')
        self.declare_parameter('yolo_conf', 0.3)
        self.detector = detector_from_params({name: self.get_parameter(name).value for name in
                                              ('detector', 'yolo_model_path', 'yolo_conf', 'reject_unknown')})
        if self.detector is not None:
            self.get_logger().info(f"검출: YOLO {self.get_parameter('yolo_model_path').value}, 신뢰도 {self.detector.conf}")
        self.pub_det = self.create_publisher(String, '/vision/detections', 10)
        self.pub_image = self.create_publisher(Image, '/vision/debug_image', 10)
        self.cv_bridge = CvBridge()
        self.calibrator = Calibrator(self.get_parameter('intrinsics_path').value,
                                     self.get_parameter('homography_path').value)

        rate = self.get_parameter('publish_rate_hz').value
        if not math.isfinite(rate) or rate <= 0:
            raise ValueError('publish_rate_hz는 양수여야 합니다.')
        topic = self.get_parameter('image_topic').value
        if topic:
            self.source = TopicFrameSource(self, topic, self.cv_bridge)
            self.get_logger().info(f'입력: ROS 영상 토픽 {topic}')
        else:
            self.source = FrameSource(camera_index=self.get_parameter('camera_index').value,
                                      camera_url=self.get_parameter('camera_url').value,
                                      image_dir=self.get_parameter('image_dir').value,
                                      rotate_deg=self.get_parameter('camera_rotate_deg').value)
        if not self.calibrator.status.ready:
            self.get_logger().warning('미보정: px만 발행합니다. mm 좌표와 각도는 null입니다.')
        self.roi_mask = None
        self.roi_size = None  # ROI 는 보정·해상도가 정해지면 바뀌지 않으므로 첫 프레임에서 한 번만 만든다
        self.seq = 0
        self.timer = self.create_timer(1.0 / rate, self.on_timer)

    def on_timer(self):
        frame = self.source.read()
        t_capture = time.time()  # 프로젝트 정의: 프레임 읽기 직후부터 ACK까지
        if frame is None:
            self.get_logger().warning('프레임을 읽지 못했습니다.', throttle_duration_sec=5.0)
            return
        options = {name: self.get_parameter(name).value for name in
                   ('blur_ksize', 'threshold', 'min_area_px', 'nut_max_aspect', 'segmentation_mode',
                    'hsv_max_saturation', 'hsv_max_dark_value', 'close_ksize', 'open_ksize')}
        try:
            packet, debug = process_frame(frame, self.calibrator, self.seq, t_capture,
                                          roi_mask=self.sheet_roi(frame), part_heights_mm=self.part_heights(),
                                          part_limits=self.part_limits, detector=self.detector, **options)
        except (ValueError, RuntimeError, cv2.error) as exc:
            self.get_logger().error(f'프레임 처리 실패: {exc}', throttle_duration_sec=5.0)
            return
        self.pub_det.publish(String(data=encode(packet).decode('utf-8').rstrip('\n')))
        image = self.cv_bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        image.header.stamp = self.get_clock().now().to_msg()
        image.header.frame_id = 'camera'
        self.pub_image.publish(image)
        self.seq += 1

    def part_heights(self) -> dict:
        types = [t for t in self.get_parameter('parallax_part_types').value if t]
        heights = list(self.get_parameter('parallax_heights_mm').value)[:len(types)]
        if len(heights) != len(types):
            raise ValueError('parallax_part_types 와 parallax_heights_mm 의 길이가 같아야 합니다.')
        return dict(zip(types, heights))

    def sheet_roi(self, frame):
        """작업 시트 안쪽·마커 제외 마스크. 보정이 없거나 sheet_roi=false 면 None (영상 전체에서 검출)."""
        if not self.get_parameter('sheet_roi').value or not self.calibrator.status.ready:
            return None
        size = (frame.shape[1], frame.shape[0])
        if size != self.roi_size:
            self.roi_mask = sheet_roi_mask(self.calibrator, size,
                                           self.get_parameter('sheet_edge_margin_mm').value,
                                           self.get_parameter('sheet_marker_margin_mm').value)
            self.roi_size = size
            self.get_logger().info(f'작업 시트 ROI 사용: 검출 영역 = 영상의 {int((self.roi_mask > 0).mean() * 100)}%')
        return self.roi_mask

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
