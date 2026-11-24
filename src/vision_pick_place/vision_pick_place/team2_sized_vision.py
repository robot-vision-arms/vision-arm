"""실물 카메라/사진/ROS 입력을 그대로 사용하고 나사별 length_mm를 추가하는 전용 실행기."""
import time
import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from vision_pick_place.vision_node import VisionNode
from vision_pick_place.packet import encode
from vision_pick_place.team2_length_measurement import LengthTracker,process_frame_with_lengths


class SizedVisionNode(VisionNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter('length_stable_frames',3)
        self.declare_parameter('length_stability_mm',2.)
        self.length_tracker=LengthTracker(self.get_parameter('length_stable_frames').value,
                                         self.get_parameter('length_stability_mm').value)

    def on_timer(self):
        frame=self.source.read();captured=time.time()
        if frame is None:return
        options={name:self.get_parameter(name).value for name in
                 ('blur_ksize','threshold','min_area_px','nut_max_aspect','segmentation_mode',
                  'hsv_max_saturation','hsv_max_dark_value','close_ksize','open_ksize')}
        try:
            packet,debug=process_frame_with_lengths(frame,self.calibrator,self.seq,captured,self.length_tracker,
                roi_mask=self.sheet_roi(frame),part_heights_mm=self.part_heights(),
                part_limits=self.part_limits,detector=self.detector,**options)
            for obj in packet['objects']:
                if obj.get('length_mm') is not None:
                    point=(round(obj['u_px']),round(obj['v_px'])+18)
                    status=obj.get('length_status','')
                    cv2.putText(debug,f"L~{obj['length_mm']:.0f} mm [{status}]",point,cv2.FONT_HERSHEY_SIMPLEX,.45,(255,0,255),1)
                if obj['part_type']=='screw':
                    head=obj.get('head_angle_deg')
                    label=f"head {head:.0f} deg" if head is not None else 'head ?'
                    cv2.putText(debug,label,(round(obj['u_px']),round(obj['v_px'])+35),
                                cv2.FONT_HERSHEY_SIMPLEX,.4,(0,180,255),1)
            self.pub_det.publish(String(data=encode(packet).decode().rstrip('\n')))
            image=self.cv_bridge.cv2_to_imgmsg(debug,encoding='bgr8')
            image.header.stamp=self.get_clock().now().to_msg();image.header.frame_id='camera'
            self.pub_image.publish(image);self.seq+=1
        except (ValueError,RuntimeError,cv2.error) as exc:
            self.get_logger().error(f'길이 포함 프레임 처리 실패: {exc}',throttle_duration_sec=5.)


def main():
    rclpy.init();node=None
    try:
        node=SizedVisionNode();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
