"""Team 2 vision adapter: retain the original detector, refine screw grasp centers."""
import math
import time
import cv2
import numpy as np
import rclpy
from std_msgs.msg import String
from vision_pick_place.vision_node import VisionNode
from vision_pick_place.vision_pipeline import process_frame
from vision_pick_place.camera_stream import preprocess
from vision_pick_place.packet import encode
from vision_pick_place.team2_shaft_pose import shaft_pose,filter_metal_support


class ShaftVisionNode(VisionNode):
    def on_timer(self):
        frame=self.source.read();captured=time.time()
        if frame is None:return
        options={name:self.get_parameter(name).value for name in
                 ('blur_ksize','threshold','min_area_px','nut_max_aspect','segmentation_mode',
                  'hsv_max_saturation','hsv_max_dark_value','close_ksize','open_ksize')}
        try:
            roi=self.sheet_roi(frame)
            packet,debug=process_frame(frame,self.calibrator,self.seq,captured,roi_mask=roi,
                                        part_heights_mm=self.part_heights(),**options)
            mask=preprocess(frame,options['blur_ksize'],options['threshold'],
                **{k:options[k] for k in ['segmentation_mode','hsv_max_saturation','hsv_max_dark_value','close_ksize','open_ksize']})
            if roi is not None:mask=cv2.bitwise_and(mask,roi)
            # 낮은 밝기의 그림자도 원본 dark-metal 분할에 들어갈 수 있다.
            # 생성 금속 모델의 밝고 채도 낮은 표면 지지가 없는 성분만 제외한다. 좌표 정답은 쓰지 않는다.
            candidates=packet['objects']
            packet['objects']=filter_metal_support(frame,mask,candidates,
                                                   max_saturation=options['hsv_max_saturation'])
            for rejected in candidates:
                if rejected not in packet['objects']:
                    point=(round(rejected['u_px']),round(rejected['v_px']))
                    cv2.circle(debug,point,12,(0,0,255),2)
                    cv2.putText(debug,'shadow rejected',point,cv2.FONT_HERSHEY_SIMPLEX,.4,(0,0,255),1)
            for obj in packet['objects']:
                if obj['part_type']!='screw':continue
                center,axis=shaft_pose(mask,np.array([obj['u_px'],obj['v_px']]))
                if self.calibrator.status.ready:
                    mm=self.calibrator.px_to_mm(np.array([center,center+axis*30]),self.part_heights().get('screw',0.))
                    delta=mm[1]-mm[0]
                    obj.update(x_mm=float(mm[0,0]),y_mm=float(mm[0,1]),angle_deg=math.degrees(math.atan2(delta[1],delta[0]))%180)
                obj.update(u_px=float(center[0]),v_px=float(center[1]))
                p=tuple(np.rint(center).astype(int));end=tuple(np.rint(center+axis*30).astype(int))
                cv2.circle(debug,p,4,(255,0,255),1);cv2.arrowedLine(debug,p,end,(255,0,255),1)
            self.pub_det.publish(String(data=encode(packet).decode().rstrip('\n')))
            image=self.cv_bridge.cv2_to_imgmsg(debug,encoding='bgr8');image.header.stamp=self.get_clock().now().to_msg();image.header.frame_id='camera'
            self.pub_image.publish(image);self.seq+=1
        except (ValueError,RuntimeError,cv2.error) as exc:self.get_logger().error(str(exc))


def main():
    rclpy.init();node=ShaftVisionNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()
