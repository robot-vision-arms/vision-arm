"""기존 검출 결과에 보정된 나사 길이만 추가한다. 원본 비전 파일은 수정하지 않는다."""
import copy
import math
import numpy as np

from vision_pick_place.camera_stream import preprocess
from vision_pick_place.vision_detector import find_objects
from vision_pick_place.vision_pipeline import process_frame
from vision_pick_place.team2_head_direction import head_direction


def detection_length_mm(det, calibrator, height_mm=0.):
    if not calibrator.status.ready:
        return None
    center=np.asarray(det.center_px,dtype=float)
    axis=np.asarray(det.long_axis_px,dtype=float)
    norm=float(np.linalg.norm(axis))
    if not math.isfinite(norm) or norm <= 0 or not math.isfinite(det.length_px) or det.length_px <= 0:
        return None
    # 한 픽셀 환산 비율을 고정하지 않고 양 끝점을 각각 호모그래피로 변환한다.
    ends=np.array([center-axis/norm*det.length_px/2,center+axis/norm*det.length_px/2])
    mm=calibrator.px_to_mm(ends,height_mm)
    length=float(np.linalg.norm(mm[1]-mm[0]))
    return length if math.isfinite(length) and 3 <= length <= 80 else None


class LengthTracker:
    def __init__(self,frames=3,tolerance_mm=2.):
        if not 2 <= frames <= 10 or not math.isfinite(tolerance_mm) or tolerance_mm <= 0:
            raise ValueError('길이 안정화는 2~10프레임과 양수 오차 범위를 사용합니다.')
        self.frames=frames;self.tolerance=tolerance_mm;self.tracks=[];self.tick=0

    def annotate(self,packet,measurements):
        packet=copy.deepcopy(packet);self.tick+=1
        self.tracks=[t for t in self.tracks if self.tick-t['seen']<=10]
        objects=[o for o in packet['objects'] if o['part_type']=='screw']
        candidates=[]
        for i,obj in enumerate(objects):
            for field in ['length_mm','length_raw_mm','length_source','length_status']:
                obj.pop(field,None)
            if obj.get('x_mm') is None or obj.get('y_mm') is None:
                continue
            for j,track in enumerate(self.tracks):
                distance=math.hypot(obj['x_mm']-track['x'],obj['y_mm']-track['y'])
                if distance <= 5:candidates.append((distance,i,j))
        assignment={};used=set()
        for _,i,j in sorted(candidates):
            if i not in assignment and j not in used:
                assignment[i]=self.tracks[j];used.add(j)
        for i,obj in enumerate(objects):
            # 검출 순서(id)가 바뀌어도 위치로 대응한다. 서로 다른 나사의 길이를 섞지 않는다.
            length=measurements.get(obj['id'])
            obj['length_status']='unavailable'
            if (isinstance(length,bool) or not isinstance(length,(int,float))
                    or not math.isfinite(length) or not 3 <= length <= 80
                    or obj.get('x_mm') is None or obj.get('y_mm') is None):
                if i in assignment:
                    track=assignment[i]
                    # 잠깐 가려진 프레임 때문에 생성 전체를 멈추지 않는다.
                    # 최근에 같은 위치에서 측정한 길이임을 상태에 명시한다.
                    if 'last_length' in track:
                        obj['length_mm']=track['last_length']
                        obj['length_source']='recent_estimate';obj['length_status']='fallback'
                continue
            track=assignment.get(i)
            if track is None:
                track={'history':[]};self.tracks.append(track)
            track.update(x=obj['x_mm'],y=obj['y_mm'],seen=self.tick)
            track['history']=(track['history']+[length])[-self.frames:]
            obj['length_raw_mm']=length
            obj['length_mm']=float(math.floor(length+.5))
            obj['length_source']='provisional_obb'
            track['last_length']=obj['length_mm']
            obj['length_status']='warming_up'
            if len(track['history'])==self.frames:
                if max(track['history'])-min(track['history']) <= self.tolerance:
                    obj['length_mm']=float(math.floor(float(np.median(track['history']))+.5))
                    obj['length_source']='calibrated_obb';obj['length_status']='stable'
                    track['last_length']=obj['length_mm']
                else:obj['length_status']='unstable'
        return packet


class _CollectedDetector:
    def __init__(self,detector):self.detector=detector;self.detections=[]
    def detect(self,frame):
        result=self.detector.detect(frame);self.detections=[obj.det for obj in result]
        return result


def process_frame_with_lengths(frame,calibrator,seq,t_capture,tracker,**options):
    options=dict(options)
    detector=options.get('detector')
    collector=_CollectedDetector(detector) if detector is not None else None
    if collector is not None:options['detector']=collector
    packet,debug=process_frame(frame,calibrator,seq,t_capture,**options)
    mask=preprocess(frame,options.get('blur_ksize',5),options.get('threshold',-1),
        **{k:options[k] for k in ['segmentation_mode','hsv_max_saturation','hsv_max_dark_value','close_ksize','open_ksize'] if k in options})
    if options.get('roi_mask') is not None:
        import cv2
        mask=cv2.bitwise_and(mask,options['roi_mask'])
    if collector is not None:
        # YOLO 추론은 한 번만 수행한다. 원본 분류·좌표 결과는 그대로 사용한다.
        detections=collector.detections
    else:
        detections=find_objects(mask,options.get('min_area_px',500))
    measurements={}
    for obj in packet['objects']:
        if obj['part_type']!='screw':continue
        matched=[d for d in detections if math.hypot(d.center_px[0]-obj['u_px'],d.center_px[1]-obj['v_px'])<.01]
        obj['head_status']='ambiguous'
        if len(matched)==1:
            head=head_direction(matched[0],mask,calibrator,(options.get('part_heights_mm') or {}).get('screw',0.))
            if head is not None:
                obj['head_angle_deg']=head;obj['head_status']='estimated'
            measurements[obj['id']]=detection_length_mm(matched[0],calibrator,
                (options.get('part_heights_mm') or {}).get('screw',0.))
    return tracker.annotate(packet,measurements),debug
