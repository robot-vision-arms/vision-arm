"""ROS 없이 검증하는 프레임 처리. 미보정 좌표는 null로 보내 mm로 오인하지 않게 한다."""
import math

import cv2
import numpy as np

from vision_pick_place.camera_stream import preprocess
from vision_pick_place.packet import make_pose_packet
from vision_pick_place.part_check import measure, reject_reason
from vision_pick_place.vision_detector import classify_part, find_objects, normalize_angle_deg


def center_inside(det, roi_mask) -> bool:
    if roi_mask is None:
        return True
    x, y = np.rint(det.center_px).astype(int)
    h, w = roi_mask.shape
    return 0 <= x < w and 0 <= y < h and roi_mask[y, x] > 0


def process_frame(frame, calibrator, seq, t_capture, *, blur_ksize=5, threshold=-1,
                  min_area_px=500, nut_max_aspect=1.5, segmentation_mode='gray',
                  hsv_max_saturation=70, hsv_max_dark_value=70, close_ksize=0, open_ksize=0,
                  roi_mask=None, part_heights_mm=None, part_limits=None, detector=None):
    """roi_mask: 원본 영상 크기의 uint8 마스크(255 = 검출할 곳, workspace_roi.sheet_roi_mask). None 이면 영상 전체.
    part_heights_mm: {part_type: 윤곽 중심 높이 mm}. 주면 그 높이로 시차를 보정한다 (calibrator 설명). 없는 종류는 0.
    part_limits: part_check.limits_from_params 결과. 주면 나사·너트 크기·구멍이 아닌 것을 unknown 으로 바꾼다.
    detector: yolo_detector.YoloDetector. 주면 규칙(전처리·윤곽·길이 비) 대신 이것으로 찾는다. roi_mask 는 상자 중심으로 거른다."""
    if calibrator.image_size is not None and calibrator.image_size != (frame.shape[1], frame.shape[0]):
        raise ValueError('입력 영상과 보정 사진의 해상도가 다릅니다. 같은 해상도로 다시 보정하세요.')
    if detector is not None and part_limits is not None:
        raise ValueError('YOLO 검출에는 part_limits 를 쓸 수 없습니다 (기준값이 규칙 윤곽용).')
    if roi_mask is not None and roi_mask.shape != frame.shape[:2]:
        raise ValueError('ROI 마스크와 입력 영상의 크기가 다릅니다.')
    debug = frame.copy()
    if roi_mask is not None:
        # 검출 영역 경계를 노란 선으로 표시 (GUI 에서 시트·마커 제외 영역이 맞게 잡혔는지 확인용)
        outline, _ = cv2.findContours(roi_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, outline, -1, (0, 220, 255), 1)
    if detector is not None:
        found = [(o.det, o.part_type, f'{o.conf:.2f}') for o in detector.detect(frame) if center_inside(o.det, roi_mask)]
    else:
        mask = preprocess(frame, blur_ksize, threshold, segmentation_mode=segmentation_mode,
                          hsv_max_saturation=hsv_max_saturation, hsv_max_dark_value=hsv_max_dark_value,
                          close_ksize=close_ksize, open_ksize=open_ksize)
        if roi_mask is not None:
            mask = cv2.bitwise_and(mask, roi_mask)
        found = [(det, classify_part(det, nut_max_aspect), None) for det in find_objects(mask, min_area_px)]
    objects = []
    for index, (det, part_type, note) in enumerate(found):
        if part_limits is not None:
            features = measure(mask, det, calibrator.px_to_mm if calibrator.status.ready else None)
            reason = reject_reason(part_type, features, part_limits)
            if reason:
                part_type, note = 'unknown', reason
        center = np.asarray(det.center_px)
        endpoint = center + np.asarray(det.long_axis_px) * det.length_px / 2
        x_mm = y_mm = angle = None
        if calibrator.status.ready:
            points_mm = calibrator.px_to_mm(np.array([center, endpoint]),
                                            (part_heights_mm or {}).get(part_type, 0.0))
            x_mm, y_mm = map(float, points_mm[0])
            delta = points_mm[1] - points_mm[0]
            if part_type == 'screw':
                angle = normalize_angle_deg(math.degrees(math.atan2(delta[1], delta[0])))
        objects.append(dict(id=index, part_type=part_type, x_mm=x_mm, y_mm=y_mm,
                            angle_deg=angle, u_px=float(center[0]), v_px=float(center[1])))
        cv2.polylines(debug, [np.rint(det.box_px).astype(np.int32)], True,
                      (150, 150, 150) if part_type == 'unknown' else (0, 255, 0), 2)
        start = tuple(np.rint(center).astype(int))
        cv2.circle(debug, start, 3, (0, 0, 255), -1)
        if part_type == 'screw':
            cv2.arrowedLine(debug, start, tuple(np.rint(endpoint).astype(int)), (255, 0, 0), 2)
        cv2.putText(debug, f'{index}: {part_type}' + (f' ({note})' if note else ''), start, cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 120, 255), 1)
    return make_pose_packet(seq, t_capture, calibrator.status.ready, objects), debug
