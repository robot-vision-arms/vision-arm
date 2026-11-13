"""ROS 없이 검증하는 프레임 처리. 미보정 좌표는 null로 보내 mm로 오인하지 않게 한다."""
import math

import cv2
import numpy as np

from vision_pick_place.camera_stream import preprocess
from vision_pick_place.packet import make_pose_packet
from vision_pick_place.vision_detector import classify_part, find_objects, normalize_angle_deg


def process_frame(frame, calibrator, seq, t_capture, *, blur_ksize=5, threshold=-1,
                  min_area_px=500, nut_max_aspect=1.5):
    if calibrator.image_size is not None and calibrator.image_size != (frame.shape[1], frame.shape[0]):
        raise ValueError('입력 영상과 보정 사진의 해상도가 다릅니다. 같은 해상도로 다시 보정하세요.')
    mask = preprocess(frame, blur_ksize, threshold)
    detections = find_objects(mask, min_area_px)
    debug = frame.copy()
    objects = []
    for index, det in enumerate(detections):
        part_type = classify_part(det, nut_max_aspect)
        center = np.asarray(det.center_px)
        endpoint = center + np.asarray(det.long_axis_px) * det.length_px / 2
        x_mm = y_mm = angle = None
        if calibrator.status.ready:
            points_mm = calibrator.px_to_mm(np.array([center, endpoint]))
            x_mm, y_mm = map(float, points_mm[0])
            delta = points_mm[1] - points_mm[0]
            if part_type == 'screw':
                angle = normalize_angle_deg(math.degrees(math.atan2(delta[1], delta[0])))
        objects.append(dict(id=index, part_type=part_type, x_mm=x_mm, y_mm=y_mm,
                            angle_deg=angle, u_px=float(center[0]), v_px=float(center[1])))
        cv2.polylines(debug, [np.rint(det.box_px).astype(np.int32)], True, (0, 255, 0), 2)
        start = tuple(np.rint(center).astype(int))
        cv2.circle(debug, start, 3, (0, 0, 255), -1)
        if part_type == 'screw':
            cv2.arrowedLine(debug, start, tuple(np.rint(endpoint).astype(int)), (255, 0, 0), 2)
        cv2.putText(debug, f'{index}: {part_type}', start, cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 120, 255), 1)
    return make_pose_packet(seq, t_capture, calibrator.status.ready, objects), debug
