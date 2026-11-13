import cv2
import numpy as np
import pytest

from vision_pick_place.vision_detector import Detection, angle_error_deg, classify_part, find_objects, normalize_angle_deg


def test_normalize_angle_wraps_to_0_180():
    assert normalize_angle_deg(190.0) == 10.0
    assert normalize_angle_deg(-30.0) == 150.0


def test_angle_error_is_symmetric_across_180():
    assert angle_error_deg(179.0, 1.0) == 2.0
    assert angle_error_deg(0.0, 90.0) == 90.0


def _det(length, width):
    return Detection((0.0, 0.0), np.zeros((4, 2)), (1.0, 0.0), length, width, length * width)


def test_classify_part_by_aspect_ratio():
    assert classify_part(_det(40.0, 8.0)) == 'screw'
    assert classify_part(_det(13.0, 12.0)) == 'nut'
    assert classify_part(_det(10.0, 0.0)) == 'unknown'


@pytest.mark.parametrize('angle', [0, 25, 89, 135])
def test_rotated_object_center_axis_and_classification(angle):
    mask = np.zeros((240, 320), dtype=np.uint8)
    box = cv2.boxPoints(((160, 120), (100, 20), angle))
    cv2.fillConvexPoly(mask, np.rint(box).astype(np.int32), 255)
    cv2.rectangle(mask, (5, 5), (8, 8), 255)  # 면적 필터로 작은 잡음 제외
    detections = find_objects(mask)
    assert len(detections) == 1
    det = detections[0]
    assert det.center_px == pytest.approx((160, 120), abs=1)
    assert det.length_px == pytest.approx(100, abs=2)
    assert det.width_px == pytest.approx(20, abs=2)
    axis_angle = np.degrees(np.arctan2(det.long_axis_px[1], det.long_axis_px[0]))
    assert angle_error_deg(axis_angle, angle) < 1
    assert classify_part(det) == 'screw'


def test_nut_hole_is_not_a_second_object_and_empty_frame_is_empty():
    mask = np.zeros((100, 100), dtype=np.uint8)
    assert find_objects(mask) == []
    cv2.circle(mask, (50, 50), 20, 255, -1)
    cv2.circle(mask, (50, 50), 8, 0, -1)
    detections = find_objects(mask)
    assert len(detections) == 1
    assert classify_part(detections[0]) == 'nut'


def test_degenerate_contour_and_classification_boundary():
    mask = np.zeros((100, 100), dtype=np.uint8)
    cv2.line(mask, (10, 10), (80, 80), 255, 1)
    assert find_objects(mask, min_area_px=0) == []
    assert classify_part(_det(15, 10)) == 'nut'
    assert classify_part(_det(float('nan'), 10)) == 'unknown'
