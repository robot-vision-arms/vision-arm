import numpy as np
import pytest

from vision_pick_place.vision_detector import Detection, angle_error_deg, classify_part, normalize_angle_deg


def test_normalize_angle_wraps_to_0_180():
    assert normalize_angle_deg(190.0) == 10.0
    assert normalize_angle_deg(-30.0) == 150.0


def test_angle_error_is_symmetric_across_180():
    assert angle_error_deg(179.0, 1.0) == 2.0
    assert angle_error_deg(0.0, 90.0) == 90.0


def _det(length, width):
    return Detection((0.0, 0.0), np.zeros((4, 2)), (1.0, 0.0), length, width, length * width)


@pytest.mark.skip(reason='classify_part 구현 전 (2주차). 구현하면 이 줄을 지운다')
def test_classify_part_by_aspect_ratio():
    assert classify_part(_det(40.0, 8.0)) == 'screw'
    assert classify_part(_det(13.0, 12.0)) == 'nut'
    assert classify_part(_det(10.0, 0.0)) == 'unknown'
