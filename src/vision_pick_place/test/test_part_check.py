"""unknown 거르기(part_check): 크기·구멍 측정, 기준값, 파이프라인, 기본은 꺼짐."""
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from vision_pick_place.kpi import load_vision_params
from vision_pick_place.part_check import Features, limits_from_params, measure, reject_reason
from vision_pick_place.vision_detector import find_objects
from vision_pick_place.vision_pipeline import process_frame

ON = dict(reject_unknown=True, screw_length_mm=[20, 60], screw_width_mm=[5, 14], nut_length_mm=[10, 20], nut_width_mm=[9, 18],
          nut_min_hole_ratio=0.1)
K = 4.0     # px/mm


class Calib:
    """위에서 똑바로 본 K px/mm 카메라."""
    image_size = None

    def __init__(self, ready=True):
        self.status = SimpleNamespace(ready=ready)

    def px_to_mm(self, p, height=0.0):
        return np.asarray(p, float) / K


def scene():
    """나사 40×8, 구멍 있는 너트 지름 13, 막힌 원 지름 13(볼트 머리), 큰 튜브 80×25 mm."""
    img = np.full((600, 900, 3), 255, np.uint8)
    cv2.rectangle(img, (100, 100), (100 + int(40 * K), 100 + int(8 * K)), (0, 0, 0), -1)
    cv2.circle(img, (500, 120), int(6.5 * K), (0, 0, 0), -1)
    cv2.circle(img, (500, 120), int(4 * K), (255, 255, 255), -1)
    cv2.circle(img, (700, 120), int(6.5 * K), (0, 0, 0), -1)
    cv2.rectangle(img, (100, 350), (100 + int(80 * K), 350 + int(25 * K)), (0, 0, 0), -1)
    return img


def kinds(limits, calib):
    packet, debug = process_frame(scene(), calib, 0, 0.0, part_limits=limits)
    assert debug.shape == scene().shape
    return {(round(o['u_px'], -1), round(o['v_px'], -1)): o for o in packet['objects']}


def test_measure_size_and_hole():
    mask = cv2.threshold(cv2.cvtColor(scene(), cv2.COLOR_BGR2GRAY), 127, 255, cv2.THRESH_BINARY_INV)[1]
    by_x = {round(d.center_px[0], -1): measure(mask, d, lambda p: p / K) for d in find_objects(mask) if d.center_px[1] < 300}
    assert (by_x[180].length_mm, by_x[180].width_mm) == pytest.approx((40, 8), abs=0.3) and by_x[180].hole_ratio == 0
    assert by_x[500].hole_ratio == pytest.approx((4 / 6.5) ** 2, abs=0.05) and by_x[700].hole_ratio == 0
    assert measure(mask, find_objects(mask)[0], None).length_mm is None


def test_default_is_off_and_keeps_old_result():
    assert limits_from_params({}) is None and limits_from_params(dict(ON, reject_unknown=False)) is None
    assert sorted(o['part_type'] for o in kinds(None, Calib()).values()) == ['nut', 'nut', 'screw', 'screw']


def test_non_parts_become_unknown_without_angle():
    found = kinds(limits_from_params(ON), Calib())
    assert [found[k]['part_type'] for k in ((180, 120), (500, 120), (700, 120), (260, 400))] == ['screw', 'nut', 'unknown', 'unknown']
    assert found[(260, 400)]['angle_deg'] is None and found[(180, 120)]['angle_deg'] is not None


def test_without_calibration_only_hole_is_checked():
    found = kinds(limits_from_params(ON), Calib(ready=False))
    assert found[(260, 400)]['part_type'] == 'screw' and found[(700, 120)]['part_type'] == 'unknown'


def test_zero_range_and_zero_hole_mean_not_checked():
    limits = limits_from_params(dict(reject_unknown=True, screw_length_mm=[0.0, 0.0]))
    assert reject_reason('screw', Features(500, 500, 0), limits) is None and reject_reason('nut', Features(500, 500, 0), limits) is None


@pytest.mark.parametrize('bad', [dict(screw_length_mm=[30, 20]), dict(nut_width_mm=[-1, 5]), dict(nut_min_hole_ratio=1.0)])
def test_bad_limits_are_rejected(bad):
    with pytest.raises(ValueError):
        limits_from_params(dict(ON, **bad))


def test_shipped_params_are_off():
    from pathlib import Path
    config = Path(__file__).resolve().parents[1] / 'config'
    assert limits_from_params(load_vision_params([config / 'params.yaml'])) is None
    assert limits_from_params(load_vision_params([config / 'params.yaml', config / 'params_green.yaml'])) is None
