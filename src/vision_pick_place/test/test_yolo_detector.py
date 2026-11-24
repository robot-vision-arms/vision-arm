"""YOLO 검출기: 모델 파일 없이 상자 계산을 확인하고, 모델이 있으면 원본(.pt) 기준값과 비교한다."""
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from vision_pick_place import yolo_detector as yd
from vision_pick_place.calibrator import Calibrator
from vision_pick_place.kpi import load_vision_params
from vision_pick_place.vision_pipeline import process_frame

ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / 'data/models/yolo_green.onnx'
REFERENCE = ROOT / 'data/models/yolo_green_reference.json'


def raw_outputs(anchors, items):
    """items: [(칸 번호, (왼, 위, 오른, 아래) 칸 단위 거리(정수), 각도 rad, 종류, 신뢰도)] → 신경망 출력 모양."""
    n = len(anchors)
    dist, cls, ang = np.zeros((64, n), np.float32), np.full((2, n), -20, np.float32), np.zeros((1, n), np.float32)
    logit = lambda p: np.log(p / (1 - p))
    for i, sides, angle, label, conf in items:
        for side, value in enumerate(sides):
            dist[side * 16 + value, i] = 50          # 그 칸에 몰아주면 기댓값 = value
        cls[label, i] = logit(conf)
        ang[0, i] = logit(angle / np.pi + 0.25)
    return dist, cls, ang


def test_decode_gives_rotated_box_in_input_pixels():
    anchors, strides = yd.make_anchors(64, 64)
    assert len(anchors) == 64 + 16 + 4
    i = 64 + 5                                        # 두 번째 층(칸 16 px) 의 (1, 1) 칸
    assert tuple(anchors[i]) == (1.5, 1.5) and strides[i] == 16
    angle = np.deg2rad(30)
    boxes, conf, label = yd.decode(*raw_outputs(anchors, [(i, (1, 2, 5, 2), angle, 1, 0.8), (0, (1, 1, 1, 1), 0.1, 0, 0.2)]),
                                   anchors, strides, 0.3)
    assert len(boxes) == 1 and label[0] == 1 and conf[0] == pytest.approx(0.8, abs=1e-4)
    # 칸 중심에서 (오른 − 왼)/2 = 2 칸을 30° 돌린 만큼 옮긴다
    expected = [(1.5 + 2 * np.cos(angle)) * 16, (1.5 + 2 * np.sin(angle)) * 16, 6 * 16, 4 * 16, angle]
    assert boxes[0] == pytest.approx(expected, abs=1e-2)


def test_decode_rejects_other_model_shape():
    anchors, strides = yd.make_anchors(64, 64)
    with pytest.raises(ValueError, match='크기'):
        yd.decode(np.zeros((64, 3)), np.zeros((2, 3)), np.zeros((1, 3)), anchors, strides, 0.3)


def test_probiou_and_nms_keep_best_of_overlapping_any_type():
    boxes = np.array([[50, 50, 40, 10, 0.2], [51, 50, 40, 10, 0.2], [50, 50, 10, 40, 0.2], [200, 50, 40, 10, 0.2]], float)
    same = yd.probiou(boxes, boxes)
    assert np.diag(same) == pytest.approx(1, abs=1e-2)
    assert same == pytest.approx(same.T, abs=1e-9)
    assert same[0, 1] > 0.7 > same[0, 2] > same[0, 3]
    assert yd.nms(boxes, np.array([0.5, 0.9, 0.6, 0.4])) == [1, 2, 3]
    assert yd.nms(np.zeros((0, 5)), np.zeros(0)) == []


def test_letterbox_keeps_ratio_and_center():
    blob, scale, (left, top) = yd.letterbox(np.full((480, 640, 3), (0, 0, 255), np.uint8), 1024, 576)
    assert blob.shape == (1, 3, 576, 1024)
    assert scale == pytest.approx(1.2) and (left, top) == (128, 0)
    assert blob[0, :, 288, 512] == pytest.approx([1, 0, 0])                  # BGR 빨강 → RGB
    assert blob[0, :, 288, 10] == pytest.approx([114 / 255] * 3)


class FakeNet:
    def __init__(self, outputs):
        self.outputs = outputs

    def setInput(self, blob):
        pass

    def forward(self, names):
        return [o[None] for o in self.outputs]


def fake_detector(items, conf=0.3):
    d = yd.YoloDetector.__new__(yd.YoloDetector)
    d.names, d.width, d.height, d.conf = ['screw', 'nut'], 320, 192, conf
    d.anchors, d.strides = yd.make_anchors(320, 192)
    d.net = FakeNet(raw_outputs(d.anchors, items))
    return d


def anchor_index(x, y):
    return y * 40 + x      # 첫 층(칸 8 px), 320 px 너비


def scene_detector():
    return fake_detector([(anchor_index(10, 10), (6, 1, 6, 1), np.deg2rad(20), 0, 0.9),      # 나사
                          (anchor_index(30, 4), (2, 2, 2, 2), 0.1, 1, 0.8)])                 # 너트


def calibration(tmp_path):
    np.savez(tmp_path / 'i.npz', camera_matrix=np.array([[500., 0, 320], [0, 500., 192], [0, 0, 1]]), dist_coeffs=np.zeros(5), rms=0.)
    np.savez(tmp_path / 'h.npz', H=np.array([[.5, 0, 0], [0, -.5, 200], [0, 0, 1]]), reproj_err_mm=0.)
    return Calibrator(str(tmp_path / 'i.npz'), str(tmp_path / 'h.npz'))


def test_process_frame_with_detector_fills_packet(tmp_path):
    frame = np.full((384, 640, 3), 60, np.uint8)                              # 입력의 2배 크기
    packet, debug = process_frame(frame, calibration(tmp_path), 3, 1.5, detector=scene_detector())
    nut, screw = packet['objects']                                            # 위 → 아래 순서
    assert [nut['id'], screw['id']] == [0, 1]
    assert (nut['part_type'], screw['part_type']) == ('nut', 'screw')
    assert nut['angle_deg'] is None                                           # 너트 각도는 보내지 않는다
    assert (screw['u_px'], screw['v_px']) == pytest.approx((168, 168), abs=1e-2)
    assert (screw['x_mm'], screw['y_mm']) == pytest.approx((84, 116), abs=1e-2)
    assert screw['angle_deg'] == pytest.approx(160, abs=0.1)                  # 영상은 y 가 아래라 20° → 160°
    assert set(screw) == {'id', 'part_type', 'x_mm', 'y_mm', 'angle_deg', 'u_px', 'v_px'}   # 패킷 형식 그대로
    assert not np.array_equal(debug, frame)


def test_process_frame_with_detector_respects_roi_and_calibration(tmp_path):
    frame = np.full((384, 640, 3), 60, np.uint8)
    roi = np.zeros((384, 640), np.uint8)
    roi[:, :320] = 255                                                        # 너트(오른쪽)는 밖
    packet, _ = process_frame(frame, Calibrator('none', 'none'), 0, 0, detector=scene_detector(), roi_mask=roi)
    assert [o['part_type'] for o in packet['objects']] == ['screw']
    assert packet['objects'][0]['x_mm'] is packet['objects'][0]['angle_deg'] is None
    with pytest.raises(ValueError, match='part_limits'):
        process_frame(frame, Calibrator('none', 'none'), 0, 0, detector=scene_detector(), part_limits={})
    with pytest.raises(ValueError, match='ROI'):
        process_frame(frame, Calibrator('none', 'none'), 0, 0, detector=scene_detector(), roi_mask=roi[:10])


def test_detector_from_params(tmp_path):
    assert yd.detector_from_params({}) is None
    assert yd.detector_from_params({'detector': 'rule'}) is None
    with pytest.raises(ValueError, match='detector'):
        yd.detector_from_params({'detector': 'svm'})
    with pytest.raises(FileNotFoundError, match='export_yolo_onnx'):
        yd.detector_from_params({'detector': 'yolo', 'yolo_model_path': str(tmp_path / 'none.onnx')})
    with pytest.raises(ValueError, match='reject_unknown'):
        yd.detector_from_params({'detector': 'yolo', 'reject_unknown': True})


def test_default_params_keep_rule_and_overlay_turns_yolo_on():
    config = ROOT / 'src/vision_pick_place/config'
    files = [config / 'params.yaml', config / 'params_green.yaml']
    assert load_vision_params(files)['detector'] == 'rule'
    merged = load_vision_params([*files, config / 'params_yolo.yaml'])
    assert merged['detector'] == 'yolo' and merged['sheet_marker_margin_mm'] == 9.0


@pytest.mark.skipif(not (MODEL.is_file() and REFERENCE.is_file()), reason='모델 파일 없음 (git 제외)')
def test_model_matches_ultralytics_reference():
    ref = json.loads(REFERENCE.read_text())
    name, expected = next(iter(ref['objects'].items()))
    frame = cv2.imread(str(Path(ref['images']) / name))
    if frame is None:
        pytest.skip('기준 사진 없음 (git 제외)')
    found = yd.YoloDetector(MODEL, ref['conf']).detect(frame)
    assert len(found) == len(expected)
    for e in expected:
        c = np.array(e['corners'])
        o = min(found, key=lambda o: np.hypot(*(np.array(o.det.center_px) - c.mean(0))))
        assert o.part_type == e['part_type'] and o.conf == pytest.approx(e['conf'], abs=0.01)
        assert np.linalg.norm(c[:, None] - o.det.box_px[None], axis=2).min(1).max() < 0.5
