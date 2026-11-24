"""재학습한 YOLOv8-OBB 를 ROS 쪽 OpenCV(4.6) 로 추론한다 (4주차, 팀원 1). torch·ultralytics 가 필요 없다.

모델 파일은 scripts/export_yolo_onnx.py 로 만든다 (.onnx + 같은 이름 .json). 신경망 부분만 ONNX 이고 상자 계산은 여기서 한다:
OpenCV 4.6 이 Ultralytics 가 내보낸 ONNX 의 마지막 상자 계산 부분을 읽지 못한다 (forward 에서 assertion 오류).

너트 상자의 각도는 믿을 수 없다 (45° 부근으로 몰림, WEEK4 동경 11-24). 패킷은 원래 너트 각도를 null 로 보낸다.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from vision_pick_place.vision_detector import Detection

OUTPUTS = ('/model.22/Concat_output_0', '/model.22/Concat_1_output_0', '/model.22/Concat_2_output_0')   # 거리 분포, 종류, 각도
STRIDES = (8, 16, 32)
REG_MAX = 16          # 변까지 거리를 나누어 예측하는 칸 수 (YOLOv8 고정값)
NMS_IOU = 0.7         # Ultralytics 기본값. finetune_obb.py 로 채점할 때와 같다
PAD_VALUE = 114
PART_TYPES = {'screw', 'nut'}


@dataclass
class YoloObject:
    det: Detection
    part_type: str
    conf: float


def detector_from_params(params: dict) -> 'YoloDetector | None':
    """설정 → 검출기. detector 가 'rule'(기본)이면 None (process_frame 이 규칙으로 찾는다)."""
    kind = params.get('detector', 'rule')
    if kind == 'rule':
        return None
    if kind != 'yolo':
        raise ValueError(f"detector 는 'rule' 또는 'yolo' 여야 합니다: {kind!r}")
    if params.get('reject_unknown', False):
        raise ValueError('detector: yolo 에서는 reject_unknown 을 쓸 수 없습니다 (기준값이 규칙 윤곽용). 하나만 켜세요.')
    return YoloDetector(params.get('yolo_model_path', ''), float(params.get('yolo_conf', 0.3)))


class YoloDetector:
    def __init__(self, model_path, conf: float = 0.3):
        path = Path(model_path)
        meta_path = path.with_suffix('.json')
        if not path.is_file() or not meta_path.is_file():
            raise FileNotFoundError(f'YOLO 모델이 없습니다: {path} 와 {meta_path.name}. scripts/export_yolo_onnx.py 로 만들거나 받아서 넣으세요.')
        if not 0 < conf < 1:
            raise ValueError('yolo_conf 는 0 과 1 사이여야 합니다.')
        meta = json.loads(meta_path.read_text())
        self.names = list(meta['names'])
        if not set(self.names) <= PART_TYPES:
            raise ValueError(f'나사·너트 모델이 아닙니다: {self.names}')
        self.width, self.height = meta['input_size']
        self.conf = conf
        self.net = cv2.dnn.readNetFromONNX(str(path))
        self.anchors, self.strides = make_anchors(self.width, self.height)

    def detect(self, frame: np.ndarray) -> list[YoloObject]:
        """BGR 영상 → 찾은 부품 (위 → 아래 순서, 규칙 검출과 같음)."""
        blob, scale, pad = letterbox(frame, self.width, self.height)
        self.net.setInput(blob)
        dist, cls, angle = self.net.forward(OUTPUTS)
        boxes, conf, label = decode(dist[0], cls[0], angle[0], self.anchors, self.strides, self.conf)
        keep = nms(boxes, conf)
        objects = []
        for i in keep:
            x, y, w, h, r = boxes[i]
            corners = box_corners((x - pad[0]) / scale, (y - pad[1]) / scale, w / scale, h / scale, r)
            det = to_detection(corners)
            if det is not None:
                objects.append(YoloObject(det, self.names[int(label[i])], float(conf[i])))
        return sorted(objects, key=lambda o: (o.det.center_px[1], o.det.center_px[0]))


def letterbox(frame: np.ndarray, width: int, height: int):
    """비율을 지켜 줄이고 남는 곳을 회색으로 채운다 (Ultralytics 와 같은 방식). → 입력 blob, 배율, (왼쪽, 위) 여백."""
    h, w = frame.shape[:2]
    scale = min(width / w, height / h)
    new_w, new_h = max(round(w * scale), 1), max(round(h * scale), 1)
    image = frame if (new_w, new_h) == (w, h) else cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    left, top = round((width - new_w) / 2 - 0.1), round((height - new_h) / 2 - 0.1)
    image = cv2.copyMakeBorder(image, top, height - new_h - top, left, width - new_w - left,
                               cv2.BORDER_CONSTANT, value=(PAD_VALUE,) * 3)
    return cv2.dnn.blobFromImage(image, 1 / 255.0, swapRB=True), scale, (left, top)


def make_anchors(width: int, height: int):
    """예측 칸의 중심 (칸 단위) 과 칸 크기(px). 모델 출력과 같은 순서."""
    points, strides = [], []
    for s in STRIDES:
        x, y = np.meshgrid(np.arange(width // s) + 0.5, np.arange(height // s) + 0.5)
        points.append(np.stack([x.ravel(), y.ravel()], 1))
        strides.append(np.full(x.size, s))
    return np.concatenate(points).astype(np.float32), np.concatenate(strides).astype(np.float32)


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -60, 60)))


def decode(dist, cls, angle, anchors, strides, min_conf):
    """신경망 출력 → (N, 5) 상자 (중심 x, y, w, h, 각도 rad; 입력 영상 px), 신뢰도, 종류 번호. min_conf 이상만."""
    if dist.shape != (4 * REG_MAX, len(anchors)):
        raise ValueError(f'YOLO 출력 크기가 다릅니다: {dist.shape}')
    score = sigmoid(cls)
    conf, label = score.max(0), score.argmax(0)
    pick = np.flatnonzero(conf > min_conf)
    logits = dist[:, pick].reshape(4, REG_MAX, -1)
    prob = np.exp(logits - logits.max(1, keepdims=True))
    prob /= prob.sum(1, keepdims=True)
    left, top, right, bottom = (prob * np.arange(REG_MAX)[None, :, None]).sum(1)
    r = (sigmoid(angle[0, pick]) - 0.25) * np.pi
    xf, yf, cos, sin = (right - left) / 2, (bottom - top) / 2, np.cos(r), np.sin(r)
    s = strides[pick]
    boxes = np.stack([(xf * cos - yf * sin + anchors[pick, 0]) * s, (xf * sin + yf * cos + anchors[pick, 1]) * s,
                      (left + right) * s, (top + bottom) * s, r], 1)
    return boxes, conf[pick], label[pick]


def probiou(a, b, eps=1e-7):
    """회전 상자끼리의 겹침 (상자를 정규분포로 보고 계산, Ultralytics batch_probiou 와 같은 식). (N, 5), (M, 5) → (N, M)."""
    def cov(box):
        w, h, r = box[:, 2] ** 2 / 12, box[:, 3] ** 2 / 12, box[:, 4]
        c, s = np.cos(r), np.sin(r)
        return w * c * c + h * s * s, w * s * s + h * c * c, (w - h) * c * s
    (a1, b1, c1), (a2, b2, c2) = (tuple(v[:, None] for v in cov(a)), cov(b))
    dx, dy = b[:, 0] - a[:, :1], a[:, 1:2] - b[:, 1]
    det = (a1 + a2) * (b1 + b2) - (c1 + c2) ** 2
    t1 = ((a1 + a2) * dy ** 2 + (b1 + b2) * dx ** 2) / (det + eps) * 0.25
    t2 = (c1 + c2) * dx * dy / (det + eps) * 0.5
    area = np.sqrt(np.maximum(a1 * b1 - c1 ** 2, 0) * np.maximum(a2 * b2 - c2 ** 2, 0))
    t3 = np.log(det / (4 * area + eps) + eps) * 0.5
    return 1 - np.sqrt(1 - np.exp(-np.clip(t1 + t2 + t3, eps, 100)) + eps)


def nms(boxes, conf, iou=NMS_IOU) -> list[int]:
    """겹치는 상자 중 신뢰도가 높은 것만. 종류를 가리지 않아 한 물체에 나사·너트가 같이 나오지 않는다."""
    if len(boxes) == 0:
        return []
    order = np.argsort(-conf, kind='stable')
    overlap = np.triu(probiou(boxes[order].astype(float), boxes[order].astype(float)), 1)
    return [int(i) for i in order[~(overlap > iou).any(0)]]


def box_corners(x, y, w, h, r) -> np.ndarray:
    cos, sin = np.cos(r), np.sin(r)
    a, b = np.array([w / 2 * cos, w / 2 * sin]), np.array([-h / 2 * sin, h / 2 * cos])
    c = np.array([x, y])
    return np.array([c + a + b, c + a - b, c - a - b, c - a + b], np.float32)


def to_detection(corners: np.ndarray) -> Detection | None:
    """꼭짓점 4개 → 규칙 검출과 같은 Detection (윤곽은 없음)."""
    edges = np.roll(corners, -1, axis=0) - corners
    lengths = np.linalg.norm(edges, axis=1)
    i = int(np.argmax(lengths))
    length, width = float(lengths[i]), float(lengths.min())
    if width <= 0:
        return None
    axis = edges[i] / length
    if axis[0] < 0 or (axis[0] == 0 and axis[1] < 0):
        axis = -axis
    center = corners.mean(axis=0)
    return Detection((float(center[0]), float(center[1])), corners, (float(axis[0]), float(axis[1])),
                     length, width, length * width)
