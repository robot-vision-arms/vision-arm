"""YOLO-OBB 재학습용 라벨: 규칙 검출로 초안 → 사람이 확인 → 학습 데이터로 내보내기 (4주차, 팀원 1).

편집 창은 gui/label_obb.py. 여기는 화면 없이 도는 계산·저장만 둔다.

폴더:  <root>/{train,val,test}/*.png              사진 (live_view.py 로 저장한 원본)
       <root>/{train,val,test}/labels/<이름>.json  라벨 원본 {image, width, height, reviewed, boxes}
내보내기: <out>/images/<split>, <out>/labels/<split>, <out>/dataset.yaml  (Ultralytics OBB 형식)

규칙:
- reviewed(사람이 확인 완료) 가 아닌 사진이 하나라도 있으면 내보내지 않는다. 초안이 정답으로 섞이면
  "규칙이 맞힌 것만 배우는" 데이터가 되고, 빠뜨린 부품은 배경으로 학습된다.
- 나사·너트가 아닌 물체는 라벨하지 않는다 (배경으로 학습). 상자가 0개인 확인 완료 사진은 빈 라벨로 내보낸다.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np

from vision_pick_place import aruco_calib
from vision_pick_place.camera_stream import preprocess
from vision_pick_place.vision_detector import classify_part, find_objects
from vision_pick_place.workspace_roi import sheet_roi_mask

CLASSES = ('screw', 'nut')           # YOLO 클래스 번호 0, 1
SPLITS = ('train', 'val', 'test')
IMAGE_SUFFIXES = ('.png', '.jpg', '.jpeg')
MIN_SIDE_PX = 3.0                    # 이보다 얇은 상자는 잘못 그린 것으로 본다


@dataclass
class Box:
    """회전 상자. 픽셀 좌표, angle_deg 는 화면에서 시계 방향 (y 가 아래). w 는 angle 방향 길이."""
    part_type: str
    cx: float
    cy: float
    w: float
    h: float
    angle_deg: float

    def corners(self) -> np.ndarray:
        a = math.radians(self.angle_deg)
        ux, uy = np.array([math.cos(a), math.sin(a)]) * self.w / 2, np.array([-math.sin(a), math.cos(a)]) * self.h / 2
        c = np.array([self.cx, self.cy])
        return np.array([c - ux - uy, c + ux - uy, c + ux + uy, c - ux + uy])

    def to_local(self, point) -> np.ndarray:
        """화면 점 → 상자 기준 좌표 (중심 원점, w 축 = x)."""
        a = math.radians(self.angle_deg)
        dx, dy = point[0] - self.cx, point[1] - self.cy
        return np.array([dx * math.cos(a) + dy * math.sin(a), -dx * math.sin(a) + dy * math.cos(a)])

    def contains(self, point) -> bool:
        x, y = self.to_local(point)
        return abs(x) <= self.w / 2 and abs(y) <= self.h / 2


def box_from_corners(part_type: str, corners) -> Box:
    """사각형 꼭짓점 4개(순서대로) → Box. 긴 변을 w 로 둔다."""
    p = np.asarray(corners, float).reshape(4, 2)
    e0, e1 = p[1] - p[0], p[2] - p[1]
    n0, n1 = float(np.hypot(*e0)), float(np.hypot(*e1))
    axis, w, h = (e0, n0, n1) if n0 >= n1 else (e1, n1, n0)
    c = p.mean(axis=0)
    return Box(part_type, float(c[0]), float(c[1]), w, h, math.degrees(math.atan2(axis[1], axis[0])) % 180.0)


def box_from_clicks(p0, p1, p2, part_type: str | None = None) -> Box:
    """장축 양 끝 두 점 + 폭을 정하는 점 하나 → Box. 종류를 안 주면 길이 비로 정한다 (규칙과 같은 1.5)."""
    p0, p1, p2 = (np.asarray(p, float) for p in (p0, p1, p2))
    d = p1 - p0
    length = float(np.hypot(*d))
    if length < MIN_SIDE_PX:
        raise ValueError('장축 두 점이 너무 가깝습니다.')
    normal = np.array([-d[1], d[0]]) / length
    half = abs(float((p2 - p0) @ normal))     # 장축에서 세 번째 점까지 = 폭의 절반
    if 2 * half < MIN_SIDE_PX:
        raise ValueError('폭이 너무 좁습니다.')
    c = (p0 + p1) / 2
    kind = part_type or ('nut' if max(length, 2 * half) / min(length, 2 * half) <= 1.5 else 'screw')
    return Box(kind, float(c[0]), float(c[1]), length, 2 * half, math.degrees(math.atan2(d[1], d[0])) % 180.0)


def resize_by_corner(box: Box, index: int, point) -> Box:
    """index 번 꼭짓점을 point 로 끈다. 반대쪽 꼭짓점은 고정, 각도는 그대로."""
    opposite = box.corners()[(index + 2) % 4]
    a = math.radians(box.angle_deg)
    d = np.asarray(point, float) - opposite
    x, y = d[0] * math.cos(a) + d[1] * math.sin(a), -d[0] * math.sin(a) + d[1] * math.cos(a)
    sx, sy = ((-1, -1), (1, -1), (1, 1), (-1, 1))[index]
    w, h = max(sx * x, MIN_SIDE_PX), max(sy * y, MIN_SIDE_PX)   # 뒤집히지 않게
    lx, ly = sx * w / 2, sy * h / 2
    c = opposite + np.array([lx * math.cos(a) - ly * math.sin(a), lx * math.sin(a) + ly * math.cos(a)])
    return Box(box.part_type, float(c[0]), float(c[1]), w, h, box.angle_deg)


class _SheetMap:
    """sheet_roi_mask 가 필요로 하는 만큼만 흉내 낸 보정. 사진 속 마커로 구하므로 보정 파일이 없어도 되고 시트가 움직여도 된다.

    렌즈 왜곡은 무시한다 (초안을 찾을 영역만 정하므로 몇 px 차이는 괜찮다).
    """
    class status:
        ready = True

    def __init__(self, H):
        self.inverse = np.linalg.inv(H)

    def mm_to_px(self, points_mm):
        return cv2.perspectiveTransform(np.asarray(points_mm, float).reshape(-1, 1, 2), self.inverse).reshape(-1, 2)


def draft_boxes(frame: np.ndarray, params: dict) -> tuple[list[Box], str]:
    """vision_node 와 같은 규칙 검출로 초안 상자를 만든다. (상자 목록, 사람에게 보일 안내)"""
    options = {k: params[k] for k in ('segmentation_mode', 'hsv_max_saturation', 'hsv_max_dark_value',
                                      'close_ksize', 'open_ksize') if k in params}
    mask = preprocess(frame, params.get('blur_ksize', 5), params.get('threshold', -1), **options)
    try:
        H, _, ids = aruco_calib.sheet_homography(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
        roi = sheet_roi_mask(_SheetMap(H), (frame.shape[1], frame.shape[0]),
                             params.get('sheet_edge_margin_mm', 3.0), params.get('sheet_marker_margin_mm', 5.0))
        mask = cv2.bitwise_and(mask, roi)
        note = f'마커 {len(ids)}개'
    except ValueError:
        note = '마커를 3개 이상 못 찾아 사진 전체에서 초안을 만들었습니다 (시트 밖 오검출 주의)'
    boxes = []
    for det in find_objects(mask, params.get('min_area_px', 500)):
        kind = classify_part(det, params.get('nut_max_aspect', 1.5))
        if kind in CLASSES:
            boxes.append(box_from_corners(kind, det.box_px))
    return boxes, f'{note}, 초안 {len(boxes)}개'


def label_path(image: Path) -> Path:
    return image.parent / 'labels' / (image.stem + '.json')


def list_images(folder: Path) -> list[Path]:
    if not folder.is_dir():
        return []
    images = sorted(p for p in folder.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    seen = {}
    for p in images:   # a.png 와 a.jpg 는 같은 라벨 파일을 쓰게 된다
        if p.stem in seen:
            raise ValueError(f'이름이 같은 사진: {seen[p.stem].name}, {p.name}. 하나의 이름을 바꾸세요.')
        seen[p.stem] = p
    return images


def save_label(image: Path, size: tuple[int, int], boxes: list[Box], reviewed: bool) -> None:
    path = label_path(image)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = dict(image=image.name, width=size[0], height=size[1], reviewed=bool(reviewed), boxes=[asdict(b) for b in boxes])
    tmp = path.with_suffix('.json.tmp')     # 저장 중 꺼져도 예전 라벨이 남게
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=1))
    os.replace(tmp, path)


def load_label(image: Path) -> dict | None:
    """없으면 None. 있으면 {'boxes': [Box], 'reviewed': bool, 'width', 'height'}."""
    path = label_path(image)
    if not path.is_file():
        return None
    data = json.loads(path.read_text())
    data['boxes'] = [Box(**b) for b in data['boxes']]
    unknown = {b.part_type for b in data['boxes']} - set(CLASSES)
    if unknown:
        raise ValueError(f'{path}: 모르는 종류 {sorted(unknown)}')
    return data


def yolo_lines(boxes: list[Box], width: int, height: int) -> list[str]:
    """'클래스 x1 y1 … x4 y4' (0~1). 사진 밖으로 나간 꼭짓점은 잘라 넣는다 (1 을 넘으면 Ultralytics 가 그 사진을 버린다)."""
    lines = []
    for b in boxes:
        xy = np.clip(b.corners() / [width, height], 0.0, 1.0)
        lines.append(f'{CLASSES.index(b.part_type)} ' + ' '.join(f'{v:.6f}' for v in xy.ravel()))
    return lines


def check_dataset(root: Path) -> tuple[dict, list[str]]:
    """(split → [(사진, 라벨)], 문제 목록). 문제가 있으면 내보내지 않는다."""
    problems, data, hashes = [], {}, {}
    for split in SPLITS:
        data[split] = []
        for image in list_images(root / split):
            label = load_label(image)
            if label is None or not label['reviewed']:
                problems.append(f'확인 완료 아님: {split}/{image.name}')
                continue
            digest = hashlib.md5(image.read_bytes()).hexdigest()
            if digest in hashes:
                problems.append(f'같은 사진이 두 번 들어 있음: {hashes[digest]}, {split}/{image.name}')
            hashes[digest] = f'{split}/{image.name}'
            data[split].append((image, label))
    for split in ('train', 'val'):
        if not list_images(root / split):
            problems.append(f'{split} 사진이 없습니다.')
    return data, problems


def export_dataset(root: Path, out: Path) -> dict:
    """확인 완료 라벨을 Ultralytics OBB 데이터로. 반환: split → {images, screw, nut}. 문제가 있으면 ValueError."""
    root, out = Path(root).resolve(), Path(out).resolve()
    data, problems = check_dataset(root)
    if problems:
        raise ValueError('\n'.join(problems))
    counts = {}
    for split, items in data.items():
        img_dir, lbl_dir = out / 'images' / split, out / 'labels' / split
        img_dir.mkdir(parents=True, exist_ok=True)
        lbl_dir.mkdir(parents=True, exist_ok=True)
        # 지난번에 내보낸 것만 지운다: 여기서 만든 심볼릭 링크와 라벨 txt, Ultralytics 캐시(옛 라벨을 다시 쓰는 원인)
        for old in img_dir.iterdir():
            if old.is_symlink():
                old.unlink()
        for old in lbl_dir.glob('*.txt'):
            old.unlink()
        (out / 'labels' / f'{split}.cache').unlink(missing_ok=True)
        counts[split] = dict(images=0, **{c: 0 for c in CLASSES})
        for image, label in items:
            (img_dir / image.name).symlink_to(image)
            lines = yolo_lines(label['boxes'], label['width'], label['height'])
            (lbl_dir / (image.stem + '.txt')).write_text(''.join(line + '\n' for line in lines))
            counts[split]['images'] += 1
            for b in label['boxes']:
                counts[split][b.part_type] += 1
    yaml = f'path: {out}\ntrain: images/train\nval: images/val\n'
    if counts['test']['images']:
        yaml += 'test: images/test\n'
    (out / 'dataset.yaml').write_text(yaml + 'names:\n' + ''.join(f'  {i}: {c}\n' for i, c in enumerate(CLASSES)))
    return counts
