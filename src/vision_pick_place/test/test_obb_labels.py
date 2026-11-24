"""라벨 도구 확인: 상자 계산, 확인 완료만 내보내기, 초안, 편집 창."""
import importlib.util
import math
import os

import cv2
import numpy as np
import pytest

from vision_pick_place import aruco_calib
from vision_pick_place import obb_labels as L

GREEN = dict(segmentation_mode='metal_hsv', hsv_max_saturation=70, hsv_max_dark_value=70, close_ksize=3, open_ksize=3,
             sheet_marker_margin_mm=9.0, sheet_edge_margin_mm=8.0, min_area_px=100)


def sheet_photo(k=4.0):
    """초록 시트를 위에서 찍은 그림 (k px/mm) + 나사 하나, 너트 하나."""
    w, h = (int(v * k) for v in aruco_calib.SHEET_MM)
    img = np.full((h, w, 3), (51, 153, 26), np.uint8)
    side = int(aruco_calib.SHEET_MARKER_MM * k)
    for i, (x, y) in aruco_calib.SHEET_MARKERS.items():
        c, r = int(x * k), int(h - (y + aruco_calib.SHEET_MARKER_MM) * k)
        q = int(8 * k)
        img[max(r - q, 0):r + side + q, max(c - q, 0):c + side + q] = 255
        img[r:r + side, c:c + side] = cv2.cvtColor(aruco_calib.marker_image(i, side), cv2.COLOR_GRAY2BGR)
    screw = cv2.boxPoints(((150 * k, 100 * k), (40 * k, 8 * k), 30.0))
    cv2.fillPoly(img, [np.rint(screw).astype(np.int32)], (190, 190, 190))
    cv2.circle(img, (int(200 * k), int(60 * k)), int(6.5 * k), (190, 190, 190), -1)
    return img


def test_corner_roundtrip_and_contains():
    b = L.Box('screw', 100, 50, 40, 8, 30)
    assert L.box_from_corners('screw', b.corners()).__dict__ == pytest.approx(b.__dict__)
    along = (100 + 19 * math.cos(math.radians(30)), 50 + 19 * math.sin(math.radians(30)))
    assert b.contains(along) and not b.contains((100, 70))


def test_three_clicks_make_box_and_guess_type():
    b = L.box_from_clicks((0, 0), (40, 0), (10, 5))
    assert (b.cx, b.cy, b.w, b.h, b.angle_deg, b.part_type) == (20, 0, 40, 10, 0, 'screw')
    assert L.box_from_clicks((0, 0), (12, 0), (3, 6)).part_type == 'nut'
    with pytest.raises(ValueError):
        L.box_from_clicks((0, 0), (0, 0), (1, 1))


@pytest.mark.parametrize('index', range(4))
def test_corner_drag_keeps_opposite_corner(index):
    b = L.Box('nut', 100, 80, 30, 20, 25)
    old = b.corners()
    target = old[index] + (old[index] - [b.cx, b.cy]) * 0.5     # 바깥으로 끌기
    new = L.resize_by_corner(b, index, target).corners()
    assert new[(index + 2) % 4] == pytest.approx(old[(index + 2) % 4])
    assert new[index] == pytest.approx(target)


def test_draft_finds_parts_but_not_markers():
    boxes, note = L.draft_boxes(sheet_photo(), GREEN)
    assert '마커 4개' in note
    assert sorted(b.part_type for b in boxes) == ['nut', 'screw']
    screw = next(b for b in boxes if b.part_type == 'screw')
    assert (screw.cx, screw.cy) == pytest.approx((600, 400), abs=2) and screw.angle_deg == pytest.approx(30, abs=2)


def test_draft_without_markers_warns():
    img = np.full((400, 600, 3), (51, 153, 26), np.uint8)
    cv2.circle(img, (300, 200), 25, (190, 190, 190), -1)
    boxes, note = L.draft_boxes(img, GREEN)
    assert len(boxes) == 1 and '못 찾아' in note


def make(root, split, name, value, boxes=(), reviewed=True, label=True):
    (root / split).mkdir(parents=True, exist_ok=True)
    path = root / split / name
    cv2.imwrite(str(path), np.full((100, 200, 3), value, np.uint8))
    if label:
        L.save_label(path, (200, 100), list(boxes), reviewed)
    return path


def test_export_only_when_everything_is_reviewed(tmp_path):
    root, out = tmp_path / 'green', tmp_path / 'out'
    make(root, 'train', 'a.png', 10, [L.Box('screw', 100, 50, 40, 8, 0)])
    make(root, 'train', 'empty.png', 20)                                       # 기타 물체만 있는 사진 = 빈 라벨
    draft = make(root, 'val', 'b.png', 30, [L.Box('nut', 195, 50, 20, 20, 0)], reviewed=False)
    with pytest.raises(ValueError, match='val/b.png'):
        L.export_dataset(root, out)
    assert not out.exists()
    L.save_label(draft, (200, 100), L.load_label(draft)['boxes'], True)
    counts = L.export_dataset(root, out)
    assert counts['train'] == dict(images=2, screw=1, nut=0) and counts['val'] == dict(images=1, screw=0, nut=1)
    assert (out / 'labels/train/a.txt').read_text().split() == ['0'] + [f'{v:.6f}' for v in (.4, .46, .6, .46, .6, .54, .4, .54)]
    assert (out / 'labels/train/empty.txt').read_text() == ''
    nut = [float(v) for v in (out / 'labels/val/b.txt').read_text().split()[1:]]
    assert max(nut) == 1.0                                                     # 사진 밖 꼭짓점은 잘린다
    assert 'test:' not in (out / 'dataset.yaml').read_text()


def test_export_again_drops_removed_photo_and_cache(tmp_path):
    root, out = tmp_path / 'green', tmp_path / 'out'
    a = make(root, 'train', 'a.png', 10)
    make(root, 'train', 'b.png', 20)
    make(root, 'val', 'c.png', 30)
    L.export_dataset(root, out)
    (out / 'labels/train.cache').write_text('old')
    a.unlink()
    L.export_dataset(root, out)
    assert [p.name for p in (out / 'images/train').iterdir()] == ['b.png']
    assert [p.name for p in (out / 'labels/train').iterdir()] == ['b.txt'] and not (out / 'labels/train.cache').exists()


def test_same_photo_in_two_splits_is_rejected(tmp_path):
    make(tmp_path, 'train', 'a.png', 10)
    make(tmp_path, 'val', 'b.png', 10)
    with pytest.raises(ValueError, match='같은 사진'):
        L.export_dataset(tmp_path, tmp_path / 'out')


def test_same_stem_is_rejected(tmp_path):
    make(tmp_path, 'train', 'a.png', 10)
    make(tmp_path, 'train', 'a.jpg', 20, label=False)
    with pytest.raises(ValueError, match='이름이 같은'):
        L.list_images(tmp_path / 'train')


@pytest.mark.skipif(importlib.util.find_spec('PySide6') is None, reason='PySide6 없음 (venv 에서 실행)')
def test_window_draft_edit_confirm_flow(tmp_path):
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    from PySide6.QtCore import QPoint, Qt
    from PySide6.QtTest import QTest
    from PySide6.QtWidgets import QApplication
    from vision_pick_place.gui.label_obb import LabelWindow
    app = QApplication.instance() or QApplication([])
    (tmp_path / 'train').mkdir()
    for name in ('a.png', 'b.png'):
        cv2.imwrite(str(tmp_path / 'train' / name), sheet_photo())
    a = tmp_path / 'train' / 'a.png'
    win = LabelWindow(tmp_path, GREEN)
    win.resize(1200, 800)
    win.show()
    app.processEvents()
    c = win.canvas
    assert len(c.boxes) == 2 and not win.reviewed

    def at(p):
        s = c.to_screen(p)
        return QPoint(round(s.x()), round(s.y()))

    win.confirm()                                                # 초안 확인 → a 저장, b 로 이동
    assert L.load_label(a)['reviewed'] and win.index == 1
    win.go(-1)
    assert win.index == 0 and win.reviewed
    nut = next(i for i, b in enumerate(c.boxes) if b.part_type == 'nut')
    old = c.boxes[nut].cx
    QTest.mousePress(c, Qt.LeftButton, pos=at((c.boxes[nut].cx, c.boxes[nut].cy)))
    QTest.mouseMove(c, at((c.boxes[nut].cx + 40, c.boxes[nut].cy)))
    QTest.mouseRelease(c, Qt.LeftButton, pos=at((old + 40, c.boxes[nut].cy)))
    assert c.boxes[nut].cx == pytest.approx(old + 40, abs=3)
    assert not L.load_label(a)['reviewed']                       # 고치면 확인 전으로 되돌아간다
    c.undo_last()
    assert c.boxes[nut].cx == pytest.approx(old)
    c.start_new()
    for p in ((300, 300), (400, 300), (350, 310)):
        QTest.mouseClick(c, Qt.LeftButton, pos=at(p))
    assert len(c.boxes) == 3 and c.current().part_type == 'screw'
    c.set_type('nut')
    assert c.current().part_type == 'nut'
    c.delete_selected()
    assert len(L.load_label(a)['boxes']) == 2
    win.close()
