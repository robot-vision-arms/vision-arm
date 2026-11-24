"""YOLO-OBB 라벨 편집 창 (4주차, 팀원 1). 초안은 규칙 검출, 사람이 고치고 [확인 완료] 한다. 계산·저장은 obb_labels.py.

    python3 -m vision_pick_place.gui.label_obb data/captures/green
    python3 -m vision_pick_place.gui.label_obb data/captures/green --status                          # 진행 상황만
    python3 -m vision_pick_place.gui.label_obb data/captures/green --export data/models/green_yolo_obb   # 학습 데이터로
"""
import argparse
import copy
import math
import sys
from pathlib import Path

import cv2
import numpy as np
from PySide6.QtCore import QPointF, Qt, Signal
from PySide6.QtGui import QBrush, QColor, QImage, QKeySequence, QPainter, QPen, QPolygonF, QShortcut
from PySide6.QtWidgets import (QApplication, QHBoxLayout, QLabel, QListWidget, QMainWindow, QPushButton,
                               QVBoxLayout, QWidget)

from vision_pick_place import obb_labels as L
from vision_pick_place.gui.main_window import configure_korean_font
from vision_pick_place.kpi import load_vision_params

ROOT_PARAMS = ['src/vision_pick_place/config/params.yaml', 'src/vision_pick_place/config/params_green.yaml']
COLORS = {'screw': QColor(255, 60, 60), 'nut': QColor(50, 150, 255)}
NAMES = {'screw': '나사', 'nut': '너트'}
HANDLE_PX = 7          # 손잡이 반지름 (화면 px)
ROTATE_GAP_PX = 28     # 회전 손잡이가 상자에서 떨어진 거리 (화면 px)
HELP = ('마우스: 상자 클릭 = 선택, 끌기 = 이동, 네모 손잡이 = 크기, 동그라미 손잡이 = 회전, 휠 = 확대, 오른쪽 끌기 = 화면 이동\n'
        'N = 새 상자(장축 양 끝 두 번 + 폭 한 번 클릭, Esc 취소)   1 = 나사   2 = 너트   Del = 삭제   Ctrl+Z = 되돌리기\n'
        '방향키 = 1 px 이동(Shift 5)   Q / E = 회전 1°(Shift 5)   F = 화면 맞춤   Enter = 확인 완료 + 다음   PgUp / PgDn = 이전 / 다음')


class Canvas(QWidget):
    edited = Signal()
    message = Signal(str)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(640, 400)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setMouseTracking(True)
        self.image = None
        self.size_px = (0, 0)
        self.boxes, self.selected, self.undo = [], -1, []
        self.scale, self.offset, self.fitted = 1.0, QPointF(0, 0), True
        self.drag = None        # (종류, 시작 점, 시작 상자)
        self.clicks = None      # 새 상자 그리는 중이면 점 목록
        self.hover = None

    # --- 데이터 ---
    def set_image(self, frame, boxes):
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        self.image = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888).copy()
        self.size_px = (w, h)
        self.boxes, self.selected, self.undo = boxes, -1, []
        self.drag = self.clicks = None
        self.fit()

    def push_undo(self):
        self.undo.append(copy.deepcopy(self.boxes))
        del self.undo[:-50]

    def undo_last(self):
        if self.undo:
            self.boxes, self.selected = self.undo.pop(), -1
            self.changed()

    def changed(self):
        self.edited.emit()
        self.update()

    def current(self):
        return self.boxes[self.selected] if 0 <= self.selected < len(self.boxes) else None

    # --- 좌표 ---
    def fit(self):
        if self.image is None:
            return
        w, h = self.size_px
        self.scale = min(self.width() / w, self.height() / h)
        self.offset = QPointF((self.width() - w * self.scale) / 2, (self.height() - h * self.scale) / 2)
        self.fitted = True
        self.update()

    def resizeEvent(self, event):
        if self.fitted:
            self.fit()

    def to_screen(self, p):
        return QPointF(p[0] * self.scale + self.offset.x(), p[1] * self.scale + self.offset.y())

    def to_image(self, q):
        return np.array([(q.x() - self.offset.x()) / self.scale, (q.y() - self.offset.y()) / self.scale])

    def rotate_handle(self, b):
        a = math.radians(b.angle_deg)
        r = b.w / 2 + ROTATE_GAP_PX / self.scale
        return np.array([b.cx + r * math.cos(a), b.cy + r * math.sin(a)])

    def handle_at(self, q):
        """선택한 상자의 손잡이: 'rotate', 0~3(꼭짓점) 또는 None."""
        b = self.current()
        if b is None:
            return None
        near = lambda p: math.hypot(*(np.array([q.x(), q.y()]) - [self.to_screen(p).x(), self.to_screen(p).y()])) <= HANDLE_PX + 3
        if near(self.rotate_handle(b)):
            return 'rotate'
        return next((i for i, c in enumerate(b.corners()) if near(c)), None)

    def box_at(self, p):
        inside = [i for i, b in enumerate(self.boxes) if b.contains(p)]
        return min(inside, key=lambda i: self.boxes[i].w * self.boxes[i].h) if inside else -1   # 겹치면 작은 쪽

    # --- 마우스 ---
    def mousePressEvent(self, event):
        if self.image is None:
            return
        self.setFocus()
        q = event.position()
        p = self.to_image(q)
        if event.button() in (Qt.RightButton, Qt.MiddleButton):
            self.drag = ('pan', q, QPointF(self.offset))
            return
        if event.button() != Qt.LeftButton:
            return
        if self.clicks is not None:
            self.add_click(p)
            return
        handle = self.handle_at(q)
        if handle is None:
            self.selected = self.box_at(p)
            if self.current() is None:
                self.update()
                return
            handle = 'move'
        self.drag = (handle, p, copy.deepcopy(self.current()), copy.deepcopy(self.boxes))
        self.update()

    def mouseMoveEvent(self, event):
        q = event.position()
        self.hover = self.to_image(q)
        if self.drag is None:
            if self.clicks:
                self.update()
            return
        kind, start, origin = self.drag[:3]
        if kind == 'pan':
            self.offset = origin + (q - start)
            self.fitted = False
        else:
            p, b = self.hover, self.current()
            if kind == 'move':
                b.cx, b.cy = origin.cx + p[0] - start[0], origin.cy + p[1] - start[1]
            elif kind == 'rotate':
                b.angle_deg = math.degrees(math.atan2(p[1] - b.cy, p[0] - b.cx)) % 360.0
            else:
                self.boxes[self.selected] = L.resize_by_corner(origin, kind, p)
        self.update()

    def mouseReleaseEvent(self, event):
        drag, self.drag = self.drag, None
        if drag and drag[0] != 'pan' and self.current() is not None and self.current() != drag[2]:
            self.undo.append(drag[3])
            self.current().angle_deg %= 180.0
            self.changed()

    def wheelEvent(self, event):
        if self.image is None:
            return
        q, p = event.position(), self.to_image(event.position())
        self.scale = min(max(self.scale * (1.2 if event.angleDelta().y() > 0 else 1 / 1.2), 0.1), 20.0)
        self.offset = QPointF(q.x() - p[0] * self.scale, q.y() - p[1] * self.scale)
        self.fitted = False
        self.update()

    # --- 상자 명령 ---
    def start_new(self):
        self.clicks, self.selected = [], -1
        self.message.emit('새 상자: 장축의 한쪽 끝을 클릭')
        self.update()

    def cancel_new(self):
        if self.clicks is not None:
            self.clicks = None
            self.message.emit('새 상자 취소')
            self.update()

    def add_click(self, p):
        self.clicks.append(p)
        if len(self.clicks) < 3:
            self.message.emit(('', '장축의 반대쪽 끝을 클릭', '폭의 한쪽 가장자리를 클릭')[len(self.clicks)])
            self.update()
            return
        points, self.clicks = self.clicks, None
        try:
            box = L.box_from_clicks(*points)
        except ValueError as e:
            self.message.emit(f'상자를 만들지 못했습니다: {e}')
            self.update()
            return
        self.push_undo()
        self.boxes.append(box)
        self.selected = len(self.boxes) - 1
        self.message.emit(f'{NAMES[box.part_type]} 추가 (종류가 다르면 1 / 2)')
        self.changed()

    def set_type(self, kind):
        b = self.current()
        if b is not None and b.part_type != kind:
            self.push_undo()
            b.part_type = kind
            self.changed()

    def delete_selected(self):
        if self.current() is not None:
            self.push_undo()
            del self.boxes[self.selected]
            self.selected = -1
            self.changed()

    def nudge(self, dx=0.0, dy=0.0, turn=0.0):
        b = self.current()
        if b is not None:
            self.push_undo()
            b.cx, b.cy, b.angle_deg = b.cx + dx, b.cy + dy, (b.angle_deg + turn) % 180.0
            self.changed()

    # --- 그리기 ---
    def paintEvent(self, event):
        g = QPainter(self)
        g.fillRect(self.rect(), QColor(30, 30, 30))
        if self.image is None:
            return
        g.setRenderHint(QPainter.Antialiasing)
        g.drawImage(QPolygonF([self.to_screen((0, 0)), self.to_screen(self.size_px)]).boundingRect(), self.image)
        for i, b in enumerate(self.boxes):
            color = COLORS[b.part_type]
            g.setPen(QPen(color, 3 if i == self.selected else 1.6))
            g.setBrush(Qt.NoBrush)
            g.drawPolygon(QPolygonF([self.to_screen(c) for c in b.corners()]))
            top = min((self.to_screen(c) for c in b.corners()), key=lambda s: s.y())
            g.drawText(top + QPointF(4, -5), f'{i + 1} {NAMES[b.part_type]}')
        b = self.current()
        if b is not None:
            g.setPen(QPen(Qt.black, 1))
            g.setBrush(QBrush(QColor(255, 230, 0)))
            for c in b.corners():
                s = self.to_screen(c)
                g.drawRect(int(s.x()) - HANDLE_PX // 2, int(s.y()) - HANDLE_PX // 2, HANDLE_PX, HANDLE_PX)
            g.drawEllipse(self.to_screen(self.rotate_handle(b)), HANDLE_PX, HANDLE_PX)
        if self.clicks:
            points = self.clicks + ([self.hover] if self.hover is not None else [])
            g.setPen(QPen(QColor(255, 230, 0), 2, Qt.DashLine))
            g.setBrush(Qt.NoBrush)
            if len(points) == 3:
                try:
                    g.drawPolygon(QPolygonF([self.to_screen(c) for c in L.box_from_clicks(*points).corners()]))
                except ValueError:
                    pass
            g.drawPolyline(QPolygonF([self.to_screen(p) for p in points[:2]]))


class LabelWindow(QMainWindow):
    def __init__(self, root: Path, params: dict):
        super().__init__()
        self.setWindowTitle(f'라벨 도구 - {root}')
        self.params = params
        self.items = [(s, p) for s in L.SPLITS for p in L.list_images(root / s)]
        self.index, self.reviewed, self.dirty = -1, False, False
        self.canvas = Canvas()
        self.canvas.edited.connect(self.on_edited)
        self.list = QListWidget()
        self.list.setFixedWidth(300)
        self.list.setFocusPolicy(Qt.NoFocus)      # 방향키가 목록이 아니라 상자에 가게
        self.list.currentRowChanged.connect(self.open)
        self.info, self.note, self.total = QLabel(), QLabel(), QLabel()
        self.note.setWordWrap(True)
        self.canvas.message.connect(self.note.setText)
        help_label = QLabel(HELP)
        help_label.setStyleSheet('color: #666;')
        left = QVBoxLayout()
        left.addWidget(self.total)
        left.addWidget(self.list, 1)
        for text, slot in (('새 상자 (N)', self.canvas.start_new), ('초안 다시 만들기', self.redraft),
                           ('확인 완료 + 다음 (Enter)', self.confirm)):
            button = QPushButton(text)
            button.setFocusPolicy(Qt.NoFocus)
            button.clicked.connect(slot)
            left.addWidget(button)
        right = QVBoxLayout()
        right.addWidget(self.info)
        right.addWidget(self.canvas, 1)
        right.addWidget(self.note)
        right.addWidget(help_label)
        body = QHBoxLayout()
        body.addLayout(left)
        body.addLayout(right, 1)
        box = QWidget()
        box.setLayout(body)
        self.setCentralWidget(box)
        c = self.canvas
        for key, slot in {'N': c.start_new, 'Esc': c.cancel_new, '1': lambda: c.set_type('screw'), '2': lambda: c.set_type('nut'),
                          'Del': c.delete_selected, 'Backspace': c.delete_selected, 'Ctrl+Z': c.undo_last, 'F': c.fit,
                          'Return': self.confirm, 'Enter': self.confirm,
                          'PgDown': lambda: self.go(1), 'PgUp': lambda: self.go(-1),
                          'Q': lambda: c.nudge(turn=-1), 'E': lambda: c.nudge(turn=1),
                          'Shift+Q': lambda: c.nudge(turn=-5), 'Shift+E': lambda: c.nudge(turn=5)}.items():
            QShortcut(QKeySequence(key), self, activated=slot)
        for key, (dx, dy) in {'Left': (-1, 0), 'Right': (1, 0), 'Up': (0, -1), 'Down': (0, 1)}.items():
            QShortcut(QKeySequence(key), self, activated=lambda dx=dx, dy=dy: c.nudge(dx, dy))
            QShortcut(QKeySequence('Shift+' + key), self, activated=lambda dx=dx, dy=dy: c.nudge(5 * dx, 5 * dy))
        for split, image in self.items:
            self.list.addItem('')
        self.refresh_list()
        first = next((i for i, (_, p) in enumerate(self.items) if not self.is_reviewed(p)), 0)
        if self.items:
            self.list.setCurrentRow(first)

    @staticmethod
    def is_reviewed(image):
        label = L.load_label(image)
        return bool(label and label['reviewed'])

    def refresh_list(self):
        done = 0
        for i, (split, image) in enumerate(self.items):
            ok = self.reviewed if i == self.index else self.is_reviewed(image)
            done += ok
            self.list.item(i).setText(f'{"[완료]" if ok else "[    ]"} {split}/{image.name}')
        self.total.setText(f'확인 완료 {done} / {len(self.items)}장')

    def open(self, index):
        if index == self.index or not 0 <= index < len(self.items):
            return
        self.save()
        split, image = self.items[index]
        frame = cv2.imread(str(image))
        if frame is None:
            self.note.setText(f'사진을 읽지 못했습니다: {image}')
            return
        self.index, self.dirty = index, False
        label = L.load_label(image)
        if label is None:
            boxes, text = L.draft_boxes(frame, self.params)
            self.reviewed, self.dirty = False, True
            self.note.setText(f'규칙 초안입니다 ({text}). 빠진 부품은 N 으로 추가하고, 나사·너트가 아닌 것은 지우세요.')
        else:
            boxes, self.reviewed = label['boxes'], label['reviewed']
            self.note.setText('저장된 라벨을 열었습니다.')
            if (label['width'], label['height']) != (frame.shape[1], frame.shape[0]):
                self.reviewed, self.dirty = False, True
                self.note.setText('라벨을 만들 때와 사진 크기가 다릅니다. 상자를 다시 확인하세요.')
        self.frame = frame
        self.canvas.set_image(frame, boxes)
        self.list.setCurrentRow(index)
        self.canvas.setFocus()
        self.update_info()

    def update_info(self):
        if self.index < 0:
            return
        split, image = self.items[self.index]
        count = {k: sum(b.part_type == k for b in self.canvas.boxes) for k in L.CLASSES}
        self.info.setText(f'{self.index + 1}/{len(self.items)}  {split}/{image.name}   나사 {count["screw"]} · 너트 {count["nut"]}   '
                          + ('확인 완료' if self.reviewed else '확인 전'))
        self.refresh_list()

    def on_edited(self):
        self.reviewed, self.dirty = False, True    # 고치면 다시 확인해야 한다
        self.save()
        self.update_info()

    def save(self):
        if self.index >= 0 and self.dirty:
            L.save_label(self.items[self.index][1], self.canvas.size_px, self.canvas.boxes, self.reviewed)
            self.dirty = False

    def confirm(self):
        if self.index < 0 or self.canvas.clicks is not None or self.canvas.drag is not None:
            return
        self.reviewed, self.dirty = True, True
        self.save()
        self.update_info()
        if self.index + 1 < len(self.items):
            self.go(1)
        else:
            self.note.setText('마지막 사진입니다. 저장했습니다.')

    def go(self, step):
        self.open(self.index + step)

    def redraft(self):
        if self.index < 0:
            return
        boxes, text = L.draft_boxes(self.frame, self.params)
        self.canvas.push_undo()
        self.canvas.boxes, self.canvas.selected = boxes, -1
        self.canvas.changed()
        self.note.setText(f'초안을 다시 만들었습니다 ({text}). Ctrl+Z 로 되돌릴 수 있습니다.')

    def closeEvent(self, event):
        self.save()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('root', type=Path, help='train/val/test(/test_new) 폴더가 있는 곳')
    parser.add_argument('--params', nargs='+', default=ROOT_PARAMS, help='초안용 검출 설정 (흰 시트는 params.yaml 하나만)')
    parser.add_argument('--status', action='store_true', help='진행 상황과 문제만 출력')
    parser.add_argument('--export', type=Path, help='확인 완료 라벨을 YOLO 학습 데이터로 저장할 폴더')
    args, qt_args = parser.parse_known_args()
    if args.status:
        _, problems = L.check_dataset(args.root)
        for s in L.SPLITS:
            images = L.list_images(args.root / s)
            print(f'{s}: {sum(LabelWindow.is_reviewed(p) for p in images)}/{len(images)}장 확인 완료')
        print('\n'.join(problems) or '문제 없음, 내보낼 수 있습니다.')
        return
    if args.export:
        try:
            counts = L.export_dataset(args.root, args.export)
        except ValueError as e:
            sys.exit(f'내보내지 않았습니다.\n{e}')
        for s, c in counts.items():
            print(f'{s}: 사진 {c["images"]}장, 나사 {c["screw"]}, 너트 {c["nut"]}')
        print(f'저장: {args.export / "dataset.yaml"}')
        return
    if not any(L.list_images(args.root / s) for s in L.SPLITS):
        sys.exit(f'{args.root} 의 train/val/test 폴더에 사진이 없습니다.')
    params = load_vision_params(args.params)
    app = QApplication([sys.argv[0]] + qt_args)
    configure_korean_font()
    win = LabelWindow(args.root, params)
    win.resize(1500, 950)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
