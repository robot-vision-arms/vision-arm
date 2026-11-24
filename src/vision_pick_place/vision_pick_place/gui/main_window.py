"""PySide6 메인 윈도우 (3주차). 과제 GUI 필수 화면 4가지 (docs/PROJECT.md 2.2) + Gazebo 시연 화면.
11-10 목업의 두 판(GUI 대시보드 + Gazebo 창)을 한 창에: 왼쪽 = 휴대폰 카메라 검출, 오른쪽 = Gazebo 시연.

    ┌ 헤더: 제목·입력 │ 연결 상태 │ [실물 → Gazebo 자동 반영] [처음부터 다시] [집기 시작] ┐
    ├──────────────────────┬──────────────────────┬──────────────────────┤
    │ ① 실물 카메라 · 검출   │ ② Gazebo 전경          │ ③ Gazebo 위 카메라 (작게) │
    │ (/vision/debug_image) │ (/sim/overview/image) │ (/sim/camera/image)   │
    │                      │                      │ 실물 모드에서만         │
    ├───────────────┬─────────────────┴──┬──────────────────────────────┤
    │ ④ 좌표 표      │ ⑤ 로봇 응답·집기 결과  │ ⑥ 보정 상태                     │
    └───────────────┴────────────────────┴──────────────────────────────┘
    번호는 왼쪽 위부터 차례로 (가상 모드는 ③ 이 없어 아래가 ③④⑤). 과제 필수 화면 4가지 = ①④⑤⑥.

실물 → Gazebo 자동 반영: 휴대폰 화면의 부품 배치가 STABLE_FRAMES 패킷 동안 그대로면 (손이 들어와 흔들리는 동안은 안 함)
/sim/spawn_objects 로 같은 위치에 가상 부품을 만든다. 다 치우면 /sim/clear_objects. 사이클 중에는 멈추고,
사이클 뒤에도 실물 배치가 그대로면 다시 만들지 않는다 (MIRROR_MOVE_MM 넘게 바뀌어야 다시 반영).

처음부터 다시 (집기가 실패해 멈췄을 때): /sim/go_ready (진공 끄기·대기 자세) → /sim/clear_objects → 실물은 지금 배치가
안정되면 /sim/spawn_objects, 가상은 바로 /sim/spawn_random. 중간에 실패하면 거기서 멈추고 이유를 보여 준다.

    source env.sh   # venv (PySide6) 를 켠다
    python3 -m vision_pick_place.gui.main_window                                         # 실물 (휴대폰, data/calib)
    python3 -m vision_pick_place.gui.main_window --source sim --calib-dir data/calib/sim  # Gazebo 가상 카메라 입력
    (ros2 run vision_pick_place gui 는 --symlink-install 빌드에서 시스템 python 으로 떠서 PySide6 를 못 찾는다)

ROS 는 ros_worker.RosWorker (QThread) 가 받고, 이 창은 Signal 로 받은 dict/영상으로 위젯만 갱신한다.
갱신 함수(on_detections 등)는 ROS 없이 가짜 데이터로 시험할 수 있다 (test/test_gui.py).
"""
import argparse
from collections import deque
from datetime import datetime
import html
import math
from pathlib import Path
import re
import sys
import time

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QImage, QPixmap
from PySide6.QtWidgets import (QApplication, QCheckBox, QFrame, QHBoxLayout, QHeaderView, QLabel, QMainWindow,
                               QPlainTextEdit, QPushButton, QSizePolicy, QTableWidget, QTableWidgetItem,
                               QVBoxLayout, QWidget)

from vision_pick_place.calibrator import Calibrator
from vision_pick_place.communication_check import percentile95
from vision_pick_place.gui.pipeline_inspector import configure_korean_font

LOG_LINES = 300       # ③ 콘솔에 남길 줄 수
SUMMARY_WINDOW = 100  # ③ ACK 성공률·P95 를 계산할 최근 송신 수
STALE_S = 2.0         # 이 시간 동안 아무것도 안 오면 연결 표시를 회색으로
STABLE_FRAMES = 4     # 실물 → Gazebo 반영: 배치가 이만큼 연속 패킷 동안 같아야 반영 (약 1초, 손이 들어오는 동안 제외)
STABLE_TOL_MM = 3.0   # 연속 패킷 사이에서 '같은 자리' 로 볼 거리
MIRROR_MOVE_MM = 5.0  # 이미 반영한 배치에서 이만큼 넘게 바뀌어야 다시 반영

# 목업 색 (11-10 GUI 대시보드 목업)
BG, CARD, BORDER, LINE = '#EEF0F3', '#FFFFFF', '#DDE1E6', '#E4E7EB'
INK, MUTED, BLUE = '#16191D', '#5B6470', '#1D5FD1'
OK_FG, OK_BG, BAD_FG, BAD_BG, OFF_FG, OFF_BG = '#13734B', '#E6F4EC', '#B42318', '#FDECEA', '#5B6470', '#E4E7EB'
PART_BADGE = {'screw': ('나사', BLUE, '#E8EFFB'), 'nut': ('너트', '#A34808', '#FBEDE3'),
              'unknown': ('모름', MUTED, '#EEF0F3')}
SERVICE_NAMES = {'/sim/run_cycle': '집기 시작', '/sim/spawn_objects': 'Gazebo 에 부품 놓기', '/sim/clear_objects': 'Gazebo 부품 지우기',
                 '/sim/spawn_random': '부품 놓기 (무작위)', '/sim/go_ready': '로봇 대기 자세로'}
CONSOLE = {'time': '#8B95A1', 'ack': '#6FD3A0', 'err': '#FF8A80', 'sim': '#F5B27A'}
MONO = "'JetBrains Mono', 'DejaVu Sans Mono', monospace"

STYLE = f"""
QMainWindow, QWidget#root {{ background: {BG}; }}
QFrame#card {{ background: {CARD}; border: 1px solid {BORDER}; border-radius: 10px; }}
QLabel {{ color: {INK}; }}
QLabel#title {{ font-size: 16px; font-weight: 700; }}
QLabel#appTitle {{ font-size: 20px; font-weight: 700; }}
QLabel#meta, QLabel#note {{ color: {MUTED}; font-size: 12px; }}
QFrame#tile {{ border: 1px solid {LINE}; border-radius: 8px; background: {CARD}; }}
QLabel#tileLabel {{ color: {MUTED}; font-size: 12px; border: none; }}
QLabel#tileValue {{ font-family: {MONO}; font-size: 16px; font-weight: 600; border: none; }}
QPushButton {{ min-height: 40px; padding: 0 16px; border-radius: 8px; border: 1px solid #C9CED6;
              background: {CARD}; color: {INK}; font-size: 14px; }}
QPushButton#primary {{ border: 1px solid {BLUE}; background: {BLUE}; color: white; }}
QPushButton:disabled {{ background: #C9CED6; border-color: #C9CED6; color: white; }}
QTableWidget {{ border: 1px solid {LINE}; border-radius: 8px; gridline-color: {LINE}; font-family: {MONO}; font-size: 15px; }}
QHeaderView::section {{ background: #F4F5F7; color: #3E4650; border: none; padding: 8px; font-weight: 500; }}
QPlainTextEdit#console {{ background: #16191D; color: #D7DCE2; border-radius: 8px; padding: 8px;
                         font-family: {MONO}; font-size: 14px; }}
"""


def layout_of(objects: list) -> list:
    """반영 대상 (나사·너트, mm 좌표 있음) 만 (종류, x, y) 로, 비교하기 쉽게 정렬."""
    return sorted((o['part_type'], float(o['x_mm']), float(o['y_mm'])) for o in objects
                  if o.get('part_type') in ('screw', 'nut') and o.get('x_mm') is not None and o.get('y_mm') is not None)


def same_layout(a: list, b: list, tol_mm: float) -> bool:
    """종류·개수가 같고 각 부품이 tol_mm 안에 있으면 같은 배치."""
    if b is None or len(a) != len(b):
        return False
    used = set()
    for kind, x, y in a:
        match = next((i for i, (k2, x2, y2) in enumerate(b)
                      if i not in used and k2 == kind and math.hypot(x - x2, y - y2) <= tol_mm), None)
        if match is None:
            return False
        used.add(match)
    return True


def _fmt(value, digits=1):
    return '—' if value is None else f'{value:.{digits}f}'


def _pill(label: QLabel, text: str, state: str):
    """목업의 둥근 상태 배지. state: ok / bad / off."""
    fg, bg = {'ok': (OK_FG, OK_BG), 'bad': (BAD_FG, BAD_BG), 'off': (OFF_FG, OFF_BG)}[state]
    label.setText(f'● {text}')
    label.setStyleSheet(f'padding: 5px 12px; border-radius: 13px; background: {bg}; color: {fg}; '
                        'font-size: 13px; font-weight: 500;')


class Card(QFrame):
    """흰 카드: 제목 + 오른쪽 작은 설명 + 내용."""

    def __init__(self, title: str):
        super().__init__()
        self.setObjectName('card')
        self.body = QVBoxLayout(self)
        self.body.setContentsMargins(16, 14, 16, 14)
        self.body.setSpacing(10)
        self.head = QHBoxLayout()
        name = QLabel(title)
        name.setObjectName('title')
        self.meta = QLabel('')
        self.meta.setObjectName('meta')
        self.head.addWidget(name)
        self.head.addStretch(1)
        self.head.addWidget(self.meta)
        self.body.addLayout(self.head)


class Tile(QFrame):
    """목업의 작은 타일: 위에 설명, 아래에 값."""

    def __init__(self, label: str, value: str = '—'):
        super().__init__()
        self.setObjectName('tile')
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(2)
        caption = QLabel(label)
        caption.setObjectName('tileLabel')
        self.value = QLabel(value)
        self.value.setObjectName('tileValue')
        layout.addWidget(caption)
        layout.addWidget(self.value)


class VideoPanel(QLabel):
    """① 영상. 창 크기에 맞춰 비율을 유지하며 줄인다."""

    def __init__(self, topic: str = '/vision/debug_image'):
        super().__init__(f'영상 없음\n({topic} 대기 중)')
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet('background: #202020; color: #bbbbbb; border-radius: 6px;')
        self._pixmap = None
        self.frame_size = None

    def show_frame(self, bgr: np.ndarray):
        if not self.isVisible() and self._pixmap is not None:
            self.frame_size = (bgr.shape[1], bgr.shape[0])   # 안 보이는 탭은 변환을 건너뛴다 (CPU 절약)
            return
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
        self._pixmap = QPixmap.fromImage(image)
        self.frame_size = (rgb.shape[1], rgb.shape[0])
        self._rescale()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._rescale()

    def _rescale(self):
        if self._pixmap is not None:
            self.setPixmap(self._pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))


class MainWindow(QMainWindow):
    def __init__(self, calib_dir: str = 'data/calib', start_worker: bool = True, source: str = 'real'):
        super().__init__()
        self.setWindowTitle('Vision Pick & Place')
        self.setStyleSheet(STYLE)
        self.calib_dir = Path(calib_dir)
        self.source = source                        # 'real' = 휴대폰 카메라, 'sim' = Gazebo 가상 카메라
        self.candidate, self.stable_count = None, 0  # 실물 → Gazebo 반영: 지금 보고 있는 배치와 연속 횟수
        self.mirrored = None                        # 마지막으로 Gazebo 에 반영한 배치
        self.busy = set()                           # 응답을 기다리는 서비스 (사이클 중에는 반영 안 함)
        self.last_packet_time = None
        self.last_ack_time = None
        self.packet_times = deque(maxlen=30)       # 패킷 수신 주기 (Hz 표시)
        self.sends = deque(maxlen=SUMMARY_WINDOW)   # (acked, latency_ms)
        self.picks = []                             # Gazebo 집기 결과 True/False
        self.pending_acks = []                      # 아직 화면에 안 쓴 ACK 의 latency_ms. 1초마다 ⑤ 위 한 줄(ack_status)을 새로 쓴다
        numbers = iter('①②③④⑤⑥')                   # 화면에 놓는 차례대로 번호
        self.next_number = lambda: next(numbers)
        self.gazebo = False
        self.resetting = False                      # [처음부터 다시] 진행 중 (로봇 복귀·부품 지우기)
        self.reset_waiting = False                  # 지운 뒤 실물 배치가 안정되기를 기다리는 중 (자동 반영이 꺼져 있어도 한 번 반영)

        root = QWidget()
        root.setObjectName('root')
        outer = QVBoxLayout(root)
        outer.setContentsMargins(20, 20, 20, 20)
        outer.setSpacing(16)
        outer.addWidget(self._header())
        top = QHBoxLayout()
        top.setSpacing(16)
        top.addWidget(self._video_card(), 5)
        top.addWidget(self._overview_card(), 5)
        if self.source == 'real':   # 가상 모드에서는 ① 이 이미 Gazebo 위 카메라라 겹친다
            top.addWidget(self._sim_top_card(), 3)
        bottom = QHBoxLayout()
        bottom.setSpacing(16)
        bottom.addWidget(self._detection_card(), 4)
        bottom.addWidget(self._log_card(), 5)
        bottom.addWidget(self._calib_card(), 3)
        outer.addLayout(top, 5)
        outer.addLayout(bottom, 3)
        self.setCentralWidget(root)

        self.load_calibration()
        self.status_timer = QTimer(self)   # 1초마다 연결 표시·패킷 끊김 갱신
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)
        # ACK 는 초당 여러 번 와서 콘솔에 쌓으면 집기 결과가 밀려 올라간다. 콘솔 위 한 줄만 새로 쓴다
        self.ack_timer = QTimer(self)
        self.ack_timer.timeout.connect(self.flush_acks)
        self.ack_timer.start(1000)
        self.update_status()

        self.worker = None
        if start_worker:
            from vision_pick_place.gui.ros_worker import RosWorker
            self.worker = RosWorker(self, sim_top=self.source == 'real')
            self.worker.detections.connect(self.on_detections)
            self.worker.vision_info.connect(self.on_vision_info)
            self.worker.frame.connect(self.video.show_frame)
            self.worker.overview.connect(self.overview.show_frame)
            if self.source == 'real':
                self.worker.sim_top.connect(self.sim_top.show_frame)
            self.worker.robot_log.connect(self.on_robot_log)
            self.worker.sim_log.connect(self.on_sim_log)
            self.worker.service_result.connect(self.on_service_result)
            self.worker.gazebo_ready.connect(self.on_gazebo_ready)
            self.worker.start()

    # ---------- 화면 구성 ----------

    def _header(self) -> QFrame:
        bar = QFrame()
        bar.setObjectName('card')
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(18, 12, 18, 12)
        layout.setSpacing(12)
        names = QVBoxLayout()
        title = QLabel('Vision Pick & Place')
        title.setObjectName('appTitle')
        self.input_label = QLabel('입력: 휴대폰 카메라 (실물)' if self.source == 'real' else '입력: Gazebo 가상 카메라')
        self.input_label.setObjectName('meta')
        names.addWidget(title)
        names.addWidget(self.input_label)
        layout.addLayout(names, 1)
        self.pill_ros, self.pill_robot, self.pill_gazebo = QLabel(), QLabel(), QLabel()
        for pill in (self.pill_ros, self.pill_robot, self.pill_gazebo):
            layout.addWidget(pill)
        layout.addSpacing(8)
        # 실물 모드: 휴대폰 화면의 배치를 Gazebo 에 그대로 (모듈 설명)
        self.auto_sync = QCheckBox('실물 → Gazebo 자동 반영')
        self.auto_sync.setChecked(self.source == 'real')
        self.auto_sync.setVisible(self.source == 'real')
        layout.addWidget(self.auto_sync)
        # 시뮬 시연용: 사이클이 끝나면 매트가 비므로 새 부품을 놓는다 (실물에서는 사람이 놓는다)
        self.spawn_button = QPushButton('부품 놓기 (무작위)')
        self.spawn_button.clicked.connect(self.request_spawn)
        self.spawn_button.setVisible(self.source == 'sim')
        layout.addWidget(self.spawn_button)
        self.reset_button = QPushButton('처음부터 다시')
        self.reset_button.setToolTip('집기가 실패해 멈췄을 때: 로봇을 대기 자세로 돌리고 Gazebo 부품을 지운 뒤 다시 놓습니다')
        self.reset_button.clicked.connect(self.request_reset)
        layout.addWidget(self.reset_button)
        self.cycle_button = QPushButton('집기 시작')
        self.cycle_button.setObjectName('primary')
        self.cycle_button.clicked.connect(self.request_cycle)
        layout.addWidget(self.cycle_button)
        return bar

    def _video_card(self) -> Card:
        name = '실물 카메라' if self.source == 'real' else '가상 카메라'
        self.video_card = Card(f'{self.next_number()} {name} · 부품 찾기')
        self.pill_detector = QLabel()      # 검출 방식 (/vision/info)
        _pill(self.pill_detector, '검출 방식 확인 중', 'off')
        self.video_card.head.addWidget(self.pill_detector)
        self.video = VideoPanel()
        self.video_card.body.addWidget(self.video, 1)
        legend = QLabel('<span style="color:#C8A400">━</span> 검출 영역 (시트 안쪽, 마커 제외) &nbsp;&nbsp; '
                        '<span style="color:#1FA34A">━</span> 검출 박스 &nbsp;&nbsp; '
                        f'<span style="color:{BLUE}">→</span> 장축 방향 (나사)')
        legend.setObjectName('note')
        self.video_card.body.addWidget(legend)
        return self.video_card

    def _overview_card(self) -> Card:
        """⑤ Gazebo 전경 카메라 (11-10 목업의 Gazebo 판). Gazebo 창을 띄우지 않아도 스폰·집기 과정을 본다."""
        self.overview_card = Card(f'{self.next_number()} Gazebo 로봇')
        self.overview = VideoPanel('/sim/overview/image')
        self.overview_card.body.addWidget(self.overview, 1)
        self.sync_status = QLabel('실물 배치를 기다리는 중' if self.source == 'real' else '가상 카메라 입력: [부품 놓기] → [집기 시작]')
        self.sync_status.setObjectName('note')
        self.overview_card.body.addWidget(self.sync_status)
        return self.overview_card

    def _sim_top_card(self) -> Card:
        """⑥ Gazebo 위 카메라 원본: 실물과 같은 자리(작업 매트)를 위에서 본 Gazebo. ① 과 나란히 보면 반영이 맞는지 바로 보인다."""
        card = Card(f'{self.next_number()} Gazebo 위에서 본 모습')
        self.sim_top = VideoPanel('/sim/camera/image')
        card.body.addWidget(self.sim_top, 1)
        note = QLabel('실물과 같은 배치인지 비교')
        note.setObjectName('note')
        card.body.addWidget(note)
        return card

    def _detection_card(self) -> Card:
        self.det_card = Card(f'{self.next_number()} 부품 위치 · 각도')
        self.det_card.meta.setText('기준: 작업 시트 왼쪽 아래 = (0, 0)')
        self.det_status = QLabel('패킷 대기 중 (/vision/detections)')
        self.det_status.setObjectName('note')
        self.det_table = QTableWidget(0, 5)
        self.det_table.setHorizontalHeaderLabels(['#', '종류', '화면 위치', '시트 위 위치 (mm)', '각도'])
        self.det_table.setColumnHidden(2, True)   # 화면 위치(px)는 보는 사람에게 필요 없어 숨긴다 (값은 채워 둔다)
        header = self.det_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.det_table.verticalHeader().setVisible(False)
        self.det_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.det_table.setSelectionMode(QTableWidget.NoSelection)
        self.det_card.body.addWidget(self.det_status)
        self.det_card.body.addWidget(self.det_table, 1)
        tiles = QHBoxLayout()
        self.tile_parts = Tile('찾은 부품')   # 판정 기준은 검출 방식(규칙·YOLO)마다 달라 적지 않는다
        self.tile_angle = Tile('각도 기준', '시트 가로축에서 0~180°')
        self.tile_target = Tile('송신 대상', '—')
        for tile in (self.tile_parts, self.tile_angle, self.tile_target):
            tiles.addWidget(tile)
        self.det_card.body.addLayout(tiles)
        return self.det_card

    def _log_card(self) -> Card:
        card = Card(f'{self.next_number()} 로봇 응답 · 집기 결과')
        self.ack_status = QLabel('로봇 응답 대기 중')
        self.ack_status.setObjectName('note')
        card.body.addWidget(self.ack_status)
        self.console = QPlainTextEdit()
        self.console.setObjectName('console')
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(LOG_LINES)
        card.body.addWidget(self.console, 1)
        tiles = QHBoxLayout()
        self.tile_ack = Tile('로봇 수신 성공률')
        self.tile_p95 = Tile('검출→송신 지연 (P95)')
        self.tile_pick = Tile('집기 성공')
        for tile in (self.tile_ack, self.tile_p95, self.tile_pick):
            tiles.addWidget(tile)
        card.body.addLayout(tiles)
        return card

    def _calib_card(self) -> Card:
        card = Card(f'{self.next_number()} 카메라 보정')
        card.meta.hide()
        self.calib_pill = QLabel()
        card.head.addWidget(self.calib_pill)
        box = QFrame()
        box.setObjectName('tile')
        rows = QVBoxLayout(box)
        rows.setContentsMargins(0, 0, 0, 0)
        rows.setSpacing(0)
        self.calib_labels = {}
        items = [('intrinsics', '카메라 렌즈 보정'),
                 ('homography', '작업 시트 보정 (화면 → mm)'), ('error', '보정 정확도 (작을수록 좋음)'),
                 ('camera', '카메라 높이')]
        for i, (key, name) in enumerate(items):
            row = QFrame()
            row.setStyleSheet(f'QFrame {{ border: none; border-top: 1px solid {LINE if i else "transparent"}; }}')
            line = QHBoxLayout(row)
            line.setContentsMargins(14, 9, 14, 9)
            left = QLabel(name)
            left.setStyleSheet('border: none; font-size: 14px;')
            value = QLabel('—')
            value.setStyleSheet(f'border: none; font-family: {MONO}; font-size: 13px;')
            value.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            line.addWidget(left)
            line.addStretch(1)
            line.addWidget(value)
            rows.addWidget(row)
            self.calib_labels[key] = value
        card.body.addWidget(box)
        note = QLabel('카메라나 작업 시트가 움직이면 시트 사진을 다시 찍어 보정합니다. 보정이 필요하면 로봇은 그 위치를 받지 않습니다.')
        note.setObjectName('note')
        note.setWordWrap(True)
        card.body.addWidget(note)
        self.reload_button = QPushButton('보정 정보 새로고침')
        self.reload_button.setToolTip('이 칸의 표시만 새로 읽습니다. 새 보정을 검출에 쓰려면 vision_node 를 다시 시작해야 합니다')
        self.reload_button.clicked.connect(self.load_calibration)
        card.body.addWidget(self.reload_button)
        card.body.addStretch(1)
        _pill(self.calib_pill, '보정 확인 중', 'off')
        return card

    # ---------- 보정 ----------

    def load_calibration(self):
        """보정 파일을 읽어 상태를 표시한다. 파일 저장 시각으로 '카메라가 움직인 뒤 다시 보정했는지' 판단한다."""
        intr, hom = self.calib_dir / 'intrinsics.npz', self.calib_dir / 'homography.npz'
        labels = self.calib_labels
        self.calib_pill.setToolTip(f'보정 폴더: {self.calib_dir}')
        try:
            calib = Calibrator(str(intr), str(hom))
        except (ValueError, OSError) as exc:
            for key in ('homography', 'error', 'camera'):
                labels[key].setText('—')
            labels['intrinsics'].setText(f'오류: {exc}')
            return
        status = calib.status

        def loaded(ok, name):
            color = OK_FG if ok else BAD_FG
            return f'<span style="color:{color}">{"완료" if ok else f"없음 ({name})"}</span>'

        labels['intrinsics'].setText(loaded(status.intrinsics_loaded, 'intrinsics.npz'))
        labels['homography'].setText(loaded(status.homography_loaded, 'homography.npz'))
        labels['error'].setText(f'{_fmt(status.reproj_err_mm, 2)} mm')
        if status.ready:
            x, y, z = calib.camera_center_mm()
            labels['camera'].setText(f'{z:.0f} mm')
            labels['camera'].setToolTip(f'시트 ({x:.0f}, {y:.0f}) mm 위')
        else:
            labels['camera'].setText('—')
        stamps = [datetime.fromtimestamp(p.stat().st_mtime).strftime('%m-%d %H:%M') for p in (intr, hom) if p.is_file()]
        self.calib_pill.setToolTip(f'보정 폴더: {self.calib_dir}\n저장 시각: {" / ".join(stamps) or "—"}\n'
                                   f'렌즈 보정 오차: {_fmt(status.rms_px, 2)} px')

    # ---------- 검출 ----------

    def on_vision_info(self, info: dict):
        kind = info.get('detector')
        _pill(self.pill_detector, {'yolo': '검출: YOLO', 'rule': '검출: 규칙'}.get(kind, f'검출: {kind}'),
              'ok' if kind == 'yolo' else 'off')

    def on_detections(self, packet: dict):
        now = time.monotonic()
        if self.last_packet_time is not None:
            self.packet_times.append(now - self.last_packet_time)
        self.last_packet_time = now
        calib_ok = bool(packet.get('calib_ok'))
        objects = packet.get('objects') or []
        self.det_table.setRowCount(len(objects))
        for row, obj in enumerate(objects):
            kind = obj.get('part_type')
            name, fg, bg = PART_BADGE.get(kind, (str(kind), MUTED, '#EEF0F3'))
            values = [str(obj.get('id', '')), name,
                      f'{_fmt(obj.get("u_px"))}, {_fmt(obj.get("v_px"))}',
                      f'{_fmt(obj.get("x_mm"))}, {_fmt(obj.get("y_mm"))}',
                      '—' if obj.get('angle_deg') is None else f'{obj["angle_deg"]:.1f}°']
            for col, text in enumerate(values):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                if col == 1:   # 목업처럼 종류를 색 배지로
                    item.setForeground(QBrush(QColor(fg)))
                    item.setBackground(QBrush(QColor(bg)))
                self.det_table.setItem(row, col, item)
        rate = f' · {1 / np.mean(self.packet_times):.1f} Hz' if self.packet_times else ''
        self.det_status.setText(f'물체 {len(objects)}개{rate}' + ('' if calib_ok else '  [보정 필요 — 위치·각도 없음]'))
        self.video_card.meta.setText(f'부품 {len(objects)}개{rate}')
        kinds = [o.get('part_type') for o in objects]
        self.tile_parts.value.setText(f'나사 {kinds.count("screw")} · 너트 {kinds.count("nut")}')
        sendable = sum(o.get('part_type') in ('screw', 'nut') and o.get('x_mm') is not None for o in objects)
        self.tile_target.value.setText(f'{sendable}개')
        _pill(self.calib_pill, '보정 완료' if calib_ok else '보정 필요', 'ok' if calib_ok else 'bad')
        if self.video.frame_size:
            name = '휴대폰 카메라 (실물)' if self.source == 'real' else 'Gazebo 가상 카메라'
            self.input_label.setText(f'입력: {name} · {self.video.frame_size[0]}×{self.video.frame_size[1]}')
        if calib_ok:
            self.mirror(objects)

    def mirror(self, objects: list):
        """실물 → Gazebo 자동 반영 (모듈 설명). 배치가 STABLE_FRAMES 연속 같고, 반영한 배치와 다르면 스폰·지우기 요청."""
        current = layout_of(objects)
        if same_layout(current, self.candidate, STABLE_TOL_MM):
            self.stable_count += 1
        else:
            self.candidate, self.stable_count = current, 1
        if not (self.auto_sync.isChecked() or self.reset_waiting) or self.busy or self.worker is None:
            return
        if self.stable_count < STABLE_FRAMES or same_layout(current, self.mirrored, MIRROR_MOVE_MM):
            return
        self.mirrored = current
        if self.reset_waiting and not current:   # 이미 지웠으니 놓을 것이 없다
            self.reset_waiting = False
            self.sync_status.setText('처음 상태로 돌아옴. 시트에 부품이 없습니다')
            return
        self.reset_waiting = False
        if current:
            names = ', '.join(PART_BADGE[k][0] for k, _, _ in current)
            self.sync_status.setText(f'실물 배치 반영 중: {names}')
            self.call_service('/sim/spawn_objects')
        else:
            self.sync_status.setText('실물에서 부품을 치움 → Gazebo 부품 지우는 중')
            self.call_service('/sim/clear_objects')

    # ---------- 로봇 응답·집기 ----------

    def log_line(self, tag: str, color: str, text: str):
        """콘솔 한 줄: 시각  태그  내용 (목업의 TX/ACK/SIM 줄)."""
        stamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.console.appendHtml(f'<span style="color:{CONSOLE["time"]}">{stamp}</span>&nbsp;&nbsp;'
                                f'<span style="color:{color}">{tag}</span>&nbsp;&nbsp;{html.escape(text)}')

    def on_robot_log(self, log: dict):
        ok = bool(log.get('acked'))
        self.sends.append((ok, log.get('latency_ms') if ok else None))
        if ok:
            self.last_ack_time = time.monotonic()
            self.pending_acks.append(log.get('latency_ms'))
        else:
            self.log_line('ERR', CONSOLE['err'], f'로봇 응답 없음  {log.get("error") or ""}'.rstrip())
        self.update_tiles()

    def flush_acks(self):
        """모아 둔 ACK 를 콘솔 위 한 줄에: '로봇 수신 확인 · 응답 14 ms (1초에 5건)'. 응답 = 검출부터 ACK 까지 평균."""
        if not self.pending_acks:
            return
        ms = [v for v in self.pending_acks if v is not None]
        self.ack_status.setText('로봇 수신 확인' + (f' · 응답 {np.mean(ms):.0f} ms' if ms else '') + f' (1초에 {len(self.pending_acks)}건)')
        self.pending_acks = []

    def on_sim_log(self, log: dict):
        ok = log.get('result') == 'ok'
        if 'obj' in log:   # run_cycle 의 물체 하나 (move_to_target 기록은 action 키만 있다)
            self.picks.append(ok)
            name = PART_BADGE.get(log.get('part_type'), (log.get('part_type'),))[0]
            seconds = (log.get('plan_ms') or 0) / 1000
            text = f'{log.get("obj")} {name} → 상자  {"성공" if ok else "실패"}  ({seconds:.1f} s)'
        else:
            text = f'{log.get("action", "")}  {"성공" if ok else "실패"}'
        if not ok and log.get('error'):
            text += f'  — {log["error"]}'
        self.log_line('SIM', CONSOLE['sim'] if ok else CONSOLE['err'], text)
        self.update_tiles()

    def update_tiles(self):
        if self.sends:
            acked = [ms for ok, ms in self.sends if ok and ms is not None]
            count = sum(ok for ok, _ in self.sends)
            self.tile_ack.value.setText(f'{100 * count / len(self.sends):.0f} % ({count}/{len(self.sends)})')
            self.tile_p95.value.setText(f'{percentile95(acked):.0f} ms' if acked else '—')
        if self.picks:
            self.tile_pick.value.setText(f'{sum(self.picks)} / {len(self.picks)}')

    # ---------- 헤더 ----------

    def on_gazebo_ready(self, ready: bool):
        self.gazebo = ready

    def update_status(self):
        now = time.monotonic()
        fresh = self.last_packet_time is not None and now - self.last_packet_time < STALE_S
        _pill(self.pill_ros, '카메라 검출' if fresh else '카메라 검출 대기', 'ok' if fresh else 'off')
        acked = self.last_ack_time is not None and now - self.last_ack_time < STALE_S
        if not acked and self.last_ack_time is not None:
            self.ack_status.setText(f'[주의] {now - self.last_ack_time:.0f}초째 로봇 응답 없음')
        _pill(self.pill_robot, '로봇 수신' if acked else '로봇 수신 대기', 'ok' if acked else 'off')
        _pill(self.pill_gazebo, 'Gazebo' if self.gazebo else 'Gazebo 없음', 'ok' if self.gazebo else 'off')
        if self.last_packet_time is not None and not fresh:
            self.det_status.setText(f'[주의] {now - self.last_packet_time:.0f}초째 패킷 없음 (vision_node 확인)')

    def request_cycle(self):
        if self.worker is None:
            return
        self.set_buttons(False)
        self.cycle_button.setText('집는 중...')
        self.call_service('/sim/run_cycle')

    def call_service(self, name: str):
        if self.worker is None:
            return
        self.busy.add(name)
        self.log_line('SIM', CONSOLE['sim'], f'{SERVICE_NAMES.get(name, name)} 요청')
        self.worker.request_service(name)

    def request_spawn(self):
        if self.worker is None:
            return
        self.set_buttons(False)
        self.call_service('/sim/spawn_random')

    def set_buttons(self, enabled: bool):
        for button in (self.cycle_button, self.spawn_button, self.reset_button):
            button.setEnabled(enabled)

    def request_reset(self):
        """처음부터 다시 (모듈 설명). 로봇이 움직이는 동안에는 버튼이 꺼져 있다."""
        if self.worker is None:
            return
        self.resetting, self.reset_waiting = True, False
        self.set_buttons(False)
        self.reset_button.setText('되돌리는 중...')
        self.sync_status.setText('처음부터 다시: 로봇을 대기 자세로 돌리는 중')
        self.call_service('/sim/go_ready')

    def on_reset_step(self, name: str, ok: bool, message: str):
        if ok and name == '/sim/go_ready':
            self.sync_status.setText('처음부터 다시: Gazebo 부품 지우는 중')
            self.call_service('/sim/clear_objects')
            return
        if ok and name == '/sim/clear_objects' and self.source == 'sim':
            self.sync_status.setText('처음부터 다시: 부품을 새로 놓는 중')
            self.call_service('/sim/spawn_random')
            return
        self.resetting = False
        self.set_buttons(True)
        self.reset_button.setText('처음부터 다시')
        if not ok:
            self.sync_status.setText(f'처음부터 다시 실패 ({SERVICE_NAMES.get(name, name)}): {message[:80]}')
        elif name == '/sim/clear_objects':   # 실물: 손이 없고 배치가 안정되면 mirror() 가 다시 놓는다
            self.candidate, self.stable_count, self.mirrored, self.reset_waiting = None, 0, None, True
            self.sync_status.setText('처음부터 다시: 실물 배치를 확인하는 중')
        else:
            self.sync_status.setText('처음 상태로 돌아옴. [집기 시작] 을 누르세요')

    def on_service_result(self, name: str, ok: bool, message: str):
        self.busy.discard(name)
        found = re.search(r'\d+/\d+', message) if name == '/sim/run_cycle' else None   # '… 5/5 개 상자 투입 …'
        label = f'집기 {found.group()}개 ' if found else f'{SERVICE_NAMES.get(name, name)} '
        self.log_line('SIM', CONSOLE['ack'] if ok else CONSOLE['err'], label + ('완료' if ok else f'실패: {message}'))
        if self.resetting and name in ('/sim/go_ready', '/sim/clear_objects', '/sim/spawn_random'):
            self.on_reset_step(name, ok, message)
        elif name == '/sim/run_cycle':
            self.set_buttons(True)
            self.cycle_button.setText('집기 시작')
            # 사이클 뒤 Gazebo 매트는 비지만 실물 배치는 그대로다. 실물이 바뀌기 전까지는 다시 반영하지 않는다
            self.mirrored = self.candidate
            if not ok:
                self.sync_status.setText('집기가 멈췄습니다. [처음부터 다시] 를 누르면 되돌립니다')
            elif self.source == 'real':
                self.sync_status.setText('집기 끝. 실물 배치를 바꾸면 다시 반영합니다')
        elif name == '/sim/spawn_random':
            self.set_buttons(True)
        elif name in ('/sim/spawn_objects', '/sim/clear_objects'):
            if ok:
                self.sync_status.setText(f'Gazebo 에 부품 {len(self.mirrored or [])}개 반영 완료' if name == '/sim/spawn_objects'
                                         else 'Gazebo 부품을 지웠습니다')
            else:
                self.sync_status.setText(f'{SERVICE_NAMES[name]} 실패: {message[:80]}')
            if not ok:
                self.mirrored = None   # 다음 안정된 배치에서 다시 시도

    def closeEvent(self, event):
        if self.worker is not None:
            self.worker.stop()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description='Vision Pick & Place GUI')
    parser.add_argument('--calib-dir', default='data/calib', help='intrinsics.npz·homography.npz 폴더 (가상 카메라: data/calib/sim)')
    parser.add_argument('--source', choices=('real', 'sim'), default='real',
                        help='vision_node 입력: real = 휴대폰 (실물 → Gazebo 자동 반영), sim = Gazebo 가상 카메라')
    args, qt_args = parser.parse_known_args()
    app = QApplication([sys.argv[0]] + qt_args)
    configure_korean_font()
    win = MainWindow(args.calib_dir, source=args.source)
    win.resize(1680 if args.source == 'real' else 1440, 960)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
