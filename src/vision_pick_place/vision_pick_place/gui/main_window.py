"""PySide6 메인 윈도우 (3주차). 과제 GUI 필수 화면 4가지 (docs/PROJECT.md 2.2), 11-10 목업과 같은 구조.

    ┌ 헤더: 제목·입력 │ 연결 상태 (ROS2 · robot_server · Gazebo) │ [보정 다시 읽기] [부품 놓기] [사이클 실행] ┐
    ├──────────────────────────────┬──────────────────────────────┤
    │ ① 영상 · 검출 ROI · 장축 방향  │ ② 좌표 · 회전각 · 부품 종류      │
    │   탭: 검출 카메라 / Gazebo 전경 │   표 + 기준 타일 3개             │
    ├──────────────────────────────┼──────────────────────────────┤
    │ ③ 가상 로봇 송수신 · 집기 결과  │ ④ 캘리브레이션 상태              │
    │   콘솔 로그 + 숫자 타일 3개     │   항목 목록 + calib_ok 배지      │
    └──────────────────────────────┴──────────────────────────────┘

    source env.sh   # venv (PySide6) 를 켠다
    python3 -m vision_pick_place.gui.main_window                              # 실물 보정 (data/calib)
    python3 -m vision_pick_place.gui.main_window --calib-dir data/calib/sim   # Gazebo 가상 카메라 보정
    (ros2 run vision_pick_place gui 는 --symlink-install 빌드에서 시스템 python 으로 떠서 PySide6 를 못 찾는다)

ROS 는 ros_worker.RosWorker (QThread) 가 받고, 이 창은 Signal 로 받은 dict/영상으로 위젯만 갱신한다.
갱신 함수(on_detections 등)는 ROS 없이 가짜 데이터로 시험할 수 있다 (test/test_gui.py).
"""
import argparse
from collections import deque
from datetime import datetime
import html
from pathlib import Path
import sys
import time

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QBrush, QColor, QImage, QPixmap
from PySide6.QtWidgets import (QApplication, QFrame, QGridLayout, QHBoxLayout, QHeaderView, QLabel, QMainWindow,
                               QPlainTextEdit, QPushButton, QSizePolicy, QTableWidget, QTableWidgetItem, QTabWidget,
                               QVBoxLayout, QWidget)

from vision_pick_place.calibrator import Calibrator
from vision_pick_place.communication_check import percentile95
from vision_pick_place.gui.pipeline_inspector import configure_korean_font

LOG_LINES = 300       # ③ 콘솔에 남길 줄 수
SUMMARY_WINDOW = 100  # ③ ACK 성공률·P95 를 계산할 최근 송신 수
STALE_S = 2.0         # 이 시간 동안 아무것도 안 오면 연결 표시를 회색으로

# 목업 색 (11-10 GUI 대시보드 목업)
BG, CARD, BORDER, LINE = '#EEF0F3', '#FFFFFF', '#DDE1E6', '#E4E7EB'
INK, MUTED, BLUE = '#16191D', '#5B6470', '#1D5FD1'
OK_FG, OK_BG, BAD_FG, BAD_BG, OFF_FG, OFF_BG = '#13734B', '#E6F4EC', '#B42318', '#FDECEA', '#5B6470', '#E4E7EB'
PART_BADGE = {'screw': ('나사', BLUE, '#E8EFFB'), 'nut': ('너트', '#A34808', '#FBEDE3'),
              'unknown': ('모름', MUTED, '#EEF0F3')}
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
QTableWidget {{ border: 1px solid {LINE}; border-radius: 8px; gridline-color: {LINE}; font-family: {MONO}; font-size: 13px; }}
QHeaderView::section {{ background: #F4F5F7; color: #3E4650; border: none; padding: 8px; font-weight: 500; }}
QPlainTextEdit#console {{ background: #16191D; color: #D7DCE2; border-radius: 8px; padding: 8px;
                         font-family: {MONO}; font-size: 12px; }}
"""


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
    def __init__(self, calib_dir: str = 'data/calib', start_worker: bool = True):
        super().__init__()
        self.setWindowTitle('Vision Pick & Place')
        self.setStyleSheet(STYLE)
        self.calib_dir = Path(calib_dir)
        self.last_packet_time = None
        self.last_ack_time = None
        self.packet_times = deque(maxlen=30)       # 패킷 수신 주기 (Hz 표시)
        self.sends = deque(maxlen=SUMMARY_WINDOW)   # (acked, latency_ms)
        self.picks = []                             # Gazebo 집기 결과 True/False
        self.pending_acks = []                      # 아직 콘솔에 안 쓴 ACK (seq, latency_ms). 1초마다 한 줄로 묶는다
        self.gazebo = False

        root = QWidget()
        root.setObjectName('root')
        outer = QVBoxLayout(root)
        outer.setContentsMargins(20, 20, 20, 20)
        outer.setSpacing(16)
        outer.addWidget(self._header())
        grid = QGridLayout()
        grid.setSpacing(16)
        grid.addWidget(self._video_card(), 0, 0)
        grid.addWidget(self._detection_card(), 0, 1)
        grid.addWidget(self._log_card(), 1, 0)
        grid.addWidget(self._calib_card(), 1, 1)
        grid.setRowStretch(0, 3)
        grid.setRowStretch(1, 2)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        outer.addLayout(grid, 1)
        self.setCentralWidget(root)

        self.load_calibration()
        self.status_timer = QTimer(self)   # 1초마다 연결 표시·패킷 끊김 갱신
        self.status_timer.timeout.connect(self.update_status)
        self.status_timer.start(1000)
        # vision_node 가 물체가 없어도 초당 몇 번씩 보내고 매번 ACK 가 와서, 한 줄씩 쓰면 집기 결과가 밀려 올라간다
        self.ack_timer = QTimer(self)
        self.ack_timer.timeout.connect(self.flush_acks)
        self.ack_timer.start(1000)
        self.update_status()

        self.worker = None
        if start_worker:
            from vision_pick_place.gui.ros_worker import RosWorker
            self.worker = RosWorker(self)
            self.worker.detections.connect(self.on_detections)
            self.worker.frame.connect(self.video.show_frame)
            self.worker.overview.connect(self.overview.show_frame)
            self.worker.robot_log.connect(self.on_robot_log)
            self.worker.sim_log.connect(self.on_sim_log)
            self.worker.cycle_result.connect(self.on_cycle_result)
            self.worker.spawn_result.connect(self.on_spawn_result)
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
        self.input_label = QLabel('입력: /vision/debug_image')
        self.input_label.setObjectName('meta')
        names.addWidget(title)
        names.addWidget(self.input_label)
        layout.addLayout(names, 1)
        self.pill_ros, self.pill_robot, self.pill_gazebo = QLabel(), QLabel(), QLabel()
        for pill in (self.pill_ros, self.pill_robot, self.pill_gazebo):
            layout.addWidget(pill)
        layout.addSpacing(8)
        reload_button = QPushButton('보정 다시 읽기')
        reload_button.clicked.connect(self.load_calibration)
        # 시뮬 시연용: 사이클이 끝나면 매트가 비므로 새 부품을 놓는다 (실물에서는 사람이 놓는다)
        self.spawn_button = QPushButton('부품 놓기 (무작위)')
        self.spawn_button.clicked.connect(self.request_spawn)
        layout.addWidget(self.spawn_button)
        self.cycle_button = QPushButton('사이클 실행 (/sim/run_cycle)')
        self.cycle_button.setObjectName('primary')
        self.cycle_button.clicked.connect(self.request_cycle)
        layout.addWidget(reload_button)
        layout.addWidget(self.cycle_button)
        return bar

    def _video_card(self) -> Card:
        self.video_card = Card('① 영상 · 검출 ROI · 장축 방향')
        self.video = VideoPanel()
        # Gazebo 전경 카메라 (11-10 목업의 Gazebo 시점). Gazebo 창을 띄우지 않아도 집기 과정을 볼 수 있다
        self.overview = VideoPanel('/sim/overview/image')
        self.video_tabs = QTabWidget()
        self.video_tabs.addTab(self.video, '검출 카메라')
        self.video_tabs.addTab(self.overview, 'Gazebo 전경')
        self.video_card.body.addWidget(self.video_tabs, 1)
        legend = QLabel('<span style="color:#C8A400">━</span> 검출 영역 (시트 안쪽, 마커 제외) &nbsp;&nbsp; '
                        '<span style="color:#1FA34A">━</span> 검출 박스 &nbsp;&nbsp; '
                        f'<span style="color:{BLUE}">→</span> 장축 방향 (나사)')
        legend.setObjectName('note')
        self.video_card.body.addWidget(legend)
        return self.video_card

    def _detection_card(self) -> Card:
        self.det_card = Card('② 좌표 · 회전각 · 부품 종류')
        self.det_card.meta.setText('기준: 작업 평면 (work_plane)')
        self.det_status = QLabel('패킷 대기 중 (/vision/detections)')
        self.det_status.setObjectName('note')
        self.det_table = QTableWidget(0, 5)
        self.det_table.setHorizontalHeaderLabels(['#', '종류', 'px (u, v)', 'mm (X, Y)', '각도'])
        header = self.det_table.horizontalHeader()
        header.setSectionResizeMode(QHeaderView.Stretch)
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.det_table.verticalHeader().setVisible(False)
        self.det_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.det_table.setSelectionMode(QTableWidget.NoSelection)
        self.det_card.body.addWidget(self.det_status)
        self.det_card.body.addWidget(self.det_table, 1)
        tiles = QHBoxLayout()
        self.tile_rule = Tile('나사 / 너트 기준', '장축/단축 > 1.5 → 나사')
        self.tile_angle = Tile('각도 기준', 'mm 평면, [0°, 180°)')
        self.tile_target = Tile('송신 대상', '—')
        for tile in (self.tile_rule, self.tile_angle, self.tile_target):
            tiles.addWidget(tile)
        self.det_card.body.addLayout(tiles)
        return self.det_card

    def _log_card(self) -> Card:
        card = Card('③ 가상 로봇 송수신 · Gazebo 집기 결과')
        card.meta.setText('/robot/log · /sim/pick_log')
        self.console = QPlainTextEdit()
        self.console.setObjectName('console')
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(LOG_LINES)
        card.body.addWidget(self.console, 1)
        tiles = QHBoxLayout()
        self.tile_ack = Tile('ACK 성공률')
        self.tile_p95 = Tile('지연 P95')
        self.tile_pick = Tile('집기 성공')
        for tile in (self.tile_ack, self.tile_p95, self.tile_pick):
            tiles.addWidget(tile)
        card.body.addLayout(tiles)
        return card

    def _calib_card(self) -> Card:
        card = Card('④ 캘리브레이션 상태')
        card.meta.hide()
        self.calib_pill = QLabel()
        card.head.addWidget(self.calib_pill)
        box = QFrame()
        box.setObjectName('tile')
        rows = QVBoxLayout(box)
        rows.setContentsMargins(0, 0, 0, 0)
        rows.setSpacing(0)
        self.calib_labels = {}
        items = [('dir', '보정 폴더'), ('intrinsics', '내부 파라미터 (ChArUco)'),
                 ('homography', '호모그래피 (ArUco 작업 시트 px → mm)'), ('error', '재투영 오차'),
                 ('camera', '카메라 높이 (시차 보정)'), ('saved', '보정 저장 시각')]
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
        note = QLabel('카메라나 작업 시트가 움직이면 시트 사진을 다시 찍어 호모그래피를 다시 계산합니다. '
                      '보정이 안 돼 있으면 calib_ok=false 로 송신되고, 로봇은 그 패킷을 거부합니다.')
        note.setObjectName('note')
        note.setWordWrap(True)
        card.body.addWidget(note)
        card.body.addStretch(1)
        _pill(self.calib_pill, 'calib_ok 대기', 'off')
        return card

    # ---------- ④ ----------

    def load_calibration(self):
        """보정 파일을 읽어 상태를 표시한다. 파일 저장 시각으로 '카메라가 움직인 뒤 다시 보정했는지' 판단한다."""
        intr, hom = self.calib_dir / 'intrinsics.npz', self.calib_dir / 'homography.npz'
        labels = self.calib_labels
        labels['dir'].setText(str(self.calib_dir))
        try:
            calib = Calibrator(str(intr), str(hom))
        except (ValueError, OSError) as exc:
            for key in ('homography', 'error', 'camera', 'saved'):
                labels[key].setText('—')
            labels['intrinsics'].setText(f'오류: {exc}')
            return
        status = calib.status

        def loaded(ok, name):
            color = OK_FG if ok else BAD_FG
            return f'<span style="color:{color}">{name} {"로드됨" if ok else "없음 (미보정)"}</span>'

        labels['intrinsics'].setText(loaded(status.intrinsics_loaded, 'intrinsics.npz'))
        labels['homography'].setText(loaded(status.homography_loaded, 'homography.npz'))
        labels['error'].setText(f'{_fmt(status.rms_px, 2)} px / {_fmt(status.reproj_err_mm, 2)} mm')
        if status.ready:
            x, y, z = calib.camera_center_mm()
            labels['camera'].setText(f'{z:.0f} mm (시트 ({x:.0f}, {y:.0f}) 위)')
        else:
            labels['camera'].setText('—')
        stamps = [datetime.fromtimestamp(p.stat().st_mtime).strftime('%m-%d %H:%M') for p in (intr, hom) if p.is_file()]
        labels['saved'].setText(' / '.join(stamps) or '—')

    # ---------- ② ----------

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
        self.det_card.meta.setText(f'seq {packet.get("seq")} · 기준: 작업 평면 (work_plane)')
        self.det_status.setText(f'물체 {len(objects)}개{rate}' + ('' if calib_ok else '  [미보정 — mm·각도 없음]'))
        self.video_card.meta.setText(f'부품 {len(objects)}개{rate}')
        sendable = sum(o.get('part_type') in ('screw', 'nut') and o.get('x_mm') is not None for o in objects)
        self.tile_target.value.setText(f'{sendable}개 (집기 최대 3)')
        _pill(self.calib_pill, f'calib_ok = {"true" if calib_ok else "false"}', 'ok' if calib_ok else 'bad')
        if self.video.frame_size:
            self.input_label.setText(f'입력: /vision/debug_image · {self.video.frame_size[0]}×{self.video.frame_size[1]}')

    # ---------- ③ ----------

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
            self.pending_acks.append((log.get('seq'), log.get('latency_ms')))
        else:
            self.flush_acks()   # 순서를 지키려고 모아 둔 ACK 를 먼저 쓴다
            self.log_line('ERR', CONSOLE['err'], f'seq={log.get("seq")}  {log.get("error") or "응답 없음"}')
        self.update_tiles()

    def flush_acks(self):
        """모아 둔 ACK 를 한 줄로: 'ACK  seq 120~124  5건, 평균 14 ms'."""
        if not self.pending_acks:
            return
        seqs = [s for s, _ in self.pending_acks]
        ms = [v for _, v in self.pending_acks if v is not None]
        span = f'seq {seqs[0]}' if len(seqs) == 1 else f'seq {seqs[0]}~{seqs[-1]}'
        self.log_line('ACK', CONSOLE['ack'], f'{span}  {len(seqs)}건' + (f', 평균 {np.mean(ms):.0f} ms' if ms else ''))
        self.pending_acks = []

    def on_sim_log(self, log: dict):
        self.flush_acks()
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
        _pill(self.pill_ros, 'ROS2 비전' if fresh else 'ROS2 비전 대기', 'ok' if fresh else 'off')
        acked = self.last_ack_time is not None and now - self.last_ack_time < STALE_S
        _pill(self.pill_robot, 'robot_server ACK' if acked else 'robot_server 대기', 'ok' if acked else 'off')
        _pill(self.pill_gazebo, 'Gazebo' if self.gazebo else 'Gazebo 없음', 'ok' if self.gazebo else 'off')
        if self.last_packet_time is not None and not fresh:
            self.det_status.setText(f'[주의] {now - self.last_packet_time:.0f}초째 패킷 없음 (vision_node 확인)')

    def request_cycle(self):
        if self.worker is None:
            return
        self.cycle_button.setEnabled(False)
        self.cycle_button.setText('사이클 실행 중...')
        self.log_line('SIM', CONSOLE['sim'], '/sim/run_cycle 요청')
        self.worker.request_cycle()

    def request_spawn(self):
        if self.worker is None:
            return
        self.spawn_button.setEnabled(False)
        self.log_line('SIM', CONSOLE['sim'], '/sim/spawn_random 요청')
        self.worker.request_spawn()

    def on_spawn_result(self, ok: bool, message: str):
        self.spawn_button.setEnabled(True)
        self.log_line('SIM', CONSOLE['sim'] if ok else CONSOLE['err'], message)

    def on_cycle_result(self, ok: bool, message: str):
        self.cycle_button.setEnabled(True)
        self.cycle_button.setText('사이클 실행 (/sim/run_cycle)')
        self.log_line('SIM', CONSOLE['ack'] if ok else CONSOLE['err'], message)

    def closeEvent(self, event):
        if self.worker is not None:
            self.worker.stop()
        super().closeEvent(event)


def main():
    parser = argparse.ArgumentParser(description='Vision Pick & Place GUI')
    parser.add_argument('--calib-dir', default='data/calib', help='intrinsics.npz·homography.npz 폴더 (가상 카메라: data/calib/sim)')
    args, qt_args = parser.parse_known_args()
    app = QApplication([sys.argv[0]] + qt_args)
    configure_korean_font()
    win = MainWindow(args.calib_dir)
    win.resize(1440, 960)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
