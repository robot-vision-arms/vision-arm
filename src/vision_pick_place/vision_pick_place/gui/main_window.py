"""PySide6 메인 윈도우 (3주차). 과제 GUI 필수 화면 4가지를 한 창에 배치한다 (docs/PROJECT.md 2.2).

    ┌───────────────────────────────┬──────────────────────┐
    │ ① 영상 + 검출 ROI + 장축 화살표 │ ② px·mm 좌표, 회전각, 종류 │
    │   (/vision/debug_image)        │ ④ 캘리브레이션 상태     │
    ├───────────────────────────────┴──────────────────────┤
    │ ③ 송수신 기록: TCP (/robot/log) + Gazebo 집기 (/sim/pick_log), 요약 (ACK 비율·지연 P95) │
    └──────────────────────────────────────────────────────┘

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
from pathlib import Path
import sys
import time

import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QImage, QPixmap
from PySide6.QtWidgets import (QApplication, QFormLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel,
                               QMainWindow, QPushButton, QSizePolicy, QSplitter, QTableWidget,
                               QTableWidgetItem, QVBoxLayout, QWidget)

from vision_pick_place.calibrator import Calibrator
from vision_pick_place.communication_check import percentile95
from vision_pick_place.gui.pipeline_inspector import configure_korean_font

PART_NAMES = {'screw': '나사', 'nut': '너트', 'unknown': '모름'}
LOG_ROWS = 200        # ③ 표에 남길 최근 기록 수
SUMMARY_WINDOW = 100  # ③ 요약(ACK 비율·P95) 을 계산할 최근 송신 수
FAIL_COLOR = QColor(255, 215, 215)
OK_COLOR = QColor(220, 245, 220)


def _fmt(value, digits=1):
    return '—' if value is None else f'{value:.{digits}f}'


class VideoPanel(QLabel):
    """① 영상. 창 크기에 맞춰 비율을 유지하며 줄인다."""

    def __init__(self):
        super().__init__('영상 없음\n(/vision/debug_image 대기 중)')
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setStyleSheet('background: #202020; color: #bbbbbb;')
        self._pixmap = None

    def show_frame(self, bgr: np.ndarray):
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        image = QImage(rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QImage.Format_RGB888).copy()
        self._pixmap = QPixmap.fromImage(image)
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
        self.calib_dir = Path(calib_dir)
        self.last_packet_time = None
        self.packet_times = deque(maxlen=30)     # 패킷 수신 주기 (Hz 표시)
        self.sends = deque(maxlen=SUMMARY_WINDOW)  # (acked, latency_ms)
        self.picks = []                          # Gazebo 집기 결과 'ok'/'failed'

        # ① 영상
        self.video = VideoPanel()
        # ② 좌표 표
        self.det_table = QTableWidget(0, 7)
        self.det_table.setHorizontalHeaderLabels(['id', '종류', 'u px', 'v px', 'x mm', 'y mm', '각도 °'])
        self.det_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.det_table.verticalHeader().setVisible(False)
        self.det_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.det_status = QLabel('패킷 대기 중 (/vision/detections)')
        det_box = QGroupBox('② 검출 좌표 (작업 시트 mm, 각도 = +X 에서 반시계 [0°, 180°))')
        det_layout = QVBoxLayout(det_box)
        det_layout.addWidget(self.det_status)
        det_layout.addWidget(self.det_table)
        # ④ 캘리브 상태
        calib_box = QGroupBox('④ 캘리브레이션 상태')
        self.calib_form = QFormLayout(calib_box)
        self.calib_labels = {}
        for key in ('dir', 'intrinsics', 'homography', 'camera', 'packet'):
            self.calib_labels[key] = QLabel('—')
            self.calib_labels[key].setWordWrap(True)
        self.calib_form.addRow('보정 폴더', self.calib_labels['dir'])
        self.calib_form.addRow('내부 파라미터', self.calib_labels['intrinsics'])
        self.calib_form.addRow('호모그래피', self.calib_labels['homography'])
        self.calib_form.addRow('카메라 높이', self.calib_labels['camera'])
        self.calib_form.addRow('패킷 calib_ok', self.calib_labels['packet'])
        # ③ 송수신 기록
        self.log_table = QTableWidget(0, 6)
        self.log_table.setHorizontalHeaderLabels(['시각', '출처', 'seq', '결과', 'ms', '내용'])
        header = self.log_table.horizontalHeader()
        for col in range(5):
            header.setSectionResizeMode(col, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.Stretch)
        self.log_table.verticalHeader().setVisible(False)
        self.log_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.log_summary = QLabel('TCP 송신 기록 없음 | Gazebo 집기 기록 없음')
        self.cycle_button = QPushButton('Gazebo 집기 한 사이클 (/sim/run_cycle)')
        self.cycle_button.clicked.connect(self.request_cycle)
        log_box = QGroupBox('③ 송수신 기록 (TCP = 가상 로봇 ACK, Gazebo = 집기 결과)')
        log_layout = QVBoxLayout(log_box)
        top = QHBoxLayout()
        top.addWidget(self.log_summary, 1)
        top.addWidget(self.cycle_button)
        log_layout.addLayout(top)
        log_layout.addWidget(self.log_table)

        video_box = QGroupBox('① 영상 (노란 선 = 검출 영역, 초록 = 검출 박스, 파랑 = 장축 방향)')
        QVBoxLayout(video_box).addWidget(self.video)
        right = QSplitter(Qt.Vertical)
        right.addWidget(det_box)
        right.addWidget(calib_box)
        upper = QSplitter(Qt.Horizontal)
        upper.addWidget(video_box)
        upper.addWidget(right)
        upper.setSizes([760, 520])
        root = QSplitter(Qt.Vertical)
        root.addWidget(upper)
        root.addWidget(log_box)
        root.setSizes([560, 240])
        self.setCentralWidget(root)

        self.load_calibration()
        self.age_timer = QTimer(self)   # 패킷이 끊겼는지 보이도록 1초마다 경과 시간 갱신
        self.age_timer.timeout.connect(self.update_packet_age)
        self.age_timer.start(1000)

        self.worker = None
        if start_worker:
            from vision_pick_place.gui.ros_worker import RosWorker
            self.worker = RosWorker(self)
            self.worker.detections.connect(self.on_detections)
            self.worker.frame.connect(self.video.show_frame)
            self.worker.robot_log.connect(self.on_robot_log)
            self.worker.sim_log.connect(self.on_sim_log)
            self.worker.cycle_result.connect(self.on_cycle_result)
            self.worker.start()

    # ---------- ④ ----------

    def load_calibration(self):
        """보정 파일을 읽어 상태를 표시한다. 파일 수정 시각으로 '카메라가 움직인 뒤 다시 보정했는지' 판단한다."""
        intr, hom = self.calib_dir / 'intrinsics.npz', self.calib_dir / 'homography.npz'
        self.calib_labels['dir'].setText(str(self.calib_dir))
        try:
            calib = Calibrator(str(intr), str(hom))
        except (ValueError, OSError) as exc:
            for key in ('intrinsics', 'homography', 'camera'):
                self.calib_labels[key].setText(f'오류: {exc}')
            return
        status = calib.status

        def stamp(path):
            return datetime.fromtimestamp(path.stat().st_mtime).strftime('%m-%d %H:%M')

        self.calib_labels['intrinsics'].setText(
            f'정상 · RMS {status.rms_px:.2f} px, 저장 {stamp(intr)}' if status.intrinsics_loaded else '없음 (미보정)')
        self.calib_labels['homography'].setText(
            f'정상 · 재투영 {status.reproj_err_mm:.2f} mm, 저장 {stamp(hom)}' if status.homography_loaded
            else '없음 (미보정)')
        if status.ready:
            x, y, z = calib.camera_center_mm()
            self.calib_labels['camera'].setText(f'{z:.0f} mm (시트 위 ({x:.0f}, {y:.0f}) mm)')
        else:
            self.calib_labels['camera'].setText('—')

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
            values = [str(obj.get('id', '')), PART_NAMES.get(obj.get('part_type'), str(obj.get('part_type'))),
                      _fmt(obj.get('u_px')), _fmt(obj.get('v_px')),
                      _fmt(obj.get('x_mm')), _fmt(obj.get('y_mm')), _fmt(obj.get('angle_deg'))]
            for col, text in enumerate(values):
                item = QTableWidgetItem(text)
                item.setTextAlignment(Qt.AlignCenter)
                self.det_table.setItem(row, col, item)
        rate = f', {1 / np.mean(self.packet_times):.1f} Hz' if self.packet_times else ''
        self.det_status.setText(f'seq {packet.get("seq")}: 물체 {len(objects)}개{rate}'
                                + ('' if calib_ok else '  [미보정 — mm·각도 없음]'))
        self.calib_labels['packet'].setText('true (보정됨)' if calib_ok else 'false — 미보정, mm 좌표를 쓰면 안 됨')

    def update_packet_age(self):
        if self.last_packet_time is not None and time.monotonic() - self.last_packet_time > 2.0:
            self.det_status.setText(f'[주의] {time.monotonic() - self.last_packet_time:.0f}초째 패킷 없음 (vision_node 확인)')

    # ---------- ③ ----------

    def add_log_row(self, source: str, seq, ok: bool, ms, text: str):
        self.log_table.insertRow(0)
        values = [datetime.now().strftime('%H:%M:%S'), source, '' if seq is None else str(seq),
                  '성공' if ok else '실패', _fmt(ms), text]
        for col, value in enumerate(values):
            item = QTableWidgetItem(value)
            item.setBackground(OK_COLOR if ok else FAIL_COLOR)
            self.log_table.setItem(0, col, item)
        while self.log_table.rowCount() > LOG_ROWS:
            self.log_table.removeRow(self.log_table.rowCount() - 1)

    def on_robot_log(self, log: dict):
        ok = bool(log.get('acked'))
        self.sends.append((ok, log.get('latency_ms') if ok else None))
        self.add_log_row('TCP', log.get('seq'), ok, log.get('latency_ms'),
                         'ACK' if ok else (log.get('error') or '응답 없음'))
        self.update_summary()

    def on_sim_log(self, log: dict):
        ok = log.get('result') == 'ok'
        if 'obj' in log:   # run_cycle 의 물체 하나 (move_to_target 기록은 action 키만 있다)
            self.picks.append(ok)
            text = f'{log.get("obj")} {PART_NAMES.get(log.get("part_type"), log.get("part_type"))} → 상자'
        else:
            text = log.get('action', '')
        if not ok and log.get('error'):
            text += f' — {log["error"]}'
        self.add_log_row('Gazebo', log.get('seq'), ok, log.get('plan_ms'), text)
        self.update_summary()

    def update_summary(self):
        parts = []
        if self.sends:
            acked = [ms for ok, ms in self.sends if ok and ms is not None]
            p95 = f', 지연 P95 {percentile95(acked):.0f} ms' if acked else ''
            parts.append(f'TCP 최근 {len(self.sends)}건: ACK {sum(ok for ok, _ in self.sends)}/{len(self.sends)}{p95}')
        else:
            parts.append('TCP 송신 기록 없음')
        parts.append(f'Gazebo 집기 {sum(self.picks)}/{len(self.picks)} 성공' if self.picks else 'Gazebo 집기 기록 없음')
        self.log_summary.setText(' | '.join(parts))

    def request_cycle(self):
        if self.worker is None:
            return
        self.cycle_button.setEnabled(False)
        self.cycle_button.setText('집기 사이클 실행 중...')
        self.worker.request_cycle()

    def on_cycle_result(self, ok: bool, message: str):
        self.cycle_button.setEnabled(True)
        self.cycle_button.setText('Gazebo 집기 한 사이클 (/sim/run_cycle)')
        if not ok:
            self.add_log_row('Gazebo', None, False, None, message)

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
    win.resize(1280, 800)
    win.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
