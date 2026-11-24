"""GUI 4화면 갱신 확인. ROS 없이 가짜 패킷·로그를 슬롯에 넣는다 (QT_QPA_PLATFORM=offscreen)."""
import importlib.util
import os

import numpy as np
import pytest

# PySide6 는 venv 에만 있다. colcon test 가 시스템 python 으로 돌면 이 파일만 건너뛴다.
# (모듈 맨 위 importorskip 은 시스템 python 에서 다른 테스트 파일 수집까지 멈추게 해서 쓰지 않는다)
pytestmark = pytest.mark.skipif(importlib.util.find_spec('PySide6') is None, reason='PySide6 없음 (venv 에서 실행)')
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def _qt():
    from PySide6.QtWidgets import QApplication
    from vision_pick_place.gui.main_window import MainWindow
    return QApplication.instance() or QApplication([]), MainWindow


@pytest.fixture
def window(tmp_path):
    app, MainWindow = _qt()
    K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]])
    np.savez(tmp_path / 'intrinsics.npz', camera_matrix=K, dist_coeffs=np.zeros(5), rms=0.56)
    # 위에서 똑바로 내려다보는 카메라 (높이 600 mm, 시트 (150, 100) mm 위): mm → px 를 거꾸로 한 H
    f, h = 900.0, 600.0
    to_px = np.array([[f / h, 0, 640 - f / h * 150], [0, -f / h, 360 + f / h * 100], [0, 0, 1]])
    np.savez(tmp_path / 'homography.npz', H=np.linalg.inv(to_px), reproj_err_mm=0.21)
    win = MainWindow(str(tmp_path), start_worker=False)
    yield win
    win.close()
    app.processEvents()


def cell(table, row, col):
    return table.item(row, col).text()


def test_calibration_panel_shows_loaded_files(window):
    labels = window.calib_labels
    assert '로드됨' in labels['intrinsics'].text() and '로드됨' in labels['homography'].text()
    assert labels['error'].text() == '0.56 px / 0.21 mm'
    assert labels['camera'].text().startswith('600 mm')


def test_missing_calibration_is_shown_as_uncalibrated(tmp_path):
    app, MainWindow = _qt()
    win = MainWindow(str(tmp_path / 'none'), start_worker=False)
    assert '없음' in win.calib_labels['intrinsics'].text() and '없음' in win.calib_labels['homography'].text()
    win.close()
    app.processEvents()


def test_detections_table_and_uncalibrated_warning(window):
    window.on_detections({'seq': 7, 'calib_ok': True, 'objects': [
        {'id': 0, 'part_type': 'screw', 'x_mm': 102.44, 'y_mm': 55.0, 'angle_deg': 37.52, 'u_px': 640.2, 'v_px': 360.8},
        {'id': 1, 'part_type': 'nut', 'x_mm': 40.1, 'y_mm': 80.3, 'angle_deg': None, 'u_px': 410.7, 'v_px': 288.0}]})
    table = window.det_table
    assert table.rowCount() == 2
    assert [cell(table, 0, c) for c in range(5)] == ['0', '나사', '640.2, 360.8', '102.4, 55.0', '37.5°']
    assert cell(table, 1, 1) == '너트' and cell(table, 1, 4) == '—'
    assert 'calib_ok = true' in window.calib_pill.text()
    assert window.tile_target.value.text().startswith('2개')
    window.on_detections({'seq': 8, 'calib_ok': False, 'objects': [
        {'id': 0, 'part_type': 'screw', 'x_mm': None, 'y_mm': None, 'angle_deg': None, 'u_px': 1.0, 'v_px': 2.0}]})
    assert '미보정' in window.det_status.text() and cell(window.det_table, 0, 3) == '—, —'
    assert 'calib_ok = false' in window.calib_pill.text()


def test_log_rows_and_summary(window):
    for seq in range(10):
        window.on_robot_log({'seq': seq, 'sent': True, 'acked': seq != 3, 'latency_ms': 10.0 + seq,
                             'error': '' if seq != 3 else 'ACK timeout'})
    window.on_sim_log({'seq': 1, 'obj': 'obj_0', 'part_type': 'screw', 'result': 'ok', 'plan_ms': 35000.0, 'error': ''})
    window.on_sim_log({'seq': 1, 'obj': 'obj_1', 'part_type': 'nut', 'result': 'failed', 'plan_ms': 2000.0,
                       'error': '[흡착 높이로 하강·진공] 흡착 실패'})
    lines = window.console.toPlainText().splitlines()
    # ACK 는 묶어서 쓰고, ERR·SIM 이 오면 그 전까지 모은 ACK 를 먼저 쓴다
    assert len(lines) == 5
    assert 'ACK' in lines[0] and 'seq 0~2' in lines[0] and '3건' in lines[0]
    assert 'ERR' in lines[1] and 'ACK timeout' in lines[1]
    assert 'seq 4~9' in lines[2] and '6건' in lines[2] and '평균 16 ms' in lines[2]
    assert 'SIM' in lines[-1] and '실패' in lines[-1] and '흡착 실패' in lines[-1]
    assert window.tile_ack.value.text() == '90 % (9/10)'
    assert window.tile_p95.value.text().endswith('ms')
    assert window.tile_pick.value.text() == '1 / 2'


def test_video_panel_accepts_bgr_frame(window):
    frame = np.zeros((720, 1280, 3), np.uint8)
    frame[:, :, 1] = 200
    window.video.show_frame(frame)
    assert window.video.pixmap() is not None and not window.video.pixmap().isNull()


class FakeWorker:
    def __init__(self):
        self.requests = []

    def request_service(self, name):
        self.requests.append(name)

    def stop(self):
        pass


def _packet(seq, objs):
    return {'seq': seq, 'calib_ok': True, 'objects': [
        {'id': i, 'part_type': k, 'x_mm': x, 'y_mm': y, 'angle_deg': None, 'u_px': 0.0, 'v_px': 0.0}
        for i, (k, x, y) in enumerate(objs)]}


def test_real_layout_is_mirrored_to_gazebo_only_when_stable(window):
    from vision_pick_place.gui.main_window import STABLE_FRAMES
    worker = FakeWorker()
    window.worker = worker
    scene = [('screw', 100.0, 80.0), ('nut', 180.0, 140.0)]
    # 손이 들어와 흔들리는 동안 (매 패킷 위치가 다름) 은 반영하지 않는다
    for i in range(6):
        window.on_detections(_packet(i, [('screw', 100.0 + 10 * i, 80.0)]))
    assert worker.requests == []
    # 같은 배치가 STABLE_FRAMES 번 이어지면 한 번만 스폰
    for i in range(STABLE_FRAMES + 3):
        window.on_detections(_packet(10 + i, [(k, x + 0.5 * (i % 2), y) for k, x, y in scene]))
    assert worker.requests == ['/sim/spawn_objects']
    window.on_service_result('/sim/spawn_objects', True, 'seq=13 스폰 2개')
    # 사이클 중에는 반영 안 함, 사이클 뒤 실물이 그대로면 다시 스폰하지 않음
    window.request_cycle()
    for i in range(STABLE_FRAMES + 1):
        window.on_detections(_packet(20 + i, scene))
    window.on_service_result('/sim/run_cycle', True, '2/2 투입')
    for i in range(STABLE_FRAMES + 1):
        window.on_detections(_packet(30 + i, scene))
    assert worker.requests == ['/sim/spawn_objects', '/sim/run_cycle']
    # 다 치우면 Gazebo 도 지운다
    for i in range(STABLE_FRAMES):
        window.on_detections(_packet(40 + i, []))
    assert worker.requests[-1] == '/sim/clear_objects'
