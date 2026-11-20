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
    assert 'RMS 0.56 px' in labels['intrinsics'].text()
    assert '재투영 0.21 mm' in labels['homography'].text()
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
    assert [cell(table, 0, c) for c in range(7)] == ['0', '나사', '640.2', '360.8', '102.4', '55.0', '37.5']
    assert cell(table, 1, 1) == '너트' and cell(table, 1, 6) == '—'
    assert window.calib_labels['packet'].text().startswith('true')
    window.on_detections({'seq': 8, 'calib_ok': False, 'objects': [
        {'id': 0, 'part_type': 'screw', 'x_mm': None, 'y_mm': None, 'angle_deg': None, 'u_px': 1.0, 'v_px': 2.0}]})
    assert '미보정' in window.det_status.text() and cell(window.det_table, 0, 4) == '—'


def test_log_rows_and_summary(window):
    for seq in range(10):
        window.on_robot_log({'seq': seq, 'sent': True, 'acked': seq != 3, 'latency_ms': 10.0 + seq,
                             'error': '' if seq != 3 else 'ACK timeout'})
    window.on_sim_log({'seq': 1, 'obj': 'obj_0', 'part_type': 'screw', 'result': 'ok', 'plan_ms': 35000.0, 'error': ''})
    window.on_sim_log({'seq': 1, 'obj': 'obj_1', 'part_type': 'nut', 'result': 'failed', 'plan_ms': 2000.0,
                       'error': '[흡착 높이로 하강·진공] 흡착 실패'})
    table = window.log_table
    assert table.rowCount() == 12
    assert cell(table, 0, 1) == 'Gazebo' and cell(table, 0, 3) == '실패' and '흡착 실패' in cell(table, 0, 5)
    summary = window.log_summary.text()
    assert 'ACK 9/10' in summary and 'P95' in summary and 'Gazebo 집기 1/2 성공' in summary


def test_video_panel_accepts_bgr_frame(window):
    frame = np.zeros((720, 1280, 3), np.uint8)
    frame[:, :, 1] = 200
    window.video.show_frame(frame)
    assert window.video.pixmap() is not None and not window.video.pixmap().isNull()
