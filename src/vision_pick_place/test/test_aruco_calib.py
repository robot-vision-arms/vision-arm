"""ArUco 캘리브레이션: 인쇄물 정의, 작업 시트 호모그래피, ChArUco 내부 파라미터, 스크립트 왕복."""
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
import pytest

from vision_pick_place import aruco_calib as ac
from vision_pick_place.calibrator import Calibrator

SCRIPTS = Path(__file__).resolve().parents[3] / 'scripts'
K = np.array([[900., 0, 640], [0, 900., 360], [0, 0, 1]])
SIZE = (1280, 720)
# 보드를 여러 각도로 기울인 자세 (rx, ry 라디안, tx, ty mm). z 는 600 mm 고정
POSES = [(0, 0, -126, -90), (0.3, 0, -150, -80), (-0.3, 0.1, -100, -100), (0.1, 0.35, -140, -90),
         (0, -0.35, -110, -70), (0.25, 0.25, -130, -100), (-0.2, -0.3, -120, -60)]


def view_of_plane(plane_image, px_per_mm, rx, ry, tx, ty, tz=600.0):
    """평면 인쇄물(행 0 = 위쪽)을 카메라 K 로 찍은 합성 사진. 렌즈 왜곡 없음."""
    rotation, _ = cv2.Rodrigues(np.array([rx, ry, 0.05]))
    to_mm = np.array([[1 / px_per_mm, 0, 0.5 / px_per_mm], [0, 1 / px_per_mm, 0.5 / px_per_mm], [0, 0, 1.]])
    homography = K @ np.column_stack([rotation[:, 0], rotation[:, 1], [tx, ty, tz]]) @ to_mm
    return cv2.warpPerspective(plane_image, homography, SIZE, flags=cv2.INTER_AREA, borderValue=255), homography


def sheet_mm_to_image_px(mm, px_per_mm):
    """시트 mm(+Y 위쪽) → render_sheet 이미지의 픽셀 중심 좌표."""
    mm = np.asarray(mm, dtype=float)
    return np.column_stack([mm[:, 0] * px_per_mm - 0.5, (ac.SHEET_MM[1] - mm[:, 1]) * px_per_mm - 0.5])


def test_sheet_markers_inside_sheet_and_distinct_from_board():
    board_ids = set(range(ac.CHARUCO_SQUARES[0] * ac.CHARUCO_SQUARES[1] // 2))
    assert not board_ids & set(ac.SHEET_MARKERS)
    for marker_id in ac.SHEET_MARKERS:
        corners = ac.sheet_marker_corners_mm(marker_id)
        assert corners.min() >= ac.SHEET_MARGIN_MM - 1e-9
        assert (corners.max(axis=0) <= np.array(ac.SHEET_MM) - ac.SHEET_MARGIN_MM + 1e-9).all()


def test_sheet_homography_recovers_mm_from_tilted_view():
    ppm = 3
    photo, homography = view_of_plane(ac.render_sheet(ppm), ppm, 0.2, -0.15, -150, -100)
    H, rms, ids = ac.sheet_homography(photo)
    assert ids == [40, 41, 42, 43]
    assert rms < 0.5  # 합성 사진은 1 mm ≈ 1.5 px 라 거칠다. 목표 위치 오차는 2 mm
    truth = np.array([[148.5, 105.0], [60.0, 40.0], [250.0, 170.0], [20.0, 190.0]])
    pixels = cv2.perspectiveTransform(sheet_mm_to_image_px(truth, ppm).reshape(-1, 1, 2), homography)
    assert cv2.perspectiveTransform(pixels, H).reshape(-1, 2) == pytest.approx(truth, abs=0.3)


def test_sheet_homography_needs_markers():
    with pytest.raises(ValueError, match='마커'):
        ac.sheet_homography(np.full((480, 640), 255, np.uint8))


def test_sheet_grid_is_brighter_than_detection_threshold():
    sheet = ac.render_sheet(4)
    marker_free = sheet[200:600, 300:900]
    assert marker_free.min() > 100  # vision_detector 의 고정 임계값 100 보다 밝아 배경으로 처리


def test_charuco_render_detects_all_inner_corners():
    _, ids = ac.detect_charuco(ac.render_charuco(4))
    assert len(ids) == (ac.CHARUCO_SQUARES[0] - 1) * (ac.CHARUCO_SQUARES[1] - 1)


def test_charuco_calibration_recovers_focal_length():
    board = ac.render_charuco(2)
    corners, ids = zip(*(ac.detect_charuco(view_of_plane(board, 2, *pose)[0]) for pose in POSES))
    rms, matrix, _ = ac.calibrate_charuco(list(corners), list(ids), SIZE)
    assert rms < 1.0
    assert matrix[0, 0] == pytest.approx(900, rel=0.03)
    assert matrix[1, 1] == pytest.approx(900, rel=0.03)


def test_scripts_round_trip(tmp_path):
    """ChArUco 사진 → calibrate_camera.py → 작업 시트 사진 → compute_homography.py --aruco → Calibrator."""
    board_dir = tmp_path / 'charuco'
    board_dir.mkdir()
    board = ac.render_charuco(2)
    for index, pose in enumerate(POSES):
        cv2.imwrite(str(board_dir / f'{index:02d}.png'), view_of_plane(board, 2, *pose)[0])
    intrinsics, homography_out = tmp_path / 'intrinsics.npz', tmp_path / 'homography.npz'
    subprocess.run([sys.executable, str(SCRIPTS / 'calibrate_camera.py'), '--images', str(board_dir),
                    '--out', str(intrinsics)], check=True, capture_output=True)

    ppm = 3
    photo, homography = view_of_plane(ac.render_sheet(ppm), ppm, 0.1, 0.1, -150, -100)
    sheet_path = tmp_path / 'sheet.png'
    cv2.imwrite(str(sheet_path), photo)
    subprocess.run([sys.executable, str(SCRIPTS / 'compute_homography.py'), '--aruco', '--image', str(sheet_path),
                    '--intrinsics', str(intrinsics), '--out', str(homography_out)], check=True, capture_output=True)

    calibrator = Calibrator(str(intrinsics), str(homography_out))
    assert calibrator.status.ready
    truth = np.array([[148.5, 105.0], [80.0, 60.0], [220.0, 150.0]])
    pixels = cv2.perspectiveTransform(sheet_mm_to_image_px(truth, ppm).reshape(-1, 1, 2), homography)
    assert calibrator.px_to_mm(pixels.reshape(-1, 2)) == pytest.approx(truth, abs=1.0)


def test_dist_model_none_keeps_zero_distortion():
    """--dist-model none: 왜곡 계수는 0 으로 고정되고 초점거리만 구한다 (폰이 이미 왜곡을 펴서 보낼 때)."""
    board = ac.render_charuco(2)
    corners, ids = zip(*(ac.detect_charuco(view_of_plane(board, 2, *pose)[0]) for pose in POSES))
    rms, matrix, dist = ac.calibrate_charuco(list(corners), list(ids), SIZE, flags=ac.DIST_MODEL_FLAGS['none'])
    assert rms < 1.0
    assert np.allclose(dist, 0)
    assert matrix[0, 0] == pytest.approx(900, rel=0.03)


def test_calibrate_script_rotate_matches_stream_orientation(tmp_path):
    """세로로 저장된 보드 사진을 --rotate 로 돌리면 결과 image_size 가 가로 스트림 크기가 된다."""
    board_dir = tmp_path / 'charuco'
    board_dir.mkdir()
    board = ac.render_charuco(2)
    for index, pose in enumerate(POSES):
        landscape = view_of_plane(board, 2, *pose)[0]
        cv2.imwrite(str(board_dir / f'{index:02d}.png'), cv2.rotate(landscape, cv2.ROTATE_90_CLOCKWISE))
    out = tmp_path / 'intrinsics.npz'
    subprocess.run([sys.executable, str(SCRIPTS / 'calibrate_camera.py'), '--images', str(board_dir),
                    '--rotate', 'ccw', '--dist-model', 'none', '--out', str(out)], check=True, capture_output=True)
    data = np.load(out)
    assert tuple(data['image_size']) == SIZE
    assert np.allclose(data['dist_coeffs'], 0)
    assert data['camera_matrix'][0, 2] == pytest.approx(640, abs=15)
