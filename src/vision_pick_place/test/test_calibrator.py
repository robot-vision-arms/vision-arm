import cv2
import numpy as np
import pytest

from vision_pick_place.calibrator import Calibrator

# 합성 카메라: 1280x720, 초점거리 1000px, 폰보다 조금 강한 왜곡
K = np.array([[1000.0, 0.0, 640.0], [0.0, 1000.0, 360.0], [0.0, 0.0, 1.0]])
DIST = np.array([-0.12, 0.03, 0.0, 0.0, 0.0])
# 합성 호모그래피: 약간 비스듬한 카메라, 대략 2 px = 1 mm
H = np.array([[0.5, 0.02, -300.0], [-0.01, -0.5, 200.0], [1e-5, 2e-5, 1.0]])


def _save(tmp_path, dist=DIST, homography=H):
    intr = tmp_path / 'intrinsics.npz'
    hom = tmp_path / 'homography.npz'
    np.savez(intr, camera_matrix=K, dist_coeffs=dist, rms=0.3)
    np.savez(hom, H=homography, reproj_err_mm=0.5)
    return intr, hom


def _distort(pts_px):
    """왜곡 없는 픽셀 → 렌즈를 거친 픽셀. undistort_points 의 정답을 거꾸로 만든다."""
    norm = (np.linalg.inv(K) @ np.c_[pts_px, np.ones(len(pts_px))].T).T
    out, _ = cv2.projectPoints(norm, np.zeros(3), np.zeros(3), K, DIST)
    return out.reshape(-1, 2)


def test_missing_files_leave_status_false(tmp_path):
    cal = Calibrator(str(tmp_path / 'none.npz'), str(tmp_path / 'none2.npz'))
    assert not cal.status.intrinsics_loaded
    assert not cal.status.homography_loaded
    assert not cal.status.ready
    with pytest.raises(RuntimeError):
        cal.px_to_mm([[640.0, 360.0]])


def test_broken_file_raises(tmp_path):
    # 없는 파일은 '미보정', 있는데 깨진 파일은 예외. 잘못된 값으로 mm 를 내보내지 않게 한다
    intr, hom = _save(tmp_path)
    np.savez(hom, wrong_key=np.eye(3))
    with pytest.raises(KeyError):
        Calibrator(str(intr), str(hom))


def test_rejects_homography_made_without_undistortion(tmp_path):
    intr, hom = _save(tmp_path)
    np.savez(hom, H=H, reproj_err_mm=0.5, undistorted=False)
    with pytest.raises(ValueError):
        Calibrator(str(intr), str(hom))


def test_load_fills_status(tmp_path):
    cal = Calibrator(*map(str, _save(tmp_path)))
    assert cal.status.ready
    assert cal.status.rms_px == pytest.approx(0.3)
    assert cal.status.reproj_err_mm == pytest.approx(0.5)


def test_undistort_recovers_ideal_pixels(tmp_path):
    cal = Calibrator(*map(str, _save(tmp_path)))
    # 가장자리(구석)까지 포함. 왜곡은 구석에서 가장 크다
    ideal = np.array([[640.0, 360.0], [100.0, 80.0], [1200.0, 650.0], [900.0, 200.0]])
    raw = _distort(ideal)
    assert np.abs(raw - ideal).max() > 5.0  # 보정 전에는 확실히 어긋나 있어야 테스트 의미가 있다
    np.testing.assert_allclose(cal.undistort_points(raw), ideal, atol=0.05)


def test_px_to_mm_matches_homography(tmp_path):
    cal = Calibrator(*map(str, _save(tmp_path)))
    ideal = np.array([[640.0, 360.0], [100.0, 80.0], [1200.0, 650.0]])
    expected = cv2.perspectiveTransform(ideal.reshape(-1, 1, 2), H).reshape(-1, 2)
    np.testing.assert_allclose(cal.px_to_mm(_distort(ideal)), expected, atol=0.05)


def test_without_intrinsics_refuses_mm(tmp_path):
    _, hom = _save(tmp_path)
    cal = Calibrator(str(tmp_path / 'none.npz'), str(hom))
    assert cal.status.homography_loaded and not cal.status.ready
    with pytest.raises(RuntimeError):
        cal.px_to_mm([[640.0, 360.0]])


def test_input_shapes(tmp_path):
    cal = Calibrator(*map(str, _save(tmp_path)))
    assert cal.px_to_mm([640, 360]).shape == (1, 2)       # 점 1개, 정수 리스트
    assert cal.px_to_mm(np.empty((0, 2))).shape == (0, 2)  # 검출 0개인 프레임
