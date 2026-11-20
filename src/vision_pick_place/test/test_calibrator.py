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


def _tilted_camera(tmp_path):
    """작업 평면 mm 좌표에서 (150, 100, 600) mm 위에 살짝 기울어진 카메라. (Calibrator, 투영 함수, 카메라 중심)."""
    K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]])
    rvec = np.array([np.pi + 0.08, 0.05, 0.02])          # 아래를 보되 조금 기울어짐
    R, _ = cv2.Rodrigues(rvec)
    center = np.array([150.0, 100.0, 600.0])
    tvec = -R @ center

    def project(xyz):
        px, _ = cv2.projectPoints(np.asarray(xyz, np.float64).reshape(-1, 1, 3), rvec, tvec, K, np.zeros(5))
        return px.reshape(-1, 2)

    plane = np.array([[x, y, 0.0] for x in (20, 150, 280) for y in (20, 100, 190)])
    H, _ = cv2.findHomography(project(plane), plane[:, :2])
    np.savez(tmp_path / 'i.npz', camera_matrix=K, dist_coeffs=np.zeros(5), rms=0.0)
    np.savez(tmp_path / 'h.npz', H=H, reproj_err_mm=0.0)
    return Calibrator(str(tmp_path / 'i.npz'), str(tmp_path / 'h.npz')), project, center


def test_camera_center_from_homography(tmp_path):
    calib, _, center = _tilted_camera(tmp_path)
    assert calib.camera_center_mm() == pytest.approx(center, abs=0.5)


def test_parallax_correction_recovers_raised_point(tmp_path):
    """평면보다 8 mm 위의 점: 보정 없이는 바깥으로 밀리고, height_mm=8 이면 제자리."""
    calib, project, _ = _tilted_camera(tmp_path)
    truth = np.array([[40.0, 30.0], [260.0, 170.0]])
    px = project(np.column_stack((truth, np.full(2, 8.0))))
    raw = calib.px_to_mm(px)
    fixed = calib.px_to_mm(px, 8.0)
    assert np.linalg.norm(raw - truth, axis=1).min() > 1.0       # 보정 안 하면 1 mm 넘게 어긋남
    assert fixed == pytest.approx(truth, abs=0.01)
    assert calib.px_to_mm(px, [8.0, 0.0])[0] == pytest.approx(truth[0], abs=0.01)   # 점마다 높이
    with pytest.raises(ValueError):
        calib.px_to_mm(px, -1.0)
