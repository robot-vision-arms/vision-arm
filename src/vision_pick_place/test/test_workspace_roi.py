"""작업 시트 ROI: 귀퉁이 마커·시트 밖 물체는 검출에서 빠지고, 시트 위 부품은 그대로 남는지."""
import cv2
import numpy as np
import pytest

from test_aruco_calib import K, SIZE, sheet_mm_to_image_px, view_of_plane
from vision_pick_place import aruco_calib as ac
from vision_pick_place.calibrator import Calibrator
from vision_pick_place.vision_detector import angle_error_deg
from vision_pick_place.vision_pipeline import process_frame
from vision_pick_place.workspace_roi import roi_polygons_mm, sheet_roi_mask

PPM = 4  # 작업 시트를 그릴 때 1 mm = 4 px
POSE = (0.1, 0.1, -150, -100)  # 카메라에서 본 시트 자세 (test_aruco_calib 와 같은 형식)


def part_corners_mm(center, length, width, angle_deg):
    """시트 mm 좌표의 회전 사각형 꼭짓점 (4, 2). angle 은 +X 에서 반시계."""
    a = np.radians(angle_deg)
    axis, normal = np.array([np.cos(a), np.sin(a)]), np.array([-np.sin(a), np.cos(a)])
    return np.array([center + sx * length / 2 * axis + sy * width / 2 * normal
                     for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1))])


def photo_of_sheet(parts=()):
    """시트(+ 그 위에 그린 검은 부품)를 카메라로 찍은 BGR 사진."""
    sheet = ac.render_sheet(PPM)
    for corners in parts:
        cv2.fillPoly(sheet, [np.rint(sheet_mm_to_image_px(corners, PPM)).astype(np.int32)], 0)
    gray, _ = view_of_plane(sheet, PPM, *POSE)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def calibrated(tmp_path, photo):
    """합성 카메라(왜곡 없음) + 시트 마커로 구한 호모그래피. 실제 흐름(compute_homography --aruco)과 같은 계산."""
    distortion = np.zeros(5)
    H, _, _ = ac.sheet_homography(cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY), K, distortion)
    intrinsics, homography = tmp_path / 'i.npz', tmp_path / 'h.npz'
    np.savez(intrinsics, camera_matrix=K, dist_coeffs=distortion, rms=0.1, image_size=np.array(SIZE))
    np.savez(homography, H=H, reproj_err_mm=0.1)
    return Calibrator(str(intrinsics), str(homography))


def test_mm_to_px_inverts_px_to_mm_with_lens_distortion(tmp_path):
    distortion = np.array([-0.12, 0.03, 0.001, -0.001, 0.0])
    H = np.array([[0.5, 0.02, -300.0], [-0.01, -0.5, 200.0], [1e-5, 2e-5, 1.0]])
    np.savez(tmp_path / 'i.npz', camera_matrix=K, dist_coeffs=distortion, rms=0.1)
    np.savez(tmp_path / 'h.npz', H=H, reproj_err_mm=0.1)
    cal = Calibrator(str(tmp_path / 'i.npz'), str(tmp_path / 'h.npz'))
    mm = np.array([[0.0, 0.0], [-120.0, 80.0], [40.0, -60.0], [-200.0, 150.0]])
    px = cal.mm_to_px(mm)
    assert np.abs(px - cv2.perspectiveTransform(mm.reshape(-1, 1, 2), np.linalg.inv(H)).reshape(-1, 2)).max() > 1
    np.testing.assert_allclose(cal.px_to_mm(px), mm, atol=1e-3)
    assert cal.mm_to_px(np.empty((0, 2))).shape == (0, 2)


def test_bare_sheet_markers_are_detected_without_roi_and_removed_with_roi(tmp_path):
    photo = photo_of_sheet()
    cal = calibrated(tmp_path, photo)
    without, _ = process_frame(photo, cal, 0, 0, min_area_px=200)
    assert len(without['objects']) >= 4  # 테스트가 의미 있으려면 ROI 없이는 마커가 부품으로 잡혀야 한다
    roi = sheet_roi_mask(cal, SIZE)
    packet, debug = process_frame(photo, cal, 0, 0, min_area_px=200, roi_mask=roi)
    assert packet['objects'] == []
    assert not np.array_equal(debug, photo)  # ROI 경계선이 디버그 영상에 그려짐


def test_parts_on_sheet_are_kept_at_true_mm_and_objects_off_sheet_are_removed(tmp_path):
    screw = part_corners_mm(np.array([150.0, 105.0]), 50, 8, 30)
    nut = part_corners_mm(np.array([80.0, 60.0]), 14, 14, 0)
    cal = calibrated(tmp_path, photo_of_sheet())
    photo = photo_of_sheet([screw, nut])
    # 시트 밖 책상 위의 검은 물체 (영상 왼쪽 위 구석, 시트 바깥)
    cv2.rectangle(photo, (20, 20), (70, 50), (0, 0, 0), -1)
    roi = sheet_roi_mask(cal, SIZE)
    assert roi[35, 45] == 0
    packet, _ = process_frame(photo, cal, 0, 0, min_area_px=200, roi_mask=roi)
    objects = sorted(packet['objects'], key=lambda o: o['part_type'])
    assert [o['part_type'] for o in objects] == ['nut', 'screw']
    nut_out, screw_out = objects
    assert np.hypot(nut_out['x_mm'] - 80, nut_out['y_mm'] - 60) < 1.0
    assert np.hypot(screw_out['x_mm'] - 150, screw_out['y_mm'] - 105) < 1.0
    assert angle_error_deg(screw_out['angle_deg'], 30) < 1.5


def test_no_roi_without_calibration(tmp_path):
    cal = Calibrator(str(tmp_path / 'none1'), str(tmp_path / 'none2'))
    assert sheet_roi_mask(cal, SIZE) is None


def test_roi_size_must_match_frame(tmp_path):
    photo = photo_of_sheet()
    cal = calibrated(tmp_path, photo)
    with pytest.raises(ValueError):
        process_frame(photo, cal, 0, 0, roi_mask=np.zeros((10, 10), np.uint8))


def test_margins_are_validated():
    sheet, markers = roi_polygons_mm(3, 5)
    assert len(markers) == len(ac.SHEET_MARKERS)
    assert sheet[:, 0].min() == pytest.approx(3) and sheet[:, 1].max() == pytest.approx(ac.SHEET_MM[1] - 3)
    with pytest.raises(ValueError):
        roi_polygons_mm(-1, 5)
    with pytest.raises(ValueError):
        roi_polygons_mm(200, 5)
