import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np
import pytest

from vision_pick_place.calibrator import Calibrator
from vision_pick_place.packet import encode
from vision_pick_place.vision_pipeline import process_frame


def calibration(tmp_path, H=None):
    intrinsics = tmp_path / 'intrinsics.npz'
    homography = tmp_path / 'homography.npz'
    np.savez(intrinsics, camera_matrix=np.array([[500., 0, 160], [0, 500., 120], [0, 0, 1]]),
             dist_coeffs=np.zeros(5), rms=0.1)
    np.savez(homography, H=H if H is not None else np.array([[2., 0, 0], [0, -3., 720], [0, 0, 1]]),
             reproj_err_mm=0.2)
    return Calibrator(str(intrinsics), str(homography))


def frame():
    image = np.full((240, 320, 3), 255, dtype=np.uint8)
    box = cv2.boxPoints(((160, 120), (100, 20), 30))
    cv2.fillConvexPoly(image, np.rint(box).astype(np.int32), (0, 0, 0))
    return image


def test_calibrated_packet_transforms_axis_in_mm_not_pixels(tmp_path):
    calibrator = calibration(tmp_path)
    image = frame()
    packet, debug = process_frame(image, calibrator, 7, 123.5, blur_ksize=1)
    obj = packet['objects'][0]
    assert packet['seq'] == 7 and packet['t_capture'] == 123.5 and packet['calib_ok'] is True
    assert (obj['x_mm'], obj['y_mm']) == pytest.approx((320, 360), abs=2)
    expected = np.degrees(np.arctan2(-3 * np.sin(np.pi / 6), 2 * np.cos(np.pi / 6))) % 180
    assert obj['angle_deg'] == pytest.approx(expected, abs=2)
    assert debug.shape == image.shape
    assert not np.array_equal(debug, image)
    assert json.loads(encode(packet)) == packet


def test_missing_calibration_publishes_pixels_without_fake_mm(tmp_path):
    calibrator = Calibrator(str(tmp_path / 'absent1'), str(tmp_path / 'absent2'))
    packet, _ = process_frame(frame(), calibrator, 0, 0)
    assert packet['calib_ok'] is False
    obj = packet['objects'][0]
    assert obj['x_mm'] is obj['y_mm'] is obj['angle_deg'] is None
    assert obj['u_px'] == pytest.approx(160, abs=1)
    with pytest.raises(RuntimeError):
        calibrator.px_to_mm([[1, 2]])


def test_nut_has_no_axis_angle_even_when_calibrated(tmp_path):
    image = np.full((240, 320, 3), 255, dtype=np.uint8)
    cv2.circle(image, (160, 120), 20, (0, 0, 0), -1)
    packet, _ = process_frame(image, calibration(tmp_path), 0, 0)
    assert packet['objects'][0]['part_type'] == 'nut'
    assert packet['objects'][0]['angle_deg'] is None


def test_singular_homography_and_point_at_infinity_are_rejected(tmp_path):
    with pytest.raises(ValueError):
        calibration(tmp_path, np.zeros((3, 3)))
    calibrator = calibration(tmp_path, np.array([[1., 0, 0], [0, 1, 0], [1, 0, -1]]))
    with pytest.raises(ValueError):
        calibrator.px_to_mm([[1, 2]])


def test_reload_clears_previous_calibration(tmp_path):
    calibrator = calibration(tmp_path)
    calibrator.load(tmp_path / 'absent1', tmp_path / 'absent2')
    assert not calibrator.status.ready
    assert calibrator.H is None and calibrator.camera_matrix is None


def test_changed_camera_resolution_is_rejected(tmp_path):
    calibrator = calibration(tmp_path)
    calibrator.image_size = (640, 480)
    with pytest.raises(ValueError, match='해상도'):
        process_frame(frame(), calibrator, 0, 0)


def test_nonzero_distortion_is_removed_before_mm_transform(tmp_path):
    calibrator = calibration(tmp_path)
    calibrator.dist_coeffs = np.array([0.2, -0.03, 0.001, -0.002, 0.])
    world = np.array([[0.1, 0.2, 1.], [-0.2, 0.1, 1.]])
    distorted, _ = cv2.projectPoints(world, np.zeros(3), np.zeros(3),
                                     calibrator.camera_matrix, calibrator.dist_coeffs)
    expected = world[:, :2] * 500 + [160, 120]
    assert calibrator.undistort_points(distorted) == pytest.approx(expected, abs=1e-4)


def test_homography_cli_round_trip(tmp_path):
    calibration(tmp_path)
    image_path = tmp_path / 'grid.png'
    cv2.imwrite(str(image_path), frame())
    points = np.array([[10, 10], [150, 10], [300, 10], [10, 100],
                       [300, 100], [10, 220], [150, 220], [300, 220]], dtype=float)
    truth = points * [2, -3] + [0, 720]
    np.savetxt(tmp_path / 'points.csv', np.column_stack((points, truth)), delimiter=',',
               header='u_px,v_px,x_mm,y_mm', comments='')
    script = Path(__file__).resolve().parents[3] / 'scripts/compute_homography.py'
    subprocess.run([sys.executable, str(script), '--image', str(image_path),
                    '--points', str(tmp_path / 'points.csv'), '--intrinsics', str(tmp_path / 'intrinsics.npz'),
                    '--out', str(tmp_path / 'computed.npz')], check=True, capture_output=True)
    calibrator = Calibrator(str(tmp_path / 'intrinsics.npz'), str(tmp_path / 'computed.npz'))
    assert calibrator.status.ready
    assert calibrator.px_to_mm(points) == pytest.approx(truth, abs=1e-5)
