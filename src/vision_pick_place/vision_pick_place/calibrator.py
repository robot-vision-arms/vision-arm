"""카메라 왜곡 보정 + 호모그래피 기반 px → mm 변환 (1주차).

파일 두 개를 읽는다 (만드는 쪽은 scripts/ 의 오프라인 스크립트):
    data/calib/intrinsics.npz  : camera_matrix, dist_coeffs, rms      ← scripts/calibrate_camera.py
    data/calib/homography.npz  : H (3x3, 왜곡 보정된 px → mm), reproj_err_mm ← scripts/compute_homography.py

px → mm 변환은 두 파일이 모두 있어야 한다 (항상 왜곡 보정 후 H 적용).
파일이 없으면 미보정(status False), 파일이 깨졌거나 값이 이상하면 ValueError.
"""
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


@dataclass
class CalibStatus:
    """GUI 의 '캘리브레이션 상태' 표시에 그대로 쓴다."""
    intrinsics_loaded: bool = False
    homography_loaded: bool = False
    rms_px: float | None = None            # 카메라 캘리브레이션 재투영 오차
    reproj_err_mm: float | None = None     # 호모그래피 기준점 재투영 오차

    @property
    def ready(self) -> bool:
        return self.intrinsics_loaded and self.homography_loaded


class Calibrator:
    def __init__(self, intrinsics_path: str, homography_path: str):
        self.camera_matrix = None
        self.dist_coeffs = None
        self.H = None
        self.image_size = None
        self.status = CalibStatus()
        self.load(Path(intrinsics_path), Path(homography_path))

    def load(self, intrinsics_path: Path, homography_path: Path) -> None:
        """파일이 없으면 예외 대신 status 에 False 로 남긴다 (GUI 가 '미보정' 으로 표시)."""
        self.camera_matrix = self.dist_coeffs = self.H = None
        self.image_size = None
        self.status = CalibStatus()
        # 파일이 없으면 미보정, 파일이 있는데 깨졌거나 값이 이상하면 예외. 잘못된 보정값으로 mm 를 내보내는 것보다 멈추는 편이 안전하다.
        if intrinsics_path.is_file():
            with np.load(intrinsics_path, allow_pickle=False) as data:
                matrix = np.asarray(data['camera_matrix'], dtype=np.float64)
                distortion = np.asarray(data['dist_coeffs'], dtype=np.float64)
                if (matrix.shape != (3, 3) or not np.isfinite(matrix).all()
                        or matrix[0, 0] <= 0 or matrix[1, 1] <= 0
                        or distortion.size not in (4, 5, 8, 12, 14)
                        or not np.isfinite(distortion).all()):
                    raise ValueError('잘못된 카메라 내부 파라미터입니다.')
                self.camera_matrix, self.dist_coeffs = matrix, distortion.reshape(-1)
                self.status.rms_px = float(data['rms'])
                if not np.isfinite(self.status.rms_px) or self.status.rms_px < 0:
                    raise ValueError('카메라 RMS는 0 이상의 유한한 값이어야 합니다.')
                if 'image_size' in data:
                    self.image_size = tuple(int(v) for v in data['image_size'])
                    if len(self.image_size) != 2 or min(self.image_size) <= 0:
                        raise ValueError('잘못된 보정 이미지 해상도입니다.')
                self.status.intrinsics_loaded = True
        if homography_path.is_file():
            with np.load(homography_path, allow_pickle=False) as data:
                H = np.asarray(data['H'], dtype=np.float64)
                if H.shape != (3, 3) or not np.isfinite(H).all() or np.linalg.matrix_rank(H) < 3:
                    raise ValueError('잘못된 호모그래피입니다.')
                # --no-undistort 로 만든 H 는 원본 픽셀 기준이라, 항상 왜곡 보정 후 H 를 쓰는 이 클래스와 맞지 않는다.
                if 'undistorted' in data and not bool(data['undistorted']):
                    raise ValueError('왜곡 보정 없이(--no-undistort) 만든 호모그래피는 쓸 수 없습니다.')
                self.H = H
                self.status.reproj_err_mm = float(data['reproj_err_mm'])
                if not np.isfinite(self.status.reproj_err_mm) or self.status.reproj_err_mm < 0:
                    raise ValueError('호모그래피 RMS는 0 이상의 유한한 값이어야 합니다.')
                self.status.homography_loaded = True

    def undistort_points(self, pts_px: np.ndarray) -> np.ndarray:
        """(N, 2) 원본 픽셀 → 왜곡 보정된 픽셀. cv2.undistortPoints(..., P=camera_matrix) 사용."""
        if not self.status.intrinsics_loaded:
            raise RuntimeError('카메라 내부 파라미터가 없습니다. scripts/calibrate_camera.py 로 intrinsics.npz 를 먼저 만드세요.')
        points = _as_points(pts_px)
        if not np.isfinite(points).all():
            raise ValueError('픽셀 좌표는 유한한 값이어야 합니다.')
        if not len(points):
            return points
        # P 를 안 넘기면 정규화 좌표(초점거리 1)가 나온다. P=camera_matrix 를 넘겨야 다시 픽셀 단위가 된다.
        return cv2.undistortPoints(points.reshape(-1, 1, 2), self.camera_matrix,
                                   self.dist_coeffs, P=self.camera_matrix).reshape(-1, 2)

    def px_to_mm(self, pts_px: np.ndarray) -> np.ndarray:
        """(N, 2) 원본 픽셀 → (N, 2) 로봇 작업 평면 mm. 왜곡 보정 후 cv2.perspectiveTransform(H)."""
        if not self.status.ready:
            raise RuntimeError('내부 파라미터와 호모그래피가 모두 필요합니다.')
        points = self.undistort_points(pts_px)
        if not len(points):
            return points
        homogeneous = np.column_stack((points, np.ones(len(points)))) @ self.H.T
        if np.any(np.abs(homogeneous[:, 2]) < 1e-12):
            raise ValueError('호모그래피로 변환할 수 없는 점입니다.')
        result = homogeneous[:, :2] / homogeneous[:, 2:3]
        if not np.isfinite(result).all():
            raise ValueError('변환 결과가 유한하지 않습니다.')
        return result


def _as_points(pts_px) -> np.ndarray:
    """리스트·튜플·(2,) 도 받아서 (N, 2) float64 로 맞춘다. OpenCV 는 정수 배열이나 (N, 2) 모양을 거부한다."""
    pts = np.asarray(pts_px, dtype=np.float64)
    if pts.size == 0:
        return np.empty((0, 2), dtype=np.float64)
    return pts.reshape(-1, 2)
