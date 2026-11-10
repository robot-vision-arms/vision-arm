"""카메라 왜곡 보정 + 호모그래피 기반 px → mm 변환 (1주차).

파일 두 개를 읽는다 (만드는 쪽은 scripts/ 의 오프라인 스크립트):
    data/calib/intrinsics.npz  : camera_matrix, dist_coeffs, rms      ← scripts/calibrate_camera.py
    data/calib/homography.npz  : H (3x3, 왜곡 보정된 px → mm), reproj_err_mm ← scripts/compute_homography.py
"""
from dataclasses import dataclass
from pathlib import Path

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
        self.status = CalibStatus()
        self.load(Path(intrinsics_path), Path(homography_path))

    def load(self, intrinsics_path: Path, homography_path: Path) -> None:
        """파일이 없으면 예외 대신 status 에 False 로 남긴다 (GUI 가 '미보정' 으로 표시)."""
        # TODO: np.load 로 읽고 self.status 갱신
        raise NotImplementedError

    def undistort_points(self, pts_px: np.ndarray) -> np.ndarray:
        """(N, 2) 원본 픽셀 → 왜곡 보정된 픽셀. cv2.undistortPoints(..., P=camera_matrix) 사용."""
        raise NotImplementedError

    def px_to_mm(self, pts_px: np.ndarray) -> np.ndarray:
        """(N, 2) 원본 픽셀 → (N, 2) 로봇 작업 평면 mm. 왜곡 보정 후 cv2.perspectiveTransform(H)."""
        raise NotImplementedError
