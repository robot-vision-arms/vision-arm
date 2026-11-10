"""외곽선 → 중심·장축 각도 계산, 나사/너트 분류 (2주차).

각도 정의 (docs/PROJECT.md 3.2 와 동일):
    로봇 좌표계(mm) 기준, +X 축에서 장축까지 반시계 방향 각도. 범위 [0, 180).
    장축은 방향이 없는 선이라 θ 와 θ+180 은 같은 자세로 본다.
cv2.minAreaRect 의 angle 은 OpenCV 버전마다 범위 규칙이 다르고 픽셀 좌표(y 축 아래 방향) 기준이므로
그대로 쓰지 않고, 사각형 꼭짓점에서 장축 벡터를 직접 구한 뒤 mm 좌표로 변환해 각도를 계산한다.
"""
from dataclasses import dataclass

import numpy as np


@dataclass
class Detection:
    center_px: tuple[float, float]       # (u, v) 픽셀
    box_px: np.ndarray                   # minAreaRect 꼭짓점 4개, shape (4, 2)
    long_axis_px: tuple[float, float]    # 장축 방향 단위벡터 (픽셀 좌표계)
    length_px: float                     # 장축 길이
    width_px: float                      # 단축 길이
    area_px: float


def find_objects(mask: np.ndarray, min_area_px: float = 500) -> list[Detection]:
    """이진 마스크에서 외곽선을 찾고, 물체마다 minAreaRect 로 중심과 장축을 구한다.

    중심은 minAreaRect 중심 대신 모멘트(cv2.moments) 무게중심을 쓰는 것도 비교해 볼 것.
    최종 목표(여러 공구)에서는 무게중심이 물체 밖에 떨어질 수 있다 (펜치, 한쪽이 트인 스패너).
    그때는 마스크 거리 변환(cv2.distanceTransform) 최댓값 = 물체 안쪽에서 가장 두꺼운 점을 집는 점으로 쓴다.
    """
    # TODO: cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # TODO: 면적 필터 → cv2.minAreaRect → cv2.boxPoints → 긴 변 벡터로 long_axis_px 계산
    raise NotImplementedError


def normalize_angle_deg(angle: float) -> float:
    """임의 각도를 장축 정의 범위 [0, 180) 로 접는다."""
    return angle % 180.0


def angle_error_deg(pred: float, true: float) -> float:
    """[0, 180) 정의에서 두 장축 각도의 차이. 179° 와 1° 의 오차는 2° 가 되어야 한다."""
    d = abs(normalize_angle_deg(pred) - normalize_angle_deg(true))
    return min(d, 180.0 - d)


def classify_part(det: Detection, nut_max_aspect: float = 1.5) -> str:
    """장축/단축 길이 비로 나사·너트를 나눈다. 너트는 거의 정다각형이라 비율이 1 에 가깝고, 나사는 길쭉하다.

    nut_max_aspect 는 초기값이다. MVTec·자체 촬영본으로 두 부품의 비율 분포를 보고 조정한다 (params.yaml).
    두 종류일 때만 쓰는 규칙이다. 공구 종류를 늘리면 형상 특징 분류기로 바꾼다 (docs/PROJECT.md 3.5).
    """
    # TODO: width_px 가 0 이면 'unknown', 아니면 length_px / width_px 를 nut_max_aspect 와 비교
    raise NotImplementedError
