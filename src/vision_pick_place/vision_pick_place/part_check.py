"""나사·너트가 아닌 물체를 unknown 으로 거르기 (4주차, 팀원 1).

규칙 분류(classify_part)는 길이 비만 봐서 시트 위의 것은 무엇이든 나사 아니면 너트가 된다 (11-20 튜브 → 나사, 스위치 → 너트).
여기서 실제 크기(mm)와 가운데 구멍을 더 보고, 범위를 벗어나면 unknown 으로 바꾼다. 스폰·집기·GUI 반영은 unknown 을 쓰지 않는다.

기준값은 우리 부품을 찍어 scripts/tune_part_limits.py 로 정한다. 기본은 꺼짐 (params.yaml reject_unknown: false).
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

# params 이름 → (최소, 최대) 범위 항목. [0, 0] 이면 그 항목은 보지 않는다
RANGE_PARAMS = ('screw_length_mm', 'screw_width_mm', 'nut_length_mm', 'nut_width_mm')


@dataclass
class Features:
    length_mm: float | None     # 보정이 없으면 None
    width_mm: float | None
    hole_ratio: float           # 가장 큰 구멍 넓이 ÷ 바깥 윤곽 넓이 (구멍이 없으면 0)


def measure(mask: np.ndarray, det, to_mm=None) -> Features:
    """det.contour 안의 구멍과 상자 크기. to_mm: (N, 2) px → (N, 2) mm (보정이 없으면 None)."""
    x, y, w, h = cv2.boundingRect(det.contour)
    filled = np.zeros((h, w), np.uint8)
    cv2.drawContours(filled, [det.contour], -1, 255, -1, offset=(-x, -y))
    holes = cv2.bitwise_and(filled, cv2.bitwise_not(mask[y:y + h, x:x + w]))
    contours, _ = cv2.findContours(holes, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    hole = max((cv2.contourArea(c) for c in contours), default=0.0)
    length = width = None
    if to_mm is not None:
        mm = np.asarray(to_mm(np.asarray(det.box_px, float)), float).reshape(4, 2)
        edges = np.linalg.norm(np.roll(mm, -1, axis=0) - mm, axis=1)
        sides = (edges[0] + edges[2]) / 2, (edges[1] + edges[3]) / 2
        length, width = float(max(sides)), float(min(sides))
    return Features(length, width, float(hole / det.area_px))


def limits_from_params(params: dict) -> dict | None:
    """설정 → 기준값. reject_unknown 이 꺼져 있으면 None (거르지 않음)."""
    if not params.get('reject_unknown', False):
        return None
    limits = {}
    for name in RANGE_PARAMS:
        low, high = (float(v) for v in params.get(name, (0.0, 0.0)))
        if not (np.isfinite(low) and np.isfinite(high)) or low < 0 or high < low:
            raise ValueError(f'{name} 은 [최소, 최대] (0 이상, 최소 ≤ 최대) 여야 합니다.')
        if high > 0:
            limits[name] = (low, high)
    hole = float(params.get('nut_min_hole_ratio', 0.0))
    if not 0 <= hole < 1:
        raise ValueError('nut_min_hole_ratio 는 0 이상 1 미만이어야 합니다.')
    limits['nut_min_hole_ratio'] = hole
    return limits


def reject_reason(part_type: str, f: Features, limits: dict) -> str | None:
    """part_type 으로 보기 어려운 이유 (영어, 영상에 그대로 쓴다). 문제없으면 None. 보정이 없으면 크기는 보지 않는다."""
    for side in ('length', 'width'):
        value, limit = getattr(f, f'{side}_mm'), limits.get(f'{part_type}_{side}_mm')
        if value is not None and limit is not None and not limit[0] <= value <= limit[1]:
            return f'{side} {value:.1f}mm'
    if part_type == 'nut' and f.hole_ratio < limits['nut_min_hole_ratio']:
        return f'hole {f.hole_ratio:.2f}'
    return None
