"""작업 시트 검출 영역(ROI) 마스크.

작업 시트 네 귀퉁이의 ArUco 마커(검은 사각형)는 이진화하면 부품처럼 보인다. 시트 밖 책상(나무 무늬·그림자)도 오검출 원인이다.
그래서 검출은 "시트 안쪽 - 마커 영역" 에서만 한다.

    ROI = 시트 사각형을 edge_margin_mm 만큼 안으로 줄인 영역
          - 각 마커 정사각형을 marker_margin_mm 만큼 키운 영역

영역은 시트 mm 좌표(aruco_calib 정의)로 정하고, Calibrator.mm_to_px 로 원본 영상 픽셀에 옮긴다.
렌즈 왜곡 때문에 직선이 영상에서 휘므로, 변을 잘게 나눈 다각형으로 옮긴다.
보정(내부 파라미터 + 호모그래피)이 없으면 시트 위치를 알 수 없으므로 ROI 를 만들지 않는다 (None).
"""
import cv2
import numpy as np

from vision_pick_place import aruco_calib

# 기본 여백 (mm). 초기값이며 실제 촬영으로 조정한다.
# 마커: 인쇄된 흰 여백(4 mm) 안쪽까지 덮고, 보정 오차(1~2 mm)가 있어도 마커 테두리 조각이 남지 않게 5 mm.
# 시트 가장자리: 종이 끝 그림자·들뜸을 피하려고 3 mm.
DEFAULT_EDGE_MARGIN_MM = 3.0
DEFAULT_MARKER_MARGIN_MM = 5.0
_STEP_MM = 2.0  # 다각형 변을 나누는 간격. 왜곡이 큰 가장자리에서도 곡선을 따라가게


def _rect_polygon(x0: float, y0: float, x1: float, y1: float, step_mm: float = _STEP_MM) -> np.ndarray:
    """(x0, y0)-(x1, y1) 사각형 둘레를 step_mm 간격 점들로. 반시계 순서, (N, 2) mm."""
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    points = []
    for (ax, ay), (bx, by) in zip(corners, corners[1:] + corners[:1]):
        count = max(int(np.ceil(np.hypot(bx - ax, by - ay) / step_mm)), 1)
        t = np.arange(count) / count
        points.append(np.column_stack((ax + (bx - ax) * t, ay + (by - ay) * t)))
    return np.concatenate(points)


def roi_polygons_mm(edge_margin_mm: float = DEFAULT_EDGE_MARGIN_MM,
                    marker_margin_mm: float = DEFAULT_MARKER_MARGIN_MM):
    """(시트 안쪽 다각형, [마커 제외 다각형 ...]). 모두 시트 mm 좌표."""
    if not (np.isfinite(edge_margin_mm) and np.isfinite(marker_margin_mm)) or edge_margin_mm < 0 or marker_margin_mm < 0:
        raise ValueError('여백은 0 이상의 유한한 값이어야 합니다.')
    width, height = aruco_calib.SHEET_MM
    if 2 * edge_margin_mm >= min(width, height):
        raise ValueError('시트 가장자리 여백이 시트보다 큽니다.')
    sheet = _rect_polygon(edge_margin_mm, edge_margin_mm, width - edge_margin_mm, height - edge_margin_mm)
    side, m = aruco_calib.SHEET_MARKER_MM, marker_margin_mm
    markers = [_rect_polygon(x - m, y - m, x + side + m, y + side + m)
               for x, y in aruco_calib.SHEET_MARKERS.values()]
    return sheet, markers


def sheet_roi_mask(calibrator, image_size: tuple[int, int],
                   edge_margin_mm: float = DEFAULT_EDGE_MARGIN_MM,
                   marker_margin_mm: float = DEFAULT_MARKER_MARGIN_MM):
    """원본 영상 크기 (너비, 높이) 의 uint8 마스크. 255 = 검출할 곳. 보정이 없으면 None."""
    if not calibrator.status.ready:
        return None
    width, height = image_size
    sheet, markers = roi_polygons_mm(edge_margin_mm, marker_margin_mm)
    mask = np.zeros((height, width), np.uint8)
    cv2.fillPoly(mask, [_to_int_px(calibrator.mm_to_px(sheet))], 255)
    for marker in markers:
        cv2.fillPoly(mask, [_to_int_px(calibrator.mm_to_px(marker))], 0)
    return mask


def _to_int_px(points_px: np.ndarray) -> np.ndarray:
    # fillPoly 는 int32 좌표만 받는다. 화면 밖 점도 그대로 두면 OpenCV 가 잘라서 그린다.
    return np.rint(points_px).astype(np.int32).reshape(-1, 1, 2)
