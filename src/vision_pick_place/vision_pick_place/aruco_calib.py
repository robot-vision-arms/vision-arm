"""ArUco 기반 캘리브레이션 정의와 계산 (1주차). 인쇄물·스크립트·테스트가 같은 값을 쓰도록 한 곳에 둔다.

두 가지 인쇄물:
    ChArUco 보드 (A4)  : 렌즈 왜곡 보정용. 손에 들고 여러 각도로 15~20장 → scripts/calibrate_camera.py
    작업 시트 (A4)      : 네 귀퉁이 ArUco 마커 (ID 40~43). 작업대에 붙여 두고 그 위에 부품을 놓는다
                          → scripts/compute_homography.py --aruco 가 px → mm 호모그래피를 자동으로 구한다

작업 시트 좌표계 (docs/PROJECT.md 3.1):
    원점 = 시트(A4 가로)의 왼쪽 아래 모서리, +X = 오른쪽(297 mm 방향), +Y = 위쪽(210 mm 방향), 단위 mm.
    Gazebo 의 work_mat(A4) 도 같은 원점·크기라서 sim.yaml 의 work_plane 값을 그대로 쓴다.
    마커는 인쇄 위치가 정해져 있으므로 자로 잴 필요가 없다. 100% 배율 인쇄만 확인한다.

OpenCV 4.6 (Ubuntu 24.04 apt) 의 옛 aruco API 와 4.7+ 의 새 API 를 둘 다 지원한다.
"""
import cv2
import numpy as np

ARUCO_DICT = cv2.aruco.DICT_4X4_50

# ChArUco 보드: 7x5 칸, 칸 36 mm, 마커 27 mm → 252 x 180 mm (A4 가로에 여백 포함해서 들어감).
# 행 수를 홀수(5)로 둔 이유: OpenCV 4.8 에서 짝수 행 보드의 무늬 규칙이 바뀌어 버전이 다르면 인식이 어긋난다.
# 보드 마커 ID 는 0~16 을 쓴다. 작업 시트 마커(40~43)와 겹치지 않는다.
CHARUCO_SQUARES = (7, 5)
CHARUCO_SQUARE_MM = 36.0
CHARUCO_MARKER_MM = 27.0

# 작업 시트: A4 가로, 귀퉁이 마커 35 mm, 시트 가장자리에서 12 mm 안쪽 (프린터 인쇄 불가 여백 회피)
SHEET_MM = (297.0, 210.0)
SHEET_MARKER_MM = 35.0
SHEET_MARGIN_MM = 12.0
# ID → 마커 왼쪽 아래 꼭짓점 위치 (mm). 왼쪽 아래부터 반시계: 40 왼쪽아래, 41 오른쪽아래, 42 오른쪽위, 43 왼쪽위
_far_x = SHEET_MM[0] - SHEET_MARGIN_MM - SHEET_MARKER_MM
_far_y = SHEET_MM[1] - SHEET_MARGIN_MM - SHEET_MARKER_MM
SHEET_MARKERS = {
    40: (SHEET_MARGIN_MM, SHEET_MARGIN_MM),
    41: (_far_x, SHEET_MARGIN_MM),
    42: (_far_x, _far_y),
    43: (SHEET_MARGIN_MM, _far_y),
}


def dictionary():
    return cv2.aruco.getPredefinedDictionary(ARUCO_DICT)


def _detector_parameters():
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # 꼭짓점을 소수점 픽셀까지
    return params


def detect_markers(gray: np.ndarray):
    """(corners 목록, ids (N,) 또는 None). corners[i] 는 (4, 2): 마커 기준 왼쪽위·오른쪽위·오른쪽아래·왼쪽아래."""
    if hasattr(cv2.aruco, 'ArucoDetector'):
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary(), _detector_parameters()).detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary(), parameters=_detector_parameters())
    if ids is None:
        return [], None
    return [np.asarray(c, dtype=np.float64).reshape(4, 2) for c in corners], ids.reshape(-1)


# ---------- ChArUco 보드 (렌즈 왜곡 보정) ----------

def charuco_board():
    cols, rows = CHARUCO_SQUARES
    if hasattr(cv2.aruco, 'CharucoBoard_create'):
        return cv2.aruco.CharucoBoard_create(cols, rows, CHARUCO_SQUARE_MM, CHARUCO_MARKER_MM, dictionary())
    return cv2.aruco.CharucoBoard((cols, rows), CHARUCO_SQUARE_MM, CHARUCO_MARKER_MM, dictionary())


def render_charuco(px_per_mm: float) -> np.ndarray:
    """보드 실제 크기 그대로(여백 0) 그린 흑백 이미지. 인쇄 PDF 와 테스트가 쓴다."""
    cols, rows = CHARUCO_SQUARES
    size = (round(cols * CHARUCO_SQUARE_MM * px_per_mm), round(rows * CHARUCO_SQUARE_MM * px_per_mm))
    board = charuco_board()
    if hasattr(board, 'generateImage'):
        return board.generateImage(size, marginSize=0, borderBits=1)
    return board.draw(size, marginSize=0, borderBits=1)


def detect_charuco(gray: np.ndarray, board=None):
    """보드의 체스판 내부 코너 (N,1,2) 와 ID (N,1). 못 찾으면 (None, None)."""
    board = board or charuco_board()
    if hasattr(cv2.aruco, 'CharucoDetector'):
        corners, ids, _, _ = cv2.aruco.CharucoDetector(board).detectBoard(gray)
        return (corners, ids) if ids is not None and len(ids) else (None, None)
    marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(gray, dictionary(), parameters=_detector_parameters())
    if marker_ids is None or not len(marker_ids):
        return None, None
    count, corners, ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, board)
    return (corners, ids) if count and ids is not None else (None, None)


def calibrate_charuco(all_corners: list, all_ids: list, image_size: tuple, board=None):
    """(rms, camera_matrix, dist_coeffs). image_size = (너비, 높이)."""
    rms, matrix, distortion, _ = calibrate_charuco_extended(all_corners, all_ids, image_size, board)
    return rms, matrix, distortion


def calibrate_charuco_extended(all_corners: list, all_ids: list, image_size: tuple, board=None):
    """(rms, camera_matrix, dist_coeffs, 사진별 재투영 오차 (N,) px). 유독 큰 사진은 빼고 다시 돌려 볼 후보."""
    board = board or charuco_board()
    if hasattr(cv2.aruco, 'calibrateCameraCharucoExtended'):
        rms, matrix, distortion, _, _, _, _, per_view = cv2.aruco.calibrateCameraCharucoExtended(
            all_corners, all_ids, board, image_size, None, None)
        return rms, matrix, distortion, np.asarray(per_view).reshape(-1)
    object_points, image_points = [], []
    for corners, ids in zip(all_corners, all_ids):
        obj, img = board.matchImagePoints(corners, ids)
        object_points.append(obj)
        image_points.append(img)
    rms, matrix, distortion, _, _, _, _, per_view = cv2.calibrateCameraExtended(
        object_points, image_points, image_size, None, None)
    return rms, matrix, distortion, np.asarray(per_view).reshape(-1)


# ---------- 작업 시트 (px → mm 호모그래피) ----------

def sheet_marker_corners_mm(marker_id: int) -> np.ndarray:
    """(4, 2) mm. detect_markers 의 꼭짓점 순서(마커 왼쪽위부터 시계 방향)와 같다. 시트 좌표는 +Y 가 위쪽."""
    x, y = SHEET_MARKERS[marker_id]
    s = SHEET_MARKER_MM
    return np.array([[x, y + s], [x + s, y + s], [x + s, y], [x, y]], dtype=np.float64)


def marker_image(marker_id: int, side_px: int) -> np.ndarray:
    if hasattr(cv2.aruco, 'generateImageMarker'):
        return cv2.aruco.generateImageMarker(dictionary(), marker_id, side_px)
    return cv2.aruco.drawMarker(dictionary(), marker_id, side_px)


def render_sheet(px_per_mm: float, grid_mm: float = 10.0, grid_gray: int = 220) -> np.ndarray:
    """작업 시트 전체(A4 가로)를 그린 흑백 이미지. 행 0 이 시트 위쪽(y = 210 mm).

    격자는 정답 자세를 그릴 때 쓰는 옅은 회색 선이다. 이진화 임계값(100 또는 Otsu)보다 밝아 배경으로 처리된다.
    """
    width, height = (round(v * px_per_mm) for v in SHEET_MM)
    image = np.full((height, width), 255, np.uint8)
    if grid_mm > 0:
        for x in np.arange(0.0, SHEET_MM[0] + 1e-6, grid_mm):
            image[:, min(round(x * px_per_mm), width - 1)] = grid_gray
        for y in np.arange(0.0, SHEET_MM[1] + 1e-6, grid_mm):
            image[min(round((SHEET_MM[1] - y) * px_per_mm), height - 1), :] = grid_gray
    side = round(SHEET_MARKER_MM * px_per_mm)
    for marker_id, (x, y) in SHEET_MARKERS.items():
        col, row = round(x * px_per_mm), round((SHEET_MM[1] - y - SHEET_MARKER_MM) * px_per_mm)
        quiet = round(4 * px_per_mm)  # 마커 둘레 흰 여백: 격자선이 마커 테두리에 붙지 않게
        image[max(row - quiet, 0):row + side + quiet, max(col - quiet, 0):col + side + quiet] = 255
        image[row:row + side, col:col + side] = marker_image(marker_id, side)
    return image


def sheet_points(gray: np.ndarray, min_markers: int = 3):
    """작업 시트 사진 → (원본 px (N, 2), 시트 mm (N, 2), 찾은 ID 목록). 마커당 꼭짓점 4개, 왜곡 보정 전 좌표."""
    corners, ids = detect_markers(gray)
    found = [] if ids is None else [int(i) for i in ids]
    pixels, millimeters, used = [], [], []
    for marker_corners, marker_id in zip(corners, found):
        if marker_id in SHEET_MARKERS and marker_id not in used:
            pixels.append(marker_corners)
            millimeters.append(sheet_marker_corners_mm(marker_id))
            used.append(marker_id)
    if len(used) < min_markers:
        raise ValueError(f'작업 시트 마커가 {len(used)}개만 보입니다 (필요 {min_markers}개 이상, '
                         f'찾은 ID {sorted(found)}). 네 귀퉁이가 모두 화면에 들어오게 하세요.')
    order = np.argsort(used, kind='stable')  # ID 순으로 정렬해 출력·디버그 그림 순서를 일정하게
    return (np.concatenate([pixels[i] for i in order]), np.concatenate([millimeters[i] for i in order]),
            sorted(used))


def sheet_homography(gray: np.ndarray, camera_matrix=None, dist_coeffs=None, min_markers: int = 3):
    """작업 시트 사진 → (H, rms_mm, 찾은 ID 목록). H 는 (왜곡 보정된) px → 시트 mm.

    마커당 꼭짓점 4개를 모두 쓰므로 마커 4개면 기준점 16개다. camera_matrix 가 있으면 꼭짓점을 먼저 왜곡 보정한다
    (calibrator.Calibrator.px_to_mm 과 같은 순서).
    """
    pixels, millimeters, used = sheet_points(gray, min_markers)
    if camera_matrix is not None:
        pixels = cv2.undistortPoints(pixels.reshape(-1, 1, 2), camera_matrix, dist_coeffs,
                                     P=camera_matrix).reshape(-1, 2)
    H, _ = cv2.findHomography(pixels, millimeters, 0)
    if H is None or not np.isfinite(H).all() or np.linalg.matrix_rank(H) < 3:
        raise ValueError('호모그래피를 계산할 수 없습니다.')
    projected = cv2.perspectiveTransform(pixels.reshape(-1, 1, 2), H).reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((projected - millimeters) ** 2, axis=1))))
    return H, rms, used
