"""카메라 내부 파라미터·왜곡 계수 추정 (1주차). ROS 없이 단독 실행.

기본은 ChArUco 보드다 (data/calib/charuco_board_A4.pdf, scripts/make_calib_sheets.py 로 생성).
보드를 손에 들고 여러 각도·위치로 15~20장 찍어 data/calib/charuco/ 에 넣고 실행한다.
보드 일부가 화면 밖으로 나가거나 가려져도 보이는 코너만으로 계산되므로 화면 가장자리까지 덮어 찍는다.

    python scripts/calibrate_camera.py --images data/calib/charuco
    → data/calib/intrinsics.npz (camera_matrix, dist_coeffs, rms, image_size)

예전 체커보드를 쓸 때 (--cols/--rows 는 '칸 수'가 아니라 '내부 코너 수', 10x7 칸 보드 → 9x6):

    python scripts/calibrate_camera.py --board chessboard --images data/calib/checkerboard --cols 9 --rows 6 --square-mm 25

--debug-dir 를 주면 찾은 코너를 그린 사진을 저장한다 (잘 찾았는지 눈으로 확인용).

[쉽게 말하면]
카메라 렌즈는 볼록해서 사진 속 직선이 살짝 휘어 보인다 (특히 사진 구석).
얼마나 휘는지 알려면 "진짜 모양을 정확히 아는 물건" 을 찍어 보면 된다.
보드는 네모 칸이 반듯하게 줄 서 있다는 걸 우리가 알고 있으니,
사진 속 칸이 얼마나 비뚤어졌는지 보고 거꾸로 렌즈의 휘는 정도를 계산한다.

결과물 두 가지:
    camera_matrix (카메라 행렬) : 초점거리(얼마나 확대해서 보나)와 사진 중심이 어디인지
    dist_coeffs   (왜곡 계수)   : 렌즈가 얼마나 휘게 만드나. calibrator.py 가 이걸로 휜 점을 편다
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'vision_pick_place'))
from vision_pick_place import aruco_calib  # noqa: E402

IMAGE_SUFFIXES = ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff')
MIN_CHARUCO_CORNERS = 6  # 이보다 적게 보이는 사진은 버린다 (7x5 보드의 내부 코너는 24개)

# 사진이 이보다 적으면 결과를 믿기 어렵다. OpenCV 문서·튜토리얼에서 흔히 권하는 10장 이상을 기준으로 잡았다.
MIN_GOOD_IMAGES = 10

# 코너 위치를 픽셀보다 더 잘게(소수점까지) 다듬을 때 "언제 멈출지" 정하는 규칙.
# 30번 고치거나, 한 번 고쳐도 0.001 픽셀밖에 안 움직이면 멈춘다. OpenCV 튜토리얼 기본값 그대로.
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def iter_images(p, folder: str):
    """(경로, 컬러, 흑백). 해상도가 섞이면 멈춘다: 해상도가 바뀌면 초점거리(픽셀 단위)도 바뀌어서 섞으면 엉터리가 된다."""
    paths = sorted(f for f in Path(folder).iterdir() if f.suffix.lower() in IMAGE_SUFFIXES)
    if not paths:
        p.error(f'사진이 없습니다: {folder}')
    size = None
    for path in paths:
        img = cv2.imread(str(path))
        if img is None:
            print(f'SKIP 읽을 수 없음: {path.name}')
            continue
        current_size = (img.shape[1], img.shape[0])
        if size is not None and size != current_size:
            p.error(f'모든 캘리브레이션 사진의 해상도가 같아야 합니다: {path.name} {current_size} ≠ {size}')
        size = current_size
        yield path, img, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def save_debug(debug_dir, path: Path, img):
    if debug_dir:
        cv2.imwrite(str(Path(debug_dir) / path.name), img)


def calibrate_charuco(p, images: str, debug_dir):
    board = aruco_calib.charuco_board()
    all_corners, all_ids, used, size = [], [], [], None
    for path, img, gray in iter_images(p, images):
        corners, ids = aruco_calib.detect_charuco(gray, board)
        if ids is None or len(ids) < MIN_CHARUCO_CORNERS:
            # 흔한 원인: 흔들림, 반사광, 보드가 너무 작게 찍힘
            print(f'SKIP 코너 {0 if ids is None else len(ids)}개: {path.name}')
            continue
        size = (gray.shape[1], gray.shape[0])
        all_corners.append(corners)
        all_ids.append(ids)
        used.append(path.name)
        print(f'OK   코너 {len(ids):2d}개: {path.name}')
        if debug_dir:
            cv2.aruco.drawDetectedCornersCharuco(img, corners, ids)
            save_debug(debug_dir, path, img)
    if len(used) < 3:
        p.error('보드 검출에 성공한 사진이 3장 이상 필요합니다. 실제 측정은 다양한 각도로 15~20장 권장.')
    rms, matrix, distortion, per_view = aruco_calib.calibrate_charuco_extended(all_corners, all_ids, size, board)
    return rms, matrix, distortion, size, used, per_view


def board_points_mm(cols: int, rows: int, square_mm: float) -> np.ndarray:
    """체커보드 코너들의 '진짜' 위치(mm). 보드는 평평하니까 z 는 전부 0.

    9x6 코너, 칸 25mm 라면 (0,0,0), (25,0,0), (50,0,0) ... (200,125,0) 처럼 54개가 나온다.
    순서는 findChessboardCorners 가 코너를 찾아 주는 순서(한 줄씩 왼→오)와 같아야 한다.
    """
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    return grid


def find_chessboard_corners(gray: np.ndarray, cols: int, rows: int) -> np.ndarray | None:
    """1단계 findChessboardCorners 로 대략 찾고, 2단계 cornerSubPix 로 0.1 픽셀 단위까지 다듬는다. 못 찾으면 None."""
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE  # 조명이 고르지 않아도 잘 찾게 해 주는 옵션
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not found:
        return None
    # (11, 11) = 코너 주변 몇 픽셀까지 보고 다듬을지(가로세로 23px 창). 칸이 아주 작게 찍히면 줄여야 한다
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)


def calibrate_chessboard(p, images: str, cols: int, rows: int, square_mm: float, debug_dir):
    if cols is None or rows is None or square_mm is None:
        p.error('--board chessboard 에는 --cols, --rows, --square-mm 가 필요합니다.')
    if cols < 2 or rows < 2 or not np.isfinite(square_mm) or square_mm <= 0:
        p.error('내부 코너 수는 각각 2 이상, 칸 크기는 양수여야 합니다.')
    board = board_points_mm(cols, rows, square_mm)
    object_points, image_points, used, size = [], [], [], None
    for path, img, gray in iter_images(p, images):
        corners = find_chessboard_corners(gray, cols, rows)
        if corners is None:
            # 흔한 원인: 보드 일부가 화면 밖으로 잘림, 흔들림, 반사광, cols/rows 를 칸 수로 잘못 넣음
            print(f'SKIP 코너 검출 실패: {path.name}')
            continue
        size = (gray.shape[1], gray.shape[0])
        object_points.append(board)
        image_points.append(corners)
        used.append(path.name)
        if debug_dir:
            cv2.drawChessboardCorners(img, (cols, rows), corners, True)
            save_debug(debug_dir, path, img)
    if len(used) < 3:
        p.error('코너 검출에 성공한 사진이 3장 이상 필요합니다. 실제 측정은 다양한 각도로 15~20장 권장.')
    rms, matrix, distortion, _, _, _, _, per_view = cv2.calibrateCameraExtended(
        object_points, image_points, size, None, None)
    return rms, matrix, distortion, size, used, np.asarray(per_view).reshape(-1)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--images', required=True)
    p.add_argument('--board', choices=('charuco', 'chessboard'), default='charuco')
    p.add_argument('--cols', type=int, help='chessboard 전용: 내부 코너 가로 수')
    p.add_argument('--rows', type=int, help='chessboard 전용: 내부 코너 세로 수')
    p.add_argument('--square-mm', type=float, help='chessboard 전용: 칸 크기 (mm)')
    p.add_argument('--out', default='data/calib/intrinsics.npz')
    p.add_argument('--debug-dir', default=None, help='주면 코너를 그린 사진을 여기에 저장한다 (잘 찾았는지 눈으로 확인용)')
    args = p.parse_args()
    if args.debug_dir:
        Path(args.debug_dir).mkdir(parents=True, exist_ok=True)

    # 핵심 계산: 모든 사진의 (진짜 mm, 사진 px) 짝을 가장 잘 설명하는 카메라 행렬·왜곡 계수를 찾는다.
    # rms = 찾은 값으로 코너를 다시 그려 봤을 때 실제 코너와 평균 몇 픽셀 어긋나는지 (작을수록 좋음)
    if args.board == 'charuco':
        rms, K, dist, size, used, per_view = calibrate_charuco(p, args.images, args.debug_dir)
    else:
        rms, K, dist, size, used, per_view = calibrate_chessboard(p, args.images, args.cols, args.rows,
                                                                  args.square_mm, args.debug_dir)
    if len(used) < MIN_GOOD_IMAGES:
        print(f'[주의] {len(used)}장으로 계산. {MIN_GOOD_IMAGES}장보다 적으면 결과가 불안정할 수 있으니 더 찍는 게 좋다')

    # 사진별 오차: 유독 큰 사진은 흔들렸거나 보드가 휘었을 가능성이 크다 → 빼고 다시 돌려 볼 후보
    print('사진별 재투영 오차(px):')
    for name, err in zip(used, per_view):
        # 평균의 2배를 넘고, 0.5px 도 넘을 때만 표시 (전체가 아주 좋을 때 0.3px 같은 작은 값까지 겁주지 않게)
        mark = '  ← 큼, 다시 찍거나 빼 볼 것' if err > max(2 * rms, 0.5) else ''
        print(f'  {name:30s} {err:.3f}{mark}')

    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output, camera_matrix=K, dist_coeffs=dist, rms=rms, image_size=np.array(size))

    # 대략적인 기준: 1px 아래면 쓸 만하고, 0.5px 아래면 좋은 편. 폰 사진 기준으로 검증한 값은 아니다.
    print(f'\n{output}: {len(used)}장, RMS={rms:.4f} px  (1 px 아래면 쓸 만함)')
    print(f'초점거리 fx={K[0, 0]:.1f}, fy={K[1, 1]:.1f}  중심 cx={K[0, 2]:.1f}, cy={K[1, 2]:.1f}')
    print(f'왜곡 계수 {np.round(dist.ravel(), 4).tolist()}')


if __name__ == '__main__':
    main()
