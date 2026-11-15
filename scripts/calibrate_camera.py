"""카메라 내부 파라미터·왜곡 계수 추정 (1주차). ROS 없이 단독 실행.

기본은 ChArUco 보드다 (data/calib/charuco_board_A4.pdf, scripts/make_calib_sheets.py 로 생성).
보드를 손에 들고 여러 각도·위치로 15~20장 찍어 data/calib/charuco/ 에 넣고 실행한다.
보드 일부가 화면 밖으로 나가거나 가려져도 보이는 코너만으로 계산되므로 화면 가장자리까지 덮어 찍는다.

    python scripts/calibrate_camera.py --images data/calib/charuco
    → data/calib/intrinsics.npz (camera_matrix, dist_coeffs, rms, image_size)

예전 체커보드를 쓸 때 (--cols/--rows 는 '칸 수'가 아니라 '내부 코너 수', 10x7 칸 보드 → 9x6):

    python scripts/calibrate_camera.py --board chessboard --images data/calib/checkerboard --cols 9 --rows 6 --square-mm 25
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


def iter_gray_images(folder: str):
    for path in sorted(Path(folder).iterdir()):
        if path.suffix.lower() in IMAGE_SUFFIXES:
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if gray is not None:
                yield path, gray


def calibrate_charuco(p, images: str):
    board = aruco_calib.charuco_board()
    all_corners, all_ids, size = [], [], None
    for path, gray in iter_gray_images(images):
        corners, ids = aruco_calib.detect_charuco(gray, board)
        if ids is None or len(ids) < MIN_CHARUCO_CORNERS:
            print(f'SKIP 코너 {0 if ids is None else len(ids)}개: {path}')
            continue
        current_size = (gray.shape[1], gray.shape[0])
        if size is not None and size != current_size:
            p.error('모든 캘리브레이션 사진의 해상도가 같아야 합니다.')
        size = current_size
        all_corners.append(corners)
        all_ids.append(ids)
        print(f'OK   코너 {len(ids):2d}개: {path}')
    if len(all_corners) < 3:
        p.error('보드 검출에 성공한 사진이 3장 이상 필요합니다. 실제 측정은 다양한 각도로 15~20장 권장.')
    rms, matrix, distortion = aruco_calib.calibrate_charuco(all_corners, all_ids, size, board)
    return rms, matrix, distortion, size, len(all_corners)


def calibrate_chessboard(p, images: str, cols: int, rows: int, square_mm: float):
    if cols is None or rows is None or square_mm is None:
        p.error('--board chessboard 에는 --cols, --rows, --square-mm 가 필요합니다.')
    if cols < 2 or rows < 2 or not np.isfinite(square_mm) or square_mm <= 0:
        p.error('내부 코너 수는 각각 2 이상, 칸 크기는 양수여야 합니다.')
    pattern = (cols, rows)
    reference = np.zeros((cols * rows, 3), np.float32)
    reference[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    object_points, image_points, size = [], [], None
    for path, gray in iter_gray_images(images):
        found, corners = cv2.findChessboardCorners(gray, pattern)
        if not found:
            print(f'SKIP 코너 검출 실패: {path}')
            continue
        current_size = (gray.shape[1], gray.shape[0])
        if size is not None and size != current_size:
            p.error('모든 캘리브레이션 사진의 해상도가 같아야 합니다.')
        size = current_size
        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                  (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        object_points.append(reference.copy())
        image_points.append(refined)
    if len(image_points) < 3:
        p.error('코너 검출에 성공한 사진이 3장 이상 필요합니다. 실제 측정은 다양한 각도로 15~20장 권장.')
    rms, matrix, distortion, _, _ = cv2.calibrateCamera(object_points, image_points, size, None, None)
    return rms, matrix, distortion, size, len(image_points)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--images', required=True)
    p.add_argument('--board', choices=('charuco', 'chessboard'), default='charuco')
    p.add_argument('--cols', type=int, help='chessboard 전용: 내부 코너 가로 수')
    p.add_argument('--rows', type=int, help='chessboard 전용: 내부 코너 세로 수')
    p.add_argument('--square-mm', type=float, help='chessboard 전용: 칸 크기 (mm)')
    p.add_argument('--out', default='data/calib/intrinsics.npz')
    args = p.parse_args()
    if args.board == 'charuco':
        rms, matrix, distortion, size, count = calibrate_charuco(p, args.images)
    else:
        rms, matrix, distortion, size, count = calibrate_chessboard(p, args.images, args.cols, args.rows,
                                                                    args.square_mm)
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output, camera_matrix=matrix, dist_coeffs=distortion, rms=rms, image_size=size)
    print(f'{output}: {count}장, RMS={rms:.4f} px')


if __name__ == '__main__':
    main()
