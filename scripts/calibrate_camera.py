"""카메라 내부 파라미터·왜곡 계수 추정 (1주차). ROS 없이 단독 실행.

체커보드를 여러 각도로 15~20장 찍어 data/calib/checkerboard/ 에 넣고 실행한다.

    python scripts/calibrate_camera.py --images data/calib/checkerboard --cols 9 --rows 6 --square-mm 25
    → data/calib/intrinsics.npz (camera_matrix, dist_coeffs, rms)

--cols/--rows 는 '칸 수'가 아니라 '내부 코너 수'다 (10x7 칸 보드 → 9x6).
"""
import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--images', required=True)
    p.add_argument('--cols', type=int, required=True)
    p.add_argument('--rows', type=int, required=True)
    p.add_argument('--square-mm', type=float, required=True)
    p.add_argument('--out', default='data/calib/intrinsics.npz')
    args = p.parse_args()
    if args.cols < 2 or args.rows < 2 or not np.isfinite(args.square_mm) or args.square_mm <= 0:
        p.error('내부 코너 수는 각각 2 이상, 칸 크기는 양수여야 합니다.')
    pattern = (args.cols, args.rows)
    reference = np.zeros((args.cols * args.rows, 3), np.float32)
    reference[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_mm
    object_points, image_points = [], []
    size = None
    for path in sorted(Path(args.images).iterdir()):
        if path.suffix.lower() not in ('.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'):
            continue
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
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
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output, camera_matrix=matrix, dist_coeffs=distortion, rms=rms, image_size=size)
    print(f'{output}: {len(image_points)}장, RMS={rms:.4f} px')


if __name__ == '__main__':
    main()
