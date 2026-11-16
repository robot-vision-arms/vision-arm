"""카메라 내부 파라미터·왜곡 계수 추정 (1주차). ROS 없이 단독 실행.

체커보드를 여러 각도로 15~20장 찍어 data/calib/checkerboard/ 에 넣고 실행한다.

    python scripts/calibrate_camera.py --images data/calib/checkerboard --cols 9 --rows 6 --square-mm 25
    → data/calib/intrinsics.npz (camera_matrix, dist_coeffs, rms, image_size)

--cols/--rows 는 '칸 수'가 아니라 '내부 코너 수'다 (10x7 칸 보드 → 9x6).

[쉽게 말하면]
카메라 렌즈는 볼록해서 사진 속 직선이 살짝 휘어 보인다 (특히 사진 구석).
얼마나 휘는지 알려면 "진짜 모양을 정확히 아는 물건" 을 찍어 보면 된다.
체커보드는 네모 칸이 반듯하게 줄 서 있다는 걸 우리가 알고 있으니,
사진 속 칸이 얼마나 비뚤어졌는지 보고 거꾸로 렌즈의 휘는 정도를 계산한다.

결과물 두 가지:
    camera_matrix (카메라 행렬) : 초점거리(얼마나 확대해서 보나)와 사진 중심이 어디인지
    dist_coeffs   (왜곡 계수)   : 렌즈가 얼마나 휘게 만드나. calibrator.py 가 이걸로 휜 점을 편다
"""
import argparse
from pathlib import Path

import cv2
import numpy as np

IMAGE_EXTS = {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff'}

# 사진이 이보다 적으면 결과를 믿기 어렵다. OpenCV 문서·튜토리얼에서 흔히 권하는 10장 이상을 기준으로 잡았다.
MIN_GOOD_IMAGES = 10

# 코너 위치를 픽셀보다 더 잘게(소수점까지) 다듬을 때 "언제 멈출지" 정하는 규칙.
# 30번 고치거나, 한 번 고쳐도 0.001 픽셀밖에 안 움직이면 멈춘다. OpenCV 튜토리얼 기본값 그대로.
SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)


def board_points_mm(cols: int, rows: int, square_mm: float) -> np.ndarray:
    """체커보드 코너들의 '진짜' 위치(mm) 를 만든다.

    보드는 평평하니까 높이(z)는 전부 0 이다.
    9x6 코너, 칸 25mm 라면 (0,0,0), (25,0,0), (50,0,0) ... (200,125,0) 처럼 54개가 나온다.
    순서는 findChessboardCorners 가 코너를 찾아 주는 순서(한 줄씩 왼→오)와 같아야 한다.
    """
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm
    return grid


def find_corners(gray: np.ndarray, cols: int, rows: int) -> np.ndarray | None:
    """흑백 사진에서 체커보드 코너를 찾는다. 못 찾으면 None.

    1단계 findChessboardCorners : 대충 어느 픽셀에 코너가 있는지 찾는다 (정수 픽셀 근처)
    2단계 cornerSubPix          : 그 주변을 자세히 봐서 0.1 픽셀 단위까지 정확하게 다듬는다
    다듬는 단계가 중요하다. 1픽셀 틀리면 나중에 mm 로 바꿀 때 그대로 오차가 된다.
    """
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE  # 조명이 고르지 않아도 잘 찾게 해 주는 옵션
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not found:
        return None
    # (11, 11) = 코너 주변 몇 픽셀까지 보고 다듬을지(가로세로 23px 창). 칸이 아주 작게 찍히면 줄여야 한다
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), SUBPIX_CRITERIA)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--images', required=True)
    p.add_argument('--cols', type=int, required=True)
    p.add_argument('--rows', type=int, required=True)
    p.add_argument('--square-mm', type=float, required=True)
    p.add_argument('--out', default='data/calib/intrinsics.npz')
    p.add_argument('--debug-dir', default=None, help='주면 코너를 그린 사진을 여기에 저장한다 (잘 찾았는지 눈으로 확인용)')
    args = p.parse_args()
    if args.cols < 2 or args.rows < 2 or not np.isfinite(args.square_mm) or args.square_mm <= 0:
        p.error('내부 코너 수는 각각 2 이상, 칸 크기는 양수여야 합니다.')

    paths = sorted(f for f in Path(args.images).iterdir() if f.suffix.lower() in IMAGE_EXTS)
    if not paths:
        p.error(f'사진이 없습니다: {args.images}')
    if args.debug_dir:
        Path(args.debug_dir).mkdir(parents=True, exist_ok=True)

    board = board_points_mm(args.cols, args.rows, args.square_mm)
    obj_points, img_points, used = [], [], []  # 사진마다: 진짜 위치(mm) 와 사진 속 위치(px) 를 짝지어 모은다
    image_size = None

    for path in paths:
        img = cv2.imread(str(path))
        if img is None:
            print(f'  [건너뜀] 읽을 수 없음: {path.name}')
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners = find_corners(gray, args.cols, args.rows)
        if corners is None:
            # 흔한 원인: 보드 일부가 화면 밖으로 잘림, 흔들림, 반사광, cols/rows 를 칸 수로 잘못 넣음
            print(f'  [건너뜀] 코너 못 찾음: {path.name}')
            continue

        # 사진 크기가 다 같아야 한다. 해상도가 바뀌면 초점거리(픽셀 단위)도 바뀌어서 섞으면 엉터리가 된다.
        size = (gray.shape[1], gray.shape[0])  # (가로, 세로)
        if image_size is not None and size != image_size:
            p.error(f'모든 캘리브레이션 사진의 해상도가 같아야 합니다: {path.name} {size} ≠ {image_size}')
        image_size = size

        obj_points.append(board)
        img_points.append(corners)
        used.append(path.name)
        if args.debug_dir:
            cv2.drawChessboardCorners(img, (args.cols, args.rows), corners, True)
            cv2.imwrite(str(Path(args.debug_dir) / path.name), img)

    print(f'코너 찾은 사진: {len(used)} / {len(paths)}')
    if len(used) < 3:
        p.error('코너 검출에 성공한 사진이 3장 이상 필요합니다. 실제 측정은 다양한 각도로 15~20장 권장.')
    if len(used) < MIN_GOOD_IMAGES:
        print(f'  [주의] {MIN_GOOD_IMAGES}장보다 적다. 결과가 불안정할 수 있으니 더 찍는 게 좋다')

    # 핵심 계산: 모든 사진의 (진짜 mm, 사진 px) 짝을 가장 잘 설명하는 카메라 행렬·왜곡 계수를 찾는다.
    # rms = 찾은 값으로 코너를 다시 그려 봤을 때 실제 코너와 평균 몇 픽셀 어긋나는지 (작을수록 좋음)
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, image_size, None, None)

    # 사진별 오차: 유독 큰 사진은 흔들렸거나 보드가 휘었을 가능성이 크다 → 빼고 다시 돌려 볼 후보
    print('사진별 재투영 오차(px):')
    for name, obj, img_pts, rvec, tvec in zip(used, obj_points, img_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = np.sqrt(np.mean(np.sum((projected - img_pts) ** 2, axis=2)))
        # 평균의 2배를 넘고, 0.5px 도 넘을 때만 표시 (전체가 아주 좋을 때 0.3px 같은 작은 값까지 겁주지 않게)
        mark = '  ← 큼, 다시 찍거나 빼 볼 것' if err > max(2 * rms, 0.5) else ''
        print(f'  {name:30s} {err:.3f}{mark}')

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, camera_matrix=K, dist_coeffs=dist, rms=rms, image_size=np.array(image_size))

    # 대략적인 기준: 1px 아래면 쓸 만하고, 0.5px 아래면 좋은 편. 폰 사진 기준으로 검증한 값은 아니다.
    print(f'\nrms = {rms:.3f} px  (1 px 아래면 쓸 만함)')
    print(f'초점거리 fx={K[0, 0]:.1f}, fy={K[1, 1]:.1f}  중심 cx={K[0, 2]:.1f}, cy={K[1, 2]:.1f}')
    print(f'왜곡 계수 {np.round(dist.ravel(), 4).tolist()}')
    print(f'저장: {out}')


if __name__ == '__main__':
    main()
