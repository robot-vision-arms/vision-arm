"""작업 평면 호모그래피 (px → mm) 계산 (1주차). ROS 없이 단독 실행.

기본은 작업 시트(data/calib/work_sheet_A4.pdf, scripts/make_calib_sheets.py 로 생성)의 네 귀퉁이 ArUco 마커다.
폰을 최종 위치에 고정하고, 작업대에 붙인 시트를 부품 없이 한 장 찍은 뒤 실행한다.
마커 꼭짓점을 자동으로(서브픽셀) 찾고, 인쇄 위치로 정해진 mm 좌표와 짝지어 H 를 구한다 (마커 4개 → 기준점 16개).

    python scripts/compute_homography.py --aruco --image data/calib/sheet.png
    → data/calib/homography.npz (H, reproj_err_mm)

좌표계: 원점 = 시트 왼쪽 아래 모서리, +X 오른쪽, +Y 위쪽 (docs/PROJECT.md 3.1, vision_pick_place/aruco_calib.py).
카메라나 시트가 조금이라도 움직이면 다시 찍어서 다시 계산한다 (마커가 그대로 붙어 있으므로 명령 한 번이면 된다).

예전 모눈종이 방식 (기준점을 직접 찍은 CSV, 형식 u_px,v_px,x_mm,y_mm):

    python scripts/compute_homography.py --image data/calib/grid.png --points data/calib/grid_points.csv
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'vision_pick_place'))
from vision_pick_place import aruco_calib  # noqa: E402


def from_points_csv(p, points_path: str, camera_matrix, dist_coeffs):
    records = np.atleast_1d(np.genfromtxt(points_path, delimiter=',', names=True))
    points = np.column_stack([records[name] for name in ('u_px', 'v_px', 'x_mm', 'y_mm')])
    if len(points) < 4 or not np.isfinite(points).all():
        p.error('유한한 기준점이 4개 이상 필요합니다.')
    pixels = cv2.undistortPoints(points[:, :2].reshape(-1, 1, 2), camera_matrix,
                                 dist_coeffs, P=camera_matrix).reshape(-1, 2)
    H, inliers = cv2.findHomography(pixels, points[:, 2:], cv2.RANSAC, 3.0)
    if H is None or not np.isfinite(H).all() or np.linalg.matrix_rank(H) < 3:
        p.error('호모그래피를 계산할 수 없습니다. 기준점을 평면 전체에 분산하세요.')
    projected = cv2.perspectiveTransform(pixels.reshape(-1, 1, 2), H).reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((projected - points[:, 2:]) ** 2, axis=1))))
    note = f'inliers={int(inliers.sum())}/{len(points)}'
    if len(points) < 8:
        note += ' (주의: 기준점 8개 이상과 별도 실측 정답으로 확인하는 것이 좋습니다)'
    return H, rms, note


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--image', required=True, help='폰을 고정한 위치에서 찍은 작업 시트(또는 모눈종이) 사진')
    p.add_argument('--aruco', action='store_true', help='작업 시트의 ArUco 마커로 자동 계산')
    p.add_argument('--points', help='모눈종이 방식: 기준점 CSV (u_px,v_px,x_mm,y_mm)')
    p.add_argument('--intrinsics', default='data/calib/intrinsics.npz')
    p.add_argument('--out', default='data/calib/homography.npz')
    args = p.parse_args()
    if args.aruco == bool(args.points):
        p.error('--aruco 또는 --points 중 하나만 지정하세요.')
    image = cv2.imread(args.image)
    if image is None:
        p.error(f'사진을 읽을 수 없습니다: {args.image}')
    if not Path(args.intrinsics).is_file():
        p.error(f'내부 파라미터가 없습니다: {args.intrinsics}. scripts/calibrate_camera.py 를 먼저 실행하세요.')
    with np.load(args.intrinsics, allow_pickle=False) as data:
        if 'image_size' in data and tuple(data['image_size']) != (image.shape[1], image.shape[0]):
            p.error('내부 파라미터 사진과 이 사진의 해상도가 다릅니다. 같은 폰·같은 해상도로 찍으세요.')
        camera_matrix, dist_coeffs = data['camera_matrix'], data['dist_coeffs']
    if args.aruco:
        try:
            H, rms, ids = aruco_calib.sheet_homography(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY),
                                                       camera_matrix, dist_coeffs)
        except ValueError as error:
            p.error(str(error))
        note = f'마커 {ids} → 기준점 {4 * len(ids)}개'
    else:
        H, rms, note = from_points_csv(p, args.points, camera_matrix, dist_coeffs)
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output, H=H, reproj_err_mm=rms)
    print(f'{output}: RMS(전체 기준점)={rms:.4f} mm, {note}')


if __name__ == '__main__':
    main()
