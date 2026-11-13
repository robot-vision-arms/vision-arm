"""작업 평면 호모그래피 (px → mm) 계산 (1주차). ROS 없이 단독 실행.

카메라를 최종 위치에 고정한 뒤 모눈종이(격자) 를 촬영하고, 격자 교차점의 픽셀 좌표와
실제 mm 좌표를 짝지어 H 를 구한다. 4점이면 계산은 되지만 오차 확인이 안 되므로 8점 이상 권장.

    python scripts/compute_homography.py --image data/calib/grid.png --points data/calib/grid_points.csv
    → data/calib/homography.npz (H, reproj_err_mm)

grid_points.csv 형식: u_px,v_px,x_mm,y_mm  (원점·축 방향 = 로봇 좌표계 정의, docs/PROJECT.md 3.1)
카메라나 종이가 조금이라도 움직이면 다시 계산해야 한다.
"""
import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--image', required=True)
    p.add_argument('--points', required=True)
    p.add_argument('--intrinsics', default='data/calib/intrinsics.npz')
    p.add_argument('--out', default='data/calib/homography.npz')
    args = p.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        p.error(f'격자 사진을 읽을 수 없습니다: {args.image}')
    records = np.atleast_1d(np.genfromtxt(args.points, delimiter=',', names=True))
    points = np.column_stack([records[name] for name in ('u_px', 'v_px', 'x_mm', 'y_mm')])
    if len(points) < 4 or not np.isfinite(points).all():
        p.error('유한한 기준점이 4개 이상 필요합니다.')
    with np.load(args.intrinsics, allow_pickle=False) as data:
        if 'image_size' in data and tuple(data['image_size']) != (image.shape[1], image.shape[0]):
            p.error('내부 파라미터 사진과 격자 사진의 해상도가 다릅니다.')
        pixels = cv2.undistortPoints(points[:, :2].reshape(-1, 1, 2), data['camera_matrix'],
                                     data['dist_coeffs'], P=data['camera_matrix']).reshape(-1, 2)
    H, inliers = cv2.findHomography(pixels, points[:, 2:], cv2.RANSAC, 3.0)
    if H is None or not np.isfinite(H).all() or np.linalg.matrix_rank(H) < 3:
        p.error('호모그래피를 계산할 수 없습니다. 기준점을 평면 전체에 분산하세요.')
    projected = cv2.perspectiveTransform(pixels.reshape(-1, 1, 2), H).reshape(-1, 2)
    errors = np.linalg.norm(projected - points[:, 2:], axis=1)
    rms = float(np.sqrt(np.mean(errors ** 2)))
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    np.savez(output, H=H, reproj_err_mm=rms)
    print(f'{output}: RMS(전체 기준점)={rms:.4f} mm, inliers={int(inliers.sum())}/{len(points)}')
    if len(points) < 8:
        print('주의: 기준점 8개 이상과 별도 실측 정답으로 확인하는 것이 좋습니다.')


if __name__ == '__main__':
    main()
