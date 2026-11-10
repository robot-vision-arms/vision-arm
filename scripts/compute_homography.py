"""작업 평면 호모그래피 (px → mm) 계산 (1주차). ROS 없이 단독 실행.

카메라를 최종 위치에 고정한 뒤 모눈종이(격자) 를 촬영하고, 격자 교차점의 픽셀 좌표와
실제 mm 좌표를 짝지어 H 를 구한다. 4점이면 계산은 되지만 오차 확인이 안 되므로 8점 이상 권장.

    python scripts/compute_homography.py --image data/calib/grid.png --points data/calib/grid_points.csv
    → data/calib/homography.npz (H, reproj_err_mm)

grid_points.csv 형식: u_px,v_px,x_mm,y_mm  (원점·축 방향 = 로봇 좌표계 정의, docs/PROJECT.md 3.1)
카메라나 종이가 조금이라도 움직이면 다시 계산해야 한다.
"""
import argparse


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--image', required=True)
    p.add_argument('--points', required=True)
    p.add_argument('--intrinsics', default='data/calib/intrinsics.npz')
    p.add_argument('--out', default='data/calib/homography.npz')
    args = p.parse_args()
    # TODO: 픽셀 좌표 왜곡 보정 → cv2.findHomography(px, mm, cv2.RANSAC) → 재투영 오차(mm) 출력 → np.savez
    raise NotImplementedError


if __name__ == '__main__':
    main()
