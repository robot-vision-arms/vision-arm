"""카메라 내부 파라미터·왜곡 계수 추정 (1주차). ROS 없이 단독 실행.

체커보드를 여러 각도로 15~20장 찍어 data/calib/checkerboard/ 에 넣고 실행한다.

    python scripts/calibrate_camera.py --images data/calib/checkerboard --cols 9 --rows 6 --square-mm 25
    → data/calib/intrinsics.npz (camera_matrix, dist_coeffs, rms)

--cols/--rows 는 '칸 수'가 아니라 '내부 코너 수'다 (10x7 칸 보드 → 9x6).
"""
import argparse


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--images', required=True)
    p.add_argument('--cols', type=int, required=True)
    p.add_argument('--rows', type=int, required=True)
    p.add_argument('--square-mm', type=float, required=True)
    p.add_argument('--out', default='data/calib/intrinsics.npz')
    args = p.parse_args()
    # TODO: cv2.findChessboardCorners → cornerSubPix → cv2.calibrateCamera → np.savez
    raise NotImplementedError


if __name__ == '__main__':
    main()
