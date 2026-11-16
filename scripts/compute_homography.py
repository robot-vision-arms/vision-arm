"""작업 평면 호모그래피 (px → mm) 계산 (1주차). ROS 없이 단독 실행.

기본은 작업 시트(data/calib/work_sheet_A4.pdf, scripts/make_calib_sheets.py 로 생성)의 네 귀퉁이 ArUco 마커다.
폰을 최종 위치에 고정하고, 작업대에 붙인 시트를 부품 없이 한 장 찍은 뒤 실행한다.
마커 꼭짓점을 자동으로(서브픽셀) 찾고, 인쇄 위치로 정해진 mm 좌표와 짝지어 H 를 구한다 (마커 4개 → 기준점 16개).

    python scripts/compute_homography.py --aruco --image data/calib/sheet.png
    → data/calib/homography.npz (H, reproj_err_mm, loo_err_mm, undistorted)

좌표계: 원점 = 시트 왼쪽 아래 모서리, +X 오른쪽, +Y 위쪽 (docs/PROJECT.md 3.1, vision_pick_place/aruco_calib.py).
카메라나 시트가 조금이라도 움직이면 다시 찍어서 다시 계산한다 (마커가 그대로 붙어 있으므로 명령 한 번이면 된다).

예전 모눈종이 방식 (기준점을 직접 찍은 CSV, 형식 u_px,v_px,x_mm,y_mm):

    python scripts/compute_homography.py --image data/calib/grid.png --points data/calib/grid_points.csv

[쉽게 말하면]
카메라가 책상을 비스듬히 내려다보면, 사진 속 네모 종이가 사다리꼴로 찍힌다.
호모그래피(H) 는 "사진 속 사다리꼴 → 위에서 똑바로 본 네모" 로 펴 주는 3x3 표다.
종이 위 점 몇 개에 대해 "사진에서는 여기(px), 실제로는 여기(mm)" 를 알려 주면
H 를 계산할 수 있고, 그다음부터는 사진 속 아무 점이나 mm 로 바꿀 수 있다.

순서:
    1. (사진 px, 실제 mm) 짝을 모은다: 마커 꼭짓점 자동 검출 또는 CSV
    2. 렌즈 때문에 휜 px 를 먼저 편다 (intrinsics.npz 필수). calibrator.py 와 똑같은 방법
    3. 편 px → mm 로 가는 H 를 구한다
    4. H 로 다시 바꿔 봤을 때 실제 mm 와 얼마나 어긋나는지(오차) 를 잰다
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'vision_pick_place'))
from vision_pick_place import aruco_calib  # noqa: E402

# RANSAC 이 "이 점은 잘못 찍은 점이다" 라고 판단하는 기준(mm). CSV(손으로 찍은 점) 에만 쓴다.
# 손으로 교차점을 찍으면 보통 1mm 안쪽으로 맞으니 3mm 넘게 어긋나면 클릭 실수로 본다. 실제 사진으로 검증 전인 초기값.
RANSAC_THRESHOLD_MM = 3.0


def read_points_csv(p, path: str) -> tuple[np.ndarray, np.ndarray]:
    """csv 를 읽어 (사진 px 목록, 실제 mm 목록) 을 돌려준다. 첫 줄은 제목 줄(u_px,v_px,x_mm,y_mm)."""
    records = np.atleast_1d(np.genfromtxt(path, delimiter=',', names=True))
    points = np.column_stack([records[name] for name in ('u_px', 'v_px', 'x_mm', 'y_mm')]).astype(np.float64)
    if len(points) < 4 or not np.isfinite(points).all():
        p.error(f'유한한 기준점이 4개 이상 필요합니다 (지금 {len(points)}개, 권장 8개 이상).')
    if len(points) < 8:
        print(f'[주의] 점이 {len(points)}개. 8개 이상과 별도 실측 정답으로 확인하는 것이 좋다')
    return points[:, :2], points[:, 2:]


def undistort(px: np.ndarray, K: np.ndarray, dist: np.ndarray) -> np.ndarray:
    """렌즈 때문에 휜 픽셀 → 편 픽셀. calibrator.Calibrator.undistort_points 와 같은 계산이어야 한다.

    여기서 편 픽셀로 H 를 만들었으니, 나중에 쓸 때도 반드시 펴고 나서 H 를 써야 한다 (calibrator.py 가 그렇게 함).
    """
    return cv2.undistortPoints(px.reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)


def apply_h(H: np.ndarray, px: np.ndarray) -> np.ndarray:
    """H 로 px → mm. 3x3 표를 곱한 뒤 마지막 숫자로 나눠 줘야 해서(원근) cv2 함수를 쓴다."""
    return cv2.perspectiveTransform(px.reshape(-1, 1, 2), H).reshape(-1, 2)


def leave_one_out_errors(px: np.ndarray, mm: np.ndarray) -> np.ndarray:
    """점 하나를 빼고 H 를 만든 뒤, 뺀 점을 맞히는지 본다. 모든 점에 대해 한 번씩.

    왜 필요한가: H 를 만든 점으로 오차를 재면 "시험 문제를 미리 보고 시험 친 것" 이라 점수가 실제보다 좋게 나온다.
    한 점씩 숨기고 맞혀 보면, 처음 보는 위치(실제로 부품이 놓일 곳) 에서의 오차에 더 가깝다.
    점이 5개 이상이어야 한다 (하나 빼도 4개가 남아야 H 를 구할 수 있음).
    마커 방식은 같은 마커의 꼭짓점 3개가 남아 있어서 실제 위치 오차보다 조금 좋게 나올 수 있다.
    """
    errs = []
    for i in range(len(px)):
        keep = np.arange(len(px)) != i
        H_i, _ = cv2.findHomography(px[keep], mm[keep], 0)  # 0 = 최소제곱. 남은 점을 전부 공평하게 쓴다
        errs.append(np.linalg.norm(apply_h(H_i, px[i:i + 1])[0] - mm[i]))
    return np.array(errs)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--image', required=True, help='폰을 고정한 위치에서 찍은 작업 시트(또는 모눈종이) 사진')
    p.add_argument('--aruco', action='store_true', help='작업 시트의 ArUco 마커로 자동 계산')
    p.add_argument('--points', help='모눈종이 방식: 기준점 CSV (u_px,v_px,x_mm,y_mm)')
    p.add_argument('--intrinsics', default='data/calib/intrinsics.npz')
    p.add_argument('--out', default='data/calib/homography.npz')
    p.add_argument('--no-undistort', action='store_true',
                   help='왜곡 보정을 안 한다. 보정 켜고/끄고 오차 비교용 (결과 파일은 calibrator.py 가 거부)')
    p.add_argument('--debug-out', default=None, help='주면 기준점과 오차를 사진 위에 그려 저장한다')
    args = p.parse_args()
    if args.aruco == bool(args.points):
        p.error('--aruco 또는 --points 중 하나만 지정하세요.')
    img = cv2.imread(args.image)
    if img is None:
        p.error(f'사진을 읽을 수 없습니다: {args.image}')

    # --- 1. (px, mm) 짝 모으기 ---
    if args.aruco:
        try:
            px_raw, mm, ids = aruco_calib.sheet_points(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        except ValueError as error:
            p.error(str(error))
        print(f'마커 {ids} → 기준점 {len(px_raw)}개')
    else:
        px_raw, mm = read_points_csv(p, args.points)
    n = len(px_raw)

    # --- 2. 휜 픽셀 펴기 ---
    # intrinsics 는 필수다. calibrator.py 는 항상 왜곡 보정 후 H 를 적용하므로, 원본 픽셀로 만든 H 는 노드에서 쓸 수 없다.
    if args.no_undistort:
        px = px_raw
        print('왜곡 보정: 끔 (--no-undistort). 오차 비교용이며 이 결과 파일은 calibrator.py 가 거부한다')
    else:
        if not Path(args.intrinsics).is_file():
            p.error(f'내부 파라미터가 없습니다: {args.intrinsics}. scripts/calibrate_camera.py 를 먼저 실행하세요.')
        with np.load(args.intrinsics, allow_pickle=False) as f:
            # 보드 사진과 해상도가 다르면 카메라 행렬이 안 맞는다 → 펴는 게 오히려 망가뜨린다
            if 'image_size' in f and tuple(f['image_size']) != (img.shape[1], img.shape[0]):
                p.error('내부 파라미터 사진과 이 사진의 해상도가 다릅니다. 같은 폰·같은 해상도로 찍으세요.')
            K, dist = f['camera_matrix'], f['dist_coeffs']
        px = undistort(px_raw, K, dist)
        print(f'왜곡 보정: 켬 ({args.intrinsics})')

    # --- 3. H 구하기 ---
    if args.aruco:
        # 마커 꼭짓점은 자동 검출이라 클릭 실수가 없다 → 전부 써서 최소제곱
        H, _ = cv2.findHomography(px, mm, 0)
        inliers = np.ones(n, dtype=bool)
    else:
        # RANSAC = 일부러 몇 점씩 뽑아 H 를 여러 번 만들어 보고, 가장 많은 점이 동의하는 H 를 고르는 방법.
        # 손으로 찍다 한두 점 잘못 찍어도 그 점에 끌려가지 않는다.
        H, inlier_mask = cv2.findHomography(px, mm, cv2.RANSAC, RANSAC_THRESHOLD_MM)
        inliers = inlier_mask.ravel().astype(bool) if inlier_mask is not None else np.zeros(n, dtype=bool)
    if H is None or not np.isfinite(H).all() or np.linalg.matrix_rank(H) < 3:
        p.error('호모그래피를 계산할 수 없습니다. 기준점을 평면 전체에 분산하세요.')

    # --- 4. 오차 재기 ---
    err = np.linalg.norm(apply_h(H, px) - mm, axis=1)  # 점마다: H 로 바꾼 위치와 진짜 mm 사이 거리
    print(f'\n점별 오차(mm)  [{inliers.sum()} / {n} 개 사용]')
    for i in range(n):
        mark = '' if inliers[i] else '  ← 제외됨, 이 점 좌표를 다시 확인할 것'
        print(f'  #{i:2d} px=({px_raw[i, 0]:7.1f},{px_raw[i, 1]:7.1f}) mm=({mm[i, 0]:6.1f},{mm[i, 1]:6.1f})  {err[i]:.3f}{mark}')

    reproj_err_mm = float(np.sqrt(np.mean(err ** 2)))  # 제외된 점까지 포함한 전체 기준점 RMS
    loo = leave_one_out_errors(px[inliers], mm[inliers]) if inliers.sum() >= 5 else None
    loo_err_mm = float(loo.mean()) if loo is not None else float('nan')

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, H=H, reproj_err_mm=reproj_err_mm, loo_err_mm=loo_err_mm, undistorted=not args.no_undistort)

    print(f'\n{out}: RMS(전체 기준점)={reproj_err_mm:.4f} mm  (H 를 만든 점 그대로 잰 값, 실제보다 좋게 나옴)')
    if loo is not None:
        print(f'한 점씩 빼고 잰 오차 = {loo_err_mm:.3f} mm  (처음 보는 위치에서의 오차에 더 가까움. KPI 에는 이 값도 같이 적을 것)')
    else:
        print('한 점씩 빼고 잰 오차: 사용한 점이 5개 미만이라 생략')

    if args.debug_out:
        # 초록 = 사용한 점, 빨강 = 제외된 점. 옆 숫자는 오차(mm)
        for (u, v), e, ok in zip(px_raw, err, inliers):
            color = (0, 200, 0) if ok else (0, 0, 255)
            cv2.circle(img, (int(round(u)), int(round(v))), 6, color, 2)
            cv2.putText(img, f'{e:.1f}', (int(u) + 8, int(v) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        cv2.imwrite(args.debug_out, img)
        print(f'확인용 그림: {args.debug_out}')


if __name__ == '__main__':
    main()
