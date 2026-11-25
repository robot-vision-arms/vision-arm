"""실측 정답 자세 수집 (4주차, 팀원 3). PROJECT.md 6.1.

정해진 자세 목록(기본 30개: 나사 24 = 0~165° 를 15° 간격으로 2번씩, 너트 6)을 하나씩 화면에 띄우고,
부품을 그 자세에 놓은 뒤 s 로 저장한다. 위치는 모두 10 mm 격자점이라 부품 중심은 시트에 인쇄된 격자 교차점에 맞춘다.

    python scripts/collect_ground_truth.py --rotate 270 --method overlay
    python scripts/collect_ground_truth.py --print-plan            # 자세 목록만 출력
    python scripts/collect_ground_truth.py --print-sheet data/ground_truth/pose_sheet.pdf   # print 방법용 인쇄 시트

화면 (live): 노란 십자/원 = 목표 중심, 파란 선 = 목표 장축 방향(나사), 오른쪽 위 = 지금 검출된 물체 수.
  s: 저장 (사진 + 정답 행 + 그 사진의 검출 결과)   n / p: 다음 / 이전 자세   q / ESC: 종료
  예측값·오차는 일부러 보여 주지 않는다. 보면서 놓으면 예측에 맞춰 놓게 되어 정답이 오염된다.
  시트 이동 감지: 화면의 ArUco 마커가 homography 로 예상한 자리에서 --max-shift-px 넘게 벗어나면
  화면에 SHEET MOVED 를 띄우고 저장을 막는다 (시트를 다시 찍어 homography 를 새로 만들어야 함).

--method
  overlay : 인쇄 없이 화면 안내선에 맞춰 놓음. 안내선은 homography 로 그리므로 정답이 보정에 의존한다
            (보정 오차가 KPI 에 덜 잡힘). 임시 수집용.
  print   : 같은 자세 목록을 인쇄한 시트의 선 위에 놓음. 정답이 보정과 독립. 최종 KPI 용.

출력 (--out-dir, 기본 data/ground_truth/<method>_<날짜>/):
  poses.csv  sample_id,image,x_mm,y_mm,angle_deg,part_type      (PROJECT.md 형식, 너트 angle_deg 는 빈칸)
  preds.csv  sample_id,n_detected,part_type,x_mm,y_mm,angle_deg  (저장한 사진을 vision_node 와 같은 처리로 검출)
  images/<sample_id>.png  회전만 적용한 원본 프레임
같은 sample_id 를 다시 저장하면 그 행만 바뀐다. 이미 저장한 자세는 시작할 때 건너뛴다.
"""
import argparse
import csv
import datetime as dt
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/vision_pick_place'))
from vision_pick_place import aruco_calib  # noqa: E402
from vision_pick_place.calibrator import Calibrator  # noqa: E402
from vision_pick_place.camera_stream import FrameSource  # noqa: E402
from vision_pick_place.kpi import load_vision_params  # noqa: E402
from vision_pick_place.vision_pipeline import process_frame  # noqa: E402
from vision_pick_place.workspace_roi import sheet_roi_mask  # noqa: E402

# 마커(귀퉁이 12~47 mm)와 여백을 피하고, 길이 40 mm 나사가 어느 각도로 놓여도 검출 영역 안에 들도록 고른 격자점
XS = (80, 120, 150, 180, 220)
YS = (50, 90, 130, 170)
GUIDE_LEN_MM = 40.0
POSE_FIELDS = ['sample_id', 'image', 'x_mm', 'y_mm', 'angle_deg', 'part_type']
PRED_FIELDS = ['sample_id', 'n_detected', 'part_type', 'x_mm', 'y_mm', 'angle_deg']


def pose_plan(n_screw_rounds: int = 2, n_nuts: int = 6) -> list[dict]:
    """항상 같은 목록을 만든다 (overlay·print 두 방법이 같은 자세를 쓰도록)."""
    positions = [(x, y) for y in YS for x in XS]
    plan = []
    n_screws = 12 * n_screw_rounds
    for k in range(n_screws + n_nuts):
        x, y = positions[(k * 7) % len(positions)]  # 7 과 20 은 서로소 → 위치가 고루 섞임
        if k < n_screws:
            plan.append(dict(part_type='screw', x_mm=x, y_mm=y, angle_deg=(15 * k) % 180))
        else:
            x, y = positions[(k * 7 + 3) % len(positions)]
            plan.append(dict(part_type='nut', x_mm=x, y_mm=y, angle_deg=None))
    for i, p in enumerate(plan, 1):
        p['sample_id'] = f'gt{i:03d}'
    return plan


def read_rows(path: Path) -> dict:
    if not path.exists():
        return {}
    with path.open(newline='') as f:
        return {r['sample_id']: r for r in csv.DictReader(f)}


def write_rows(path: Path, fields: list, rows: dict) -> None:
    tmp = path.with_suffix('.tmp')
    with tmp.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for key in sorted(rows):
            w.writerow(rows[key])
    tmp.replace(path)


def fmt(v):
    return '' if v is None else (f'{v:.3f}' if isinstance(v, float) else v)


def draw_guide(img, calib, pose):
    c = calib.mm_to_px(np.array([[pose['x_mm'], pose['y_mm']]]))[0]
    ci = tuple(np.rint(c).astype(int))
    cv2.drawMarker(img, ci, (0, 220, 255), cv2.MARKER_CROSS, 24, 2)
    if pose['part_type'] == 'screw':
        a = math.radians(pose['angle_deg'])
        h = GUIDE_LEN_MM / 2
        ends_mm = np.array([[pose['x_mm'] - h * math.cos(a), pose['y_mm'] - h * math.sin(a)],
                            [pose['x_mm'] + h * math.cos(a), pose['y_mm'] + h * math.sin(a)]])
        e = np.rint(calib.mm_to_px(ends_mm)).astype(int)
        cv2.line(img, tuple(e[0]), tuple(e[1]), (255, 80, 0), 2)
    else:
        r = np.linalg.norm(calib.mm_to_px(np.array([[pose['x_mm'] + 8, pose['y_mm']]]))[0] - c)
        cv2.circle(img, ci, int(r), (0, 220, 255), 2)


def sheet_shift_px(frame, calib) -> float | None:
    """보이는 시트 마커 꼭짓점이 homography 로 예상한 위치에서 벗어난 최대 거리(px). 마커가 안 보이면 None."""
    corners, ids = aruco_calib.detect_markers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return None
    shifts = [np.linalg.norm(c - calib.mm_to_px(aruco_calib.sheet_marker_corners_mm(int(i))), axis=1).max()
              for c, i in zip(corners, ids) if int(i) in aruco_calib.SHEET_MARKERS]
    return max(shifts) if shifts else None


GREEN_BGR = (51, 153, 26)        # Gazebo 작업 매트와 같은 초록 (RGB 0.10, 0.60, 0.20)
GREEN_LINE_BGR = (0, 90, 0)      # 초록 시트의 선·글자: 채도가 높고 너무 어둡지 않아 metal_hsv 에서 배경
GREEN_INSET_MM = 6.0             # 프린터가 가장자리 몇 mm 를 못 칠하므로 초록은 이만큼 안쪽부터 (바깥 흰 띠)
GREEN_QUIET_MM = 8.0             # 초록 바탕에서는 마커 흰 여백이 4 mm 면 ArUco 가 마커를 버린다 (DECISIONS 12절)


def render_green_base(px_per_mm: float):
    """초록 작업 시트 바탕: 흰 종이 + 안쪽 초록 + 마커 (흰 여백 8 mm). 10 mm 격자는 그리지 않는다 (회색이면 부품으로 잡힘)."""
    width, height = (round(v * px_per_mm) for v in aruco_calib.SHEET_MM)
    img = np.full((height, width, 3), 255, np.uint8)
    inset = round(GREEN_INSET_MM * px_per_mm)
    img[inset:height - inset, inset:width - inset] = GREEN_BGR
    side, quiet = round(aruco_calib.SHEET_MARKER_MM * px_per_mm), round(GREEN_QUIET_MM * px_per_mm)
    for marker_id, (x, y) in aruco_calib.SHEET_MARKERS.items():
        col, row = round(x * px_per_mm), round((aruco_calib.SHEET_MM[1] - y - aruco_calib.SHEET_MARKER_MM) * px_per_mm)
        img[max(row - quiet, 0):row + side + quiet, max(col - quiet, 0):col + side + quiet] = 255
        img[row:row + side, col:col + side] = cv2.cvtColor(aruco_calib.marker_image(marker_id, side), cv2.COLOR_GRAY2BGR)
    return img


def render_pose_sheet(path: Path, plan: list, px_per_mm: float = 12.0, green: bool = False) -> None:
    """작업 시트(마커·10 mm 격자) 위에 자세 목록을 옅은 선으로 그린 A4 가로 PDF/PNG. 100% 배율로 인쇄한다.

    선은 회색(170)이라 이진화(Otsu, 오늘 임계값 약 120)에서 배경으로 처리된다. 부품을 선 위에 놓으면 실측값 = 그린 값.
    green=True: 초록 바탕 (그림자를 부품으로 잡지 않게, WEEK2 동경 11-20). 선·글자는 진한 초록이라 metal_hsv 에서 배경이다.
    검출 설정은 params.yaml 위에 config/params_green.yaml 을 겹쳐 쓴다 (metal_hsv, 마커 여백 9 mm, 가장자리 여백 8 mm).
    """
    img = render_green_base(px_per_mm) if green else cv2.cvtColor(aruco_calib.render_sheet(px_per_mm), cv2.COLOR_GRAY2BGR)
    h = img.shape[0]

    def px(x, y):
        return int(round(x * px_per_mm)), int(round(h - y * px_per_mm))
    gray, w = (GREEN_LINE_BGR if green else (170, 170, 170)), max(1, round(0.25 * px_per_mm))
    for q in plan:
        x, y = q['x_mm'], q['y_mm']
        cv2.drawMarker(img, px(x, y), gray, cv2.MARKER_CROSS, round(4 * px_per_mm), w)
        if q['part_type'] == 'screw':
            a, L = math.radians(q['angle_deg']), GUIDE_LEN_MM / 2
            cv2.line(img, px(x - L * math.cos(a), y - L * math.sin(a)), px(x + L * math.cos(a), y + L * math.sin(a)), gray, w)
            lx, ly = x + (L + 2) * math.cos(a), y + (L + 2) * math.sin(a)
        else:
            cv2.circle(img, px(x, y), round(8 * px_per_mm), gray, w)
            lx, ly = x + 9, y + 3
        cv2.putText(img, f"{q['sample_id'][2:]}", px(lx, ly), cv2.FONT_HERSHEY_SIMPLEX, 0.12 * px_per_mm, gray, w)
    # 인쇄 배율 확인용 100 mm 선 (자로 재서 100 mm 인지 확인)
    scale = GREEN_LINE_BGR if green else (120, 120, 120)
    cv2.line(img, px(60, 8), px(160, 8), scale, w)
    for xe in (60, 160):
        cv2.line(img, px(xe, 6.5), px(xe, 9.5), scale, w)
    cv2.putText(img, f"pose sheet{' (green)' if green else ''}: print at 100% (line = 100 mm), A4 landscape", px(165, 7),
                cv2.FONT_HERSHEY_SIMPLEX, 0.12 * px_per_mm, scale, w)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(path.with_suffix('.png')), img)
    if path.suffix.lower() == '.pdf':
        from PIL import Image
        Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB)).save(path, resolution=px_per_mm * 25.4)


def detect(frame, calib, roi, params):
    packet, _ = process_frame(frame, calib, 0, time.time(), roi_mask=roi, **params)
    return packet['objects']


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--url', default=None, help='기본: params.yaml 의 camera_url')
    p.add_argument('--rotate', type=int, default=None, help='기본: params.yaml 의 camera_rotate_deg')
    p.add_argument('--method', choices=('overlay', 'print'), default='overlay')
    p.add_argument('--out-dir', default=None)
    p.add_argument('--params', nargs='+', default=[str(ROOT / 'src/vision_pick_place/config/params.yaml')],
                   help='vision_node 설정 YAML, 여러 개면 뒤 파일이 덮음 (초록 시트: params.yaml params_green.yaml)')
    p.add_argument('--print-plan', action='store_true')
    p.add_argument('--print-sheet', default=None, help='자세를 그린 인쇄용 시트 저장 경로 (.pdf 면 PNG 도 같이)')
    p.add_argument('--green', action='store_true', help='--print-sheet 를 초록 바탕으로 (그림자 대책, config/params_green.yaml 과 함께)')
    p.add_argument('--max-shift-px', type=float, default=3.0, help='마커가 이보다 많이 벗어나면 저장 막음')
    p.add_argument('--only', default='', help='이 자세만 수집 (예: gt002,gt013). 다시 찍을 때')
    args = p.parse_args()

    plan = pose_plan()
    if args.only:
        wanted = {s.strip() for s in args.only.split(',') if s.strip()}
        unknown = wanted - {q['sample_id'] for q in plan}
        if unknown:
            sys.exit(f'없는 sample_id: {sorted(unknown)}')
        plan = [q for q in plan if q['sample_id'] in wanted]
    if args.print_plan:
        for q in plan:
            print(q['sample_id'], q['part_type'], q['x_mm'], q['y_mm'], q['angle_deg'])
        return
    if args.print_sheet:
        render_pose_sheet(Path(args.print_sheet), plan, green=args.green)
        print('저장:', args.print_sheet)
        return

    vp = load_vision_params(args.params)
    url = args.url or vp['camera_url']
    rotate = vp.get('camera_rotate_deg', 0) if args.rotate is None else args.rotate
    calib = Calibrator(str(ROOT / vp['intrinsics_path']), str(ROOT / vp['homography_path']))
    if not calib.status.ready:
        sys.exit('보정 파일이 없습니다 (intrinsics.npz, homography.npz).')
    det_params = {k: vp[k] for k in ('blur_ksize', 'threshold', 'min_area_px', 'nut_max_aspect',
                                     'segmentation_mode', 'hsv_max_saturation', 'hsv_max_dark_value',
                                     'close_ksize', 'open_ksize') if k in vp}

    out = Path(args.out_dir) if args.out_dir else ROOT / f'data/ground_truth/{args.method}_{dt.date.today():%Y%m%d}'
    (out / 'images').mkdir(parents=True, exist_ok=True)
    poses, preds = read_rows(out / 'poses.csv'), read_rows(out / 'preds.csv')
    print(f'저장 폴더: {out}  (이미 저장 {len(poses)}/{len(plan)})  회전 {rotate}  방법 {args.method}', flush=True)

    src = FrameSource(camera_url=url, rotate_deg=rotate)
    roi = None
    idx = next((i for i, q in enumerate(plan) if q['sample_id'] not in poses), 0)
    win = 'ground truth  (s: save, n: next, p: prev, q: quit)'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    try:
        while True:
            frame = src.read()
            if frame is None:
                print('프레임을 읽지 못했습니다. 다시 시도합니다.', flush=True)
                continue
            if calib.image_size is not None and calib.image_size != (frame.shape[1], frame.shape[0]):
                sys.exit(f'해상도 {frame.shape[1]}x{frame.shape[0]} 가 보정 {calib.image_size} 와 다릅니다. --rotate 를 확인하세요.')
            if roi is None:
                roi = sheet_roi_mask(calib, (frame.shape[1], frame.shape[0]),
                                     vp.get('sheet_edge_margin_mm', 3.0), vp.get('sheet_marker_margin_mm', 5.0))
            pose = plan[idx]
            n_det = len(detect(frame, calib, roi, det_params))
            view = frame.copy()
            draw_guide(view, calib, pose)
            ang = '-' if pose['angle_deg'] is None else f"{pose['angle_deg']} deg"
            done = 'SAVED' if pose['sample_id'] in poses else ''
            cv2.putText(view, f"{pose['sample_id']} ({idx + 1}/{len(plan)})  {pose['part_type']}  "
                        f"x={pose['x_mm']} y={pose['y_mm']}  angle={ang}  {done}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(view, f'detected: {n_det}', (1100, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 160, 0) if n_det == 1 else (0, 0, 255), 2)
            cv2.putText(view, f'saved {len(poses)}/{len(plan)}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            shift = sheet_shift_px(frame, calib)
            moved = shift is None or shift > args.max_shift_px
            if moved:
                msg = 'NO MARKERS' if shift is None else f'SHEET MOVED {shift:.0f}px - redo homography, saving blocked'
                cv2.putText(view, msg, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 3)
            cv2.imshow(win, view)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord('q'), 27):
                break
            if k == ord('n'):
                idx = (idx + 1) % len(plan)
            if k == ord('p'):
                idx = (idx - 1) % len(plan)
            if k == ord('s') and moved:
                print(f'저장 안 함: 시트 마커가 보정 위치에서 {shift if shift is not None else "?"} px 벗어남. '
                      '시트를 다시 찍어 homography 를 새로 만드세요.', flush=True)
            elif k == ord('s'):
                sid = pose['sample_id']
                rel = f'images/{sid}.png'
                cv2.imwrite(str(out / rel), frame)
                objs = detect(frame, calib, roi, det_params)
                # 여러 개면 목표 위치에 가장 가까운 것 (예측값은 화면에 보여 주지 않음)
                best = min(objs, key=lambda o: math.hypot(o['x_mm'] - pose['x_mm'], o['y_mm'] - pose['y_mm']),
                           default=None)
                poses[sid] = dict(sample_id=sid, image=rel, x_mm=pose['x_mm'], y_mm=pose['y_mm'],
                                  angle_deg=fmt(pose['angle_deg']), part_type=pose['part_type'])
                preds[sid] = dict(sample_id=sid, n_detected=len(objs),
                                  part_type=best['part_type'] if best else '',
                                  x_mm=fmt(best and best['x_mm']), y_mm=fmt(best and best['y_mm']),
                                  angle_deg=fmt(best and best['angle_deg']))
                write_rows(out / 'poses.csv', POSE_FIELDS, poses)
                write_rows(out / 'preds.csv', PRED_FIELDS, preds)
                print(f'{sid} 저장 (검출 {len(objs)}개)', flush=True)
                nxt = next((i for i in range(idx + 1, idx + 1 + len(plan))
                            if plan[i % len(plan)]['sample_id'] not in poses), None)
                if nxt is None:
                    print('전부 저장했습니다.', flush=True)
                else:
                    idx = nxt % len(plan)
    finally:
        src.release()
        cv2.destroyAllWindows()
    print(f'저장 {len(poses)}/{len(plan)} → {out}')


if __name__ == '__main__':
    main()
