"""KPI 계산 (4주차, docs/PROJECT.md 7장). ROS 없이 CSV·사진만으로 계산한다. scripts/evaluate_kpi.py 가 부른다.

    위치 MAE (mm)   : 예측 (x, y) 와 실측의 유클리드 거리 평균          기준 ≤ 2 mm   (필수)
    각도 MAE (deg)  : [0, 180) 정의 오차 평균, 정답 각도가 있는 나사만   기준 ≤ 3°     (필수)
    송신 성공률      : 처음 N(100)회 송신 시도 중 ACK 받은 비율          기준 100 %    (필수)
    검출→송신 P95   : ACK 받은 송신의 latency_ms 95 백분위수            기준 ≤ 300 ms (필수)
    분류 정확도      : part_type 예측 = 실측 비율                       기준 ≥ 95 %   (선택)
    Pick 성공률      : Gazebo 사이클에서 맞는 상자에 들어간 비율 (20개 이상) 기준 ≥ 90 % (선택)

정답과 예측의 대응: 같은 사진에서 정답 위치에 가장 가까운 검출 하나 (match_radius_mm 안). 못 찾으면 '미검출'.
"""
import csv
import math
from pathlib import Path

import numpy as np

from vision_pick_place.communication_check import percentile95
from vision_pick_place.vision_detector import angle_error_deg

CRITERIA = {   # 지표: (기준값, 방향, 필수 여부)
    'position_mae_mm': (2.0, '<=', True),
    'angle_mae_deg': (3.0, '<=', True),
    'send_success_rate': (1.0, '>=', True),
    'latency_p95_ms': (300.0, '<=', True),
    'classification_accuracy': (0.95, '>=', False),
    'pick_success_rate': (0.90, '>=', False),
}
TRUE_WORDS = {'true', '1', 'yes', 'ok'}


def _bool(value) -> bool:
    return str(value).strip().lower() in TRUE_WORDS


def _float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def load_vision_params(paths) -> dict:
    """vision_node 파라미터 YAML 여러 개를 순서대로 겹친다 (뒤 파일이 앞 값을 덮음). ros2 의 --params-file 여러 번과 같다.
    예: [params.yaml, params_green.yaml] = 초록 시트 설정."""
    import yaml
    merged = {}
    for path in ([paths] if isinstance(paths, (str, Path)) else paths):
        merged.update(yaml.safe_load(Path(path).read_text())['vision_node']['ros__parameters'])
    return merged


def read_csv(path) -> list[dict]:
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def match_detection(truth: dict, detections: list[dict], match_radius_mm: float = 15.0):
    """정답 하나에 가장 가까운 검출 (mm 좌표가 있는 것만). 반경 밖이면 None."""
    x, y = float(truth['x_mm']), float(truth['y_mm'])
    best, best_d = None, match_radius_mm
    for det in detections:
        dx, dy = _float(det.get('x_mm')), _float(det.get('y_mm'))
        if dx is None or dy is None:
            continue
        d = math.hypot(dx - x, dy - y)
        if d <= best_d:
            best, best_d = det, d
    return best


def pose_metrics(truth_rows: list[dict], detections_by_sample: dict, match_radius_mm: float = 15.0) -> dict:
    """위치·각도 MAE, 분류 정확도. detections_by_sample: sample_id → 그 사진의 검출 목록 (x_mm, y_mm, angle_deg, part_type)."""
    rows, pos, ang, signed, correct = [], [], [], [], 0
    for t in truth_rows:
        det = match_detection(t, detections_by_sample.get(t['sample_id'], []), match_radius_mm)
        row = dict(sample_id=t['sample_id'], part_type=t['part_type'], detected=det is not None)
        if det is not None:
            dx, dy = float(det['x_mm']) - float(t['x_mm']), float(det['y_mm']) - float(t['y_mm'])
            row.update(pred_type=det.get('part_type'), dx_mm=dx, dy_mm=dy, error_mm=math.hypot(dx, dy))
            pos.append(row['error_mm'])
            correct += det.get('part_type') == t['part_type']
            ta, pa = _float(t.get('angle_deg')), _float(det.get('angle_deg'))
            if ta is not None and pa is not None:
                row['angle_error_deg'] = angle_error_deg(pa, ta)
                ang.append(row['angle_error_deg'])
                signed.append((pa - ta + 90.0) % 180.0 - 90.0)
        rows.append(row)
    n = len(truth_rows)
    detected = [r for r in rows if r['detected']]
    return dict(
        samples=n, detected=len(detected),
        position_mae_mm=float(np.mean(pos)) if pos else None,
        position_median_mm=float(np.median(pos)) if pos else None,
        position_max_mm=float(np.max(pos)) if pos else None,
        bias_dx_mm=float(np.mean([r['dx_mm'] for r in detected])) if detected else None,
        bias_dy_mm=float(np.mean([r['dy_mm'] for r in detected])) if detected else None,
        angle_mae_deg=float(np.mean(ang)) if ang else None,
        angle_samples=len(ang),
        angle_signed_mean_deg=float(np.mean(signed)) if signed else None,
        angle_signed_std_deg=float(np.std(signed)) if signed else None,
        # 미검출은 틀린 것으로 센다 (분모 = 정답 수)
        classification_accuracy=correct / n if n else None,
        rows=rows)


def comm_metrics(comm_rows: list[dict], count: int = 100) -> dict:
    """robot_bridge 의 logs/comm_*.csv: 처음 count 번의 송신 시도로 성공률·P95. 시도가 count 보다 적으면 있는 만큼."""
    attempts = [r for r in comm_rows if _bool(r.get('attempted', 'True'))][:count]
    acked = [r for r in attempts if _bool(r.get('acked'))]
    latencies = [v for r in acked if _bool(r.get('latency_valid', 'True'))
                 and (v := _float(r.get('latency_ms'))) is not None]
    return dict(attempts=len(attempts), acked=len(acked), requested=count,
                send_success_rate=len(acked) / len(attempts) if attempts else None,
                latency_p95_ms=percentile95(latencies), latency_samples=len(latencies),
                latency_median_ms=float(np.median(latencies)) if latencies else None)


def pick_metrics(pick_rows: list[dict]) -> dict:
    """pick_place_node 의 logs/sim_pick_*.csv (물체 한 줄씩): result == ok 비율."""
    rows = [r for r in pick_rows if r.get('obj')]
    ok = sum(r.get('result') == 'ok' for r in rows)
    return dict(picks=len(rows), picked=ok, pick_success_rate=ok / len(rows) if rows else None)


def judge(name: str, value) -> str:
    """기준과 비교: '달성' / '미달' / '측정 안 함'."""
    if value is None:
        return '측정 안 함'
    target, direction, _ = CRITERIA[name]
    passed = value <= target if direction == '<=' else value >= target
    return '달성' if passed else '미달'


def predict_from_images(truth_rows: list[dict], image_for, intrinsics_path: str, homography_path: str | None,
                        params: dict, part_heights_mm: dict | None = None) -> dict:
    """정답 사진마다 vision_node 와 같은 처리(process_frame + 작업 시트 ROI)로 검출한다. sample_id → 검출 목록.

    homography_path 가 None 이면 사진마다 작업 시트 마커로 호모그래피를 다시 구한다 (촬영 중 시트가 움직인 경우).
    image_for: 정답 한 줄 → 사진 경로.
    """
    import tempfile

    import cv2

    from vision_pick_place import aruco_calib
    from vision_pick_place.calibrator import Calibrator
    from vision_pick_place.part_check import limits_from_params
    from vision_pick_place.vision_pipeline import process_frame
    from vision_pick_place.workspace_roi import sheet_roi_mask

    options = {k: params[k] for k in ('blur_ksize', 'threshold', 'min_area_px', 'nut_max_aspect', 'segmentation_mode',
                                      'hsv_max_saturation', 'hsv_max_dark_value', 'close_ksize', 'open_ksize')
               if k in params}
    out = {}
    with tempfile.TemporaryDirectory() as tmp:
        fixed = Calibrator(intrinsics_path, homography_path) if homography_path else None
        for t in truth_rows:
            path = image_for(t)
            frame = cv2.imread(str(path))
            if frame is None:
                raise FileNotFoundError(f'사진을 읽지 못했습니다: {path}')
            calib = fixed
            if calib is None:
                with np.load(intrinsics_path) as k:
                    K, dist = k['camera_matrix'], k['dist_coeffs']
                H, rms, _ = aruco_calib.sheet_homography(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), K, dist)
                np.savez(Path(tmp) / 'h.npz', H=H, reproj_err_mm=rms)
                calib = Calibrator(intrinsics_path, str(Path(tmp) / 'h.npz'))
            roi = sheet_roi_mask(calib, (frame.shape[1], frame.shape[0]),
                                 params.get('sheet_edge_margin_mm', 3.0), params.get('sheet_marker_margin_mm', 5.0)) \
                if params.get('sheet_roi', True) else None
            packet, _ = process_frame(frame, calib, 0, 0.0, roi_mask=roi, part_heights_mm=part_heights_mm,
                                      part_limits=limits_from_params(params), **options)
            out[t['sample_id']] = packet['objects']
    return out
