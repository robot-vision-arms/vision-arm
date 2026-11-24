"""회전 상자 검출 채점 (재학습 비교용, 4주차 팀원 1). 노트북 model_comparison 과 같은 대응 규칙: 회전 IoU ≥ 0.5, IoU 큰 순서로 1:1.

상자 하나 = dict(corners=(4, 2) px, part_type, conf). 이미지별 목록을 dict 로 넘긴다.
"맞힘" 은 위치와 종류가 모두 맞은 것이다 (로봇이 쓰는 결과).
"""
import cv2
import numpy as np

from vision_pick_place.obb_labels import box_from_corners
from vision_pick_place.vision_detector import angle_error_deg

IOU_MIN = 0.5
ANGLE_MIN_ASPECT = 1.5     # 이보다 뭉툭한 정답(너트)은 각도를 재지 않는다


def iou(a, b) -> float:
    a, b = np.asarray(a, np.float32), np.asarray(b, np.float32)
    inter = float(cv2.intersectConvexConvex(a, b)[0])
    union = float(cv2.contourArea(a)) + float(cv2.contourArea(b)) - inter
    return inter / union if union > 0 else 0.0


def match(preds: list[dict], truths: list[dict]) -> list[tuple[int, int]]:
    """[(예측 번호, 정답 번호)]. 종류는 보지 않고 위치만 대응한다."""
    pairs = sorted(((iou(p['corners'], t['corners']), i, j) for i, p in enumerate(preds) for j, t in enumerate(truths)), reverse=True)
    out, used_p, used_t = [], set(), set()
    for value, i, j in pairs:
        if value >= IOU_MIN and i not in used_p and j not in used_t:
            used_p.add(i)
            used_t.add(j)
            out.append((i, j))
    return out


def inside(corners, roi_mask) -> bool:
    if roi_mask is None:
        return True
    x, y = np.rint(np.mean(corners, axis=0)).astype(int)
    h, w = roi_mask.shape
    return 0 <= x < w and 0 <= y < h and roi_mask[y, x] > 0


def score(preds_by_image: dict, truths_by_image: dict, px_to_mm: dict | None = None) -> dict:
    """truths_by_image 의 모든 사진을 채점한다 (예측이 없는 사진도). px_to_mm: 사진 → H (px → mm), 없으면 mm 오차는 None."""
    n_pred = n_truth = located = correct = 0
    center_px, center_mm, angles = [], [], []
    for name, truths in truths_by_image.items():
        preds = preds_by_image.get(name, [])
        n_pred, n_truth = n_pred + len(preds), n_truth + len(truths)
        for i, j in match(preds, truths):
            p, t = preds[i], truths[j]
            located += 1
            correct += p['part_type'] == t['part_type']
            pc, tc = np.mean(p['corners'], axis=0), np.mean(t['corners'], axis=0)
            center_px.append(float(np.hypot(*(pc - tc))))
            H = (px_to_mm or {}).get(name)
            if H is not None:
                mm = cv2.perspectiveTransform(np.array([[pc], [tc]], float), H).reshape(2, 2)
                center_mm.append(float(np.hypot(*(mm[0] - mm[1]))))
            tb, pb = box_from_corners('', t['corners']), box_from_corners('', p['corners'])
            if tb.w / max(tb.h, 1e-6) > ANGLE_MIN_ASPECT:
                angles.append(angle_error_deg(pb.angle_deg, tb.angle_deg))
    precision = correct / n_pred if n_pred else 0.0
    recall = correct / n_truth if n_truth else 0.0
    mean = lambda v: float(np.mean(v)) if v else None
    return dict(images=len(truths_by_image), truths=n_truth, preds=n_pred, located=located, correct=correct,
                wrong_type=located - correct, missed=n_truth - located, false_positives=n_pred - located,
                precision=precision, recall=recall,
                f1=2 * precision * recall / (precision + recall) if precision + recall else 0.0,
                center_px=mean(center_px), center_mm=mean(center_mm), angle_deg=mean(angles))


def filter_conf(preds_by_image: dict, conf: float) -> dict:
    return {k: [p for p in v if p['conf'] >= conf] for k, v in preds_by_image.items()}


def choose_conf(preds_by_image: dict, truths_by_image: dict, grid=None) -> float:
    """검증 사진에서 F1 이 가장 높은 신뢰도 기준. 같은 값이 이어지면 그 구간의 가운데 (끝값은 새 사진에서 불안정)."""
    grid = np.round(np.arange(0.05, 0.951, 0.05), 2) if grid is None else np.asarray(grid)
    f1 = np.array([score(filter_conf(preds_by_image, c), truths_by_image)['f1'] for c in grid])
    best = int(np.argmax(f1))
    end = best
    while end + 1 < len(grid) and f1[end + 1] == f1[best]:
        end += 1
    return float(grid[(best + end) // 2])
