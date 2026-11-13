"""MVTec 회전 박스와 비교하는 픽셀 검출 평가. mm KPI·13종 분류 평가는 아님.

PYTHONPATH=src/vision_pick_place python scripts/evaluate_detector.py \
    --annotations data/mvtec_screws/mvtec_screws_test.json --images data/mvtec_screws/images

bbox는 README_v1.0.txt의 (row, col, width, height, phi[radian]) 정의를 따른다.
검출 박스와 정답 박스를 회전 IoU가 큰 순으로 일대일 대응한다.
"""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

import cv2
import numpy as np

from vision_pick_place.camera_stream import preprocess
from vision_pick_place.vision_detector import angle_error_deg, find_objects


def evaluate(annotations, images_dir, *, threshold=-1, blur_ksize=5, min_area_px=500, iou_threshold=0.5):
    by_image = defaultdict(list)
    for annotation in annotations['annotations']:
        by_image[annotation['image_id']].append(annotation)
    tp = predicted = expected = 0
    centers, angles = [], []
    for record in annotations['images']:
        path = Path(images_dir) / record['file_name']
        frame = cv2.imread(str(path))
        if frame is None:
            raise ValueError(f'이미지를 읽지 못했습니다: {path}')
        detections = find_objects(preprocess(frame, blur_ksize, threshold), min_area_px)
        truth = []
        for annotation in by_image[record['id']]:
            row, col, width, height, phi = annotation['bbox']
            rect = ((col - 0.5, row - 0.5), (width, height), -math.degrees(phi))
            truth.append((rect, cv2.boxPoints(rect)))
        predicted += len(detections)
        expected += len(truth)
        candidates = []
        for i, det in enumerate(detections):
            area = float(cv2.contourArea(det.box_px))
            for j, (rect, box) in enumerate(truth):
                intersection, _ = cv2.intersectConvexConvex(det.box_px, box)
                union = area + rect[1][0] * rect[1][1] - intersection
                iou = intersection / union if union > 0 else 0
                if iou >= iou_threshold:
                    candidates.append((iou, i, j))
        used_predictions, used_truth = set(), set()
        for _, i, j in sorted(candidates, reverse=True):
            if i in used_predictions or j in used_truth:
                continue
            used_predictions.add(i)
            used_truth.add(j)
            det, (rect, _) = detections[i], truth[j]
            centers.append(float(np.linalg.norm(np.asarray(det.center_px) - rect[0])))
            width, height = rect[1]
            if min(width, height) > 0 and max(width, height) / min(width, height) > 1.5:
                pred_angle = math.degrees(math.atan2(det.long_axis_px[1], det.long_axis_px[0]))
                true_angle = rect[2] + (90 if height > width else 0)
                angles.append(angle_error_deg(pred_angle, true_angle))
        tp += len(used_truth)
    return dict(images=len(annotations['images']), ground_truth=expected, predictions=predicted,
                matched=tp, precision=tp / predicted if predicted else 0,
                recall=tp / expected if expected else 0,
                center_mae_px=float(np.mean(centers)) if centers else None,
                angle_mae_deg=float(np.mean(angles)) if angles else None,
                angle_samples=len(angles), iou_threshold=iou_threshold,
                threshold=threshold, blur_ksize=blur_ksize, min_area_px=min_area_px,
                matching='greedy rotated IoU; errors on matched objects only')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--annotations', required=True)
    parser.add_argument('--images', required=True)
    parser.add_argument('--threshold', type=int, default=-1)
    parser.add_argument('--blur-ksize', type=int, default=5)
    parser.add_argument('--min-area-px', type=float, default=500)
    parser.add_argument('--iou', type=float, default=0.5)
    parser.add_argument('--out')
    args = parser.parse_args()
    if not 0 < args.iou <= 1:
        parser.error('--iou는 (0, 1] 범위여야 합니다.')
    cv2.setNumThreads(1)
    report = evaluate(json.loads(Path(args.annotations).read_text()), args.images,
                      threshold=args.threshold, blur_ksize=args.blur_ksize,
                      min_area_px=args.min_area_px, iou_threshold=args.iou)
    output = json.dumps(report, indent=2, ensure_ascii=False)
    print(output)
    if args.out:
        path = Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(output + '\n')


if __name__ == '__main__':
    main()
