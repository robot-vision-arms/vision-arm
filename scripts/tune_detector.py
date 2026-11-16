"""검증 split의 F1으로 전처리 설정 선택. test split 평가와 분리해서 실행한다.

PYTHONPATH=src/vision_pick_place:$PYTHONPATH python scripts/tune_detector.py \
  --validation data/mvtec_screws/mvtec_screws_val.json --images data/mvtec_screws/images
"""
import argparse
import json
from pathlib import Path

import cv2

from evaluate_detector import evaluate


def candidates():
    for closing, opening in [(0, 0), (3, 3), (5, 3), (3, 5)]:
        yield dict(segmentation_mode='gray', close_ksize=closing, open_ksize=opening,
                   hsv_max_saturation=70, hsv_max_dark_value=70)
    for saturation, dark in [(50, 50), (70, 50), (90, 50), (50, 70),
                              (70, 70), (90, 70), (70, 90), (90, 90)]:
        yield dict(segmentation_mode='metal_hsv', close_ksize=3, open_ksize=3,
                   hsv_max_saturation=saturation, hsv_max_dark_value=dark)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--validation', required=True, help='설정 선택에 사용할 validation 어노테이션')
    parser.add_argument('--images', required=True)
    parser.add_argument('--out', default='logs/week2_preprocessing_validation.json')
    args = parser.parse_args()
    cv2.setNumThreads(1)
    annotations = json.loads(Path(args.validation).read_text())
    reports = []
    for config in candidates():
        report = evaluate(annotations, args.images, threshold=100, blur_ksize=5,
                          min_area_px=500, **config)
        reports.append(report)
        print(f'{config}: P={report["precision"]:.4f}, R={report["recall"]:.4f}, F1={report["f1"]:.4f}', flush=True)
    best = max(reports, key=lambda report: report['f1'])
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(dict(validation=args.validation, selection_metric='F1 at rotated IoU >= 0.5',
                                     selected=best, candidates=reports), indent=2, ensure_ascii=False) + '\n')
    print(f'선택 결과: {json.dumps(best, ensure_ascii=False)}', flush=True)


if __name__ == '__main__':
    main()
