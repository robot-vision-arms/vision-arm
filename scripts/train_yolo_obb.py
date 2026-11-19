"""MVTec Screws → YOLO OBB 데이터 변환과 YOLOv8n-OBB 학습 (모델 비교 방식 D, DECISIONS.md 11절).

학습은 CPU 로 오래 걸려서 노트북(notebooks/model_comparison.ipynb) 밖에서 이 스크립트로 돌리고,
노트북은 같은 함수로 데이터를 만들고 학습이 끝난 가중치·로그를 읽어 평가한다.

실행 (.venv-ml, SETUP.md Step 8):
    source .venv-ml/bin/activate
    python scripts/train_yolo_obb.py                 # 변환 + 학습
    python scripts/train_yolo_obb.py --convert-only  # 변환만

결과: data/models/mvtec_yolo_obb/ (데이터·dataset.yaml), data/models/runs/yolo_obb/ (가중치·학습 로그). git 제외.
"""
import argparse
import json
import math
import os
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
MVTEC = ROOT / 'data' / 'mvtec_screws'
OUT = ROOT / 'data' / 'models' / 'mvtec_yolo_obb'
RUNS = ROOT / 'data' / 'models' / 'runs'

# 학습 설정. 노트북도 이 값을 읽어 표에 적는다.
TRAIN_ARGS = dict(
    model='yolov8n-obb.pt',  # 가장 작은 OBB 모델 (DOTA 사전학습). CPU 라서 n 크기만 현실적
    imgsz=1024,              # 원본 1920×1440. 640 이면 가장 작은 너트(장축 약 66 px)가 22 px 로 줄어 1024 사용
    epochs=40,               # CPU 1 epoch 약 3.2분 (i5-12400, imgsz 1024 실측) → 약 2시간
    patience=10,             # val 성능이 10 epoch 동안 안 오르면 조기 종료
    batch=8,
    workers=4,
    device='cpu',
    degrees=180.0,           # 부품은 아무 방향으로나 놓이므로 회전 증강을 크게
    flipud=0.5,
    fliplr=0.5,
    mosaic=1.0,
    seed=0,
    deterministic=True,
)


def rect_from_bbox(bbox):
    """MVTec bbox [row, col, width, height, phi(rad, 반시계)] → OpenCV RotatedRect.

    scripts/evaluate_detector.py 와 같은 변환: 중심은 0.5 px 보정, 각도는 부호를 바꿔 도 단위.
    """
    row, col, width, height, phi = bbox
    return ((col - 0.5, row - 0.5), (width, height), -math.degrees(phi))


def convert(out: Path = OUT) -> Path:
    """MVTec train/val/test 분할을 YOLO OBB 형식으로. 이미지는 복사하지 않고 심볼릭 링크."""
    names = None
    for split in ('train', 'val', 'test'):
        data = json.loads((MVTEC / f'mvtec_screws_{split}.json').read_text())
        if names is None:
            names = {c['id']: c['name'] for c in data['categories']}
        img_dir, lbl_dir = out / 'images' / split, out / 'labels' / split
        img_dir.mkdir(parents=True, exist_ok=True)
        lbl_dir.mkdir(parents=True, exist_ok=True)
        by_image = {}
        for a in data['annotations']:
            by_image.setdefault(a['image_id'], []).append(a)
        for rec in data['images']:
            src = MVTEC / 'images' / rec['file_name']
            dst = img_dir / rec['file_name']
            if not dst.exists():
                os.symlink(src, dst)
            w, h = rec['width'], rec['height']
            lines = []
            for a in by_image.get(rec['id'], []):
                corners = cv2.boxPoints(rect_from_bbox(a['bbox']))
                xy = ' '.join(f'{x / w:.6f} {y / h:.6f}' for x, y in corners)
                lines.append(f"{a['category_id'] - 1} {xy}")   # YOLO 클래스 번호는 0 부터
            (lbl_dir / (Path(rec['file_name']).stem + '.txt')).write_text('\n'.join(lines) + '\n')
    yaml = out / 'dataset.yaml'
    yaml.write_text(f'path: {out}\ntrain: images/train\nval: images/val\ntest: images/test\nnames:\n'
                    + ''.join(f'  {cid - 1}: {name}\n' for cid, name in sorted(names.items())))
    return yaml


def train(yaml: Path, name: str = 'yolo_obb'):
    from ultralytics import YOLO
    RUNS.mkdir(parents=True, exist_ok=True)
    os.chdir(RUNS)                       # 사전학습 가중치가 여기로 내려받아진다 (저장소 루트를 어지럽히지 않게)
    args = dict(TRAIN_ARGS)
    model = YOLO(args.pop('model'))
    model.train(data=str(yaml), project=str(RUNS), name=name, exist_ok=True, plots=True, **args)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--convert-only', action='store_true')
    parser.add_argument('--epochs', type=int, help='시간 측정용으로 줄일 때')
    parser.add_argument('--name', default='yolo_obb')
    args = parser.parse_args()
    yaml = convert()
    print('dataset:', yaml)
    if args.convert_only:
        return
    if args.epochs:
        TRAIN_ARGS['epochs'] = args.epochs
    train(yaml, args.name)


if __name__ == '__main__':
    main()
