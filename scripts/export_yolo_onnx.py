"""재학습한 YOLOv8-OBB (.pt) → vision_node 가 쓰는 모델 파일 (.onnx + .json). 4주차, 팀원 1.

    .venv-ml/bin/python scripts/export_yolo_onnx.py                # 내보내기 + 원본(.pt)과 비교할 기준값 저장
    python3 scripts/export_yolo_onnx.py --verify                   # ROS 환경(.venv, OpenCV 4.6)에서 기준값과 비교

OpenCV 4.6 이 못 읽는 마지막 상자 계산 부분을 잘라 낸다 (상자 계산은 vision_pick_place/yolo_detector.py).
기준값은 val 사진으로 만든다 (test 는 다시 쓰지 않는다). 모델은 저장소에 올리지 않는다. onnx 는 .venv-ml 에만 설치 (docs/SETUP.md Step 8).
"""
import argparse
import hashlib
import json
import shutil
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'vision_pick_place'))

from vision_pick_place.yolo_detector import OUTPUTS, YoloDetector  # noqa: E402

WEIGHTS = ROOT / 'data/models/runs/green/mvtec_n36_s0/weights/best.pt'
OUT = ROOT / 'data/models/yolo_green.onnx'
IMAGES = ROOT / 'data/captures/green/val'
INPUT_SIZE = (1024, 576)      # 1280×720 사진을 긴 변 1024 로 줄인 크기. 학습·채점(imgsz 1024)과 같다


def reference_path(out: Path) -> Path:
    return out.with_name(out.stem + '_reference.json')


def export(args):
    import onnx
    from ultralytics import YOLO
    with tempfile.TemporaryDirectory() as tmp:
        copy = Path(shutil.copy(args.weights, Path(tmp) / 'model.pt'))    # 원본 폴더에 중간 파일을 남기지 않는다
        model = YOLO(str(copy))
        if model.task != 'obb':
            raise ValueError(f'회전 상자(obb) 모델이 아닙니다: {model.task}')
        full = model.export(format='onnx', imgsz=INPUT_SIZE[::-1], opset=12, simplify=False, dynamic=False)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        onnx.utils.extract_model(str(full), str(args.out), ['images'], list(OUTPUTS))
    names = [model.names[i] for i in range(len(model.names))]
    args.out.with_suffix('.json').write_text(json.dumps(dict(
        names=names, input_size=list(INPUT_SIZE), weights=str(args.weights.resolve().relative_to(ROOT)) if args.weights.resolve().is_relative_to(ROOT) else args.weights.name,
        weights_sha256=hashlib.sha256(args.weights.read_bytes()).hexdigest()), indent=1))
    model = YOLO(str(args.weights))
    reference = {}
    for image in sorted(args.images.glob('*.png')):
        r = model.predict(cv2.imread(str(image)), imgsz=max(INPUT_SIZE), conf=args.conf, device='cpu', verbose=False,
                          agnostic_nms=True)[0]
        reference[image.name] = [dict(part_type=names[int(k)], conf=float(s), corners=c.tolist()) for c, k, s in
                                 zip(r.obb.xyxyxyxy.numpy(), r.obb.cls.numpy(), r.obb.conf.numpy())]
    reference_path(args.out).write_text(json.dumps(dict(conf=args.conf, images=str(args.images), objects=reference)))
    print('저장:', args.out, f'({len(reference)}장 기준값 {sum(map(len, reference.values()))}개)')


def verify(args) -> bool:
    ref = json.loads(reference_path(args.out).read_text())
    detector = YoloDetector(args.out, ref['conf'])
    center, corner, conf, times, problems, count = [0.0], [0.0], [0.0], [], [], 0
    for name, expected in ref['objects'].items():
        frame = cv2.imread(str(Path(ref['images']) / name))
        start = time.perf_counter()
        found = detector.detect(frame)
        times.append((time.perf_counter() - start) * 1000)
        if len(found) != len(expected):
            problems.append(f'{name}: 개수 {len(found)} (기준 {len(expected)})')
        left = list(found)
        for e in expected:
            c = np.array(e['corners'])
            if not left:
                break
            o = min(left, key=lambda o: np.hypot(*(np.array(o.det.center_px) - c.mean(0))))
            left = [x for x in left if x is not o]
            count += 1
            center.append(float(np.hypot(*(np.array(o.det.center_px) - c.mean(0)))))
            # 같은 상자라도 시작 꼭짓점이 다를 수 있어 가장 가까운 꼭짓점끼리 비교
            corner.append(float(np.linalg.norm(c[:, None] - o.det.box_px[None], axis=2).min(1).max()))
            conf.append(abs(o.conf - e['conf']))
            if o.part_type != e['part_type']:
                problems.append(f"{name}: 종류 {o.part_type} (기준 {e['part_type']})")
    print(f'OpenCV {cv2.__version__}: {count}개 비교, 최대 차이 중심 {max(center):.3f} px · 꼭짓점 {max(corner):.3f} px · '
          f'신뢰도 {max(conf):.4f}, 시간 중앙값 {np.median(times):.0f} ms')
    if max(corner) > 0.5 or max(conf) > 0.01:
        problems.append('차이가 허용값(꼭짓점 0.5 px, 신뢰도 0.01)보다 큽니다.')
    for p in problems:
        print('다름:', p)
    return not problems


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--weights', type=Path, default=WEIGHTS)
    parser.add_argument('--out', type=Path, default=OUT)
    parser.add_argument('--images', type=Path, default=IMAGES)
    parser.add_argument('--conf', type=float, default=0.3)
    parser.add_argument('--verify', action='store_true', help='내보내지 않고 기준값과 비교만')
    args = parser.parse_args()
    if not args.verify:
        export(args)
    sys.exit(0 if verify(args) else 1)


if __name__ == '__main__':
    main()
