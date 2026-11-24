"""우리 사진으로 YOLOv8n-OBB 미세조정과 비교 (4주차, 팀원 1). 라벨은 gui/label_obb.py 로 만들어 --export 한 것.

    .venv-ml/bin/python scripts/finetune_obb.py train    --data data/models/green_yolo_obb --dry-run   # 계획·예상 시간만
    .venv-ml/bin/python scripts/finetune_obb.py train    --data data/models/green_yolo_obb
    .venv-ml/bin/python scripts/finetune_obb.py evaluate --data data/models/green_yolo_obb                  # 검증 사진
    .venv-ml/bin/python scripts/finetune_obb.py evaluate --data data/models/green_yolo_obb --split test     # 마지막에 한 번

train: 시작 가중치(mvtec, dota) × 사진 수(10, 20, 40) 를 차례로 학습. 끝난 것은 건너뛴다 (데이터·설정이 바뀌면 다시 학습).
  - 사진 수가 달라도 가중치 갱신 횟수(--iters)를 같게 한다. epoch 를 같게 하면 사진이 적을수록 덜 배워서 "사진 수 효과" 와 섞인다.
  - 적은 사진은 많은 사진의 부분집합이다 (10 ⊂ 20 ⊂ 40). 사진이 모자라면 있는 만큼만 쓰고 실제 장수로 적는다.
evaluate: 규칙, 재학습 전 MVTec 모델, 학습한 모델들을 같은 채점(obb_eval)으로 비교.
  - 모두 같은 시트 ROI 안만 본다. YOLO 신뢰도 기준은 항상 검증(val) 사진에서 고른다.
  - test 는 설정을 다 정한 뒤 한 번만. test_new = 학습에 없던 부품만 놓은 사진.
결과: data/models/runs/green/<시작>_n<장수>_s<seed>/, 비교표 data/models/runs/green/eval_<split>.csv·json. git 제외.
"""
import argparse
import csv
import hashlib
import json
import math
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'vision_pick_place'))

from vision_pick_place import aruco_calib, obb_eval, obb_labels  # noqa: E402
from vision_pick_place.kpi import load_vision_params  # noqa: E402
from vision_pick_place.workspace_roi import sheet_roi_mask  # noqa: E402

RUNS = ROOT / 'data/models/runs'
INITS = {'mvtec': RUNS / 'yolo_obb/weights/best.pt', 'dota': RUNS / 'yolov8n-obb.pt'}
MVTEC_NUT = {6, 8, 9, 10}     # 0 부터 센 번호 (type_007, 009, 010, 011). notebooks/model_comparison.py 와 같음
PARAMS = [ROOT / 'src/vision_pick_place/config/params.yaml', ROOT / 'src/vision_pick_place/config/params_green.yaml']
TRAIN_ARGS = dict(
    optimizer='AdamW', lr0=0.001,   # 'auto' 는 총 반복 수에 따라 바뀔 수 있어 고정 (비교 조건을 같게)
    batch=8, nbs=8,                 # nbs 기본 64 면 배치 8번에 한 번만 갱신한다. 사진이 적을 때 갱신이 10번 남짓이 된다
    degrees=180.0, flipud=0.5, fliplr=0.5, mosaic=1.0,   # train_yolo_obb.py 와 같은 증강
    workers=4, device='cpu', deterministic=True, plots=False, patience=0)
SECONDS_PER_IMAGE = 0.71            # MVTec 실측: 269장 1 epoch 약 3.2분 (i5-12400, imgsz 1024, 다른 프로그램 없이)


def images_of(data: Path, split: str) -> list[Path]:
    return obb_labels.list_images(data / 'images' / split)


def label_file(image: Path) -> Path:
    return image.parents[2] / 'labels' / image.parent.name / (image.stem + '.txt')


def pick_subset(images: list[Path], n: int, seed: int) -> list[Path]:
    """seed 로 섞은 앞 n 장. n 이 커지면 앞의 것을 포함한다."""
    order = sorted(images)
    random.Random(seed).shuffle(order)
    return sorted(order[:n])


def fingerprint(images: list[Path], settings: dict) -> str:
    h = hashlib.md5(json.dumps(settings, sort_keys=True, default=str).encode())
    for image in images:
        h.update(image.name.encode() + image.read_bytes() + label_file(image).read_bytes())
    return h.hexdigest()


def plan_runs(data: Path, inits, sizes, seeds, iters, imgsz) -> list[dict]:
    train, val = images_of(data, 'train'), images_of(data, 'val')
    if not train or not val:
        raise ValueError(f'{data} 에 train/val 사진이 없습니다. label_obb --export 를 먼저 하세요.')
    runs = []
    for seed in seeds:
        for n in sorted({min(s, len(train)) for s in sizes}):
            subset = pick_subset(train, n, seed)
            for init in inits:
                batches = math.ceil(n / TRAIN_ARGS['batch'])
                epochs = math.ceil(iters / batches)
                settings = dict(init=init, weights=hashlib.md5(INITS[init].read_bytes()).hexdigest(),
                                epochs=epochs, imgsz=imgsz, seed=seed, **TRAIN_ARGS)
                runs.append(dict(name=f'{init}_n{n}_s{seed}', init=init, n=n, seed=seed, epochs=epochs, imgsz=imgsz,
                                 updates=epochs * batches, subset=subset, fingerprint=fingerprint(subset + val, settings),
                                 minutes=epochs * n * SECONDS_PER_IMAGE * (imgsz / 1024) ** 2 / 60))
    return runs


def finished(out: Path, run: dict) -> bool:
    done = out / run['name'] / 'done.json'
    return done.is_file() and json.loads(done.read_text())['fingerprint'] == run['fingerprint']


def train(args):
    data = args.data.resolve()
    runs = plan_runs(data, args.init, args.sizes, args.seeds, args.iters, args.imgsz)
    todo = [r for r in runs if not finished(args.out, r)]
    for r in runs:
        print(f"{r['name']:16s} 사진 {r['n']:3d}장  epoch {r['epochs']:4d}  갱신 {r['updates']}회  "
              + (f"예상 {r['minutes']:.0f}분" if r in todo else '이미 끝남'))
    print(f'남은 학습 {len(todo)}개, 예상 약 {sum(r["minutes"] for r in todo) / 60:.1f}시간 (검증 시간 제외, Gazebo 등을 끈 상태 기준)')
    if args.dry_run:
        return
    from ultralytics import YOLO
    for r in todo:
        run_dir = args.out.resolve() / r['name']
        run_dir.mkdir(parents=True, exist_ok=True)
        (run_dir / 'done.json').unlink(missing_ok=True)
        listing = run_dir / 'train.txt'
        listing.write_text('\n'.join(str(p) for p in r['subset']) + '\n')
        yaml = run_dir / 'dataset.yaml'
        yaml.write_text(f'path: {data}\ntrain: {listing}\nval: images/val\nnames:\n'
                        + ''.join(f'  {i}: {c}\n' for i, c in enumerate(obb_labels.CLASSES)))
        for cache in (data / 'labels').glob('*.cache'):
            cache.unlink()
        print(f"=== {r['name']} 학습 시작", flush=True)
        start = time.time()
        YOLO(str(INITS[r['init']])).train(data=str(yaml), project=str(args.out.resolve()), name=r['name'], exist_ok=True,
                                          epochs=r['epochs'], imgsz=r['imgsz'], seed=r['seed'], **TRAIN_ARGS)
        (run_dir / 'done.json').write_text(json.dumps(dict(
            fingerprint=r['fingerprint'], init=r['init'], n=r['n'], seed=r['seed'], epochs=r['epochs'], imgsz=r['imgsz'],
            updates=r['updates'], minutes=(time.time() - start) / 60, train_images=[p.name for p in r['subset']]), indent=1))


# --- 평가 ---
def load_split(data: Path, split: str, params: dict) -> dict:
    """사진, 정답 상자, 시트 ROI, px→mm. 사진 속 마커로 구한다. 마커를 못 찾으면 사진 전체에서 채점하고 알린다."""
    frames, truths, rois, to_mm, excluded = {}, {}, {}, {}, 0
    for image in images_of(data, split):
        frame = cv2.imread(str(image))
        h, w = frame.shape[:2]
        try:
            H, _, _ = aruco_calib.sheet_homography(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
            rois[image.name] = sheet_roi_mask(obb_labels.SheetMap(H), (w, h), params.get('sheet_edge_margin_mm', 3.0),
                                              params.get('sheet_marker_margin_mm', 5.0))
            to_mm[image.name] = H
        except ValueError:
            print(f'주의: {image.name} 에서 마커를 못 찾아 사진 전체로 채점합니다 (mm 오차 제외)')
            rois[image.name] = None
        boxes = []
        for line in label_file(image).read_text().splitlines():
            v = line.split()
            boxes.append(dict(part_type=obb_labels.CLASSES[int(v[0])], corners=np.array(v[1:], float).reshape(4, 2) * [w, h]))
        truths[image.name] = [b for b in boxes if obb_eval.inside(b['corners'], rois[image.name])]
        excluded += len(boxes) - len(truths[image.name])
        frames[image.name] = frame
    if excluded:
        print(f'주의: {split} 정답 {excluded}개가 시트 검출 영역 밖(마커·가장자리 여백)이라 모든 방식에서 뺐습니다.')
    return dict(frames=frames, truths=truths, rois=rois, to_mm=to_mm)


def rule_preds(d: dict, params: dict):
    out, times = {}, []
    for name, frame in d['frames'].items():
        start = time.perf_counter()
        boxes, _ = obb_labels.draft_boxes(frame, params)
        times.append((time.perf_counter() - start) * 1000)
        out[name] = [dict(corners=b.corners(), part_type=b.part_type, conf=1.0) for b in boxes]
    return out, float(np.median(times))


def yolo_preds(model, d: dict, imgsz: int, to_type):
    out, times = {}, []
    model.predict(next(iter(d['frames'].values())), imgsz=imgsz, device='cpu', verbose=False)   # 첫 실행 준비
    for name, frame in d['frames'].items():
        start = time.perf_counter()
        # agnostic_nms: 같은 물체에 나사·너트 상자가 겹쳐 나오지 않게 (11-20 실물에서 발생)
        r = model.predict(frame, imgsz=imgsz, conf=0.05, device='cpu', verbose=False, agnostic_nms=True)[0]
        times.append((time.perf_counter() - start) * 1000)
        out[name] = [p for p in (dict(corners=c, part_type=to_type(int(k)), conf=float(s)) for c, k, s in
                                 zip(r.obb.xyxyxyxy.numpy(), r.obb.cls.numpy(), r.obb.conf.numpy()))
                     if obb_eval.inside(p['corners'], d['rois'][name])]
    return out, float(np.median(times))


def draw(d: dict, preds: dict, folder: Path):
    """정답 초록, 예측 빨강(나사)·파랑(너트)."""
    folder.mkdir(parents=True, exist_ok=True)
    for name, frame in d['frames'].items():
        img = frame.copy()
        for t in d['truths'][name]:
            cv2.polylines(img, [np.rint(t['corners']).astype(np.int32)], True, (0, 220, 0), 1)
        for p in preds.get(name, []):
            color = (60, 60, 255) if p['part_type'] == 'screw' else (255, 150, 50)
            cv2.polylines(img, [np.rint(p['corners']).astype(np.int32)], True, color, 2)
            x, y = np.rint(np.mean(p['corners'], axis=0)).astype(int)
            cv2.putText(img, f"{p['part_type']} {p['conf']:.2f}", (int(x) + 8, int(y) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        cv2.imwrite(str(folder / (Path(name).stem + '.jpg')), img)


def evaluate(args):
    from ultralytics import YOLO
    data, params = args.data.resolve(), load_vision_params([str(p) for p in args.params])
    val = load_split(data, 'val', params)
    target = val if args.split == 'val' else load_split(data, args.split, params)
    if not target['frames']:
        raise ValueError(f'{args.split} 사진이 없습니다.')
    rows = []

    def add(name, preds, ms, conf=None, **extra):
        rows.append(dict(method=name, conf=conf, ms=ms, **extra, **obb_eval.score(preds, target['truths'], target['to_mm'])))
        if args.draw:
            draw(target, preds, args.out / f'draw_{args.split}' / name)

    preds, ms = rule_preds(target, params)
    add('rule', preds, ms)
    models = [('mvtec_asis', INITS['mvtec'], 1024, lambda k: 'nut' if k in MVTEC_NUT else 'screw', {})]
    for done_path in sorted(args.out.glob('*/done.json')):
        done = json.loads(done_path.read_text())
        planned = plan_runs(data, [done['init']], [done['n']], [done['seed']], args.iters, done['imgsz'])
        if planned[0]['fingerprint'] != done['fingerprint']:
            print(f'건너뜀: {done_path.parent.name} (학습 뒤 데이터나 설정, --iters 가 바뀜. train 을 다시 하세요)')
            continue
        models.append((done_path.parent.name, done_path.parent / 'weights/best.pt', done['imgsz'],
                       lambda k: obb_labels.CLASSES[k], dict(init=done['init'], n=done['n'], seed=done['seed'])))
    for name, weights, imgsz, to_type, extra in models:
        model = YOLO(str(weights))
        on_val, ms = yolo_preds(model, val, imgsz, to_type)
        conf = obb_eval.choose_conf(on_val, val['truths'])
        if target is not val:
            preds, ms = yolo_preds(model, target, imgsz, to_type)
        else:
            preds = on_val
        add(name, obb_eval.filter_conf(preds, conf), ms, conf, **extra)

    fmt = lambda v, f: '-' if v is None else format(v, f)
    print(f"\n{args.split}: 사진 {rows[0]['images']}장, 부품 {rows[0]['truths']}개"
          + ('  (conf 를 고른 사진이라 YOLO 점수는 실제보다 좋게 나온다)' if args.split == 'val' else ''))
    print(f"{'방식':18s} {'맞힘':>8s} {'종류틀림':>5s} {'놓침':>4s} {'오검출':>4s} {'F1':>6s} {'중심px':>6s} {'중심mm':>6s} {'각도':>6s} {'ms':>5s} {'conf':>5s}")
    for r in rows:
        print(f"{r['method']:18s} {r['correct']:4d}/{r['truths']:<4d} {r['wrong_type']:7d} {r['missed']:6d} {r['false_positives']:6d} "
              f"{r['f1']:7.3f} {fmt(r['center_px'], '7.2f')} {fmt(r['center_mm'], '7.2f')} {fmt(r['angle_deg'], '6.2f')} {r['ms']:6.0f} {fmt(r['conf'], '5.2f')}")
    if rows[0]['truths'] < 100:
        print(f"부품이 {rows[0]['truths']}개뿐이라 한두 개 차이는 우열로 보지 않는다. 라벨 초안이 규칙에서 나와 규칙에 약간 유리할 수 있다.")
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / f'eval_{args.split}.json').write_text(json.dumps(rows, indent=1))
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(args.out / f'eval_{args.split}.csv', 'w', newline='') as f:
        writer = csv.DictWriter(f, keys)
        writer.writeheader()
        writer.writerows(rows)
    print('저장:', args.out / f'eval_{args.split}.csv')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    for name, func in (('train', train), ('evaluate', evaluate)):
        p = sub.add_parser(name)
        p.set_defaults(func=func)
        p.add_argument('--data', type=Path, required=True, help='label_obb --export 폴더')
        p.add_argument('--out', type=Path, default=RUNS / 'green')
        p.add_argument('--iters', type=int, default=400, help='학습 하나의 가중치 갱신 횟수 (사진 수와 무관하게 같게)')
        if name == 'train':
            p.add_argument('--init', nargs='+', choices=sorted(INITS), default=['mvtec', 'dota'])
            p.add_argument('--sizes', nargs='+', type=int, default=[10, 20, 40])
            p.add_argument('--seeds', nargs='+', type=int, default=[0])
            p.add_argument('--imgsz', type=int, default=1024)
            p.add_argument('--dry-run', action='store_true', help='계획과 예상 시간만')
        else:
            p.add_argument('--split', choices=('val', 'test', 'test_new'), default='val')
            p.add_argument('--params', nargs='+', type=Path, default=PARAMS, help='규칙 검출 설정 (흰 시트는 params.yaml 하나만)')
            p.add_argument('--draw', action='store_true', help='정답·예측을 그린 사진도 저장')
    args = parser.parse_args()
    try:
        args.func(args)
    except ValueError as e:
        sys.exit(str(e))


if __name__ == '__main__':
    main()
