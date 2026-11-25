"""규칙+SVM 을 우리 초록 시트 사진에서 규칙·YOLO 와 같은 채점으로 비교 (4주차, 팀원 1). 결과는 docs/WEEK3.md.

    .venv-ml/bin/python scripts/compare_svm.py --data data/models/green_yolo_obb                # val
    .venv-ml/bin/python scripts/compare_svm.py --data data/models/green_yolo_obb --split test   # 설정을 정한 뒤 한 번

규칙이 찾은 상자를 그대로 두고 나사/너트 분류만 SVM 으로 바꾼다 (notebooks/model_comparison 의 B-SVM 과 같은 특징 22개).
  svm_mvtec : MVTec train 정답 상자로 학습 (13종 → 나사/너트), 설정은 MVTec val. 우리 사진을 하나도 안 씀
  svm_green : 우리 train 정답 상자로 학습 (나사·너트 2종), 설정은 우리 train 안에서 사진 단위 5겹 교차검증
분류만 바뀌므로 '맞힘' 은 규칙이 찾은 수(located)를 넘을 수 없다. 정답 상자로 분류만 한 정확도도 같이 낸다.
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'src' / 'vision_pick_place'))

import finetune_obb as ft  # noqa: E402
from vision_pick_place import obb_eval  # noqa: E402
from vision_pick_place.camera_stream import preprocess  # noqa: E402
from vision_pick_place.kpi import load_vision_params  # noqa: E402

GRID = [dict(C=c, gamma=g) for c in (1, 10, 100) for g in ('scale', 0.01)]      # 노트북과 같은 후보
MVTEC_MASK = dict(blur_ksize=5, threshold=-1, segmentation_mode='metal_hsv', hsv_max_saturation=70, hsv_max_dark_value=90,
                  close_ksize=3, open_ksize=3)                                   # 노트북 A-hsv
MASK_KEYS = ('blur_ksize', 'threshold', 'segmentation_mode', 'hsv_max_saturation', 'hsv_max_dark_value', 'close_ksize', 'open_ksize')


def shape_features(img, rect, mask_options):
    """notebooks/model_comparison.py 의 shape_features 와 같은 22개 (크기, 모양, Hu 모멘트, 색)."""
    (cx, cy), (w, h), ang = rect
    if w < h:
        w, h, ang = h, w, ang + 90
    M = cv2.getRotationMatrix2D((cx, cy), ang, 1.0)
    W, H = int(w * 1.1) + 12, int(h * 1.3) + 12
    M[:, 2] += (W / 2 - cx, H / 2 - cy)
    crop = cv2.warpAffine(img, M, (W, H), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT)
    mask = preprocess(crop, **mask_options) > 0
    f = [w, h, math.log(w), w / max(h, 1), w * h]
    cnts, hier = cv2.findContours(mask.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if cnts:
        k = int(np.argmax([cv2.contourArea(c) for c in cnts]))
        c, area = cnts[k], cv2.contourArea(cnts[k])
        holes = [a for a in (cv2.contourArea(cnts[i]) for i in range(len(cnts)) if hier[0][i][3] == k) if a > 0.02 * area]
        hu = cv2.HuMoments(cv2.moments(c)).ravel()
        hu = -np.sign(hu) * np.log10(np.abs(hu) + 1e-30)
        solidity, n_holes, ratio = area / max(cv2.contourArea(cv2.convexHull(c)), 1), len(holes), sum(holes) / max(area, 1)
    else:
        hu, solidity, n_holes, ratio = np.zeros(7), 0, 0, 0
    prof = mask.mean(axis=0)
    n = max(len(prof) // 6, 1)
    mid = prof[2 * n:-2 * n].mean() if len(prof) > 4 * n else prof.mean()
    f += [mask.mean(), solidity, n_holes, ratio, max(prof[:n].mean(), prof[-n:].mean()) / max(mid, 1e-3), *hu]
    hsv, gray = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV), cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    fg = mask if mask.any() else np.ones_like(mask)
    f += [hsv[..., 1][fg].mean(), hsv[..., 1][fg].std(), hsv[..., 2][fg].mean(), hsv[..., 2][fg].std(), gray[fg].std()]
    return np.array(f, np.float32)


def build(params):
    from sklearn.pipeline import make_pipeline
    from sklearn.preprocessing import StandardScaler
    from sklearn.svm import SVC
    return make_pipeline(StandardScaler(), SVC(kernel='rbf', **params))


def truth_features(d, options):
    X, y, groups = [], [], []
    for name, truths in d['truths'].items():
        for t in truths:
            X.append(shape_features(d['frames'][name], cv2.minAreaRect(np.float32(t['corners'])), options))
            y.append(t['part_type'])
            groups.append(name)
    return np.array(X), np.array(y), np.array(groups)


def train_mvtec(cache):
    """노트북이 저장한 MVTec 정답 상자 특징 (data/models/cache/feat_*_gt.npz) 으로 학습."""
    tr, va = (np.load(cache / f'feat_{s}_gt.npz') for s in ('train', 'val'))
    models = [(build(p).fit(tr['X'], tr['y']), p) for p in GRID]
    model, params = max(models, key=lambda m: (m[0].predict(va['X']) == va['y']).mean())
    return model, params, lambda c: 'nut' if int(c) in ft.MVTEC_NUT else 'screw'


def train_green(d, options):
    from sklearn.model_selection import GroupKFold, cross_val_score
    X, y, groups = truth_features(d, options)
    scores = [cross_val_score(build(p), X, y, groups=groups, cv=GroupKFold(5)).mean() for p in GRID]
    params = GRID[int(np.argmax(scores))]
    return build(params).fit(X, y), params, str, max(scores), len(y)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--split', choices=('val', 'test', 'test_new'), default='val')
    p.add_argument('--cache', type=Path, default=ROOT / 'data/models/cache')
    p.add_argument('--out', type=Path, default=ft.RUNS / 'green')
    args = p.parse_args()
    params = load_vision_params([str(x) for x in ft.PARAMS])
    green = {k: params[k] for k in MASK_KEYS}
    data = args.data.resolve()
    train, target = ft.load_split(data, 'train', params), ft.load_split(data, args.split, params)
    g_model, g_params, g_type, cv, n = train_green(train, green)
    m_model, m_params, m_type = train_mvtec(args.cache)
    print(f'svm_green: train 부품 {n}개, 교차검증 정확도 {cv:.3f}, {g_params} / svm_mvtec: {m_params}')
    rule, rule_ms = ft.rule_preds(target, params)
    rows = [dict(method='rule', ms=rule_ms, **obb_eval.score(rule, target['truths'], target['to_mm']))]
    X, y, _ = truth_features(target, green)
    Xm, _, _ = truth_features(target, MVTEC_MASK)
    ratio = np.where(X[:, 3] <= params['nut_max_aspect'], 'nut', 'screw')
    only = dict(rule=float((ratio == y).mean()))
    for name, model, to_type, options, feats in (('rule+svm_mvtec', m_model, m_type, MVTEC_MASK, Xm),
                                                 ('rule+svm_green', g_model, g_type, green, X)):
        preds, times = {}, []
        for image, boxes in rule.items():
            start = time.perf_counter()
            f = [shape_features(target['frames'][image], cv2.minAreaRect(np.float32(b['corners'])), options) for b in boxes]
            cls = model.predict(np.array(f)) if f else []
            times.append((time.perf_counter() - start) * 1000)
            preds[image] = [dict(b, part_type=to_type(c)) for b, c in zip(boxes, cls)]
        rows.append(dict(method=name, ms=rule_ms + float(np.median(times)), **obb_eval.score(preds, target['truths'], target['to_mm'])))
        only[name] = float(np.mean([to_type(c) == t for c, t in zip(model.predict(feats), y)]))
    print(f"{args.split}: 부품 {rows[0]['truths']}개. 위치를 맞힌 상자는 규칙과 같음 (최대 맞힘 {rows[0]['located']})")
    for r in rows:
        print(f"{r['method']:16s} 맞힘 {r['correct']:2d}/{r['truths']} 종류틀림 {r['wrong_type']} 놓침 {r['missed']} 오검출 {r['false_positives']} "
              f"F1 {r['f1']:.3f} {r['ms']:.0f} ms | 정답 상자로 분류만 {only[r['method']] * 100:.1f}%")
    (args.out / f'eval_svm_{args.split}.json').write_text(json.dumps(dict(rows=rows, class_only=only, svm_green=g_params, svm_mvtec=m_params), indent=1))


if __name__ == '__main__':
    main()
