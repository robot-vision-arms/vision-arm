"""라벨한 사진에서 나사·너트 크기·구멍을 재서 unknown 거르기 기준값을 제안한다 (4주차, 팀원 1). ROS 없이 실행.

    python3 scripts/tune_part_limits.py data/captures/green
    python3 scripts/tune_part_limits.py data/captures/green --margin 0.15

기준값은 train 사진으로만 정하고, val 사진에서 진짜 부품을 잘못 버린 수와 기타 물체를 거른 수를 보여 준다 (test 는 안 본다).
기타 물체 = 규칙이 잡았지만 라벨에 없는 것. 붙은 부품처럼 라벨과 어긋난 상자는 빼고 따로 센다.
출력된 값을 params_green.yaml 에 옮겨 적는다. 확인 완료한 라벨만 쓴다.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src' / 'vision_pick_place'))

from vision_pick_place import obb_eval, obb_labels  # noqa: E402
from vision_pick_place.kpi import load_vision_params  # noqa: E402
from vision_pick_place.part_check import RANGE_PARAMS, measure, reject_reason  # noqa: E402
from vision_pick_place.vision_detector import classify_part  # noqa: E402

PARAMS = [ROOT / 'src/vision_pick_place/config/params.yaml', ROOT / 'src/vision_pick_place/config/params_green.yaml']
MIN_SAMPLES = 10      # 종류마다 이보다 적으면 기준값을 믿기 어렵다


def collect(root: Path, split: str, params: dict) -> dict:
    """{'screw': [Features], 'nut': [...], 'other': [(규칙이 본 종류, Features)], 'unclear': 개수, 'wrong_rule': 개수}"""
    out = dict(screw=[], nut=[], other=[], unclear=0, wrong_rule=0, photos=0)
    for image in obb_labels.list_images(root / split):
        label = obb_labels.load_label(image)
        if label is None or not label['reviewed']:
            print(f'건너뜀 (확인 완료 아님): {split}/{image.name}')
            continue
        mask, detections, H, _ = obb_labels.detect(cv2.imread(str(image)), params)
        if H is None:
            print(f'건너뜀 (마커를 못 찾아 mm 를 잴 수 없음): {split}/{image.name}')
            continue
        out['photos'] += 1
        for det in detections:
            kind = classify_part(det, params.get('nut_max_aspect', 1.5))
            overlaps = [(obb_eval.iou(det.box_px, b.corners()), b.part_type) for b in label['boxes']]
            best = max(overlaps, default=(0.0, None))
            f = measure(mask, det, obb_labels.mm_mapper(H))
            if best[0] >= obb_eval.IOU_MIN and sum(v > 0.1 for v, _ in overlaps) == 1:
                if best[1] == kind:
                    out[kind].append(f)
                else:
                    out['wrong_rule'] += 1     # 규칙이 종류를 틀린 진짜 부품. 기준값으로 고칠 수 없다
            elif best[0] < 0.05:
                out['other'].append((kind, f))
            else:
                out['unclear'] += 1            # 붙은 부품, 어긋난 상자
    return out


def suggest(train: dict, margin: float) -> dict:
    limits = {'nut_min_hole_ratio': 0.0}
    for name in RANGE_PARAMS:
        kind, side, _ = name.split('_')
        values = [getattr(f, f'{side}_mm') for f in train[kind]]
        if values:
            limits[name] = (round(min(values) * (1 - margin), 1), round(max(values) * (1 + margin), 1))
    holes = [f.hole_ratio for f in train['nut']]
    if holes and min(holes) >= 0.04:          # 구멍이 안 보이는 너트가 있으면 구멍으로 거르면 안 된다
        limits['nut_min_hole_ratio'] = round(min(holes) / 2, 3)
    return limits


def result(data: dict, limits: dict) -> dict:
    return dict(lost={k: sum(reject_reason(k, f, limits) is not None for f in data[k]) for k in ('screw', 'nut')},
                removed=sum(reject_reason(k, f, limits) is not None for k, f in data['other']))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('root', type=Path, help='label_obb 로 라벨한 폴더 (train, val)')
    parser.add_argument('--params', nargs='+', type=Path, default=PARAMS)
    parser.add_argument('--margin', type=float, default=0.10, help='train 최소·최대에서 더 넓힐 비율')
    args = parser.parse_args()
    params = load_vision_params([str(p) for p in args.params])
    data = {s: collect(args.root, s, params) for s in ('train', 'val')}
    train = data['train']
    if not train['screw'] and not train['nut']:
        sys.exit('train 에서 규칙이 맞게 잡은 나사·너트가 없습니다.')
    for kind in ('screw', 'nut'):
        fs = train[kind]
        if fs:
            a = np.array([[f.length_mm, f.width_mm, f.hole_ratio] for f in fs])
            lo, hi = a.min(0), a.max(0)
            print(f'{kind:5s} {len(fs):3d}개  길이 {lo[0]:.1f}~{hi[0]:.1f} mm  폭 {lo[1]:.1f}~{hi[1]:.1f} mm  구멍비 {lo[2]:.2f}~{hi[2]:.2f}')
        if len(fs) < MIN_SAMPLES:
            print(f'주의: {kind} 표본이 {len(fs)}개뿐입니다 ({MIN_SAMPLES}개 이상 권장).')
    limits = suggest(train, args.margin)
    if train['nut'] and limits['nut_min_hole_ratio'] == 0:
        print('구멍이 거의 안 보이는 너트가 있어 구멍 기준은 끕니다 (켜면 진짜 너트를 버립니다).')
    for split, d in data.items():
        r = result(d, limits)
        print(f"{split}: 사진 {d['photos']}장, 잘못 버린 나사 {r['lost']['screw']}/{len(d['screw'])}, 너트 {r['lost']['nut']}/{len(d['nut'])}, "
              f"거른 기타 물체 {r['removed']}/{len(d['other'])}, 판단 제외 {d['unclear']}, 규칙이 종류를 틀린 부품 {d['wrong_rule']}")
    if not data['val']['photos']:
        print('주의: val 사진이 없어 새 사진에서 어떤지 확인하지 못했습니다.')
    print('\n# params_green.yaml 의 vision_node.ros__parameters 에 넣을 값\n    reject_unknown: true')
    for name in RANGE_PARAMS:
        low, high = limits.get(name, (0.0, 0.0))
        print(f'    {name}: [{low}, {high}]')
    print(f"    nut_min_hole_ratio: {limits['nut_min_hole_ratio']}")


if __name__ == '__main__':
    main()
