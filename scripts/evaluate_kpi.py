"""KPI 4종 + 선택 2종 계산 (4주차, docs/PROJECT.md 7장). 계산은 vision_pick_place/kpi.py.

    # 실물 정답 사진으로 위치·각도·분류 (사진마다 vision_node 와 같은 처리로 검출, 시차 보정 켠 값·끈 값 둘 다)
    PYTHONPATH=src/vision_pick_place python scripts/evaluate_kpi.py \\
        --truth data/ground_truth/print_20261127/poses.csv --per-image-homography
    # 송신·지연, Gazebo 집기까지 같이
    ... --comm logs/comm_20261127_*.csv --pick logs/sim_pick_*.csv --out logs/kpi_20261127.json

    --per-image-homography : 사진마다 시트 마커로 호모그래피를 다시 구한다 (촬영 중 시트가 움직였을 때).
                             없으면 --homography 하나로 전부 계산한다.
    --images-dir           : 사진을 poses.csv 의 image 경로 대신 이 폴더에서 같은 파일 이름으로 찾는다.
    --preds                : 이미 저장한 검출 결과 CSV (sample_id, part_type, x_mm, y_mm, angle_deg) 를 쓰면 사진 처리를 건너뛴다.

결과 표에는 측정 장비(CPU·OpenCV·해상도)를 같이 출력한다. 실측 전에는 달성으로 쓰지 않는다 (PROJECT.md 7장).
"""
import argparse
import glob
import json
import os
from pathlib import Path
import platform

import yaml

from vision_pick_place import kpi

LABELS = {
    'position_mae_mm': ('위치 MAE', 'mm', 2), 'angle_mae_deg': ('각도 MAE', '°', 2),
    'send_success_rate': ('송신 성공률', '%', 1), 'latency_p95_ms': ('검출→송신 P95', 'ms', 1),
    'classification_accuracy': ('분류 정확도', '%', 1), 'pick_success_rate': ('Pick 성공률', '%', 1),
}


def show(name, value):
    if value is None:
        return '—'
    _, unit, digits = LABELS[name]
    return f'{value * 100:.{digits}f} %' if unit == '%' else f'{value:.{digits}f} {unit}'


def expand(patterns):
    files = sorted({f for p in patterns or [] for f in glob.glob(p)})
    if patterns and not files:
        raise SystemExit(f'파일이 없습니다: {patterns}')
    return files


def environment(params_path, truth_rows, image_for):
    cpu = next((line.split(':', 1)[1].strip() for line in Path('/proc/cpuinfo').read_text().splitlines()
                if line.startswith('model name')), platform.processor()) if Path('/proc/cpuinfo').exists() else ''
    info = dict(cpu=cpu, python=platform.python_version(), ros=os.environ.get('ROS_DISTRO', ''), params=str(params_path))
    try:
        import cv2
        info['opencv'] = cv2.__version__
        if truth_rows:
            frame = cv2.imread(str(image_for(truth_rows[0])))
            if frame is not None:
                info['image_size'] = f'{frame.shape[1]}x{frame.shape[0]}'
    except ImportError:
        pass
    return info


def parse_heights(text):
    """'screw=4,nut=3.25' → {'screw': 4.0, 'nut': 3.25}"""
    return {k: float(v) for k, v in (item.split('=') for item in text.split(',') if item)}


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--truth', help='정답 poses.csv (sample_id,image,x_mm,y_mm,angle_deg,part_type)')
    p.add_argument('--images-dir', help='사진을 이 폴더에서 파일 이름으로 찾는다')
    p.add_argument('--preds', help='저장된 검출 결과 CSV (사진 처리 대신)')
    p.add_argument('--params', default='src/vision_pick_place/config/params.yaml')
    p.add_argument('--intrinsics', default='data/calib/intrinsics.npz')
    p.add_argument('--homography', default='data/calib/homography.npz')
    p.add_argument('--per-image-homography', action='store_true')
    p.add_argument('--parallax', choices=['both', 'off', 'on'], default='both',
                   help='시차 보정 (calibrator.px_to_mm). both = 끈 값·켠 값 둘 다 계산')
    p.add_argument('--parallax-heights', default='screw=4,nut=3.25', help='켤 때 종류별 높이 mm')
    p.add_argument('--match-radius-mm', type=float, default=15.0)
    p.add_argument('--comm', nargs='*', help='robot_bridge logs/comm_*.csv (여러 개면 시간순으로 이어 붙임)')
    p.add_argument('--comm-count', type=int, default=100)
    p.add_argument('--pick', nargs='*', help='pick_place_node logs/sim_pick_*.csv')
    p.add_argument('--out', help='결과 JSON')
    args = p.parse_args()
    if not (args.truth or args.comm or args.pick):
        p.error('--truth, --comm, --pick 중 하나 이상 필요합니다.')

    params = yaml.safe_load(Path(args.params).read_text())['vision_node']['ros__parameters']
    truth_rows = kpi.read_csv(args.truth) if args.truth else []
    base = Path(args.truth).parent if args.truth else Path('.')

    def image_for(row):
        if args.images_dir:
            return Path(args.images_dir) / Path(row['image']).name
        return base / row['image']

    report = dict(environment=environment(args.params, truth_rows, image_for), pose={}, comm=None, pick=None)
    if truth_rows:
        if args.preds:
            by_sample = {}
            for r in kpi.read_csv(args.preds):
                by_sample.setdefault(r['sample_id'], []).append(r)
            variants = {'저장된 검출': by_sample}
        else:
            variants = {}
            heights = parse_heights(args.parallax_heights)
            for name, h in (('시차 보정 끔', None), ('시차 보정 켬', heights)):
                if args.parallax == 'both' or (args.parallax == 'on') == (h is not None):
                    variants[name] = kpi.predict_from_images(
                        truth_rows, image_for, args.intrinsics,
                        None if args.per_image_homography else args.homography, params, h)
        for name, by_sample in variants.items():
            report['pose'][name] = kpi.pose_metrics(truth_rows, by_sample, args.match_radius_mm)
    if args.comm:
        rows = [r for f in expand(args.comm) for r in kpi.read_csv(f)]
        report['comm'] = kpi.comm_metrics(rows, args.comm_count)
    if args.pick:
        report['pick'] = kpi.pick_metrics([r for f in expand(args.pick) for r in kpi.read_csv(f)])

    # ---- 출력 ----
    env = report['environment']
    print(f"측정 장비: {env.get('cpu', '')} | Python {env.get('python')} | OpenCV {env.get('opencv', '?')} "
          f"| ROS {env.get('ros') or '-'} | 사진 {env.get('image_size', '-')}")
    print(f"{'지표':<14}{'기준':<12}" + ''.join(f'{name:<18}' for name in report['pose']) + '판정')
    for metric in ('position_mae_mm', 'angle_mae_deg', 'classification_accuracy'):
        if not report['pose']:
            break
        target, direction, required = kpi.CRITERIA[metric]
        values = [m[metric] for m in report['pose'].values()]
        crit = f"{direction} {show(metric, target)}"
        print(f"{LABELS[metric][0]:<14}{crit:<12}" + ''.join(f'{show(metric, v):<18}' for v in values)
              + ' / '.join(kpi.judge(metric, v) for v in values) + ('' if required else ' (선택)'))
    for name, m in report['pose'].items():
        print(f"  [{name}] 검출 {m['detected']}/{m['samples']}, 위치 중앙 {m['position_median_mm']:.2f} 최대 "
              f"{m['position_max_mm']:.2f} mm, 치우침 dx {m['bias_dx_mm']:+.2f} dy {m['bias_dy_mm']:+.2f} mm, "
              f"각도 부호 평균 {m['angle_signed_mean_deg']:+.2f}° 표준편차 {m['angle_signed_std_deg']:.2f}° "
              f"(나사 {m['angle_samples']}개)" if m['detected'] else f"  [{name}] 검출 0/{m['samples']}")
    if report['comm']:
        c = report['comm']
        for metric in ('send_success_rate', 'latency_p95_ms'):
            target, direction, _ = kpi.CRITERIA[metric]
            print(f"{LABELS[metric][0]:<14}{direction + ' ' + show(metric, target):<12}{show(metric, c[metric]):<18}"
                  f"{kpi.judge(metric, c[metric])}")
        print(f"  송신 시도 {c['attempts']}/{c['requested']}, ACK {c['acked']}, 지연 중앙 "
              f"{c['latency_median_ms'] or 0:.1f} ms (표본 {c['latency_samples']})"
              + ('  [시도 수가 기준보다 적음]' if c['attempts'] < c['requested'] else ''))
    if report['pick']:
        k = report['pick']
        target, direction, _ = kpi.CRITERIA['pick_success_rate']
        print(f"{'Pick 성공률':<14}{direction + ' ' + show('pick_success_rate', target):<12}"
              f"{show('pick_success_rate', k['pick_success_rate']):<18}{kpi.judge('pick_success_rate', k['pick_success_rate'])} (선택)")
        print(f"  집기 {k['picked']}/{k['picks']}" + ('  [20개 미만]' if k['picks'] < 20 else ''))
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str))
        print('saved', args.out)


if __name__ == '__main__':
    main()
