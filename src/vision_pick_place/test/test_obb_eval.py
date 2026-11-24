"""재학습 채점(obb_eval)과 학습 계획(scripts/finetune_obb.py) 확인. ultralytics 없이."""
import importlib.util
from pathlib import Path

import cv2
import numpy as np
import pytest

from vision_pick_place import obb_eval as E
from vision_pick_place import obb_labels as L


def box(kind, cx, cy, w=40, h=10, angle=0, conf=1.0):
    return dict(part_type=kind, corners=L.Box(kind, cx, cy, w, h, angle).corners(), conf=conf)


def test_perfect_prediction():
    truth = {'a': [box('screw', 50, 50, angle=170), box('nut', 100, 50, 14, 14)], 'b': []}
    s = E.score(truth, truth)
    assert (s['images'], s['correct'], s['missed'], s['false_positives'], s['f1']) == (2, 2, 0, 0, 1.0)
    assert s['center_px'] == 0 and s['angle_deg'] == pytest.approx(0, abs=1e-3) and s['center_mm'] is None


def test_each_error_kind_is_counted_once():
    truth = {'a': [box('screw', 50, 50), box('nut', 100, 50, 14, 14), box('nut', 150, 50, 14, 14)]}
    preds = {'a': [box('screw', 52, 50, angle=179), box('screw', 100, 50, 14, 14), box('nut', 300, 300, 14, 14)]}
    s = E.score(preds, truth)
    assert (s['located'], s['correct'], s['wrong_type'], s['missed'], s['false_positives']) == (2, 1, 1, 1, 1)
    assert s['precision'] == pytest.approx(1 / 3) and s['recall'] == pytest.approx(1 / 3)
    assert s['center_px'] == pytest.approx(1.0) and s['angle_deg'] == pytest.approx(1.0, abs=0.01)


def test_merged_box_over_two_parts_is_not_a_hit():
    """11-20: 붙은 너트 2개를 한 상자로 묶은 것을 정답으로 세었던 실수를 막는다."""
    truth = {'a': [box('nut', 100, 100, 14, 14), box('nut', 100, 116, 14, 14)]}
    s = E.score({'a': [box('nut', 100, 108, 16, 32)]}, truth)
    assert (s['correct'], s['missed'], s['false_positives']) == (0, 2, 1)


def test_duplicate_box_counts_as_false_positive_and_mm_error():
    truth = {'a': [box('screw', 50, 50)]}
    preds = {'a': [box('screw', 50, 50), box('nut', 51, 50)]}
    s = E.score(preds, truth, {'a': np.diag([0.5, 0.5, 1.0])})
    assert (s['correct'], s['false_positives']) == (1, 1) and s['center_mm'] == 0


def test_conf_is_chosen_in_middle_of_best_range():
    truth = {'a': [box('screw', 50, 50)]}
    preds = {'a': [box('screw', 50, 50, conf=0.8), box('nut', 200, 200, 14, 14, conf=0.3)]}
    assert 0.5 <= E.choose_conf(preds, truth) <= 0.6     # 0.35~0.80 이 모두 최고


def test_roi_filter():
    mask = np.zeros((100, 100), np.uint8)
    mask[:, 50:] = 255
    assert E.inside(box('nut', 70, 50, 14, 14)['corners'], mask) and not E.inside(box('nut', 20, 50, 14, 14)['corners'], mask)
    assert not E.inside(box('nut', 500, 50, 14, 14)['corners'], mask) and E.inside(box('nut', 500, 50)['corners'], None)


@pytest.fixture
def script(tmp_path, monkeypatch):
    path = Path(__file__).resolve().parents[3] / 'scripts' / 'finetune_obb.py'
    spec = importlib.util.spec_from_file_location('finetune_obb', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for name in module.INITS:
        (tmp_path / f'{name}.pt').write_bytes(name.encode())
    monkeypatch.setattr(module, 'INITS', {n: tmp_path / f'{n}.pt' for n in module.INITS})
    return module


def dataset(root, n_train=30):
    for split, n in (('train', n_train), ('val', 3)):
        (root / split).mkdir(parents=True)
        for i in range(n):
            path = root / split / f'{split}{i:02d}.png'
            cv2.imwrite(str(path), np.full((50, 80, 3), [i, n, 7], np.uint8))
            L.save_label(path, (80, 50), [L.Box('screw', 40, 25, 30, 8, i)], True)
    L.export_dataset(root, root.parent / 'out')
    return root.parent / 'out'


def test_plan_same_updates_nested_subsets_and_capped_size(script, tmp_path):
    data = dataset(tmp_path / 'green')
    runs = script.plan_runs(data, ['mvtec', 'dota'], [10, 20, 40, 100], [0], 400, 1024)
    assert [r['name'] for r in runs] == ['mvtec_n10_s0', 'dota_n10_s0', 'mvtec_n20_s0', 'dota_n20_s0', 'mvtec_n30_s0', 'dota_n30_s0']
    assert all(400 <= r['updates'] < 400 * 1.02 for r in runs)          # 사진 수가 달라도 갱신 횟수가 같다
    by = {r['name']: set(r['subset']) for r in runs}
    assert by['mvtec_n10_s0'] < by['mvtec_n20_s0'] < by['mvtec_n30_s0'] and by['mvtec_n10_s0'] == by['dota_n10_s0']
    assert all(script.label_file(p).is_file() for p in by['mvtec_n30_s0'])


def test_label_change_makes_run_unfinished(script, tmp_path):
    data = dataset(tmp_path / 'green', 5)
    run = script.plan_runs(data, ['mvtec'], [5], [0], 10, 320)[0]
    (tmp_path / 'runs' / run['name']).mkdir(parents=True)
    (tmp_path / 'runs' / run['name'] / 'done.json').write_text(f'{{"fingerprint": "{run["fingerprint"]}"}}')
    assert script.finished(tmp_path / 'runs', run)
    script.label_file(run['subset'][0]).write_text('')
    assert not script.finished(tmp_path / 'runs', script.plan_runs(data, ['mvtec'], [5], [0], 10, 320)[0])
