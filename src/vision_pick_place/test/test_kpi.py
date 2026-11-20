"""KPI 계산 (kpi.py) 확인. 손으로 계산할 수 있는 작은 예로 정의를 고정한다."""
import pytest

from vision_pick_place import kpi


def truth(sid, part, x, y, angle=''):
    return dict(sample_id=sid, image=f'images/{sid}.png', x_mm=str(x), y_mm=str(y), angle_deg=str(angle), part_type=part)


def test_pose_metrics_position_angle_and_classification():
    rows = [truth('gt1', 'screw', 100, 100, 179), truth('gt2', 'nut', 50, 50), truth('gt3', 'screw', 200, 100, 30)]
    detections = {
        'gt1': [dict(part_type='screw', x_mm=103, y_mm=104, angle_deg=1.0),      # 5 mm, 179°↔1° = 2°
                dict(part_type='nut', x_mm=150, y_mm=100, angle_deg=None)],     # 먼 검출은 무시
        'gt2': [dict(part_type='screw', x_mm=50, y_mm=51, angle_deg=None)],     # 1 mm, 종류 틀림
        'gt3': [dict(part_type='screw', x_mm=260, y_mm=100, angle_deg=30.0)],   # 60 mm → 반경 밖 = 미검출
    }
    m = kpi.pose_metrics(rows, detections, match_radius_mm=15)
    assert m['detected'] == 2 and m['samples'] == 3
    assert m['position_mae_mm'] == pytest.approx(3.0)          # (5 + 1) / 2
    assert m['angle_mae_deg'] == pytest.approx(2.0) and m['angle_samples'] == 1
    assert m['angle_signed_mean_deg'] == pytest.approx(2.0)    # 1° - 179° → +2° (접은 부호 오차)
    assert m['classification_accuracy'] == pytest.approx(1 / 3)  # 미검출도 틀린 것으로 센다
    assert m['bias_dx_mm'] == pytest.approx(1.5) and m['bias_dy_mm'] == pytest.approx(2.5)


def test_comm_metrics_uses_first_n_attempts_and_valid_latency():
    rows = [dict(seq=i, attempted='True', acked='True', latency_ms=str(10 + i), latency_valid='True') for i in range(5)]
    rows[1].update(acked='False', latency_ms='')                     # 실패 1
    rows[2].update(latency_valid='False')                             # 시계 튐: 성공이지만 지연 표본에서 뺌
    rows.append(dict(seq=99, attempted='True', acked='False', latency_ms='', latency_valid='False'))  # count 밖
    m = kpi.comm_metrics(rows, count=5)
    assert (m['attempts'], m['acked']) == (5, 4)
    assert m['send_success_rate'] == pytest.approx(0.8)
    assert m['latency_samples'] == 3                                  # 10, 13, 14
    assert m['latency_p95_ms'] == pytest.approx(13.9)


def test_pick_metrics_counts_object_rows_only():
    rows = [dict(obj='obj_0', result='ok'), dict(obj='obj_1', result='failed'), dict(obj='', result='ok')]
    m = kpi.pick_metrics(rows)
    assert (m['picks'], m['picked']) == (2, 1) and m['pick_success_rate'] == 0.5


@pytest.mark.parametrize('name, value, expected', [
    ('position_mae_mm', 1.99, '달성'), ('position_mae_mm', 2.01, '미달'),
    ('send_success_rate', 1.0, '달성'), ('send_success_rate', 0.99, '미달'),
    ('angle_mae_deg', None, '측정 안 함')])
def test_judge_against_project_criteria(name, value, expected):
    assert kpi.judge(name, value) == expected


def test_load_vision_params_overlays_later_files(tmp_path):
    base, green = tmp_path / 'params.yaml', tmp_path / 'params_green.yaml'
    base.write_text('vision_node:\n  ros__parameters:\n    segmentation_mode: "gray"\n    min_area_px: 500\n')
    green.write_text('vision_node:\n  ros__parameters:\n    segmentation_mode: "metal_hsv"\n')
    merged = kpi.load_vision_params([base, green])
    assert merged == {'segmentation_mode': 'metal_hsv', 'min_area_px': 500}
    assert kpi.load_vision_params(str(base))['segmentation_mode'] == 'gray'
