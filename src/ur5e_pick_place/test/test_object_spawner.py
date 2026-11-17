import math
from pathlib import Path

import pytest

from ur5e_pick_place.grasp import PlanePose
from ur5e_pick_place.object_spawner import (SPAWN_LIFT_M, load_sim_params, model_file, plan_spawns,
                                            plane_from_params, slot_name)

PKG = Path(__file__).resolve().parents[1]
PLANE = PlanePose(0.30, -0.15, 0.0, 0.0)
TYPES = ['screw', 'nut']


def _packet(*objects, calib_ok=True):
    return {'seq': 7, 't_capture': 0.0, 'calib_ok': calib_ok, 'objects': list(objects)}


def _obj(i, part_type='screw', x=100.0, y=50.0, angle=30.0):
    return {'id': i, 'part_type': part_type, 'x_mm': x, 'y_mm': y, 'angle_deg': angle, 'u_px': 1.0, 'v_px': 2.0}


def test_screw_goes_to_plane_position_with_vision_angle():
    reqs, skipped = plan_spawns(_packet(_obj(4, x=100.0, y=50.0, angle=30.0)), PLANE, TYPES, 3)
    assert skipped == []
    (r,) = reqs
    assert (r.name, r.part_type, r.source_id) == ('obj_0', 'screw', 4)
    assert math.isclose(r.x, 0.40) and math.isclose(r.y, -0.10)
    assert math.isclose(r.z, SPAWN_LIFT_M)
    assert math.isclose(math.degrees(r.yaw_rad), 30.0)


def test_nut_without_angle_and_rotated_plane():
    plane = PlanePose(0.30, -0.15, 0.02, 90.0)
    (r,), _ = plan_spawns(_packet(_obj(0, 'nut', x=100.0, y=0.0, angle=None)), plane, TYPES, 3)
    assert math.isclose(r.x, 0.30, abs_tol=1e-9) and math.isclose(r.y, -0.05)
    assert math.isclose(r.z, 0.02 + SPAWN_LIFT_M)
    assert math.isclose(math.degrees(r.yaw_rad), 90.0)  # 각도 없음 → 0° + 평면 회전


def test_uncalibrated_packet_is_rejected():
    with pytest.raises(ValueError):
        plan_spawns(_packet(_obj(0), calib_ok=False), PLANE, TYPES, 3)


def test_unknown_missing_mm_and_extra_objects_are_skipped_slots_stay_contiguous():
    packet = _packet(_obj(0, 'unknown'), _obj(1, 'screw'), _obj(2, 'nut', x=None),
                     _obj(3, 'nut'), _obj(4, 'screw'), _obj(5, 'nut'))
    reqs, skipped = plan_spawns(packet, PLANE, TYPES, 3)
    assert [(r.name, r.source_id) for r in reqs] == [('obj_0', 1), ('obj_1', 3), ('obj_2', 4)]
    assert len(skipped) == 3
    assert any('unknown' in s for s in skipped) and any('mm' in s for s in skipped) and any('슬롯' in s for s in skipped)


def test_empty_packet_spawns_nothing():
    assert plan_spawns(_packet(), PLANE, TYPES, 3) == ([], [])


def test_sim_yaml_and_models_match_spawner():
    p = load_sim_params(PKG / 'config' / 'sim.yaml')
    plane = plane_from_params(p)
    assert (plane.x, plane.y) == (p['work_plane_x'], p['work_plane_y'])
    for part_type in p['part_types']:
        assert model_file(PKG / 'models', part_type).is_file()
    xacro = (PKG / 'urdf' / 'ur5e_vacuum.urdf.xacro').read_text()
    for k in range(p['max_objects']):   # DetachableJoint 슬롯 수 = max_objects
        assert f'k="{k}"' in xacro
    assert slot_name(p['max_objects'] - 1) == f"obj_{p['max_objects'] - 1}"


def test_rpy_to_quaternion_matches_known_rotations():
    from ur5e_pick_place.object_spawner import rpy_to_quaternion
    assert rpy_to_quaternion(0, 0, 0) == pytest.approx((0, 0, 0, 1))
    s = math.sqrt(0.5)
    assert rpy_to_quaternion(0, math.pi / 2, 0) == pytest.approx((0, s, 0, s))   # Y 축 90°
    assert rpy_to_quaternion(0, 0, math.pi / 2) == pytest.approx((0, 0, s, s))   # Z 축 90°


def test_marker_shapes_come_from_model_sdf():
    from ur5e_pick_place.object_spawner import marker_shape
    screw = marker_shape(model_file(PKG / 'models', 'screw'))
    assert screw.shape == 'cylinder'
    assert screw.scale == pytest.approx((0.008, 0.008, 0.04))      # 반지름 4 mm, 길이 40 mm
    assert screw.xyz == pytest.approx((0, 0, 0.004))               # 바닥에 누운 원기둥 중심
    s = math.sqrt(0.5)
    assert screw.quat == pytest.approx((0, s, 0, s), abs=1e-4)     # 원기둥 축을 모델 +X(장축)로 눕힘
    nut = marker_shape(model_file(PKG / 'models', 'nut'))
    assert nut.shape == 'cylinder' and nut.scale == pytest.approx((0.013, 0.013, 0.006))
    assert nut.quat == pytest.approx((0, 0, 0, 1))
    assert len(nut.rgba) == 4


def test_marker_shape_box_and_unknown_geometry(tmp_path):
    from ur5e_pick_place.object_spawner import marker_shape
    box = tmp_path / 'box.sdf'
    box.write_text('<sdf version="1.9"><model name="m"><link name="link"><pose>0 0 0.01 0 0 0</pose>'
                   '<visual name="v"><geometry><box><size>0.02 0.01 0.02</size></box></geometry></visual>'
                   '</link></model></sdf>')
    shape = marker_shape(box)
    assert shape.shape == 'box' and shape.scale == pytest.approx((0.02, 0.01, 0.02))
    assert shape.xyz == pytest.approx((0, 0, 0.01)) and shape.rgba == pytest.approx((0.7, 0.7, 0.7, 1))
    mesh = tmp_path / 'mesh.sdf'
    mesh.write_text('<sdf version="1.9"><model name="m"><link name="link"><visual name="v"><geometry>'
                    '<mesh><uri>x.dae</uri></mesh></geometry></visual></link></model></sdf>')
    assert marker_shape(mesh) is None
