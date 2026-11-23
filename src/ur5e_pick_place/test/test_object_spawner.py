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


# ---- 진공 컵 흡착 조건 (suction_check) ----
from ur5e_pick_place.grasp import tool_down_quaternion
from ur5e_pick_place.object_spawner import (HOLE_RADIUS_M, SuctionLimits, cup_tips, marker_shape as _shape,
                                            part_mass, rotate, suction_check)

SCREW = _shape(PKG / 'models' / 'screw' / 'model.sdf')
NUT = _shape(PKG / 'models' / 'nut' / 'model.sdf')
SCREW_KG = part_mass(PKG / 'models' / 'screw' / 'model.sdf')
NUT_KG = part_mass(PKG / 'models' / 'nut' / 'model.sdf')
P = (0.4, -0.05, 0.0)


def _yaw_q(deg):
    return (0.0, 0.0, math.sin(math.radians(deg) / 2), math.cos(math.radians(deg) / 2))


def _screw(tip, tool_yaw=0.0, part_yaw=30.0, lim=SuctionLimits(), kg=SCREW_KG):
    return suction_check(tip, tool_down_quaternion(tool_yaw), P, _yaw_q(part_yaw), SCREW, kg, 0.0, lim)


def _along(mm, z):
    """나사(장축 30°) 위에서 장축 방향으로 mm 만큼 옮긴 컵 끝 위치."""
    a = math.radians(30)
    return (P[0] + math.cos(a) * mm / 1000, P[1] + math.sin(a) * mm / 1000, z)


def _across(mm, z):
    a = math.radians(30 + 90)
    return (P[0] + math.cos(a) * mm / 1000, P[1] + math.sin(a) * mm / 1000, z)


def test_rotate_and_masses():
    assert rotate(_yaw_q(90), (1, 0, 0)) == pytest.approx((0, 1, 0), abs=1e-9)
    assert rotate(tool_down_quaternion(0), (0, 0, 1)) == pytest.approx((0, 0, -1), abs=1e-9)  # 툴 Z 가 아래
    assert SCREW_KG == pytest.approx(0.02) and NUT_KG == pytest.approx(0.005)


def test_cup_on_top_of_screw_is_held_in_any_yaw():
    for tool_yaw in (0.0, 30.0, 77.0):                         # 둥근 컵이라 방향은 상관없다
        ok, reason, m = _screw((P[0], P[1], 0.009), tool_yaw)   # 나사 윗면 8 mm, 컵 끝 9 mm → 간격 1 mm
        assert ok, reason
        assert m['gap_mm'] == pytest.approx(1.0, abs=0.01)
        assert m['force_n'] > 2 * m['weight_n']                 # 60 kPa × Ø5 mm ≈ 1.18 N > 2 × 0.196 N
    ok, reason, _ = _screw(_along(15.0, 0.009))                 # 장축 방향으로 15 mm 옮겨도 나사 위
    assert ok, reason
    ok, reason, m = _screw(_across(0.8, 0.009))                 # 옆으로 0.8 mm 까지는 주름 컵이 메운다
    assert ok, reason
    assert m['surface_drop_mm'] < 2.0


@pytest.mark.parametrize('tip, word', [
    ((P[0], P[1], 0.15), '떨어져'),          # 150 mm 위
    ((P[0], P[1], 0.012), '떨어져'),         # 간격 4 mm: 예전 패드 기준으로는 붙었지만 컵은 2 mm 까지
    ((P[0], P[1], 0.004), '눌렀'),           # 4 mm 파고듦
    (_along(19.0, 0.009), '끝을 벗어남'),    # 나사 끝(20 mm)에 컵이 걸침
    (_across(1.2, 0.009), '밀착 안 됨'),     # 옆으로 1.2 mm: 컵 입술이 닿는 둥근 면 높이 차 2.5 mm > 2 mm
    (_across(2.0, 0.009), '면을 벗어남'),    # 옆으로 2 mm: 컵(반지름 2.5)이 나사(반지름 4) 밖으로
])
def test_screw_rejected(tip, word):
    ok, reason, _ = _screw(tip)
    assert not ok and word in reason, reason


def test_weight_and_tilt_limits():
    ok, reason, _ = _screw((P[0], P[1], 0.009), kg=0.1)        # 100 g 이면 1.18 N < 2 × 0.98 N
    assert not ok and '흡착력' in reason
    t = math.radians(30) / 2                                    # 툴을 X 축으로 30° 기울임
    tilt = (math.sin(t), 0.0, 0.0, math.cos(t))
    x, y, z, w = tool_down_quaternion(0.0)
    q = (tilt[3] * x + tilt[0] * w + tilt[1] * z - tilt[2] * y,
         tilt[3] * y - tilt[0] * z + tilt[1] * w + tilt[2] * x,
         tilt[3] * z + tilt[0] * y - tilt[1] * x + tilt[2] * w,
         tilt[3] * w - tilt[0] * x - tilt[1] * y - tilt[2] * z)
    ok, reason, m = suction_check((P[0], P[1], 0.009), q, P, _yaw_q(30), SCREW, SCREW_KG)
    assert not ok and '아래를' in reason and m['tilt_deg'] == pytest.approx(30, abs=0.5)


def test_nut_with_hole_cannot_be_held_like_real_m8_nut():
    hole = HOLE_RADIUS_M['nut']
    for offset in (0.0, 0.005):                                 # 구멍 위, 테두리 위 모두
        tip = (P[0] + offset, P[1], 0.007)
        ok, reason, _ = suction_check(tip, tool_down_quaternion(0), P, _yaw_q(0), NUT, NUT_KG, hole)
        assert not ok and '폭' in reason                       # 테두리 2.5 mm < 컵 5 mm
    # 구멍이 없다고 하면(꽉 찬 원판) 가운데에서 잡힌다
    ok, reason, m = suction_check((P[0], P[1], 0.007), tool_down_quaternion(0), P, _yaw_q(0), NUT, NUT_KG, 0.0)
    assert ok, reason
    # 더 작은 컵(지름 2 mm)이면 테두리에 올릴 수 있지만 구멍 위는 여전히 샌다
    small = SuctionLimits(cup_diameter=0.002, safety_factor=1.0)
    ok, reason, _ = suction_check((P[0], P[1], 0.007), tool_down_quaternion(0), P, _yaw_q(0), NUT, NUT_KG, hole,
                                  small)
    assert not ok and '구멍 위' in reason
    ok, reason, _ = suction_check((P[0] + 0.0052, P[1], 0.007), tool_down_quaternion(0), P, _yaw_q(0), NUT,
                                  NUT_KG, hole, small)
    assert ok, reason


def test_two_cup_head_positions():
    tool = (0.4, -0.05, 0.1)
    for yaw in (0.0, 90.0):
        (big, big_tip, big_ring), (ring, ring_tip, is_ring) = cup_tips(tool, tool_down_quaternion(yaw), 0.05)
        assert (big, big_ring, ring, is_ring) == ('5 mm 컵', False, '링 컵', True)
        assert big_tip == pytest.approx((0.4, -0.05, 0.05))              # 5 mm 컵: tool0 바로 아래 50 mm
        assert ring_tip[2] == pytest.approx(0.05)                        # 두 컵 입술 끝 높이는 같다
        assert math.dist(big_tip, ring_tip) == pytest.approx(0.012)      # 링 컵은 12 mm 옆
    # 툴을 90° 돌리면 링 컵도 같이 돈다
    _, a, _ = cup_tips(tool, tool_down_quaternion(0.0), 0.05)[1]
    _, b, _ = cup_tips(tool, tool_down_quaternion(90.0), 0.05)[1]
    va, vb = (a[0] - 0.4, a[1] + 0.05), (b[0] - 0.4, b[1] + 0.05)
    assert va[0] * vb[0] + va[1] * vb[1] == pytest.approx(0.0, abs=1e-9)


def _ring_nut(dx, z=0.0065):
    return suction_check((P[0] + dx, P[1], z), tool_down_quaternion(0), P, _yaw_q(0), NUT, NUT_KG,
                         HOLE_RADIUS_M['nut'], SuctionLimits(), ring=True)


def test_ring_cup_with_pin_holds_nut_at_center():
    for dx in (0.0, 0.001, 0.0014):                              # 핀이 1.5 mm 어긋남까지 끌어 맞춘다
        ok, reason, m = _ring_nut(dx)
        assert ok, reason
        assert m['centered_offset_mm'] == pytest.approx(0.2)    # 맞춘 뒤 남는 틈 (구멍 8 - 핀 7.6) / 2
        assert m['force_n'] == pytest.approx(2.97, abs=0.01)     # 60 kPa × 고리 면적 (반지름 4.5 ~ 6 mm)
        assert m['force_n'] > 2 * m['weight_n']
    ok, reason, _ = _ring_nut(0.0016)
    assert not ok and '핀이 구멍에 안 들어감' in reason
    ok, reason, _ = _ring_nut(0.0, z=0.009)                      # 3 mm 위
    assert not ok and '떨어져' in reason


def test_ring_cup_rejects_screw_and_bad_pin():
    ok, reason, _ = suction_check((P[0], P[1], 0.0085), tool_down_quaternion(30), P, _yaw_q(30), SCREW, SCREW_KG,
                                  0.0, SuctionLimits(), ring=True)
    assert not ok and '면을 벗어남' in reason                  # 바깥 지름 12 mm > 나사 지름 8 mm
    thick = SuctionLimits(pin_diameter=0.0085)
    ok, reason, _ = suction_check((P[0], P[1], 0.0065), tool_down_quaternion(0), P, _yaw_q(0), NUT, NUT_KG,
                                  HOLE_RADIUS_M['nut'], thick, ring=True)
    assert not ok and '굵음' in reason
    narrow = SuctionLimits(ring_inner_diameter=0.008)            # 안쪽 입술이 구멍 가장자리에 걸림
    ok, reason, _ = suction_check((P[0], P[1], 0.0065), tool_down_quaternion(0), P, _yaw_q(0), NUT, NUT_KG,
                                  HOLE_RADIUS_M['nut'], narrow, ring=True)
    assert not ok and '안쪽 입술' in reason


def test_part_top_height_from_models():
    from ur5e_pick_place.object_spawner import part_top_m
    assert part_top_m(_shape(model_file(PKG / 'models', 'screw'))) == pytest.approx(0.008)   # 누운 원기둥: 지름 8 mm
    assert part_top_m(_shape(model_file(PKG / 'models', 'nut'))) == pytest.approx(0.006)     # 선 원기둥: 두께 6 mm


def test_random_packet_is_spaced_and_spawnable():
    import math
    import random
    from ur5e_pick_place.object_spawner import random_packet
    for seed in range(20):
        packet = random_packet(random.Random(seed))
        objs = packet['objects']
        assert sorted(o['part_type'] for o in objs) == ['nut', 'screw', 'screw']
        assert all(70 <= o['x_mm'] <= 227 and 40 <= o['y_mm'] <= 170 for o in objs)
        assert all(math.dist((a['x_mm'], a['y_mm']), (b['x_mm'], b['y_mm'])) >= 34.9
                   for i, a in enumerate(objs) for b in objs[i + 1:])
        assert all((o['angle_deg'] is None) == (o['part_type'] == 'nut') for o in objs)
        requests, skipped = plan_spawns(packet, PlanePose(0.3, -0.15, 0.0, 0.0), ['screw', 'nut'], 3)
        assert len(requests) == 3 and not skipped


def test_existing_slots_only_when_pose_info_is_fresh():
    """clear() 는 pose/info 로 본 슬롯만 지운다. 정보가 없거나 오래되면 None (전부 지우는 예전 동작)."""
    from ur5e_pick_place.object_spawner import existing_slots
    poses = {'obj_0': (10.0, (0, 0, 0), (0, 0, 0, 1)), 'obj_1': (9.0, (0, 0, 0), (0, 0, 0, 1))}
    assert existing_slots(poses, None, 10.1) is None              # 아직 한 번도 못 받음
    assert existing_slots(poses, 9.0, 10.1) is None               # 마지막 메시지가 오래됨 (정지·끊김)
    assert existing_slots(poses, 10.0, 10.1) == {'obj_0'}         # obj_1 은 새 메시지에 없음 = 지워짐
    assert existing_slots({}, 10.0, 10.1) == set()                # 장면에 부품 없음 → 지울 것 없음
