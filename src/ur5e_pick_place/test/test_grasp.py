import math

import pytest

from ur5e_pick_place.grasp import (PlanePose, cup_tool_xy, inside_bin, nearest_equivalent_yaw_deg, tool_down_quaternion,
                                   work_to_base, wrap180)


def test_work_to_base_applies_offset_and_yaw():
    plane = PlanePose(0.30, -0.15, 0.0, 90.0)
    x, y, z = work_to_base(100.0, 0.0, plane)   # 종이 +X 100 mm 가 90° 돌아 base +Y 방향
    assert math.isclose(x, 0.30, abs_tol=1e-9) and math.isclose(y, -0.05, abs_tol=1e-9) and z == 0.0


def test_nearest_yaw_picks_short_rotation():
    assert math.isclose(nearest_equivalent_yaw_deg(170.0, 0.0), -10.0)
    assert math.isclose(nearest_equivalent_yaw_deg(10.0, 180.0), -170.0)
    assert math.isclose(nearest_equivalent_yaw_deg(30.0, 20.0), 30.0)


def test_tool_down_quaternion_is_unit_and_points_down():
    qx, qy, qz, qw = tool_down_quaternion(37.0)
    assert math.isclose(qx * qx + qy * qy + qz * qz + qw * qw, 1.0)
    # 툴 Z 축(0,0,1)을 회전시킨 결과의 z 성분 = 1 - 2(qx² + qy²) 가 -1 이어야 아래를 본다
    assert math.isclose(1 - 2 * (qx * qx + qy * qy), -1.0)


@pytest.mark.parametrize('angle, expected', [(180, -180), (-180, -180), (540, -180), (-370, -10)])
def test_wrap180_boundary(angle, expected):
    assert wrap180(angle) == expected


def test_tool_x_axis_aligns_with_requested_yaw():
    yaw = math.radians(37)
    qx, qy, qz, qw = tool_down_quaternion(37)
    assert 1 - 2 * (qy * qy + qz * qz) == pytest.approx(math.cos(yaw))
    assert 2 * (qx * qy + qw * qz) == pytest.approx(math.sin(yaw))


def test_cup_tool_xy_moves_tool_opposite_to_ring_cup():
    assert cup_tool_xy(0.45, 0.01, 0.012) == pytest.approx((0.438, 0.01))
    assert cup_tool_xy(0.45, 0.01, 0.012, 90.0) == pytest.approx((0.45, -0.002))
    assert cup_tool_xy(0.45, 0.01, 0.0) == (0.45, 0.01)


def test_inside_bin():
    assert inside_bin((0.01, 0.84, 0.005), 0.0, 0.85)
    assert inside_bin((0.0, 0.911, 0.013), 0.0, 0.85)       # 벽 근처, 다른 부품 위 (11-20 실제 사례)
    assert not inside_bin((0.0, 0.77, 0.005), 0.0, 0.85)    # 상자 밖 (벽 너머)
    assert not inside_bin((0.0, 0.85, 0.10), 0.0, 0.85)     # 아직 공중


def test_needs_lift_only_when_low_over_the_mat():
    from ur5e_pick_place.grasp import needs_lift
    assert needs_lift(-0.1, 0.01, 0.25)            # 부품 옆에서 멈춤
    assert not needs_lift(-0.1, 0.25, 0.25)        # 이미 이동 높이 (대기 자세)
    assert not needs_lift(0.82, 0.15, 0.25)        # 상자 위에서 멈춤: 경유점으로 돌아간다
    assert not needs_lift(0.0, 0.9, 0.25)          # 시작 자세 (팔을 세움)
