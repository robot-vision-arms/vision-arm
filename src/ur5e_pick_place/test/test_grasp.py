import math

import pytest

from ur5e_pick_place.grasp import PlanePose, nearest_equivalent_yaw_deg, tool_down_quaternion, work_to_base, wrap180


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
