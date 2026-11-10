"""작업 평면 좌표 → UR5e base_link 좌표, 흡착 툴 자세 계산 (2주차). ROS 에 의존하지 않는 순수 계산.

프레임 정의 (docs/PROJECT.md 3.4):
    work_plane : 격자 종이 좌표계. 비전 패킷의 x_mm, y_mm, angle_deg 가 이 좌표계 값
    base_link  : UR5e 베이스. Gazebo 안에서 work_plane 을 어디에 둘지는 config/sim.yaml 의 work_plane_* 로 정한다
툴은 항상 아래(-Z)를 보고, 길쭉한 흡착 패드의 긴 변(tool0 의 +X)을 물체 장축에 맞춘다.
"""
import math
from dataclasses import dataclass


@dataclass
class PlanePose:
    """base_link 기준 work_plane 원점 위치(m)와 Z 축 회전(deg)."""
    x: float
    y: float
    z: float
    yaw_deg: float


def wrap180(angle_deg: float) -> float:
    """각도를 [-180, 180) 로 접는다."""
    # TODO: % 360 을 이용해 접기 (180 은 -180 으로)
    raise NotImplementedError


def work_to_base(x_mm: float, y_mm: float, plane: PlanePose) -> tuple[float, float, float]:
    """work_plane 의 (x_mm, y_mm) → base_link 의 (x, y, z) [m]. 평면 위의 점이므로 z 는 평면 높이."""
    # TODO: mm → m, plane.yaw_deg 만큼 Z 축 회전 후 (plane.x, plane.y) 더하기. z = plane.z
    raise NotImplementedError


def nearest_equivalent_yaw_deg(angle_deg: float, current_yaw_deg: float) -> float:
    """장축 각도는 θ 와 θ+180 이 같은 자세라, 둘 중 현재 손목 yaw 에 가까운 쪽을 [-180, 180) 로 돌려준다.

    이렇게 안 하면 170° 물체를 집으려고 손목을 -10° 대신 +170° 까지 돌리는 일이 생긴다.
    """
    # TODO: 후보 (angle_deg, angle_deg + 180) 중 wrap180(후보 - current_yaw_deg) 의 절댓값이 작은 쪽 → wrap180
    raise NotImplementedError


def tool_down_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    """툴이 아래를 보면서 Z 축으로 yaw 만큼 돈 자세의 쿼터니언 (x, y, z, w).

    q = Rz(yaw) · Rx(180°). Rx(180°) = (1, 0, 0, 0), Rz(ψ) = (0, 0, sin ψ/2, cos ψ/2) 의 해밀턴 곱으로 구한다.
    """
    # TODO: 두 쿼터니언을 곱한 식을 직접 전개해 (x, y, z, w) 반환. test/test_grasp.py 로 아래를 보는지 확인
    raise NotImplementedError
