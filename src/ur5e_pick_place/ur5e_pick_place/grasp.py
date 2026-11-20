"""작업 평면 좌표 → UR5e base_link 좌표, 흡착 툴 자세 계산 (2주차). ROS 에 의존하지 않는 순수 계산.

프레임 정의 (docs/PROJECT.md 3.4):
    work_plane : 작업 시트 좌표계 (왼쪽 아래 원점, +X 오른쪽, +Y 위쪽, mm). 비전 패킷의 x_mm, y_mm, angle_deg 가 이 좌표계 값
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
    return (angle_deg + 180.0) % 360.0 - 180.0


def work_to_base(x_mm: float, y_mm: float, plane: PlanePose) -> tuple[float, float, float]:
    """work_plane 의 (x_mm, y_mm) → base_link 의 (x, y, z) [m]. 평면 위의 점이므로 z 는 평면 높이."""
    yaw = math.radians(plane.yaw_deg)
    x, y = x_mm / 1000.0, y_mm / 1000.0
    return (plane.x + math.cos(yaw) * x - math.sin(yaw) * y,
            plane.y + math.sin(yaw) * x + math.cos(yaw) * y, plane.z)


def nearest_equivalent_yaw_deg(angle_deg: float, current_yaw_deg: float) -> float:
    """장축 각도는 θ 와 θ+180 이 같은 자세라, 둘 중 현재 손목 yaw 에 가까운 쪽을 [-180, 180) 로 돌려준다.

    이렇게 안 하면 170° 물체를 집으려고 손목을 -10° 대신 +170° 까지 돌리는 일이 생긴다.
    """
    nearest = min((angle_deg, angle_deg + 180.0),
                  key=lambda candidate: abs(wrap180(candidate - current_yaw_deg)))
    return wrap180(nearest)


def tool_down_quaternion(yaw_deg: float) -> tuple[float, float, float, float]:
    """툴이 아래를 보면서 Z 축으로 yaw 만큼 돈 자세의 쿼터니언 (x, y, z, w).

    q = Rz(yaw) · Rx(180°). Rx(180°) = (1, 0, 0, 0), Rz(ψ) = (0, 0, sin ψ/2, cos ψ/2) 의 해밀턴 곱으로 구한다.
    """
    half_yaw = math.radians(yaw_deg) / 2.0
    return (math.cos(half_yaw), math.sin(half_yaw), 0.0, 0.0)


def cup_tool_xy(part_x: float, part_y: float, cup_offset_m: float, tool_yaw_deg: float = 0.0) -> tuple[float, float]:
    """컵이 tool0 +X 로 cup_offset_m 떨어져 있을 때(링 컵), 컵 중심이 부품 중심에 오도록 하는 tool0 의 (x, y)."""
    yaw = math.radians(tool_yaw_deg)
    return part_x - cup_offset_m * math.cos(yaw), part_y - cup_offset_m * math.sin(yaw)


def inside_bin(xyz, bin_x: float, bin_y: float, half_width_m: float = 0.05,
               z_range_m: tuple[float, float] = (0.003, 0.04)) -> bool:
    """부품 위치가 상자 안쪽 바닥 근처인지. 상자 벽 안쪽 폭 0.14 m 에서 여유를 두고 ±0.05 m (팀원 2 투입 검사와 같은 기준)."""
    x, y, z = xyz
    return abs(x - bin_x) < half_width_m and abs(y - bin_y) < half_width_m and z_range_m[0] < z < z_range_m[1]
