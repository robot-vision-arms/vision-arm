"""Gazebo world(SDF) 의 고정 물체를 MoveIt 충돌 박스로 바꾼다 (2주차).

MoveIt(move_group) 은 Gazebo world 파일을 읽지 않는다. 알려 주지 않으면 MoveIt 머릿속에는 로봇만 있어서,
상자 벽이나 바닥을 뚫고 지나가는 경로를 계획할 수 있다. 그래서 pick_place_node 가 시작할 때 이 모듈로
worlds/workcell.sdf 를 읽어 같은 위치·크기의 충돌 박스를 등록한다 (RViz 에 초록색으로 보인다).
크기·위치를 코드에 따로 적지 않으므로 workcell.sdf 의 상자를 옮기거나 늘리면 MoveIt 에도 자동으로 반영된다.

대상: <static>true</static> 인 모델의 <collision> 중 box, plane.
    - box   → 그대로
    - plane → 얇은 판 (윗면이 평면보다 floor_clearance_m 아래). 로봇 바닥면이 평면에 딱 붙어 있어서
              윗면을 평면에 맞추면 시작 자세부터 '충돌' 로 잡힌다
    - 그 밖(mesh, cylinder 등) 은 건너뛴다. 충돌이 없는 모델(work_mat 등 표시용) 도 자동으로 빠진다
좌표는 world 기준이고, UR5e 는 world 원점에 회전 없이 스폰되므로 world = base_link 이다.
"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class CollisionBox:
    id: str                                   # '<모델>/<collision 이름>'
    size: tuple[float, float, float]          # m
    position: tuple[float, float, float]      # m, base_link 기준
    quat_xyzw: tuple[float, float, float, float]


def _pose(element) -> np.ndarray:
    """요소 바로 아래 <pose> 를 4x4 행렬로. 없으면 단위행렬. SDF 의 rpy 는 고정축 X→Y→Z (R = Rz·Ry·Rx)."""
    pose = element.find('pose')
    if pose is None:
        return np.eye(4)
    if pose.get('relative_to'):
        raise ValueError(f'relative_to 가 있는 pose 는 지원하지 않습니다: {element.get("name")}')
    x, y, z, roll, pitch, yaw = (float(v) for v in pose.text.split())
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = (x, y, z)
    return T


def _quat_xyzw(R: np.ndarray) -> tuple[float, float, float, float]:
    """회전행렬 → 쿼터니언 (x, y, z, w)."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (x, y, z, w)


def collision_boxes(sdf_path: str, floor_thickness_m: float = 0.02,
                    floor_clearance_m: float = 0.002) -> list[CollisionBox]:
    """SDF world 파일 → 고정 물체의 충돌 박스 목록."""
    world = ET.parse(sdf_path).getroot().find('world')
    if world is None:
        raise ValueError(f'<world> 가 없습니다: {sdf_path}')
    boxes = []
    for model in world.findall('model'):
        static = model.find('static')
        if static is None or static.text.strip().lower() not in ('true', '1'):
            continue   # 움직이는 모델은 MoveIt 에 고정 장애물로 넣으면 안 된다
        T_model = _pose(model)
        for link in model.findall('link'):
            T_link = T_model @ _pose(link)
            for collision in link.findall('collision'):
                T = T_link @ _pose(collision)
                box, plane = collision.find('geometry/box'), collision.find('geometry/plane')
                if box is not None:
                    size = tuple(float(v) for v in box.find('size').text.split())
                elif plane is not None:
                    normal = np.array([float(v) for v in plane.find('normal').text.split()])
                    if not np.allclose(normal / np.linalg.norm(normal), (0, 0, 1)):
                        continue   # 바닥(위를 향한 평면) 만 판으로 바꾼다
                    sx, sy = (float(v) for v in plane.find('size').text.split())
                    size = (sx, sy, floor_thickness_m)
                    shift = np.eye(4)
                    shift[2, 3] = -(floor_thickness_m / 2 + floor_clearance_m)
                    T = T @ shift
                else:
                    continue
                if len(size) != 3 or min(size) <= 0:
                    raise ValueError(f'잘못된 박스 크기: {model.get("name")}/{collision.get("name")}')
                boxes.append(CollisionBox(id=f'{model.get("name")}/{collision.get("name")}', size=size,
                                          position=tuple(float(v) for v in T[:3, 3]),
                                          quat_xyzw=_quat_xyzw(T[:3, :3])))
    return boxes
