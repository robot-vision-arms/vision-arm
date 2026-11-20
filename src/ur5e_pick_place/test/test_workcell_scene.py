import math
from pathlib import Path

import numpy as np
import pytest

from ur5e_pick_place.workcell_scene import collision_boxes

WORLD = Path(__file__).resolve().parents[1] / 'worlds' / 'workcell.sdf'


def by_id(boxes):
    return {box.id: box for box in boxes}


def test_workcell_has_floor_and_both_bins():
    boxes = by_id(collision_boxes(str(WORLD)))
    # 상자 2개 × (바닥 + 벽 4개) + 바닥 판. work_mat 은 충돌이 없는 표시용이라 빠져야 한다
    assert len(boxes) == 11
    assert not any(name.startswith('work_mat') for name in boxes)
    wall = boxes['screw_bin/wall_px']
    assert wall.size == (0.005, 0.15, 0.06)
    assert np.allclose(wall.position, (0.0725, 0.85, 0.03))
    assert np.allclose(boxes['nut_bin/floor'].position, (0.0, -0.85, 0.0025))


def test_floor_plane_sits_just_below_robot_base():
    floor = by_id(collision_boxes(str(WORLD), floor_thickness_m=0.02, floor_clearance_m=0.002))['ground_plane/collision']
    assert floor.size == (10.0, 10.0, 0.02)
    assert math.isclose(floor.position[2] + floor.size[2] / 2, -0.002)   # 윗면이 z=0 보다 2 mm 아래


def test_model_yaw_rotates_child_offset(tmp_path):
    sdf = tmp_path / 'w.sdf'
    sdf.write_text("""<sdf version="1.9"><world name="w">
      <model name="bin"><static>true</static><pose>1 0 0 0 0 1.5707963267948966</pose>
        <link name="l"><collision name="c"><pose>0.1 0 0 0 0 0</pose>
          <geometry><box><size>0.1 0.2 0.3</size></box></geometry></collision></link></model>
      <model name="moving"><link name="l"><collision name="c">
          <geometry><box><size>1 1 1</size></box></geometry></collision></link></model>
    </world></sdf>""")
    (box,) = collision_boxes(str(sdf))   # static 이 아닌 모델은 빠진다
    assert np.allclose(box.position, (1.0, 0.1, 0.0))   # 모델이 90° 돌아서 +X 0.1 이 +Y 0.1 이 된다
    half = math.sqrt(0.5)
    assert np.allclose(box.quat_xyzw, (0, 0, half, half))


def test_relative_to_is_rejected(tmp_path):
    sdf = tmp_path / 'w.sdf'
    sdf.write_text("""<sdf version="1.9"><world name="w"><model name="m"><static>true</static>
      <pose relative_to="x">0 0 0 0 0 0</pose></model></world></sdf>""")
    with pytest.raises(ValueError):
        collision_boxes(str(sdf))


@pytest.mark.parametrize('sdf', sorted((WORLD.parents[1] / 'models').glob('*/model.sdf')) + [WORLD],
                         ids=lambda p: f'{p.parent.name}/{p.name}')
def test_sdf_files_are_valid_xml(sdf):
    """XML 주석 안의 '--' 같은 실수는 Gazebo 는 넘어가도 pick_place_node·스폰 노드의 파서는 죽는다 (11-20)."""
    import xml.etree.ElementTree as ET
    ET.parse(sdf)


def test_top_camera_sees_the_whole_work_mat():
    """가상 카메라 화면(바닥 기준)에 작업 매트 A4 가 여백을 두고 다 들어오는지."""
    import xml.etree.ElementTree as ET
    root = ET.parse(WORLD).getroot()
    cam = root.find(".//model[@name='top_camera']")
    x, y, z = (float(v) for v in cam.findtext('pose').split()[:3])
    camera = cam.find('.//sensor/camera')
    hfov = float(camera.findtext('horizontal_fov'))
    w, h = int(camera.findtext('image/width')), int(camera.findtext('image/height'))
    half_w = z * math.tan(hfov / 2)
    half_h = half_w * h / w
    mat = next(i for i in root.iter('include') if i.findtext('name') == 'work_mat')
    mx, my = (float(v) for v in mat.findtext('pose').split()[:2])
    assert abs(mx - x) + 0.297 / 2 < half_w and abs(my - y) + 0.21 / 2 < half_h
