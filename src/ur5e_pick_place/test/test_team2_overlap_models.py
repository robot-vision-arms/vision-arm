from pathlib import Path
import xml.etree.ElementTree as ET
from ur5e_pick_place.team2_overlap_models import slot_collision_model


def test_slot_masks_ignore_other_parts_but_keep_floor_contact(tmp_path):
    source=Path(__file__).resolve().parents[1]/'models/screw/model.sdf'
    original=source.read_bytes();masks=[]
    for i in range(10):
        path=slot_collision_model(source,f'obj_{i}',tmp_path)
        tree=ET.parse(path)
        mask=int(tree.findtext('.//collide_bitmask'));masks.append(mask)
        assert mask & 0xffff  # 바닥·상자와 접촉은 유지한다.
        assert tree.findtext('.//geometry/cylinder/length')=='0.04'
    assert all(a & b == 0 for i,a in enumerate(masks) for b in masks[i+1:])
    assert source.read_bytes()==original


def test_old_colored_profile_is_gray_in_gazebo_copy_only(tmp_path):
    source=tmp_path/'old.sdf'
    source.write_text('''<sdf version="1.9"><model name="screw"><link name="link">
      <collision name="collision"><geometry><sphere><radius>0.004</radius></sphere></geometry></collision>
      <visual name="old"><material><diffuse>1 0.4 0 1</diffuse></material></visual>
      <visual name="detail"><material><diffuse>0 0 1 1</diffuse></material></visual>
    </link></model></sdf>''')
    original=source.read_bytes()
    result=ET.parse(slot_collision_model(source,'obj_0',tmp_path/'copy'))
    for visual in result.findall('.//visual'):
        assert visual.findtext('material/diffuse')=='0.58 0.58 0.58 1'
        assert visual.findtext('material/ambient')=='0.22 0.22 0.22 1'
    assert source.read_bytes()==original
