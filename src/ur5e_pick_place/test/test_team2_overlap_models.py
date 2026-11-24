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
