from pathlib import Path
import math
import xml.etree.ElementTree as ET

import pytest

from ur5e_pick_place.object_spawner import SpawnRequest
from ur5e_pick_place.team2_demo_profile import generate_profile
from ur5e_pick_place.team2_spawn_safety import overlapping_pairs, assert_spawn_clear, collision_footprint, placed_footprint
from ur5e_pick_place.team2_spawner_adapter import VerifiedGzSpawner

ROOT=Path(__file__).resolve().parents[3]
MODELS=ROOT/'src/ur5e_pick_place/models'


def part(name,kind,x,y=0.,angle=0.):
    return SpawnRequest(name,kind,x,y,.001,math.radians(angle))


def test_reported_screw_pairs_overlap_with_original_geometry():
    pair=[part('obj_0','screw',0.),part('obj_1','screw',.0329)]
    overlap=overlapping_pairs(pair,MODELS)
    assert len(overlap)==1
    assert overlap[0][2]==pytest.approx(-.0071,abs=1e-5)
    with pytest.raises(ValueError,match='obj_0 ↔ obj_1'):
        assert_spawn_clear(pair,MODELS)


def test_shortened_models_resolve_length_overlap_without_moving_centers(tmp_path):
    profile=generate_profile(tmp_path,ROOT,realistic=True,screw_length_mm=20.)
    models=Path(profile['asset_prefix'])/'share/ur5e_pick_place/models'
    import yaml
    params=yaml.safe_load(Path(profile['sim_params']).read_text())['pick_place_node']['ros__parameters']
    assert Path(params['team2_models_dir'])==models
    for pair in [[part('obj_0','screw',0.),part('obj_1','screw',.0329)],
                 [part('obj_0','screw',0.),part('obj_1','nut',.0244)]]:
        assert overlapping_pairs(pair,MODELS)
        assert_spawn_clear(pair,models)
    sdf=ET.parse(models/'screw/model.sdf').getroot()
    assert float(sdf.findtext('./model/link/collision/geometry/cylinder/length'))==.02
    assert float(sdf.findtext('./model/link/visual/geometry/cylinder/length'))==.02
    assert sdf.findtext("./model/link/visual[@name='team2_detail']/geometry/mesh/scale")=='1 1 0.5'
    assert float(sdf.findtext('./model/link/inertial/mass'))==.02
    assert float(sdf.findtext('./model/link/collision/geometry/cylinder/radius'))==.004
    nut=ET.parse(models/'nut/model.sdf').getroot().find('model/link')
    original=ET.parse(MODELS/'nut/model.sdf').getroot().find('model/link')
    for tag in ['collision','inertial','pose']:
        assert ET.tostring(nut.find(tag))==ET.tostring(original.find(tag))


@pytest.mark.parametrize('length',[0.,9.,41.,float('nan')])
def test_invalid_length_is_rejected(tmp_path,length):
    with pytest.raises(ValueError,match='Screw length'):
        generate_profile(tmp_path,ROOT,screw_length_mm=length)


def test_still_overlapping_widths_are_rejected_before_any_deletion(tmp_path,monkeypatch):
    from ur5e_pick_place.object_spawner import GzSpawner
    profile=generate_profile(tmp_path,ROOT,screw_length_mm=20.)
    node=object.__new__(VerifiedGzSpawner)
    node.models_dir=Path(profile['asset_prefix'])/'share/ur5e_pick_place/models'
    calls=[]
    monkeypatch.setattr(GzSpawner,'spawn_all',lambda *args:calls.append('mutated'))
    with pytest.raises(ValueError,match='가상 부품 겹침'):
        node.spawn_all([part('obj_0','screw',0.),part('obj_1','screw',0.,.006)])
    assert calls==[]


def test_crossing_screws_and_nuts_are_checked():
    assert overlapping_pairs([part('obj_0','screw',0.,angle=45),part('obj_1','screw',0.,angle=135)],MODELS)
    assert overlapping_pairs([part('obj_0','nut',0.),part('obj_1','nut',.01)],MODELS)
    assert_spawn_clear([part('obj_0','screw',0.),part('obj_1','nut',.08)],MODELS)


def test_overlapping_image_boxes_do_not_mean_physical_overlap():
    requests=[part('obj_0','screw',0.,angle=45),part('obj_1','screw',.008,-.008,45)]
    polygons=[placed_footprint(r,collision_footprint(MODELS/'screw/model.sdf')) for r in requests]
    for axis in [0,1]:
        assert max(p[axis] for p in polygons[0]) > min(p[axis] for p in polygons[1])
        assert max(p[axis] for p in polygons[1]) > min(p[axis] for p in polygons[0])
    assert_spawn_clear(requests,MODELS)
