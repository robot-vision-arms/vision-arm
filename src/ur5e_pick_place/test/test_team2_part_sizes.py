from pathlib import Path
from types import SimpleNamespace
import copy
import xml.etree.ElementTree as ET
import pytest

from ur5e_pick_place.team2_demo_profile import generate_profile
from ur5e_pick_place.team2_part_sizes import ensure_length_model,apply_packet_sizes,length_key
from ur5e_pick_place.object_spawner import SpawnRequest,marker_shape,model_file
from ur5e_pick_place.team2_spawn_safety import assert_spawn_clear
from ur5e_pick_place.team2_pick_place_node import Team2PickPlaceNode

ROOT=Path(__file__).resolve().parents[3]


def test_individual_sizes_keep_centers_and_correct_shared_mesh_scaling(tmp_path):
    profile=generate_profile(tmp_path,ROOT,realistic=True,screw_length_mm=20.)
    models=Path(profile['asset_prefix'])/'share/ur5e_pick_place/models'
    original=(models/'screw/model.sdf').read_bytes()
    packet={'objects':[dict(id=0,length_mm=15.),dict(id=1,length_mm=35.)]}
    snapshot=copy.deepcopy(packet)
    requests=[SpawnRequest('obj_0','screw',.1,.1,.001,0.,0),SpawnRequest('obj_1','screw',.128,.1,.001,0.,1)]
    sized=apply_packet_sizes(requests,packet,models)
    assert [r.x for r in sized]==[r.x for r in requests]
    assert packet==snapshot and (models/'screw/model.sdf').read_bytes()==original
    for request,length in zip(sized,[15.,35.]):
        sdf=ET.parse(model_file(models,request.part_type)).getroot()
        assert float(sdf.findtext('./model/link/collision/geometry/cylinder/length'))==length/1000
        scale=list(map(float,sdf.findtext("./model/link/visual[@name='team2_detail']/geometry/mesh/scale").split()))
        assert scale==pytest.approx([1.,1.,length/40])
        assert float(sdf.findtext('./model/link/inertial/mass'))==.02
    assert_spawn_clear(sized,models)


def test_missing_sizes_use_old_models_and_invalid_sizes_fail(tmp_path):
    request=SpawnRequest('obj_0','screw',0.,0.,.001,0.,0)
    assert apply_packet_sizes([request],{'objects':[]},tmp_path)==[request]
    for value in [True,float('nan'),-1.,61.]:
        with pytest.raises(ValueError):length_key(value)


def test_picker_uses_spawned_size_before_new_camera_estimate(tmp_path):
    profile=generate_profile(tmp_path,ROOT)
    node=object.__new__(Team2PickPlaceNode)
    node.models_dir=Path(profile['asset_prefix'])/'share/ur5e_pick_place/models'
    node.object_lengths={'obj_0':15.}
    node.latest={'objects':[dict(id=0,length_mm=35.)]}
    target=SpawnRequest('obj_0','screw',.1,.1,.001,0.,0)
    selected=node.sized_target(target)
    assert selected.name==target.name and selected.x==target.x
    assert marker_shape(model_file(node.models_dir,selected.part_type)).scale[2]==.015


def test_individual_markers_keep_original_classes_and_actual_lengths(tmp_path):
    from visualization_msgs.msg import Marker, MarkerArray
    profile=generate_profile(tmp_path,ROOT)
    models=Path(profile['asset_prefix'])/'share/ur5e_pick_place/models'
    node=object.__new__(Team2PickPlaceNode)
    node.shapes={kind:marker_shape(model_file(models,kind)) for kind in ['screw','nut']}
    markers=[]
    for index,(kind,length) in enumerate([('screw',15.),('screw',35.),('nut',None)]):
        path=ensure_length_model(models,length)[1] if length else model_file(models,kind)
        shape=marker_shape(path)
        marker=Marker();marker.header.frame_id=f'obj_{index}'
        marker.type=Marker.CYLINDER;marker.action=Marker.ADD
        marker.scale.x,marker.scale.y,marker.scale.z=shape.scale
        markers.append(marker)
    node.on_markers(MarkerArray(markers=markers))
    assert node.object_types=={'obj_0':'screw','obj_1':'screw','obj_2':'nut'}
    assert node.object_lengths=={'obj_0':15.,'obj_1':35.}


def test_original_model_directory_cannot_receive_variants(tmp_path):
    with pytest.raises(ValueError,match='프로필'):
        ensure_length_model(tmp_path,25.)


@pytest.mark.parametrize('status',['warming_up','unstable','unavailable','stable'])
def test_measurement_failure_never_silently_uses_default_length(tmp_path,status):
    request=SpawnRequest('obj_0','screw',0.,0.,.001,0.,0)
    with pytest.raises(ValueError,match='측정이 준비되지'):
        apply_packet_sizes([request],{'objects':[dict(id=0,length_status=status)]},tmp_path)
