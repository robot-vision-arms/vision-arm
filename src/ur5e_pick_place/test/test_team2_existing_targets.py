from dataclasses import replace
import pytest
from ur5e_pick_place.object_spawner import SpawnRequest
from ur5e_pick_place.team2_pick_place_node import bind_existing_targets, ready_reached


def test_detection_order_maps_to_existing_ids_instead_of_replacing_models():
    targets=[SpawnRequest('obj_0','nut',.48,-.01,.001,0,0),
             SpawnRequest('obj_1','screw',.39,-.07,.001,0,1)]
    actual={'obj_0':dict(part_type='screw',x=.39,y=-.07),
            'obj_1':dict(part_type='nut',x=.48,y=-.01)}
    bound=bind_existing_targets(targets,actual)
    assert [t.name for t in bound]==['obj_1','obj_0']
    assert [t.part_type for t in bound]==['nut','screw']


def test_wrong_classification_is_rejected_without_replacing_the_model():
    target=SpawnRequest('obj_0','nut',.39,-.07,.001,0,0)
    with pytest.raises(ValueError,match='종류 불일치'):
        bind_existing_targets([target],{'obj_0':dict(part_type='screw',x=.39,y=-.07)})


@pytest.mark.parametrize('observed', [{},
    {'obj_0':dict(part_type='nut',x=.4,y=-.07)},
    {'obj_0':dict(part_type='nut',x=.39,y=-.07),'obj_1':dict(part_type='nut',x=.392,y=-.07)}])
def test_missing_distant_or_ambiguous_objects_are_rejected(observed):
    with pytest.raises(ValueError,match='대응 실패'):
        bind_existing_targets([SpawnRequest('obj_0','nut',.39,-.07,.001,0,0)],observed)


def test_two_detections_cannot_claim_the_same_model():
    t=SpawnRequest('obj_0','nut',.39,-.07,.001,0,0)
    with pytest.raises(ValueError,match='중복'):
        bind_existing_targets([t,replace(t,source_id=1)],{'obj_0':dict(part_type='nut',x=.39,y=-.07)})


def test_placement_success_must_not_hide_failed_final_return():
    expected=[0.,-1.,1.,1.,1.,0.]
    assert ready_reached(expected, expected)
    assert not ready_reached([1.,-1.,1.,1.,1.,0.], expected)
    assert not ready_reached([],expected)
    assert not ready_reached([float('nan')]*6, expected)


def test_manual_target_stays_exact_while_cycle_can_use_inside_bin_goal(monkeypatch):
    from types import SimpleNamespace
    from ur5e_pick_place.team2_pick_place_node import Team2PickPlaceNode
    from ur5e_pick_place.pick_place_node import PickPlaceNode
    calls=[]
    monkeypatch.setattr(PickPlaceNode,'move_tip',lambda self,x,y,z,yaw=0.,straight=False:
                        calls.append((x,y,z)) or 1)
    node=object.__new__(Team2PickPlaceNode)
    node._cycle_routing=False
    assert node.move_tip(0.,.85,.15)==1
    assert calls==[(0.,.85,.15)]
    node._cycle_routing=True
    node.value=lambda name:dict(bin_drop_z=.15,work_plane_z=0.,travel_height_m=.25,bin_inward_drop_m=.035)[name]
    node.get_parameter=lambda name:SimpleNamespace(value=[.85,-.85])
    node.get_logger=lambda:SimpleNamespace(info=lambda message:None)
    calls.clear();node.move_tip(0.,.85,.15)
    assert calls[0]==(.2,.55,.25)
    assert calls[-1][1]==pytest.approx(.815)
