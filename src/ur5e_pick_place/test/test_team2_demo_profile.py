from pathlib import Path
import math
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from ur5e_pick_place.team2_demo_profile import generate_profile,scene_packet
from ur5e_pick_place.team2_pick_place_node import cylinder_extents

ROOT=Path(__file__).resolve().parents[3]


def test_ten_parts_are_separated_and_reproducible():
    a=scene_packet(10,5);assert a==scene_packet(10,5)
    assert len(a['objects'])==10
    assert sum(o['part_type']=='nut' for o in a['objects'])==5
    for i,o in enumerate(a['objects']):
        for p in a['objects'][i+1:]:assert math.hypot(o['x_mm']-p['x_mm'],o['y_mm']-p['y_mm'])>=45
    with pytest.raises(ValueError):scene_packet(11)


def test_visual_profile_preserves_mass_collision_and_local_pose(tmp_path):
    profile=generate_profile(tmp_path,ROOT,realistic=True,randomize=True,seed=4)
    for kind in ['screw','nut']:
        original=ET.parse(ROOT/f'src/ur5e_pick_place/models/{kind}/model.sdf').getroot().find('model/link')
        changed=ET.parse(Path(profile['asset_prefix'])/f'share/ur5e_pick_place/models/{kind}/model.sdf').getroot().find('model/link')
        for tag in ['pose','inertial','collision']:
            assert ET.tostring(original.find(tag))==ET.tostring(changed.find(tag))
        assert changed.find("visual[@name='team2_detail']/geometry/mesh/uri") is not None
    assert profile['max_objects']==10 and profile['bin_width_m']==.22
    assert profile['camera_height_m']!=.75
    with np.load(Path(profile['calibration_dir'])/'homography.npz') as data:
        assert np.isfinite(data['H']).all() and np.linalg.matrix_rank(data['H'])==3
    assert (Path(profile['asset_prefix'])/'share/ament_index/resource_index/packages/ur5e_pick_place').exists()


def test_cylinder_bounds_change_with_actual_orientation():
    assert cylinder_extents((0.,0.,1.),.004,.02)==pytest.approx([.004,.004,.02])
    assert cylinder_extents((1.,0.,0.),.004,.02)==pytest.approx([.02,.004,.004])


def test_scene_projection_uses_gazebo_camera_axes():
    from ur5e_pick_place.team2_scene_labels import project_world
    K=np.array([[1000.,0,640],[0,1000.,360],[0,0,1.]])
    pose=[.4485,-.045,.75,0.,math.pi/2,math.pi/2]
    p=project_world([[.4485,-.045,0],[.4585,-.045,0],[.4485,-.035,0]],pose,K)
    assert p[0]==pytest.approx([640.,360.])
    assert p[1,0]>640 and p[2,1]<360


def test_realistic_mesh_labels_do_not_use_hidden_placeholder(tmp_path):
    from ur5e_pick_place.team2_scene_labels import visible_vertices
    profile=generate_profile(tmp_path,ROOT,realistic=True)
    model=Path(profile['asset_prefix'])/'share/ur5e_pick_place/models/nut/model.sdf'
    points=visible_vertices(model)
    assert len(points)==192
    assert np.ptp(points[:,2])==pytest.approx(.006)


def test_mesh_faces_point_outward_so_top_surface_is_visible(tmp_path):
    from ur5e_pick_place.team2_demo_profile import write_mesh
    mesh=tmp_path/'nut.obj';write_mesh(mesh,'nut');vertices=[];faces=[]
    for line in mesh.read_text().splitlines():
        if line.startswith('v '):vertices.append(list(map(float,line.split()[1:])))
        elif line.startswith('f '):faces.append([int(v)-1 for v in line.split()[1:]])
    p=np.asarray(vertices)
    for face in faces:
        tri=p[face];normal=np.cross(tri[1]-tri[0],tri[2]-tri[0])
        if np.allclose(tri[:,2],.003):assert normal[2]>0
        if np.allclose(tri[:,2],-.003):assert normal[2]<0


def test_compact_return_skips_only_intermediate_ready_joint_move(monkeypatch):
    from types import SimpleNamespace
    from ur5e_pick_place.team2_pick_place_node import Team2PickPlaceNode
    from ur5e_pick_place.pick_place_node import PickPlaceNode
    node=object.__new__(Team2PickPlaceNode);calls=[]
    node._cycle_routing=True;node.ready_joints=[0.]*6;node._compact_return_active=True
    node.tf_buffer=SimpleNamespace(lookup_transform=lambda *a:SimpleNamespace(transform=SimpleNamespace(translation=SimpleNamespace(y=.815))))
    node.value=lambda n:0. if n=='work_plane_z' else .25
    node.get_logger=lambda:SimpleNamespace(info=lambda msg:None)
    monkeypatch.setattr(PickPlaceNode,'move_tip',lambda self,*a:calls.append('waypoint'))
    monkeypatch.setattr(PickPlaceNode,'go_ready',lambda self:calls.append('ready'))
    node.go_ready();assert calls==['waypoint','waypoint']
    calls.clear();node._compact_return_active=False;node.go_ready()
    assert calls==['waypoint','waypoint','ready']


@pytest.mark.parametrize('acceleration', [.5, 0., -1., float('nan'), 1.1])
def test_acceleration_is_validated_before_motion(monkeypatch, acceleration):
    from types import SimpleNamespace
    from ur5e_pick_place.team2_pick_place_node import Team2PickPlaceNode
    from ur5e_pick_place.pick_place_node import PickPlaceNode
    node=object.__new__(Team2PickPlaceNode)
    node.value=lambda name:acceleration
    node.moveit2=SimpleNamespace(max_acceleration=0.)
    prepared=[]
    monkeypatch.setattr(PickPlaceNode,'prepare_motion',lambda self:prepared.append(True))
    if acceleration==.5:
        node.prepare_motion()
        assert prepared and node.moveit2.max_acceleration==.5
    else:
        with pytest.raises(ValueError):node.prepare_motion()
        assert not prepared


def test_position_variation_is_bounded_reproducible_and_rejects_unsafe_range():
    for seed in range(20):
        plain=scene_packet(10,seed)
        varied=scene_packet(10,seed,5.)
        assert varied==scene_packet(10,seed,5.) and varied!=plain
        for a,b in zip(plain['objects'],varied['objects']):
            assert abs(a['x_mm']-b['x_mm'])<=5
            assert abs(a['y_mm']-b['y_mm'])<=5
            assert a['part_type']==b['part_type'] and a['angle_deg']==b['angle_deg']
        for i,a in enumerate(varied['objects']):
            for b in varied['objects'][i+1:]:
                assert math.hypot(a['x_mm']-b['x_mm'],a['y_mm']-b['y_mm'])>=35
    for invalid in [-1.,6.,float('nan')]:
        with pytest.raises(ValueError):scene_packet(10,0,invalid)
