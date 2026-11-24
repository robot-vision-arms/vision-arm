from pathlib import Path
from types import SimpleNamespace
import subprocess
import pytest
from ur5e_pick_place.team2_spawner_adapter import VerifiedGzSpawner,created_model_matches

ROOT=Path(__file__).resolve().parents[3]


def fixture_model():
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.scene_pb2 import Scene
    request=EntityFactory(name='obj_7',sdf_filename=str(ROOT/'src/ur5e_pick_place/models/nut/model.sdf'))
    request.pose.position.x=.45;request.pose.position.y=-.02;request.pose.orientation.w=1.
    scene=Scene();model=scene.model.add(name='obj_7',id=184);model.pose.CopyFrom(request.pose)
    cylinder=model.link.add().visual.add().geometry.cylinder
    cylinder.radius=.0065;cylinder.length=.006
    return request,scene


def test_matching_requires_pose_orientation_and_actual_geometry():
    request,scene=fixture_model();model=scene.model[0]
    assert created_model_matches(model,request)
    model.pose.position.x+=.01;assert not created_model_matches(model,request)
    model.pose.CopyFrom(request.pose);model.link[0].visual[0].geometry.cylinder.length=.04
    assert not created_model_matches(model,request)


def test_lost_create_reply_is_recovered_without_sending_another_create(monkeypatch):
    from gz.msgs10.boolean_pb2 import Boolean
    request,scene=fixture_model();node=object.__new__(VerifiedGzSpawner);node.world='workcell';node.timeout_s=.01
    node.scene=lambda:scene;calls=[]
    def run(cmd,**kw):
        calls.append(cmd);return subprocess.CompletedProcess(cmd,1,stdout='',stderr='response lost')
    monkeypatch.setattr(subprocess,'run',run)
    assert node._request('/world/workcell/create',request,Boolean)
    assert len(calls)==1


def test_missing_or_wrong_model_keeps_creation_failed(monkeypatch):
    from gz.msgs10.boolean_pb2 import Boolean
    request,scene=fixture_model();scene.model[0].pose.position.x+=.1
    node=object.__new__(VerifiedGzSpawner);node.world='workcell';node.timeout_s=.01;node.scene=lambda:scene
    monkeypatch.setattr(subprocess,'run',lambda *a,**k:subprocess.CompletedProcess(a,1,stdout=''))
    ticks=iter([0.,0.,7.]);monkeypatch.setattr('ur5e_pick_place.team2_spawner_adapter.time.monotonic',lambda:next(ticks))
    monkeypatch.setattr('ur5e_pick_place.team2_spawner_adapter.time.sleep',lambda _:None)
    assert not node._request('/world/workcell/create',request,Boolean)


def test_explicit_reply_loss_injection_verifies_state_without_duplicate(monkeypatch):
    from gz.msgs10.boolean_pb2 import Boolean
    request,scene=fixture_model();node=object.__new__(VerifiedGzSpawner)
    node.world='workcell';node.timeout_s=.01;node.scene=lambda:scene;calls=[]
    monkeypatch.setenv('TEAM2_TEST_LOST_CREATE_REPLY','obj_7:1')
    def run(cmd,**kw):
        calls.append(cmd);return subprocess.CompletedProcess(cmd,0,stdout='data: true\n')
    monkeypatch.setattr(subprocess,'run',run)
    assert node._request('/world/workcell/create',request,Boolean)
    assert len(calls)==1


def test_lost_remove_reply_recovers_only_after_actual_disappearance(monkeypatch):
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_pb2 import Entity
    from gz.msgs10.scene_pb2 import Scene
    node=object.__new__(VerifiedGzSpawner);node.world='workcell';node.timeout_s=.01
    node.scene=lambda:Scene()
    monkeypatch.setattr(subprocess,'run',lambda *a,**k:subprocess.CompletedProcess(a,1,stdout=''))
    assert node._request('/world/workcell/remove',Entity(name='obj_7'),Boolean)


def test_adapter_restores_original_factories_even_if_startup_fails(monkeypatch):
    import rclpy
    import tf2_ros
    from ur5e_pick_place import team2_spawner_adapter as adapter
    old_spawner=adapter.original.GzSpawner;old_spin=rclpy.spin
    seen=[]
    def listener(*args,**kwargs):seen.append(kwargs['spin_thread']);return None
    monkeypatch.setattr(tf2_ros,'TransformListener',listener)
    def fail_startup():
        assert adapter.original.GzSpawner is VerifiedGzSpawner
        from tf2_ros import TransformListener
        TransformListener(None,spin_thread=True)
        raise RuntimeError('test startup failure')
    monkeypatch.setattr(adapter.original,'main',fail_startup)
    with pytest.raises(RuntimeError,match='test startup failure'):adapter.main()
    assert seen==[False]
    assert adapter.original.GzSpawner is old_spawner
    assert rclpy.spin is old_spin
    assert tf2_ros.TransformListener is listener


@pytest.mark.parametrize('raises', [False,True])
def test_scene_request_is_frozen_despite_real_detection_and_restored(raises):
    from ur5e_pick_place.team2_spawner_adapter import SceneRequestLatch
    from ur5e_pick_place.team2_demo_profile import scene_packet
    from ur5e_pick_place.object_spawner import PlanePose
    real={'seq':188,'calib_ok':True,'objects':[]}
    node=SimpleNamespace(latest=real,plane=PlanePose(.3,-.15,0.,0.),part_types=['screw','nut'],max_objects=3)
    seen=[]
    def spawn(request,response):
        seen.append(node.latest)
        if raises:raise RuntimeError('test spawn failure')
        response.success=True;return response
    node.on_spawn=spawn;latch=SceneRequestLatch(node);packet=scene_packet(3,6)
    assert latch.receive(packet)=={'seq':6,'accepted':True}
    response=SimpleNamespace(success=False)
    if raises:
        with pytest.raises(RuntimeError):latch.spawn(None,response)
    else:assert latch.spawn(None,response).success
    assert seen==[packet] and node.latest is real
    with pytest.raises(ValueError):latch.receive(real)
