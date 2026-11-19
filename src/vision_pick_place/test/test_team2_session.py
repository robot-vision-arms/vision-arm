import json
import pytest
from vision_pick_place.team2_session import load_session, validate_link_parameters, check_gazebo


def test_session_keeps_gazebo_partition_and_world(tmp_path):
    world = tmp_path/'workcell.sdf'
    world.write_text('<sdf><world name="workcell"/></sdf>')
    path = tmp_path/'session.json'
    data = dict(world_file=str(world), gz_partition='project-test', ros_domain_id='108')
    path.write_text(json.dumps(data))
    session = load_session(path, '108')
    assert session['gz_partition'] == 'project-test'
    assert session['world_name'] == 'workcell'
    with pytest.raises(ValueError, match='mismatch'):
        load_session(path, '0')
    data['gz_partition'] = ''
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='partition'):
        load_session(path, '108')


@pytest.fixture
def link_params():
    return dict(world_name='workcell', work_plane_x=.30, work_plane_y=-.15,
                work_plane_z=0., work_plane_yaw_deg=0., tool_length_m=.05,
                max_objects=3, part_types=['screw', 'nut'], use_sim_time=True)


def test_matching_robot_and_spawner_parameters(link_params):
    validate_link_parameters(link_params, dict(link_params))


@pytest.mark.parametrize('name,value', [('work_plane_x', .40), ('tool_length_m', .06),
                                      ('part_types', ['nut', 'screw']), ('max_objects', 2),
                                      ('use_sim_time', False)])
def test_mismatched_robot_and_spawner_parameters_are_rejected(link_params, name, value):
    with pytest.raises(ValueError, match=name):
        validate_link_parameters(link_params, dict(link_params, **{name: value}))


def test_missing_shared_parameter_is_rejected(link_params):
    running = dict(link_params)
    del running['tool_length_m']
    with pytest.raises(ValueError, match='tool_length_m'):
        validate_link_parameters(link_params, running)


def test_missing_session_explains_how_to_start(tmp_path):
    with pytest.raises(ValueError, match='Start project sim.launch.py'):
        load_session(tmp_path/'absent.json')


@pytest.mark.parametrize('stdout,error', [('name: "workcell"', None),
                                        ('model { name: "ground_plane" }', None),
                                        ('header {}', 'empty scene'),
                                        ('Service call timed out', 'invalid scene'),
                                        ('name: "another_world"', 'world mismatch')])
def test_gazebo_response_is_a_scene_not_just_nonempty_text(monkeypatch, stdout, error):
    from types import SimpleNamespace
    monkeypatch.setattr('vision_pick_place.team2_session.subprocess.run',
                        lambda *a, **k: SimpleNamespace(returncode=0, stdout=stdout))
    if error:
        with pytest.raises(RuntimeError, match=error):
            check_gazebo({'world_name':'workcell'}, {})
    else:
        check_gazebo({'world_name':'workcell'}, {})
