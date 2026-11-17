import json
import pytest
from vision_pick_place.team2_session import load_session


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
