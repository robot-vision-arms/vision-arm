"""Team 2 adapter for linking TCP to another team's unmodified spawner."""
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


def load_session(path, ros_domain_id='0'):
    # 팀원 1의 TF/이동과 팀원 3의 스폰이 동일한 월드·ROS 도메인을 사용하도록 연결 경계를 검사한다.
    data = json.loads(Path(path).read_text())
    partition = data.get('gz_partition')
    if not isinstance(partition, str) or not partition:
        raise ValueError('Missing Gazebo partition. Start project sim.launch.py first.')
    if str(data.get('ros_domain_id', '0')) != str(ros_domain_id):
        raise ValueError(f"ROS_DOMAIN_ID mismatch: simulator={data.get('ros_domain_id')}, terminal={ros_domain_id}. "
                         'Use the same domain in both terminals.')
    world_path = Path(data['world_file']).resolve(strict=True)
    world = ET.parse(world_path).getroot().find('world')
    if world is None:
        raise ValueError('Session SDF has no world')
    return dict(data, world_file=str(world_path), world_name=world.attrib['name'])


def check_gazebo(session, env):
    result = subprocess.run(['gz', 'service', '-s', f"/world/{session['world_name']}/scene/info",
        '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene', '--timeout', '4000', '--req', ''],
        capture_output=True, text=True, timeout=7, env=env)
    if result.returncode or not result.stdout.strip():
        raise RuntimeError('Recorded Gazebo session is not responding. Restart sim.launch.py; '
                           'a remaining RViz window does not prove Gazebo is running.')
