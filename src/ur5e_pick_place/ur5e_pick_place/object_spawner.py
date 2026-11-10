"""Gazebo 에 가상 나사/너트 스폰 (3주차).

비전 결과(work_plane mm, 각도, part_type)를 grasp.work_to_base 로 base_link 좌표로 바꾼 뒤 같은 자리에 모델을 놓는다.
모델 이름은 obj_0 ~ obj_{max_objects-1} 로 고정한다. urdf/ur5e_vacuum.urdf.xacro 의 DetachableJoint 가
붙일 모델 이름(child_model)을 미리 알아야 하기 때문.

로봇 베이스가 Gazebo world 원점에 있으므로 base_link 좌표를 world 좌표로 그대로 쓴다.
ros_gz_sim 의 create 실행 파일을 subprocess 로 부른다:
    ros2 run ros_gz_sim create -world workcell -file <share>/models/<part_type>/model.sdf -name obj_0 -x .. -y .. -z .. -Y ..
"""
from pathlib import Path



def model_file(models_dir: Path, part_type: str) -> Path:
    """part_type 이름 = models/ 아래 폴더 이름. 공구 종류를 늘릴 때 폴더만 추가하면 된다."""
    return models_dir / part_type / 'model.sdf'


def slot_name(k: int) -> str:
    return f'obj_{k}'


def spawn(models_dir: Path, world: str, name: str, part_type: str,
          x: float, y: float, z: float, yaw_rad: float) -> None:
    """모델 원점이 바닥면 중심이라 z 는 작업 평면 높이를 그대로 넣으면 된다 (models/*/model.sdf 주석 참고)."""
    # TODO: subprocess.run(['ros2', 'run', 'ros_gz_sim', 'create', '-world', world, '-file', str(model_file(models_dir, part_type)),
    #        '-name', name, '-x', ..., '-y', ..., '-z', ..., '-Y', ...], check=True, timeout=10)
    raise NotImplementedError


def remove(world: str, name: str) -> None:
    """다음 사이클 전에 치울 때. DetachableJoint 가 지운 뒤 다시 만든 모델을 잘 잡는지는 확인 필요 (docs/DECISIONS.md 4장)."""
    # TODO: gz service -s /world/<world>/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean
    #       --timeout 1000 --req 'name: "<name>" type: MODEL'
    raise NotImplementedError
