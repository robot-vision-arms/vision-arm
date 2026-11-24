"""Reuse the installed UR launch; select the Team 2 Gazebo lifecycle wrapper."""
import importlib.util
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    path=os.path.join(get_package_share_directory('ur_simulation_gz'),'launch','ur_sim_control.launch.py')
    spec=importlib.util.spec_from_file_location('team2_original_ur_launch',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    setup=module.launch_setup
    def team2_setup(context,*args,**kwargs):
        actions=setup(context,*args,**kwargs);result=[];replaced=0
        for action in actions:
            if isinstance(action,IncludeLaunchDescription):
                # UR 원본의 Gazebo 실행 연결만 바꾼다. URDF·스폰·컨트롤러 설정은 원본 그대로다.
                action.launch_description_source.get_launch_description(context)
                location=action.launch_description_source.location
                if not location.endswith('/gz_sim.launch.py'):
                    raise RuntimeError('Unexpected installed UR launch include: '+location)
                result.append(Node(package='team2_move_group',executable='team2_gz_server',
                    arguments=[LaunchConfiguration('world_file')],output='screen'))
                result.append(ExecuteProcess(cmd=['gz','sim','-g'],
                    condition=IfCondition(LaunchConfiguration('gazebo_gui')),output='log'))
                replaced+=1
            else:result.append(action)
        if replaced!=1:raise RuntimeError('Expected exactly one Gazebo include in installed UR launch')
        return result
    module.launch_setup=team2_setup
    return module.generate_launch_description()
