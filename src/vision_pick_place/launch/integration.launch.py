"""Vision → bridge → TCP server → robot targets, without starting other team GUI code."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('vision_pick_place')
    return LaunchDescription([
        DeclareLaunchArgument('vision_params', default_value=os.path.join(share, 'config', 'params_mvtec.yaml')),
        DeclareLaunchArgument('port', default_value='9000'),
        Node(package='vision_pick_place', executable='robot_server',
             arguments=['--port', LaunchConfiguration('port'), '--publish-ros']),
        Node(package='vision_pick_place', executable='robot_bridge',
             parameters=[{'robot_port': LaunchConfiguration('port')}]),
        Node(package='vision_pick_place', executable='vision_node',
             parameters=[LaunchConfiguration('vision_params')]),
    ])
