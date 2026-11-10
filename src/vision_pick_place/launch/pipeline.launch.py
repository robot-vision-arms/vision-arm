"""비전 노드 + 로봇 브리지 + GUI 를 한 번에 띄운다. 가상 로봇 서버는 별도 터미널에서 먼저 실행.

    ros2 run vision_pick_place robot_server
    ros2 launch vision_pick_place pipeline.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('vision_pick_place'), 'config', 'params.yaml')
    return LaunchDescription([
        Node(package='vision_pick_place', executable='vision_node', parameters=[params]),
        Node(package='vision_pick_place', executable='robot_bridge', parameters=[params]),
        Node(package='vision_pick_place', executable='gui'),
    ])
