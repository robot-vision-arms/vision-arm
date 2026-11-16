"""Team 2 transport only. Existing vision/GUI/robot nodes run separately."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='9000'),
        DeclareLaunchArgument('ack_timeout_s', default_value='1.0'),
        DeclareLaunchArgument('log_dir', default_value='logs'),
        Node(package='vision_pick_place', executable='robot_server',
             arguments=['--port', LaunchConfiguration('port'), '--publish-ros',
                        '--log-dir', LaunchConfiguration('log_dir')]),
        Node(package='vision_pick_place', executable='robot_bridge',
             parameters=[{
                 'robot_port': ParameterValue(LaunchConfiguration('port'), value_type=int),
                 'ack_timeout_s': ParameterValue(LaunchConfiguration('ack_timeout_s'), value_type=float),
                 'log_dir': LaunchConfiguration('log_dir'),
             }]),
    ])
