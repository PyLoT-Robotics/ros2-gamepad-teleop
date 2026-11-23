from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = str(Path(get_package_share_directory('gamepad_teleop')) / 'config/gamepad.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default),
        Node(package='joy', executable='joy_node', parameters=[{'autorepeat_rate': 20.0}]),
        Node(package='gamepad_teleop', executable='gamepad_teleop_node',
             parameters=[{'config_file': LaunchConfiguration('config_file')}], output='screen'),
    ])
