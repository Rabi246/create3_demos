import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('create3_demos'), 'config', 'bump_and_go.yaml')
    return LaunchDescription([
        Node(
            package='create3_demos',
            executable='bump_and_go',
            name='bump_and_go',
            parameters=[params],
            output='screen',
        ),
    ])
