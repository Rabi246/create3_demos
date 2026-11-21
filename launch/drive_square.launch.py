import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('create3_demos'), 'config', 'drive_square.yaml')
    return LaunchDescription([
        Node(
            package='create3_demos',
            executable='drive_square',
            name='drive_square',
            parameters=[params],
            output='screen',
        ),
    ])
