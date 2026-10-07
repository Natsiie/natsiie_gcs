"""Start the image click checker; camera, serial bridge and TF run separately."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('identify_pack'),
                          'config', 'pixel_to_base.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        Node(package='identify_pack', executable='pixel_to_base', name='pixel_to_base',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
