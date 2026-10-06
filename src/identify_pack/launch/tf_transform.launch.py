"""Launch camera/gimbal TF independently of recognition and serial control."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('identify_pack'),
                                  'config', 'tf_transform.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_config),
        Node(package='identify_pack', executable='tf_transform', name='tf_transform',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
