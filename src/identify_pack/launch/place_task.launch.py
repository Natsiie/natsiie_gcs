"""Start placement coordination alongside existing camera, TF and serial nodes."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('identify_pack'), 'config', 'place_task.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('start_detectors', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='/camera/secondary/image_raw'),
        DeclareLaunchArgument('show', default_value='false'),
        Node(package='identify_pack', executable='ring_station_detect_node',
             condition=IfCondition(LaunchConfiguration('start_detectors')),
             output='screen', parameters=[{
                 'image_topic': LaunchConfiguration('image_topic'),
                 'show': ParameterValue(LaunchConfiguration('show'), value_type=bool)}]),
        Node(package='identify_pack', executable='stacked_ring_detect_node',
             condition=IfCondition(LaunchConfiguration('start_detectors')),
             output='screen', parameters=[{
                 'image_topic': LaunchConfiguration('image_topic'),
                 'show': ParameterValue(LaunchConfiguration('show'), value_type=bool)}]),
        Node(package='identify_pack', executable='place_task', name='place_task',
             output='screen', parameters=[LaunchConfiguration('params_file')]),
    ])
