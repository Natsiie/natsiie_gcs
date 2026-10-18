"""Run full-frame tracking on an existing image topic, optionally start a camera."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('identify_pack')
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='false'),
        DeclareLaunchArgument('device', default_value='/dev/camera_upper'),
        DeclareLaunchArgument('image_topic', default_value='/camera/secondary/image_raw'),
        DeclareLaunchArgument('show', default_value='false'),
        DeclareLaunchArgument('detector_mode', default_value='hsv_blob'),
        DeclareLaunchArgument('flip_code', default_value='0'),
        DeclareLaunchArgument('color_config', default_value=os.path.join(share, 'color_config.json')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'tracking_color.yaml')),
        Node(
            package='identify_pack', executable='camera_node', name='tracking_color_camera',
            condition=IfCondition(LaunchConfiguration('start_camera')), output='screen',
            parameters=[{
                'device': LaunchConfiguration('device'),
                'image_topic': LaunchConfiguration('image_topic'),
                'width': 640, 'height': 480, 'fps': 25, 'fourcc': 'MJPG',
                'flip_code': ParameterValue(LaunchConfiguration('flip_code'), value_type=int),
                'qr_decode': False, 'show': False,
            }],
        ),
        Node(
            package='identify_pack', executable='tracking_color_node', name='tracking_color_node',
            output='screen', parameters=[LaunchConfiguration('params_file'), {
                'detector_mode': LaunchConfiguration('detector_mode'),
                'image_topic': LaunchConfiguration('image_topic'),
                'color_config': LaunchConfiguration('color_config'),
                'show': ParameterValue(LaunchConfiguration('show'), value_type=bool),
            }],
        ),
    ])
