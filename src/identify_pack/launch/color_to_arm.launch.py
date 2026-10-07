"""Camera -> color centroids -> image-time base projection -> serial targets."""

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
    image_topic = LaunchConfiguration('image_topic')
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera_device', default_value='/dev/camera_upper'),
        DeclareLaunchArgument('image_topic', default_value='/camera/secondary/image_raw'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('show', default_value='false'),
        DeclareLaunchArgument('roi_file', default_value=os.path.join(share, 'ROI_Range.txt')),
        DeclareLaunchArgument('color_config', default_value=os.path.join(share, 'color_config.json')),
        DeclareLaunchArgument('tf_params_file', default_value=os.path.join(
            share, 'config', 'tf_transform.yaml')),
        DeclareLaunchArgument('pixel_params_file', default_value=os.path.join(
            share, 'config', 'pixel_to_base.yaml')),
        Node(package='identify_pack', executable='camera_node', name='camera_secondary',
             condition=IfCondition(LaunchConfiguration('start_camera')), output='screen',
             parameters=[{'device': LaunchConfiguration('camera_device'),
                          'image_topic': image_topic}]),
        Node(package='identify_pack', executable='color_detect_node', name='color_detect_node',
             output='screen', parameters=[{
                 'image_topic': image_topic,
                 'roi_file': LaunchConfiguration('roi_file'),
                 'color_config': LaunchConfiguration('color_config'),
                 'show': ParameterValue(LaunchConfiguration('show'), value_type=bool),
                 'centers_topic': '/color/centers_pixel',
                 'camera_frame': 'camera_optical_frame',
             }]),
        Node(package='identify_pack', executable='tf_transform', name='tf_transform',
             output='screen', parameters=[LaunchConfiguration('tf_params_file')]),
        Node(package='identify_pack', executable='pixel_to_base', name='pixel_to_base',
             output='screen', parameters=[LaunchConfiguration('pixel_params_file'), {
                 'image_topic': image_topic, 'show': False, 'auto_project': True,
                 'plane_mode': 'object', 'centers_topic': '/color/centers_pixel',
                 'targets_topic': '/color/targets_base',
             }]),
        Node(package='identify_pack', executable='serial_bridge_node', name='serial_bridge_node',
             output='screen', arguments=['--serial', LaunchConfiguration('serial_port'),
                                         '--baud', LaunchConfiguration('baud')],
             parameters=[{'target_topic': '/color/targets_base',
                          'send_legacy_color': False, 'target_max_hz': 5.0}]),
    ])
