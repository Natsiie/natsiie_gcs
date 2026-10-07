"""旧颜色链路命令兼容入口；所有节点统一在 race_start.launch.py 中定义。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('identify_pack')
    image_topic = LaunchConfiguration('image_topic')
    return LaunchDescription([
        # 保留旧参数名，下面映射到总入口的副相机开关和设备参数。
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('camera_device', default_value='/dev/camera_upper'),
        DeclareLaunchArgument('image_topic', default_value='/camera/secondary/image_raw'),
        # 串口桥发送颜色目标坐标使用的串口参数。
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        # 是否显示颜色识别窗口，以及识别配置文件路径。
        DeclareLaunchArgument('show', default_value='false'),
        DeclareLaunchArgument('roi_file', default_value=os.path.join(share, 'ROI_Range.txt')),
        DeclareLaunchArgument('color_config', default_value=os.path.join(share, 'color_config.json')),
        DeclareLaunchArgument('tf_params_file', default_value=os.path.join(
            share, 'config', 'tf_transform.yaml')),
        DeclareLaunchArgument('pixel_params_file', default_value=os.path.join(
            share, 'config', 'pixel_to_base.yaml')),
        # 兼容旧命令时只启用颜色用副相机；具体节点由总入口直接创建。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'race_start.launch.py')),
            launch_arguments={
                'start_main_camera': 'false',
                'start_secondary_camera': LaunchConfiguration('start_camera'),
                'secondary_camera_device': LaunchConfiguration('camera_device'),
                'secondary_image_topic': image_topic,
                'show': LaunchConfiguration('show'),
                'serial_port': LaunchConfiguration('serial_port'),
                'baud': LaunchConfiguration('baud'),
                'roi_file': LaunchConfiguration('roi_file'),
                'color_config': LaunchConfiguration('color_config'),
                'tf_params_file': LaunchConfiguration('tf_params_file'),
                'pixel_params_file': LaunchConfiguration('pixel_params_file'),
            }.items(),
        ),
    ])
