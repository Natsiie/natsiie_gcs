"""Launch file: 启动相机、QR 解码节点与串口桥接节点。

图像话题、摄像头设备可通过 launch 参数覆盖；串口参数在 `serial_config.json` 中修改
（安装后位于 share/identify_pack/serial_config.json，源码位于 identify_pack/serial/serial_config.json）。
调试时可加 `show:=true` 打开 QR 解码的可视化窗口。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic', default='/camera/image_raw')
    camera_device = LaunchConfiguration('camera_device', default='/dev/video4')
    min_consistent = LaunchConfiguration('min_consistent', default='3')
    show = LaunchConfiguration('show', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value=image_topic),
        DeclareLaunchArgument('camera_device', default_value=camera_device),
        DeclareLaunchArgument('min_consistent', default_value=min_consistent),
        DeclareLaunchArgument('show', default_value=show),

        # 相机节点：打开主机摄像头并发布到 image_topic
        Node(
            package='identify_pack',
            executable='camera_node',
            name='camera_node',
            output='screen',
            parameters=[{
                'device': camera_device,
                'image_topic': image_topic,
            }],
        ),

        # QR 解码节点：订阅图像话题，发布 QR_Decode
        Node(
            package='identify_pack',
            executable='qr_decode_node',
            name='qr_decode_node',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'min_consistent': ParameterValue(min_consistent, value_type=int),
                'show': ParameterValue(show, value_type=bool),
            }],
        ),

        # 串口桥接节点：串口参数由 serial_config.json 提供
        Node(
            package='identify_pack',
            executable='qr_serial_bridge',
            name='qr_serial_bridge',
            output='screen',
        ),
    ])
