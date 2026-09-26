"""Launch file: 启动主/副两个相机、QR 解码、霍夫圆检测与串口桥接节点。

主相机（QR 解码）与副相机（霍夫圆/颜色识别）分开打开：
  - 主相机发布到 /camera/main/image_raw
  - 副相机发布到 /camera/secondary/image_raw

设备路径用 launch 参数覆盖；串口参数在 serial_config.json 中修改。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 主相机（QR）话题 / 副相机（霍夫圆）话题
MAIN_TOPIC = '/camera/main/image_raw'
SECONDARY_TOPIC = '/camera/secondary/image_raw'


def generate_launch_description():
    main_camera_device = LaunchConfiguration('main_camera_device', default='/dev/camera_front')
    secondary_camera_device = LaunchConfiguration('secondary_camera_device', default='/dev/camera_upper')
    min_consistent = LaunchConfiguration('min_consistent', default='3')
    show = LaunchConfiguration('show', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('main_camera_device', default_value=main_camera_device),
        DeclareLaunchArgument('secondary_camera_device', default_value=secondary_camera_device),
        DeclareLaunchArgument('min_consistent', default_value=min_consistent),
        DeclareLaunchArgument('show', default_value=show),

        # 主相机节点：给 QR 解码
        Node(
            package='identify_pack',
            executable='camera_node',
            name='camera_main',
            output='screen',
            parameters=[{
                'device': main_camera_device,
                'image_topic': MAIN_TOPIC,
            }],
        ),

        # 副相机节点：给霍夫圆/颜色识别
        Node(
            package='identify_pack',
            executable='camera_node',
            name='camera_secondary',
            output='screen',
            parameters=[{
                'device': secondary_camera_device,
                'image_topic': SECONDARY_TOPIC,
            }],
        ),

        # QR 解码节点：订阅主相机
        Node(
            package='identify_pack',
            executable='qr_decode_node',
            name='qr_decode_node',
            output='screen',
            parameters=[{
                'image_topic': MAIN_TOPIC,
                'min_consistent': ParameterValue(min_consistent, value_type=int),
                'show': ParameterValue(show, value_type=bool),
            }],
        ),

        # 霍夫圆检测节点：订阅副相机
        Node(
            package='identify_pack',
            executable='color_identify_node',
            name='color_identify_node',
            output='screen',
            parameters=[{
                'image_topic': SECONDARY_TOPIC,
            }],
        ),

        # # ROI 叠加节点：订阅副相机（调试用）
        # Node(
        #     package='identify_pack',
        #     executable='roi_overlay_node',
        #     name='roi_overlay_node',
        #     output='screen',
        #     parameters=[{
        #         'image_topic': SECONDARY_TOPIC,
        #     }],
        # ),

        # 串口桥接节点：串口参数由 serial_config.json 提供
        Node(
            package='identify_pack',
            executable='qr_serial_bridge',
            name='qr_serial_bridge',
            output='screen',
        ),
    ])
