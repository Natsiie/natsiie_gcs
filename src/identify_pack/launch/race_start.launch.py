"""Launch file: 启动主/副两个相机、QR 解码、圆/数字、色块、转盘检测与串口桥接节点。

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

        # # QR 解码节点：订阅主相机
        # Node(
        #     package='identify_pack',
        #     executable='qr_decode_node',
        #     name='qr_decode_node',
        #     output='screen',
        #     parameters=[{
        #         'image_topic': MAIN_TOPIC,
        #         'min_consistent': ParameterValue(min_consistent, value_type=int),
        #         'show': ParameterValue(show, value_type=bool),
        #     }],
        # ),

        # # 霍夫圆检测节点：订阅副相机（只做圆+数字，发布 /circle_detect、/Order）
        # Node(
        #     package='identify_pack',
        #     executable='circle_detect_node',
        #     name='circle_detect_node',
        #     output='screen',
        #     parameters=[{
        #         'image_topic': SECONDARY_TOPIC,
        #     }],
        # ),

        # # 色块识别节点：订阅副相机（ROI 内主色锁定 + 六色 + 转盘运动判定，发布 /Color、/ColorMotion）
        # Node(
        #     package='identify_pack',
        #     executable='color_detect_node',
        #     name='color_detect_node',
        #     output='screen',
        #     parameters=[{
        #         'image_topic': SECONDARY_TOPIC,
        #         'min_blob_area': 300,
        #         'min_blob_dimension': 12,
        #         # 色块 ROI：优先读 roi_file（每行 x,y,w,h，可多个）；读不到则用单个 roi_* 参数
        #         'roi_file': '/home/lulu/gcs/natsiie_gcs/src/identify_pack/identify_pack/scripts/ROI_Range.txt',
        #         'roi_x': 0,
        #         'roi_y': 0,
        #         'roi_w': 0,
        #         'roi_h': 0,
        #         # ROI 内主色锁定：面积最大的颜色锁定物块，盖住被误切的小面积颜色（如蓝↔浅蓝）
        #         'lock_dominant_color': True,
        #         # 转盘转动/停稳判定：跨帧追踪色块质心算速度，先见转动再见停稳
        #         'motion_detect': True,
        #         'moving_speed_px_s': 30.0,
        #         'required_moving_frames': 2,
        #         'near_stop_speed_px_s': 25.0,
        #         'required_low_speed_frames': 3,
        #     }],
        # ),

        # 转盘外圆检测节点：订阅副相机（发布 /Turntable，格式 cx,cy,radius,stable）
        Node(
            package='identify_pack',
            executable='turntable_detect_node',
            name='turntable_detect_node',
            output='screen',
            parameters=[{
                'image_topic': SECONDARY_TOPIC,
                'show': True,
            }],
        ),

        # # 三圆环工位检测节点：订阅副相机（发布 /ring_station，格式 dx_mm,dy_mm,dyaw_deg,corr_*,within）
        # Node(
        #     package='identify_pack',
        #     executable='ring_station_detect_node',
        #     name='ring_station_detect_node',
        #     output='screen',
        #     parameters=[{
        #         'image_topic': SECONDARY_TOPIC,
        #         'show': True,
        #     }],
        # ),

        # # 叠放位检测节点：订阅副相机（发布 /stacked_ring，格式 dx_mm,dy_mm,dyaw_deg,corr_*,within）
        # Node(
        #     package='identify_pack',
        #     executable='stacked_ring_detect_node',
        #     name='stacked_ring_detect_node',
        #     output='screen',
        #     parameters=[{
        #         'image_topic': SECONDARY_TOPIC,
        #         'show': True,
        #     }],
        # ),

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
