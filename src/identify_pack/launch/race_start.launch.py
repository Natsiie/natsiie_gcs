"""总启动入口：主/副相机、颜色识别、TF、坐标投影和统一串口桥。

主相机（QR 解码）与副相机（霍夫圆/颜色识别）分开打开：
  - 主相机发布到 /camera/main/image_raw
  - 副相机发布到 /camera/secondary/image_raw

二维码任务码控制颜色抓取顺序，其他工位识别节点暂未启用。
设备路径、串口和标定配置可用 launch 参数覆盖。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 主相机（QR）话题 / 副相机（霍夫圆）话题
MAIN_TOPIC = '/camera/main/image_raw'
SECONDARY_TOPIC = '/camera/secondary/image_raw'


def generate_launch_description():
    share = get_package_share_directory('identify_pack')
    main_camera_device = LaunchConfiguration('main_camera_device', default='/dev/camera_front')
    secondary_camera_device = LaunchConfiguration('secondary_camera_device', default='/dev/camera_upper')
    min_consistent = LaunchConfiguration('min_consistent', default='3')
    show = LaunchConfiguration('show', default='false')
    secondary_image_topic = LaunchConfiguration('secondary_image_topic')

    return LaunchDescription([
        DeclareLaunchArgument('main_camera_device', default_value=main_camera_device),
        DeclareLaunchArgument('secondary_camera_device', default_value=secondary_camera_device),
        DeclareLaunchArgument('secondary_image_topic', default_value=SECONDARY_TOPIC),
        DeclareLaunchArgument('min_consistent', default_value=min_consistent),
        DeclareLaunchArgument('show', default_value=show),
        # QR 任务码由主相机读取；需要只调颜色时可改为 false。
        DeclareLaunchArgument('start_main_camera', default_value='true'),
        # 副相机用于颜色识别，默认打开；如果已经由其他终端启动，可改为 false。
        DeclareLaunchArgument('start_secondary_camera', default_value='true'),
        # 统一串口桥使用的设备和波特率。
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        # 颜色识别 ROI、HSV 配置以及 TF/像素投影参数文件。
        DeclareLaunchArgument('roi_file', default_value=os.path.join(share, 'ROI_Range.txt')),
        DeclareLaunchArgument('color_config', default_value=os.path.join(share, 'color_config.json')),
        # 运动检测区域可单独指定；留空表示整幅图像，不等同于抓取 ROI。
        DeclareLaunchArgument('motion_roi_file', default_value=''),
        DeclareLaunchArgument('moving_speed_px_s', default_value='15.0'),
        DeclareLaunchArgument('near_stop_speed_px_s', default_value='3.0'),
        DeclareLaunchArgument('required_low_speed_frames', default_value='5'),
        DeclareLaunchArgument('tf_params_file', default_value=os.path.join(
            share, 'config', 'tf_transform.yaml')),
        DeclareLaunchArgument('pixel_params_file', default_value=os.path.join(
            share, 'config', 'pixel_to_base.yaml')),

        # 主相机节点：自己解码 QR，读到内容后发布 /QR_Decode 并关闭摄像头。
        Node(
            package='identify_pack',
            executable='camera_node',
            name='camera_main',
            condition=IfCondition(LaunchConfiguration('start_main_camera')),
            output='screen',
            parameters=[{
                'device': main_camera_device,
                'image_topic': MAIN_TOPIC,
                'qr_decode': True,
                'stop_after_qr': True,
                'qr_min_consistent': ParameterValue(min_consistent, value_type=int),
            }],
        ),

        # 副相机节点：给霍夫圆/颜色识别
        Node(
            package='identify_pack',
            executable='camera_node',
            name='camera_secondary',
            condition=IfCondition(LaunchConfiguration('start_secondary_camera')),
            output='screen',
            parameters=[{
                'device': secondary_camera_device,
                'image_topic': secondary_image_topic,
            }],
        ),

        # 颜色识别：订阅副相机图像，读取 ROI 和 HSV 配置，发布带原图时间戳的像素中心。
        Node(
            package='identify_pack',
            executable='color_detect_node',
            name='color_detect_node',
            output='screen',
            parameters=[{
                'image_topic': secondary_image_topic,
                'roi_file': LaunchConfiguration('roi_file'),
                'color_config': LaunchConfiguration('color_config'),
                'motion_roi_file': LaunchConfiguration('motion_roi_file'),
                'require_single_roi': True,
                'moving_speed_px_s': ParameterValue(
                    LaunchConfiguration('moving_speed_px_s'), value_type=float),
                'near_stop_speed_px_s': ParameterValue(
                    LaunchConfiguration('near_stop_speed_px_s'), value_type=float),
                'required_low_speed_frames': ParameterValue(
                    LaunchConfiguration('required_low_speed_frames'), value_type=int),
                'show': ParameterValue(show, value_type=bool),
                'centers_topic': '/color/centers_pixel',
                'camera_frame': 'camera_optical_frame',
            }],
        ),

        # TF：加载相机外参，结合下位机反馈角度发布底座到相机的变换链。
        Node(
            package='identify_pack',
            executable='tf_transform',
            name='tf_transform',
            output='screen',
            parameters=[LaunchConfiguration('tf_params_file')],
        ),

        # 二维码任务码：已并入 camera_main（qr_decode=True），不再单独启动，
        # 否则两个节点会重复发布 /QR_Decode。
        # Node(
        #     package='identify_pack',
        #     executable='qr_decode_node',
        #     name='qr_decode_node',
        #     condition=IfCondition(LaunchConfiguration('start_main_camera')),
        #     output='screen',
        #     parameters=[{
        #         'image_topic': MAIN_TOPIC,
        #         'min_consistent': ParameterValue(min_consistent, value_type=int),
        #         'show': ParameterValue(show, value_type=bool),
        #     }],
        # ),

        # 像素投影：按图像时刻 TF 和物块顶面高度计算基座 XYZ，ROS 输出单位为米。
        Node(
            package='identify_pack',
            executable='pixel_to_base',
            name='pixel_to_base',
            output='screen',
            parameters=[LaunchConfiguration('pixel_params_file'), {
                'image_topic': secondary_image_topic,
                'show': False,
                'auto_project': True,
                'plane_mode': 'object',
                'centers_topic': '/color/centers_pixel',
                'targets_topic': '/color/targets_base',
            }],
        ),

        # 统一串口桥：目标 XYZ 从米转为毫米，以 0x08 帧发送；接收 0x07 云台角度反馈。
        Node(
            package='identify_pack',
            executable='serial_bridge_node',
            name='serial_bridge_node',
            output='screen',
            arguments=['--serial', LaunchConfiguration('serial_port'),
                       '--baud', LaunchConfiguration('baud')],
            parameters=[{
                'target_topic': '/color/targets_base',
                'send_legacy_color': False,  # 不再向串口转发旧的像素字符串。
                'target_max_hz': 5.0,       # 每种颜色最多发送 5 帧/秒。
                'task_mode': True,          # 按二维码顺序和抓取完成确认发送目标。
                'wait_between_batches': True,
                'centers_topic': '/color/centers_pixel',
                'camera_frame': 'camera_optical_frame',
            }],
        ),

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

        # 颜色识别、TF、坐标投影和串口桥已在本文件上方直接定义。

        # # 转盘外圆检测节点：订阅副相机（发布 /Turntable，格式 cx,cy,radius,stable）
        # Node(
        #     package='identify_pack',
        #     executable='turntable_detect_node',
        #     name='turntable_detect_node',
        #     output='screen',
        #     parameters=[{
        #         'image_topic': SECONDARY_TOPIC,
        #         'show': True,
        #     }],
        # ),

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

        # QR 结果也可由统一串口桥转发；不要再启动旧 qr_serial_bridge，以免重复占用串口。
    ])
