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
        # QR 任务码由主相机读取；当前无 /dev/camera_front 设备，默认关闭（需要时再开 true）。
        DeclareLaunchArgument('start_main_camera', default_value='false'),
        # 副相机用于颜色识别，默认打开；如果已经由其他终端启动，可改为 false。
        DeclareLaunchArgument('start_secondary_camera', default_value='true'),
        # 仅副相机启用重复帧看门狗；0 表示关闭，避免固定图像源误触发。
        DeclareLaunchArgument('secondary_frozen_frame_timeout_sec', default_value='5.0'),
        DeclareLaunchArgument('secondary_camera_fps', default_value='25'),
        DeclareLaunchArgument('secondary_camera_fourcc', default_value='YUYV'),
        # 副相机画面翻转：-1=上下颠倒(180°), 1=左右镜像, 0=不翻转。
        DeclareLaunchArgument('secondary_camera_flip_code', default_value='0'),
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
        # 转盘运动检测开关；调试识别时可设为 false 跳过「先动后停」的门控。
        DeclareLaunchArgument('motion_detect', default_value='true'),
        # 任务模式：true=按二维码顺序+抓取确认每个颜色只发一次；false=连续发送坐标（调试通信链路用）。
        DeclareLaunchArgument('task_mode', default_value='true'),
        DeclareLaunchArgument('tf_params_file', default_value=os.path.join(
            share, 'config', 'tf_transform.yaml')),
        DeclareLaunchArgument('pixel_params_file', default_value=os.path.join(
            share, 'config', 'pixel_to_base.yaml')),
        # 放置/堆料任务参数：阶段顺序、相机标定、平面高度与释放高度（enabled 默认 false）。
        DeclareLaunchArgument('place_task_params_file', default_value=os.path.join(
            share, 'config', 'place_task.yaml')),
        # 夹爪到位检测：复用颜色识别同一 ROI，连续识别阈值 5s，默认关闭（调试后再开）。
        DeclareLaunchArgument('start_gripper_arrival', default_value='true'),
        # 到位信号话题：gripper_arrival 发布 / serial_bridge 订阅，两处共用同一参数。
        DeclareLaunchArgument('arrived_topic', default_value='/arm/arrived'),
        # 到位连续识别阈值（秒），调试时可调小加快测试。
        DeclareLaunchArgument('arrive_confirm_sec', default_value='5.0'),
        # 到位检测独立 ROI 文件（与抓取 ROI 分开，粉色舵机外壳出现区域）。
        DeclareLaunchArgument('arrival_roi_file', default_value=os.path.join(
            share, 'Arrival_ROI.txt')),

        # 主相机节点：自己解码 QR，读到内容后发布 /QR_Decode 并关闭摄像头。
        Node(
            package='identify_pack',
            executable='camera_node',
            name='camera_main',
            condition=IfCondition(LaunchConfiguration('start_main_camera')),
            output='screen',
            respawn=True,
            respawn_delay=2.0,
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
            respawn=True,
            respawn_delay=2.0,
            parameters=[{
                'device': secondary_camera_device,
                'image_topic': secondary_image_topic,
                'fps': ParameterValue(LaunchConfiguration('secondary_camera_fps'), value_type=int),
                'fourcc': LaunchConfiguration('secondary_camera_fourcc'),
                'flip_code': ParameterValue(
                    LaunchConfiguration('secondary_camera_flip_code'), value_type=int),
                'frozen_frame_timeout_sec': ParameterValue(
                    LaunchConfiguration('secondary_frozen_frame_timeout_sec'), value_type=float),
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
                'motion_detect': ParameterValue(
                    LaunchConfiguration('motion_detect'), value_type=bool),
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

        # 夹爪到位检测：粉色标记进入 arrival_roi_file（独立 ROI）连续满 min_confirm_sec 秒发布到位信号。
        Node(
            package='identify_pack',
            executable='gripper_arrival',
            name='gripper_arrival',
            condition=IfCondition(LaunchConfiguration('start_gripper_arrival')),
            output='screen',
            parameters=[{
                'image_topic': secondary_image_topic,
                'color_config': LaunchConfiguration('color_config'),
                'arrival_roi_file': LaunchConfiguration('arrival_roi_file'),
                'min_blob_area': 300,
                'min_blob_dimension': 12,
                'min_confirm_sec': ParameterValue(
                    LaunchConfiguration('arrive_confirm_sec'), value_type=float),
                'publish_topic': LaunchConfiguration('arrived_topic'),
                'show': ParameterValue(show, value_type=bool),
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
                'task_mode': ParameterValue(
                    LaunchConfiguration('task_mode'), value_type=bool),
                'wait_between_batches': True,
                'centers_topic': '/color/centers_pixel',
                'camera_frame': 'camera_optical_frame',
                'arrived_topic': LaunchConfiguration('arrived_topic'),
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

        # 三圆环工位检测节点：订阅副相机（发布 /ring_station，格式 dx_mm,dy_mm,dyaw_deg,corr_*,within）
        Node(
            package='identify_pack',
            executable='ring_station_detect_node',
            name='ring_station_detect_node',
            output='screen',
            parameters=[{
                'image_topic': secondary_image_topic,
                'show': ParameterValue(show, value_type=bool),
                'camera_frame': 'camera_optical_frame',
                'centers_topic': '/ring_station/centers_pixel',
            }],
        ),

        # 叠放位（堆料）检测节点：订阅副相机（发布 /stacked_ring，格式 dx_mm,dy_mm,dyaw_deg,corr_*,within）
        Node(
            package='identify_pack',
            executable='stacked_ring_detect_node',
            name='stacked_ring_detect_node',
            output='screen',
            parameters=[{
                'image_topic': secondary_image_topic,
                'show': ParameterValue(show, value_type=bool),
                'camera_frame': 'camera_optical_frame',
                'centers_topic': '/stacked_ring/centers_pixel',
            }],
        ),

        # 放置/堆料任务协调：按阶段顺序把环/叠放位中心换算成放置指令发布到 /arm/place_command，
        # 由下方统一串口桥转发（0x0A）并接收下位机完成反馈（0x0B -> /arm/place_result）。
        Node(
            package='identify_pack',
            executable='place_task',
            name='place_task',
            output='screen',
            parameters=[LaunchConfiguration('place_task_params_file')],
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

        # QR 结果也可由统一串口桥转发；不要再启动旧 qr_serial_bridge，以免重复占用串口。
    ])
