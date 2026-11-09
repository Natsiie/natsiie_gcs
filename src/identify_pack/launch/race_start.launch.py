"""总启动入口：相机、颜色识别、停稳 OK、投影、转盘检测与串口桥。

各功能用 ``start_*`` / ``enable_*`` / ``*_show`` 参数单独开关。
当前默认启用副相机、颜色识别、像素投影、转盘外圆与圆心对齐、
串口桥和叠放位检测；主相机（扫码）默认关闭，粉色夹爪到位与三圆环
工位检测保持注释。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
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
    # 主相机分辨率/帧率：默认 640x480 保证 pyzbar 解码不吃满 CPU；需要更远/更小的码时再上 1280x720。
    main_camera_width = LaunchConfiguration('main_camera_width', default='640')
    main_camera_height = LaunchConfiguration('main_camera_height', default='480')
    main_camera_fps = LaunchConfiguration('main_camera_fps', default='25')
    # 主相机格式：默认 MJPG，避免 YUYV 占满 USB2.0 带宽；相机不支持时会由驱动回退。
    main_camera_fourcc = LaunchConfiguration('main_camera_fourcc', default='MJPG')
    min_consistent = LaunchConfiguration('min_consistent', default='3')
    show = LaunchConfiguration('show', default='true')
    secondary_image_topic = LaunchConfiguration('secondary_image_topic')

    return LaunchDescription([
        # ros2 launch 会用管道接管子节点输出，rcutils 检测到非 TTY 会关闭颜色；
        # 显式打开，WARN 才会显示为黄色、ERROR 为红色。
        SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
        DeclareLaunchArgument('main_camera_device', default_value=main_camera_device),
        DeclareLaunchArgument('secondary_camera_device', default_value=secondary_camera_device),
        DeclareLaunchArgument('main_camera_width', default_value=main_camera_width),
        DeclareLaunchArgument('main_camera_height', default_value=main_camera_height),
        DeclareLaunchArgument('main_camera_fps', default_value=main_camera_fps),
        DeclareLaunchArgument('main_camera_fourcc', default_value=main_camera_fourcc),
        DeclareLaunchArgument('secondary_image_topic', default_value=SECONDARY_TOPIC),
        DeclareLaunchArgument('min_consistent', default_value=min_consistent),
        DeclareLaunchArgument('show', default_value=show),
        DeclareLaunchArgument('tracking_color_params_file', default_value=os.path.join(
            share, 'config', 'tracking_color.yaml')),
        DeclareLaunchArgument('ring_digit_templates', default_value=os.path.join(
            share, 'ring_station_digit_templates')),
        # QR 任务码由主相机读取；主相机已在 Pi 上按串号绑定为 /dev/camera_front。
        # 当前先关闭主相机，单独验证转盘位姿矫正；需要扫码时再设为 true。
        DeclareLaunchArgument('start_main_camera', default_value='true'),
        # 副相机用于颜色/工位识别，默认打开；如果已经由其他终端启动，可改为 false。
        DeclareLaunchArgument('start_secondary_camera', default_value='true'),
        # 仅副相机启用重复帧看门狗；0 表示关闭，避免固定图像源误触发。
        DeclareLaunchArgument('secondary_frozen_frame_timeout_sec', default_value='5.0'),
        DeclareLaunchArgument('secondary_camera_fps', default_value='25'),
        # 两台相机默认 MJPG：压缩传输，避免 YUYV 原始画质占满 USB2.0 带宽导致卡死。
        DeclareLaunchArgument('secondary_camera_fourcc', default_value='MJPG'),
        # 副相机画面翻转：-1=上下颠倒(180°), 1=左右镜像, 0=不翻转。
        DeclareLaunchArgument('secondary_camera_flip_code', default_value='0'),
        # 统一串口桥使用的设备和波特率。
        DeclareLaunchArgument('serial_port', default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        # 颜色识别 ROI、HSV 配置以及 TF/像素投影参数文件。
        DeclareLaunchArgument('roi_file', default_value=os.path.join(share, 'ROI_Range.txt')),
        DeclareLaunchArgument('color_config', default_value=os.path.join(share, 'color_config.json')),
        # 物块停稳判定：ROI 内物块连续保持位置不变达到阈值后发抓取 OK；
        # 不再门控色块/坐标输出。hold_frames>0 时直接按帧数（便于实测调参）；
        # 0 表示按 hold_sec × 实测 fps 换算，默认 1.0s 在 25fps 下约 25 帧。
        DeclareLaunchArgument('stationary_hold_sec', default_value='1.0'),
        DeclareLaunchArgument('stationary_hold_frames', default_value='0'),
        DeclareLaunchArgument('stationary_max_pixel_move', default_value='10.0'),
        DeclareLaunchArgument('stationary_miss_frames', default_value='2'),
        # 帧率估计：0=按图像时间戳自动估计，调试时可固定成实测帧率。
        DeclareLaunchArgument('stationary_fps', default_value='0.0'),
        # 停稳后是否发抓取 OK（/arm/arrived -> 串口桥 GrabCmd 0x09）。
        DeclareLaunchArgument('publish_arrived', default_value='true'),
        # 任务模式：true=按二维码顺序+抓取确认每个颜色只发一次；false=连续发送坐标（调试通信链路用）。
        DeclareLaunchArgument('task_mode', default_value='true'),
        # 颜色识别调试日志：仅调试时打开，正常比赛保持 false。
        # 日志写到 <color_log_dir>/color_debug_YYYYmmdd_HHMMSS.jsonl。
        DeclareLaunchArgument('debug_color_log', default_value='false'),
        DeclareLaunchArgument('color_log_dir', default_value=os.path.join(
            os.getcwd(), 'log', 'color_debug')),
        DeclareLaunchArgument('tf_params_file', default_value=os.path.join(
            share, 'config', 'tf_transform.yaml')),
        DeclareLaunchArgument('pixel_params_file', default_value=os.path.join(
            share, 'config', 'pixel_to_base.yaml')),
        # pixel_to_base 的调试窗口默认关闭，无图形环境下也能正常运行。
        DeclareLaunchArgument('pixel_show', default_value='false'),
        # 转盘外圆检测 + 圆心对齐：检测结果经串口桥转发 0x03/0x0E。
        DeclareLaunchArgument('start_turntable_detect', default_value='true'),
        DeclareLaunchArgument('turntable_show', default_value='true'),
        DeclareLaunchArgument('enable_turntable_seed', default_value='true'),
        # 放置/堆料任务参数：阶段顺序、相机标定、平面高度与释放高度（enabled 默认 false）。
        DeclareLaunchArgument('place_task_params_file', default_value=os.path.join(
            share, 'config', 'place_task.yaml')),
        # 夹爪到位检测（粉色在 ROI 连续 5s）：已由 color_detect_node 的停稳 OK 取代，
        # 默认关闭；需要回退到旧的粉色触发逻辑时再设为 true。
        DeclareLaunchArgument('start_gripper_arrival', default_value='false'),
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
                'fourcc': main_camera_fourcc,  # 主相机用 MJPEG，提升扫码清晰度
                'width': ParameterValue(main_camera_width, value_type=int),
                'height': ParameterValue(main_camera_height, value_type=int),
                'fps': ParameterValue(main_camera_fps, value_type=int),
                'qr_min_consistent': ParameterValue(min_consistent, value_type=int),
                'show': ParameterValue(show, value_type=bool),
            }],
        ),

        # 上相机：全图颜色跟踪图像源。
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

        # # 全图多物块跟踪：发布独立 /tracking_color/* 话题。
        # Node(
        #     package='identify_pack',
        #     executable='tracking_color_node',
        #     name='tracking_color_node',
        #     output='screen',
        #     parameters=[LaunchConfiguration('tracking_color_params_file'), {
        #         'image_topic': secondary_image_topic,
        #         'color_config': LaunchConfiguration('color_config'),
        #         'show': ParameterValue(show, value_type=bool),
        #     }],
        # ),

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
                'require_single_roi': True,
                'stationary_hold_sec': ParameterValue(
                    LaunchConfiguration('stationary_hold_sec'), value_type=float),
                'stationary_hold_frames': ParameterValue(
                    LaunchConfiguration('stationary_hold_frames'), value_type=int),
                'stationary_max_pixel_move': ParameterValue(
                    LaunchConfiguration('stationary_max_pixel_move'), value_type=float),
                'stationary_miss_frames': ParameterValue(
                    LaunchConfiguration('stationary_miss_frames'), value_type=int),
                'stationary_fps': ParameterValue(
                    LaunchConfiguration('stationary_fps'), value_type=float),
                'publish_arrived': ParameterValue(
                    LaunchConfiguration('publish_arrived'), value_type=bool),
                'arrived_topic': LaunchConfiguration('arrived_topic'),
                'show': ParameterValue(show, value_type=bool),
                'centers_topic': '/color/centers_pixel',
                'camera_frame': 'camera_optical_frame',
                # 常驻休眠：由 supervisor 按流程使能。
                'enabled': False,
                'enable_topic': '/color/enable',
            }],
        ),

        # 颜色识别调试日志：仅 debug_color_log:=true 时启动，正常比赛默认关闭。
        Node(
            package='identify_pack',
            executable='color_log_node',
            name='color_log_node',
            condition=IfCondition(LaunchConfiguration('debug_color_log')),
            output='screen',
            parameters=[{
                'log_dir': LaunchConfiguration('color_log_dir'),
                'log_prefix': 'color_debug',
            }],
        ),

        # # 夹爪到位检测：粉色标记进入 arrival_roi_file（独立 ROI）连续满 min_confirm_sec 秒发布到位信号。
        # Node(
        #     package='identify_pack',
        #     executable='gripper_arrival',
        #     name='gripper_arrival',
        #     condition=IfCondition(LaunchConfiguration('start_gripper_arrival')),
        #     output='screen',
        #     parameters=[{
        #         'image_topic': secondary_image_topic,
        #         'color_config': LaunchConfiguration('color_config'),
        #         'arrival_roi_file': LaunchConfiguration('arrival_roi_file'),
        #         'min_blob_area': 300,
        #         'min_blob_dimension': 12,
        #         'min_confirm_sec': ParameterValue(
        #             LaunchConfiguration('arrive_confirm_sec'), value_type=float),
        #         'publish_topic': LaunchConfiguration('arrived_topic'),
        #         'show': ParameterValue(show, value_type=bool),
        #     }],
        # ),

        # TF：加载相机外参，结合下位机反馈角度发布底座到相机的变换链。
        Node(
            package='identify_pack',
            executable='tf_transform',
            name='tf_transform',
            output='screen',
            parameters=[LaunchConfiguration('tf_params_file')],
        ),

        # # 二维码任务码：已并入 camera_main（qr_decode=True），不再单独启动，
        # # 否则两个节点会重复发布 /QR_Decode。
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
                'show': ParameterValue(
                    LaunchConfiguration('pixel_show'), value_type=bool),
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

        # 霍夫圆检测节点：订阅副相机（只做圆+数字，发布 /circle_detect、/Order）
        # Node(
        #     package='identify_pack',
        #     executable='circle_detect_node',
        #     name='circle_detect_node',
        #     output='screen',
        #     parameters=[{
        #         'image_topic': SECONDARY_TOPIC,
        #     }],
        # ),

        # 转盘外圆检测节点：订阅副相机，发布 /Turntable (0x03) 与 /TurntableAlign (0x0E)。
        Node(
            package='identify_pack',
            executable='turntable_detect_node',
            name='turntable_detect_node',
            condition=IfCondition(LaunchConfiguration('start_turntable_detect')),
            output='screen',
            parameters=[{
                'image_topic': secondary_image_topic,
                'color_config': LaunchConfiguration('color_config'),
                'enable_material_seed': ParameterValue(
                    LaunchConfiguration('enable_turntable_seed'), value_type=bool),
                'show': ParameterValue(
                    LaunchConfiguration('turntable_show'), value_type=bool),
                # 常驻休眠：由 supervisor 按流程使能。
                'enabled': False,
                'enable_topic': '/turntable/enable',
            }],
        ),

        # 三圆环工位检测节点：第一层放置用它的三环同时定位。
        # 订阅副相机，发布 /ring_station 与 /ring_station/centers_pixel。
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
                'digit_templates': LaunchConfiguration('ring_digit_templates'),
                # 常驻休眠：由 supervisor 按流程使能。
                'enabled': False,
                'enable_topic': '/ring_station/enable',
            }],
        ),

        # # 叠放位（堆料）检测节点：订阅副相机（发布 /stacked_ring，格式 dx_mm,dy_mm,dyaw_deg,corr_*,within）
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
                # 启动时休眠，等 pixel_to_base 在第 3 次放置后发
                # /stacked_ring/enable=true 再订阅图像，降低并行负载。
                'enabled': False,
                'enable_topic': '/stacked_ring/enable',
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

        # 比赛流程 supervisor：按批次/区域切换检测节点、累积 5 帧像素均值，
        # 等待下位机 0x11 AlignDone 后启动放置阶段。
        Node(
            package='identify_pack',
            executable='race_supervisor_node',
            name='race_supervisor_node',
            output='screen',
        ),

        # ROI 叠加节点：订阅副相机（调试用）
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
