# 相机到云台的外参标定

继续使用已有 `camera_matrix.npy`、`dist_coeffs.npy`，无需重新标定内参。
此工具只求固定安装变换，不会启动机械臂或向串口发送动作。

## 准备坐标点

1. 云台回实际零位并保持静止。`gimbal_link` 原点设在旋转轴上，Z 向上，
   X 沿零位机械臂伸出方向，Y 按右手系确定。
2. 在一个水平硬板上设置至少 6 个、建议 9～15 个清晰标记，覆盖抓取区域。
   使用同一种明确的点定义，例如十字交点，避免点击标记边缘。
3. 测出每个点相对旋转轴的 X、Y（mm），以及该平面相对坐标原点的 Z（mm）。
   点不能共线；Z 可以为负，不是“相机到标记的距离”。
4. 相机焦距、分辨率与内参采集时一致，当前参数对应 640×480。
   画面裁剪、翻转、缩放后不能直接使用原内参。

## 运行

在 `/home/lulu/gcs` 下运行，`--plane-z-mm` 必须替换为实测高度。
下面的 `0` 仅在标记平面恰好为坐标系 Z=0 时适用。

```bash
python3 natsiie_gcs/src/identify_pack/identify_pack/scripts/calibrate_camera_extrinsics.py \
  --matrix camera_matrix.npy \
  --distortion dist_coeffs.npy \
  --device /dev/camera_upper \
  --plane-z-mm 0 \
  --output camera_extrinsics.json
```

按 Space 冻结画面；左键点击标记后在终端输入该点 `X Y`（mm）。
采集至少 6 点，按 S 求解保存；U 撤销最后一点，Q 退出。
整个采集期间相机、云台、标记都保持不动。工具直接使用原始像素和畸变参数求解，
不要先将图像去畸变后再用同一畸变参数标定。

输出 JSON 包含内参、分辨率、对应点、逐点误差以及：

- `rotation_matrix`、`translation_m`、`quaternion_xyzw`：用于静态 TF。
- `parent_from_camera_matrix`：把相机光学坐标（m）变换到 `gimbal_link`（m）。
- `fit_point_errors_mm`：采样点拟合误差，不是独立验证误差。

已有输出不会被覆盖；重新标定时选择另一个 `--output` 文件名。
也可通过 `--image` 从保存的原始图像采点，或用 `--points` 离线求解，
支持读取已有结果 JSON 中的 `samples`。单独的点文件格式为：

```json
[
  {"pixel": [320, 240], "xy_mm": [100, 0]},
  {"pixel": [400, 240], "xy_mm": [200, 0]}
]
```

示例只展示格式；实际需至少 6 个实测对应点。

## 检查与后续

输出相机位置应与实际安装方向和距离相符。程序会打印静态 TF 发布命令，
parent 是 `gimbal_link`，child 是 `camera_optical_frame`；只应有一个节点发布该关系。
相机光学系 X 向图像右、Y 向下、Z 沿光轴向前，不能当作普通机械坐标轴。

用不参与拟合的点检查毫米定位误差，底座动态 TF 接入后还需在多个云台角度验证。
当前内参 RMS 为 1.5522 px，实际定位精度以这些独立检查结果为准。

下一步需要约定下位机实际云台角度的反馈格式，建立
`gimbal_base -> gimbal_link` 动态 TF，并使用拍照时的变换。
`camera_geometry.pixel_to_plane` 已支持去畸变视线与指定 Z 平面求交；
实际物料顶面高度与标记平面可以不同，但必须填写实际高度。
此工具的结果尚未接入颜色识别话题或串口协议。
