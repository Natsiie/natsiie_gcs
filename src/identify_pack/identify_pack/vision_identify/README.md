# QR_Code_identify

轻量级 QR 码识别脚本，基于 `pyzbar` 与 `opencv`，实现连续多帧一致性确认并可选通过串口发送结果。

快速开始:

1. 安装依赖：

```bash
pip install -r requirements.txt
```

2. 运行脚本（示例）：

```bash
python QR_Code_identify.py --device /dev/video_xia0 --min-consistent 3 --serial /dev/ttyUSB0 --baud 9600
```

参数说明：
- `--device` 摄像头设备。
- `--min-consistent` 连续相同帧数以确认二维码。
- `--serial` 可选，指定串口发送识别结果。
- `--baud` 串口波特率。
- `--show` 显示摄像头窗口（用于调试）。
