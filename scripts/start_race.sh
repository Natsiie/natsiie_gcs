# #!/usr/bin/env bash
# # race_start.launch.py 的开机自启动入口（由 systemd 服务 race_start.service 调用）。
# # 系统服务无图形界面，show 默认 false；如需临时开窗口调试可手动 SHOW=true 运行本脚本。
# set -euo pipefail

# WORKSPACE=/home/lulu/gcs/natsiie_gcs
# SERIAL_PORT=${SERIAL_PORT:-/dev/ttyUSB0}
# SERIAL_WAIT_SEC=${SERIAL_WAIT_SEC:-30}
# SHOW=${SHOW:-false}

# source /opt/ros/humble/setup.bash
# source "${WORKSPACE}/install/setup.bash"

# # 开机时 USB 枚举较慢，而 serial_bridge 没有 respawn，等串口出现再拉起，避免启动即崩。
# if [ ! -e "${SERIAL_PORT}" ]; then
#     echo "等待串口 ${SERIAL_PORT} 出现（最多 ${SERIAL_WAIT_SEC}s）..."
#     for _ in $(seq 1 "${SERIAL_WAIT_SEC}"); do
#         [ -e "${SERIAL_PORT}" ] && break
#         sleep 1
#     done
#     [ -e "${SERIAL_PORT}" ] || echo "警告：${SERIAL_PORT} 仍未出现，继续启动（serial_bridge 可能失败）。"
# fi

# exec ros2 launch identify_pack race_start.launch.py show:=${SHOW} "$@"
