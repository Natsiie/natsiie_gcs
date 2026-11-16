#!/usr/bin/env bash
# race_start.launch.py 的开机自启动入口（由 systemd 服务 race_start.service 调用）。
# 仓库路径、ROS 发行版、串口都从当前机器推导，两台机器共用同一份脚本。
# 系统服务无图形界面，show 默认 false；如需临时开窗口调试可手动 SHOW=true 运行本脚本。
set -euo pipefail

# 仓库根目录由脚本位置推导，不写死用户路径。
WORKSPACE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

# ROS 发行版：优先 $ROS_DISTRO，否则取 /opt/ros 下最新的一个（主机 humble、Pi jazzy）。
if [ -z "${ROS_DISTRO:-}" ] || [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    ROS_DISTRO=$(ls -1 /opt/ros 2>/dev/null | sort -V | tail -n 1 || true)
fi
if [ -z "${ROS_DISTRO}" ] || [ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    echo "错误：找不到可用的 ROS 2 安装（/opt/ros/*/setup.bash）" >&2
    exit 1
fi

SERIAL_PORT=${SERIAL_PORT:-/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0}
SERIAL_WAIT_SEC=${SERIAL_WAIT_SEC:-30}
SHOW=${SHOW:-false}
# 开机后 systemd/udev/网络还没完全就绪时拉起 launch，DDS 发现容易不稳；
# 等串口出现后再固定延时几秒，确保系统服务真正起来后再启动。
START_DELAY_SEC=${START_DELAY_SEC:-8}

# ROS 的 setup.bash 会引用未定义变量，source 期间临时关闭 -u，否则 set -u 会直接退出。
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${WORKSPACE}/install/setup.bash"
set -u

# 开机时 USB 枚举较慢，而 serial_bridge 没有 respawn，等串口出现再拉起，避免启动即崩。
if [ ! -e "${SERIAL_PORT}" ]; then
    echo "等待串口 ${SERIAL_PORT} 出现（最多 ${SERIAL_WAIT_SEC}s）..."
    for _ in $(seq 1 "${SERIAL_WAIT_SEC}"); do
        if [ -e "${SERIAL_PORT}" ]; then
            break
        fi
        sleep 1
    done
    [ -e "${SERIAL_PORT}" ] || echo "警告：${SERIAL_PORT} 仍未出现，继续启动（serial_bridge 可能失败）。"
fi

# 串口就绪后再等 START_DELAY_SEC 秒：开机自启时 systemd/网络/udev 可能还在收尾，
# 立即拉起会让 DDS 发现和相机枚举更容易出问题。
if [ "${START_DELAY_SEC}" -gt 0 ] 2>/dev/null; then
    echo "延时 ${START_DELAY_SEC}s 等待系统服务就绪..."
    sleep "${START_DELAY_SEC}"
fi

# 单机场景：所有 ROS 节点都在本机，DDS 只走 loopback，彻底避免 Wi-Fi 地址
# 变化/重连导致的 DDS 发现分区。需要主机远程看话题时用 ROS_DISCOVERY_RANGE=SUBNET 覆盖。
export ROS_AUTOMATIC_DISCOVERY_RANGE=${ROS_DISCOVERY_RANGE:-LOCALHOST}

# 三个调试窗口开关全部跟随 SHOW：show 覆盖相机/颜色/圆环/叠料，
# turntable_show 是转盘独立参数（默认 true），pixel_show 是投影窗口。
# 无图形环境下任何一个开着都会 cv2.imshow -> Qt 连不上 display -> SIGABRT。
exec ros2 launch identify_pack race_start.launch.py \
    show:=${SHOW} turntable_show:=${SHOW} pixel_show:=${SHOW} "$@"
