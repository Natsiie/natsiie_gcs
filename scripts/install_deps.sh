#!/usr/bin/env bash
# install_deps.sh — 安装 identify_pack 运行/标定/测试所需的依赖（Ubuntu/Debian + ROS 2）
#
# 用法:
#   ./scripts/install_deps.sh              # 安装 apt + ROS 依赖
#   ./scripts/install_deps.sh --check      # 只检查当前环境，不安装
#   ./scripts/install_deps.sh --no-ros     # 跳过 ROS 相关 apt 包（ROS 由其他方式提供）
#   ./scripts/install_deps.sh --with-udev  # 顺带安装相机/串口固定名 udev 规则
#
# 说明:
#   - 本脚本不会安装 ROS 2 本身；系统里必须已有 /opt/ros/<distro>。
#   - OpenCV / numpy / scipy 优先用 apt（python3-opencv 等），避免 pip 版 OpenCV
#     与 ROS cv_bridge 链接的 OpenCV 冲突。
#   - 只有 apt 缺失的纯 Python 包才用 pip 兜底（见仓库根目录 requirements.txt）。
set -euo pipefail

REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
NO_ROS=0
CHECK_ONLY=0
WITH_UDEV=0

usage() {
    sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --check) CHECK_ONLY=1 ;;
        --no-ros) NO_ROS=1 ;;
        --with-udev) WITH_UDEV=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "未知参数: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if ! command -v apt-get >/dev/null 2>&1; then
    echo "错误：本脚本只支持 apt（Ubuntu/Debian）。" >&2
    echo "其他系统请手动安装：python3-opencv numpy scipy pyzbar pyserial pytest v4l-utils" >&2
    exit 1
fi
if ! command -v python3 >/dev/null 2>&1; then
    echo "错误：找不到 python3。" >&2
    exit 1
fi

# ---- ROS 发行版：优先 $ROS_DISTRO，否则取 /opt/ros 下最新 ----
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    DISTRO="$ROS_DISTRO"
else
    DISTRO=$(ls -1 /opt/ros 2>/dev/null | sort -V | tail -n 1 || true)
fi
if [[ -z "${DISTRO:-}" || ! -f "/opt/ros/${DISTRO}/setup.bash" ]]; then
    echo "错误：找不到 ROS 2（/opt/ros/<distro>/setup.bash）。" >&2
    echo "请先安装 ROS 2（树莓派推荐 Jazzy），再运行本脚本。" >&2
    exit 1
fi
echo "== ROS 发行版: ${DISTRO}"

APT_PKGS=(
    python3-pip python3-setuptools
    python3-numpy python3-scipy python3-opencv
    python3-serial libzbar0 python3-pytest
    v4l-utils
)
# pyzbar 在部分 Ubuntu 版本没有 apt 包，有就装上，没有就让 pip 兜底。
if apt-cache show python3-pyzbar >/dev/null 2>&1; then
    APT_PKGS+=(python3-pyzbar)
fi

if [[ "$NO_ROS" -eq 0 ]]; then
    APT_PKGS+=(
        "ros-${DISTRO}-ros-base"
        "ros-${DISTRO}-rclpy"
        "ros-${DISTRO}-std-msgs"
        "ros-${DISTRO}-sensor-msgs"
        "ros-${DISTRO}-geometry-msgs"
        "ros-${DISTRO}-rcl-interfaces"
        "ros-${DISTRO}-tf2-ros"
        "ros-${DISTRO}-tf2-geometry-msgs"
        "ros-${DISTRO}-cv-bridge"
        "ros-${DISTRO}-ament-index-python"
        "ros-${DISTRO}-launch-ros"
    )
fi

apt_install() {
    local pkg="$1"
    if dpkg -s "$pkg" >/dev/null 2>&1; then
        echo "  · $pkg 已安装"
        return 0
    fi
    if sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "$pkg"; then
        echo "  ✓ $pkg"
        return 0
    fi
    echo "  ✗ $pkg 安装失败" >&2
    return 1
}

if [[ "$CHECK_ONLY" -eq 0 ]]; then
    echo "== apt-get update"
    sudo apt-get update
    echo "== 安装 apt 依赖（已安装的会跳过）"
    FAILED=()
    for pkg in "${APT_PKGS[@]}"; do
        apt_install "$pkg" || FAILED+=("$pkg")
    done
    if [[ ${#FAILED[@]} -gt 0 ]]; then
        echo "以下 apt 包安装失败（可能当前源里没有，稍后用 pip 兜底或手动处理）：" >&2
        printf '  - %s\n' "${FAILED[@]}" >&2
    fi

    if [[ "$WITH_UDEV" -eq 1 && -f "$REPO/udev/99-camera-symlinks.rules" ]]; then
        echo "== 安装 udev 固定名规则"
        sudo install -m 0644 "$REPO/udev/99-camera-symlinks.rules" /etc/udev/rules.d/
        sudo udevadm control --reload-rules
        sudo udevadm trigger --action=add --subsystem-match=video4linux
        sudo udevadm trigger --action=add --subsystem-match=tty
    fi
fi

# ---- 校验 Python 模块 ----
# ROS 的 setup.bash 会引用未定义变量，source 期间临时关闭 -u。
set +u
# shellcheck disable=SC1090
source "/opt/ros/${DISTRO}/setup.bash"
set -u

check_modules() {
    python3 - <<'PY'
import importlib

MODULES = [
    "numpy", "scipy", "cv2", "pyzbar", "serial", "pytest",
    "rclpy", "cv_bridge", "tf2_ros", "tf2_geometry_msgs",
    "geometry_msgs", "sensor_msgs", "std_msgs", "rcl_interfaces",
    "ament_index_python", "launch", "launch_ros",
]
missing = []
for name in MODULES:
    try:
        importlib.import_module(name)
    except Exception:
        missing.append(name)
print(" ".join(missing))
PY
}

echo "== 校验 Python 依赖"
MISSING=$(check_modules)

if [[ -z "$MISSING" ]]; then
    echo "  ✓ 所有 Python 模块可用"
else
    echo "  缺少模块: $MISSING"

    # 纯 Python 包用 pip 兜底；OpenCV / ROS 模块不用 pip（避免 ABI / 链接冲突）。
    PIP_ARGS=()
    for module in $MISSING; do
        case "$module" in
            numpy) PIP_ARGS+=("numpy<2") ;;
            scipy) PIP_ARGS+=("scipy") ;;
            pyzbar) PIP_ARGS+=("pyzbar") ;;
            serial) PIP_ARGS+=("pyserial") ;;
            pytest) PIP_ARGS+=("pytest") ;;
        esac
    done

    if [[ ${#PIP_ARGS[@]} -gt 0 ]]; then
        echo "== pip 兜底安装: ${PIP_ARGS[*]}"
        # Ubuntu 24.04 的 PEP 668：先尝试 --user，必要时加 --break-system-packages。
        if ! python3 -m pip install --user "${PIP_ARGS[@]}"; then
            python3 -m pip install --user --break-system-packages "${PIP_ARGS[@]}"
        fi
    fi

    MISSING=$(check_modules)
    if [[ -n "$MISSING" ]]; then
        echo "仍有模块不可用: $MISSING" >&2
        echo "ROS 相关请按发行版补齐，例如：" >&2
        echo "  sudo apt-get install ros-${DISTRO}-cv-bridge ros-${DISTRO}-tf2-ros \\" >&2
        echo "       ros-${DISTRO}-tf2-geometry-msgs ros-${DISTRO}-sensor-msgs" >&2
        echo "OpenCV 请用 python3-opencv，不要用 pip 版（会和 cv_bridge 冲突）。" >&2
        exit 1
    fi
    echo "  ✓ pip 兜底后所有 Python 模块可用"
fi

cat <<EOF

== 依赖安装完成
下一步：
  cd "${REPO}"
  colcon build --symlink-install --packages-select identify_pack
  source install/setup.bash

检查设备固定名：
  ls -l /dev/camera_front /dev/camera_upper /dev/serial/by-id/

如相机打不开，可把用户加入 video/render 组（重新登录生效）：
  sudo usermod -aG video,render "\$USER"
EOF
