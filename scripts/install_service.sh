#!/usr/bin/env bash
# 按当前机器（用户、用户组、仓库路径）生成并安装 race_start systemd 服务。
# 生成的服务文件在 /etc/systemd/system/，属于机器相关配置，不进入版本控制。
set -euo pipefail

REPO=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
TEMPLATE="${REPO}/scripts/race_start.service.in"
TARGET=/etc/systemd/system/race_start.service
USER_NAME=$(id -un)
GROUP_NAME=$(id -gn)

if [ ! -f "${TEMPLATE}" ]; then
    echo "错误：找不到模板 ${TEMPLATE}" >&2
    exit 1
fi
if [ ! -x "${REPO}/scripts/start_race.sh" ]; then
    echo "错误：${REPO}/scripts/start_race.sh 不存在或不可执行" >&2
    exit 1
fi

TMP=$(mktemp)
trap 'rm -f "${TMP}"' EXIT
sed -e "s|@USER@|${USER_NAME}|g" \
    -e "s|@GROUP@|${GROUP_NAME}|g" \
    -e "s|@REPO@|${REPO}|g" \
    "${TEMPLATE}" > "${TMP}"

echo "安装参数:"
echo "  User=${USER_NAME} Group=${GROUP_NAME}"
echo "  Repo=${REPO}"
sudo install -m 0644 "${TMP}" "${TARGET}"
sudo systemctl daemon-reload
sudo systemctl enable race_start.service
echo "已安装: ${TARGET}"
echo "启动/重启: sudo systemctl restart race_start.service"
echo "查看状态: sudo systemctl status race_start.service"
