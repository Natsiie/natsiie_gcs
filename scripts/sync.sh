#!/usr/bin/env bash
# 联调同步仪表盘（Linux / Jetson 侧）
# 用法:
#   ./scripts/sync.sh             # 先 fetch 再显示
#   ./scripts/sync.sh --no-fetch  # 不联网，只看本地已知的 origin/main
set -u

FETCH=1
[ "${1:-}" = "--no-fetch" ] && FETCH=0

[ -d .git ] || { echo "✗ 当前目录不是 git 仓库: $(pwd)"; exit 1; }

if [ "$FETCH" = 1 ]; then
  echo "== 正在 fetch origin ..."
  git fetch --prune origin || true
fi

branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null)
short=$(git rev-parse --short HEAD 2>/dev/null)
read -r ahead behind <<<"$(git rev-list --left-right --count HEAD...origin/main 2>/dev/null)"

echo
echo "== 当前分支: ${branch} (${short})"
echo "   相对 origin/main:  领先 ${ahead:-?}, 落后 ${behind:-?}"

dirty=$(git status --porcelain 2>/dev/null | wc -l | tr -d ' ')
[ "${dirty:-0}" -gt 0 ] && echo "   ⚠ 工作区有 ${dirty} 处未提交改动（联调前先处理干净）"

classify() {  # 从 stdin 读改动文件列表
  local fw=0 vis=0 doc=0 proto=0 other=0
  while IFS= read -r f; do
    [ -z "$f" ] && continue
    case "$f" in
      stm32_firmware/Devices/Serial.c|stm32_firmware/Devices/Serial.h|src/identify_pack/identify_pack/serial/*) proto=1 ;;
    esac
    case "$f" in
      stm32_firmware/*) fw=1 ;;
      src/*|udev/*|scripts/*) vis=1 ;;
      DEVLOG.md|PROTOCOL.md|README*) doc=1 ;;
    esac
  done
  local out=""
  [ "$proto" = 1 ] && out="${out}协议+"
  [ "$fw" = 1 ] && out="${out}固件+"
  [ "$vis" = 1 ] && out="${out}视觉+"
  [ "$doc" = 1 ] && out="${out}文档+"
  [ -z "$out" ] && out="其他"
  echo "${out%+}"
}

if [ "${behind:-0}" -gt 0 ]; then
  echo
  echo "== origin/main 上你还没拿到的 ${behind} 个提交（新 → 旧）"
  seen_proto=0; seen_vis=0
  while IFS='|' read -r sha date author subject; do
    [ -z "$sha" ] && continue
    tag=$(git show --name-only --pretty=format: "$sha" 2>/dev/null | classify)
    case "$tag" in *协议*) seen_proto=1 ;; esac
    case "$tag" in *视觉*) seen_vis=1 ;; esac
    printf '   [%-8s] %s %s  %s\n' "$tag" "$sha" "$date" "$subject"
  done < <(git log --pretty=format:'%h|%ad|%an|%s' --date=short HEAD..origin/main 2>/dev/null)
  echo
  echo "== 建议"
  echo "   git pull --rebase origin main"
  [ "$seen_vis" = 1 ] && echo "   colcon build --packages-select identify_pack && source install/setup.bash   # 视觉侧有更新"
  [ "$seen_proto" = 1 ] && echo "   ⚠ 有协议相关改动 → 先看 PROTOCOL.md 是否同步，再烧录/重跑"
fi

if [ "${ahead:-0}" -gt 0 ]; then
  echo
  echo "== 你本地有 ${ahead} 个提交还没推上去"
  git log --pretty=format:'   %h %ad %an  %s' --date=short origin/main..HEAD
  echo
  echo "   git push origin HEAD"
fi

if [ "${behind:-0}" -eq 0 ] && [ "${ahead:-0}" -eq 0 ]; then
  echo
  echo "== 已与 origin/main 一致"
fi
