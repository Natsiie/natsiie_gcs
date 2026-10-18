## 改了什么

<!-- 一句话说清楚；如果是联调修复，写清楚现象和根因 -->

## 影响面（必勾）

- [ ] **动了串口协议**（TYPE 号 / payload / 语义）→ 必须同步更新 `PROTOCOL.md`，并在此说明：改了什么、对方要做什么
- [ ] 只动下位机 `stm32_firmware/`
- [ ] 只动上位机 `src/`
- [ ] 部署/环境（`udev/`、`scripts/`、systemd、标定参数）

## 验证情况（必勾，没验证的写"未验证"）

- [ ] 固件编译通过（Keil，或 `tools/keil-build.ps1`）
- [ ] 上位机 `colcon build --packages-select identify_pack` 通过
- [ ] 相关单测通过
- [ ] **实机联调验证过** → 已在 `DEVLOG.md` 记录双方 commit 与结果
- [ ] 仅桌面/离线验证，未上实机

## 需要对方做什么

<!-- 例如：重新烧录固件 / 重启 serial_bridge / 重跑相机标定 / 什么都不用做 -->
