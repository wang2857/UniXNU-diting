# iOS 18 基线计划（分支 `ios-18`）

> 状态：规划中，**未实现**。`main` 仍是 iPadOS 18.7.10（22H374）/ arm64。
> 本分支只做 iPhone iOS 18 的移植准备；在设备实测通过前，不要在 README 或发布页宣称“支持 iOS 18”。

## 目标

- 目标系统：iPhone **iOS 18.6.2（22G100）/ Darwin 24 / arm64e（PAC）**
- 身份：`iPhone17,1`（iPhone 16 Pro）或 `iPhone17,2`；与当前 ADT 的 `iPhone17,1` 方向一致
- 参考实现：Pong（Nothing Phone 2，SM8475）的 UniXNU 记录
  - `board.toml` 选 `ios-18.6.2-22G100` + `arm64e`
  - 已观察到 SIM 身份、网络注册、IPv4/IPv6 WAN、纯蜂窝 HTTP 200；calls/SMS/call audio 仍未完成
- 不在本分支范围：改变 `main` 上已验证的 iPadOS 状态。所有改动只进 `ios-18`，设备验证后再 PR 到 `main`

## 关键差距（先解决再动代码）

1. **OS profile 缺失**
   - UniXNU 上游 `profiles/os` 目前只有 `ipados-17.7.11-21H461`、`ipados-26.5.2-23F84`、`ipados-27.0.1-24A446`、`macos-27.3-26C5035r`，没有 `ios-18.6.2-22G100`。
   - Pong 的设备仓 `Chengdong-Yu/UniXNU-pong` 目前 404（可能私有/改名）；记录页可用，但 profile 文件需向作者/社区获取，或按记录重建。
   - 在拿到 profile 前，不伪造 `os-profiles/ios-18.6.2-22G100/profile.toml`，也不把 `board.toml` 的 `os_profile` 切过去。
2. **arm64e / PAC**
   - 当前 Diting 是 arm64；iPhone iOS 18 需要 arm64e。
   - Pong 已证明 SM8475 可跑 arm64e，但必须在本机复核 PAC 没有被 hypervisor 拦截；对应 UniXNU `profiles/arch/arm64e`。
3. **IPSW 与身份**
   - 需要 `iPhone17,1`/`iPhone17,2` 的 iOS 18.6.2（22G100）restore，替换 `[ipsw]`；J171AP 的 iPadOS 输入不能再用于 iOS 目标。
4. **数据与分区**
   - iOS 目标同样写 userdata 窗口；换 profile 可能要求重建 Data/卷 UUID；先备份，卷 UUID 必须与内核配置一致。

## 工作项

- [ ] 从 Pong 作者/社区取得或重建 `ios-18.6.2-22G100` profile（XNU pin、补丁序列、ABI、prelink roots、IPSW 固定）。
- [ ] 新增 `configs/diting_ios18_defconfig`；`arch:arm64e` + `os:ios` 片段；保留 `soc:configs/sm8475.config`。
- [ ] `board.toml` 增加 `[os_overrides."ios-18.6.2-22G100"]`，不改主线默认；`kernel.defconfig` 指向新 defconfig。
- [ ] 更新 `[ipsw]` 为 iPhone17,x 的 18.6.2 / 22G100；ADT 身份保持 `iPhone17,1`，`artwork-device-idiom = phone`。
- [ ] arm64e 内核构建 + prelink + arm64e kext 输入；先用最小 payload 验证内核入口与平台专家。
- [ ] UFS/卷窗口：按 iOS profile 的 `[userdata] windows` 重排；TWRP 读 UUID；备份现有 Data。
- [ ] 驱动复用 `DitingPlatform24`（A730/QCA6490/UFS）、Goodix GT9916、CS35L41；按 [GAP-ANALYSIS.md](GAP-ANALYSIS.md) 尽量回到官方 HID/显示/打包路径，不再新增自造层。
- [ ] 显示：1220×2712、120Hz、DSC/背光；确认 iOS 缩放与 phone idiom。
- [ ] 构建/CI：`scripts/package.sh --os ios-18.6.2-22G100`；`package.yml` 增加该 profile 的可选输入；不动模板脚本。
- [ ] `porting.toml` 记录 iOS 各区域（configuration/firmware/boot/display/input/GPU/network/audio/power），状态从 `planned` 起，只有真机日志才升到 `device-verified`。
- [ ] 设备验收：进 iOS 桌面、触控、120Hz、GPU/Metal、Wi-Fi；记录 calls/SMS/call audio 的已知缺口。
- [ ] 验收通过后再 PR 到 `main`；README 只写“iOS 18 基线已设备验证”当且仅当有日志。

## 风险

- profile 不可得：没有 `ios-18.6.2-22G100` 就只剩“重建 profile”一条路，工作量大幅上升。
- arm64e/PAC：SM8475 的 PAC 是否被 hypervisor 完全放开需实测；Pong 成功不等于 Diting 自动成功。
- iOS 用户态/ADT：iPhone 身份、MobileGestalt、基带/电话服务与 iPadOS 差异大；calls/SMS 在 Pong 也未完成。
- 数据：写 userdata 会清 Android 数据；保留 boot_b/TWRP 与原厂线刷包。
- 时间：按“周”计，不是一次提交能完成；分支先保留计划，不宣称可用。

## 参考

- Pong 设备记录：https://unixnu.com/zh/devices/pong
- UniXNU arm64e：https://unixnu.com/zh/porting/arm64e
- 设备仓库标准：https://unixnu.com/zh/porting/device-repository
- 适配记录：https://unixnu.com/zh/porting/records
- 差距审计：[GAP-ANALYSIS.md](GAP-ANALYSIS.md)
- 当前状态：[STATUS.md](STATUS.md)
