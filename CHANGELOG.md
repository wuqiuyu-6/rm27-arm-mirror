# Changelog

## [0.2.2] - 2026-11-26

### Added
- Ubuntu 24.04 支持：examples/*.sh 启动脚本、README Ubuntu 章节
- 串口自动探测（Windows 取 COM 口，Linux 优先 /dev/serial/by-id/*）
- GitHub Actions CI（ubuntu-latest，Python 3.10 / 3.12）

### Changed
- 默认模型改为 **无夹爪** 的 rm_v7_scene.xml；带夹爪模型改用 --model 指定
- 两个模型的关节范围都按固件电机限位放宽（J2 −1.0~2.4、J3 −0.8~2.2、J4 ±3.0）
- 重写 README 与 docs/calibration.md：修正串口说明、去掉过时的夹爪章节、
  补充已知的 sign/zero 与固件标定值不一致的问题

### Fixed
- 网页端网格四元数顺序错误（MuJoCo w,x,y,z 被当成 three.js x,y,z,w），
  导致连杆朝向错乱、看起来像模型散架
- 相机缺少 up = +Z，模型被渲染成躺倒

## [0.2.1] - 2026-11-13

### Changed
- 串口默认改为 **COM9**（config/joints.yaml 的 serial.port）
- CLI 与网页端未显式指定端口时都从配置读取
- 启动 bat 不再写死 COM6


## [0.2.0] - 2026-11-11

### Changed
- config/joints.yaml 成为六轴映射的**唯一来源**：CLI 与网页端都从它读取
- CLI 默认值改为“读配置”，--sign/--zero/--flip 仅作临时覆盖
- 网页端不再硬编码 sign/zero
- 修正 joints.yaml 中关于 joint3 零位的过时注释

### Fixed
- 桌面端 bat 未带 --flip 导致方向与实机相反


## [0.1.0] - 2026-11-10

### Added
- 串口帧解析（58 字节，`A5 5A` 帧头，CRC16 校验）
- MuJoCo 六轴臂实时镜像（`view` / `print` / `record` 三种模式）
- 六轴符号与零位配置（`config/joints.yaml`）
- 协议与标定文档、协议单测（含逐位翻错检测）与映射单测

### Known issues
- 仿真模型 joint3 的关节范围与实机不一致，默认不夹限位
- 仅支持 Windows 串口名（COMx）与 pyserial 后端
