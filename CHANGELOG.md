# Changelog

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
