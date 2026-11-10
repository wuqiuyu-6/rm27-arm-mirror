# Changelog

## [0.1.0] - 2026-11-10

### Added
- 串口帧解析（58 字节，`A5 5A` 帧头，CRC16 校验）
- MuJoCo 六轴臂实时镜像（`view` / `print` / `record` 三种模式）
- 六轴符号与零位配置（`config/joints.yaml`）
- 协议与标定文档、协议单测（含逐位翻错检测）与映射单测

### Known issues
- 仿真模型 joint3 的关节范围与实机不一致，默认不夹限位
- 仅支持 Windows 串口名（COMx）与 pyserial 后端
