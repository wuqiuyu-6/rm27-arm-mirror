# 串口帧协议

## 出处

控制板固件仓库 `3-20-new-controller-arm` 的发送端实现：

- `User/gravity/gravity_capture.c` —— 组帧并 `HAL_UART_Transmit_DMA`
- `User/motor/dm4310_drv.c` —— CAN 反馈（DM4310 MIT 模式）解码，帧里 `q`/`tau` 的来源
- `User/gravity/gravity_config.h` —— 编码器角到模型角的标定（`encoder_zero` / `position_sign`）
- `User/2-MWL(Middleware Layer)/src/CRC8_CRC16.c` —— CRC16 表与 `append_CRC16_check_sum`

## 帧结构（58 字节）

| 偏移 | 长度 | 类型 | 含义 |
|---|---|---|---|
| 0 | 2 | bytes | 帧头 `A5 5A` |
| 2 | 2 | uint16 LE | 序号 `seq`，每帧 +1 |
| 4 | 4 | uint32 LE | 固件 `HAL_GetTick()` 毫秒 |
| 8 | 24 | float32 LE × 6 | 关节角 `q[6]`（rad，模型坐标） |
| 32 | 24 | float32 LE × 6 | 反馈力矩 `tau[6]`（N·m） |
| 56 | 2 | uint16 LE | CRC16（覆盖前 56 字节） |

## CRC16

- 多项式：`0x8408`（反射形式，等价 CRC-16/X-25 风格）
- 初值：`0xFFFF`
- 无最终异或
- 覆盖范围：前 56 字节（不含 CRC 自身）

实现见 `src/arm_mirror/protocol.py::crc16`，单测覆盖"任意单字节任一位翻转必须被检出"。

## 关节角坐标系

固件在发帧前调用 `GravityComp_EncoderToModelQ()`，把 DM4310 反馈的电机角换算成 **URDF/DH 模型坐标**
（`q_DH = q_URDF`，见固件 README）。因此：

- `q[0..5]` 就是 joint1..joint6 的模型角，**不需要再套编码器零位/符号**；
- 与仿真模型之间的方向差异（整体反向、个别轴差 180°）在 `config/joints.yaml` 里补偿，见
  [`calibration.md`](calibration.md)。

## 帧率与带宽

- 实测约 **32 Hz**（1996 B/s @ 115200）
- 理论最小 58 B/帧 × 32 Hz ≈ 1.9 kB/s，115200 8N1 上限约 11.5 kB/s，余量充足
