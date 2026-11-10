"""RM27 真机姿态镜像 · 串口帧协议（纯函数，无第三方依赖）。

帧格式来自控制板固件 `User/gravity/gravity_capture.c`：

    A5 5A | uint16 seq | uint32 tick(ms) | float q[6] | float tau[6] | CRC16
      2        2            4                24             24          2   = 58 字节

    q[6]   : 关节角（rad），URDF/DH 模型坐标（固件已用 GravityComp_EncoderToModelQ 换算）
    tau[6] : 反馈力矩（N·m），已乘 torque_sign
    CRC16  : poly 0x8408、init 0xFFFF，覆盖前 56 字节（固件 CRC8_CRC16.c）
"""

from __future__ import annotations

import struct
from typing import Iterator, NamedTuple

MAGIC = b"\xa5\x5a"
FRAME_LEN = 58
CRC_LEN = 2
PAYLOAD_LEN = FRAME_LEN - CRC_LEN  # 56

Q_OFFSET = 8
Q_LEN = 24
TAU_OFFSET = 32
TAU_LEN = 24

JOINT_COUNT = 6


def crc16(data: bytes) -> int:
    """CRC-16/X-25 风格（poly 0x8408 反射、init 0xFFFF），与固件一致。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc >> 1) ^ 0x8408) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def frame_ok(frame: bytes) -> bool:
    """整帧校验：长度 + 帧头 + CRC16。"""
    if len(frame) != FRAME_LEN or not frame.startswith(MAGIC):
        return False
    expected = frame[56] | (frame[57] << 8)
    return crc16(frame[:PAYLOAD_LEN]) == expected


class Frame(NamedTuple):
    seq: int
    tick_ms: int
    q: tuple[float, ...]   # 6 个关节角（rad，模型坐标）
    tau: tuple[float, ...]  # 6 个反馈力矩（N·m）


def decode(frame: bytes) -> Frame:
    """解析一帧；不做校验，调用方用 frame_ok() 先判断。"""
    if len(frame) != FRAME_LEN:
        raise ValueError(f"帧长度应为 {FRAME_LEN}，实际 {len(frame)}")
    if not frame.startswith(MAGIC):
        raise ValueError("帧头不是 A5 5A")
    seq = int.from_bytes(frame[2:4], "little")
    tick = int.from_bytes(frame[4:8], "little")
    q = struct.unpack("<6f", frame[Q_OFFSET:Q_OFFSET + Q_LEN])
    tau = struct.unpack("<6f", frame[TAU_OFFSET:TAU_OFFSET + TAU_LEN])
    return Frame(seq, tick, q, tau)


def encode(seq: int, tick_ms: int, q, tau) -> bytes:
    """构造一帧（测试与自检用；固件端是发送方，这里只需能编码）。"""
    if len(q) != JOINT_COUNT or len(tau) != JOINT_COUNT:
        raise ValueError("q/tau 必须是 6 个元素")
    frame = bytearray(FRAME_LEN)
    frame[0:2] = MAGIC
    frame[2:4] = int(seq & 0xFFFF).to_bytes(2, "little")
    frame[4:8] = int(tick_ms & 0xFFFFFFFF).to_bytes(4, "little")
    frame[Q_OFFSET:Q_OFFSET + Q_LEN] = struct.pack("<6f", *[float(v) for v in q])
    frame[TAU_OFFSET:TAU_OFFSET + TAU_LEN] = struct.pack("<6f", *[float(v) for v in tau])
    crc = crc16(bytes(frame[:PAYLOAD_LEN]))
    frame[56], frame[57] = crc & 0xFF, (crc >> 8) & 0xFF
    return bytes(frame)


def iter_frames(stream: Iterator[bytes], check_crc: bool = True) -> Iterator[Frame]:
    """从字节块流里切帧；坏帧/CRC 错帧直接丢弃。"""
    buf = bytearray()
    for chunk in stream:
        if chunk:
            buf += chunk
        while True:
            idx = buf.find(MAGIC)
            if idx < 0:
                buf.clear()
                break
            if idx:
                del buf[:idx]
            if len(buf) < FRAME_LEN:
                break
            frame = bytes(buf[:FRAME_LEN])
            del buf[:FRAME_LEN]
            if check_crc and not frame_ok(frame):
                continue
            yield decode(frame)
