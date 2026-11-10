"""串口读取：把 COM 口变成协议帧流。"""

from __future__ import annotations

from typing import Iterator

import serial
import serial.tools.list_ports

from .protocol import Frame, iter_frames

DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 115200


def available_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


def describe_ports() -> str:
    ports = serial.tools.list_ports.comports()
    return ", ".join(f"{p.device} ({p.description})" for p in ports) or "（无）"


def read_frames(port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
                check_crc: bool = True, read_size: int = 512) -> Iterator[Frame]:
    """持续读帧；端口打不开时抛 SerialException。"""
    with serial.Serial(port, baud, timeout=0.05) as ser:
        def chunks() -> Iterator[bytes]:
            while True:
                data = ser.read(read_size)
                if data:
                    yield data
        yield from iter_frames(chunks(), check_crc=check_crc)
