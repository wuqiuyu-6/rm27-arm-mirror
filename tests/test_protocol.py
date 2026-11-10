"""协议单测：造帧 -> 解析 -> CRC，含随机帧与坏帧注入。"""

from __future__ import annotations

import random
import struct

import pytest

from arm_mirror import protocol


def test_round_trip():
    frame = protocol.encode(7, 123456, [0.1, -0.2, 0.3, 1.2, 0.0, 0.0],
                            [1.0, -2.0, 3.0, 0.5, 0.0, 0.0])
    assert len(frame) == protocol.FRAME_LEN
    assert frame.startswith(protocol.MAGIC)
    assert protocol.frame_ok(frame)

    decoded = protocol.decode(frame)
    assert decoded.seq == 7
    assert decoded.tick_ms == 123456
    assert decoded.q == pytest.approx([0.1, -0.2, 0.3, 1.2, 0.0, 0.0], abs=1e-6)
    assert decoded.tau == pytest.approx([1.0, -2.0, 3.0, 0.5, 0.0, 0.0], abs=1e-6)


def test_random_round_trip():
    rng = random.Random(20261110)
    for _ in range(1000):
        q = [rng.uniform(-3.0, 3.0) for _ in range(6)]
        tau = [rng.uniform(-10.0, 10.0) for _ in range(6)]
        frame = protocol.encode(rng.randrange(65536), rng.randrange(2**32), q, tau)
        assert protocol.frame_ok(frame)
        d = protocol.decode(frame)
        assert d.q == pytest.approx(q, abs=1e-5)
        assert d.tau == pytest.approx(tau, abs=1e-4)


def test_crc_detects_any_single_byte_flip():
    """56 字节 × 8 位，每一位翻转都必须被 CRC 检出。"""
    frame = bytearray(protocol.encode(1, 2, [0.1] * 6, [0.2] * 6))
    assert protocol.frame_ok(bytes(frame))
    for byte in range(protocol.PAYLOAD_LEN):
        for bit in range(8):
            bad = bytearray(frame)
            bad[byte] ^= 1 << bit
            assert not protocol.frame_ok(bytes(bad)), f"翻位未被检出: byte={byte} bit={bit}"


def test_rejects_bad_header_and_length():
    good = protocol.encode(1, 2, [0.0] * 6, [0.0] * 6)
    assert not protocol.frame_ok(b"\x00" + good[1:])
    assert not protocol.frame_ok(good[:-1])
    assert not protocol.frame_ok(good + b"\x00")


def test_iter_frames_skips_garbage_and_partial():
    a = protocol.encode(1, 10, [0.0] * 6, [0.0] * 6)
    b = protocol.encode(2, 20, [0.5] * 6, [0.0] * 6)
    stream = [b"garbage", a[:20], a[20:] + b[:10], b[10:]]
    frames = list(protocol.iter_frames(iter(stream)))
    assert [f.seq for f in frames] == [1, 2]


def test_iter_frames_drops_bad_crc():
    good = protocol.encode(1, 10, [0.0] * 6, [0.0] * 6)
    bad = bytearray(good)
    bad[10] ^= 0xFF
    frames = list(protocol.iter_frames(iter([bytes(bad) + good])))
    assert [f.seq for f in frames] == [1]
