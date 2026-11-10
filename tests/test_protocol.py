"""共享内存协议单测：布局、seqlock 读写、命令区往返。"""

from __future__ import annotations

import numpy as np
import pytest

from rm27_assembly.protocol import TASK, Channel, Layout


def test_layout_grows_with_nu():
    a = Layout(18, 17, 8)
    b = Layout(18, 17, 13)
    assert b.size > a.size
    assert a.off_ctrl == b.off_ctrl
    assert a.off_task == a.off_cmd2 + 32


def test_task_dtype_has_expected_fields():
    assert TASK.names[:4] == ("phase", "playing", "cycle", "lock")
    assert "axis" in TASK.names and "c1" in TASK.names


@pytest.fixture()
def channel():
    ch = Channel(18, 17, 8, name="rm27_test_shm", create=True)
    yield ch
    try:
        ch.close()
        ch.unlink()
    except Exception:
        pass


def test_state_round_trip(channel):
    for k in range(5):
        channel.publish_state(k * 0.001, np.zeros(18), np.zeros(17),
                              task=dict(phase=k, playing=1, cycle=0, lock=0,
                                        depth=1.0, p_deg=0.0, q_deg=0.0, grip=0.0,
                                        axis=0.0, c1=0.0))
        st = channel.read_state()
        assert st is not None
        t, qpos, qvel, task = st
        assert abs(t - k * 0.001) < 1e-12
        assert task["phase"] == k


def test_cmd_round_trip(channel):
    ctrl = np.arange(8, dtype=float)
    ext = np.array([0.1, 1.57, 0.0, 1.0])
    channel.publish_cmd(0.0, ctrl, ext)
    t, got_ctrl, got_ext = channel.read_cmd()
    assert np.allclose(got_ctrl, ctrl)
    assert np.allclose(got_ext, ext)


def test_manual_flag(channel):
    channel.set_manual(True)
    assert channel.manual_mode() is True
    channel.set_manual(False)
    assert channel.manual_mode() is False
