"""上位机公共工具：共享内存插值 + 等通道。"""

import time
from pathlib import Path

import numpy as np
import mujoco

HERE = Path(__file__).resolve().parent
from .paths import SCENE


def interpolate(model, snapshots, t_render):
    """按关节流形插值，返回 (qpos, qvel, 标记)。"""
    if not snapshots:
        return None, None, "starved"
    if t_render <= snapshots[0][0]:
        _, q, v = snapshots[0]
        return q, v, "underrun"
    if t_render >= snapshots[-1][0]:
        _, q, v = snapshots[-1]
        return q, v, "starved"
    for i in range(len(snapshots) - 1):
        t0, q0, v0 = snapshots[i]
        t1, q1, v1 = snapshots[i + 1]
        if t0 <= t_render <= t1:
            span = t1 - t0
            alpha = 0.0 if span < 1e-9 else (t_render - t0) / span
            dq = np.zeros(model.nv)
            mujoco.mj_differentiatePos(model, dq, 1.0, q0, q1)
            q = q0.copy()
            mujoco.mj_integratePos(model, q, dq, alpha)
            v = v0 + (v1 - v0) * alpha
            return q, v, "ok"
    _, q, v = snapshots[-1]
    return q, v, "starved"


def wait_for_channel(name, nq, nv, nu, timeout=15.0):
    """等服务端把共享内存建起来。"""
    from .protocol import Channel
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        try:
            return Channel(nq, nv, nu, name=name, create=False)
        except FileNotFoundError:
            time.sleep(0.2)
    raise SystemExit(f"等了 {timeout:.0f}s 还没等到通道 '{name}'，服务端起了吗？")
