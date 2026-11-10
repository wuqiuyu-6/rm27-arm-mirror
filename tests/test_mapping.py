"""映射单测：关节符号/零位、模型范围、越界检测。"""

from __future__ import annotations

import numpy as np
import pytest

from arm_mirror.model import DEFAULT_MODEL, ArmModel, JointMap


def test_joint_map_identity():
    jm = JointMap()
    out = jm.apply([0.1, -0.2, 0.3, 1.2, 0.0, 0.0])
    assert out == pytest.approx([0.1, -0.2, 0.3, 1.2, 0.0, 0.0])


def test_joint_map_flip_and_zero():
    jm = JointMap.parse("-1,-1,-1,-1,-1,-1", "0,0,3.141592653589793,0,0,0")
    out = jm.apply([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert out[2] == pytest.approx(np.pi)
    assert np.allclose(out[[0, 1, 3, 4, 5]], 0.0)


def test_joint_map_rejects_wrong_length():
    with pytest.raises(ValueError):
        JointMap(sign=(1.0,) * 5)


@pytest.mark.skipif(not DEFAULT_MODEL.is_file(), reason="模型文件不存在")
def test_model_loads_six_joints():
    arm = ArmModel()
    assert arm.model.njnt == 6
    assert arm.joint_names == ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    assert len(arm.lo) == 6 and len(arm.hi) == 6


@pytest.mark.skipif(not DEFAULT_MODEL.is_file(), reason="模型文件不存在")
def test_clamp_and_outside():
    arm = ArmModel()
    q = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
    clamped = arm.clamp(q)
    assert np.all(clamped <= arm.hi + 1e-9)
    assert set(arm.outside(q)) == set(arm.joint_names)
