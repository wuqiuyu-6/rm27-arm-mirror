"""配置与映射单测：yaml 解析、sign/zero 取值、JointMap 应用。"""

from __future__ import annotations

import numpy as np
import pytest

from arm_mirror.config import JOINT_NAMES, joint_map, parse, serial
from arm_mirror.model import JointMap

SAMPLE = """
# 注释
sign:
  joint1: -1.0
  joint2: -1.0
  joint3: -1.0
  joint4: -1.0
  joint5: -1.0
  joint6: -1.0

zero:
  joint1: 0.0
  joint2: 0.0
  joint3: 3.141592653589793
  joint4: 0.0
  joint5: 0.0
  joint6: 0.0

serial:
  port: COM7
  baud: 921600
"""


def test_parse_two_level():
    data = parse(SAMPLE)
    assert set(data) == {"sign", "zero", "serial"}
    assert data["sign"]["joint1"] == -1
    assert data["zero"]["joint3"] == pytest.approx(np.pi)
    assert data["serial"]["port"] == "COM7"
    assert data["serial"]["baud"] == 921600


def test_joint_map_from_text(tmp_path):
    p = tmp_path / "joints.yaml"
    p.write_text(SAMPLE, encoding="utf-8")
    sign, zero = joint_map(p)
    assert sign == (-1.0,) * 6
    assert zero[2] == pytest.approx(np.pi)
    assert serial(p) == {"port": "COM7", "baud": 921600}


def test_joint_map_defaults_when_missing(tmp_path):
    p = tmp_path / "empty.yaml"
    p.write_text("sign:\n  joint1: -1.0\n", encoding="utf-8")
    sign, zero = joint_map(p)
    assert sign[0] == -1.0 and sign[1] == 1.0     # 缺项按恒等
    assert zero == (0.0,) * 6


def test_repo_config_is_loaded_and_flips_all():
    """仓库自带的 config 应该是六轴全 -1、零位全 0。"""
    sign, zero = joint_map()
    assert sign == (-1.0,) * 6
    assert zero == (0.0,) * 6
    assert len(JOINT_NAMES) == 6


def test_joint_map_apply():
    jm = JointMap(sign=(-1.0,) * 6, zero=(0.0,) * 6)
    out = jm.apply([0.1, -0.2, 0.3, 1.2, 0.0, 0.0])
    assert out == pytest.approx([-0.1, 0.2, -0.3, -1.2, 0.0, 0.0])
