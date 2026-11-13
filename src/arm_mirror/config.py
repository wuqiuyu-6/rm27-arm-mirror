"""配置读取：六轴映射（sign/zero）与串口参数。

`config/joints.yaml` 是唯一来源；CLI 参数只做临时覆盖。
不依赖 PyYAML —— 本文件结构固定（两层键值），用极简解析即可。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "config" / "joints.yaml"

JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


def _scalar(text: str) -> Any:
    text = text.strip()
    try:
        return float(text) if any(c in text for c in ".eE") else int(text)
    except ValueError:
        return text


def parse(text: str) -> dict[str, dict[str, Any]]:
    """解析两层 YAML（section: 下面一排 key: value），忽略注释与空行。"""
    data: dict[str, dict[str, Any]] = {}
    section: str | None = None
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if not line.startswith((" ", "\t")) and line.rstrip().endswith(":"):
            section = line.strip()[:-1]
            data[section] = {}
            continue
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        if section is None:
            continue
        data[section][key.strip()] = _scalar(value)
    return data


def load(path: Path | str | None = None) -> dict[str, dict[str, Any]]:
    p = Path(path) if path else DEFAULT_CONFIG
    if not p.is_file():
        raise FileNotFoundError(f"找不到配置文件 {p}")
    return parse(p.read_text(encoding="utf-8"))


def joint_map(path: Path | str | None = None) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """返回 (sign, zero)，各 6 个值；缺项按恒等/零处理。"""
    cfg = load(path)
    sign_cfg = cfg.get("sign", {})
    zero_cfg = cfg.get("zero", {})
    sign = tuple(float(sign_cfg.get(n, 1.0)) for n in JOINT_NAMES)
    zero = tuple(float(zero_cfg.get(n, 0.0)) for n in JOINT_NAMES)
    return sign, zero


def serial(path: Path | str | None = None) -> dict[str, Any]:
    cfg = load(path).get("serial", {})
    return dict(port=str(cfg.get("port", "COM8")), baud=int(cfg.get("baud", 115200)))
