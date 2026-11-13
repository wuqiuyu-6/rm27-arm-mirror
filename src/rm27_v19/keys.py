"""v1.9 键盘手动装配：键位与速度积分（纯函数，无 MuJoCo 依赖）。"""

from __future__ import annotations

from dataclasses import dataclass, field

# 键位表（与页面一致）
KEYMAP = {
    "w": ("chassis_y", +1), "s": ("chassis_y", -1),
    "d": ("chassis_x", +1), "a": ("chassis_x", -1),
    "e": ("chassis_yaw", +1), "q": ("chassis_yaw", -1),
    "g": ("gripper", -1), "h": ("gripper", +1),   # g = 合（负方向），h = 开
    "z": ("p_axis", +1), "x": ("p_axis", -1),
    "c": ("q_axis", +1), "v": ("q_axis", -1),
}

# 速度（单位：米/秒、弧度/秒）
SPEED = dict(chassis_x=0.15, chassis_y=0.15, chassis_yaw=0.8,
             gripper=0.10, p_axis=0.5, q_axis=0.5,
             joint=0.5)

SLOW = 0.2          # 按住 Shift 的速度倍率
RAMP_TIME = 0.15    # 0 → 满速 的斜坡时间


@dataclass
class Command:
    """一次按键状态下算出的速度指令。"""
    chassis_x: float = 0.0
    chassis_y: float = 0.0
    chassis_yaw: float = 0.0
    gripper: float = 0.0
    p_axis: float = 0.0
    q_axis: float = 0.0
    joint: dict[int, float] = field(default_factory=dict)   # 关节号(0~5) -> 速度


def axis_of(key: str) -> tuple[str, int] | None:
    k = key.lower()
    if k in KEYMAP:
        return KEYMAP[k]
    if k in ("arrowup", "arrowdown"):
        return ("joint", +1 if k == "arrowup" else -1)
    return None


def command_from_keys(keys, selected_joint: int, slow: bool = False) -> Command:
    """按键集合 -> 速度指令（不含斜坡与积分）。"""
    cmd = Command()
    scale = SLOW if slow else 1.0
    for key in keys:
        a = axis_of(key)
        if not a:
            continue
        axis, sign = a
        v = SPEED.get(axis, SPEED["joint"]) * sign * scale
        if axis == "joint":
            cmd.joint[selected_joint] = cmd.joint.get(selected_joint, 0.0) + v
        else:
            setattr(cmd, axis, getattr(cmd, axis) + v)
    return cmd


@dataclass
class Ramp:
    """把速度指令平滑成实际速度（避免一按就满速）。"""
    value: float = 0.0

    def step(self, target: float, dt: float) -> float:
        rate = abs(target) / RAMP_TIME if target else 1.0 / 0.15
        step = rate * dt
        if target > self.value:
            self.value = min(target, self.value + step)
        elif target < self.value:
            self.value = max(target, self.value - step)
        return self.value
