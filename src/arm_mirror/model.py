"""仿真模型：加载 MJCF、把真机关节角映射成仿真关节角。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_MODEL = REPO_ROOT / "model" / "arm_v7" / "arm_v7_scene.xml"

JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


@dataclass(frozen=True)
class JointMap:
    """sim_q = sign * raw_q + zero（rad）。

    sign/zero 来自实机标定：先整体取反，再按关节修单个轴。
    """
    sign: tuple[float, ...] = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
    zero: tuple[float, ...] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if len(self.sign) != 6 or len(self.zero) != 6:
            raise ValueError("sign/zero 必须是 6 个值")

    @classmethod
    def parse(cls, sign: str, zero: str) -> "JointMap":
        s = tuple(float(v) for v in sign.split(","))
        z = tuple(float(v) for v in zero.split(","))
        return cls(sign=s, zero=z)  # type: ignore[arg-type]

    def apply(self, raw) -> np.ndarray:
        return np.asarray(raw, dtype=float) * np.asarray(self.sign) + np.asarray(self.zero)


class ArmModel:
    """包一层 MuJoCo，只做本工程需要的几件事。"""

    def __init__(self, xml: Path | str = DEFAULT_MODEL):
        import mujoco  # 延迟导入，单测协议时不需要 GL

        self._mj = mujoco
        self.path = Path(xml)
        self.model = mujoco.MjModel.from_xml_path(str(self.path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.joint_names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
            for j in range(self.model.njnt)
        ]
        if self.model.njnt != 6:
            raise RuntimeError(f"期望 6 个关节，模型里是 {self.model.njnt} 个")

        self.lo, self.hi = self._ranges()
        self.has_range = self.hi > self.lo

    def _ranges(self):
        lo, hi = [], []
        for j in range(self.model.njnt):
            r = self.model.jnt_range[j]
            lo.append(float(r[0]))
            hi.append(float(r[1]))
        lo_a, hi_a = np.array(lo), np.array(hi)
        # 无范围的关节（如 joint4 连续）给一个软范围，方便可视化/限幅
        lo_a[hi_a <= lo_a], hi_a[hi_a <= lo_a] = -3.0, 3.0
        return lo_a, hi_a

    def set_joints(self, q) -> None:
        self.data.qpos[:6] = np.asarray(q, dtype=float)
        self.data.qvel[:] = 0.0
        self._mj.mj_forward(self.model, self.data)

    def clamp(self, q) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=float), self.lo, self.hi)

    def outside(self, q) -> list[str]:
        q = np.asarray(q, dtype=float)
        return [self.joint_names[i] for i in range(6) if not (self.lo[i] - 1e-9 <= q[i] <= self.hi[i] + 1e-9)]
