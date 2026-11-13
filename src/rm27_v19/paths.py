"""v1.9 路径与配置。"""

from __future__ import annotations

import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = Path(os.environ.get("RM27_V19_ROOT", HERE.parents[1])).resolve()

SCENE = Path(os.environ.get("RM27_V19_SCENE", ROOT / "model" / "v19" / "scene_v19.xml"))
WEB = ROOT / "web"
WEB_INDEX = WEB / "index.html"
WEB_SCENE_JSON = WEB / "scene.json"

SHM_NAME = os.environ.get("RM27_V19_SHM", "rm27_v19_shm")
PORT = int(os.environ.get("RM27_V19_PORT", "8009"))

# 夹爪（米，滑轨坐标）
GRIP_OPEN = 0.022
GRIP_CLOSE = -0.005

# 关节/底座范围（与场景一致）
CHASSIS_X = (-1.6, 1.6)
CHASSIS_Y = (-1.6, 1.6)
CHASSIS_YAW = (-3.10, 3.10)

# 判定阈值
DEPTH_OK = 0.095         # 插入 ≥95 mm
AXIS_OK = 3.0            # 轴对齐 <3°
P_OK = 2.0               # P 90° ±2°
Q_TARGET = 51.6          # Q 目标角
Q_OK = 2.0
HOLD_OK = 1.0            # 保持 1 s
