"""路径集中：相对仓库根，可用环境变量覆盖。"""

from __future__ import annotations

import os
from pathlib import Path

HERE = Path(__file__).resolve().parent          # src/rm27_assembly
ROOT = Path(os.environ.get("RM27_ASSEMBLY_ROOT", HERE.parents[1])).resolve()

MODEL_DIR = ROOT / "model"
SCENE = Path(os.environ.get("RM27_SCENE", MODEL_DIR / "physical" / "physical_scene.xml"))
NPZ = Path(os.environ.get("RM27_NPZ", ROOT / "data" / "rm27_task.npz"))

WEB_DIR = ROOT / "web"
WEB_INDEX = WEB_DIR / "index.html"
WEB_SCENE_JSON = WEB_DIR / "scene.json"

SHM_NAME = os.environ.get("RM27_SHM", "rm27_shm")
PORT = int(os.environ.get("RM27_PORT", "8005"))
