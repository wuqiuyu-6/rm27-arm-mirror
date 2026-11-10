"""重新录制轨迹（可选）。

轨迹 `data/rm27_task.npz` 是用 `run_physical.py --record` 在**建模工作区**里生成的，
那里有完整的 build_physical.py / meshes / 动力学调参。本仓库只带录制结果，不带生成器。

如果你有建模工作区，设置环境变量后运行：

    set RM27_MODEL_WORKSPACE=D:\\workplacedeepseek\\model\\physical
    python tools/record_task.py

它会调用该目录下的 run_physical.py --record，并把产物拷到 data/rm27_task.npz。
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NPZ = ROOT / "data" / "rm27_task.npz"


def main() -> int:
    ws = os.environ.get("RM27_MODEL_WORKSPACE")
    if not ws:
        print("未设置 RM27_MODEL_WORKSPACE，无法重录。")
        print("轨迹是离线生成的，仓库里已有 data/rm27_task.npz，一般不需要重录。")
        return 2

    script = Path(ws) / "run_physical.py"
    if not script.is_file():
        print(f"找不到 {script}")
        return 2

    print(f"调用 {script} --record …")
    rc = subprocess.call([sys.executable, str(script), "--record"], cwd=ws)
    if rc != 0:
        return rc

    produced = Path(ws) / ".." / ".." / "projects" / "omx-sim" / "versions" / "v1.6-rm27" / "vendor" / "rm27_task.npz"
    produced = produced.resolve()
    if produced.is_file():
        NPZ.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(produced, NPZ)
        print(f"已更新 {NPZ}")
        return 0

    print("重录完成，但没找到产物，请手动拷到 data/rm27_task.npz")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
