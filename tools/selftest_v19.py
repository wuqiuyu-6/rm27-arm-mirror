"""v1.9 自检：用 v1.7 的录制轨迹驱动新场景（含销子碰撞），看能否插入并完成 P/Q。"""
import sys
from pathlib import Path

import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

SCENE = ROOT / "model" / "v19" / "scene_v19.xml"
NPZ = ROOT / "data" / "rm27_task.npz"

m = mujoco.MjModel.from_xml_path(str(SCENE))
d = mujoco.MjData(m)
rec = np.load(NPZ)
ctrl = rec["ctrl"]; core = rec["core"]; eqd = rec["eqd"]
mount = rec["mount"]; park = rec["park"]

# 按名字映射执行器（录制里是 13 个，场景里 16 个）
names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
old_names = names[:13]                     # a1..a6, fl, fr, ch_x, ch_y, ch_yaw, fx, fy
aidx = {n: i for i, n in enumerate(names)}

cq = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
      for n in ("c_joint1", "c_joint2", "c_joint3")]
eq_mount = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "mount")
eq_park = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "park")
gp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pin")
bu = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "unit")
uj = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "unit_free")]

d.qpos[:] = rec["qpos0"]
d.qvel[:] = 0
mujoco.mj_forward(m, d)

prev = d.xpos[bu].copy()
worst = 0.0
depth_at = None
for i in range(len(ctrl)):
    for k, n in enumerate(old_names):
        d.ctrl[aidx[n]] = ctrl[i][k]
    # 核心三关节用位置伺服跟踪录制值
    d.ctrl[aidx["c1"]] = core[i][0]
    d.ctrl[aidx["c2"]] = core[i][1]
    d.ctrl[aidx["c3"]] = core[i][2]
    d.eq_active[eq_park] = park[i]
    d.eq_active[eq_mount] = mount[i]
    m.eq_data[eq_mount][0:10] = eqd[i][0:10]
    mujoco.mj_step(m, d)
    p = d.xpos[bu].copy()
    worst = max(worst, float(np.linalg.norm(p - prev)))
    prev = p
    if depth_at is None and mount[i] == 1:
        ax = d.geom_xmat[gp].reshape(3, 3)[:, 2]
        free = d.geom_xpos[gp] - ax * float(m.geom_size[gp][1])
        depth_at = float(np.dot(d.xpos[bu] - free, ax))
        uax = d.xmat[bu].reshape(3, 3)[:, 2]
        axis_at = float(np.degrees(np.arccos(np.clip(abs(np.dot(uax, ax)), -1, 1))))

ax = d.geom_xmat[gp].reshape(3, 3)[:, 2]
free = d.geom_xpos[gp] - ax * float(m.geom_size[gp][1])
depth_end = float(np.dot(d.xpos[bu] - free, ax))
print(f"插入瞬间: 深度 {depth_at*1000:.1f} mm  轴对齐 {axis_at:.1f}°")
print(f"末态:     深度 {depth_end*1000:.1f} mm")
print(f"P {np.degrees(d.qpos[cq[1]]):.1f}°  Q {np.degrees(d.qpos[cq[2]]):.1f}°")
print(f"全程单帧最大跳变 {worst*1000:.2f} mm")
