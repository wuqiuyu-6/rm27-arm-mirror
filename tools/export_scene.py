"""把装配场景 MJCF 导出成前端能吃的 JSON（web/scene.json）。"""

from __future__ import annotations

import base64
import json
from pathlib import Path

import numpy as np
import mujoco

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "model" / "physical" / "physical_scene.xml"
OUT = ROOT / "web" / "scene.json"

GEOM_NAMES = {
    int(mujoco.mjtGeom.mjGEOM_PLANE): "plane",
    int(mujoco.mjtGeom.mjGEOM_BOX): "box",
    int(mujoco.mjtGeom.mjGEOM_SPHERE): "sphere",
    int(mujoco.mjtGeom.mjGEOM_CAPSULE): "capsule",
    int(mujoco.mjtGeom.mjGEOM_CYLINDER): "cylinder",
    int(mujoco.mjtGeom.mjGEOM_MESH): "mesh",
}


def b64(arr: np.ndarray) -> str:
    return base64.b64encode(arr.tobytes()).decode("ascii")


def export(model_path: Path = MODEL) -> dict:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    bodies = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b) or f"body{b}"
              for b in range(model.nbody)]

    geoms, cache = [], {}
    for g in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if name.startswith("col_"):
            continue
        gtype = GEOM_NAMES.get(int(model.geom_type[g]), "other")
        if gtype not in ("mesh", "box", "sphere", "cylinder", "capsule", "plane"):
            continue
        buf = {}
        if gtype == "mesh":
            mid = int(model.geom_dataid[g])
            if mid not in cache:
                va, vn = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
                fa, fn = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
                verts = np.ascontiguousarray(model.mesh_vert[va:va + vn].astype(np.float32))
                faces = np.ascontiguousarray(model.mesh_face[fa:fa + fn].astype(np.uint32))
                cache[mid] = dict(v=b64(verts), f=b64(faces), nv=int(vn), nf=int(fn))
            buf = cache[mid]
        geoms.append(dict(
            body=int(model.geom_bodyid[g]), name=name, type=gtype,
            pos=[float(x) for x in model.geom_pos[g]],
            quat=[float(x) for x in model.geom_quat[g]],
            size=[float(x) for x in model.geom_size[g]],
            rgba=[float(x) for x in model.geom_rgba[g]],
            **buf,
        ))

    return dict(
        scene="rm27",
        nbody=len(bodies),
        bodies=bodies,
        geoms=geoms,
        nq=int(model.nq), nu=int(model.nu),
        actuator_names=[mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, a) or f"act{a}"
                        for a in range(model.nu)],
        ctrl_range=[[float(lo), float(hi)] for lo, hi in model.actuator_ctrlrange],
        camera=dict(lookat=[0.33, 0.02, 0.30], distance=1.05, azimuth=35.0, elevation=-18.0),
    )


def main() -> None:
    data = export()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(data), encoding="utf-8")
    ntri = sum(g.get("nf", 0) for g in data["geoms"])
    print(f"导出 {OUT}")
    print(f"  body {data['nbody']}  geom {len(data['geoms'])}  三角面 {ntri}")
    print(f"  大小 {OUT.stat().st_size / 1024 / 1024:.2f} MB")


if __name__ == "__main__":
    main()
