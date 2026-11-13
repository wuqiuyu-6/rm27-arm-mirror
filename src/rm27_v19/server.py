"""v1.9 物理端：真实碰撞 + 键盘手动装配 + 可选真机镜像。

    1 kHz 物理步进，位置伺服跟踪"目标"；键盘（或真机串口）改变目标。

用法：
    python -m rm27_v19            # 默认 8009
    python -m rm27_v19 --no-serial
    python -m rm27_v19 --serial-port COM6
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np

from . import paths as P
from .keys import Command, Ramp, command_from_keys

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


class Sim:
    def __init__(self, scene: Path = P.SCENE, use_serial: bool = False,
                 serial_port: str = "COM6", baud: int = 115200):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(scene))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self.dt = float(self.model.opt.timestep)

        def jid(n):
            return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)

        def aid(n):
            return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)

        self.joint_q = {n: self.model.jnt_qposadr[jid(n)] for n in JOINTS}
        self.joint_d = {n: self.model.jnt_dofadr[jid(n)] for n in JOINTS}
        self.joint_lo = {n: float(self.model.jnt_range[jid(n)][0]) for n in JOINTS}
        self.joint_hi = {n: float(self.model.jnt_range[jid(n)][1]) for n in JOINTS}
        for n in JOINTS:
            if self.joint_hi[n] <= self.joint_lo[n]:      # joint4 连续
                self.joint_lo[n], self.joint_hi[n] = -3.4, 3.4

        self.act = {n: aid(n) for n in ("a1", "a2", "a3", "a4", "a5", "a6", "fl", "fr")}
        self.chassis = {
            "x": (self.model.jnt_qposadr[jid("chassis_x")], aid("ch_x")),
            "y": (self.model.jnt_qposadr[jid("chassis_y")], aid("ch_y")),
            "yaw": (self.model.jnt_qposadr[jid("chassis_yaw")], aid("ch_yaw")),
        }
        self.fine = [aid("fx"), aid("fy")]
        self.cq = {n: self.model.jnt_qposadr[jid(n)] for n in ("c_joint1", "c_joint2", "c_joint3")}
        self.cact = {n: aid(n) for n in ("c1", "c2", "c3")}

        self.unit_q = self.model.jnt_qposadr[jid("unit_free")]
        self.b_unit = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "unit")
        self.b_c3 = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "c_link3")
        self.g_pin = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "pin")
        self.eq_mount = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "mount")
        self.eq_park = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_EQUALITY, "park")

        # 目标：11 个位置伺服 + 核心三关节
        self.tgt_joint = np.zeros(6)
        self.tgt_chassis = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.tgt_grip = P.GRIP_OPEN
        self.tgt_core = {"c1": 0.0, "c2": 0.0, "c3": 0.0}

        self.keys: set[str] = set()
        self.key_lock = threading.Lock()
        self.selected_joint = 0
        self.slow = False
        self.ramps = {k: Ramp() for k in ("chassis_x", "chassis_y", "chassis_yaw",
                                          "gripper", "p_axis", "q_axis")}
        self.joint_ramps = [Ramp() for _ in range(6)]
        self.serial_joints = False
        self.serial_ok = False
        self.serial_msg = "未连接" if use_serial else "已禁用"
        self.magnet = False
        self.reset()

        if use_serial:
            threading.Thread(target=self._serial_loop,
                             args=(serial_port, baud), daemon=True).start()

    # ---------------- 复位 ----------------
    def reset(self) -> None:
        self.data.qpos[:] = 0.0
        self.data.qvel[:] = 0.0
        # 单元放回取件台
        self.data.qpos[self.unit_q:self.unit_q + 3] = [0.42, -0.19, 0.144]
        self.data.qpos[self.unit_q + 3:self.unit_q + 7] = [1, 0, 0, 0]
        self.data.eq_active[self.eq_mount] = 0
        self.data.eq_active[self.eq_park] = 1
        self.mj.mj_forward(self.model, self.data)
        self.tgt_joint[:] = 0.0
        self.tgt_chassis = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.tgt_grip = P.GRIP_OPEN
        self.tgt_core = {"c1": 0.0, "c2": 0.0, "c3": 0.0}
        self.magnet = False

    # ---------------- 按键 ----------------
    def set_keys(self, keys, selected=None, slow=False) -> None:
        with self.key_lock:
            self.keys = {str(k).lower() for k in keys}
            if selected is not None:
                self.selected_joint = int(selected) % 6
            self.slow = bool(slow)

    def toggle_magnet(self, on: bool | None = None) -> bool:
        self.magnet = (not self.magnet) if on is None else bool(on)
        if self.magnet:
            R1 = self.data.xmat[self.b_c3].reshape(3, 3)
            p1 = self.data.xpos[self.b_c3]
            R2 = self.data.xmat[self.b_unit].reshape(3, 3)
            p2 = self.data.xpos[self.b_unit]
            qq = np.zeros(4)
            self.mj.mju_mat2Quat(qq, (R1.T @ R2).flatten())
            self.model.eq_data[self.eq_mount][0:3] = 0.0
            self.model.eq_data[self.eq_mount][3:6] = R1.T @ (p2 - p1)
            self.model.eq_data[self.eq_mount][6:10] = qq
            self.model.eq_data[self.eq_mount][10] = 1.0
        self.data.eq_active[self.eq_mount] = 1 if self.magnet else 0
        return self.magnet

    def set_grip(self, open_: bool) -> None:
        self.tgt_grip = P.GRIP_OPEN if open_ else P.GRIP_CLOSE

    # ---------------- 真机镜像 ----------------
    def _serial_loop(self, port: str, baud: int) -> None:
        sys.path.insert(0, str(P.ROOT / "src"))
        from arm_mirror.serial_link import available_ports, read_frames

        while True:
            if port not in available_ports():
                self.serial_ok, self.serial_msg = False, f"{port} 不存在"
                time.sleep(2.0)
                continue
            try:
                self.serial_ok, self.serial_msg = True, "已连接"
                for frame in read_frames(port, baud):
                    from arm_mirror.config import joint_map
                    sign, zero = joint_map()
                    q = np.asarray(frame.q) * np.asarray(sign) + np.asarray(zero)
                    self.tgt_joint[:] = np.clip(q, [self.joint_lo[n] for n in JOINTS],
                                                [self.joint_hi[n] for n in JOINTS])
                    self.serial_joints = True
            except Exception as exc:
                self.serial_ok, self.serial_msg = False, f"断开：{exc}"
                time.sleep(1.0)

    # ---------------- 控制 ----------------
    def _apply_keys(self, dt: float) -> None:
        with self.key_lock:
            keys = set(self.keys)
            slow = self.slow
        cmd = command_from_keys(keys, self.selected_joint, slow)

        def ramp(key, target, r):
            return r.step(target, dt)

        vx = ramp("chassis_x", cmd.chassis_x, self.ramps["chassis_x"])
        vy = ramp("chassis_y", cmd.chassis_y, self.ramps["chassis_y"])
        vyaw = ramp("chassis_yaw", cmd.chassis_yaw, self.ramps["chassis_yaw"])
        self.tgt_chassis["x"] = float(np.clip(self.tgt_chassis["x"] + vx * dt, *P.CHASSIS_X))
        self.tgt_chassis["y"] = float(np.clip(self.tgt_chassis["y"] + vy * dt, *P.CHASSIS_Y))
        self.tgt_chassis["yaw"] = float(np.clip(self.tgt_chassis["yaw"] + vyaw * dt, *P.CHASSIS_YAW))

        vg = ramp("gripper", cmd.gripper, self.ramps["gripper"])
        self.tgt_grip = float(np.clip(self.tgt_grip + vg * dt, P.GRIP_CLOSE, P.GRIP_OPEN))

        vp = ramp("p_axis", cmd.p_axis, self.ramps["p_axis"])
        vq = ramp("q_axis", cmd.q_axis, self.ramps["q_axis"])
        self.tgt_core["c2"] = float(np.clip(self.tgt_core["c2"] + vp * dt, 0.0, 1.5708))
        self.tgt_core["c3"] = float(np.clip(self.tgt_core["c3"] + vq * dt, -1.5708, 1.5708))

        if not self.serial_joints:
            for i in range(6):
                v = cmd.joint.get(i, 0.0)
                vv = self.joint_ramps[i].step(v, dt)
                lo, hi = self.joint_lo[JOINTS[i]], self.joint_hi[JOINTS[i]]
                self.tgt_joint[i] = float(np.clip(self.tgt_joint[i] + vv * dt, lo, hi))

    def step(self) -> None:
        dt = self.dt
        self._apply_keys(dt)
        for n, a in self.act.items():
            pass
        ctrl = self.data.ctrl
        for i, n in enumerate(JOINTS):
            ctrl[self.act[f"a{i+1}"]] = self.tgt_joint[i]
        half = self.tgt_grip
        ctrl[self.act["fl"]] = half
        ctrl[self.act["fr"]] = -half
        for axis, (qadr, act) in self.chassis.items():
            ctrl[act] = self.tgt_chassis[axis]
        ctrl[self.cact["c1"]] = self.tgt_core["c1"]
        ctrl[self.cact["c2"]] = self.tgt_core["c2"]
        ctrl[self.cact["c3"]] = self.tgt_core["c3"]
        self.mj.mj_step(self.model, self.data)

    # ---------------- 测量 ----------------
    def measure(self) -> dict:
        pin_p = self.data.geom_xpos[self.g_pin]
        pin_R = self.data.geom_xmat[self.g_pin].reshape(3, 3)
        axis = pin_R[:, 2]
        free = pin_p - axis * float(self.model.geom_size[self.g_pin][1])
        u = self.data.xpos[self.b_unit]
        depth = float(np.dot(u - free, axis))
        uax = self.data.xmat[self.b_unit].reshape(3, 3)[:, 2]
        axis_err = float(np.degrees(np.arccos(np.clip(abs(np.dot(uax, axis)), -1, 1))))
        p_deg = float(np.degrees(self.data.qpos[self.cq["c_joint2"]]))
        q_deg = float(np.degrees(self.data.qpos[self.cq["c_joint3"]]))
        grip_l = float(self.data.qpos[self.model.jnt_qposadr[
            self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_JOINT, "finger_l")]])
        return dict(depth_mm=depth * 1000.0, axis_deg=axis_err, p_deg=p_deg, q_deg=q_deg,
                    grip_mm=grip_l * 1000.0)

    def judgments(self) -> dict:
        m = self.measure()
        return dict(
            grasped=abs(m["depth_mm"]) > 20 and m["grip_mm"] < -1.0,
            inserted=m["depth_mm"] > P.DEPTH_OK * 1000 and m["axis_deg"] < P.AXIS_OK,
            p_ok=abs(m["p_deg"] - 90.0) < P.P_OK,
            q_ok=abs(m["q_deg"] - P.Q_TARGET) < P.Q_OK,
        )

    # ---------------- 主循环 ----------------
    def run(self, hz: float = 1000.0) -> None:
        """按墙钟追帧：落后多少就补多少步（上限 50 步/轮），保证 1 秒墙钟 ≈ 1 秒物理时间。"""
        dt = 1.0 / hz
        next_t = time.perf_counter()
        while True:
            now = time.perf_counter()
            behind = int((now - next_t) / dt)
            if behind > 0:
                n = min(behind, 50)
                for _ in range(n):
                    self.step()
                next_t += n * dt
                if behind > 50:
                    next_t = now
            else:
                delay = next_t - time.perf_counter()
                if delay > 0:
                    time.sleep(min(0.001, delay))


def make_handler(sim: Sim):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, ctype, body: bytes):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj):
            self._send(200, "application/json", json.dumps(obj).encode())

        def do_GET(self):
            p = urlparse(self.path).path
            if p in ("/", "/index.html"):
                self._send(200, "text/html; charset=utf-8", P.WEB_INDEX.read_bytes())
            elif p == "/scene.json":
                self._send(200, "application/json", P.WEB_SCENE_JSON.read_bytes())
            elif p.startswith("/vendor/"):
                f = (P.WEB / p.lstrip("/")).resolve()
                if not str(f).startswith(str(P.WEB)) or not f.is_file():
                    self._send(404, "text/plain", b"not found")
                    return
                self._send(200, "text/javascript", f.read_bytes())
            elif p == "/pose":
                pose = np.concatenate([sim.data.xpos, sim.data.xquat], axis=1)
                self._json(dict(b=[[float(v) for v in row] for row in pose]))
            elif p == "/state":
                st = dict(sim.measure())
                st.update(judgments=sim.judgments(),
                          magnet=bool(sim.magnet),
                          serial_ok=sim.serial_ok, serial_msg=sim.serial_msg,
                          joints=[float(v) for v in sim.tgt_joint],
                          chassis=dict(sim.tgt_chassis),
                          grip=float(sim.tgt_grip),
                          core=dict(sim.tgt_core),
                          selected=sim.selected_joint)
                self._json(st)
            elif p == "/magnet":
                qs = parse_qs(urlparse(self.path).query)
                v = qs.get("on", ["toggle"])[0]
                on = None if v == "toggle" else v in ("1", "true", "on")
                self._json(dict(magnet=sim.toggle_magnet(on)))
            elif p == "/gripper":
                qs = parse_qs(urlparse(self.path).query)
                v = qs.get("open", ["toggle"])[0]
                open_ = (not (sim.tgt_grip > P.GRIP_CLOSE)) if v == "toggle" else v in ("1", "true", "open")
                sim.set_grip(open_)
                self._json(dict(grip=sim.tgt_grip, open=open_))
            elif p == "/reset":
                sim.reset()
                self._json(dict(ok=True))
            else:
                self._send(404, "text/plain", b"not found")

        def do_POST(self):
            p = urlparse(self.path).path
            n = int(self.headers.get("Content-Length", 0) or 0)
            body = self.rfile.read(n) if n else b"{}"
            if p == "/keys":
                try:
                    payload = json.loads(body or b"{}")
                except Exception:
                    payload = {}
                sim.set_keys(payload.get("keys", []), payload.get("selected"),
                             payload.get("slow", False))
                self._json(dict(ok=True, keys=sorted(sim.keys)))
            else:
                self._send(404, "text/plain", b"not found")

    return Handler


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=P.PORT)
    ap.add_argument("--scene", default=str(P.SCENE))
    ap.add_argument("--no-serial", action="store_true")
    ap.add_argument("--serial-port", default="COM6")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args(argv)

    sim = Sim(Path(args.scene), use_serial=not args.no_serial,
              serial_port=args.serial_port, baud=args.baud)
    threading.Thread(target=sim.run, daemon=True).start()
    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(sim))
    print(f"v1.9 手动装配: http://127.0.0.1:{args.port}")
    print(f"场景 {args.scene}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
