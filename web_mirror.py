"""v1.8 网页上位机：真机关节角 -> 仿真臂（含夹爪按钮）。

结构（和 v1.6/v1.7 一致）：

    serial 线程 ──关节目标──► 物理/位姿循环（60 Hz，mj_step）──► /pose、/state
                                   ▲
    浏览器 ── /gripper?open=0|1 ───┘（夹爪按钮）

页面：web/index.html（three.js，读 /scene.json + /pose）。

用法：
    python web_mirror.py --port 8007
    python web_mirror.py --port 8007 --no-serial     # 没有真机时也能看页面
"""

from __future__ import annotations

import argparse
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import numpy as np

HERE = Path(__file__).resolve().parent
WEB = HERE / "web"
MODEL = HERE / "model" / "arm_v7" / "arm_v7_gripper.xml"

JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
SIGN = np.array([-1.0] * 6)          # 与 config/joints.yaml 一致
ZERO = np.zeros(6)
OPEN_POS = 0.028                     # 夹爪张开（米）
CLOSE_POS = 0.000                    # 合拢


class State:
    def __init__(self, model_path: Path, use_serial: bool, port: str, baud: int):
        import mujoco

        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.joint_qadr = {
            n: self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in JOINTS
        }
        self.grip_act = [
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
            for n in ("grip_l", "grip_r")
        ]
        self.arm_dofs = [
            self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in JOINTS]
        self.grip_qadr = [
            self.model.jnt_qposadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]
            for n in ("finger_l", "finger_r")
        ]

        self.port = port
        self.baud = baud
        self.use_serial = use_serial
        self.q = np.zeros(6)             # 目标关节角（仿真侧）
        self.q_raw = np.zeros(6)         # 真机原始
        self.tau = np.zeros(6)
        self.grip_open = True
        self.serial_ok = False
        self.serial_msg = "未连接" if use_serial else "已禁用（--no-serial）"
        self.frames = 0
        self.rate = 0.0
        self._lock = threading.Lock()

    # ---------------- 串口线程 ----------------
    def start_serial(self) -> None:
        if not self.use_serial:
            return
        threading.Thread(target=self._serial_loop, daemon=True).start()

    def _serial_loop(self) -> None:
        from arm_mirror.serial_link import available_ports, describe_ports, read_frames

        while True:
            if self.port not in available_ports():
                with self._lock:
                    self.serial_ok = False
                    self.serial_msg = f"{self.port} 不存在；当前串口 {describe_ports()}"
                time.sleep(2.0)
                continue
            try:
                with self._lock:
                    self.serial_msg = "已连接"
                    self.serial_ok = True
                for frame in read_frames(self.port, self.baud):
                    with self._lock:
                        self.q_raw = np.array(frame.q, dtype=float)
                        self.q = SIGN * self.q_raw + ZERO
                        self.tau = np.array(frame.tau, dtype=float)
                        self.frames += 1
            except Exception as exc:  # 掉线重连
                with self._lock:
                    self.serial_ok = False
                    self.serial_msg = f"断开：{exc}"
                time.sleep(1.0)

    # ---------------- 仿真循环 ----------------
    def run(self, hz: float = 60.0) -> None:
        dt = 1.0 / hz
        next_t = time.perf_counter()
        n = 0
        t0 = time.perf_counter()
        target_grip = OPEN_POS if self.grip_open else CLOSE_POS
        while True:
            with self._lock:
                q = self.q.copy()
                want_open = self.grip_open
            goal = OPEN_POS if want_open else CLOSE_POS
            step = 0.15 * dt          # 夹爪速度上限 0.15 m/s
            target_grip += max(-step, min(step, goal - target_grip))

            for i, name in enumerate(JOINTS):
                self.data.qpos[self.joint_qadr[name]] = q[i]
                self.data.qvel[self.arm_dofs] = 0.0
            for act in self.grip_act:
                self.data.ctrl[act] = target_grip
            for _ in range(2):
                self.mj.mj_step(self.model, self.data)
                for i, name in enumerate(JOINTS):   # 机械臂仍然按真机写死
                    self.data.qpos[self.joint_qadr[name]] = q[i]
                    self.data.qvel[self.arm_dofs] = 0.0

            n += 1
            now = time.perf_counter()
            if now - t0 > 0.5:
                with self._lock:
                    self.rate = n / (now - t0)
                n = 0
                t0 = now
            next_t += dt
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)

    # ---------------- 读取接口 ----------------
    def pose(self):
        return np.concatenate([self.data.xpos, self.data.xquat], axis=1)

    def state(self) -> dict:
        with self._lock:
            return dict(
                joints=[float(v) for v in self.q],
                raw=[float(v) for v in self.q_raw],
                tau=[float(v) for v in self.tau],
                grip_open=bool(self.grip_open),
                grip_l=float(self.data.qpos[self.grip_qadr[0]]),
                grip_r=float(self.data.qpos[self.grip_qadr[1]]),
                serial_ok=self.serial_ok,
                serial_msg=self.serial_msg,
                frames=self.frames,
                rate=self.rate,
            )

    def set_grip(self, open_: bool) -> None:
        with self._lock:
            self.grip_open = bool(open_)


def make_handler(state: State):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
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
                self._send(200, "text/html; charset=utf-8", (WEB / "index.html").read_bytes())
            elif p == "/scene.json":
                f = WEB / "scene.json"
                if not f.is_file():
                    self._send(404, "text/plain", b"run tools/export_scene.py first")
                    return
                self._send(200, "application/json", f.read_bytes())
            elif p.startswith("/vendor/"):
                f = (WEB / p.lstrip("/")).resolve()
                if not str(f).startswith(str(WEB)) or not f.is_file():
                    self._send(404, "text/plain", b"not found")
                    return
                ctype = "text/javascript" if f.suffix == ".js" else "application/octet-stream"
                self._send(200, ctype, f.read_bytes())
            elif p == "/pose":
                pose = state.pose()
                self._json(dict(b=[[float(v) for v in row] for row in pose]))
            elif p == "/state":
                self._json(state.state())
            elif p == "/gripper":
                qs = parse_qs(urlparse(self.path).query)
                val = qs.get("open", ["toggle"])[0]
                if val in ("1", "true", "open"):
                    state.set_grip(True)
                elif val in ("0", "false", "close"):
                    state.set_grip(False)
                else:
                    state.set_grip(not state.grip_open)
                self._json(dict(ok=True, grip_open=state.grip_open))
            else:
                self._send(404, "text/plain", b"not found")

    return Handler


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8007)
    ap.add_argument("--serial-port", default="COM6")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--no-serial", action="store_true")
    args = ap.parse_args()

    st = State(MODEL, not args.no_serial, args.serial_port, args.baud)
    threading.Thread(target=st.run, daemon=True).start()
    st.start_serial()

    srv = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(st))
    print(f"网页上位机: http://127.0.0.1:{args.port}")
    print(f"模型 {MODEL}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
