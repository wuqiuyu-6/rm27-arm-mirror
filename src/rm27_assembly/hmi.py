"""
Web 上位机服务端：把 MuJoCo 的画面和状态通过 HTTP 推给浏览器。

  GET  /            上位机页面
  GET  /stream      MJPEG 视频流（<img src="/stream"> 直接显示）
  GET  /state       状态 JSON：fps / 延迟 / 关节角 / 模式
  GET  /frame.jpg   当前一帧（调试用）
  GET  /set?j=a,b,c,d,e    设置 5 个执行器目标
  GET  /mode?manual=1|0    手动 / 演示
  GET  /reset              全部归零

架构和 hmi.py 一样：不自己跑物理，只从共享内存读 sim_server 的状态。
"""

import os
os.environ.setdefault("MUJOCO_GL", "glfw")

import argparse
import collections
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse, parse_qs

import numpy as np
import mujoco
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
from .interp import interpolate, wait_for_channel
from .paths import SCENE, WEB_DIR, WEB_INDEX
from . import paths
from .protocol import PHASES

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

SCENES = {"rm27": SCENE}
W, H = 800, 600
FPS = 60
JPEG_Q = 72


class SimView:
    """后台线程：读共享内存 -> 插值 -> 算 body 位姿 / 任务数据。"""

    def __init__(self, scene_key, shm_name, delay):
        self.scene_key = scene_key
        self.model = mujoco.MjModel.from_xml_path(str(SCENES[scene_key]))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.model.vis.global_.offwidth = max(int(self.model.vis.global_.offwidth), W)
        self.model.vis.global_.offheight = max(int(self.model.vis.global_.offheight), H)
        self.renderer = None      # 必须在渲染线程里创建：GL 上下文是线程绑定的

        self.cam = mujoco.MjvCamera()
        self.cam.lookat[:] = [0.33, 0.02, 0.30]
        self.cam.distance = 1.05
        self.cam.azimuth = 35
        self.cam.elevation = -18

        # 8 个执行器对应的关节 qpos 地址（给滑块显示实际角度）
        self.act_qadr = []
        for a in range(self.model.nu):
            j = int(self.model.actuator_trnid[a, 0])
            self.act_qadr.append(int(self.model.jnt_qposadr[j]))

        self.ch = wait_for_channel(shm_name, self.model.nq, self.model.nv, self.model.nu)
        self.ch.set_manual(False)

        self.delay = delay
        self.buf = collections.deque(maxlen=600)
        self.stats = collections.Counter()

        self.lock = threading.RLock()
        self.jpg = None
        self.fps = 0.0
        self.latency = 0.0
        self.joint_now = np.zeros(self.model.nu)
        self.ctrl_target = np.zeros(self.model.nu)
        self.ext_core = np.zeros(3)
        self.ext_lock = 0.0
        self.manual = False
        self._stop = False
        self.last_hit = time.time()
        self.last_render_req = 0.0
        self.always_render = False
        self.sim_t = 0.0
        self.body_pose = np.zeros((self.model.nbody, 7))
        self.task = dict(phase=0, playing=0, cycle=0, depth=0.0,
                         p_deg=0.0, q_deg=0.0, grip=0.0)

        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    # ---------------------------------------------------------------- 交互
    def touch(self):
        self.last_hit = time.time()

    def request_render(self):
        """有人要看图片（/2d 或 /frame.jpg），才把 MuJoCo 渲染打开。"""
        self.last_render_req = time.time()

    def render_needed(self):
        if self.always_render:
            return True
        return (time.time() - self.last_render_req) < 3.0

    def set_ctrl(self, vals):
        with self.lock:
            self.ctrl_target = np.asarray(vals, dtype=float)[:self.model.nu]

    def get_ctrl(self):
        with self.lock:
            return self.ctrl_target.copy()

    def set_ext(self, core, lock):
        """核心三关节 + 磁吸（手动通道）"""
        with self.lock:
            self.ext_core = np.asarray(core, dtype=float)[:3]
            self.ext_lock = 1.0 if lock else 0.0

    def get_ext(self):
        with self.lock:
            return np.concatenate([self.ext_core, [self.ext_lock]])

    def set_manual(self, on):
        with self.lock:
            was = self.manual
            self.manual = bool(on)
            if on and not was:
                # 切进手动：用当前实际角度垫底，核心三关节也用当前值垫底，
                # 否则会把核心瞬间拉到 0（单元会被 weld 拽下来）
                self.ctrl_target = self.joint_now.copy()
                t = self.task
                self.ext_core = np.array([t.get('c1', 0.0), np.radians(t.get('p_deg', 0.0)),
                                          np.radians(t.get('q_deg', 0.0))], dtype=float)
        self.ch.set_manual(bool(on))

    # ---------------------------------------------------------------- 循环
    def _pull(self):
        got = 0
        while got < 50:
            st = self.ch.read_state()
            if st is None:
                self.stats["retry"] += 1
                break
            t, qpos, qvel, task = st
            if self.buf and t <= self.buf[-1][0] + 1e-9:
                break
            self.buf.append((t, qpos, qvel))
            self.task = task
            got += 1
        if len(self.buf) > 1 and self.buf[0][0] < self.buf[-1][0] - 1.0:
            self.buf.popleft()

    def _loop(self):
        # 注意：GL 上下文绑定线程，渲染器必须在本线程里创建，否则 render() 出来是全黑。
        # 而且只在有人要图片时才建 —— 3D 页面只用 /pose，不需要 MuJoCo 渲染。
        dt_frame = 1.0 / FPS
        t_next = time.perf_counter()
        n = 0
        t_stat = t_next
        while not self._stop:
            t0 = time.perf_counter()
            self._pull()

            if self.manual:
                self.ch.publish_cmd(0.0, self.get_ctrl(), self.get_ext())

            if self.buf:
                t_latest = self.buf[-1][0]
                q, v, flag = interpolate(self.model, self.buf, t_latest - self.delay)
                self.stats[flag] += 1
                self.data.qpos[:] = q
                self.data.qvel[:] = v
                mujoco.mj_forward(self.model, self.data)

                pose = np.concatenate(
                    [self.data.xpos, self.data.xquat], axis=1)   # (nbody, 7)

                # 只有 /stream 或 /frame.jpg 最近被访问过才渲染 JPEG
                jpg = None
                if self.render_needed():
                    if self.renderer is None:
                        self.renderer = mujoco.Renderer(self.model, H, W)
                    self.renderer.update_scene(self.data, camera=self.cam)
                    jpg = _encode_jpeg(self.renderer.render())
                    self.stats["rendered"] += 1
                else:
                    self.stats["skipped"] += 1

                with self.lock:
                    if jpg is not None:
                        self.jpg = jpg
                    self.latency = self.delay * 1000
                    self.joint_now = np.array([self.data.qpos[a] for a in self.act_qadr])
                    self.body_pose = pose.copy()
                    self.sim_t = t_latest

            n += 1
            now = time.perf_counter()
            if now - t_stat >= 0.5:
                with self.lock:
                    self.fps = n / (now - t_stat)
                n, t_stat = 0, now

            t_next += dt_frame
            sleep = t_next - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                t_next = time.perf_counter()

    def snapshot(self):
        with self.lock:
            return dict(jpg=self.jpg, fps=self.fps, latency=self.latency,
                        joints=self.joint_now.tolist(),
                        ctrl=(self.ctrl_target if self.manual else self.joint_now).tolist(),
                        ext=self.get_ext().tolist(),
                        manual=self.manual,
                        task=dict(self.task),
                        stats=dict(self.stats))

    def pose_snapshot(self):
        """所有 body 的世界位姿 (x,y,z,qw,qx,qy,qz)，前端直接拿来摆模型。"""
        with self.lock:
            return self.sim_t, self.body_pose.tolist()

    def stop(self):
        self._stop = True
        time.sleep(0.15)
        try:
            self.ch.request_quit()
            self.ch.close()
        except Exception:
            pass
        try:
            if self.renderer is not None:
                self.renderer.close()
        except Exception:
            pass


def _encode_jpeg(rgb):
    import io
    buf = io.BytesIO()
    Image.fromarray(rgb).save(buf, format="JPEG", quality=JPEG_Q)
    return buf.getvalue()


def lan_ips():
    """列出本机的局域网 IPv4，方便告诉另一台电脑访问哪个地址。"""
    ips = []
    try:
        import socket
        host = socket.gethostname()
        for info in socket.getaddrinfo(host, None, socket.AF_INET):
            ip = info[4][0]
            if ip not in ips and not ip.startswith("127."):
                ips.append(ip)
    except Exception:
        pass
    if not ips:
        try:
            import socket
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ips.append(s.getsockname()[0])
            s.close()
        except Exception:
            pass
    return ips


# -------------------------------------------------------------------- HTTP
class Handler(BaseHTTPRequestHandler):
    view: SimView = None
    html: bytes = b""
    token: str = ""

    def log_message(self, *a):        # 别刷屏
        pass

    # ---- 工具
    def _send(self, code, ctype, body):
        if isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj):
        self._send(200, "application/json", json.dumps(obj))

    CTYPES = {".js": "text/javascript; charset=utf-8",
              ".json": "application/json",
              ".png": "image/png",
              ".css": "text/css; charset=utf-8"}

    def _vendor(self, p):
        """静态文件（three.js 和导出的场景 JSON）。"""
        rel = p[len("/vendor/"):]
        root = (WEB_DIR / "vendor").resolve()
        f = (root / rel).resolve()
        if root not in f.parents or not f.is_file():
            self._send(404, "text/plain", "not found")
            return
        ctype = self.CTYPES.get(f.suffix.lower(), "application/octet-stream")
        self._send(200, ctype, f.read_bytes())

    # ---- 路由
    def do_GET(self):
        self.view.touch()
        u = urlparse(self.path)
        qs = parse_qs(u.query)
        p = u.path

        # 跨机访问时保护"数据"和"控制"接口；静态资源（three.js/几何）不设限
        if self.token and not p.startswith("/vendor/"):
            if qs.get("t", [""])[0] != self.token:
                self._send(403, "text/plain; charset=utf-8",
                           "需要 token：在地址后面加 ?t=你的token")
                return

        if p in ("/", "/index.html", "/3d", "/3d/"):
            # 默认给 three.js 客户端渲染版（清晰、无压缩）
            self._send(200, "text/html; charset=utf-8",
                       WEB_INDEX.read_bytes())
        elif p in ("/2d", "/2d/"):
            # 备用：MJPEG 推流版（画面是 JPEG，会有压缩感）
            self._send(200, "text/html; charset=utf-8", self.html)
        elif p == "/pose":
            t, pose = self.view.pose_snapshot()
            self._json({"t": t, "b": pose})
        elif p == "/phases":
            from .protocol import PHASES as _PH
            self._json({"phases": list(_PH)})
        elif p == "/scene.json":
            f = WEB_DIR / f"scene_{self.view.scene_key}.json"
            if not f.is_file():
                self._send(404, "text/plain",
                           f"缺少 {f.name}，先跑 python export_scene.py {self.view.scene_key}")
                return
            self._send(200, "application/json", f.read_bytes())
        elif p.startswith("/vendor/"):
            self._vendor(p)
        elif p == "/state":
            s = self.view.snapshot()
            s.pop("jpg", None)
            self._json(s)
        elif p == "/frame.jpg":
            self.view.request_render()
            s = self.view.snapshot()
            for _ in range(200):                   # 刚打开渲染时等一下第一帧（首次要建 GL 上下文）
                if s["jpg"]:
                    break
                time.sleep(0.01)
                s = self.view.snapshot()
            if s["jpg"]:
                self._send(200, "image/jpeg", s["jpg"])
            else:
                self._send(503, "text/plain", "no frame yet")
        elif p == "/stream":
            self.view.request_render()
            self._stream()
        elif p == "/set":
            raw = qs.get("j", [""])[0]
            try:
                vals = [float(x) for x in raw.split(",")]
                if len(vals) == self.view.model.nu:
                    self.view.set_ctrl(vals)
                    self.view.set_manual(True)   # 用户一动滑块就接管
            except ValueError:
                pass
            # 核心三关节 + 磁吸（c=c1,c2,c3&lock=0/1）
            rawc = qs.get("c", [""])[0]
            lock = qs.get("lock", [None])[0]
            if rawc or lock is not None:
                try:
                    core = ([float(x) for x in rawc.split(",")]
                            if rawc else self.view.ext_core.tolist())
                    lk = (lock not in (None, "0", "false", "")) if lock is not None else bool(self.view.ext_lock)
                    self.view.set_ext(core, lk)
                    self.view.set_manual(True)
                except ValueError:
                    pass
            self._json({"ok": True, "ctrl": self.view.get_ctrl().tolist(),
                        "ext": self.view.get_ext().tolist()})
        elif p == "/mode":
            m = qs.get("manual", ["1"])[0] not in ("0", "false", "")
            self.view.set_manual(m)
            self._json({"ok": True, "manual": m})
        elif p == "/task":
            cmd = qs.get("cmd", [""])[0]
            if cmd == "play":
                self.view.set_manual(False)
                self.view.ch.request_play()
            elif cmd == "pause":
                self.view.set_manual(True)
            elif cmd == "reset":
                self.view.set_manual(False)
                self.view.ch.request_reset()
            self._json({"ok": True, "cmd": cmd})
        elif p == "/reset":
            self.view.set_ctrl([0.0] * self.view.model.nu)
            self.view.set_manual(True)
            self._json({"ok": True})
        else:
            self._send(404, "text/plain", "not found")

    do_POST = do_GET

    # ---- MJPEG
    def _stream(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.end_headers()
        try:
            while True:
                self.view.touch()            # 推流期间也算活跃，避免被空闲看门狗误杀
                s = self.view.snapshot()
                jpg = s["jpg"]
                if jpg:
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                time.sleep(1.0 / FPS)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


# -------------------------------------------------------------------- 入口
def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", choices=list(SCENES), default="rm27")
    ap.add_argument("--shm-name", default=paths.SHM_NAME)
    ap.add_argument("--delay", type=float, default=0.05)
    ap.add_argument("--port", type=int, default=paths.PORT)
    ap.add_argument("--host", default="127.0.0.1",
                    help="监听地址。跨机访问用 0.0.0.0")
    ap.add_argument("--token", default="",
                    help="访问口令。对外暴露时强烈建议设置")
    ap.add_argument("--idle-timeout", type=float, default=20.0,
                    help="浏览器关掉后多少秒自动退出（0=不退出）")
    ap.add_argument("--always-render", action="store_true",
                    help="强制一直渲染 JPEG（调试 / 老机器兼容）")
    args = ap.parse_args(argv)

    view = SimView(args.scene, args.shm_name, args.delay)
    view.always_render = args.always_render
    Handler.view = view
    _html2d = WEB_DIR / "webui.html"
    Handler.html = _html2d.read_bytes() if _html2d.is_file() else b"<h1>2D view not installed</h1>"
    Handler.token = args.token

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    srv.daemon_threads = True

    if args.idle_timeout > 0:
        def watch():
            while True:
                time.sleep(1.5)
                if time.time() - view.last_hit > args.idle_timeout:
                    print("\n浏览器已断开，自动退出。")
                    srv.shutdown()
                    return
        threading.Thread(target=watch, daemon=True).start()

    print("-" * 62)
    print(f"本机访问:   http://127.0.0.1:{args.port}"
          + (f"?t={args.token}" if args.token else ""))
    if args.host in ("0.0.0.0", ""):
        for ip in lan_ips():
            print(f"局域网访问: http://{ip}:{args.port}"
                  + (f"?t={args.token}" if args.token else ""))
        if not args.token:
            print("!! 警告：没有设置 --token，同一局域网内任何人都能控制这台仿真")
    print("-" * 62)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n退出中...")
    finally:
        srv.shutdown()
        view.stop()


if __name__ == "__main__":
    main()
