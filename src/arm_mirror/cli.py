"""命令行入口：print（只看读数）/ view（实时窗口）/ record（录视频）。"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

from . import protocol
from .model import DEFAULT_MODEL, ArmModel, JointMap
from .serial_link import DEFAULT_BAUD, DEFAULT_PORT, describe_ports, read_frames


def _add_common(p: argparse.ArgumentParser) -> None:
    """公共参数：主命令和子命令都挂一份，支持两种写法：
    `arm-mirror --port COM6 view` 与 `arm-mirror view --port COM6`。
    """
    default = argparse.SUPPRESS
    p.add_argument("--port", default=DEFAULT_PORT if default is None else default,
                   help="串口（默认 COM6）")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD if default is None else default,
                   help="波特率（默认 115200）")
    p.add_argument("--model", default=str(DEFAULT_MODEL) if default is None else default,
                   help="MJCF 路径")
    p.add_argument("--sign", default="1,1,1,1,1,1" if default is None else default,
                   help="六轴方向，+1/-1")
    p.add_argument("--zero", default="0,0,0,0,0,0" if default is None else default,
                   help="六轴零位偏置（rad）")
    p.add_argument("--flip", action="store_true", default=False if default is None else default,
                   help="六轴全部取反（等价 --sign=-1,...）")
    p.add_argument("--clamp", action="store_true", default=False if default is None else default,
                   help="超出模型关节范围时夹住")
    p.add_argument("--no-crc", action="store_true", default=False if default is None else default,
                   help="不校验 CRC（仅排查）")
    p.add_argument("--alpha", type=float, default=1.0 if default is None else default,
                   help="平滑系数，1=不平滑")
    p.add_argument("--seconds", type=float, default=0.0 if default is None else default,
                   help="运行时长（0=直到关闭）")


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="arm_mirror", description="RM27 真机姿态镜像")
    _add_common(p)
    sub = p.add_subparsers(dest="cmd")
    for name, help_text in (("print", "只打印读数"), ("view", "实时窗口"), ("record", "录视频")):
        sp = sub.add_parser(name, help=help_text)
        _add_common(sp)
        if name == "record":
            sp.add_argument("--out", default="mirror.mp4")
    return p


def _run(args) -> int:
    jm = JointMap.parse("-1,-1,-1,-1,-1,-1" if args.flip else args.sign, args.zero)
    arm = ArmModel(args.model)

    viewer = renderer = None
    cam = None
    mj = arm._mj
    if args.cmd == "view":
        import mujoco.viewer as mjv
        cam = mjv.launch_passive(arm.model, arm.data, show_left_ui=False)
    elif args.cmd == "record":
        cam = mj.MjvCamera()
        cam.lookat[:] = [0.12, 0.06, 0.36]
        cam.distance = 1.30
        cam.azimuth = 135
        cam.elevation = -12
        renderer = mj.Renderer(arm.model, 720, 960)

    q = np.zeros(6)
    n = 0
    frames = []
    t0 = time.time()
    try:
        for frame in read_frames(args.port, args.baud, check_crc=not args.no_crc):
            n += 1
            target = jm.apply(frame.q)
            if args.clamp:
                target = arm.clamp(target)
            q = target if args.alpha >= 1.0 else q + args.alpha * (target - q)

            if args.cmd == "print":
                print(f"seq={frame.seq:6d} " +
                      "  ".join(f"{arm.joint_names[i]}:{np.rad2deg(q[i]):+7.2f}°" for i in range(6)) +
                      "  | tau " + " ".join(f"{v:+6.2f}" for v in frame.tau))
            else:
                arm.set_joints(q)
                if viewer is not None:
                    viewer.sync()
                    if not viewer.is_running():
                        break
                if renderer is not None and n % 3 == 0:
                    renderer.update_scene(arm.data, camera=cam)
                    frames.append(renderer.render().copy())

            if args.seconds and time.time() - t0 >= args.seconds:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if viewer is not None:
            viewer.close()
        if renderer is not None:
            renderer.close()

    if frames and args.cmd == "record":
        import imageio.v2 as imageio
        imageio.mimsave(args.out, frames, fps=10, quality=8, macro_block_size=1)
        print(f"视频 -> {args.out}")

    dt = max(1e-9, time.time() - t0)
    print(f"共 {n} 帧，{n / dt:.1f} Hz")
    return 0


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    args = _build_parser().parse_args(argv)
    if not args.cmd:
        args.cmd = "view"

    from .serial_link import available_ports
    if args.port not in available_ports():
        print(f"打不开 {args.port}：系统当前串口 {describe_ports()}")
        print("检查 USB、驱动、板子是否上电（设备管理器里应能看到该 COM 口）。")
        return 2

    return _run(args)


if __name__ == "__main__":
    raise SystemExit(main())
