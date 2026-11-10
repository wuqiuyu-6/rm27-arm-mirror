"""命令行入口：rm27-assembly server | hmi | all | export。"""

from __future__ import annotations

import argparse
import subprocess
import sys
import threading
import time

from . import __version__, paths


def _add_common(p: argparse.ArgumentParser) -> None:
    """公共参数；子命令上也挂一份，支持 `all --port 8010` 与 `--port 8010 all`。"""
    s = argparse.SUPPRESS
    p.add_argument("--shm", default=s, help="共享内存名（默认 rm27_shm）")
    p.add_argument("--port", type=int, default=s, help="网页端口（默认 8005）")
    p.add_argument("--speed", type=float, default=s, help="回放倍速")
    p.add_argument("--delay", type=float, default=s, help="网页插值延迟（秒）")
    p.add_argument("--idle-timeout", type=float, default=s,
                   help="无浏览器 N 秒后自动退出；0 = 不退出（默认 0）")


def _ns(ns, name, default):
    return getattr(ns, name, default)


def cmd_server(ns) -> int:
    from .server import main
    return main(["--shm-name", _ns(ns, "shm", paths.SHM_NAME),
                 "--speed", str(_ns(ns, "speed", 1.0))]) or 0


def cmd_hmi(ns) -> int:
    from .hmi import main
    return main(["--port", str(_ns(ns, "port", paths.PORT)),
                 "--shm-name", _ns(ns, "shm", paths.SHM_NAME),
                 "--delay", str(_ns(ns, "delay", 0.05)),
                 "--idle-timeout", str(_ns(ns, "idle_timeout", 0.0))]) or 0


def cmd_all(ns) -> int:
    from .hmi import main as hmi_main
    from .server import main as server_main

    th = threading.Thread(target=server_main,
                          args=(["--shm-name", _ns(ns, "shm", paths.SHM_NAME),
                                 "--speed", str(_ns(ns, "speed", 1.0))],),
                          daemon=True)
    th.start()
    time.sleep(1.5)          # 让物理端先把共享内存建起来
    return hmi_main(["--port", str(_ns(ns, "port", paths.PORT)),
                     "--shm-name", _ns(ns, "shm", paths.SHM_NAME),
                     "--delay", str(_ns(ns, "delay", 0.05)),
                 "--idle-timeout", str(_ns(ns, "idle_timeout", 0.0))]) or 0


def cmd_export(ns) -> int:
    return subprocess.call([sys.executable, str(paths.ROOT / "tools" / "export_scene.py")])


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="rm27-assembly", description=f"RM27 装配仿真 v{__version__}")
    _add_common(p)
    sub = p.add_subparsers(dest="cmd", required=True)
    for name, help_text in (("server", "只起物理端（1 kHz 回放）"),
                            ("hmi", "只起网页端"),
                            ("all", "物理端 + 网页端（推荐）"),
                            ("export", "导出 web/scene.json")):
        sp = sub.add_parser(name, help=help_text)
        _add_common(sp)
    return p


def main(argv=None) -> int:
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass
    ns = build_parser().parse_args(argv)
    return {"server": cmd_server, "hmi": cmd_hmi, "all": cmd_all, "export": cmd_export}[ns.cmd](ns)


if __name__ == "__main__":
    raise SystemExit(main())
