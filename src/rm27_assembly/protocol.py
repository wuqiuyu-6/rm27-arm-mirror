"""
共享内存状态通道：seqlock 防撕裂 + 固定内存布局（RM27 版，多了任务数据块）

内存布局（全部小端、8 字节对齐）
--------------------------------------------------------------------------------
偏移 0                          头部 40 字节
    u32 magic | u32 version | u64 seq | f64 t | i32 nq | i32 nv | i32 nu | u32 flags
偏移 40                         qpos  f64[nq]
偏移 40+8*nq                    qvel  f64[nv]
偏移 40+8*nq+8*nv               指令头 16 字节 : u64 cmd_seq | f64 cmd_t
偏移 +16                        ctrl  f64[nu]      (上位机 -> 物理端)
偏移 +8*nu                      任务块 64 字节
    i32 phase | i32 playing | i32 cycle | i32 pad
    f64 depth_mm | f64 p_deg | f64 q_deg | f64 grip_n

seqlock 规则
--------------------------------------------------------------------------------
写端: seq += 1 (变奇数) -> 写数据 -> seq += 1 (变偶数)
读端: 读 seq (奇数则重试) -> 拷数据 -> 再读 seq, 两次相同才采信
"""

import numpy as np
from multiprocessing import shared_memory

MAGIC = 0x524D3237          # 'RM27'
VERSION = 2                 # v1.7: 头部加了 size 字段；TASK/命令区布局变了必须升版本
MAX_RETRY = 4000

HDR = np.dtype([
    ("magic", "<u4"), ("version", "<u4"),
    ("seq", "<u8"), ("t", "<f8"),
    ("nq", "<i4"), ("nv", "<i4"), ("nu", "<i4"), ("flags", "<u4"),
    ("size", "<u4"),        # 本端算出的布局总字节数，两端必须一致
])
FLAG_QUIT = 1          # 上位机置位，物理端优雅退出
FLAG_MANUAL = 2        # 上位机置位 = 手动接管，物理端暂停回放
FLAG_PLAY = 4          # 上位机置位 = 请求把整段任务再放一遍
FLAG_RESET = 8         # 上位机置位 = 把场景复位到初始状态

CMDH = np.dtype([("seq", "<u8"), ("t", "<f8")])

TASK = np.dtype([
    ("phase", "<i4"), ("playing", "<i4"), ("cycle", "<i4"), ("lock", "<i4"),
    ("depth", "<f8"), ("p_deg", "<f8"), ("q_deg", "<f8"), ("grip", "<f8"),
    ("axis", "<f8"), ("c1", "<f8"),
])

# 阶段名：**唯一来源**。run_physical 用 TAG_TO_PHASE 生成 marks，页面上位机通过 /phases 拉取。
PHASES = ["home", "pre", "approach", "close", "settle", "lift", "hold", "roll",
          "staging", "insert", "insert-hold", "insert2", "P", "retreat", "Q", "done"]
# report() 用的标签 -> 阶段下标；没在这里登记的标签会在录制时直接报错
TAG_TO_PHASE = {name: i for i, name in enumerate(PHASES)}


class Layout:
    """根据自由度算出各段偏移，服务端和客户端必须得到同一个布局。"""

    def __init__(self, nq: int, nv: int, nu: int):
        self.nq, self.nv, self.nu = int(nq), int(nv), int(nu)
        self.off_hdr = 0
        self.off_qpos = HDR.itemsize
        self.off_qvel = self.off_qpos + 8 * self.nq
        self.off_cmdh = self.off_qvel + 8 * self.nv
        self.off_ctrl = self.off_cmdh + CMDH.itemsize
        self.off_cmd2 = self.off_ctrl + 8 * self.nu      # 附加指令：核心三关节 + 磁吸
        self.off_task = self.off_cmd2 + 8 * 4
        self.size = self.off_task + TASK.itemsize

    def __repr__(self):
        return (f"Layout(nq={self.nq}, nv={self.nv}, nu={self.nu}, "
                f"size={self.size}B)")


class Channel:
    """一条双向通道：状态(物理->上位机) + 指令(上位机->物理) + 任务块。"""

    @staticmethod
    def _create_or_attach(name, create, size, timeout=6.0):
        """create=True：清掉残留的通道再建。Windows 上如果上位机还映射着它，
        unlink 会失败 —— 等几秒还清不掉就直接**挂上去复用**（布局一致就行），
        这样物理端可以先起、后起，或者中途重启，不用非得先关上位机。"""
        import time as _t
        if not create:
            return shared_memory.SharedMemory(name=name, create=False, size=size)
        t0 = _t.monotonic()
        while True:
            try:
                stale = shared_memory.SharedMemory(name=name)
            except FileNotFoundError:
                break
            try:
                stale.close()
                stale.unlink()
                break
            except Exception:
                stale.close()
                if _t.monotonic() - t0 > timeout:
                    shm = shared_memory.SharedMemory(name=name, create=False, size=size)
                    if shm.size < size:
                        raise RuntimeError(
                            f"共享内存 '{name}' 尺寸不一致（{shm.size} < {size}），先关掉上位机再起")
                    return shm
                _t.sleep(0.3)
        try:
            return shared_memory.SharedMemory(name=name, create=True, size=size)
        except FileExistsError:
            # 名字还在（别的进程映射着）——直接挂上去复用
            return shared_memory.SharedMemory(name=name, create=False, size=size)

    def __init__(self, nq, nv, nu, name="rm27_shm", create=False):
        self.layout = Layout(nq, nv, nu)
        self.li = self.layout
        self.shm = self._create_or_attach(name, create, self.li.size)
        buf = self.shm.buf
        self.h = np.ndarray(1, HDR, buf, self.li.off_hdr)
        self.qpos = np.ndarray(self.li.nq, "<f8", buf, self.li.off_qpos)
        self.qvel = np.ndarray(self.li.nv, "<f8", buf, self.li.off_qvel)
        self.cmdh = np.ndarray(1, CMDH, buf, self.li.off_cmdh)
        self.ctrl = np.ndarray(self.li.nu, "<f8", buf, self.li.off_ctrl)
        self.cmd2 = np.ndarray(4, "<f8", buf, self.li.off_cmd2)
        self.task = np.ndarray(1, TASK, buf, self.li.off_task)

        if create:
            self.h["magic"][0] = MAGIC
            self.h["version"][0] = VERSION
            self.h["nq"][0] = self.li.nq
            self.h["nv"][0] = self.li.nv
            self.h["nu"][0] = self.li.nu
            self.h["seq"][0] = 0
            self.h["t"][0] = 0.0
            self.h["flags"][0] = 0
            self.h["size"][0] = self.li.size
            self.cmdh["seq"][0] = 0
            self.ctrl[:] = 0.0
            self.cmd2[:] = 0.0
            self.task["phase"][0] = 0
            self.task["playing"][0] = 0
            self.task["cycle"][0] = 0
            self.task["lock"][0] = 0
            self.task["depth"][0] = 0.0
            self.task["p_deg"][0] = 0.0
            self.task["q_deg"][0] = 0.0
            self.task["grip"][0] = 0.0
            self.task["axis"][0] = 0.0
        else:
            if int(self.h["magic"][0]) != MAGIC:
                raise RuntimeError("共享内存 magic 不匹配：不是本程序的通道，或服务端没起来")
            if int(self.h["version"][0]) != VERSION:
                raise RuntimeError(f"协议版本不一致: {int(self.h['version'][0])} != {VERSION}")
            for k in ("nq", "nv", "nu"):
                if int(self.h[k][0]) != getattr(self.li, k):
                    raise RuntimeError(f"模型维度不一致: {k}")
            if int(self.h["size"][0]) != self.li.size:
                raise RuntimeError(
                    f"布局不一致: 物理端 {int(self.h['size'][0])}B != 本端 {self.li.size}B"
                    "（多半是版本/字段对不上，请两端用同一版代码）")

    # ---------------------------------------------------------------- 状态
    def publish_state(self, t, qpos, qvel, task=None):
        h = self.h
        h["seq"][0] = h["seq"][0] + 1
        h["t"][0] = t
        self.qpos[:] = qpos
        self.qvel[:] = qvel
        if task is not None:
            for k, v in task.items():
                self.task[k][0] = v
        h["seq"][0] = h["seq"][0] + 1

    def read_state(self):
        h = self.h
        for _ in range(MAX_RETRY):
            s1 = int(h["seq"][0])
            if s1 & 1:
                continue
            t = float(h["t"][0])
            qpos = self.qpos.copy()
            qvel = self.qvel.copy()
            task = {k: self.task[k][0].item() for k in TASK.names}
            if int(h["seq"][0]) == s1:
                return t, qpos, qvel, task
        return None

    # ---------------------------------------------------------------- 指令
    def publish_cmd(self, t, ctrl, ext=None):
        c = self.cmdh
        c["seq"][0] = c["seq"][0] + 1
        c["t"][0] = t
        self.ctrl[:] = ctrl
        if ext is not None:
            self.cmd2[:] = ext
        c["seq"][0] = c["seq"][0] + 1

    def read_cmd(self):
        c = self.cmdh
        for _ in range(MAX_RETRY):
            s1 = int(c["seq"][0])
            if s1 & 1:
                continue
            t = float(c["t"][0])
            ctrl = self.ctrl.copy()
            ext = self.cmd2.copy()
            if int(c["seq"][0]) == s1:
                return t, ctrl, ext
        return None

    # ---------------------------------------------------------------- 标志
    def _set_flag(self, bit, on):
        f = int(self.h["flags"][0])
        self.h["flags"][0] = (f | bit) if on else (f & ~bit)

    def _get_flag(self, bit):
        return bool(int(self.h["flags"][0]) & bit)

    def request_quit(self): self._set_flag(FLAG_QUIT, True)
    def quit_requested(self): return self._get_flag(FLAG_QUIT)

    def set_manual(self, on): self._set_flag(FLAG_MANUAL, on)
    def manual_mode(self): return self._get_flag(FLAG_MANUAL)

    def request_play(self): self._set_flag(FLAG_PLAY, True)
    def play_requested(self): return self._get_flag(FLAG_PLAY)
    def clear_play(self): self._set_flag(FLAG_PLAY, False)

    def request_reset(self): self._set_flag(FLAG_RESET, True)
    def reset_requested(self): return self._get_flag(FLAG_RESET)
    def clear_reset(self): self._set_flag(FLAG_RESET, False)

    def close(self):
        try:
            self.shm.close()
        except Exception:
            pass

    def unlink(self):
        try:
            self.shm.unlink()
        except FileNotFoundError:
            pass
