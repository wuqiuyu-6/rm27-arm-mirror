"""
物理端（RM27 装配）：1 kHz 回放离线录好的装配轨迹，把状态写进共享内存。

轨迹由 `model/physical/run_physical.py --record` 生成（vendor/rm27_task.npz），
内含每毫秒的 ctrl[8]、核心三关节 qpos、两个 weld 的开关与相对位姿、插入深度、夹持力。

用法：
  python sim_server.py                 # 建通道、复位、等上位机点「播放」
  python sim_server.py --autoplay      # 起来就放
  python sim_server.py --duration 0    # 一直跑（默认），Ctrl+C 停
"""

import os
os.environ["MUJOCO_GL"] = "disable"

import argparse
import sys
import time
from pathlib import Path

import numpy as np
import mujoco

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from . import paths
from .protocol import Channel, PHASES

SCENE = paths.SCENE
TASK = paths.NPZ


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--shm-name", default=paths.SHM_NAME)
    ap.add_argument("--duration", type=float, default=0.0, help="0 = 一直跑")
    ap.add_argument("--autoplay", action="store_true")
    ap.add_argument("--speed", type=float, default=1.0)
    args = ap.parse_args(argv)

    if not TASK.is_file():
        raise SystemExit(f"缺少 {TASK}，先在 model/physical 里跑 python run_physical.py --record")

    rec = np.load(TASK)
    ctrl = rec["ctrl"]; core = rec["core"]
    park = rec["park"]; mount = rec["mount"]; eqd = rec["eqd"]
    brake = rec["brake"] if "brake" in rec else np.zeros(len(mount), dtype=np.int32)
    depth = rec["depth"]; grip = rec["grip"]; marks = rec["marks"]
    qpos0 = rec["qpos0"]; dt = float(rec["dt"])
    n = len(ctrl)
    marks_i = marks[:, 0].astype(int); marks_p = marks[:, 1].astype(int)

    m = mujoco.MjModel.from_xml_path(str(SCENE))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)

    def jadr(name):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
        return j
    core_q = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
              for n in ("c_joint1", "c_joint2", "c_joint3")]
    core_d = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
              for n in ("c_joint1", "c_joint2", "c_joint3")]
    eq_park = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "park")
    eq_mount = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "mount")
    g_pin = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "pin")
    b3 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "c_link3")
    bu = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "unit")
    uj_q = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "unit_free")]
    lock_on = False
    manual_unlocked = False

    print(f"场景 {SCENE.name}  nq={m.nq} nv={m.nv} nu={m.nu}")
    print(f"轨迹 {n} 步 / {n * dt:.1f}s   dt={dt * 1000:.0f} ms")

    ch = Channel(m.nq, m.nv, m.nu, name=args.shm_name, create=True)
    print(f"共享内存 '{args.shm_name}' {ch.layout}")
    ch.set_manual(False)

    def reset_scene():
        d.qpos[:] = qpos0
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)

    reset_scene()
    i = 0
    playing = args.autoplay
    cycle = 0
    ch.set_manual(False)

    t_sim = 0.0
    t_wall0 = time.perf_counter()
    t_stat = t_wall0
    n_pub = 0
    speed = max(0.1, args.speed)
    print("就绪：等上位机点「播放」…" if not playing else "自动播放中…")

    try:
        while True:
            # ---- 上位机请求 ----
            if ch.quit_requested():
                print("\n收到上位机的退出请求。")
                break
            if ch.reset_requested():
                ch.clear_reset()
                reset_scene()
                i = 0
                playing = False
                print("  复位到初始状态。")
            if ch.play_requested():
                ch.clear_play()
                reset_scene()
                i = 0
                playing = True
                cycle += 1
                print(f"  >> 开始第 {cycle} 次装配")
            manual = ch.manual_mode()
            if manual:
                playing = False

            # ---- 一步物理 ----
            locked = False
            if manual:
                st = ch.read_cmd()
                if st is not None:
                    _, cmd_ctrl, ext = st
                    d.ctrl[:] = cmd_ctrl[:m.nu]
                    # 核心三关节手动通道（ext[0:3]），磁吸 ext[3]
                    d.qpos[core_q] = ext[0:3]
                    d.qvel[core_d] = 0.0
                    want_lock = ext[3] > 0.5
                    # 插入到位后自动吸住（真实磁吸就是这样：一插上就吸住），
                    # 否则手动松开夹爪单元会直接掉下来
                    if not want_lock and not lock_on and not manual_unlocked:
                        pc = d.geom_xpos[g_pin]
                        ax = d.geom_xmat[g_pin].reshape(3, 3)[:, 2]
                        free = pc - ax * float(m.geom_size[g_pin][1])
                        uo = d.qpos[uj_q:uj_q + 3]
                        dep = float(np.dot(uo - free, ax))
                        uax = d.xmat[bu].reshape(3, 3)[:, 2]
                        axd = float(np.degrees(np.arccos(np.clip(abs(np.dot(uax, ax)), -1, 1))))
                        # 侧向偏差：销子自由端到"单元轴线"的垂距，>15 mm 说明没对准孔
                        lat = uo - free
                        lat = lat - uax * float(np.dot(lat, uax))
                        lateral = float(np.linalg.norm(lat))
                        if dep > 0.080 and axd < 15.0 and lateral < 0.015:
                            want_lock = True
                            print(f"  插入到位 -> 磁吸自动吸住  depth={dep*1000:.1f}mm axis={axd:.1f}°")
                    if want_lock and not lock_on:
                        # 激活磁吸（weld）：按当前相对位姿设 relpose
                        R1 = d.xmat[b3].reshape(3, 3); p1 = d.xpos[b3]
                        R2 = d.xmat[bu].reshape(3, 3); p2 = d.xpos[bu]
                        qq = np.zeros(4)
                        mujoco.mju_mat2Quat(qq, (R1.T @ R2).flatten())
                        m.eq_data[eq_mount][0:3] = 0.0
                        m.eq_data[eq_mount][3:6] = R1.T @ (p2 - p1)
                        m.eq_data[eq_mount][6:10] = qq
                        m.eq_data[eq_mount][10] = 1.0
                        d.eq_active[eq_mount] = 1
                        lock_on = True
                        print("  磁吸 ON（手动）")
                    elif not want_lock and lock_on:
                        d.eq_active[eq_mount] = 0
                        lock_on = False
                        manual_unlocked = True     # 用户主动松开磁吸，不再自动吸
                        print("  磁吸 OFF（手动）")
                    locked = want_lock
                mujoco.mj_step(m, d)
            elif playing and i < n:
                d.ctrl[:] = ctrl[i]
                d.qpos[core_q] = core[i]
                d.qvel[core_d] = 0.0          # 必须清速度，否则每步"瞬移"会把约束冲爆
                d.eq_active[eq_park] = park[i]
                # 机构抱闸：录制里每个阶段决定（臂驱动时不抱）
                for _n in ("c_joint2", "c_joint3"):
                    _j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, _n)
                    m.dof_frictionloss[m.jnt_dofadr[_j]] = 0.5 if brake[i] else 0.0
                d.eq_active[eq_mount] = mount[i]
                m.eq_data[eq_mount][0:10] = eqd[i][0:10]
                mujoco.mj_step(m, d)
                i += 1
                if i >= n:
                    playing = False
                    print("  == 本次装配结束（点播放可重放）")
            else:
                # 暂停：保持最后一帧
                mujoco.mj_forward(m, d)

            t_sim += dt
            n_pub += 1
            # 按真实时间对齐：跑到前面就睡一小会儿，避免 100% 占一核
            _ahead = (t_wall0 + t_sim) - time.perf_counter()
            if _ahead > 0.0002:
                time.sleep(min(0.0015, _ahead))

            # ---- 发布 ----
            di = min(i, n - 1)
            # 实时算插入深度和轴对齐（手动模式也一样）
            pc = d.geom_xpos[g_pin].copy()
            ax = d.geom_xmat[g_pin].reshape(3, 3)[:, 2]
            half = float(m.geom_size[g_pin][1])
            free = pc - ax * half
            uo = d.qpos[uj_q:uj_q + 3]
            live_depth = float(np.dot(uo - free, ax)) * 1000.0
            uax = d.xmat[bu].reshape(3, 3)[:, 2]
            axis_deg = float(np.degrees(np.arccos(np.clip(abs(np.dot(uax, ax)), -1, 1))))
            # 当前正在执行的阶段 = 下一个 mark；末尾用 done
            ph = PHASES.index("done")
            for k in range(len(marks_i)):
                if marks_i[k] > di:
                    ph = int(marks_p[k])
                    break
            ch.publish_state(t_sim, d.qpos, d.qvel, task=dict(
                phase=int(ph),
                playing=int(playing),
                cycle=int(cycle),
                lock=int(lock_on),
                depth=live_depth,
                p_deg=float(np.degrees(d.qpos[core_q[1]])),
                q_deg=float(np.degrees(d.qpos[core_q[2]])),
                grip=float(np.mean(np.abs(d.actuator_force[6:8]))),
                axis=axis_deg,
                c1=float(d.qpos[core_q[0]]),
            ))

            now = time.perf_counter()
            if now - t_stat >= 1.0:
                rate = n_pub / (now - t_stat)
                tag = "PAUSE" if not playing else ("MANUAL" if manual else "PLAY")
                print(f"  t={t_sim:7.2f}s  发布 {rate:7.1f} 帧/秒  {tag}  "
                      f"阶段={PHASES[ph]:{len(max(PHASES, key=len))}s}  "
                      f"深度={depth[di]*1000:6.1f}mm")
                t_stat, n_pub = now, 0

            if args.duration and t_sim >= args.duration:
                break

            # 实时节拍（按 dt / speed 走）
            target = t_wall0 + t_sim / speed
            sleep = target - time.perf_counter()
            if sleep > 0.0002:
                time.sleep(min(sleep, 0.002))
    except KeyboardInterrupt:
        pass
    finally:
        print("\n退出：关闭共享内存。")
        ch.close()
        ch.unlink()


if __name__ == "__main__":
    main()
