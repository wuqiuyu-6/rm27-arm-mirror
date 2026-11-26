# rm27-arm-mirror

把**真实六轴小臂**的关节角实时映射到 MuJoCo 仿真臂上：真机怎么动，仿真就怎么动。

- 数据源：控制板串口（默认 `COM8 @ 115200`，留空自动探测），58 字节/帧，实测约 30 Hz
- 仿真模型：`model/arm_v7/arm_v7_scene.xml`（六轴，**无夹爪**）
- 方向：**单向**（只读真机，不向真机发任何指令）
- 平台：Windows / Ubuntu 24.04

## 分支说明

| 分支 | 内容 |
|---|---|
| `main` | **v1.8 真机姿态镜像**（串口关节角 → 仿真臂，桌面窗口 + 网页） |
| `v1.9-manual` | v1.9 键盘手动装配（真机臂 + 科技核心 + 能量单元，真实碰撞，三级对矿） |
| `assembly-sim` | v1.7 装配仿真（1 kHz 离线轨迹回放 + 网页） |

## 快速开始（Windows）

```powershell
git clone https://github.com/wuqiuyu-6/rm27-arm-mirror.git
cd rm27-arm-mirror
python -m venv .venv
.venv\Scripts\activate
pip install -e ".[dev]"
```

接好串口，然后：

```powershell
arm-mirror view                  # 实时窗口（默认）
arm-mirror print                 # 只打印关节角 + 力矩
arm-mirror record --out mir.mp4  # 录视频（需要 ffmpeg）
```

也可以双击 `examples/start-mirror.bat`。

### 网页上位机

```powershell
python tools/export_scene.py        # 生成 web/scene.json（模型改了才需要重跑）
python web_mirror.py --port 8007    # 打开 http://127.0.0.1:8007
```

或双击 `examples/start-web-mirror.bat`（会自动开浏览器）。

页面内容：

| 区域 | 说明 |
|---|---|
| 左侧 | three.js 3D 视图，随真机姿态实时更新（左键旋转 / 右键平移 / 滚轮缩放） |
| 右侧「关节角」 | J1~J6 实时角度 |
| 右侧「夹爪」 | **仅当模型带夹爪时出现**（默认无夹爪，该区域自动隐藏） |
| 右侧「状态」 | 串口连接状态、仿真帧率 |

没有真机时用 `python web_mirror.py --no-serial` 只看页面（机械臂停在零位）。

## 快速开始（Ubuntu 24.04）

```bash
sudo apt update
sudo apt install -y python3-venv libgl1 libegl1 libglew2.2 ffmpeg
git clone -b main https://github.com/wuqiuyu-6/rm27-arm-mirror.git
cd rm27-arm-mirror
python3 -m venv .venv && . .venv/bin/activate
pip install -e ".[dev]"

sudo usermod -aG dialout "$USER"    # 串口权限，重新登录生效
ls /dev/serial/by-id/               # 确认设备名

pytest -q                           # 16 passed
python -m arm_mirror print          # 只看读数
python -m arm_mirror view           # 窗口（需要图形界面）
python -m web_mirror --port 8007    # 网页

./examples/start-web-mirror.sh      # 或直接用脚本
```

平台差异：

| 项 | Windows | Ubuntu |
|---|---|---|
| 串口名 | `COM8` | `/dev/ttyUSB0`（或 `/dev/serial/by-id/...`） |
| 启动脚本 | `examples/*.bat` | `examples/*.sh` |
| 无图形界面 | — | `export MUJOCO_GL=egl` |
| 录 mp4 | 需 ffmpeg | `apt install ffmpeg` |

串口留空时自动探测：Windows 取第一个 `COM` 口，Linux 优先 `/dev/serial/by-id/*`（插拔不变名）。

## 参数

| 参数 | 作用 |
|---|---|
| `--port COM8` | 串口设备（默认读 `config/joints.yaml`） |
| `--baud 115200` | 波特率 |
| `--model PATH` | 换 MJCF（如 `model/arm_v7/arm_v7_gripper.xml`） |
| `--no-serial` | 不接串口，只看页面/窗口 |
| `--clamp` | 超出模型关节范围时夹住 |
| `--no-crc` | 不校验 CRC（仅排查） |
| `--alpha 1.0` | 平滑系数，1 = 不平滑 |
| `--sign` / `--zero` / `--flip` | 临时覆盖映射（持久修改请改 `config/joints.yaml`） |

> 注意：用 `--model` 换成带夹爪的模型后，需要按该模型重新导出 `web/scene.json`，否则页面里的 body 会对不上。

## 帧格式

来自控制板固件 `User/gravity/gravity_capture.c`：

```
A5 5A | uint16 seq | uint32 tick(ms) | float q[6] | float tau[6] | CRC16
  2        2            4                24             24          2   = 58 B
```

- `q[6]`：关节角（rad），URDF/DH 模型坐标（固件已完成编码器→模型角换算）
- `tau[6]`：反馈力矩（N·m）
- `CRC16`：poly 0x8408 / init 0xFFFF，覆盖前 56 字节

细节见 [`docs/protocol.md`](docs/protocol.md)。

## 关节映射

`config/joints.yaml` 是唯一来源（CLI 与网页端都从这里读）：

```
sim_q = sign * raw_q + zero
```

当前取值：`sign` 全 `-1`，`zero` 全 `0`。

> **已知问题**：这组值和固件自带标定（`gravity_config.h`：`position_sign`、`encoder_zero`）不一致，
> 实机 J2/J3 读数映射后会让仿真臂摆出超出原有局限的姿态。复标步骤见
> [`docs/calibration.md`](docs/calibration.md)。

## 关节范围

模型里的关节范围已按固件的电机限位（`motor_protect.c` 的 `motor_limits`）放宽，
以便跟随真机的实际行程：

| 关节 | 放宽后范围（rad） | 固件电机限位 |
|---|---|---|
| joint1 | ±2.0 | ±2.0 |
| joint2 | −1.0 ~ 2.4 | −2.4 ~ 1.0 |
| joint3 | −0.8 ~ 2.2 | −2.2 ~ 0.8 |
| joint4 | ±3.0 | ±3.0 |
| joint5 | ±2.0 | ±2.0 |
| joint6 | −3.3 ~ 3.2 | −3.2 ~ 3.3 |

## 目录

```
src/arm_mirror/          桌面端镜像（config / protocol / serial_link / model / cli）
web_mirror.py            网页上位机（3D 页面 + 关节读数）
tools/export_scene.py    把 MJCF 导出成 web/scene.json
web/                     页面与 three.js（index.html、vendor/、scene.json）
model/arm_v7/            MJCF + STL 网格（arm_v7_scene.xml 默认无夹爪；arm_v7_gripper.xml 带夹爪）
config/joints.yaml       六轴符号与零位（唯一来源）
docs/                    协议与标定说明
examples/                一键启动脚本（Windows .bat / Ubuntu .sh）
tests/                   协议、配置与映射单测
.github/workflows/       CI（ubuntu-latest，Python 3.10 / 3.12）
```

## 已知限制

- 只做**姿态镜像**，不做动力学/接触，不向真机发指令；
- 需真机在线；串口断开时程序退出（`print` 模式便于排查）；
- 六轴映射的 **sign/zero 尚未按固件标定值校准**（见上「已知问题」），姿态可能与真机有固定偏差；
- 夹爪是仿真件：真机的夹爪状态不在 58 字节帧里，按钮只驱动仿真（仅带夹爪模型有）。

## 许可

MIT，见 [LICENSE](LICENSE)。
