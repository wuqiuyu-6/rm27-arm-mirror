# rm27-arm-mirror

把**真实六轴小臂**的关节角实时映射到 MuJoCo 仿真臂上：真机怎么动，仿真就怎么动。

- 数据源：控制板串口（默认 `COM6 @ 115200`），58 字节/帧、约 32 Hz
- 仿真模型：`model/arm_v7/arm_v7_scene.xml`（六轴，无夹爪）
- 方向：**单向**（只读真机，不向真机发任何指令）

## 快速开始

```bash
git clone https://github.com/wuqiuyu-6/rm27-arm-mirror.git
cd rm27-arm-mirror
python -m venv .venv
.venv\Scripts\activate          # Windows
pip install -e .
```

接好串口，然后：

```bash
arm-mirror view                 # 实时窗口（默认）
arm-mirror print                # 只打印关节角 + 力矩
arm-mirror record --out mir.mp4 # 录视频
```

或者直接双击 `examples/start-mirror.bat`（未 `pip install` 时它用 `python -m arm_mirror`，
需要先设置 `PYTHONPATH=src`，或在仓库根目录执行一次 `pip install -e .`）。

### 网页上位机（3D 页面 + 夹爪按钮）

```bash
python tools/export_scene.py        # 生成 web/scene.json（模型改了才需要重跑）
python web_mirror.py --port 8007    # 打开 http://127.0.0.1:8007
```

也可以双击 `examples/start-web-mirror.bat`（会自动开浏览器）。

页面内容：

| 区域 | 说明 |
|---|---|
| 左侧 | three.js 3D 视图，随真机姿态实时更新（左键旋转 / 右键平移 / 滚轮缩放） |
| 右侧「关节角」 | J1~J6 实时角度 |
| 右侧「夹爪」 | **打开 / 关闭**两个按钮，分别把夹爪驱动到张开（28 mm）和合拢 |
| 右侧「状态」 | 串口连接状态、仿真帧率 |

说明：

- 页面的 6 轴**只做镜像**（写 qpos，不接收指令）；
- **夹爪是仿真里的**：按钮控制仿真夹爪的开合（模型里 `gripper_base` + `finger_l/finger_r` 两个滑轨 + 两个位置伺服）；
- 没有真机时可用 `python web_mirror.py --no-serial` 只看页面（机械臂停在零位，夹爪按钮仍可用）。

## 帧格式

来自控制板固件 `User/gravity/gravity_capture.c`：

```
A5 5A | uint16 seq | uint32 tick(ms) | float q[6] | float tau[6] | CRC16
  2        2            4                24             24          2   = 58 B
```

- `q[6]`：关节角（rad），URDF/DH 模型坐标（固件已完成编码器→模型角换算）
- `tau[6]`：反馈力矩（N·m）
- `CRC16`：poly 0x8408 / init 0xFFFF，覆盖前 56 字节

细节与出处见 [`docs/protocol.md`](docs/protocol.md)。

## 关节映射

默认配置（`config/joints.yaml`）：

```
sim_q = sign * raw_q + zero
```

六个轴 `sign` 全为 `-1`（实机与模型的关节轴方向整体相反），`zero` 目前全 0。
逐轴标定方法与历史取值见 [`docs/calibration.md`](docs/calibration.md)。

## 目录

```
src/arm_mirror/     桌面端镜像（protocol / serial_link / model / cli）
web_mirror.py       网页上位机（3D 页面 + 夹爪按钮）
tools/export_scene.py   把 MJCF 导出成 web/scene.json
web/                页面与 three.js（index.html、vendor/、scene.json）
model/arm_v7/       MJCF + STL 网格（arm_v7_scene.xml / arm_v7_gripper.xml）
config/joints.yaml  六轴符号与零位
docs/               协议与标定说明
examples/           一键启动脚本（桌面窗口 / 网页）
tests/              协议与映射单测
```

## 已知限制

- 只做姿态镜像，不做动力学/接触；
- 需真机在线；串口断开时程序退出（可查看 `print` 模式排查）；
- 仿真模型 `joint3` 的范围（0~72°）与实机不完全一致，因此默认**不夹限位**，需要时加 `--clamp`；
- 夹爪是**仿真件**：按钮只驱动仿真夹爪，不会给真机发指令（真机的夹爪数据不在 58 字节帧里）。

## 许可

MIT，见 [LICENSE](LICENSE)。
