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
src/arm_mirror/     代码（protocol / serial_link / model / cli）
model/arm_v7/       MJCF + STL 网格
config/joints.yaml  六轴符号与零位
docs/               协议与标定说明
examples/           一键启动脚本
tests/              协议与映射单测
```

## 已知限制

- 只做姿态镜像，不做动力学/接触；
- 需真机在线；串口断开时程序退出（可查看 `print` 模式排查）；
- 仿真模型 `joint3` 的范围（0~72°）与实机不完全一致，因此默认**不夹限位**，需要时加 `--clamp`。

## 许可

MIT，见 [LICENSE](LICENSE)。
