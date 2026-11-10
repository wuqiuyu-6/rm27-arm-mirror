# rm27-assembly

RMUC2027 科技核心装配仿真：**1 kHz 离线轨迹回放 + 网页上位机**。

- 物理端：MuJoCo，1 ms 步长，回放录制好的轨迹（`data/rm27_task.npz`）
- 网页端：three.js 3D 视图（60 fps），插值显示、播放/暂停/复位、手动接管、磁吸与核心三关节
- 装配流程：取件 → 插入 100 mm → 绕 P 轴 90° → 绕 Q 轴 51.6° → 保持

## 快速开始

```bash
git clone -b assembly-sim https://github.com/wuqiuyu-6/rm27-arm-mirror.git
cd rm27-arm-mirror
pip install -e .
python tools/export_scene.py     # 生成 web/scene.json（约 17 MB，不入库）
rm27-assembly all                # 起物理端 + 网页端
```

浏览器打开 **http://127.0.0.1:8005**，点「播放」。

也可以直接双击 `examples/start-assembly.bat`。

## 实测指标

| 项 | 值 |
|---|---|
| 插入深度 | **100.0 mm** |
| P 轴 | **90.0°** |
| Q 轴 | **51.6°** |
| 销心误差 | 0.0 mm |
| 物理端发布频率 | ≈1000 帧/秒 |
| 网页端 | 60 fps，延迟 50 ms |

## 架构

```
┌──────────────┐   seqlock   ┌──────────────┐   HTTP    ┌──────────┐
│ server.py    │ ──共享内存──►│ hmi.py       │ ────────► │ 浏览器    │
│ 1 kHz 回放   │  qpos/qvel  │ 60 fps 插值   │ /pose     │ three.js │
└──────────────┘  任务块     └──────────────┘ /state    └──────────┘
```

- 共享内存布局见 [`docs/protocol.md`](docs/protocol.md)（seqlock 防撕裂）
- 装配流程与规则依据见 [`docs/task.md`](docs/task.md)

## 目录

```
src/rm27_assembly/   包（paths / protocol / interp / server / hmi / cli）
tools/               导出场景、重录轨迹
model/               场景 MJCF + 网格（37 MB）
data/rm27_task.npz   录制轨迹（1.5 MB）
web/                 页面 + three.js + 生成的 scene.json
config/scene.yaml    共享内存名 / 端口 / 期望指标
docs/                协议与流程说明
examples/            一键启动
tests/               共享内存协议单测
```

## 已知限制

- 轨迹是**离线录制回放**，不是在线规划；手动把单元推歪后重放不会自适应；
- 网页端 3D 用全量网格（约 66 万三角面，scene.json 17 MB），首次加载 1~3 s；
- 物理端的核心三关节在手动模式下是运动学写入，不是真伺服；
- Q 轴由核心机构驱动（平行两指夹爪无法传递绕开合轴的力矩），不是机械臂直接转。

## 许可

MIT，见 [LICENSE](LICENSE)。
