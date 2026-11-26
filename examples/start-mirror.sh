#!/usr/bin/env bash
# RM27 六轴臂姿态镜像（桌面窗口版）
set -e
cd "$(dirname "$0")/.."

if [ -x ".venv/bin/python" ]; then
    PY=".venv/bin/python"
else
    PY="${PYTHON:-python3}"
fi

echo ""
echo "  [rm27-arm-mirror] MuJoCo 窗口（左键旋转 / 右键平移 / 滚轮缩放）"
echo "  串口见 config/joints.yaml，或加 --port /dev/ttyUSB0"
echo ""

exec "$PY" -m arm_mirror view "$@"