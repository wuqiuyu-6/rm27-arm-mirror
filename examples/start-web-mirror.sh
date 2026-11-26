#!/usr/bin/env bash
# RM27 六轴臂镜像（网页版）
set -e
cd "$(dirname "$0")/.."

PORT="${PORT:-8007}"
if [ -x ".venv/bin/python" ]; then
    PY=".venv/bin/python"
else
    PY="${PYTHON:-python3}"
fi

echo ""
echo "  [rm27-arm-mirror] web HMI  http://127.0.0.1:${PORT}"
echo "  串口见 config/joints.yaml；没有真机时加 --no-serial"
echo ""

if command -v xdg-open >/dev/null 2>&1; then
    ( sleep 1; xdg-open "http://127.0.0.1:${PORT}" >/dev/null 2>&1 || true ) &
fi

exec "$PY" web_mirror.py --port "$PORT" "$@"