#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

mkdir -p artifacts/logs artifacts/tmp

LOG="$REPO_ROOT/artifacts/logs/container-verification-$(date +%Y%m%d-%H%M%S).log"

echo "验证日志：$LOG"

set +e
sudo docker compose \
    --file compose.yaml \
    run \
    --rm \
    --entrypoint bash \
    simenv \
    -lc '
set -Eeo pipefail

source /opt/ros/noetic/setup.bash

set -u

echo "===== 操作系统 ====="
grep "^PRETTY_NAME=" /etc/os-release

echo
echo "===== 当前用户 ====="
id

echo
echo "===== ROS ====="
rosversion -d
rosversion roscpp

echo
echo "===== Gazebo ====="
set +e
gazebo --version
GAZEBO_STATUS=$?
set -e

if [ "$GAZEBO_STATUS" -ne 0 ] && [ "$GAZEBO_STATUS" -ne 255 ]; then
    echo "Gazebo版本检查异常，退出码：$GAZEBO_STATUS"
    exit "$GAZEBO_STATUS"
fi

echo "Gazebo版本检查完成，退出码：$GAZEBO_STATUS"

echo
echo "===== CUDA编译器 ====="
nvcc --version | tail -n 1

echo
echo "===== RTX 5090 ====="
nvidia-smi \
    --query-gpu=name,driver_version,memory.total \
    --format=csv,noheader

echo
echo "===== LibTorch ====="
test -f /opt/libtorch/share/cmake/Torch/TorchConfig.cmake
test -f /opt/libtorch/lib/libtorch.so
test -f /opt/libtorch/lib/libtorch_cuda.so
echo "LibTorch CMake、CPU库和CUDA库均存在"

echo
echo "===== 项目目录写入权限 ====="
TEST_FILE=/workspace/SimEnv/artifacts/tmp/container-write-test
printf "container write test\n" > "$TEST_FILE"
test -s "$TEST_FILE"
rm -f "$TEST_FILE"
echo "项目目录写入权限正常"
' 2>&1 | tee "$LOG"

VERIFY_STATUS="${PIPESTATUS[0]}"
set -e

echo
echo "容器验证退出码：$VERIFY_STATUS"
echo "完整日志：$LOG"

exit "$VERIFY_STATUS"