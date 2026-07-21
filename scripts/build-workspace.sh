#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

mkdir -p artifacts/logs

LOG="$REPO_ROOT/artifacts/logs/workspace-build-$(date +%Y%m%d-%H%M%S).log"
JOBS="${CATKIN_JOBS:-8}"

echo "项目目录：$REPO_ROOT"
echo "并行任务数：$JOBS"
echo "完整日志：$LOG"

if [ -e build ] || [ -e devel ]; then
    echo "检测到已有容器构建产物，本次将执行增量编译。"
else
    echo "未检测到 build/devel，本次为干净编译。"
fi

set +e
sudo docker compose \
    --file compose.yaml \
    run \
    --rm \
    --entrypoint bash \
    --env CATKIN_JOBS="$JOBS" \
    simenv \
    -s 2>&1 <<'CONTAINER_SCRIPT' | tee "$LOG"

set -Eeo pipefail

source /opt/ros/noetic/setup.bash
cd /workspace/SimEnv

echo "===== 容器构建环境 ====="
echo "ROS_DISTRO=$ROS_DISTRO"
echo "CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
echo "TORCH_CUDA_ARCH_LIST=$TORCH_CUDA_ARCH_LIST"
echo "并行任务数=$CATKIN_JOBS"

catkin_make \
    -j"$CATKIN_JOBS" \
    -l"$CATKIN_JOBS" \
    -DCMAKE_BUILD_TYPE=Release \
    -DTORCH_CUDA_ARCH_LIST=12.0 \
    -DLOCAL_SENSING_CUDA_ARCH=120 \
    -DENABLE_CUDA=OFF
CONTAINER_SCRIPT

BUILD_STATUS="${PIPESTATUS[0]}"
set -e

echo
echo "工作区编译退出码：$BUILD_STATUS"
echo "完整日志：$LOG"

if [ "$BUILD_STATUS" -ne 0 ]; then
    echo "编译失败。请保留 build、devel 和日志，以便增量修复。"
    exit "$BUILD_STATUS"
fi

echo "SimEnv完整工作区编译成功。"