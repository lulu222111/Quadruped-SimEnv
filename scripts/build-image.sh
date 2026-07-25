#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

mkdir -p artifacts/logs

LOG="$REPO_ROOT/artifacts/logs/docker-build-$(date +%Y%m%d-%H%M%S).log"

# 读取目标镜像名称（与 compose.yaml 中 ${SIMENV_IMAGE:-simenv:noetic-cu128} 一致）
SIMENV_IMAGE="$(sudo docker compose --file compose.yaml config 2>/dev/null \
    | sed -n 's/^ *image: *//p' | head -1)"
SIMENV_IMAGE="${SIMENV_IMAGE:-simenv:noetic-cu128}"

echo "项目目录：$REPO_ROOT"
echo "目标镜像：$SIMENV_IMAGE"
echo "构建日志：$LOG"
echo "开始构建 $SIMENV_IMAGE"

set +e
sudo docker compose \
    --file compose.yaml \
    build \
    --progress=plain \
    simenv \
    2>&1 | tee "$LOG"

BUILD_STATUS="${PIPESTATUS[0]}"
set -e

echo
echo "Docker构建退出码：$BUILD_STATUS"
echo "完整日志：$LOG"

if [ "$BUILD_STATUS" -ne 0 ]; then
    echo "构建失败。请保留日志，不要清理Docker缓存。"
    exit "$BUILD_STATUS"
fi

echo "Docker镜像构建成功：$SIMENV_IMAGE"