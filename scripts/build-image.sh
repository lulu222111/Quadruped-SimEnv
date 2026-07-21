#!/usr/bin/env bash
set -Eeuo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

mkdir -p artifacts/logs

LOG="$REPO_ROOT/artifacts/logs/docker-build-$(date +%Y%m%d-%H%M%S).log"

echo "项目目录：$REPO_ROOT"
echo "构建日志：$LOG"
echo "开始构建 simenv:noetic-cu128"

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

echo "Docker镜像构建成功。"