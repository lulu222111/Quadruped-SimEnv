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

# --- Submodule pre-check (fail early if missing) ---
MISSING_SUBMODULES=""
for sm in src/fast_lio src/livox_ros_driver; do
    if [ ! -f "$REPO_ROOT/$sm/CMakeLists.txt" ] && \
       [ -z "$(find "$REPO_ROOT/$sm" -name 'CMakeLists.txt' -maxdepth 2 2>/dev/null | head -1)" ]; then
        MISSING_SUBMODULES="$MISSING_SUBMODULES $sm"
    fi
done

if [ -n "$MISSING_SUBMODULES" ]; then
    echo "ERROR: 以下子模块未初始化或内容缺失:${MISSING_SUBMODULES}" >&2
    echo "" >&2
    echo "请先运行:" >&2
    echo "  git submodule update --init --recursive" >&2
    echo "" >&2
    echo "或使用递归克隆:" >&2
    echo "  git clone --recurse-submodules <repo-url>" >&2
    exit 1
fi

# Nested ikd-Tree submodule check
if [ ! -f "$REPO_ROOT/src/fast_lio/include/ikd-Tree/ikd_Tree.cpp" ]; then
    echo "ERROR: fast_lio 嵌套子模块 include/ikd-Tree 未初始化。" >&2
    echo "请运行:  git submodule update --init --recursive" >&2
    echo "或手动:  cd src/fast_lio && git submodule update --init include/ikd-Tree" >&2
    exit 1
fi

# Patch file existence check
LIVOX_PATCH="$REPO_ROOT/scripts/patches/livox_ros_driver_skip_sdk_clone.patch"
if [ ! -f "$LIVOX_PATCH" ]; then
    echo "ERROR: livox_ros_driver 补丁文件缺失: $LIVOX_PATCH" >&2
    echo "此补丁是离线构建的必要文件，请确认仓库完整。" >&2
    exit 1
fi

echo "所有子模块和依赖已就绪。"

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

# --- SimEnv: apply livox_ros_driver patch (skip Livox-SDK clone when SDK absent) ---
LIVOX_PATCH="/workspace/SimEnv/scripts/patches/livox_ros_driver_skip_sdk_clone.patch"

# Verify submodule present in container
if [ ! -f /workspace/SimEnv/src/livox_ros_driver/livox_ros_driver/CMakeLists.txt ]; then
    echo "FATAL: src/livox_ros_driver submodule not initialized inside container." >&2
    echo "Run 'git submodule update --init --recursive' on the host." >&2
    exit 1
fi

if [ ! -f "$LIVOX_PATCH" ]; then
    echo "FATAL: livox_ros_driver patch file missing inside container: $LIVOX_PATCH" >&2
    exit 1
fi

# Patch: fail-fast — reverse-check (already applied?) then forward-apply.
# If BOTH fail, exit non-zero — no silent continuation with unpatched code.
if git -C /workspace/SimEnv/src/livox_ros_driver apply --reverse --check "$LIVOX_PATCH" >/dev/null 2>&1; then
    echo "livox_ros_driver: patch already applied (idempotent skip)"
elif git -C /workspace/SimEnv/src/livox_ros_driver apply "$LIVOX_PATCH" 2>&1; then
    echo "livox_ros_driver: SDK-clone-skip patch applied"
else
    echo "FATAL: failed to apply livox_ros_driver patch." >&2
    echo "The patch is required for offline builds. Check if the submodule" >&2
    echo "commit matches the patch baseline." >&2
    exit 1
fi

# Verify ikd-Tree in container
if [ ! -f /workspace/SimEnv/src/fast_lio/include/ikd-Tree/ikd_Tree.cpp ]; then
    echo "FATAL: fast_lio nested submodule include/ikd-Tree missing inside container." >&2
    echo "Run 'git submodule update --init --recursive' on the host." >&2
    exit 1
fi

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
