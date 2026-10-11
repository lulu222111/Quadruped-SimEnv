#!/usr/bin/env bash
set -Eeuo pipefail

# ============================================================================
# setup-user-container.sh — SimEnv 多用户容器配置脚本
# ============================================================================
#
# 功能：
#   1. 自动检测当前用户信息（用户名、UID、GID、HOME、DISPLAY）
#   2. 检测当前仓库绝对路径
#   3. 生成适用于当前成员的 .env 文件
#   4. 如果 .env 已存在，生成 .env.new 供用户比较
#   5. 运行 docker compose config 进行静态验证
#
# 安全约束：
#   - 不覆盖已有 .env
#   - 不执行 docker system prune
#   - 不删除或停止任何容器
#   - 不启动 Gazebo 仿真
#   - 不修改其他用户文件
# ============================================================================

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# ---- 检测当前用户信息 ----
CURRENT_USER="$(whoami)"
CURRENT_UID="$(id -u)"
CURRENT_GID="$(id -g)"
CURRENT_HOME="$(eval echo ~"$CURRENT_USER")"
CURRENT_DISPLAY="${DISPLAY:-:0}"

echo "============================================"
echo " SimEnv 多用户容器配置"
echo "============================================"
echo "  用户名：          $CURRENT_USER"
echo "  UID：             $CURRENT_UID"
echo "  GID：             $CURRENT_GID"
echo "  HOME：            $CURRENT_HOME"
echo "  DISPLAY：         $CURRENT_DISPLAY"
echo "  仓库路径：        $REPO_ROOT"
echo "============================================"
echo ""

# ---- 检查仓库路径是否在当前用户 HOME 下 ----
case "$REPO_ROOT" in
    "$CURRENT_HOME"*)
        echo "✓ 仓库位于当前用户 HOME 下"
        ;;
    *)
        echo "⚠ 警告：仓库路径 ($REPO_ROOT) 不在当前用户 HOME ($CURRENT_HOME) 下"
        echo "  请确认这是您自己的 SimEnv 仓库"
        ;;
esac

# ---- 推断镜像名称 ----
if [ "$CURRENT_UID" = "1000" ]; then
    # UID 1000 可直接使用共享镜像
    RECOMMENDED_IMAGE="simenv:noetic-cu128"
    echo ""
    echo "当前 UID 为 1000，可直接使用共享镜像 simenv:noetic-cu128"
else
    # 非 1000 UID 需要个人镜像
    RECOMMENDED_IMAGE="simenv:noetic-cu128-${CURRENT_USER}"
    echo ""
    echo "当前 UID 为 ${CURRENT_UID}（非 1000），推荐使用个人镜像标签"
    echo "推荐镜像：$RECOMMENDED_IMAGE"
fi

# ---- 推断容器名称和主机名 ----
CONTAINER_NAME="simenv-${CURRENT_USER}"
HOSTNAME="simenv-${CURRENT_USER}"

# ---- 推断 Xauthority 路径 ----
XAUTHORITY_PATH="/run/user/${CURRENT_UID}/gdm/Xauthority"
if [ ! -f "$XAUTHORITY_PATH" ]; then
    # 尝试其他常见位置
    if [ -f "${CURRENT_HOME}/.Xauthority" ]; then
        XAUTHORITY_PATH="${CURRENT_HOME}/.Xauthority"
    fi
fi

# ---- 生成 .env 内容 ----
read -r -d '' ENV_CONTENT <<EOF || true
# SimEnv 容器配置 — 由 setup-user-container.sh 自动生成
# 生成时间：$(date '+%Y-%m-%d %H:%M:%S')
# 用户：$CURRENT_USER (UID=$CURRENT_UID, GID=$CURRENT_GID)

COMPOSE_PROJECT_NAME=$CONTAINER_NAME
SIMENV_IMAGE=$RECOMMENDED_IMAGE
SIMENV_CONTAINER_NAME=$CONTAINER_NAME
SIMENV_HOSTNAME=$HOSTNAME
SIMENV_WORKSPACE=$REPO_ROOT
LOCAL_UID=$CURRENT_UID
LOCAL_GID=$CURRENT_GID
SIMENV_DISPLAY=$CURRENT_DISPLAY
SIMENV_XAUTHORITY=$XAUTHORITY_PATH
EOF

# ---- 决定写入哪个文件 ----
if [ -f "$REPO_ROOT/.env" ]; then
    TARGET="$REPO_ROOT/.env.new"
    echo ""
    echo "============================================"
    echo " .env 已存在，生成 .env.new 供比较"
    echo "============================================"
    echo "已有：$REPO_ROOT/.env"
    echo "新增：$REPO_ROOT/.env.new"
    echo ""
    echo "比较差异："
    echo "  diff -u .env .env.new"
    echo ""
    echo "确认无误后替换："
    echo "  mv .env.new .env"
else
    TARGET="$REPO_ROOT/.env"
    echo ""
    echo "正在生成 .env ..."
fi

echo "$ENV_CONTENT" > "$TARGET"
echo "✓ 已写入：$TARGET"

# ---- 静态验证 ----
echo ""
echo "============================================"
echo " docker compose config 静态验证"
echo "============================================"

# 使用生成的 .env 文件运行 docker compose config
# 如果生成的是 .env.new，需要复制后验证
if [ "$TARGET" = "$REPO_ROOT/.env.new" ]; then
    # 临时使用 .env.new 作为 .env 进行验证
    cp "$REPO_ROOT/.env" "$REPO_ROOT/.env.bak" 2>/dev/null || true
    cp "$REPO_ROOT/.env.new" "$REPO_ROOT/.env"
    set +e
    sudo docker compose --file "$REPO_ROOT/compose.yaml" config --quiet 2>&1
    CONFIG_STATUS=$?
    set -e
    # 恢复原始 .env
    mv "$REPO_ROOT/.env.bak" "$REPO_ROOT/.env" 2>/dev/null || rm -f "$REPO_ROOT/.env"
    # 重新生成 .env.new（因为被 mv 了）
    echo "$ENV_CONTENT" > "$REPO_ROOT/.env.new"
else
    set +e
    sudo docker compose --file "$REPO_ROOT/compose.yaml" config --quiet 2>&1
    CONFIG_STATUS=$?
    set -e
fi

if [ "$CONFIG_STATUS" -eq 0 ]; then
    echo "✓ docker compose config 解析成功"
else
    echo "✗ docker compose config 解析失败（退出码：$CONFIG_STATUS）"
    echo "  请检查上述错误信息并修正 .env 中的配置"
fi

# ---- 验证挂载源 ----
echo ""
echo "============================================"
echo " 挂载源验证"
echo "============================================"
RESOLVED_WORKSPACE="$(cd "$REPO_ROOT" && sudo docker compose --file "$REPO_ROOT/compose.yaml" config 2>/dev/null | grep -oP '/workspace/SimEnv.*source:\s*\K[^ ]+' || echo "无法解析")"
if [ "$RESOLVED_WORKSPACE" = "$REPO_ROOT" ] || [[ "$RESOLVED_WORKSPACE" = "." ]]; then
    echo "✓ 挂载源指向当前仓库：$REPO_ROOT"
else
    echo "挂载源：$RESOLVED_WORKSPACE"
    echo "当前仓库：$REPO_ROOT"
    if [ "$RESOLVED_WORKSPACE" != "无法解析" ] && [ "$RESOLVED_WORKSPACE" != "$REPO_ROOT" ]; then
        echo "⚠ 挂载源与当前仓库不一致，请检查 SIMENV_WORKSPACE 配置"
    fi
fi

# ---- 汇总提示 ----
echo ""
echo "============================================"
echo " 配置完成"
echo "============================================"

if [ "$CURRENT_UID" != "1000" ]; then
    echo ""
    echo "您的 UID ($CURRENT_UID) 不是 1000，需要构建个人镜像："
    echo ""
    echo "  cd $REPO_ROOT"
    echo "  sudo docker compose build simenv"
    echo ""
    echo "构建将复用已有缓存（仅用户创建步骤需要重建，通常数秒完成）。"
fi

if [ "$TARGET" = "$REPO_ROOT/.env.new" ]; then
    echo ""
    echo "请比较并确认 .env.new，然后替换："
    echo ""
    echo "  cd $REPO_ROOT"
    echo "  diff -u .env .env.new"
    echo "  mv .env.new .env"
else
    echo ""
    echo "现在可以启动您的容器："
    echo ""
    echo "  cd $REPO_ROOT"
    echo "  sudo docker compose up -d --no-build simenv"
    echo "  sudo docker compose exec simenv bash"
fi

echo ""
echo "⚠  注意："
echo "  - 使用 host 网络模式，同一时间只能运行一套完整 Gazebo 仿真"
echo "  - 请勿操作其他成员的容器"
echo "  - 请勿执行 docker system prune"
