#!/bin/bash
set -e

# ================================
# 获取当前脚本所在目录
# ================================
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[INFO] Base directory: $BASE_DIR"

# ================================
# 参数检查
# ================================
if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <tmux_session> <node1> <node2> ..."
    exit 1
fi

# ================================
# tmux session
# ================================
TMUX_SESSION="$1"
shift

# ================================
# conda 环境
# ================================
CONDA_ENV_NAME="qwen_tts_online"
PYTHON_EXEC="$(conda run -n "$CONDA_ENV_NAME" which python)"

if [[ ! -x "$PYTHON_EXEC" ]]; then
    echo "[ERROR] Cannot find python in env: $CONDA_ENV_NAME"
    exit 1
fi
echo "[INFO] Using python: $PYTHON_EXEC"

# ================================
# 如果 session 已存在
# ================================
if tmux has-session -t "$TMUX_SESSION" 2>/dev/null; then
    echo "[WARN] Killing existing session: $TMUX_SESSION"
    tmux kill-session -t "$TMUX_SESSION"
fi

# ================================
# 创建 session
# ================================
tmux new-session -d -s "$TMUX_SESSION"

WINDOW_INDEX=0

# ================================
# 启动节点
# ================================
for NODE in "$@"
do
    # -------------------------------
    # 转换成绝对路径（基于脚本目录）
    # -------------------------------
    if [[ "$NODE" == /* ]]; then
        # 已经是绝对路径
        ABS_NODE="$NODE"
    else
        ABS_NODE="$BASE_DIR/$NODE"
    fi

    if [[ "$WINDOW_INDEX" -eq 0 ]]; then
        TARGET="$TMUX_SESSION"
    else
        tmux new-window -t "$TMUX_SESSION"
        TARGET="$TMUX_SESSION:$WINDOW_INDEX"
    fi

    echo "[INFO] Starting: $ABS_NODE"

    # -------------------------------
    # 判断 Python / C++ 执行方式
    # -------------------------------
    if [[ "$ABS_NODE" == *.py ]]; then
        tmux send-keys -t "$TARGET" "$PYTHON_EXEC $ABS_NODE" C-m
    else
        tmux send-keys -t "$TARGET" "$ABS_NODE" C-m
    fi

    WINDOW_INDEX=$((WINDOW_INDEX + 1))
done

# ================================
# 提示信息
# ================================
echo
echo "[INFO] All nodes started in tmux session: $TMUX_SESSION"
echo "[INFO] Attach: tmux attach -t $TMUX_SESSION"
echo "[INFO] Detach: Ctrl+b then d"