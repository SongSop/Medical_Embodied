#!/bin/bash
set -e

# ================================
# tmux session 名
# ================================
TMUX_SESSION="dialog_manager"


PYTHON_NODES=(
    # 
    # "../scripts/keep_dia.py"

    # ros node 管理节点
    "../src/llm_node_py/llm_node_py/ros2_node_dialog_manager.py"

    # 行为树接口节点
    "../src/dialog/dialog/bt_hci_interface.py"
)

# ================================
# 1️⃣B 定义要启动的 C++ 节点可执行文件
# ================================
CPP_NODES=(

)

# ================================
# launch 脚本
# ================================
LAUNCH_SCRIPT="./launch_ros_nodes.sh"

# ================================
# 获取当前脚本所在目录
# ================================
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[INFO] Base directory: $BASE_DIR"

# ================================
# launch 脚本路径（绝对路径）
# ================================
LAUNCH_SCRIPT="$BASE_DIR/launch_ros_nodes.sh"


# ================================
# 检查 launch 脚本
# ================================
if [[ ! -f "$LAUNCH_SCRIPT" ]]; then
    echo "[ERROR] launch_ros_nodes.sh not found"
    exit 1
fi


echo "[INFO] Starting dialog system..."
echo "[INFO] tmux session: $TMUX_SESSION"


# ================================
# 合并节点列表
# ================================
ALL_NODES=("${PYTHON_NODES[@]}" "${CPP_NODES[@]}")


# ================================
# 调用 launch
# ================================
$LAUNCH_SCRIPT "$TMUX_SESSION" "${ALL_NODES[@]}"


echo
echo "[INFO] Dialog system started"
echo "[INFO] Attach tmux:"
echo "tmux attach -t $TMUX_SESSION"
