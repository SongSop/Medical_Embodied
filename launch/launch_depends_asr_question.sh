#!/bin/bash
set -e

# ================================
# tmux session 名
# ================================
TMUX_SESSION="depends_asr_question"


# ================================
# Python 节点列表
# ================================
PYTHON_NODES=(
    # 是否需要呼叫护士
    "../src/llm_node_py/llm_node_py/ros2_node_nurse_alert.py"
    # 一次性的 tts
    "../src/llm_node_py/llm_node_py/ros2_node_tts_oneshot.py"
    # 大模型-知识库
    "../src/llm_node_py/llm_node_py/ros2_node_llm_medical.py"
    # 大模型-联网查找
    "../src/llm_node_py/llm_node_py/ros2_node_llm_network.py"
    # 大模型路由器，负责把 question 分发给不同的节点
    "../src/llm_node_py/llm_node_py/ros2_node_question_router.py"
    # 管理历史对话
    "../src/llm_node_py/llm_node_py/ros2_node_dialog_history.py"
)


# ================================
# C++ 节点列表
# ================================
CPP_NODES=(
    # ASR 节点
    "../install/llm_node_cpp/lib/llm_node_cpp/ros_node_keyword_asr"
    # ASR, KWS 的管理节点
    # "../../../devel/lib/llm_node/ros_node_keyword_asr"
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

