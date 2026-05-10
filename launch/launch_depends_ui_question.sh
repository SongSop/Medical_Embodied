#!/bin/bash
set -e

# ================================
# tmux session 名
# ================================
TMUX_SESSION="depends_ui_question"


PYTHON_NODES=(
    # 实时的 tts
    # "../src/llm_node_py/llm_node_py/ros2_node_tts_realtime.py"
    
    # 实时 TTS + 流式 TTS
    "../src/llm_node_py/llm_node_py/ros2_node_tts_local.py"


    # 输入大模型的流式文本，向外输出整个文本
    "../src/llm_node_py/llm_node_py/ros2_node_stream_text.py"
    # 管理多个 question
    "../src/llm_node_py/llm_node_py/ros2_node_question_manager.py"
    # 把语音输出速度和文本输出速度进行对齐
    "../src/llm_node_py/llm_node_py/ros2_node_llm_ui_stream_bridge.py"
    # 给 ui 节点单独开了一个大模型
    "../src/llm_node_py/llm_node_py/ros2_node_llm_ui.py"
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
