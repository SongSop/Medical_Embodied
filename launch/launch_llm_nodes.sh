#!/bin/bash
set -e

# ================================
# 脚本列表
# ================================
SCRIPTS=(
    "./launch_depends_asr_question.sh"
    "./launch_depends_ui_question.sh"
    # "./launch_dialog_manager_tmux.sh"
)

# =========================================
# 获取当前脚本所在目录
# =========================================
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "[INFO] Base directory: $BASE_DIR"

# =========================================
# 转换成绝对路径
# =========================================
ABS_SCRIPTS=()
for s in "${SCRIPTS[@]}"; do
    ABS_PATH="$BASE_DIR/$s"
    ABS_SCRIPTS+=("$ABS_PATH")
done


echo "[INFO] Starting all services..."


# ================================
# 遍历脚本列表
# ================================
for SCRIPT in "${ABS_SCRIPTS[@]}"
do

    echo
    echo "[INFO] Launching: $SCRIPT"

    if [[ ! -f "$SCRIPT" ]]; then
        echo "[ERROR] Script not found: $SCRIPT"
        exit 1
    fi

    if [[ ! -x "$SCRIPT" ]]; then
        echo "[WARN] Script not executable, adding permission"
        chmod +x "$SCRIPT"
    fi

    bash "$SCRIPT"

done


echo
echo "[INFO] All services started successfully."