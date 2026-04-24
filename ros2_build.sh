#!/bin/bash
set -e

# ================================
# 默认参数
# ================================
PARALLEL_WORKERS=2        # 同时编译几个 package
BUILD_JOBS=8              # 每个 package 使用多少核

# 新增：用于存放指定的 pkg
SELECTED_PKGS=()

# ================================
# 参数解析
# ================================
while [[ $# -gt 0 ]]; do
  case "$1" in
    --workers)
      PARALLEL_WORKERS="$2"
      shift 2
      ;;
    --jobs)
      BUILD_JOBS="$2"
      shift 2
      ;;
    *)
      # 修改：把未知参数当作 pkg 名
      SELECTED_PKGS+=("$1")
      shift
      ;;
  esac
done

echo "[INFO] parallel workers: $PARALLEL_WORKERS"
echo "[INFO] build jobs per package: $BUILD_JOBS"

# 如果指定了 pkg，打印一下
if [ ${#SELECTED_PKGS[@]} -gt 0 ]; then
  echo "[INFO] Selected packages: ${SELECTED_PKGS[*]}"
else
  echo "[INFO] Building all packages"
fi

# ================================
# 获取系统 Python
# ================================
SYSTEM_PYTHON=/usr/bin/python3

if [ -z "$SYSTEM_PYTHON" ]; then
  echo "[ERROR] Cannot find system Python. Please ensure Python is installed."
  exit 1
fi

echo "[INFO] Using Python: $SYSTEM_PYTHON"

# ================================
# 检查 ROS workspace
# ================================
CURRENT_DIR=$(pwd)

if [ ! -d "$CURRENT_DIR/src" ]; then
  echo "[ERROR] No 'src' directory found. Ensure you're in a ROS 2 workspace."
  exit 1
fi

echo "[INFO] Building ROS 2 workspace at: $CURRENT_DIR"

# ================================
# 构建
# ================================
colcon build \
  ${SELECTED_PKGS:+--packages-select "${SELECTED_PKGS[@]}"} \
  --parallel-workers $PARALLEL_WORKERS \
  --cmake-args \
    -DPython3_EXECUTABLE=$SYSTEM_PYTHON \
    -DCMAKE_BUILD_PARALLEL_LEVEL=$BUILD_JOBS

# ================================
# 完成提示
# ================================
echo "[INFO] Build complete."
echo "source $CURRENT_DIR/install/setup.bash"

