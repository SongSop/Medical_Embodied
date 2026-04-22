#!/bin/bash
set -e

# ================================
# 默认参数
# ================================
PARALLEL_WORKERS=2        # 同时编译几个 package
BUILD_JOBS=8              # 每个 package 使用多少核

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
      echo "[ERROR] Unknown parameter: $1"
      echo "Usage: $0 [--workers N] [--jobs N]"
      exit 1
      ;;
  esac
done

echo "[INFO] parallel workers: $PARALLEL_WORKERS"
echo "[INFO] build jobs per package: $BUILD_JOBS"

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
# fuck ros2
colcon build \
  --parallel-workers $PARALLEL_WORKERS \
  --cmake-args \
    -DPython3_EXECUTABLE=$SYSTEM_PYTHON \
    -DCMAKE_BUILD_PARALLEL_LEVEL=$BUILD_JOBS

# ================================
# 完成提示
# ================================
echo "[INFO] Build complete."
echo "source $CURRENT_DIR/install/setup.bash"

