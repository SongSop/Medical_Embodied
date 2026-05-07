#!/bin/bash
set -e

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
cd "$SCRIPT_DIR"

NAV_DIR="$SCRIPT_DIR/src/nav"

if [ ! -d "$NAV_DIR" ]; then
  echo "[ERROR] No 'src/nav' directory found. Ensure you're in the ROS 2 workspace root."
  exit 1
fi

NAV_PKGS=()

while IFS= read -r package_xml; do
  pkg_name=$(sed -n 's:.*<name>\(.*\)</name>.*:\1:p' "$package_xml" | head -n 1)
  if [ -n "$pkg_name" ]; then
    NAV_PKGS+=("$pkg_name")
  fi
done < <(find "$NAV_DIR" -mindepth 2 -maxdepth 2 -name package.xml | sort)

if [ ${#NAV_PKGS[@]} -eq 0 ]; then
  echo "[ERROR] No ROS 2 packages found in src/nav."
  exit 1
fi

echo "[INFO] Building nav packages: ${NAV_PKGS[*]}"

PARALLEL_WORKERS=8
BUILD_JOBS=8

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
      echo "[ERROR] Unknown argument: $1"
      echo "Usage: ./nav_build.sh [--workers N] [--jobs N]"
      exit 1
      ;;
  esac
done

SYSTEM_PYTHON=/usr/bin/python3
MPI_C_COMPILER=$(command -v mpicc || true)
MPI_C_HEADER_DIR=/usr/lib/x86_64-linux-gnu/openmpi/include
MPI_C_LIBRARY=/usr/lib/x86_64-linux-gnu/libmpi.so

CMAKE_ARGS=(
  -DPython3_EXECUTABLE="$SYSTEM_PYTHON"
  -DCMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS"
  -DBUILD_TESTING=OFF
)

if [ -x "$MPI_C_COMPILER" ] && [ -d "$MPI_C_HEADER_DIR" ] && [ -f "$MPI_C_LIBRARY" ]; then
  echo "[INFO] Using MPI C compiler: $MPI_C_COMPILER"
  CMAKE_ARGS+=(
    -DMPI_C_COMPILER="$MPI_C_COMPILER"
    -DMPI_C_HEADER_DIR="$MPI_C_HEADER_DIR"
    -DMPI_C_LIB_NAMES=mpi
    -DMPI_mpi_LIBRARY="$MPI_C_LIBRARY"
  )
fi

echo "[INFO] parallel workers: $PARALLEL_WORKERS"
echo "[INFO] build jobs per package: $BUILD_JOBS"
echo "[INFO] Using Python: $SYSTEM_PYTHON"
echo "[INFO] Building ROS 2 workspace at: $SCRIPT_DIR"

colcon build \
  --packages-up-to "${NAV_PKGS[@]}" \
  --parallel-workers "$PARALLEL_WORKERS" \
  --cmake-args \
    "${CMAKE_ARGS[@]}"

echo "[INFO] Build complete."
echo "source $SCRIPT_DIR/install/setup.bash"
