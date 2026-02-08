#!/usr/bin/env bash
set -e

# Go to script dir, then workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Clean previous build artifacts (avoid stale CMake + Cargo trash builds)
rm -rf build install log
rm -rf src/sentireader_ros2/target

# Build: suppress warnings, fail only on errors
colcon build \
  --cmake-args -DCMAKE_CXX_FLAGS="-w" -DCMAKE_C_FLAGS="-w"

# Source workspace if build succeeded
source install/setup.bash

echo "Build OK, workspace sourced :)"
