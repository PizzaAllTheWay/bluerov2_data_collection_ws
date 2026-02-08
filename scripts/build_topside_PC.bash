#!/usr/bin/env bash
set -e

# Go to script dir, then workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Clean previous build artifacts (avoids stale CMake trash builds)
rm -rf build install log

# Build: suppress warnings, fail only on errors
# ? Note: For topside PC we don't need to build all teh packages  
colcon build \
  --packages-select \
  apos_interfaces \
  bluerov_interfaces \
  brov2_interfaces \
  --cmake-args -DCMAKE_CXX_FLAGS="-w" -DCMAKE_C_FLAGS="-w"

# Source workspace if build succeeded
source install/setup.bash

echo "Build OK, workspace sourced :)"





