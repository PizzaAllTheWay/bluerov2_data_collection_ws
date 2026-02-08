#!/usr/bin/env bash
set -e

# Go to script dir, then workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS_DIR"

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash
source install/setup.bash

DATA_DIR="$WS_DIR/data"

# Check data directory exists and is not empty
if [ ! -d "$DATA_DIR" ] || [ -z "$(ls -A "$DATA_DIR")" ]; then
  echo "[ERROR] Data directory is empty or does not exist: $DATA_DIR"
  exit 1
fi

# If dataset specified, use it
if [ $# -eq 1 ]; then
  BAG_PATH="$DATA_DIR/$1"
  if [ ! -d "$BAG_PATH" ]; then
    echo "[ERROR] Specified dataset not found: $BAG_PATH"
    exit 1
  fi
else
  # Otherwise pick latest dataset
  BAG_PATH="$(ls -td "$DATA_DIR"/* | head -n 1)"
fi

echo "[INFO] Playing back dataset: $(basename "$BAG_PATH")"

ros2 bag play "$BAG_PATH"
