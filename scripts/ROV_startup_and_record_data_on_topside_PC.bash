#!/usr/bin/env bash
set -euo pipefail

############################################################
# PATH SAFETY
############################################################
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$WS_DIR"

############################################################
# ROS TOPICS TO RECORD
############################################################
ROS_TOPICS_TO_RECORD=(
  /bluerov/imu
  /bluerov/dvl
  /bluerov/ahrs
  /bluerov/mag
  /bluerov/odom
  /bluerov/references
  /bluerov/u
  /sonar_data
)

############################################################
# REMOTE CONFIG (SBC)
############################################################
REMOTE_USER=khadas
REMOTE_IP=192.168.42.12
REMOTE="${REMOTE_USER}@${REMOTE_IP}"

# Paths on the SBC
REMOTE_ROS_SETUP="/opt/ros/jazzy/setup.bash"
REMOTE_HOME="/home/${REMOTE_USER}"
REMOTE_WS="${REMOTE_HOME}/data_collection_ws"
REMOTE_WS_SETUP="${REMOTE_WS}/install/setup.bash"

############################################################
# SSH BOOTSTRAP 
############################################################
# Ensure passwordless SSH exists.
# Blocks until the correct password is entered ONCE.
ssh_bootstrap() {
  if ssh -o BatchMode=yes "$REMOTE" true 2>/dev/null; then
    echo "[SSH] Key auth already configured"
    return
  fi

  echo "[SSH] No SSH key found for $REMOTE"
  echo "[SSH] You will be asked for the password UNTIL correct"

  while true; do
    ssh-copy-id "$REMOTE" && break
    echo "[SSH] Wrong password, try again"
  done

  echo "[SSH] SSH key installed successfully"
}

############################################################
# MONO REMOTE (ROV) TMUX EXECUTOR
############################################################
remote_execute() {
  # Custom TMUX terminal name passe down
  local TMUX_NAME="$1"
  shift
  # Custom command passed down
  local CMD="$*"

  # Configuring ROS environment
  local CMD_ROS="source ${REMOTE_ROS_SETUP}
                 export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
                 export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"

  # Configuring workspace environment
  local CMD_WS="cd ${REMOTE_WS}
                source ${REMOTE_WS_SETUP}"

  # Combine all
  local CMD_ALL="${CMD_ROS}
                 ${CMD_WS}
                 ${CMD}
                 exec bash"

  # Execute remote
  ssh "$REMOTE" "tmux new-session -d -s ${TMUX_NAME} '$CMD_ALL'"
}

############################################################
# MONO TOPSIDE (PC) TMUX EXECUTOR
############################################################
topside_execute() {
  # Custom TMUX terminal name passe down
  local TMUX_NAME="$1"
  shift
  # Custom command passed down
  local CMD="$*"

  # Configuring ROS environment
  local CMD_ROS="source ${REMOTE_ROS_SETUP}
                 export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
                 export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET"

  # Configuring workspace environment
  local CMD_WS="source '$WS_DIR/install/setup.bash'"

  # Combine all
  local CMD_ALL="${CMD_ROS}
                 ${CMD_WS}
                 ${CMD}
                 exec bash"

  # Execute on topside PC
  tmux new-session -d -s "${TMUX_NAME}" "$CMD_ALL"
}

############################################################
# GLOBAL KILL (REMOTE + TOPSIDE)
############################################################
cleanup() {
  echo "[CLEANUP] Stopping rosbag recording"

  # Gracefully stop rosbag so metadata.yaml is written
  bash -c "tmux send-keys -t data_recording C-c 2>/dev/null || true" || true
  sleep 2

  echo "[CLEANUP] Killing all ROS 2 + tmux"

  CMD_CLEAN_ALL="pkill tmux || true
                 pkill -f /opt/ros || true"

  # Remote
  ssh "$REMOTE" "bash -c '$CMD_CLEAN_ALL'" || true

  # Topside
  bash -c "$CMD_CLEAN_ALL" || true
}

############################################################
# REMOTE (ROV) START FUNCTIONS
############################################################
start_actuators() {
  remote_execute actuators "ros2 launch bluerov_actuator_driver actuator_driver.launch.py"
}

start_pid() {
  remote_execute pid "ros2 run bluerov2_pid bluerov2_pid_exe"
}

start_sonar() {
  # In order to communicate with sonar we must establish a connection with its subnet and add it to our IP routing
  local CMD_SONAR="
    sudo ip addr flush dev eth0
    sudo ip addr add 192.168.2.10/24 dev eth0
    sudo ip link set eth0 up
    ros2 run brov2_sonar brov2_sonar_exe
  "

  remote_execute sonar "$CMD_SONAR"
}

start_barometer() {
  remote_execute barometer "ros2 run brov2_barometer brov2_barometer_node"
}

start_dvl() {
  remote_execute dvl "./scripts/start_DVL.exp"
}

start_sentireader() {
  remote_execute sentireader "ros2 run sentireader_ros2 sentireader_ros2"
}

start_camera() {
  remote_execute camera "./scripts/start_ROV_cam_stream.bash"
}

start_teleop() {
  remote_execute teleop "ros2 run bluerov2_teleop bluerov2_teleop_exe"
}

############################################################
# TOPSIDE (PC) START FUNCTIONS
############################################################
start_joy() {
  topside_execute joy "ros2 run joy joy_node"
}

start_data_recording() {
  local ROS_DATA_BAG_DIR="$WS_DIR/data/$(date +%Y%m%d_%H%M%S)"

  topside_execute data_recording "ros2 bag record --disable-keyboard-controls --topics ${ROS_TOPICS_TO_RECORD[*]} -o ${ROS_DATA_BAG_DIR}"
}

############################################################
# LAUNCH SEQUENCE
############################################################
echo "[INFO] Launching ROV stack"

# Connect to remote (ROV)
ssh_bootstrap

# Actuators (ROV)
start_actuators
sleep 2

# Sensors (ROV)
start_dvl # ? NOTE: Must be here before sentireader as sentireader has DVL connection but we need to manually activate DVL
sleep 2
start_sentireader
start_sonar
start_camera
start_barometer

# Control (ROV)
start_teleop
start_pid

# Operator (Topside PC)
start_joy

# Data gathering
start_data_recording

# ? NOTE: This will only run when you press CTRL+C or an error occurs, otherwise it will not run
trap cleanup EXIT SIGINT SIGTERM

echo "[INFO] All running. Ctrl+C to stop."
while true
do
    sleep 1
done

