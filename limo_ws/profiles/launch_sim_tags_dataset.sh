#!/usr/bin/env bash
set -euo pipefail

cd /home/jpdark/Downloads/robot_ws
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# Starts parking world with many tags, but disables parking autonomy stack.
ros2 launch control_limo sim_unified.launch.py \
  world:=/home/jpdark/Downloads/robot_ws/src/gz_apriltag_env/worlds/walls_apriltag_limo.sdf \
  parking_mode:=false \
  run_ippe_localization:=false
