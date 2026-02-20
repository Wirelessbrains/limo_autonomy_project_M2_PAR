#!/usr/bin/env bash
set -euo pipefail

cd /home/jpdark/Downloads/robot_ws
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 launch control_limo real_nav_minimal.launch.py
