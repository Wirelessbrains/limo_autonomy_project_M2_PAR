#!/usr/bin/env bash
set -euo pipefail

cd /home/jpdark/Downloads/robot_ws
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 launch limo_apriltag_tools real_perception_localization.launch.py
