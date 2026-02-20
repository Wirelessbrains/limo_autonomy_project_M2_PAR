#!/usr/bin/env bash
set -euo pipefail

cd /home/jpdark/Downloads/robot_ws
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}
set +u
source /opt/ros/humble/setup.bash
set -u

colcon build --symlink-install \
  --packages-select control_limo limo_apriltag_tools limo_online_relocalization gz_apriltag_env

set +u
source install/setup.bash
set -u
echo "SIM build completed."
