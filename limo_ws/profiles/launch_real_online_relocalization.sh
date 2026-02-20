#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <reference_csv_path> [pose_topic]"
  exit 1
fi

REFERENCE_CSV="$1"
POSE_TOPIC="${2:-/tag_only_base_pose}"

cd /home/jpdark/Downloads/robot_ws
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

ros2 launch limo_online_relocalization online_relocalization.launch.py \
  reference_csv:="$REFERENCE_CSV" \
  frame_id:=map \
  pose_topic:="$POSE_TOPIC" \
  pose_msg_type:=pose_stamped \
  distance_mode:=2d \
  publish_rate:=10.0
