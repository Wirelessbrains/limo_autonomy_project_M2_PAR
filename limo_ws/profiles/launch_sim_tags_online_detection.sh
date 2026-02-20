#!/usr/bin/env bash
set -euo pipefail

if [ $# -lt 1 ]; then
  echo "Usage: $0 <map_yaml_path>"
  echo "Example:"
  echo "  bash $0 outputs/walls_tags_run_01_outputs/tag_map_walls.yaml"
  exit 1
fi

MAP_YAML="$1"

cd /home/jpdark/Downloads/robot_ws
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES-}
export AMENT_PYTHON_EXECUTABLE=${AMENT_PYTHON_EXECUTABLE-$(command -v python3)}
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

if [ ! -f "$MAP_YAML" ]; then
  echo "[error] map_yaml not found: $MAP_YAML"
  exit 1
fi

ros2 launch control_limo sim_unified.launch.py \
  world:=/home/jpdark/Downloads/robot_ws/src/gz_apriltag_env/worlds/walls_apriltag_limo.sdf \
  parking_mode:=false \
  run_ippe_localization:=true \
  map_yaml:="$MAP_YAML"
