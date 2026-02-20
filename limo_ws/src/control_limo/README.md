# control_limo

`control_limo` contains the parking behavior stack for the LIMO platform.

## Nodes

- `go_to_pose`
  - Low-level pose controller.
  - Consumes goal poses and publishes velocity commands.

- `parking_spot_navigator`
  - Selects parking targets.
  - Publishes staged parking goals.

- `parking_spot_randomizer_node`
  - Randomizes parking occupancy in simulation.

- `parking_spot_indicator_node`
  - Updates visual parking indicators from occupancy state.

## Main launch file

- `launch/tags_parking_full.launch.py`

This launch starts:

1. Ignition Gazebo parking world.
2. ROS-GZ bridges for pose, cmd_vel, camera, and contact sensors.
3. AprilTag perception pipeline from `limo_apriltag_tools`.
4. IPPE localization pipeline from `limo_apriltag_tools`.
5. Parking randomizer + controller + navigator + indicator nodes.

## Run full parking pipeline

```bash
cd /home/jpdark/Downloads/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch control_limo tags_parking_full.launch.py
```

## Useful launch arguments

```bash
ros2 launch control_limo tags_parking_full.launch.py \
  target_spot_index:=2 \
  align_to_corridor:=true \
  reach_tolerance:=0.20 \
  distance_tolerance:=0.20
```

Common arguments:

- `world`
- `map_yaml`
- `target_spot_index` (`-1` = automatic spot selection)
- `align_to_corridor`
- `require_final_orientation`
- `reach_tolerance`
- `distance_tolerance`
- `parking_entry_y_offset`
- `decision_wp_offset_03`
- `park_lock_distance`

## Build

```bash
cd /home/jpdark/Downloads/robot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select control_limo
```

## Quick troubleshooting

1. Confirm world and bridges are running.
2. Confirm `/detections` is active.
3. Confirm `/tag_only_base_pose` is active.
4. Confirm navigator is publishing goals.
5. Confirm controller is publishing to `/model/limo_01/cmd_vel`.
