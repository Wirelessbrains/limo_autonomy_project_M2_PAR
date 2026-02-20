# limo_apriltag_tools

ROS 2 package for AprilTag perception, camera-info handling, overlay visualization, and offline mapping/relocalization wrappers.

## Nodes

- `camera_info_publisher_node`
  - Publishes `sensor_msgs/msg/CameraInfo` from YAML calibration.

- `camera_info_relay_node`
  - Republishes camera info to a target topic.

- `udp_camera_receiver_node`
  - Receives JPEG frames via UDP and publishes image + camera info.

- `tag_overlay_node`
  - Draws detected tags over image output.

- `ippe_localizer`
  - Computes pose from tag detections and static tag map.

- `static_map_publisher`
  - Publishes static TF transforms for tags from YAML map.

## Launch

- `launch/apriltag_camera_pipeline.launch.py`

Quick run:

```bash
cd /home/jpdark/Downloads/robot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch limo_apriltag_tools apriltag_camera_pipeline.launch.py
```

## Offline tools

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py <bag_path> <output_dir>
python3 scripts/trajectory_analysis/analyze_distance_angle_to_trajectory.py <ref> <query> <output_dir>
```

Equivalent wrappers:

- `ros2 run limo_apriltag_tools build_tag_map_offline ...`
- `ros2 run limo_apriltag_tools analyze_distance_to_trajectory -- ...`

## Key config files

- `config/apriltag_params.yaml`
- `config/camera_info_default.yaml`
- `config/webcam_calibration.yaml`
- `config/webcam_calibration_robot.yaml`
- `config/tag_map_parking.yaml`
