# Record, Map, and Relocalize with AprilTags

![Project Avatar](https://www.generationrobots.com/19724-product_cover/robot-mobile-open-source-limo-compatible-ros1-et-ros2-limo-standard-version.jpg)

**Primary track:** real LIMO robot pipeline (offline + online relocalization).  
**Bonus track:** simulation workflow for validation.

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)

---

## Authors

| Name | Role | Contact |
|------|------|----------|
| **MARTINS DO LAGO REIS João Pedro** | Real Robot Pipeline, Relocalization, Evaluation | joao_pedro.martins_do_lago_reis@etu.uca.fr |
| **DA SILVA RAMOS Yann Kelvem** | Simulation, ROS Architecture, Integration | yann_kelvem.da_silva_ramos@etu.uca.fr |

---

## Project Tracks

1. **Primary (Real Robot)**
- Record trajectory data with AprilTag detections.
- Build reference trajectory and tag map offline.
- Run offline relocalization analysis.
- Run online relocalization in RViz.

2. **Bonus (Simulation)**
- Reproduce the same dataset flow in `walls_apriltag_limo.sdf`.
- Record rosbag using joystick teleoperation.
- Validate online relocalization before real-world runs.

---

## Build

### Real profile

```bash
cd /home/jpdark/Downloads/robot_ws
bash profiles/build_real.sh
```

### Simulation profile

```bash
cd /home/jpdark/Downloads/robot_ws
bash profiles/build_sim.sh
```

---

## Primary Workflow (Real Robot)

### 1) Start perception/localization

```bash
bash profiles/launch_real_perception.sh
```

### 2) Record a rosbag (new terminal)

```bash
source /opt/ros/humble/setup.bash
source /home/jpdark/Downloads/robot_ws/install/setup.bash
ros2 bag record \
  /camera_info \
  /detections \
  /tag_only_base_pose \
  -o dataset/real_run_01
```

### 3) Build map + trajectory offline

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/real_run_01 \
  outputs/real_run_01_outputs
```

Main outputs:
- `outputs/real_run_01_outputs/trajetoria_camera.csv`
- `outputs/real_run_01_outputs/mapa_tags.csv`
- `outputs/real_run_01_outputs/tag_map.yaml`
- `outputs/real_run_01_outputs/mapa_trajetoria_interativo.html`

### 4) Offline relocalization analysis (optional)

```bash
python3 scripts/trajectory_analysis/analyze_distance_angle_to_trajectory.py \
  outputs/real_run_01_outputs \
  outputs/real_run_01_outputs \
  outputs/real_run_01_compare \
  --mode point --viz-mode debug
```

### 5) Online relocalization

```bash
bash profiles/launch_real_online_relocalization.sh \
  outputs/real_run_01_outputs/trajetoria_camera.csv \
  /tag_only_base_pose
```

Online topics:
- `/relocalization/reference_path`
- `/relocalization/current_pose`
- `/relocalization/nearest_pose`
- `/relocalization/markers`
- `/relocalization/distance_m`
- `/relocalization/heading_error_deg`

---

## Bonus Workflow (Simulation Dataset + Online Relocalization)

### 1) Launch walls scenario

```bash
bash profiles/launch_sim_tags_dataset.sh
```

### 2) Start joystick teleop (new terminal)

```bash
bash profiles/launch_teleop_joy_sim.sh
```

### 3) Record compact dataset (new terminal)

```bash
bash profiles/record_sim_dataset.sh walls_tags_run_01 minimal
```

### 4) Build map + trajectory offline

```bash
python3 scripts/trajectory_analysis/build_tag_map_offline.py \
  dataset/walls_tags_run_01 \
  outputs/walls_tags_run_01_outputs
```

### 5) Optional frame transform (trajectory + YAML together)

```bash
python3 profiles/rotate_reference_frame.py \
  --traj-in outputs/walls_tags_run_01_outputs/trajetoria_camera.csv \
  --traj-out outputs/walls_tags_run_01_outputs/trajetoria_camera_xz.csv \
  --map-in outputs/walls_tags_run_01_outputs/tag_map.yaml \
  --map-out outputs/walls_tags_run_01_outputs/tag_map_walls_xz.yaml \
  --yaw-deg 0 --tx 0 --ty 0 --tz 0
```

### 6) Relaunch simulation with selected map YAML

```bash
bash profiles/launch_sim_tags_online_detection.sh \
  outputs/walls_tags_run_01_outputs/tag_map_walls_xz.yaml
```

### 7) Start online relocalization + RViz

```bash
bash profiles/launch_sim_online_relocalization.sh \
  outputs/walls_tags_run_01_outputs/trajetoria_camera_xz.csv \
  /tag_only_pose pose_stamped map xz
```

---

## Offline vs Online Relocalization

- **Offline:** batch metrics and plots from recorded data.
- **Online:** live nearest-point distance and heading error from current pose to reference trajectory.

Both modes use `trajetoria_camera*.csv` as the reference trajectory source.

---

## Workspace Layout

```bash
robot_ws/
├── dataset/
├── outputs/
├── profiles/
├── scripts/
│   └── trajectory_analysis/
├── src/
│   ├── control_limo/
│   ├── limo_apriltag_tools/
│   ├── limo_online_relocalization/
│   └── gz_apriltag_env/
└── README.md
```
