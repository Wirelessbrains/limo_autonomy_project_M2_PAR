# limo_apriltag_navigation

![Project Avatar](https://www.generationrobots.com/19724-product_cover/robot-mobile-open-source-limo-compatible-ros1-et-ros2-limo-standard-version.jpg)

**Wireless Brains - AprilTag-Based Autonomous Navigation for the LIMO Robot**

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/index.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## Abstract

This project presents an autonomous navigation system for the AgileX LIMO robot, developed in ROS 2, which utilizes AprilTags as visual fiducial markers. The system implements a "Teach, Replay & Resume" methodology, enabling the robot to: (1) Learn a route via manual teleoperation; (2) Replay the route autonomously; and (3) Localize itself and resume the route from any point on the path. The entire development process is first validated in a Gazebo simulation environment before deployment on the physical hardware.

---

## Authors

* **Person 1:** [MARTINS DO LAGO REIS João Pedro] - (Simulation & ROS Architect) - [joao_pedro.martins_do_lago_reis@etu.uca.fr]
* **Person 2:** [DA SILVA RAMOS Yann Kelvem] - (Navigation Logic & State Machine Engineer) - [yann_kelvem.da_silva_ramos@etu.uca.fr]

---

---

## Project Documentation

All detailed project documentation has been moved to the `/docs` folder.

| Document | Description |
| :--- | :--- |
| **[Project Specifications](./docs/cahier_des_charges.md)** | The "Cahier de Charges": objectives and functional/non-functional requirements. |
| **[System Architecture](./docs/architecture.md)** | The core methodology (Teach, Replay, Resume) and software architecture (ROS Nodes). |
| **[Project Timeline](./docs/gantt.png)** | The Gantt chart detailing the project plan and task division. |
| **[Experiments & Results](./docs/experiments.md)** | Demonstration GIFs, videos, and validation of the system. |
| **[References](./docs/references.bib)** | All scientific and technical references in BibTeX format. |

---

## Code Structure

The repository is organized as follows:

├─ README.md             # This file
├─ docs/                 # All project documentation
│  ├─ specifications.md  # Project objectives and requirements (formerly Cahier de Charges)
│  ├─ gantt.png          # Project timeline and task allocation
│  ├─ architecture.md    # System methodology (Teach, Replay, Resume) and ROS node details
│  ├─ experiments.md     # Validation, simulation results, and demonstration GIFs
│  ├─ references.bib     # BibTeX file for all scientific references
│  └─ system_architecture.png # Screenshot of the ROS 2 node graph (rqt_graph)
│
├─ src/                  # ROS 2 Python packages (the "brain")
│  ├─ mapping/           # Stage 1: Teach-run node and waypoint saving logic
│  ├─ route_follow/      # Stage 2: Autonomous route following node (Nav2 client)
│  └─ relocalization/    # Stage 3: Localization state machine (Search, Localize, Resume)
│
├─ config/               # YAML parameter files
│  ├─ nav2_params.yaml   # Parameters for the Nav2 stack
│  └─ apriltag.yaml      # Configs for the apriltag_ros node (tag size, family)
│
├─ launch/               # ROS 2 launch files
│  ├─ sim_teach.launch.py   # Launches Gazebo + mapping node
│  └─ sim_nav.launch.py     # Launches Gazebo + relocalization node
│
├─ simulation/           # Simulation-specific assets
│  ├─ worlds/            # Gazebo world files (.world)
│  └─ rviz/              # RViz configuration files (.rviz)
│
├─ scripts/              # Utility scripts (not ROS nodes)
│  └─ waypoint_parser.py # Helper script to read/write waypoint files
│
└─ .github/              # GitHub-specific files
   └─ ISSUE_TEMPLATE/
      └─ task.md         # Template for creating new development tasks


