# Project Scope and Requirements (Cahier des Charges Style)

## 1) Mission Statement
Build a practical and reproducible system to:

- learn a trajectory from AprilTag observations,
- reconstruct map and trajectory references offline,
- relocalize a mobile robot with respect to the learned trajectory,
- provide clear, measurable outputs for practical robotics evaluation.

## 2) Client Requirement (Core)
Primary client requirement:
- **trajectory deviation from a learned path** for a mobile robot (*"déviation de trajectoire apprise"*).

This project specifically answers:
- "Where is the robot relative to the learned trajectory now?"
- "What are the distance and heading deviation to the nearest reference point?"

## 3) In-Scope Features

### 3.1 Offline pipeline
- Ingest rosbag recordings.
- Extract camera/tag observations.
- Build reference outputs:
  - trajectory CSV,
  - tag map CSV/YAML,
  - diagnostic and analysis outputs.

### 3.2 Online relocalization
- Subscribe to live pose input.
- Load reference trajectory.
- Compute and publish:
  - nearest-point distance,
  - heading error.
- Visualize in RViz.

### 3.3 Operational workflows
- Real robot workflow (primary).
- Simulation workflow (bonus) for repeatable validation.

## 4) Explicit Non-Goals (Out-of-Scope)
To avoid scope confusion, the following are not project targets in this phase:

- full autonomous global navigation,
- SLAM-based mapping of unknown large-scale environments,
- autonomous obstacle avoidance and path replanning,
- mission-level behavior trees and fleet orchestration,
- generalized "navigation in any world" claims.

## 5) Success Criteria
The project is considered successful when:

1. A reference trajectory can be learned and reconstructed from recorded data.
2. Online relocalization continuously reports nearest-point distance and heading deviation.
3. RViz clearly shows reference path, current pose, nearest pose, and error markers.
4. The workflow is reproducible through documented commands.

## 6) Deliverables
- Source code and launch/profile scripts.
- Offline analysis scripts and generated examples.
- Online relocalization node and RViz views.
- Documentation for execution and scope boundaries.

## 7) Boundary Statement
This repository is an applied practical project focused on trajectory reference and relocalization around AprilTag-based perception. It is **not** presented as a complete, generic, production-grade navigation platform.
