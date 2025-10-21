# 📊 Project Gantt Chart — LIMO AprilTag Autonomous Navigation

```mermaid
gantt
  title LIMO Project - AprilTag Autonomous Navigation
  dateFormat  YYYY-MM-DD
  axisFormat  %d/%m
  tickInterval 1week
  todayMarker off

  section M1 – Mapping (Teach)
  Environment setup + docs       :a1, 2025-10-21, 7d
  AprilTag detection & calibration :a2, after a1, 10d
  Teach-run (manual teleoperation) :a3, after a2, 8d
  Dataset recording & validation   :a4, after a3, 5d
  Deliverable M1 (docs + demo)     :milestone, after a4, 0d

  section M2 – Route Following (Replay)
  Waypoint generation & smoothing :b1, 2025-11-20, 10d
  Path-following controller (PID/twist) :b2, after b1, 10d
  Simulation tests (Gazebo)       :b3, after b2, 7d
  Deliverable M2 (autonomous replay validated) :milestone, after b3, 0d

  section M3 – Relocalization (Resume)
  Algorithm for nearest-tag search :c1, 2025-12-15, 8d
  Resume logic integration         :c2, after c1, 8d
  Simulation validation (random start points) :c3, after c2, 7d
  Deliverable M3 (resume phase validated) :milestone, after c3, 0d

  section M4 – Integration, Testing & Reporting
  Integration (mapping + replay + resume) :d1, 2026-01-10, 10d
  Experimentation & performance metrics   :d2, after d1, 14d
  Report + final documentation            :d3, after d2, 10d
  Video presentation + delivery           :d4, 2026-02-15, 5d
  Deliverable M4 (Final presentation)     :milestone, after d4, 0d
