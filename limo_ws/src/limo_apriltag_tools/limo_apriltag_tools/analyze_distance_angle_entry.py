#!/usr/bin/env python3
import runpy
import sys
from pathlib import Path


def _find_workspace_script() -> Path:
    candidates = []

    cwd = Path.cwd()
    candidates.append(cwd / "scripts" / "trajectory_analysis" / "analyze_distance_angle_to_trajectory.py")

    this_file = Path(__file__).resolve()
    if len(this_file.parents) >= 4:
        ws_src = this_file.parents[3]
        candidates.append(ws_src / "scripts" / "trajectory_analysis" / "analyze_distance_angle_to_trajectory.py")

    for parent in this_file.parents:
        candidates.append(parent / "scripts" / "trajectory_analysis" / "analyze_distance_angle_to_trajectory.py")

    for p in candidates:
        if p.exists():
            return p
    raise FileNotFoundError("Could not find scripts/trajectory_analysis/analyze_distance_angle_to_trajectory.py")


def main() -> None:
    script = _find_workspace_script()
    sys.argv = [str(script)] + sys.argv[1:]
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
