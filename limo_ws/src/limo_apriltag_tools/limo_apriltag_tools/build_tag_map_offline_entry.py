#!/usr/bin/env python3
import runpy
import sys
from pathlib import Path


def _find_workspace_script() -> Path:
    candidates = []

    cwd = Path.cwd()
    candidates.append(cwd / "scripts" / "trajectory_analysis" / "build_tag_map_offline.py")

    this_file = Path(__file__).resolve()
    # Typical path from source workspace:
    # <ws>/src/limo_apriltag_tools/limo_apriltag_tools/build_tag_map_offline_entry.py
    if len(this_file.parents) >= 4:
        ws_src = this_file.parents[3]
        candidates.append(ws_src / "scripts" / "trajectory_analysis" / "build_tag_map_offline.py")

    # Typical path from installed package:
    # <ws>/install/limo_apriltag_tools/lib/pythonX/site-packages/limo_apriltag_tools/...
    for parent in this_file.parents:
        candidates.append(parent / "scripts" / "trajectory_analysis" / "build_tag_map_offline.py")

    for p in candidates:
        if p.exists():
            return p
    raise FileNotFoundError("Could not find scripts/trajectory_analysis/build_tag_map_offline.py")


def main() -> None:
    script = _find_workspace_script()
    sys.argv = [str(script)] + sys.argv[1:]
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
