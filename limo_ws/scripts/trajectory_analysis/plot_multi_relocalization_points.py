#!/usr/bin/env python3
import argparse
import csv
import os
from pathlib import Path
from typing import List, Dict

import numpy as np
import pandas as pd
import plotly.graph_objects as go


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    x, y, z, w = qx, qy, qz, qw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def short_tag_label(tag_name: str) -> str:
    s = str(tag_name)
    return s.split(":")[-1] if ":" in s else s


def load_point_csv(path: str) -> Dict[str, float]:
    data: Dict[str, float] = {}
    with open(path, "r", newline="") as f:
        r = csv.reader(f)
        header = next(r, None)
        if header != ["metric", "value"]:
            raise RuntimeError(f"Invalid point CSV format: {path}")
        for k, v in r:
            try:
                data[k] = float(v)
            except Exception:
                pass
    required = [
        "distance_D_m",
        "heading_error_phi_deg",
        "robot_x_m",
        "robot_y_m",
        "robot_z_m",
        "reference_x_m",
        "reference_y_m",
        "reference_z_m",
        "robot_qx",
        "robot_qy",
        "robot_qz",
        "robot_qw",
        "reference_qx",
        "reference_qy",
        "reference_qz",
        "reference_qw",
    ]
    missing = [k for k in required if k not in data]
    if missing:
        raise RuntimeError(f"Missing keys in {path}: {missing}")
    return data


def main() -> None:
    p = argparse.ArgumentParser(description="Create one combined 3D map with multiple relocalization points.")
    p.add_argument("reference_output_dir", help="Reference output directory containing trajetoria_camera.csv and mapa_tags.csv")
    p.add_argument("compare_dirs", nargs="+", help="Compare directories containing point_pose_didactic.csv")
    p.add_argument("output_html", help="Output HTML path")
    p.add_argument("--tf-axis-len", type=float, default=0.10, help="Axis length for TF drawing (meters)")
    args = p.parse_args()

    ref_dir = Path(args.reference_output_dir)
    ref_csv = ref_dir / "trajetoria_camera.csv"
    tags_csv = ref_dir / "mapa_tags.csv"
    if not ref_csv.exists():
        raise RuntimeError(f"Missing reference trajectory: {ref_csv}")

    ref_df = pd.read_csv(ref_csv)
    ref_xyz = ref_df[["x", "y", "z"]].to_numpy(dtype=float)

    tags_df = None
    if tags_csv.exists():
        tags_df = pd.read_csv(tags_csv)

    points = []
    for d in args.compare_dirs:
        cdir = Path(d)
        p_csv = cdir / "point_pose_didactic.csv"
        if not p_csv.exists():
            raise RuntimeError(f"Missing point_pose_didactic.csv in {cdir}")
        vals = load_point_csv(str(p_csv))
        points.append((cdir.name, vals))

    fig = go.Figure()

    fig.add_trace(
        go.Scatter3d(
            x=ref_xyz[:, 0], y=ref_xyz[:, 1], z=ref_xyz[:, 2],
            mode="lines", line=dict(color="black", width=6), name="reference trajectory"
        )
    )

    if tags_df is not None and len(tags_df) > 0:
        tag_labels = [short_tag_label(t) for t in tags_df["tag"].astype(str).tolist()]
        fig.add_trace(
            go.Scatter3d(
                x=tags_df["x"].to_numpy(dtype=float),
                y=tags_df["y"].to_numpy(dtype=float),
                z=tags_df["z"].to_numpy(dtype=float),
                mode="markers+text",
                marker=dict(size=4, color="orange"),
                text=tag_labels,
                textposition="top center",
                textfont=dict(size=10, color="rgb(60,80,120)"),
                name="map tags",
            )
        )

    palette = [
        "royalblue", "crimson", "seagreen", "darkorange", "purple", "teal", "brown", "magenta"
    ]

    summary_rows = []
    for i, (name, v) in enumerate(points):
        color = palette[i % len(palette)]
        rx, ry, rz = v["robot_x_m"], v["robot_y_m"], v["robot_z_m"]
        tx, ty, tz = v["reference_x_m"], v["reference_y_m"], v["reference_z_m"]
        d = v["distance_D_m"]
        phi = v["heading_error_phi_deg"]

        fig.add_trace(
            go.Scatter3d(
                x=[rx], y=[ry], z=[rz], mode="markers+text",
                marker=dict(size=6, color=color), text=[name], textposition="top center",
                name=f"robot: {name}",
                hovertemplate=f"{name}<br>D={d:.3f} m<br>phi={phi:.2f} deg<extra></extra>",
            )
        )
        r_robot = quat_to_rotmat(v["robot_qx"], v["robot_qy"], v["robot_qz"], v["robot_qw"])
        r_ref = quat_to_rotmat(v["reference_qx"], v["reference_qy"], v["reference_qz"], v["reference_qw"])
        tf_colors = [("X", "red", 0), ("Y", "green", 1), ("Z", "blue", 2)]
        for axis_name, color_axis, axis_idx in tf_colors:
            # robot TF
            end_r = np.array([rx, ry, rz], dtype=float) + float(args.tf_axis_len) * r_robot[:, axis_idx]
            fig.add_trace(
                go.Scatter3d(
                    x=[rx, float(end_r[0])],
                    y=[ry, float(end_r[1])],
                    z=[rz, float(end_r[2])],
                    mode="lines",
                    line=dict(color=color_axis, width=6),
                    name=f"robot TF {axis_name}",
                    showlegend=(i == 0),
                )
            )
            # nearest reference TF
            end_t = np.array([tx, ty, tz], dtype=float) + float(args.tf_axis_len) * r_ref[:, axis_idx]
            fig.add_trace(
                go.Scatter3d(
                    x=[tx, float(end_t[0])],
                    y=[ty, float(end_t[1])],
                    z=[tz, float(end_t[2])],
                    mode="lines",
                    line=dict(color=color_axis, width=4, dash="dot"),
                    name=f"nearest ref TF {axis_name}",
                    showlegend=(i == 0),
                )
            )
        fig.add_trace(
            go.Scatter3d(
                x=[tx], y=[ty], z=[tz], mode="markers",
                marker=dict(size=5, color=color, symbol="diamond"),
                name=f"nearest ref: {name}",
                showlegend=False,
            )
        )
        fig.add_trace(
            go.Scatter3d(
                x=[rx, tx], y=[ry, ty], z=[rz, tz], mode="lines",
                line=dict(color=color, width=5, dash="dash"),
                name=f"distance link: {name}",
                showlegend=False,
            )
        )

        summary_rows.append((name, f"{d:.4f}", f"{phi:.2f}"))

    fig.add_trace(
        go.Table(
            header=dict(values=["Case", "D [m]", "phi [deg]"], fill_color="#f6f8fa", align="left"),
            cells=dict(values=[[r[0] for r in summary_rows], [r[1] for r in summary_rows], [r[2] for r in summary_rows]], align="left"),
            domain=dict(x=[0.0, 0.30], y=[0.02, 0.90]),
        )
    )

    fig.update_layout(
        title="All Relocalization Points on One Reference Map",
        scene=dict(
            xaxis_title="X [m]",
            yaxis_title="Y [m]",
            zaxis_title="Z [m]",
            aspectmode="data",
            domain=dict(x=[0.32, 0.995], y=[0.02, 0.98]),
            camera=dict(eye=dict(x=1.45, y=1.25, z=0.95)),
        ),
        annotations=[
            dict(
                x=0.01, y=0.98, xref="paper", yref="paper",
                text="<b>Combined Relocalization View</b><br>Each colored point = one relocalization case.<br>Dashed segment = D to nearest point on reference trajectory.",
                showarrow=False, align="left", bgcolor="rgba(255,255,255,0.85)", bordercolor="rgba(0,0,0,0.2)", borderwidth=1,
            )
        ],
        height=820,
        margin=dict(l=30, r=20, t=52, b=20),
        legend=dict(orientation="v", yanchor="top", y=0.98, xanchor="right", x=0.995, bgcolor="rgba(255,255,255,0.7)")
    )

    out = Path(args.output_html)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(str(out), include_plotlyjs="cdn")
    print(f"Combined map HTML: {out}")


if __name__ == "__main__":
    main()
