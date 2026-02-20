#!/usr/bin/env python3
import argparse
import csv
import math
from pathlib import Path
import yaml


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def rotate_xy(x, y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * x - s * y, s * x + c * y


def rotate_traj_csv(inp: Path, out: Path, yaw: float, tx: float, ty: float, tz: float):
    with inp.open('r', newline='') as f:
        rows = list(csv.DictReader(f))
        fieldnames = list(rows[0].keys()) if rows else []

    for r in rows:
        x = float(r['x'])
        y = float(r['y'])
        z = float(r.get('z', 0.0))
        xr, yr = rotate_xy(x, y, yaw)
        r['x'] = f"{xr + tx:.9f}"
        r['y'] = f"{yr + ty:.9f}"
        r['z'] = f"{z + tz:.9f}"

        if {'qx', 'qy', 'qz', 'qw'}.issubset(r.keys()):
            q = (float(r['qx']), float(r['qy']), float(r['qz']), float(r['qw']))
            qyaw = yaw_to_quat(yaw)
            qn = quat_mul(qyaw, q)
            r['qx'] = f"{qn[0]:.9f}"
            r['qy'] = f"{qn[1]:.9f}"
            r['qz'] = f"{qn[2]:.9f}"
            r['qw'] = f"{qn[3]:.9f}"

    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)


def rotate_tag_map_yaml(inp: Path, out: Path, yaw: float, tx: float, ty: float, tz: float):
    data = yaml.safe_load(inp.read_text())
    tags = data.get('tags', {})
    qyaw = yaw_to_quat(yaw)

    for tag, v in tags.items():
        p = v['position']
        x, y, z = float(p['x']), float(p['y']), float(p['z'])
        xr, yr = rotate_xy(x, y, yaw)
        v['position'] = {'x': xr + tx, 'y': yr + ty, 'z': z + tz}

        q = v['orientation']
        qv = (float(q['x']), float(q['y']), float(q['z']), float(q['w']))
        qn = quat_mul(qyaw, qv)
        v['orientation'] = {'x': qn[0], 'y': qn[1], 'z': qn[2], 'w': qn[3]}

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(data, sort_keys=False))


def main():
    ap = argparse.ArgumentParser(description='Apply same global SE(2)+z transform to trajectory CSV and tag map YAML.')
    ap.add_argument('--traj-in', required=True)
    ap.add_argument('--traj-out', required=True)
    ap.add_argument('--map-in', required=True)
    ap.add_argument('--map-out', required=True)
    ap.add_argument('--yaw-deg', type=float, default=0.0)
    ap.add_argument('--tx', type=float, default=0.0)
    ap.add_argument('--ty', type=float, default=0.0)
    ap.add_argument('--tz', type=float, default=0.0)
    args = ap.parse_args()

    yaw = math.radians(args.yaw_deg)
    rotate_traj_csv(Path(args.traj_in), Path(args.traj_out), yaw, args.tx, args.ty, args.tz)
    rotate_tag_map_yaml(Path(args.map_in), Path(args.map_out), yaw, args.tx, args.ty, args.tz)
    print(f'Wrote: {args.traj_out}')
    print(f'Wrote: {args.map_out}')


if __name__ == '__main__':
    main()
