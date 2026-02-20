#!/usr/bin/env python3
import csv
import sys
from pathlib import Path

if len(sys.argv) < 3:
    print("Usage: generate_tag_map_yaml_from_csv.py <mapa_tags.csv> <output.yaml>")
    sys.exit(1)

csv_path = Path(sys.argv[1])
out_path = Path(sys.argv[2])

if not csv_path.exists():
    raise FileNotFoundError(f"CSV not found: {csv_path}")

rows = []
with csv_path.open('r', newline='') as f:
    reader = csv.DictReader(f)
    for r in reader:
        rows.append(r)

if not rows:
    raise RuntimeError("Empty mapa_tags.csv")

anchor = rows[0]['tag']
out_path.parent.mkdir(parents=True, exist_ok=True)
with out_path.open('w', encoding='utf-8') as f:
    f.write('world_frame: "map"\n')
    f.write(f'anchor_tag: "{anchor}"\n')
    f.write('tags:\n')
    for r in rows:
        tag = r['tag']
        x = float(r['x'])
        y = float(r['y'])
        z = float(r['z'])
        qx = float(r['qx'])
        qy = float(r['qy'])
        qz = float(r['qz'])
        qw = float(r['qw'])
        f.write(f'  {tag}:\n')
        f.write(f'    position: {{x: {x:.6f}, y: {y:.6f}, z: {z:.6f}}}\n')
        f.write(f'    orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}\n')

print(f"Wrote: {out_path}")
