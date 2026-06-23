#!/usr/bin/env python3
"""world_to_map.py — generate an occupancy-grid map directly from the Gazebo world.

This is the robust ALTERNATIVE to running SLAM: instead of driving a car around
to discover obstacles, we read the known obstacle boxes out of the SDF world file
and rasterise them into a PGM + YAML occupancy grid that the planner nodes load
directly (same format as `nav2_map_server map_saver`).

  P5 PGM, value 0 = occupied (black), 255 = free (white), trinary mode.
  Bottom-up row order handled to match map_loader.hpp / nav2 convention.

Usage:
  python3 src/planner_gazebo_demo/scripts/world_to_map.py
  python3 .../world_to_map.py --world W.sdf --out maps/planner_map --res 0.05 \
      --xmin -8.4 --xmax 8.4 --ymin -7.4 --ymax 7.4 --border 0.0 --inflate 0.0
"""

import argparse
import os
import xml.etree.ElementTree as ET


def parse_boxes(sdf_path):
    """Return [(cx, cy, sx, sy), ...] for every <model> that has a <box>."""
    root = ET.parse(sdf_path).getroot()
    boxes = []
    for model in root.iter("model"):
        pose_el = model.find("pose")
        box_size = None
        for size_el in model.iter("size"):          # first box size under the model
            box_size = size_el.text.split()
            break
        if pose_el is None or box_size is None:
            continue
        p = [float(v) for v in pose_el.text.split()]
        s = [float(v) for v in box_size]
        boxes.append((p[0], p[1], s[0], s[1]))       # x, y, size_x, size_y
    return boxes


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(pkg, "worlds", "planner_world.sdf"))
    ap.add_argument("--out", default=os.path.join(pkg, "maps", "planner_map"),
                    help="output path prefix (writes <out>.pgm and <out>.yaml)")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--xmin", type=float, default=-8.4)
    ap.add_argument("--xmax", type=float, default=8.4)
    ap.add_argument("--ymin", type=float, default=-7.4)
    ap.add_argument("--ymax", type=float, default=7.4)
    ap.add_argument("--border", type=float, default=0.15,
                    help="thickness of an enclosing boundary wall (m); 0 = none")
    ap.add_argument("--inflate", type=float, default=0.0,
                    help="extra margin grown around each obstacle (m)")
    args = ap.parse_args()

    res = args.res
    W = int(round((args.xmax - args.xmin) / res))
    H = int(round((args.ymax - args.ymin) / res))
    FREE, OCC = 255, 0
    grid = bytearray([FREE]) * (W * H)              # grid[row_from_bottom * W + col]

    def set_occ(col, row):
        if 0 <= col < W and 0 <= row < H:
            grid[row * W + col] = OCC

    # Rasterise obstacle boxes (+ optional inflation).
    boxes = parse_boxes(args.world)
    for (cx, cy, sx, sy) in boxes:
        x0, x1 = cx - sx / 2 - args.inflate, cx + sx / 2 + args.inflate
        y0, y1 = cy - sy / 2 - args.inflate, cy + sy / 2 + args.inflate
        c0 = int((x0 - args.xmin) / res); c1 = int((x1 - args.xmin) / res)
        r0 = int((y0 - args.ymin) / res); r1 = int((y1 - args.ymin) / res)
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                set_occ(c, r)

    # Optional enclosing boundary wall.
    if args.border > 1e-6:
        bt = max(1, int(round(args.border / res)))
        for r in range(H):
            for c in range(W):
                if r < bt or r >= H - bt or c < bt or c >= W - bt:
                    grid[r * W + c] = OCC

    # Write PGM (P5). PGM row 0 = top = y_max, so emit rows bottom->top reversed.
    pgm = args.out + ".pgm"
    with open(pgm, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (W, H))
        for r in range(H - 1, -1, -1):
            f.write(bytes(grid[r * W:(r + 1) * W]))

    # Write YAML.
    yaml = args.out + ".yaml"
    img = os.path.basename(pgm)
    with open(yaml, "w") as f:
        f.write(
            "image: %s\nmode: trinary\nresolution: %g\n"
            "origin: [%g, %g, 0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.25\n"
            % (img, res, args.xmin, args.ymin))

    occ = sum(1 for b in grid if b == OCC)
    print("wrote %s (%dx%d @ %.3f m) and %s" % (pgm, W, H, res, yaml))
    print("obstacles=%d  occupied_cells=%d  free_cells=%d" % (len(boxes), occ, W * H - occ))
    print("origin=[%.2f, %.2f]  -> use in planner: map_yaml:=%s" % (args.xmin, args.ymin, yaml))


if __name__ == "__main__":
    main()
