#!/usr/bin/env python3
"""Generate a deterministic occupancy map for the small Gazebo room.

The SLAM path should be used for the actual Gazebo -> SLAM demonstration. This
script provides a known-good fallback map for planner/closed_loop launches and
for environments where running Gazebo is not practical.
"""

import argparse
import os
import xml.etree.ElementTree as ET


def parse_boxes(world_path):
    root = ET.parse(world_path).getroot()
    boxes = []
    for model in root.iter("model"):
        pose_el = model.find("pose")
        size_el = None
        for candidate in model.iter("size"):
            size_el = candidate
            break
        if pose_el is None or size_el is None:
            continue
        pose = [float(v) for v in pose_el.text.split()]
        size = [float(v) for v in size_el.text.split()]
        boxes.append((pose[0], pose[1], size[0], size[1], model.get("name", "box")))
    return boxes


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    pkg = os.path.dirname(here)
    ap = argparse.ArgumentParser()
    ap.add_argument("--world", default=os.path.join(pkg, "worlds", "small_room.world"))
    ap.add_argument("--out", default=os.path.join(pkg, "maps", "small_map"))
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--xmin", type=float, default=-3.35)
    ap.add_argument("--xmax", type=float, default=3.35)
    ap.add_argument("--ymin", type=float, default=-3.35)
    ap.add_argument("--ymax", type=float, default=3.35)
    ap.add_argument("--inflate", type=float, default=0.02)
    args = ap.parse_args()

    width = int(round((args.xmax - args.xmin) / args.res))
    height = int(round((args.ymax - args.ymin) / args.res))
    free, occ = 255, 0
    grid = bytearray([free]) * (width * height)

    def set_occ(c, r):
        if 0 <= c < width and 0 <= r < height:
            grid[r * width + c] = occ

    boxes = parse_boxes(args.world)
    for cx, cy, sx, sy, _name in boxes:
        x0 = cx - sx / 2.0 - args.inflate
        x1 = cx + sx / 2.0 + args.inflate
        y0 = cy - sy / 2.0 - args.inflate
        y1 = cy + sy / 2.0 + args.inflate
        c0 = int((x0 - args.xmin) / args.res)
        c1 = int((x1 - args.xmin) / args.res)
        r0 = int((y0 - args.ymin) / args.res)
        r1 = int((y1 - args.ymin) / args.res)
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                set_occ(c, r)

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    pgm_path = args.out + ".pgm"
    with open(pgm_path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (width, height))
        for r in range(height - 1, -1, -1):
            f.write(bytes(grid[r * width:(r + 1) * width]))

    yaml_path = args.out + ".yaml"
    with open(yaml_path, "w") as f:
        f.write(
            "image: %s\nmode: trinary\nresolution: %g\n"
            "origin: [%g, %g, 0]\nnegate: 0\n"
            "occupied_thresh: 0.65\nfree_thresh: 0.25\n"
            % (os.path.basename(pgm_path), args.res, args.xmin, args.ymin))

    occupied = sum(1 for cell in grid if cell == occ)
    print("wrote %s and %s" % (pgm_path, yaml_path))
    print("boxes=%d occupied_cells=%d free_cells=%d" % (len(boxes), occupied, width * height - occupied))


if __name__ == "__main__":
    main()
