#!/usr/bin/env python3

"""Validate RMF navigation lanes against a ROS occupancy map."""

import argparse
import math
from pathlib import Path

import yaml


def read_pgm(path):
    """Read an 8-bit binary PGM occupancy image."""

    with path.open("rb") as file:

        def read_token():
            while True:
                char = file.read(1)

                if not char:
                    raise EOFError("Unexpected end of PGM header")

                if char == b"#":
                    file.readline()
                    continue

                if char.isspace():
                    continue

                token = bytearray(char)

                while True:
                    char = file.read(1)

                    if not char or char.isspace():
                        return token.decode()

                    token.extend(char)

        if read_token() != "P5":
            raise ValueError("Expected binary PGM (P5)")

        width = int(read_token())
        height = int(read_token())
        max_value = int(read_token())

        if max_value != 255:
            raise ValueError("Expected 8-bit PGM")

        pixels = file.read()

    if len(pixels) != width * height:
        raise ValueError("Invalid PGM image size")

    return width, height, pixels


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--map-yaml", type=Path, required=True)
    parser.add_argument("--graph", type=Path, required=True)
    parser.add_argument("--clearance", type=float, default=0.5)

    args = parser.parse_args()

    with args.map_yaml.open() as file:
        map_config = yaml.safe_load(file)

    with args.graph.open() as file:
        graph_config = yaml.safe_load(file)

    resolution = float(map_config["resolution"])
    origin_x, origin_y, origin_yaw = map_config["origin"]

    if resolution <= 0:
        raise ValueError("Invalid map resolution")

    if abs(origin_yaw) > 1e-9:
        raise ValueError("Rotated maps are not supported")

    radius = args.clearance

    if not math.isfinite(radius) or radius <= 0:
        raise ValueError("Invalid clearance radius")

    image_path = args.map_yaml.parent / map_config["image"]
    width, height, pixels = read_pgm(image_path)

    level = graph_config["levels"]["warehouse"]

    vertices = level["vertices"]
    lanes = level["lanes"]

    directed = {
        (int(lane[0]), int(lane[1]))
        for lane in lanes
    }

    if len(directed) != len(lanes):
        raise ValueError("Duplicate directed lanes")

    for start, end in directed:
        if (end, start) not in directed:
            raise ValueError(
                f"Missing reverse lane: {start} -> {end}"
            )

    physical_edges = sorted({
        tuple(sorted((start, end)))
        for start, end in directed
    })

    cells = math.ceil(radius / resolution)

    offsets = [
        (dx, dy)
        for dy in range(-cells, cells + 1)
        for dx in range(-cells, cells + 1)
        if math.hypot(dx, dy) * resolution <= radius
    ]

    def is_clear(x, y):
        col = math.floor((x - origin_x) / resolution)
        row = height - 1 - math.floor((y - origin_y) / resolution)

        for dx, dy in offsets:
            px = col + dx
            py = row + dy

            if not (0 <= px < width and 0 <= py < height):
                return False

            if pixels[py * width + px] < 230:
                return False

        return True

    print("===== RMF LANE CLEARANCE VALIDATION =====")
    print(f"Waypoints: {len(vertices)}")
    print(f"Directed lanes: {len(lanes)}")
    print(f"Physical connections: {len(physical_edges)}")
    print(f"Clearance radius: {radius:.2f} m")
    print()

    passed = 0

    for start, end in physical_edges:
        a = vertices[start]
        b = vertices[end]

        x1, y1 = float(a[0]), float(a[1])
        x2, y2 = float(b[0]), float(b[1])

        length = math.hypot(x2 - x1, y2 - y1)

        steps = max(1, math.ceil(length / resolution))

        blocked_at = None

        for step in range(steps + 1):
            t = step / steps

            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)

            if not is_clear(x, y):
                blocked_at = (x, y)
                break

        name_a = a[2].get("name", str(start))
        name_b = b[2].get("name", str(end))

        if blocked_at is None:
            print(
                f"{start:2d} <-> {end:2d} "
                f"{name_a} / {name_b}: CLEAR"
            )
            passed += 1
        else:
            print(
                f"{start:2d} <-> {end:2d} "
                f"{name_a} / {name_b}: BLOCKED "
                f"at ({blocked_at[0]:.2f}, {blocked_at[1]:.2f})"
            )

    print()
    print(f"Clear connections: {passed}/{len(physical_edges)}")

    if passed != len(physical_edges):
        raise SystemExit("LANE CLEARANCE VALIDATION: FAILED")

    print("LANE CLEARANCE VALIDATION: PASSED")


if __name__ == "__main__":
    main()