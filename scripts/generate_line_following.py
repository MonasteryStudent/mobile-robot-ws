#!/usr/bin/env python3

from copy import deepcopy
import math
from pathlib import Path
import xml.etree.ElementTree as ET

WORLD_DIR = (
    Path(__file__).resolve().parents[1]
    / "src" / "mobile_robot_bringup" / "worlds"
)


def build_arc(radius, total_angle_deg, segment_count, overlap):
    """Return one (x, y, yaw, length) tuple per arc segment.

    x and y are the midpoint of the strip, in meters.
    yaw is the direction from the start point to the end point, in radians.
    length is the chord length plus the overlap, in meters.
    """
    segment_angle = math.radians(total_angle_deg / segment_count)
    length = 2 * radius * math.sin(segment_angle / 2) + overlap

    segments = []

    for i in range(segment_count):
        start_angle = i * segment_angle
        end_angle = (i + 1) * segment_angle
        
        start_x = radius * math.sin(start_angle)
        end_x = radius * math.sin(end_angle)
        start_y = radius * (1 - math.cos(start_angle))
        end_y = radius * (1 - math.cos(end_angle))

        x = (start_x + end_x) / 2
        y = (start_y + end_y) / 2

        yaw = (start_angle + end_angle) / 2

        segments.append((x, y, yaw, length))

    return segments


def main():
    source = WORLD_DIR / "line_following.sdf"
    target = WORLD_DIR / "line_following_oval_tight.sdf"

    tree = ET.parse(source)
    world = tree.getroot().find("world")
    straight = world.find("model[@name='straight_line']")
    plane_size = "link/visual/geometry/plane/size"

    # Keep the visual and material as a template for the arc strips.
    template = deepcopy(straight)

    radius = 1.0
    straight_length = 3.0
    line_width = 0.08

    # The straight section runs from x = -straight_length to x = 0.
    straight.find("pose").text = f"{-straight_length / 2:.6f} 0 0.001 0 0 0"
    straight.find(plane_size).text = f"{straight_length:.6f} {line_width:.6f}"

    first_arc = build_arc(
        radius=radius,
        total_angle_deg=180.0,
        segment_count=60,
        overlap=0.004,
    )

    # Each entry contains: name, x, y, yaw, length.
    sections = [
        (f"arc_right_{i:02d}", x, y, yaw, length)
        for i, (x, y, yaw, length) in enumerate(first_arc)
    ]

    sections.append(
        ("return_straight", -straight_length / 2, 2 * radius, 0.0, straight_length)
    )

    for i, (x, y, yaw, length) in enumerate(first_arc):
        sections.append((
            f"arc_left_{i:02d}",
            -straight_length - x,
            2 * radius - y,
            yaw - math.pi,
            length,
        ))

    for name, x, y, yaw, length in sections:
        model = deepcopy(template)
        model.set("name", name)
        model.find("pose").text = f"{x:.6f} {y:.6f} 0.001 0 0 {yaw:.6f}"
        model.find(plane_size).text = f"{length:.6f} {line_width:.6f}"
        world.append(model)

    ET.indent(tree, space="  ")
    with target.open("xb") as output:
        tree.write(output, encoding="utf-8", xml_declaration=True)
    print(f"Created {target}")


if __name__ == "__main__":
    main()
