"""A fixed curved corridor shared by Gazebo collision geometry and perception."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

# Sparse static cylinders around a radius-2 m, 0.5 rad arc. All metres.
OBSTACLES = tuple(
    ((2 + offset) * math.sin(a), 2 - (2 + offset) * math.cos(a), 0.08)
    for a in (0.18, 0.42)
    for offset in (-0.9, 0.9)
)
ROBOT_RADIUS = 0.5
COLLISION_MARGIN = 0.05


def corridor_world(family, destination):
    source = Path(__file__).resolve().parents[1] / "worlds" / f"empty_{family}.sdf"
    document = ET.parse(source)
    world = document.getroot().find("world")
    for index, (x, y, radius) in enumerate(OBSTACLES):
        model = ET.SubElement(world, "model", name=f"corridor_{index}")
        ET.SubElement(model, "static").text = "true"
        ET.SubElement(model, "pose").text = f"{x} {y} 0.4 0 0 0"
        link = ET.SubElement(model, "link", name="cylinder")
        for kind in ("collision", "visual"):
            item = ET.SubElement(link, kind, name=kind)
            cylinder = ET.SubElement(ET.SubElement(item, "geometry"), "cylinder")
            ET.SubElement(cylinder, "radius").text = str(radius)
            ET.SubElement(cylinder, "length").text = "0.8"
    destination = Path(destination).resolve()
    document.write(destination, encoding="utf-8", xml_declaration=True)
    return destination


def clearance(start, end):
    """Minimum swept center-chord clearance after inflating the circular body."""
    dx, dy = end[0] - start[0], end[1] - start[1]
    length2 = dx * dx + dy * dy
    result = math.inf
    for x, y, radius in OBSTACLES:
        t = (
            min(1, max(0, ((x - start[0]) * dx + (y - start[1]) * dy) / length2))
            if length2
            else 0
        )
        result = min(
            result,
            math.hypot(start[0] + t * dx - x, start[1] + t * dy - y)
            - radius
            - ROBOT_RADIUS
            - COLLISION_MARGIN,
        )
    return result
