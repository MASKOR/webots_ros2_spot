#!/usr/bin/env python3
"""Load a map image (PGM/PPM) + YAML and publish it as a nav_msgs/OccupancyGrid.

A tiny stand-in for nav2's map_server for the maze world: it reads the occupancy
image produced by ``maze_generator.py`` and republishes it on a latched topic so
RViz / planners can pick it up without bringing up the whole nav2 stack.
"""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

OCC_OCCUPIED = 100
OCC_FREE = 0
OCC_UNKNOWN = -1


def _read_pgm(path):
    """Return (width, height, maxval, [pixel values]) for a binary/ascii PGM."""
    with open(path, "rb") as f:
        data = f.read()

    # Tokenise the header, skipping '#' comment lines.
    tokens = []
    i = 0
    n = len(data)
    while len(tokens) < 4:
        while i < n and data[i] in b" \t\r\n":
            i += 1
        if i < n and data[i:i + 1] == b"#":
            while i < n and data[i] not in b"\r\n":
                i += 1
            continue
        start = i
        while i < n and data[i] not in b" \t\r\n":
            i += 1
        tokens.append(data[start:i])

    magic, width, height, maxval = tokens
    width, height, maxval = int(width), int(height), int(maxval)

    # Exactly one whitespace byte separates the header from the payload.
    i += 1

    if magic == b"P5":
        pixels = list(data[i:i + width * height])
    elif magic == b"P2":
        pixels = [int(v) for v in data[i:].split()[:width * height]]
    else:
        raise ValueError(f"Unsupported PGM magic {magic!r} in {path}")

    if len(pixels) != width * height:
        raise ValueError(
            f"{path}: expected {width * height} pixels, got {len(pixels)}"
        )
    return width, height, maxval, pixels


class MappingServer(Node):
    def __init__(self):
        super().__init__("mapping_server")

        default_yaml = os.path.join(
            get_package_share_directory("webots_spot"), "map", "maze.yaml"
        )

        self.declare_parameter("yaml_filename", default_yaml)
        self.declare_parameter("topic", "map")
        self.declare_parameter("frame_id", "map")
        # 0.0 -> publish once (latched only). >0 -> also republish on a timer.
        self.declare_parameter("publish_period_sec", 0.0)

        yaml_filename = self.get_parameter("yaml_filename").value
        self.frame_id = self.get_parameter("frame_id").value
        period = self.get_parameter("publish_period_sec").value

        self.grid = self._load_grid(yaml_filename)

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        topic = self.get_parameter("topic").value
        self.pub = self.create_publisher(OccupancyGrid, topic, qos)

        self._publish()
        self.get_logger().info(
            f"Published {self.grid.info.width}x{self.grid.info.height} "
            f"OccupancyGrid on '{topic}' (frame '{self.frame_id}')"
        )

        if period and period > 0.0:
            self.create_timer(period, self._publish)

    def _load_grid(self, yaml_filename):
        with open(yaml_filename) as f:
            meta = yaml.safe_load(f)

        image_path = meta["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(
                os.path.dirname(os.path.abspath(yaml_filename)), image_path
            )

        resolution = float(meta["resolution"])
        origin = [float(v) for v in meta["origin"]]
        negate = int(meta.get("negate", 0))
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.196))
        mode = meta.get("mode", "trinary")

        width, height, maxval, pixels = _read_pgm(image_path)

        def to_cell(px):
            shade = px / maxval
            occ = shade if negate else (1.0 - shade)
            if occ > occupied_thresh:
                return OCC_OCCUPIED
            if occ < free_thresh:
                return OCC_FREE
            if mode == "scale":
                return round(occ * 100)
            return OCC_UNKNOWN  # trinary

        # Per-pixel occupancy, in image order (row 0 = top of the image).
        row_cells = [to_cell(px) for px in pixels]

        # OccupancyGrid data starts at cell (0, 0) = the map origin (bottom-left),
        # so flip the image rows top<->bottom.
        data = []
        for row in range(height - 1, -1, -1):
            data.extend(row_cells[row * width:(row + 1) * width])

        grid = OccupancyGrid()
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = origin[0]
        grid.info.origin.position.y = origin[1]
        grid.info.origin.position.z = 0.0
        yaw = origin[2] if len(origin) > 2 else 0.0
        grid.info.origin.orientation.z = math.sin(yaw / 2.0)
        grid.info.origin.orientation.w = math.cos(yaw / 2.0)
        grid.data = data
        return grid

    def _publish(self):
        now = self.get_clock().now().to_msg()
        self.grid.header.stamp = now
        self.grid.header.frame_id = self.frame_id
        self.grid.info.map_load_time = now
        self.pub.publish(self.grid)


def main(args=None):
    rclpy.init(args=args)
    node = MappingServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
