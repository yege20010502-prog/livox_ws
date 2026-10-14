#!/usr/bin/env python3
"""Capture navigation-zone polygon vertices from RViz Publish Point.

This tool only records and previews zones.  It does not modify the PCD or the
planner collision map until a separate, reviewed mask-fusion step is enabled.
"""

import argparse
import math
import os
from pathlib import Path
import tempfile
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import yaml


COLORS = {
    "keepout": (1.0, 0.08, 0.05),
    "passable": (0.05, 0.45, 1.0),
    "preferred": (1.0, 0.75, 0.05),
}


def polygon_area(points):
    return abs(sum(
        points[i][0] * points[(i + 1) % len(points)][1]
        - points[(i + 1) % len(points)][0] * points[i][1]
        for i in range(len(points))
    )) * 0.5


def load_zones(path):
    if not path.exists():
        return {"version": 1, "frame_id": "map", "zones": []}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if data.get("version") != 1 or not isinstance(data.get("zones", []), list):
        raise ValueError("Unsupported or invalid zone file")
    if data.get("frame_id", "map") != "map":
        raise ValueError("Zone file frame_id must be map")
    data.setdefault("frame_id", "map")
    data.setdefault("zones", [])
    return data


def save_zones(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    text = yaml.safe_dump(data, allow_unicode=True, sort_keys=False)
    fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=path.parent)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(text)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def make_markers(zones, active=None):
    array = MarkerArray()
    cleanup = Marker()
    cleanup.action = Marker.DELETEALL
    array.markers.append(cleanup)
    all_zones = list(zones)
    if active:
        all_zones.append(active)
    stamp = rclpy.clock.Clock().now().to_msg()
    marker_id = 0
    for zone in all_zones:
        points = zone.get("points", [])
        if not points:
            continue
        red, green, blue = COLORS[zone["type"]]
        line = Marker()
        line.header.frame_id = "map"
        line.header.stamp = stamp
        line.ns = "navigation_zones"
        line.id = marker_id
        marker_id += 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.06
        line.color.r, line.color.g, line.color.b, line.color.a = red, green, blue, 1.0
        line.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2]) + 0.05) for p in points]
        if len(points) >= 3:
            p = points[0]
            line.points.append(Point(x=float(p[0]), y=float(p[1]), z=float(p[2]) + 0.05))
        array.markers.append(line)
        for index, p in enumerate(points, 1):
            dot = Marker()
            dot.header = line.header
            dot.ns = "navigation_zone_vertices"
            dot.id = marker_id
            marker_id += 1
            dot.type = Marker.SPHERE
            dot.action = Marker.ADD
            dot.pose.position = Point(x=float(p[0]), y=float(p[1]), z=float(p[2]) + 0.07)
            dot.pose.orientation.w = 1.0
            dot.scale.x = dot.scale.y = dot.scale.z = 0.13
            dot.color.r, dot.color.g, dot.color.b, dot.color.a = red, green, blue, 1.0
            array.markers.append(dot)
    return array


class ZoneCapture(Node):
    def __init__(self, data, zone_type, name, vertex_count, topic):
        super().__init__("nav_zone_editor")
        self.data = data
        self.zone_type = zone_type
        self.name = name
        self.vertex_count = vertex_count
        self.points = []
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(MarkerArray, "/nav3d/navigation_zone_markers", qos)
        self.subscription = self.create_subscription(PointStamped, topic, self.on_point, 10)
        self.timer = self.create_timer(0.5, self.publish_preview)

    def on_point(self, message):
        if message.header.frame_id != "map":
            self.get_logger().error(
                f"Rejected point in frame '{message.header.frame_id}'; RViz Fixed Frame must be map")
            return
        p = [float(message.point.x), float(message.point.y), float(message.point.z)]
        if self.points and math.dist(p[:2], self.points[-1][:2]) < 0.05:
            self.get_logger().warning("Ignored point less than 5 cm from previous vertex")
            return
        self.points.append(p)
        print(f"vertex {len(self.points)}/{self.vertex_count}: x={p[0]:.3f} y={p[1]:.3f} z={p[2]:.3f}", flush=True)
        self.publish_preview()

    def publish_preview(self):
        active = {"name": self.name, "type": self.zone_type, "points": self.points}
        self.publisher.publish(make_markers(self.data["zones"], active))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", type=Path, help="navigation_zones.yaml")
    sub = parser.add_subparsers(dest="command", required=True)
    add = sub.add_parser("add", help="capture a polygon from RViz")
    add.add_argument("--type", choices=COLORS, required=True)
    add.add_argument("--name", required=True)
    add.add_argument("--vertices", type=int, required=True)
    add.add_argument("--topic", default="/nav3d/zone_point")
    sub.add_parser("list", help="list saved zones")
    delete = sub.add_parser("delete", help="delete one zone by name")
    delete.add_argument("--name", required=True)
    preview = sub.add_parser("preview", help="publish saved zones until Ctrl-C")
    preview.add_argument("--topic", default="/nav3d/navigation_zone_markers")
    return parser.parse_args()


def main():
    args = parse_args()
    data = load_zones(args.file)
    if args.command == "list":
        for zone in data["zones"]:
            print(f"{zone['name']}: type={zone['type']} vertices={len(zone['points'])}")
        return
    if args.command == "delete":
        kept = [z for z in data["zones"] if z.get("name") != args.name]
        if len(kept) == len(data["zones"]):
            raise SystemExit(f"Zone not found: {args.name}")
        data["zones"] = kept
        save_zones(args.file, data)
        print(f"Deleted zone '{args.name}'")
        return
    if args.command == "add":
        if args.vertices < 3:
            raise SystemExit("A polygon requires at least 3 vertices")
        if any(z.get("name") == args.name for z in data["zones"]):
            raise SystemExit(f"Zone name already exists: {args.name}")
        rclpy.init()
        node = ZoneCapture(data, args.type, args.name, args.vertices, args.topic)
        print(f"Waiting for {args.vertices} points on {args.topic}; RViz Fixed Frame must be map", flush=True)
        try:
            while rclpy.ok() and len(node.points) < args.vertices:
                rclpy.spin_once(node, timeout_sec=0.2)
            if len(node.points) != args.vertices:
                raise SystemExit("Capture interrupted; zone was not saved")
            area = polygon_area(node.points)
            if area < 0.02:
                raise SystemExit(f"Polygon area is too small ({area:.4f} m^2); zone was not saved")
            zone = {"name": args.name, "type": args.type, "points": node.points}
            data["zones"].append(zone)
            save_zones(args.file, data)
            node.publisher.publish(make_markers(data["zones"]))
            time.sleep(0.5)
            print(f"Saved {args.type} zone '{args.name}', area={area:.3f} m^2, file={args.file}")
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    rclpy.init()
    node = Node("nav_zone_preview")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(MarkerArray, args.topic, qos)
    timer = node.create_timer(0.5, lambda: publisher.publish(make_markers(data["zones"])))
    print(f"Previewing {len(data['zones'])} zones on {args.topic}; Ctrl-C to stop")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(timer)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
