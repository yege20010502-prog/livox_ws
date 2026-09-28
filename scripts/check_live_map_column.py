#!/usr/bin/env python3
"""Compare one map-frame XY column with the current FAST-LIO cloud."""

import argparse
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("--radius", type=float, default=0.1)
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("check_live_map_column")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples = []

    def on_cloud(msg):
        try:
            transform = buffer.lookup_transform("map", msg.header.frame_id, Time(), timeout=Duration(seconds=0.0))
        except TransformException:
            return
        q = transform.transform.rotation
        t = transform.transform.translation
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        r00 = 1 - 2 * (yy * yy + zz * zz)
        r01 = 2 * (xx * yy - zz * ww)
        r02 = 2 * (xx * zz + yy * ww)
        r10 = 2 * (xx * yy + zz * ww)
        r11 = 1 - 2 * (xx * xx + zz * zz)
        r12 = 2 * (yy * zz - xx * ww)
        r20 = 2 * (xx * zz - yy * ww)
        r21 = 2 * (yy * zz + xx * ww)
        r22 = 1 - 2 * (xx * xx + yy * yy)
        bins = {}
        total = 0
        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            mx = r00 * x + r01 * y + r02 * z + t.x
            my = r10 * x + r11 * y + r12 * z + t.y
            if abs(mx - args.x) > args.radius or abs(my - args.y) > args.radius:
                continue
            mz = r20 * x + r21 * y + r22 * z + t.z
            height_bin = math.floor(mz / 0.2 + 1e-9)
            bins[height_bin] = bins.get(height_bin, 0) + 1
            total += 1
        samples.append((total, bins))

    subscription = node.create_subscription(PointCloud2, "/fastlio2/world_cloud", on_cloud, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and time.monotonic() < deadline and len(samples) < 10:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(f"frames={len(samples)} center=({args.x:.3f},{args.y:.3f}) radius={args.radius:.3f}")
    merged = {}
    for _, bins in samples:
        for k, count in bins.items():
            merged[k] = merged.get(k, 0) + count
    print("points_per_z_bin_0.2m=", sorted(merged.items()))
    print("points_per_frame=", [count for count, _ in samples])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
