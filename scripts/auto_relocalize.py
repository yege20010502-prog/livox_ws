#!/usr/bin/env python3
"""Estimate map->odom from a live FAST-LIO scan, then optionally relocalize.

The program deliberately fails closed when several map locations fit similarly.
Run without --execute first to inspect the estimated transform.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

import rclpy
from interface.srv import IsValid, Relocalize
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


def voxel_downsample(points, resolution):
    if len(points) == 0:
        return points
    cells = np.floor(points / resolution).astype(np.int32)
    _, indexes = np.unique(cells, axis=0, return_index=True)
    return points[np.sort(indexes)]


def read_pcd_xyz(path):
    with open(path, "rb") as stream:
        header = {}
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header is incomplete")
            key, _, value = line.decode("ascii").strip().partition(" ")
            header[key] = value.split()
            if key == "DATA":
                break
        if header["DATA"] != ["binary"]:
            raise ValueError("Only binary PCD maps are supported")
        fields = header["FIELDS"]
        sizes = list(map(int, header["SIZE"]))
        kinds = header["TYPE"]
        counts = list(map(int, header["COUNT"]))
        if any(count != 1 for count in counts):
            raise ValueError("PCD fields with COUNT > 1 are unsupported")
        formats = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "u1",
                   ("U", 2): "<u2", ("U", 4): "<u4", ("I", 4): "<i4"}
        dtype = np.dtype([(name, formats[(kind, size)])
                          for name, kind, size in zip(fields, kinds, sizes)])
        data = np.frombuffer(stream.read(), dtype=dtype)
        if len(data) != int(header["POINTS"][0]):
            raise ValueError("PCD point count does not match header")
        return np.column_stack((data["x"], data["y"], data["z"]))


def rotate_xy(points, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((c * points[:, 0] - s * points[:, 1],
                            s * points[:, 0] + c * points[:, 1]))


def coarse_candidates(map_xy, scan_xy, cell_size=0.25, step=0.5):
    lower = map_xy.min(axis=0) - 1.0
    upper = map_xy.max(axis=0) + 1.0
    shape = np.ceil((upper - lower) / cell_size).astype(int) + 1
    occupied = np.zeros(tuple(shape), dtype=bool)
    cells = np.floor((map_xy - lower) / cell_size).astype(int)
    occupied[cells[:, 0], cells[:, 1]] = True
    distance = distance_transform_edt(~occupied) * cell_size

    xs = np.arange(map_xy[:, 0].min(), map_xy[:, 0].max() + step, step)
    ys = np.arange(map_xy[:, 1].min(), map_xy[:, 1].max() + step, step)
    tx, ty = np.meshgrid(xs, ys, indexing="ij")
    translations = np.column_stack((tx.ravel(), ty.ravel()))
    best = []
    for yaw in np.deg2rad(np.arange(-180, 180, 10)):
        rotated = rotate_xy(scan_xy, yaw)
        for begin in range(0, len(translations), 256):
            batch = translations[begin:begin + 256]
            indices = np.floor((rotated[None, :, :] + batch[:, None, :] - lower) /
                               cell_size).astype(int)
            inside = ((indices[:, :, 0] >= 0) & (indices[:, :, 0] < shape[0]) &
                      (indices[:, :, 1] >= 0) & (indices[:, :, 1] < shape[1]))
            ix = np.clip(indices[:, :, 0], 0, shape[0] - 1)
            iy = np.clip(indices[:, :, 1], 0, shape[1] - 1)
            residuals = np.where(inside, distance[ix, iy], 1.0)
            scores = np.mean(np.minimum(residuals, 1.0), axis=1)
            for index in np.argsort(scores)[:3]:
                best.append((float(scores[index]), batch[index, 0], batch[index, 1], yaw))
    best.sort(key=lambda item: item[0])
    distinct = []
    for candidate in best:
        if all(math.hypot(candidate[1] - other[1], candidate[2] - other[2]) > 0.7 or
               abs(math.remainder(candidate[3] - other[3], 2 * math.pi)) > 0.25
               for other in distinct):
            distinct.append(candidate)
        if len(distinct) >= 15:
            break
    return distinct


def refine(map_tree, scan_xyz, candidate):
    _, x, y, yaw = candidate
    for _ in range(20):
        transformed = scan_xyz.copy()
        transformed[:, :2] = rotate_xy(scan_xyz[:, :2], yaw) + (x, y)
        distances, indices = map_tree.query(transformed, distance_upper_bound=0.6)
        good = np.isfinite(distances)
        if np.count_nonzero(good) < 30:
            break
        source = scan_xyz[good, :2]
        target = map_tree.data[indices[good], :2]
        source_center = source.mean(axis=0)
        target_center = target.mean(axis=0)
        u, _, vt = np.linalg.svd((source - source_center).T @
                                 (target - target_center))
        correction = np.eye(2)
        correction[1, 1] = np.linalg.det(u @ vt)
        rotation = u @ correction @ vt
        new_yaw = math.atan2(rotation[0, 1], rotation[0, 0])
        # Row-vector convention: source @ rotation = target.
        new_translation = target_center - source_center @ rotation
        if np.linalg.norm(new_translation - (x, y)) < 0.002 and abs(
                math.remainder(new_yaw - yaw, 2 * math.pi)) < 0.002:
            x, y, yaw = *new_translation, new_yaw
            break
        x, y, yaw = *new_translation, new_yaw
    transformed = scan_xyz.copy()
    transformed[:, :2] = rotate_xy(scan_xyz[:, :2], yaw) + (x, y)
    distances, _ = map_tree.query(transformed)
    return (float(np.mean(np.minimum(distances, 1.0))),
            float(np.mean(distances < 0.30)), float(np.median(distances)),
            float(x), float(y), float(yaw))


class ScanCollector(Node):
    def __init__(self, duration, radius):
        super().__init__("auto_relocalize_scan_collector")
        self.duration = duration
        self.radius = radius
        self.scans = []
        self.last_stamp = None
        self.odom_stamp = None
        self.odom_frame = None
        self.robot_xy = None
        self.create_subscription(PointCloud2, "/fastlio2/world_cloud", self.cloud_cb, 10)
        self.create_subscription(Odometry, "/fastlio2/lio_odom", self.odom_cb, 10)

    def odom_cb(self, msg):
        self.odom_stamp = msg.header.stamp
        self.odom_frame = msg.header.frame_id
        self.robot_xy = np.array((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def cloud_cb(self, msg):
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if msg.header.frame_id != "odom" or stamp == self.last_stamp:
            return
        self.last_stamp = stamp
        points = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"),
                                                 skip_nans=True)
        if len(points) and self.robot_xy is not None:
            self.scans.append(points[
                np.linalg.norm(points[:, :2] - self.robot_xy, axis=1) < self.radius])

    def collect(self):
        deadline = time.monotonic() + self.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if len(self.scans) < 5 or self.odom_stamp is None or self.odom_frame != "odom":
            raise RuntimeError("Need >=5 fresh world_cloud scans and odom in frame 'odom'")
        return np.concatenate(self.scans)


def call_service(node, client, request, timeout):
    if not client.wait_for_service(timeout_sec=5.0):
        raise RuntimeError("Required localizer service is unavailable")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError("Localizer service timed out or failed")
    return future.result()


def live_transform(node, target, source, max_age=1.0):
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            transform = buffer.lookup_transform(target, source, rclpy.time.Time())
        except TransformException:
            continue
        stamp = transform.header.stamp
        age = (node.get_clock().now().nanoseconds -
               (stamp.sec * 10**9 + stamp.nanosec)) / 1e9
        if 0 <= age <= max_age:
            q = transform.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y * q.y + q.z * q.z))
            t = transform.transform.translation
            return float(t.x), float(t.y), yaw
    raise RuntimeError(f"No fresh {target}->{source} TF within 5 seconds")


def map_odom_from_base(map_base, odom_base):
    mx, my, myaw = map_base
    ox, oy, oyaw = odom_base
    yaw = math.remainder(myaw - oyaw, 2 * math.pi)
    c, s = math.cos(yaw), math.sin(yaw)
    return mx - (c * ox - s * oy), my - (s * ox + c * oy), yaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("map", help="The same PCD map used by Nav3D")
    parser.add_argument("--execute", action="store_true", help="Call /relocalize after checks")
    parser.add_argument("--learn-start", action="store_true",
                        help="Save today's verified map->base parking pose for future runs")
    parser.add_argument("--anchor-file", help="Parking pose JSON; default: MAP.start_pose.json")
    parser.add_argument("--anchor-radius", type=float, default=1.0)
    parser.add_argument("--scan-seconds", type=float, default=3.0)
    parser.add_argument("--scan-radius", type=float, default=8.0)
    args = parser.parse_args()
    if args.scan_seconds < 1.0 or args.scan_radius < 2.0:
        parser.error("scan duration must be >=1 s and radius >=2 m")

    rclpy.init()
    node = ScanCollector(args.scan_seconds, args.scan_radius)
    try:
        anchor_file = Path(args.anchor_file or (args.map + ".start_pose.json"))
        if args.learn_start:
            if args.execute:
                raise RuntimeError("--learn-start and --execute cannot be combined")
            pose = live_transform(node, "map", "base_link")
            anchor_file.write_text(json.dumps({"x": pose[0], "y": pose[1],
                                               "yaw": pose[2], "map": str(Path(args.map).resolve())},
                                              indent=2) + "\n")
            print(f"Saved current parking pose to {anchor_file}: {pose}")
            return 0
        print("Collecting stationary FAST-LIO scans; do not move the robot...", flush=True)
        scan = node.collect()
        map_points = read_pcd_xyz(args.map)
        map_points = map_points[np.isfinite(map_points).all(axis=1)]
        scan = scan[np.isfinite(scan).all(axis=1)]
        height_band = lambda p: p[(p[:, 2] >= -0.33) & (p[:, 2] <= 0.39)]
        map_xyz = voxel_downsample(height_band(map_points), 0.10)
        scan_xyz = voxel_downsample(height_band(scan), 0.10)
        if len(map_xyz) < 1000 or len(scan_xyz) < 100:
            raise RuntimeError("Too few obstacle-height map/scan points for global matching")
        # Stable, spatially distributed points keep the coarse search bounded.
        if len(scan_xyz) > 400:
            scan_xyz = scan_xyz[np.linspace(0, len(scan_xyz) - 1, 400).astype(int)]
        candidates = coarse_candidates(map_xyz[:, :2], scan_xyz[:, :2])
        tree = cKDTree(map_xyz)
        solutions = sorted((refine(tree, scan_xyz, item) for item in candidates))
        if not solutions:
            raise RuntimeError("No map alignment candidate")
        if anchor_file.exists():
            saved = json.loads(anchor_file.read_text())
            if saved.get("map") != str(Path(args.map).resolve()):
                raise RuntimeError("Parking anchor belongs to a different map")
            odom_base = live_transform(node, "odom", "base_link")
            expected = map_odom_from_base(
                (float(saved["x"]), float(saved["y"]), float(saved["yaw"])), odom_base)
            nearby = [item for item in solutions
                      if math.hypot(item[3] - expected[0], item[4] - expected[1]) <=
                      args.anchor_radius and
                      abs(math.remainder(item[5] - expected[2], 2 * math.pi)) <= 0.5]
            if not nearby:
                raise RuntimeError("No scan match near the learned parking location")
            if nearby[0][0] - solutions[0][0] > 0.06:
                raise RuntimeError("Parking anchor disagrees with the best global scan match")
            solutions = nearby
            print(f"Using learned parking anchor: expected map->odom "
                  f"({expected[0]:.3f}, {expected[1]:.3f}, {expected[2]:.3f})")
        score, inliers, median, x, y, yaw = solutions[0]
        print(f"Best map->odom: x={x:.3f} y={y:.3f} yaw={yaw:.4f} rad "
              f"score={score:.3f} inliers={inliers:.2%} median={median:.3f} m")
        if len(solutions) > 1:
            other = solutions[1]
            print(f"Second candidate: x={other[3]:.3f} y={other[4]:.3f} "
                  f"yaw={other[5]:.4f} score={other[0]:.3f}")
        ambiguous = len(solutions) > 1 and solutions[1][0] - score < 0.06 and (
            math.hypot(solutions[1][3] - x, solutions[1][4] - y) > 0.5 or
            abs(math.remainder(solutions[1][5] - yaw, 2 * math.pi)) > 0.2)
        if score > 0.30 or inliers < 0.65 or median > 0.22 or ambiguous:
            raise RuntimeError("Global match is weak or ambiguous; refusing relocalization")
        if not args.execute:
            print("Dry run only. Re-run with --execute after reviewing this result.")
            return 0
        request = Relocalize.Request()
        request.pcd_path = args.map
        request.x, request.y, request.z = x, y, 0.0
        request.yaw, request.pitch, request.roll = yaw, 0.0, 0.0
        response = call_service(node, node.create_client(Relocalize, "/relocalize"),
                                request, 30.0)
        if not response.success:
            raise RuntimeError(f"/relocalize rejected request: {response.message}")
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, node)
        check = node.create_client(IsValid, "/relocalize_check")
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            result = call_service(node, check, IsValid.Request(code=0), 3.0)
            if result.valid:
                try:
                    transform = tf_buffer.lookup_transform("map", "odom", rclpy.time.Time())
                    age = node.get_clock().now().nanoseconds - (
                        transform.header.stamp.sec * 10**9 + transform.header.stamp.nanosec)
                    if 0 <= age < 10**9:
                        print("Relocalization succeeded with a fresh map->odom TF.")
                        return 0
                except TransformException:
                    pass
            rclpy.spin_once(node, timeout_sec=0.2)
        raise RuntimeError("Localizer did not produce valid, fresh map->odom TF")
    except (RuntimeError, OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
