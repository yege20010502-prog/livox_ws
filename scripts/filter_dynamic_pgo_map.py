#!/usr/bin/env python3
"""Remove transient PGO points using keyframe observation consistency.

The source map is never modified. Ground points are protected; elevated
voxels must be observed in separated keyframes to be retained.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from prepare_pgo_map import estimate_floor, read_binary_pcd


def rotation_matrix(q):
    w, x, y, z = q
    return np.asarray([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=np.float64)


def load_poses(path):
    poses = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        fields = line.split()
        if not fields:
            continue
        if len(fields) < 8:
            raise ValueError(f"Invalid poses line {line_number}")
        name = fields[0]
        translation = np.asarray([float(v) for v in fields[1:4]], dtype=np.float64)
        quaternion = np.asarray([float(v) for v in fields[4:8]], dtype=np.float64)
        norm = np.linalg.norm(quaternion)
        if norm < 1e-9:
            raise ValueError(f"Invalid quaternion on poses line {line_number}")
        poses.append((name, translation, quaternion / norm))
    if not poses:
        raise ValueError("No poses found")
    return poses


def voxel_keys(xyz, resolution):
    return np.floor(xyz / resolution + 1e-9).astype(np.int32)


def write_binary_pcd(path, header, selected):
    with path.open("xb") as stream:
        stream.write(b"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        for key in ("FIELDS", "SIZE", "TYPE", "COUNT"):
            values = header.get(key, ["1"] * len(header["FIELDS"]))
            stream.write((key + " " + " ".join(values) + "\n").encode())
        stream.write(
            f"WIDTH {len(selected)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            f"POINTS {len(selected)}\nDATA binary\n".encode())
        selected.tofile(stream)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="height/XY cropped navigation candidate")
    parser.add_argument("output", type=Path, help="new filtered PCD (must not exist)")
    parser.add_argument("--poses", type=Path, required=True)
    parser.add_argument("--patches", type=Path, required=True)
    parser.add_argument("--resolution", type=float, default=0.15,
                        help="consistency voxel size in metres")
    parser.add_argument("--min-frame-gap", type=int, default=5,
                        help="first/last observing keyframes must differ by this amount")
    parser.add_argument("--ground-band", type=float, default=0.15,
                        help="protect points within this height above estimated floor")
    parser.add_argument("--floor-z", type=float)
    args = parser.parse_args()
    if args.resolution <= 0 or args.min_frame_gap < 1 or args.ground_band < 0:
        parser.error("invalid filtering parameter")
    source, output = args.input.resolve(), args.output.resolve()
    report = output.with_suffix(".json")
    if source == output or output.exists() or report.exists():
        parser.error("output/report exists or equals input; choose a new output")
    if not args.patches.is_dir() or not args.poses.is_file():
        parser.error("--patches directory or --poses file does not exist")

    poses = load_poses(args.poses)
    observations = {}
    used_frames = 0
    for frame_index, (name, translation, quaternion) in enumerate(poses):
        patch_path = args.patches / name
        if not patch_path.is_file():
            continue
        _, patch = read_binary_pcd(patch_path)
        local = np.column_stack((patch["x"], patch["y"], patch["z"])).astype(np.float64)
        local = local[np.isfinite(local).all(axis=1)]
        world = local @ rotation_matrix(quaternion).T + translation
        keys = np.unique(voxel_keys(world, args.resolution), axis=0)
        for key_array in keys:
            key = tuple(int(v) for v in key_array)
            record = observations.get(key)
            if record is None:
                observations[key] = [frame_index, frame_index, 1]
            else:
                record[1] = frame_index
                record[2] += 1
        used_frames += 1

    header, points = read_binary_pcd(source)
    xyz = np.column_stack((points["x"], points["y"], points["z"])).astype(np.float64)
    finite = np.isfinite(xyz).all(axis=1)
    floor_z = args.floor_z
    if floor_z is None:
        floor_z, _ = estimate_floor(xyz[finite, 2])
    keys = voxel_keys(xyz, args.resolution)
    persistent = np.zeros(len(points), dtype=bool)
    for index in np.flatnonzero(finite):
        record = observations.get(tuple(int(v) for v in keys[index]))
        persistent[index] = record is not None and record[2] >= 2 and \
            record[1] - record[0] >= args.min_frame_gap
    ground = finite & (xyz[:, 2] <= floor_z + args.ground_band)
    keep = finite & (ground | persistent)
    if keep.sum() < 1000:
        raise ValueError("filter would leave too few points")
    selected = points[keep]
    write_binary_pcd(output, header, selected)
    summary = {
        "source": str(source), "output": str(output),
        "source_points": int(len(points)), "kept_points": int(keep.sum()),
        "removed_points": int(len(points) - keep.sum()),
        "protected_ground_points": int(ground.sum()),
        "persistent_elevated_points": int((persistent & ~ground).sum()),
        "floor_z": float(floor_z), "ground_band": args.ground_band,
        "consistency_resolution": args.resolution,
        "min_frame_gap": args.min_frame_gap, "poses": len(poses),
        "patches_used": used_frames, "coordinates_changed": False,
        "warning": "Candidate only: compare against source before navigation",
    }
    report.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
