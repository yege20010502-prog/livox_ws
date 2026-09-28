#!/usr/bin/env python3
"""Create a conservative binary PCD navigation candidate from a PGO map.

Only height-crops the map; it never levels, translates, downsamples, or
classifies occupied columns as free. The source file is never modified.
"""

import argparse
import json
from pathlib import Path

import numpy as np


def read_binary_pcd(path):
    header = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header has no DATA line")
            parts = line.decode("ascii").strip().split()
            if parts:
                header[parts[0].upper()] = parts[1:]
            if parts and parts[0].upper() == "DATA":
                offset = stream.tell()
                break
    if header["DATA"] != ["binary"]:
        raise ValueError("Only DATA binary PCD is supported")
    fields = header["FIELDS"]
    sizes = [int(v) for v in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
    if not (len(fields) == len(sizes) == len(types) == len(counts)):
        raise ValueError("Invalid PCD field definition")
    if any(c != 1 for c in counts):
        raise ValueError("Multi-count PCD fields are not supported")
    dtype_names = []
    dtype_formats = []
    for name, size, kind in zip(fields, sizes, types):
        code = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "u1",
                ("U", 2): "<u2", ("U", 4): "<u4", ("I", 1): "i1",
                ("I", 2): "<i2", ("I", 4): "<i4"}.get((kind, size))
        if code is None:
            raise ValueError(f"Unsupported PCD field: {name} {kind}{size}")
        dtype_names.append(name)
        dtype_formats.append(code)
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("PCD must contain x, y, z fields")
    dtype = np.dtype({"names": dtype_names, "formats": dtype_formats})
    count = int(header["POINTS"][0])
    if path.stat().st_size - offset != count * dtype.itemsize:
        raise ValueError("PCD byte count does not match POINTS and fields")
    points = np.memmap(path, dtype=dtype, mode="r", offset=offset, shape=(count,))
    return header, points


def estimate_floor(z):
    # Ignore the dominant ceiling by searching only the lower part of the map.
    low, high = np.quantile(z, [0.01, 0.35])
    if high - low < 0.05:
        raise ValueError("Too little vertical range to identify a floor")
    edges = np.arange(low, high + 0.05, 0.05)
    hist, edges = np.histogram(z, bins=edges)
    peak = int(np.argmax(hist))
    center = float((edges[peak] + edges[peak + 1]) / 2)
    near = z[(z >= center - 0.05) & (z <= center + 0.05)]
    if len(near) < max(1000, len(z) * 0.01):
        raise ValueError("No strong floor peak; provide --floor-z after inspection")
    return float(np.median(near)), int(len(near))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="PGO original map.pcd")
    parser.add_argument("output", type=Path, help="new DATA binary PCD path")
    parser.add_argument("--floor-z", type=float, help="measured floor Z; omit for automatic estimate")
    parser.add_argument("--below", type=float, default=0.15, help="retain this far below floor (m)")
    parser.add_argument("--above", type=float, default=2.0, help="retain this far above floor (m)")
    args = parser.parse_args()
    if args.below < 0 or args.above <= 0:
        parser.error("--below must be >= 0 and --above must be > 0")
    source = args.input.resolve()
    output = args.output.resolve()
    report = output.with_suffix(".json")
    if source == output or output.exists() or report.exists():
        parser.error("output/report already exists or is the input; choose a new filename")
    if not output.parent.is_dir():
        parser.error("output directory must already exist")
    header, points = read_binary_pcd(source)
    xyz = np.column_stack((points["x"], points["y"], points["z"]))
    valid = np.isfinite(xyz).all(axis=1)
    if valid.sum() < 1000:
        raise ValueError("Too few finite XYZ points")
    z = np.asarray(points["z"][valid], dtype=np.float64)
    floor_z, support = (float(args.floor_z), None) if args.floor_z is not None else estimate_floor(z)
    lower, upper = floor_z - args.below, floor_z + args.above
    keep = valid & (points["z"] >= lower) & (points["z"] <= upper)
    if keep.sum() < 1000:
        raise ValueError("Crop would leave too few points")
    if keep.sum() / valid.sum() < 0.10:
        raise ValueError("Crop would remove over 90% of finite points; check floor Z")
    selected = points[keep]
    # Exclusive creation protects both the source and any earlier candidates.
    with output.open("xb") as stream:
        stream.write(b"# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n")
        for key in ("FIELDS", "SIZE", "TYPE", "COUNT"):
            stream.write((key + " " + " ".join(header.get(key, ["1"] * len(header["FIELDS"]))) + "\n").encode())
        stream.write(f"WIDTH {len(selected)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(selected)}\nDATA binary\n".encode())
        selected.tofile(stream)
    summary = {
        "source": str(source), "output": str(output), "source_points": int(len(points)),
        "kept_points": int(len(selected)), "removed_points": int(len(points) - len(selected)),
        "floor_z": floor_z, "floor_peak_support": support, "min_z": lower, "max_z": upper,
        "source_z_percentiles_01_05_50_95_99": np.quantile(z, [0.01, 0.05, 0.5, 0.95, 0.99]).tolist(),
        "coordinates_changed": False,
        "warning": "Candidate only: visually verify floor, obstacles, and traversability before navigation",
    }
    with report.open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
