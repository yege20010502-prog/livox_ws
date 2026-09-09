#!/usr/bin/env python3

from pathlib import Path
import numpy as np
import yaml

package_dir = Path("/home/electron/maps/map_obstacles")
ground_pcd = Path(
    "/home/electron/nav/livox_ws/maps/map831/map_ground_ascii.pcd"
)

with (package_dir / "meta.yaml").open("r", encoding="utf-8") as f:
    meta = yaml.safe_load(f)

layers = np.load(package_dir / "layers.npz", allow_pickle=False)

resolution = float(meta["resolution"])
xmin, ymin, _ = map(float, meta["bounds"]["min"])
xmax, ymax, _ = map(float, meta["bounds"]["max"])

width = int(round((xmax - xmin) / resolution))
height = int(round((ymax - ymin) / resolution))


def load_ascii_pcd(path):
    lines = path.read_text(encoding="utf-8").splitlines()
    data_line = next(
        i for i, line in enumerate(lines)
        if line.strip().lower() == "data ascii"
    )
    return np.loadtxt(lines[data_line + 1:], dtype=np.float64)


def rasterize(points):
    mask = np.zeros((height, width), dtype=bool)

    ix = np.floor((points[:, 0] - xmin) / resolution).astype(int)
    iy = np.floor((points[:, 1] - ymin) / resolution).astype(int)

    valid = (
        (ix >= 0) & (ix < width) &
        (iy >= 0) & (iy < height)
    )

    mask[iy[valid], ix[valid]] = True
    return mask


def dilate(mask, radius):
    output = np.zeros_like(mask)

    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius:
                continue

            source_y0 = max(0, -dy)
            source_y1 = min(height, height - dy)
            source_x0 = max(0, -dx)
            source_x1 = min(width, width - dx)

            target_y0 = source_y0 + dy
            target_y1 = source_y1 + dy
            target_x0 = source_x0 + dx
            target_x1 = source_x1 + dx

            output[target_y0:target_y1, target_x0:target_x1] |= \
                mask[source_y0:source_y1, source_x0:source_x1]

    return output


ground_points = load_ascii_pcd(ground_pcd)
preblocked_points = np.asarray(
    layers["preblocked_points"], dtype=np.float64
)

free_mask = rasterize(ground_points)

# 填补激光扫描线之间的小空隙，半径2格=0.2米
free_mask = dilate(free_mask, 2)

occupied_mask = rasterize(preblocked_points)

# PGM：0占据、205未知、254自由
grid = np.full((height, width), 205, dtype=np.uint8)
grid[free_mask] = 254

# 障碍物最后覆盖，避免自由区域覆盖障碍物
grid[occupied_mask] = 0

pgm_grid = np.flipud(grid)

with (package_dir / "map.pgm").open("wb") as f:
    f.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
    f.write(pgm_grid.tobytes())

(package_dir / "map.yaml").write_text(
    f"""image: map.pgm
mode: trinary
resolution: {resolution}
origin: [{xmin}, {ymin}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""",
    encoding="utf-8",
)

print("地图尺寸:", width, "x", height)
print("占据:", np.count_nonzero(grid == 0))
print("自由:", np.count_nonzero(grid == 254))
print("未知:", np.count_nonzero(grid == 205))
