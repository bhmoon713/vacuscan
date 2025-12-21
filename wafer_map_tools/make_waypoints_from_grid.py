#!/usr/bin/env python3
"""
Generate wafer waypoint YAML (same style as waypoints_old.yaml) from a grid.

Grid values:
  0 = empty/outside wafer
  1 = good die  -> generate waypoint
  2 = reject die -> ignored (no waypoint)

Coordinate mapping:
  - Grid center is treated as (0,0) in mm.
  - +x to the right (increasing column)
  - +y upward (decreasing row index)

Scan order:
  - row-by-row serpentine to reduce travel
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Tuple, Dict, Any
import math
import yaml


@dataclass
class RouteConfig:
    name: str = "scan_from_grid"
    tol_mm: float = 30.0
    timeout_s: float = 8.0
    pause_s: float = 1.2
    # Optional pause inserted every N points (0 disables)
    pause_every_n: int = 0
    pause_each_s: float = 0.0

    # Pitches
    x_pitch_mm: float = 10.0
    y_pitch_mm: float = 12.0

    # Optional offsets (mm) if your physical origin isn't at grid center
    x_offset_mm: float = 0.0
    y_offset_mm: float = 0.0

    # Add "go home" start/end points like your old file often does
    add_home_start: bool = True
    add_home_end: bool = True
    home_x_mm: float = 0.0
    home_y_mm: float = 0.0

    # Rounding for nicer YAML numbers
    round_mm: int = 3


def grid_center(cols: int, rows: int) -> Tuple[float, float]:
    """
    Returns (cx, cy) as the center in grid coordinates (can be half-integers).
    Example: cols=25 -> cx=12.0, rows=23 -> cy=11.0
    """
    cx = (cols - 1) / 2.0
    cy = (rows - 1) / 2.0
    return cx, cy


def cell_to_xy_mm(r: int, c: int, cx: float, cy: float, cfg: RouteConfig) -> Tuple[float, float]:
    # +x to the right, +y upward
    x = (c - cx) * cfg.x_pitch_mm + cfg.x_offset_mm
    y = (cy - r) * cfg.y_pitch_mm + cfg.y_offset_mm

    if cfg.round_mm is not None:
        x = round(x, cfg.round_mm)
        y = round(y, cfg.round_mm)
    return x, y


def serpentine_points(grid: List[List[int]]) -> List[Tuple[int, int]]:
    rows = len(grid)
    cols = len(grid[0]) if rows else 0
    pts: List[Tuple[int, int]] = []
    for r in range(rows):
        cols_iter = range(cols) if (r % 2 == 0) else range(cols - 1, -1, -1)
        for c in cols_iter:
            if grid[r][c] == 1:
                pts.append((r, c))
    return pts


def build_route_points(grid: List[List[int]], cfg: RouteConfig) -> List[Dict[str, Any]]:
    rows = len(grid)
    cols = len(grid[0]) if rows else 0
    if rows == 0 or cols == 0:
        return []

    cx, cy = grid_center(cols, rows)

    rc_points = serpentine_points(grid)

    out: List[Dict[str, Any]] = []

    # Start at home
    if cfg.add_home_start:
        out.append({"x_mm": cfg.home_x_mm, "y_mm": cfg.home_y_mm})

    # Convert each good-die cell to {x_mm, y_mm}
    for i, (r, c) in enumerate(rc_points):
        x_mm, y_mm = cell_to_xy_mm(r, c, cx, cy, cfg)
        out.append({"x_mm": x_mm, "y_mm": y_mm})

        # Optional pause insert
        if cfg.pause_every_n > 0 and cfg.pause_each_s > 0.0:
            if (i + 1) % cfg.pause_every_n == 0:
                out.append({"pause_s": float(cfg.pause_each_s)})

    # End at home
    if cfg.add_home_end:
        out.append({"x_mm": cfg.home_x_mm, "y_mm": cfg.home_y_mm})

    return out


def make_yaml_dict(grid: List[List[int]], cfg: RouteConfig) -> Dict[str, Any]:
    route = {
        cfg.name: {
            "tol_mm": float(cfg.tol_mm),
            "timeout_s": float(cfg.timeout_s),
            "pause_s": float(cfg.pause_s),
            "points": build_route_points(grid, cfg),
        }
    }
    return {"routes": route}


def main():
    # ====== YOUR GRID HERE ======
    grid = [
        [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
        [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
        [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],
        [0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0],
        [0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0],
        [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
        [0,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
        [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
        [0,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,0],
        [0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0],
        [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],
        [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],
        [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
        [0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0],
    ]

    cfg = RouteConfig(
        name="scan_from_grid",
        tol_mm=30,
        timeout_s=8,
        pause_s=1.2,
        x_pitch_mm=10.0,   # <-- your x pitch
        y_pitch_mm=12.0,   # <-- your y pitch
        add_home_start=True,
        add_home_end=True,
        home_x_mm=0.0,
        home_y_mm=0.0,
        pause_every_n=0,   # e.g. 25 if you want a pause every 25 points
        pause_each_s=0.0,  # e.g. 0.2 seconds
    )

    data = make_yaml_dict(grid, cfg)

    out_path = "waypoints_from_grid.yaml"
    with open(out_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(
            data,
            f,
            sort_keys=False,
            default_flow_style=False,
            width=120,
        )

    good_count = sum(v == 1 for row in grid for v in row)
    reject_count = sum(v == 2 for row in grid for v in row)
    print(f"Wrote: {out_path}")
    print(f"good dies (1)   : {good_count}")
    print(f"reject dies (2) : {reject_count} (ignored)")
    print("Done.")


if __name__ == "__main__":
    main()
