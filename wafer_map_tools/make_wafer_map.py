#!/usr/bin/env python3
import math
from pathlib import Path
import cv2
import numpy as np
import yaml

# =========================
# 0 = empty, 1 = GOOD, 2 = REJECT
# =========================
grid = [
    [0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0],
    [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
    [0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0],
    [0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,1,1,0,0],
    [0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0],
    [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
    [1,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [0,1,1,1,1,2,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,0],
    [0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0],
    [0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0],
    [0,0,0,1,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,1,1,0,0,0],
    [0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0],
    [0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0],
    [0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0],
    [0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0],
]


# =========================
# Image drawing settings
# =========================
DIE_PX = 24
GAP_PX = 2
MARGIN_PX = 30
DRAW_GRID_LINES = True

WHITE  = (255, 255, 255)
YELLOW = (0, 255, 255)     # BGR
GRAY   = (160, 160, 160)   # BGR
OUTLINE = (60, 60, 60)

# =========================
# Waypoint generation settings (mm)
# =========================
PITCH_X_MM = 10.0
PITCH_Y_MM = 12.0

# Where is (0,0) in the grid?
# "center" means the center cell between middle indices becomes origin.
# For odd sizes (like 25x23), center is an actual cell.
ORIGIN_MODE = "center"  # "center" or "topleft"

# If ORIGIN_MODE == "topleft", you can place physical origin offsets (mm)
ORIGIN_OFFSET_X_MM = 0.0
ORIGIN_OFFSET_Y_MM = 0.0

# Scan order for waypoints:
# "row_major" = top->bottom rows, left->right within row
# "snake" = top->bottom, alternating left->right then right->left
SCAN_ORDER = "snake"  # "row_major" or "snake"

OUT_DIR = Path("output")
OUT_DIR.mkdir(exist_ok=True)

OUT_PNG = OUT_DIR / "wafer_map.png"
OUT_YAML = OUT_DIR / "waypoints.yaml"


def grid_to_xy_mm(r: int, c: int, rows: int, cols: int) -> tuple[float, float]:
    """
    Convert grid (row, col) to physical (x_mm, y_mm).
    +x to the right, +y downward by default (image coordinates).
    If you want +y upward, flip the sign on y.
    """
    if ORIGIN_MODE == "center":
        c0 = (cols - 1) / 2.0
        r0 = (rows - 1) / 2.0
        x = (c - c0) * PITCH_X_MM
        y = (r - r0) * PITCH_Y_MM
        return (x, y)

    # top-left origin
    x = ORIGIN_OFFSET_X_MM + c * PITCH_X_MM
    y = ORIGIN_OFFSET_Y_MM + r * PITCH_Y_MM
    return (x, y)


def build_waypoints(grid: list[list[int]]) -> list[dict]:
    rows = len(grid)
    cols = max(len(row) for row in grid)
    for i, row in enumerate(grid):
        if len(row) != cols:
            raise ValueError(f"Row {i} length {len(row)} != {cols}")

    pts = []
    for r in range(rows):
        cols_range = range(cols)
        if SCAN_ORDER == "snake" and (r % 2 == 1):
            cols_range = reversed(range(cols))

        for c in cols_range:
            if grid[r][c] != 1:
                continue  # ONLY good die => waypoint
            x_mm, y_mm = grid_to_xy_mm(r, c, rows, cols)
            pts.append({
                "x": float(x_mm),
                "y": float(y_mm),
                "row": int(r),
                "col": int(c),
            })

    return pts


def draw_map(grid: list[list[int]]) -> np.ndarray:
    rows = len(grid)
    cols = max(len(row) for row in grid)
    for i, row in enumerate(grid):
        if len(row) != cols:
            raise ValueError(f"Row {i} length {len(row)} != {cols}")

    W = MARGIN_PX * 2 + cols * DIE_PX + (cols - 1) * GAP_PX
    H = MARGIN_PX * 2 + rows * DIE_PX + (rows - 1) * GAP_PX

    img = np.ones((H, W, 3), dtype=np.uint8) * 255

    for r in range(rows):
        for c in range(cols):
            val = grid[r][c]
            if val == 0:
                continue

            x0 = MARGIN_PX + c * (DIE_PX + GAP_PX)
            y0 = MARGIN_PX + r * (DIE_PX + GAP_PX)
            x1 = x0 + DIE_PX
            y1 = y0 + DIE_PX

            if val == 1:
                color = YELLOW
            elif val == 2:
                color = GRAY
            else:
                # unknown values -> draw red-ish as debug
                color = (40, 40, 255)

            cv2.rectangle(img, (x0, y0), (x1, y1), color, -1)
            if DRAW_GRID_LINES:
                cv2.rectangle(img, (x0, y0), (x1, y1), OUTLINE, 1)

    return img


def main():
    # 1) Draw image
    img = draw_map(grid)
    cv2.imwrite(str(OUT_PNG), img)

    # 2) Make waypoints yaml (only GOOD=1)
    points = build_waypoints(grid)

    doc = {
        "routes": {
            "wafer_good_dies": {
                "meta": {
                    "pitch_x_mm": PITCH_X_MM,
                    "pitch_y_mm": PITCH_Y_MM,
                    "origin_mode": ORIGIN_MODE,
                    "origin_offset_x_mm": ORIGIN_OFFSET_X_MM,
                    "origin_offset_y_mm": ORIGIN_OFFSET_Y_MM,
                    "scan_order": SCAN_ORDER,
                    "rows": len(grid),
                    "cols": max(len(r) for r in grid),
                },
                "points": points
            }
        }
    }

    with open(OUT_YAML, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False)

    print(f"Saved: {OUT_PNG}")
    print(f"Saved: {OUT_YAML}")
    print(f"Waypoints created (GOOD=1 only): {len(points)}")


if __name__ == "__main__":
    main()

