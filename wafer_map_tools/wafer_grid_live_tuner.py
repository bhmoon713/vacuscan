#!/usr/bin/env python3
"""
Wafer grid live tuner (OpenCV)

Features
- Loads a wafer image
- Detects ring inner circle (Hough) -> px_per_mm
- Derives wafer circle from ring (300/350)
- Live classification tuning with sliders:
    roi_half_px, std_thr, edge_thr, mean_thr_no_die, inside_margin_mm, min_neighbors, show_labels
- Live geometry tuning with keyboard (panel on right):
    pitch_x_mm, pitch_y_mm, x_offset_mm, y_offset_mm
- Press:
    1/2/3/4 : select geometry parameter
    Arrow Up/Down : +/- 0.01 mm
    Arrow Left/Right : +/- 0.10 mm
    SHIFT (optional): use hotkeys Z/X for step size (fine/coarse)
    R : RUN export (grid txt + waypoints csv/yaml + debug overlay PNG + params txt)
    S : Save overlay PNG only
    P : Print current parameters
    Q / ESC : quit

Install:
  pip install opencv-python numpy pyyaml   (pyyaml optional)

Example:
  python3 wafer_grid_live_tuner.py --image /mnt/data/wafer_image.png \
    --rows 27 --cols 27 --pitch_x_mm 10.040 --pitch_y_mm 10.040 \
    --x_offset_mm 5 --y_offset_mm 6
"""

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import List, Tuple
import yaml

import cv2
import numpy as np

try:
    import yaml
except Exception:
    yaml = None


# ============================================================
# Hard-coded waypoint YAML header (standard format)
# ============================================================

WAYPOINT_HEADER = {
    "routes": {
        "scan_from_grid": {
            "tol_mm": 0.05,
            "timeout_s": 2.0,
            "pause_s": 0.0,
            # "points" will be inserted dynamically
        }
    }
}


# ---------------------------- Data types ----------------------------

@dataclass
class Circle:
    cx: float
    cy: float
    r: float


@dataclass
class GridGeom:
    rows: int
    cols: int
    pitch_x_px: float
    pitch_y_px: float
    grid_cx_px: float
    grid_cy_px: float

    @property
    def origin_x_px(self) -> float:
        return self.grid_cx_px - (self.cols - 1) * self.pitch_x_px / 2.0

    @property
    def origin_y_px(self) -> float:
        return self.grid_cy_px - (self.rows - 1) * self.pitch_y_px / 2.0


# ---------------------------- Helpers ----------------------------

def ensure_dir(path: str) -> None:
    d = os.path.dirname(os.path.abspath(path))
    if d and not os.path.exists(d):
        os.makedirs(d, exist_ok=True)


def clamp_roi(img: np.ndarray, x0: int, y0: int, x1: int, y1: int) -> np.ndarray:
    h, w = img.shape[:2]
    x0 = max(0, min(w, x0))
    x1 = max(0, min(w, x1))
    y0 = max(0, min(h, y0))
    y1 = max(0, min(h, y1))
    if x1 <= x0 or y1 <= y0:
        return img[0:0, 0:0]
    return img[y0:y1, x0:x1]


def die_center_px(geom: GridGeom, r: int, c: int) -> Tuple[float, float]:
    x = geom.origin_x_px + c * geom.pitch_x_px
    y = geom.origin_y_px + r * geom.pitch_y_px
    return x, y


def outer_band_mask(h: int, w: int, band_ratio: float) -> np.ndarray:
    band = int(min(h, w) * band_ratio)
    m = np.zeros((h, w), dtype=np.uint8)
    m[:band, :] = 255
    m[-band:, :] = 255
    m[:, :band] = 255
    m[:, -band:] = 255
    return m


def detect_ring_inner_circle_hough(gray: np.ndarray,
                                  band_ratio: float = 0.22,
                                  dp: float = 1.2,
                                  param1: int = 140,
                                  param2: int = 28,
                                  r_min_frac: float = 0.42,
                                  r_max_frac: float = 0.55,
                                  center_tol_frac: float = 0.08) -> Circle:
    """
    Robust ring detection: HoughCircles on outer band with constraints.
    """
    g = gray
    h, w = g.shape[:2]
    cx0, cy0 = w / 2.0, h / 2.0

    mask = outer_band_mask(h, w, band_ratio)
    roi = cv2.bitwise_and(g, g, mask=mask)
    blur = cv2.GaussianBlur(roi, (9, 9), 1.5)

    minR = int(min(h, w) * r_min_frac)
    maxR = int(min(h, w) * r_max_frac)

    circles = cv2.HoughCircles(
        blur, cv2.HOUGH_GRADIENT,
        dp=dp,
        minDist=min(h, w) * 0.35,
        param1=param1,
        param2=param2,
        minRadius=minR,
        maxRadius=maxR
    )
    if circles is None:
        raise RuntimeError("Ring detection failed. Try lowering ring_param2 (e.g. 22) or adjust radius fractions.")

    cand = np.squeeze(circles).astype(np.float32)
    if cand.ndim == 1:
        cand = cand[None, :]

    tol = min(h, w) * center_tol_frac
    good = []
    for (cx, cy, r) in cand:
        if abs(cx - cx0) <= tol and abs(cy - cy0) <= tol:
            good.append((float(cx), float(cy), float(r)))
    if not good:
        good = [(float(cx), float(cy), float(r)) for (cx, cy, r) in cand]

    def score(c):
        cx, cy, r = c
        dc = math.hypot(cx - cx0, cy - cy0)
        return (r * 2.0) - dc

    cx, cy, r = max(good, key=score)
    return Circle(cx=cx, cy=cy, r=r)


# ---------------------------- Classification ----------------------------

def classify_die(gray: np.ndarray,
                 x: float, y: float,
                 roi_half: int,
                 std_thr: float,
                 edge_thr: float,
                 mean_thr_no_die: float) -> int:
    """
    Returns:
      1 = remnant die (waypoint)
      4 = no-die (ignore)

    Tuned for your use case: low std_thr + low edge_thr can catch weak dies.
    mean_thr_no_die is used as a "bright+flat => no-die" rule; you can set it low to mostly disable.
    """
    xi, yi = int(round(x)), int(round(y))
    roi = clamp_roi(gray, xi - roi_half, yi - roi_half, xi + roi_half, yi + roi_half)
    if roi.size == 0:
        return 4

    mean_val = float(np.mean(roi))
    std_val = float(np.std(roi))

    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    edges = cv2.Canny(blur, 40, 120)
    edge_density = float(np.mean(edges > 0))

    # Bright + flat => no-die
    if mean_val > mean_thr_no_die and std_val < std_thr:
        return 4

    # Textured or edged => die
    if std_val >= std_thr or edge_density >= edge_thr:
        return 1

    return 4


def build_grid(gray: np.ndarray,
               wafer: Circle,
               geom: GridGeom,
               inside_margin_px: float,
               roi_half: int,
               std_thr: float,
               edge_thr: float,
               mean_thr_no_die: float) -> np.ndarray:
    grid = np.zeros((geom.rows, geom.cols), dtype=np.int32)

    die_half_diag = 0.5 * math.sqrt((geom.pitch_x_px / 2) ** 2 + (geom.pitch_y_px / 2) ** 2)

    for r in range(geom.rows):
        for c in range(geom.cols):
            x, y = die_center_px(geom, r, c)
            dist = math.hypot(x - wafer.cx, y - wafer.cy)

            if dist > (wafer.r - inside_margin_px - die_half_diag):
                grid[r, c] = 0
                continue

            grid[r, c] = classify_die(
                gray, x, y,
                roi_half=roi_half,
                std_thr=std_thr,
                edge_thr=edge_thr,
                mean_thr_no_die=mean_thr_no_die
            )
    return grid


def remove_isolated_dies(grid: np.ndarray, min_neighbors: int = 0) -> np.ndarray:
    """
    Optional cleanup: drop isolated green singletons.
    If min_neighbors <= 0: no change.
    """
    if min_neighbors <= 0:
        return grid
    out = grid.copy()
    H, W = grid.shape
    for r in range(H):
        for c in range(W):
            if out[r, c] != 1:
                continue
            neighbors = 0
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == 0 and dc == 0:
                        continue
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < H and 0 <= cc < W and out[rr, cc] == 1:
                        neighbors += 1
            if neighbors < min_neighbors:
                out[r, c] = 4
    return out


# ---------------------------- Outputs ----------------------------

def save_grid_text(grid: np.ndarray, path: str) -> None:
    ensure_dir(path)
    with open(path, "w", newline="") as f:
        for r in range(grid.shape[0]):
            f.write(",".join(str(int(v)) for v in grid[r]) + ",\n")


def grid_to_waypoints(grid: np.ndarray,
                      geom: GridGeom,
                      wafer: Circle,
                      px_per_mm: float,
                      home_x_mm: float,
                      home_y_mm: float,
                      y_down_is_positive: bool = True) -> List[dict]:
    wps = []

    rows, cols = grid.shape[0], grid.shape[1]
    for r in range(rows):
        # snake: even row L->R, odd row R->L
        col_range = range(cols) if (r % 2 == 0) else range(cols - 1, -1, -1)

        for c in col_range:
            if int(grid[r, c]) != 1:
                continue

            x_px, y_px = die_center_px(geom, r, c)
            dx_px = x_px - wafer.cx
            dy_px = y_px - wafer.cy

            x_mm = dx_px / px_per_mm
            y_mm = dy_px / px_per_mm
            if not y_down_is_positive:
                y_mm = -y_mm

            wps.append({
                "row": int(r),
                "col": int(c),
                "x_mm": float(home_x_mm + x_mm),
                "y_mm": float(home_y_mm + y_mm),
                "x_px": float(x_px),
                "y_px": float(y_px),
            })

    return wps



def save_waypoints_csv(waypoints: List[dict], path: str) -> None:
    ensure_dir(path)
    fields = ["row", "col", "x_mm", "y_mm", "x_px", "y_px"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        for wp in waypoints:
            w.writerow({k: wp.get(k) for k in fields})


def save_waypoints_yaml(waypoints, out_yaml):
    data = WAYPOINT_HEADER.copy()

    data["routes"]["scan_from_grid"]["points"] = waypoints

    with open(out_yaml, "w") as f:
        yaml.safe_dump(
            data,
            f,
            sort_keys=False,
            default_flow_style=False,
            indent=2
        )

    print(f"[OK] Waypoints saved to {out_yaml} ({len(waypoints)} points)")



def save_params_txt(path: str, params: dict) -> None:
    ensure_dir(path)
    with open(path, "w") as f:
        for k, v in params.items():
            f.write(f"{k}: {v}\n")


# ---------------------------- Drawing ----------------------------

def draw_grid_lines(img: np.ndarray, geom: GridGeom, thickness: int = 1) -> None:
    h, w = img.shape[:2]
    # vertical
    k_min = int(math.floor((0 - geom.grid_cx_px) / geom.pitch_x_px)) - 2
    k_max = int(math.ceil((w - geom.grid_cx_px) / geom.pitch_x_px)) + 2
    for k in range(k_min, k_max + 1):
        x = int(round(geom.grid_cx_px + k * geom.pitch_x_px))
        if 0 <= x < w:
            cv2.line(img, (x, 0), (x, h - 1), (0, 0, 0), thickness, cv2.LINE_AA)
    # horizontal
    k_min = int(math.floor((0 - geom.grid_cy_px) / geom.pitch_y_px)) - 2
    k_max = int(math.ceil((h - geom.grid_cy_px) / geom.pitch_y_px)) + 2
    for k in range(k_min, k_max + 1):
        y = int(round(geom.grid_cy_px + k * geom.pitch_y_px))
        if 0 <= y < h:
            cv2.line(img, (0, y), (w - 1, y), (0, 0, 0), thickness, cv2.LINE_AA)


def make_overlay(base_bgr: np.ndarray,
                 ring: Circle,
                 wafer: Circle,
                 geom: GridGeom,
                 grid: np.ndarray,
                 px_per_mm: float,
                 show_labels: bool) -> np.ndarray:
    img = base_bgr.copy()

    # ring inner (purple) + wafer (yellow)
    cv2.circle(img, (int(ring.cx), int(ring.cy)), int(ring.r), (255, 0, 255), 2)
    cv2.circle(img, (int(wafer.cx), int(wafer.cy)), int(wafer.r), (0, 255, 255), 2)
    # grid center (blue)
    cv2.circle(img, (int(geom.grid_cx_px), int(geom.grid_cy_px)), 4, (255, 0, 0), -1)

    draw_grid_lines(img, geom, thickness=1)

    overlay = img.copy()
    alpha = 0.35
    box = int(max(6, round(min(geom.pitch_x_px, geom.pitch_y_px) * 0.45)))

    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
            v = int(grid[r, c])
            x, y = die_center_px(geom, r, c)
            xi, yi = int(round(x)), int(round(y))

            if v == 0:
                color = (200, 200, 200)
            elif v == 1:
                color = (0, 255, 0)      # waypoint die
            else:
                color = (0, 0, 255)      # no-die

            x0, y0 = max(0, xi - box), max(0, yi - box)
            x1, y1 = min(img.shape[1] - 1, xi + box), min(img.shape[0] - 1, yi + box)
            cv2.rectangle(overlay, (x0, y0), (x1, y1), color, thickness=-1)

            if show_labels and v != 0:
                cv2.putText(overlay, f"{r},{c}", (xi - box, yi),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1, cv2.LINE_AA)

    out = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    info = [
        f"px_per_mm={px_per_mm:.4f}",
        "Keys: 1-4 select | arrows adjust | R export | S save | Q quit",
    ]
    for i, line in enumerate(info):
        cv2.putText(out, line, (10, 25 + 22*i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, line, (10, 25 + 22*i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (255, 255, 255), 1, cv2.LINE_AA)

    return out


def draw_right_panel(canvas: np.ndarray,
                     panel_x: int,
                     selected_param: int,
                     pitch_x_mm: float,
                     pitch_y_mm: float,
                     x_offset_mm: float,
                     y_offset_mm: float,
                     step_fine: float,
                     step_coarse: float) -> None:
    """
    Draw a parameter panel on the right side (inside the expanded canvas).
    """
    h, w = canvas.shape[:2]
    x0 = panel_x + 20
    y0 = 40

    # translucent background
    panel = canvas.copy()
    cv2.rectangle(panel, (panel_x, 0), (w - 1, h - 1), (0, 0, 0), -1)
    canvas[:] = cv2.addWeighted(panel, 0.25, canvas, 0.75, 0)

    def line(i, label, value, sel_idx):
        y = y0 + i * 34
        color = (0, 255, 255) if sel_idx == selected_param else (255, 255, 255)
        cv2.putText(canvas, f"{label}: {value:.3f}", (x0, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2, cv2.LINE_AA)

    cv2.putText(canvas, "Live Geometry", (x0, y0 - 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.85, (255, 255, 255), 2, cv2.LINE_AA)

    line(0, "1 pitch_x_mm", pitch_x_mm, 1)
    line(1, "2 pitch_y_mm", pitch_y_mm, 2)
    line(2, "3 x_offset_mm", x_offset_mm, 3)
    line(3, "4 y_offset_mm", y_offset_mm, 4)

    cv2.putText(canvas, f"Arrows: Up/Down +/-{step_fine:.2f}  Left/Right +/-{step_coarse:.2f}",
                (x0, y0 + 4 * 34 + 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Z: fine=0.001  X: fine=0.01  C: fine=0.05",
                (x0, y0 + 4 * 34 + 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(canvas, "Tip: select param with 1-4",
                (x0, y0 + 4 * 34 + 62),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (220, 220, 220), 1, cv2.LINE_AA)


# ---------------------------- Trackbar utilities ----------------------------

def tb_get(window: str, name: str) -> int:
    return cv2.getTrackbarPos(name, window)

# ---------------------------- creates a standalone panel image ----------------------------

def make_param_panel_img(selected_param, pitch_x_mm, pitch_y_mm, x_offset_mm, y_offset_mm,
                         step_fine, step_coarse, roi_half, std_thr, edge_thr, mean_thr_no_die,
                         inside_margin_mm, min_neighbors, show_labels):
    W, H = 520, 360
    img = np.zeros((H, W, 3), dtype=np.uint8)

    # Title
    cv2.putText(img, "Live Geometry", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (255, 255, 255), 2, cv2.LINE_AA)

    items = [
        (1, "pitch_x_mm", pitch_x_mm),
        (2, "pitch_y_mm", pitch_y_mm),
        (3, "x_offset_mm", x_offset_mm),
        (4, "y_offset_mm", y_offset_mm),
    ]
    y = 80
    for idx, name, val in items:
        color = (0, 255, 255) if idx == selected_param else (255, 255, 255)
        cv2.putText(img, f"{idx} {name}: {val:.3f}", (20, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
        y += 38

    cv2.putText(img, f"Arrows: Up/Down +/-{step_fine:.3f}  Left/Right +/-{step_coarse:.2f}",
                (20, y + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
    cv2.putText(img, "Z: fine=0.001  X: fine=0.01  C: fine=0.05",
                (20, y + 38), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)

    # Add classification values too (optional but useful)
    y2 = y + 90
    cv2.putText(img, "Classification", (20, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                (255, 255, 255), 2, cv2.LINE_AA)
    y2 += 35
    lines = [
        f"roi_half_px: {roi_half}",
        f"std_thr: {std_thr:.2f}",
        f"edge_thr: {edge_thr:.3f}",
        f"mean_thr_no_die: {mean_thr_no_die:.1f}",
        f"inside_margin_mm: {inside_margin_mm:.2f}",
        f"min_neighbors: {min_neighbors}",
        f"show_labels: {show_labels}",
    ]
    for ln in lines:
        cv2.putText(img, ln, (20, y2), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (220, 220, 220), 1, cv2.LINE_AA)
        y2 += 26

    return img



# ---------------------------- Key handling ----------------------------

def is_arrow_key(k: int) -> Tuple[bool, str]:
    """
    Robust arrow key detection across platforms.
    Returns (True, direction) where direction in {"up","down","left","right"}.
    """
    # Common codes from cv2.waitKeyEx on Windows/Linux
    up = {2490368, 65362}
    down = {2621440, 65364}
    left = {2424832, 65361}
    right = {2555904, 65363}

    if k in up:
        return True, "up"
    if k in down:
        return True, "down"
    if k in left:
        return True, "left"
    if k in right:
        return True, "right"
    return False, ""


# ---------------------------- Main ----------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)

    ap.add_argument("--rows", type=int, required=True)
    ap.add_argument("--cols", type=int, required=True)

    ap.add_argument("--pitch_x_mm", type=float, required=True)
    ap.add_argument("--pitch_y_mm", type=float, required=True)

    ap.add_argument("--x_offset_mm", type=float, default=0.0)
    ap.add_argument("--y_offset_mm", type=float, default=0.0)

    ap.add_argument("--ring_inner_d_mm", type=float, default=350.0)
    ap.add_argument("--wafer_d_mm", type=float, default=300.0)

    # outputs (used on RUN)
    ap.add_argument("--out_dir", default="out_live")
    ap.add_argument("--base_name", default="wafer")

    ap.add_argument("--home_x_mm", type=float, default=0.0)
    ap.add_argument("--home_y_mm", type=float, default=0.0)
    ap.add_argument("--y_up", action="store_true")

    args = ap.parse_args()

    base = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if base is None:
        raise FileNotFoundError(args.image)
    gray = cv2.cvtColor(base, cv2.COLOR_BGR2GRAY)

    # Detect ring once
    ring = detect_ring_inner_circle_hough(gray)
    px_per_mm = ring.r / (args.ring_inner_d_mm / 2.0)

    # Derive wafer from ring
    wafer_scale = args.wafer_d_mm / args.ring_inner_d_mm
    wafer = Circle(cx=ring.cx, cy=ring.cy, r=ring.r * wafer_scale)

    # ---- Live geometry variables (keyboard controlled) ----
    pitch_x_mm = float(args.pitch_x_mm)
    pitch_y_mm = float(args.pitch_y_mm)
    x_offset_mm = float(args.x_offset_mm)
    y_offset_mm = float(args.y_offset_mm)

    selected_param = 3  # default select x_offset (often tuned most)
    step_fine = 0.01
    step_coarse = 0.10

    # Window
    win = "Wafer Live Tuner"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    panel_win = "Live Params"
    cv2.namedWindow(panel_win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(panel_win, 520, 360)   # bigger panel window

    # Trackbars (classification)
    cv2.createTrackbar("roi_half_px", win, 30, 80, lambda x: None)
    cv2.createTrackbar("std_thr_x10", win, 60, 500, lambda x: None)          # 6.0 default
    cv2.createTrackbar("edge_thr_x1000", win, 30, 200, lambda x: None)       # 0.03 default
    cv2.createTrackbar("mean_thr_no_die", win, 100, 255, lambda x: None)
    cv2.createTrackbar("inside_margin_x10", win, 0, 100, lambda x: None)     # 0.0 mm
    cv2.createTrackbar("min_neighbors", win, 0, 6, lambda x: None)
    cv2.createTrackbar("show_labels", win, 0, 1, lambda x: None)

    last_params = None
    last_canvas = None
    last_grid = None

    panel_w = 420

    while True:
        # ----- read sliders -----
        roi_half = max(5, tb_get(win, "roi_half_px"))
        std_thr = tb_get(win, "std_thr_x10") / 10.0
        edge_thr = tb_get(win, "edge_thr_x1000") / 1000.0
        mean_thr_no_die = tb_get(win, "mean_thr_no_die")
        inside_margin_mm = tb_get(win, "inside_margin_x10") / 10.0
        min_neighbors = tb_get(win, "min_neighbors")
        show_labels = tb_get(win, "show_labels") == 1

        # ----- recompute geometry from live mm -----
        pitch_x_px = pitch_x_mm * px_per_mm
        pitch_y_px = pitch_y_mm * px_per_mm
        x_offset_px = x_offset_mm * px_per_mm
        y_offset_px = y_offset_mm * px_per_mm

        geom = GridGeom(
            rows=args.rows,
            cols=args.cols,
            pitch_x_px=float(pitch_x_px),
            pitch_y_px=float(pitch_y_px),
            grid_cx_px=float(wafer.cx + x_offset_px),
            grid_cy_px=float(wafer.cy + y_offset_px),
        )

        inside_margin_px = inside_margin_mm * px_per_mm

        params = (
            roi_half, std_thr, edge_thr, mean_thr_no_die, inside_margin_mm,
            min_neighbors, show_labels,
            # live geometry:
            round(pitch_x_mm, 4), round(pitch_y_mm, 4),
            round(x_offset_mm, 4), round(y_offset_mm, 4)
        )

        if params != last_params:
            grid = build_grid(
                gray=gray,
                wafer=wafer,
                geom=geom,
                inside_margin_px=inside_margin_px,
                roi_half=roi_half,
                std_thr=std_thr,
                edge_thr=edge_thr,
                mean_thr_no_die=mean_thr_no_die
            )
            grid = remove_isolated_dies(grid, min_neighbors=min_neighbors)

            overlay = make_overlay(
                base_bgr=base,
                ring=ring,
                wafer=wafer,
                geom=geom,
                grid=grid,
                px_per_mm=px_per_mm,
                show_labels=show_labels
            )

            # Create canvas with right panel
            h, w = overlay.shape[:2]
            canvas = np.zeros((h, w + panel_w, 3), dtype=np.uint8)
            canvas[:, :w] = overlay
            draw_right_panel(
                canvas, panel_x=w,
                selected_param=selected_param,
                pitch_x_mm=pitch_x_mm,
                pitch_y_mm=pitch_y_mm,
                x_offset_mm=x_offset_mm,
                y_offset_mm=y_offset_mm,
                step_fine=step_fine,
                step_coarse=step_coarse
            )

            last_params = params
            last_canvas = canvas
            last_grid = grid

        cv2.imshow(win, last_canvas)
        panel_img = make_param_panel_img(
            selected_param, pitch_x_mm, pitch_y_mm, x_offset_mm, y_offset_mm,
            step_fine, step_coarse,
            roi_half, std_thr, edge_thr, mean_thr_no_die,
            inside_margin_mm, min_neighbors, show_labels
        )
        cv2.imshow(panel_win, panel_img)

        # Use waitKeyEx for arrow keys
        key = cv2.waitKeyEx(30)

        if key in (27, ord('q'), ord('Q')):
            break

        # 🔽 ADD HERE
        if key in (ord('d'), ord('D')):
            pitch_x_mm = args.pitch_x_mm
            pitch_y_mm = args.pitch_y_mm
            x_offset_mm = args.x_offset_mm
            y_offset_mm = args.y_offset_mm

            cv2.setTrackbarPos("roi_half_px", win, 22)
            cv2.setTrackbarPos("std_thr_x10", win, 68)
            cv2.setTrackbarPos("edge_thr_x1000", win, 0)
            cv2.setTrackbarPos("mean_thr_no_die", win, 120)

            last_params = None
            print("[RESET] Defaults restored")

        # ----- select parameter -----
        if key == ord('1'):
            selected_param = 1
            last_params = None
        elif key == ord('2'):
            selected_param = 2
            last_params = None
        elif key == ord('3'):
            selected_param = 3
            last_params = None
        elif key == ord('4'):
            selected_param = 4
            last_params = None

        # ----- change fine step size quickly -----
        if key in (ord('z'), ord('Z')):
            step_fine = 0.001
            last_params = None
        elif key in (ord('x'), ord('X')):
            step_fine = 0.01
            last_params = None
        elif key in (ord('c'), ord('C')):
            step_fine = 0.05
            last_params = None

        # ----- arrow adjust -----
        ok, direction = is_arrow_key(key)
        if ok:
            if direction in ("up", "down"):
                delta = step_fine if direction == "up" else -step_fine
            else:
                delta = step_coarse if direction == "right" else -step_coarse

            if selected_param == 1:
                pitch_x_mm = max(0.001, pitch_x_mm + delta)
            elif selected_param == 2:
                pitch_y_mm = max(0.001, pitch_y_mm + delta)
            elif selected_param == 3:
                x_offset_mm = x_offset_mm + delta
            elif selected_param == 4:
                y_offset_mm = y_offset_mm + delta

            last_params = None  # force refresh

        # ----- print current params -----
        if key in (ord('p'), ord('P')):
            print("\n[PARAMS]")
            print(f"pitch_x_mm={pitch_x_mm:.4f}  pitch_y_mm={pitch_y_mm:.4f}")
            print(f"x_offset_mm={x_offset_mm:.4f}  y_offset_mm={y_offset_mm:.4f}")
            print(f"roi_half_px={roi_half}  std_thr={std_thr:.3f}  edge_thr={edge_thr:.3f}  mean_thr_no_die={mean_thr_no_die}")
            print(f"inside_margin_mm={inside_margin_mm:.2f}  min_neighbors={min_neighbors}  show_labels={show_labels}")

        # ----- Save overlay only -----
        if key in (ord('s'), ord('S')):
            ensure_dir(os.path.join(args.out_dir, "dummy.txt"))
            out_png = os.path.join(args.out_dir, f"{args.base_name}_overlay.png")
            cv2.imwrite(out_png, last_canvas)
            print(f"[SAVED] {out_png}")

        # ----- RUN export -----
        if key in (ord('r'), ord('R')):
            ensure_dir(os.path.join(args.out_dir, "dummy.txt"))

            grid_path = os.path.join(args.out_dir, f"{args.base_name}_grid.txt")
            csv_path = os.path.join(args.out_dir, f"{args.base_name}_waypoints.csv")
            yaml_path = os.path.join(args.out_dir, f"{args.base_name}_waypoints.yaml")
            png_path = os.path.join(args.out_dir, f"{args.base_name}_debug.png")
            params_path = os.path.join(args.out_dir, f"{args.base_name}_params.txt")

            # Save debug image (with panel)
            cv2.imwrite(png_path, last_canvas)

            # Save grid
            save_grid_text(last_grid, grid_path)

            # Rebuild geom (current)
            pitch_x_px = pitch_x_mm * px_per_mm
            pitch_y_px = pitch_y_mm * px_per_mm
            x_offset_px = x_offset_mm * px_per_mm
            y_offset_px = y_offset_mm * px_per_mm
            geom_now = GridGeom(
                rows=args.rows, cols=args.cols,
                pitch_x_px=float(pitch_x_px),
                pitch_y_px=float(pitch_y_px),
                grid_cx_px=float(wafer.cx + x_offset_px),
                grid_cy_px=float(wafer.cy + y_offset_px),
            )

            waypoints = grid_to_waypoints(
                grid=last_grid,
                geom=geom_now,
                wafer=wafer,
                px_per_mm=px_per_mm,
                home_x_mm=args.home_x_mm,
                home_y_mm=args.home_y_mm,
                y_down_is_positive=(not args.y_up)
            )
            save_waypoints_csv(waypoints, csv_path)
            save_waypoints_yaml(waypoints, yaml_path)

            params_dict = {
                "pitch_x_mm": pitch_x_mm,
                "pitch_y_mm": pitch_y_mm,
                "x_offset_mm": x_offset_mm,
                "y_offset_mm": y_offset_mm,
                "roi_half_px": roi_half,
                "std_thr": std_thr,
                "edge_thr": edge_thr,
                "mean_thr_no_die": mean_thr_no_die,
                "inside_margin_mm": inside_margin_mm,
                "min_neighbors": min_neighbors,
                "show_labels": show_labels,
                "px_per_mm": px_per_mm,
            }
            save_params_txt(params_path, params_dict)

            print(f"[RUN] Exported:")
            print(f"  {png_path}")
            print(f"  {grid_path}")
            print(f"  {csv_path}  (count={len(waypoints)})")
            if yaml is not None:
                print(f"  {yaml_path}")
            print(f"  {params_path}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
Can 