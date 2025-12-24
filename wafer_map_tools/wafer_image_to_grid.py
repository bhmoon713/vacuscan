#!/usr/bin/env python3
"""
Remnant wafer grid + waypoint generator (robust: detect RING inner circle, derive wafer)

Known constants (your setup):
  - ring inner diameter = 350mm (radius 175mm)
  - wafer diameter      = 300mm (radius 150mm)
  - wafer is near center and concentric with ring (small errors)

Pipeline:
  1) Detect ring inner circle using HoughCircles on an outer-band mask
  2) px_per_mm = ring_r_px / 175
  3) wafer circle = same center, radius = ring_r_px * (300/350)
  4) Convert pitch/offset mm -> px using px_per_mm
  5) Build grid and classify dies: 0 (outside), 1 (remaining), 4 (missing)
  6) Output: grid txt, waypoints csv/yaml, debug overlay

Install:
  pip install opencv-python numpy pyyaml   (pyyaml optional)
"""

import argparse
import csv
import math
import os
from dataclasses import dataclass
from typing import List, Tuple, Optional

import cv2
import numpy as np

try:
    import yaml
except Exception:
    yaml = None


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


# ---------------------------- Ring detection (robust) ----------------------------

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
    Detect ring inner circle using HoughCircles on outer-band-only image.
    Adds strong constraints:
      - radius must be in [r_min_frac, r_max_frac] * min(h,w)
      - center must be close to image center (wafer always centered)
    """
    g = gray if gray.ndim == 2 else cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    h, w = g.shape[:2]
    cx0, cy0 = w / 2.0, h / 2.0

    mask = outer_band_mask(h, w, band_ratio)
    roi = cv2.bitwise_and(g, g, mask=mask)

    blur = cv2.GaussianBlur(roi, (9, 9), 1.5)

    minR = int(min(h, w) * r_min_frac)
    maxR = int(min(h, w) * r_max_frac)

    circles = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=dp,
        minDist=min(h, w) * 0.35,
        param1=param1,
        param2=param2,
        minRadius=minR,
        maxRadius=maxR,
    )

    if circles is None:
        raise RuntimeError(
            "Ring HoughCircles failed. Try: lower --ring_param2 (e.g. 22) "
            "or adjust --ring_band_ratio / radius fractions."
        )

    cand = np.squeeze(circles).astype(np.float32)
    if cand.ndim == 1:
        cand = cand[None, :]

    # Filter by center closeness (wafer centered)
    tol = min(h, w) * center_tol_frac
    good = []
    for (cx, cy, r) in cand:
        if abs(cx - cx0) <= tol and abs(cy - cy0) <= tol:
            good.append((cx, cy, r))

    if not good:
        # fallback: allow all, but still pick best by score
        good = [(float(cx), float(cy), float(r)) for (cx, cy, r) in cand]
    else:
        good = [(float(cx), float(cy), float(r)) for (cx, cy, r) in good]

    # Score: prefer large radius + center closeness
    def score(c):
        cx, cy, r = c
        dc = math.hypot(cx - cx0, cy - cy0)
        return (r * 2.0) - dc

    best = max(good, key=score)
    return Circle(cx=best[0], cy=best[1], r=best[2])


# ---------------------------- Classification (0/1/4) ----------------------------

def classify_die(gray: np.ndarray,
                 x: float, y: float,
                 roi_half: int,
                 mean_thr_no_die: float,
                 std_thr_texture: float,
                 edge_thr: float) -> int:
    """
    Returns:
      1 = remnant die (WAYPOINT)
      4 = no die / empty area (IGNORE)
    """

    xi, yi = int(round(x)), int(round(y))
    roi = clamp_roi(gray, xi - roi_half, yi - roi_half,
                          xi + roi_half, yi + roi_half)
    if roi.size == 0:
        return 4

    mean_val = float(np.mean(roi))
    std_val  = float(np.std(roi))

    # Edge density
    blur = cv2.GaussianBlur(roi, (5, 5), 0)
    edges = cv2.Canny(blur, 40, 120)
    edge_density = float(np.mean(edges > 0))  # 0..1

    # ---- RULES ----

    # 1) Extremely bright region = NO DIE (thin red background)
    if mean_val > mean_thr_no_die + 15:
        return 4

    # 2) Bright and flat = NO DIE
    if mean_val > mean_thr_no_die and std_val < std_thr_texture:
        return 4

    # 3) Textured or edged = REMNANT DIE (dark red or green)
    if std_val >= std_thr_texture or edge_density >= edge_thr:
        return 1

    # 4) Dark region (even if flat) = REMNANT DIE
    if mean_val < mean_thr_no_die:
        return 1

    # 5) Default fallback = NO DIE
    return 4


def build_grid(gray: np.ndarray,
               wafer: Circle,
               geom: GridGeom,
               inside_margin_px: float,
               roi_half: int,
               mean_thr_no_die: float,   # ✅ ADD
               std_thr: float,
               edge_thr: float) -> np.ndarray:
    grid = np.zeros((geom.rows, geom.cols), dtype=np.int32)

    # conservative half-diagonal to avoid classifying cells straddling the edge
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
                mean_thr_no_die=mean_thr_no_die,
                std_thr_texture=std_thr,
                edge_thr=edge_thr
            )


    return grid


# ---------------------------- Outputs ----------------------------

def save_grid_text(grid: np.ndarray, path: str) -> None:
    ensure_dir(path)
    with open(path, "w", newline="") as f:
        for r in range(grid.shape[0]):
            f.write(",".join(str(int(v)) for v in grid[r]) + ",\n")  # trailing comma


def grid_to_waypoints(grid: np.ndarray,
                      geom: GridGeom,
                      wafer: Circle,
                      px_per_mm: float,
                      home_x_mm: float,
                      home_y_mm: float,
                      y_down_is_positive: bool = True) -> List[dict]:
    wps = []
    for r in range(grid.shape[0]):
        for c in range(grid.shape[1]):
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


def save_waypoints_yaml(waypoints: List[dict], path: str) -> None:
    if yaml is None:
        print("[WARN] PyYAML not installed -> skipping YAML output. Install: pip install pyyaml")
        return
    ensure_dir(path)
    with open(path, "w") as f:
        yaml.safe_dump({"waypoints": waypoints}, f, sort_keys=False)


def draw_grid_lines(img: np.ndarray, geom: GridGeom, color=(0, 0, 0), thickness=1) -> None:
    h, w = img.shape[:2]

    k_min = int(math.floor((0 - geom.grid_cx_px) / geom.pitch_x_px)) - 2
    k_max = int(math.ceil((w - geom.grid_cx_px) / geom.pitch_x_px)) + 2
    for k in range(k_min, k_max + 1):
        x = int(round(geom.grid_cx_px + k * geom.pitch_x_px))
        if 0 <= x < w:
            cv2.line(img, (x, 0), (x, h - 1), color, thickness, cv2.LINE_AA)

    k_min = int(math.floor((0 - geom.grid_cy_px) / geom.pitch_y_px)) - 2
    k_max = int(math.ceil((h - geom.grid_cy_px) / geom.pitch_y_px)) + 2
    for k in range(k_min, k_max + 1):
        y = int(round(geom.grid_cy_px + k * geom.pitch_y_px))
        if 0 <= y < h:
            cv2.line(img, (0, y), (w - 1, y), color, thickness, cv2.LINE_AA)


def make_debug_image(base_bgr: np.ndarray,
                     ring: Circle,
                     wafer: Circle,
                     geom: GridGeom,
                     grid: np.ndarray,
                     px_per_mm: float,
                     ring_d_mm: float,
                     wafer_d_mm: float,
                     pitch_x_mm: float,
                     pitch_y_mm: float,
                     x_offset_mm: float,
                     y_offset_mm: float,
                     draw_labels: bool = True) -> np.ndarray:
    img = base_bgr.copy()
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # ring inner (purple) + wafer (yellow)
    cv2.circle(img, (int(round(ring.cx)), int(round(ring.cy))), int(round(ring.r)), (255, 0, 255), 2)
    cv2.circle(img, (int(round(wafer.cx)), int(round(wafer.cy))), int(round(wafer.r)), (0, 255, 255), 2)

    # grid center (blue)
    cv2.circle(img, (int(round(geom.grid_cx_px)), int(round(geom.grid_cy_px))), 4, (255, 0, 0), -1)

    draw_grid_lines(img, geom, color=(0, 0, 0), thickness=1)

    # overlay cell colors
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
                color = (0, 255, 0)
            else:
                color = (0, 0, 255)

            x0, y0 = max(0, xi - box), max(0, yi - box)
            x1, y1 = min(img.shape[1] - 1, xi + box), min(img.shape[0] - 1, yi + box)
            cv2.rectangle(overlay, (x0, y0), (x1, y1), color, thickness=-1)

            if draw_labels and v != 0:
                cv2.putText(overlay, f"{r},{c}", (xi - box, yi),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 0), 1, cv2.LINE_AA)

    out = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)

    info = [
        f"ring inner: {ring_d_mm:.1f}mm -> r_px={ring.r:.1f} => px_per_mm={px_per_mm:.4f}",
        f"wafer: {wafer_d_mm:.1f}mm (derived) -> r_px={wafer.r:.1f}",
        f"pitch: x={pitch_x_mm:.3f}mm ({geom.pitch_x_px:.2f}px)  y={pitch_y_mm:.3f}mm ({geom.pitch_y_px:.2f}px)",
        f"offset(center): x={x_offset_mm:.2f}mm  y={y_offset_mm:.2f}mm  (+x right, +y down)",
        f"center: ring=({ring.cx:.1f},{ring.cy:.1f}) wafer=({wafer.cx:.1f},{wafer.cy:.1f}) grid=({geom.grid_cx_px:.1f},{geom.grid_cy_px:.1f})",
    ]
    for i, line in enumerate(info):
        cv2.putText(out, line, (10, 25 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, line, (10, 25 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (255, 255, 255), 1, cv2.LINE_AA)

    return out


# ---------------------------- Main ----------------------------

def main():
    ap = argparse.ArgumentParser()

    ap.add_argument("--image", required=True)
    ap.add_argument("--rows", type=int, required=True)
    ap.add_argument("--cols", type=int, required=True)

    ap.add_argument("--pitch_x_mm", type=float, required=True)
    ap.add_argument("--pitch_y_mm", type=float, required=True)

    ap.add_argument("--x_offset_mm", type=float, default=0.0,
                    help="grid center offset from wafer center (mm), +x right")
    ap.add_argument("--y_offset_mm", type=float, default=0.0,
                    help="grid center offset from wafer center (mm), +y down")

    ap.add_argument("--ring_inner_d_mm", type=float, default=350.0)
    ap.add_argument("--wafer_d_mm", type=float, default=300.0)

    # ring detection tuning (Hough)
    ap.add_argument("--ring_band_ratio", type=float, default=0.22)
    ap.add_argument("--ring_param2", type=int, default=28,
                    help="Hough param2 (lower -> easier detection, more false circles)")
    ap.add_argument("--ring_r_min_frac", type=float, default=0.42)
    ap.add_argument("--ring_r_max_frac", type=float, default=0.55)

    # classification tuning
    ap.add_argument("--inside_margin_mm", type=float, default=0.0)
    ap.add_argument("--roi_half_px", type=int, default=30)
    ap.add_argument("--std_thr", type=float, default=6.0)
    ap.add_argument("--edge_thr", type=float, default=0.03)
    ap.add_argument("--mean_thr_no_die", type=float, default=100.0)


    # waypoint reference
    ap.add_argument("--home_x_mm", type=float, default=0.0)
    ap.add_argument("--home_y_mm", type=float, default=0.0)
    ap.add_argument("--y_up", action="store_true")

    # outputs
    ap.add_argument("--out_grid", default="wafer_grid.txt")
    ap.add_argument("--out_csv", default="waypoints.csv")
    ap.add_argument("--out_yaml", default="waypoints.yaml")
    ap.add_argument("--out_debug", default="debug_overlay.png")
    ap.add_argument("--no_labels", action="store_true")

    args = ap.parse_args()

    base = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if base is None:
        raise FileNotFoundError(args.image)
    gray = cv2.cvtColor(base, cv2.COLOR_BGR2GRAY)

    # 1) ring detection (Hough, outer band)
    ring = detect_ring_inner_circle_hough(
        gray,
        band_ratio=args.ring_band_ratio,
        param2=args.ring_param2,
        r_min_frac=args.ring_r_min_frac,
        r_max_frac=args.ring_r_max_frac,
    )

    ring_r_mm = args.ring_inner_d_mm / 2.0
    px_per_mm = ring.r / ring_r_mm

    # 2) wafer derived from ring
    wafer_scale = args.wafer_d_mm / args.ring_inner_d_mm
    wafer = Circle(cx=ring.cx, cy=ring.cy, r=ring.r * wafer_scale)

    # 3) geometry mm -> px
    pitch_x_px = args.pitch_x_mm * px_per_mm
    pitch_y_px = args.pitch_y_mm * px_per_mm
    x_offset_px = args.x_offset_mm * px_per_mm
    y_offset_px = args.y_offset_mm * px_per_mm

    grid_cx = wafer.cx + x_offset_px
    grid_cy = wafer.cy + y_offset_px

    geom = GridGeom(
        rows=args.rows,
        cols=args.cols,
        pitch_x_px=float(pitch_x_px),
        pitch_y_px=float(pitch_y_px),
        grid_cx_px=float(grid_cx),
        grid_cy_px=float(grid_cy),
    )

    inside_margin_px = args.inside_margin_mm * px_per_mm

    # 4) build grid
    grid = build_grid(
        gray=gray,
        wafer=wafer,
        geom=geom,
        inside_margin_px=inside_margin_px,
        roi_half=args.roi_half_px,
        mean_thr_no_die=args.mean_thr_no_die,  # ✅ PASS IT
        std_thr=args.std_thr,
        edge_thr=args.edge_thr,
    )

    # 5) outputs
    save_grid_text(grid, args.out_grid)
    waypoints = grid_to_waypoints(
        grid=grid,
        geom=geom,
        wafer=wafer,
        px_per_mm=px_per_mm,
        home_x_mm=args.home_x_mm,
        home_y_mm=args.home_y_mm,
        y_down_is_positive=(not args.y_up),
    )
    save_waypoints_csv(waypoints, args.out_csv)
    save_waypoints_yaml(waypoints, args.out_yaml)

    dbg = make_debug_image(
        base_bgr=base,
        ring=ring,
        wafer=wafer,
        geom=geom,
        grid=grid,
        px_per_mm=px_per_mm,
        ring_d_mm=args.ring_inner_d_mm,
        wafer_d_mm=args.wafer_d_mm,
        pitch_x_mm=args.pitch_x_mm,
        pitch_y_mm=args.pitch_y_mm,
        x_offset_mm=args.x_offset_mm,
        y_offset_mm=args.y_offset_mm,
        draw_labels=(not args.no_labels),
    )
    ensure_dir(args.out_debug)
    cv2.imwrite(args.out_debug, dbg)

    print(f"[OK] ring: cx={ring.cx:.2f}, cy={ring.cy:.2f}, r_px={ring.r:.2f}, px_per_mm={px_per_mm:.5f}")
    print(f"[OK] wafer (derived): r_px={wafer.r:.2f}")
    print(f"[OK] pitch_px: x={geom.pitch_x_px:.2f}, y={geom.pitch_y_px:.2f}")
    print(f"[OK] saved: {args.out_grid}, {args.out_csv}, {args.out_yaml}, {args.out_debug}")


if __name__ == "__main__":
    main()
