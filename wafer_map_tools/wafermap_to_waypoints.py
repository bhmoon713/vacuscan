#!/usr/bin/env python3
import argparse
import math
import os
import yaml
import numpy as np

try:
    import cv2
except ImportError:
    raise SystemExit("Missing OpenCV. Install: sudo apt install python3-opencv")

def classify_color(bgr):
    """
    Return one of: 'GOOD', 'REJECT', 'EMPTY'
    Based on your image: yellow squares = GOOD, gray squares = REJECT.
    """
    hsv = cv2.cvtColor(np.uint8([[bgr]]), cv2.COLOR_BGR2HSV)[0, 0]
    h, s, v = int(hsv[0]), int(hsv[1]), int(hsv[2])

    # Yellow: high saturation, hue around ~20-40 in OpenCV HSV
    is_yellow = (15 <= h <= 45) and (s >= 80) and (v >= 80)

    # Gray: low saturation, mid brightness
    is_gray = (s <= 40) and (50 <= v <= 220)

    if is_yellow:
        return "GOOD"
    if is_gray:
        return "REJECT"
    return "EMPTY"

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True, help="Input wafer map image (png/jpg).")
    ap.add_argument("--out_dir", default="out_map", help="Output folder.")
    ap.add_argument("--pitch_mm", type=float, default=5.0, help="Die pitch (mm).")
    ap.add_argument("--route_name", default="scan_good_dies", help="Waypoint route name.")
    ap.add_argument("--min_area", type=float, default=200.0, help="Min contour area to keep.")
    ap.add_argument("--max_area", type=float, default=10000.0, help="Max contour area to keep.")
    ap.add_argument("--debug", action="store_true", help="Write debug overlay image.")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"Could not read image: {args.image}")

    # --- Preprocess ---
    # Detect square-like dies by finding strong edges and contours.
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # Close gaps
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # For each contour, get bounding box, filter by size/aspect
    dies = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < args.min_area or area > args.max_area:
            continue

        x, y, w, h = cv2.boundingRect(c)
        if w < 8 or h < 8:
            continue

        aspect = w / float(h)
        if not (0.7 <= aspect <= 1.3):
            continue

        # Sample color at the center of bbox to classify
        cx = x + w // 2
        cy = y + h // 2
        bgr = img[cy, cx].tolist()
        status = classify_color(bgr)

        # We only care about GOOD and REJECT dies (EMPTY are usually background lines)
        if status in ("GOOD", "REJECT"):
            dies.append({
                "cx": cx, "cy": cy,
                "bbox": (x, y, w, h),
                "status": status
            })

    if len(dies) < 20:
        print(f"[WARN] Only detected {len(dies)} dies. You may need to tune --min_area/--max_area.")
    else:
        print(f"[OK] Detected {len(dies)} dies (GOOD/REJECT).")

    # --- Convert pixel centers to grid (row/col) ---
    # Strategy:
    #   Cluster x-centers into columns and y-centers into rows using median spacing.
    xs = np.array(sorted([d["cx"] for d in dies]))
    ys = np.array(sorted([d["cy"] for d in dies]))

    def estimate_step(vals):
        diffs = np.diff(vals)
        diffs = diffs[(diffs > 2) & (diffs < 200)]  # ignore tiny/noise and huge jumps
        if len(diffs) == 0:
            return None
        return float(np.median(diffs))

    step_x = estimate_step(xs)
    step_y = estimate_step(ys)

    if step_x is None or step_y is None:
        raise SystemExit("Could not estimate grid spacing. Try adjusting contour filters or use a cleaner image.")

    # Define grid origin as min x/y
    min_x = min(d["cx"] for d in dies)
    min_y = min(d["cy"] for d in dies)

    # Map each die to integer grid indices using rounding by step
    for d in dies:
        d["col"] = int(round((d["cx"] - min_x) / step_x))
        d["row"] = int(round((d["cy"] - min_y) / step_y))

    # Normalize rows/cols so they start at 0..N
    cols = [d["col"] for d in dies]
    rows = [d["row"] for d in dies]
    cmin, rmin = min(cols), min(rows)
    for d in dies:
        d["col"] -= cmin
        d["row"] -= rmin

    ncols = max(d["col"] for d in dies) + 1
    nrows = max(d["row"] for d in dies) + 1

    # Build a grid with EMPTY default
    grid = [["EMPTY" for _ in range(ncols)] for __ in range(nrows)]
    for d in dies:
        grid[d["row"]][d["col"]] = d["status"]

    # --- Save a simple CSV-like map for sanity ---
    map_txt = os.path.join(args.out_dir, "die_map.txt")
    with open(map_txt, "w") as f:
        f.write(f"# rows={nrows} cols={ncols}\n")
        f.write("# legend: G=GOOD R=REJECT .=EMPTY\n")
        for r in range(nrows):
            line = ""
            for c in range(ncols):
                s = grid[r][c]
                line += "G" if s == "GOOD" else ("R" if s == "REJECT" else ".")
            f.write(line + "\n")
    print(f"[OK] Wrote: {map_txt}")

    # --- Generate waypoint YAML (GOOD dies only) ---
    # Choose center of grid as origin for (0,0).
    col_center = (ncols - 1) / 2.0
    row_center = (nrows - 1) / 2.0

    points = []
    for r in range(nrows):
        for c in range(ncols):
            if grid[r][c] != "GOOD":
                continue
            x_mm = (c - col_center) * args.pitch_mm
            y_mm = (row_center - r) * args.pitch_mm  # flip so row-up = +Y
            points.append({"x": float(x_mm), "y": float(y_mm)})

    # Optional: serpentine ordering (reduces travel)
    # Re-order points row by row
    good_by_row = {}
    for p in points:
        # Convert y back to row approx for sorting (invert)
        r_est = int(round(row_center - (p["y"] / args.pitch_mm)))
        good_by_row.setdefault(r_est, []).append(p)

    ordered = []
    for rr in sorted(good_by_row.keys()):
        row_pts = sorted(good_by_row[rr], key=lambda q: q["x"])
        if rr % 2 == 1:
            row_pts = list(reversed(row_pts))
        ordered.extend(row_pts)

    out_yaml = os.path.join(args.out_dir, "waypoints.yaml")
    data = {
        "routes": {
            args.route_name: {
                "points": ordered
            }
        }
    }
    with open(out_yaml, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print(f"[OK] Wrote: {out_yaml} (GOOD dies: {len(ordered)})")

    # --- Optional “STIF-like” text output (TEMPLATE) ---
    # This is a generic placeholder. Your real STIF may need different keywords.
    out_stif = os.path.join(args.out_dir, "map.stif.txt")
    with open(out_stif, "w") as f:
        f.write("STIF\n")
        f.write(f"ROWS {nrows}\n")
        f.write(f"COLS {ncols}\n")
        f.write("ORIGIN CENTER\n")
        f.write("BINS GOOD=1 REJECT=255 EMPTY=0\n")
        f.write("MAP\n")
        for r in range(nrows):
            row_bins = []
            for c in range(ncols):
                s = grid[r][c]
                b = 1 if s == "GOOD" else (255 if s == "REJECT" else 0)
                row_bins.append(str(b))
            f.write(" ".join(row_bins) + "\n")
        f.write("END\n")
    print(f"[OK] Wrote (template): {out_stif}")

    # --- Debug overlay ---
    if args.debug:
        dbg = img.copy()
        for d in dies:
            x, y, w, h = d["bbox"]
            color = (0, 255, 0) if d["status"] == "GOOD" else (100, 100, 100)
            cv2.rectangle(dbg, (x, y), (x+w, y+h), color, 2)
            cv2.putText(dbg, f"{d['row']},{d['col']}", (x, y-3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)
        out_dbg = os.path.join(args.out_dir, "debug_overlay.png")
        cv2.imwrite(out_dbg, dbg)
        print(f"[OK] Wrote: {out_dbg}")

if __name__ == "__main__":
    main()
