"""Measure the gate's inner opening in reconstruction units and derive metric scale.

Method:
  1. For each chosen image, start from rough hand-picked corners of the banner hole,
     refine each edge by Canny+Hough line fitting in a band around the seed edge,
     and intersect adjacent lines -> sub-pixel corners. Annotated overlays are saved
     for visual verification.
  2. Undistort corners (OPENCV model), triangulate each corner across views by DLT
     using transforms.json poses (same frame as the exported splat - verified by
     projection earlier).
  3. Opening width/height in splat units -> scale = KNOWN_OPENING / measured.

Run:  ~/miniconda3/envs/py312/bin/python measure_gate_opening.py
"""
import json
import os

import cv2
import numpy as np

DATA = os.path.expanduser(os.environ.get("GATE_DATASET_DIR", "~/Downloads/IMG_4701"))
OUT = os.path.join(DATA, "measure_debug")
os.makedirs(OUT, exist_ok=True)

KNOWN_OPENING = 1.50  # metres, user-provided inner opening of the gate

# Rough banner-hole corners (TL, TR, BR, BL) picked visually per image.
SEEDS = {
    "images/00005.jpg": [(383, 450), (643, 477), (650, 806), (408, 837)],
    "images/00028.jpg": [(299, 428), (577, 448), (588, 800), (303, 793)],
    "images/00095.jpg": [(332, 422), (674, 458), (676, 812), (334, 838)],
}

with open(os.path.join(DATA, "transforms.json")) as fp:
    meta = json.load(fp)
frames = {f["file_path"]: f for f in meta["frames"]}
K = np.array([[meta["fl_x"], 0, meta["cx"]], [0, meta["fl_y"], meta["cy"]], [0, 0, 1]])
dist = np.array([meta["k1"], meta["k2"], meta["p1"], meta["p2"]])


def refine_edges(img_path, seeds):
    """Fit the 4 hole edges as lines, return 4 refined corners (TL,TR,BR,BL).

    Edges are taken from the BLUE BANNER boundary (color mask), not raw grayscale
    gradients. The wooden frame has depth: from an oblique view the strongest
    grayscale edge on the far side of the hole is the frame's BACK edge, which
    inflates the measured opening by the frame depth (observed: +0.3 units on the
    right side with all capture views left of the gate). The blue/not-blue boundary
    is the banner cutout on the FRONT face - the actual aperture.
    """
    img = cv2.imread(os.path.join(DATA, img_path))
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # saturated blue banner: hue ~100-135, moderate+ saturation
    blue = cv2.inRange(hsv, (95, 60, 30), (140, 255, 255))
    blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    edges_img = cv2.Canny(blue, 40, 120)
    seeds = np.array(seeds, float)

    lines = []
    overlay = img.copy()
    for i in range(4):
        p0, p1 = seeds[i], seeds[(i + 1) % 4]
        seed_dir = (p1 - p0) / np.linalg.norm(p1 - p0)
        seed_ang = np.degrees(np.arctan2(seed_dir[1], seed_dir[0]))
        # mask: a band around the seed segment (shrunk 12% from each corner so the
        # fit is not polluted by the perpendicular edge at the corners)
        mask = np.zeros(img.shape[:2], np.uint8)
        a = (p0 + 0.12 * (p1 - p0)).astype(int)
        b = (p1 - 0.12 * (p1 - p0)).astype(int)
        cv2.line(mask, tuple(a), tuple(b), 255, 25)
        band = cv2.bitwise_and(edges_img, mask)

        segs = cv2.HoughLinesP(band, 1, np.pi / 360, threshold=30,
                               minLineLength=25, maxLineGap=8)
        pts, wts = [], []
        if segs is not None:
            for x1, y1, x2, y2 in segs[:, 0]:
                ang = np.degrees(np.arctan2(y2 - y1, x2 - x1))
                dif = abs((ang - seed_ang + 90) % 180 - 90)
                if dif < 8:  # keep segments parallel to the seed edge
                    L = np.hypot(x2 - x1, y2 - y1)
                    pts += [(x1, y1), (x2, y2)]
                    wts += [L, L]
        if len(pts) < 4:
            print(f"  WARNING {img_path} edge {i}: Hough found nothing, keeping seed line")
            line_pt, line_dir = p0, seed_dir
        else:
            P = np.array(pts, float)
            w = np.array(wts, float)[:, None]
            c = (P * w).sum(0) / w.sum()
            Pc = (P - c) * np.sqrt(w)
            _, _, Vt = np.linalg.svd(Pc, full_matrices=False)
            line_pt, line_dir = c, Vt[0]
        lines.append((line_pt, line_dir))
        q0 = (line_pt - 600 * line_dir).astype(int)
        q1 = (line_pt + 600 * line_dir).astype(int)
        cv2.line(overlay, tuple(q0), tuple(q1), (0, 255, 0), 1)

    corners = []
    for i in range(4):
        (pA, dA), (pB, dB) = lines[i - 1], lines[i]  # edge into corner i, edge out
        M = np.stack([dA, -dB], 1)
        t = np.linalg.lstsq(M, pB - pA, rcond=None)[0]
        c = pA + t[0] * dA
        corners.append(c)
        cv2.circle(overlay, tuple(c.astype(int)), 5, (0, 0, 255), 2)
        cv2.circle(overlay, tuple(seeds[i].astype(int)), 3, (255, 200, 0), 1)
    cv2.imwrite(os.path.join(OUT, os.path.basename(img_path).replace(".jpg", "_edges.jpg")), overlay)
    return np.array(corners)


def projection_matrix(frame):
    """OpenCV projection P = K [R|t] from the OpenGL c2w in transforms.json."""
    c2w = np.array(frame["transform_matrix"], float)
    c2w_cv = c2w.copy()
    c2w_cv[:3, 1] *= -1  # GL -> CV: flip Y and Z camera axes
    c2w_cv[:3, 2] *= -1
    w2c = np.linalg.inv(c2w_cv)
    return K @ w2c[:3, :]


all_corners, Ps = {}, {}
for path, seeds in SEEDS.items():
    print(f"refining {path} ...")
    c = refine_edges(path, seeds)
    # undistort to ideal pinhole pixel coords (P = K [R|t] assumes no distortion)
    und = cv2.undistortPoints(c.reshape(-1, 1, 2), K, dist, P=K).reshape(-1, 2)
    all_corners[path] = und
    Ps[path] = projection_matrix(frames[path])
    print(f"  corners (undistorted px): {np.round(und, 1).tolist()}")

# DLT triangulation of each corner across all views
names = ["TL", "TR", "BR", "BL"]
X = []
for ci in range(4):
    A = []
    for path in SEEDS:
        u, v = all_corners[path][ci]
        P = Ps[path]
        A.append(u * P[2] - P[0])
        A.append(v * P[2] - P[1])
    _, _, Vt = np.linalg.svd(np.array(A))
    Xh = Vt[-1]
    X.append(Xh[:3] / Xh[3])
X = np.array(X)

print("\ntriangulated corners (splat units):")
for n, x in zip(names, X):
    print(f"  {n}: {np.round(x, 4).tolist()}")

# reprojection errors
print("\nreprojection error (px):")
for path in SEEDS:
    P = Ps[path]
    proj = (P @ np.hstack([X, np.ones((4, 1))]).T).T
    proj = proj[:, :2] / proj[:, 2:3]
    err = np.linalg.norm(proj - all_corners[path], axis=1)
    print(f"  {path}: {np.round(err, 2).tolist()}")

TL, TR, BR, BL = X
sides = {
    "top   (TL-TR)": np.linalg.norm(TR - TL),
    "bottom(BL-BR)": np.linalg.norm(BR - BL),
    "left  (TL-BL)": np.linalg.norm(BL - TL),
    "right (TR-BR)": np.linalg.norm(BR - TR),
}
print("\nopening sides (splat units):")
for k, v in sides.items():
    print(f"  {k}: {v:.4f}")
d1, d2 = np.linalg.norm(BR - TL), np.linalg.norm(BL - TR)
print(f"  diag1: {d1:.4f}  diag2: {d2:.4f}  (square if equal: ratio {d1/d2:.4f})")

mean_side = np.mean(list(sides.values()))
scale = KNOWN_OPENING / mean_side
print(f"\nmean side  : {mean_side:.4f} splat units")
print(f"SCALE      : {scale:.6f} m / splat-unit   (known opening {KNOWN_OPENING} m)")
print(f"per-side scale spread: "
      f"{KNOWN_OPENING/max(sides.values()):.4f} .. {KNOWN_OPENING/min(sides.values()):.4f}")

result = {
    "known_opening_m": KNOWN_OPENING,
    "corners_splat": {n: x.tolist() for n, x in zip(names, X)},
    "sides_splat": {k: float(v) for k, v in sides.items()},
    "scale_m_per_unit": float(scale),
}
with open(os.path.join(DATA, "gate_scale.json"), "w") as fp:
    json.dump(result, fp, indent=2)
print(f"\nwrote {os.path.join(DATA, 'gate_scale.json')}")
