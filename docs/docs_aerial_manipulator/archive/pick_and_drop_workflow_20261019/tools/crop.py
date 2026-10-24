#!/usr/bin/env python3
"""Turn snapshots.sh's raw Pick & Drop tab grabs into the workflow page's pictures (2026-10-19).

    PYTHONNOUSERSITE=1 /usr/bin/python3 -I crop.py <grab dir> <page dir> [<isaac frame dir>]

Every highlight box is computed from the widget rectangles the harness saved beside each grab
(<name>.json), so a layout change in the station moves the boxes with it. With an Isaac frame
dir (isaac_1.png / isaac_75.png, the left half of the 10-18 pick-and-drop video at 1 s and 75 s),
the two scene pictures are cropped too.
"""
import json
import os
import sys

import cv2

RED = (79, 82, 232)          # BGR, the doc's highlight red
GRAB, PAGE = sys.argv[1], sys.argv[2]
ISAAC = sys.argv[3] if len(sys.argv) > 3 else None
os.makedirs(PAGE, exist_ok=True)


def load(name):
    return cv2.imread(os.path.join(GRAB, name + ".png")), json.load(open(os.path.join(GRAB, name + ".json")))


def find(doc, cls, text):
    for w in doc["widgets"]:
        if w["class"] == cls and w["text"] == text:
            return w
    raise SystemExit(f"no {cls} {text!r}")


def group(doc, prefix):
    for w in doc["widgets"]:
        if w["class"] == "QGroupBox" and w["text"].startswith(prefix):
            return w
    raise SystemExit(f"no group {prefix!r}")


def union(ws, pad=4, vpad=1):
    # vpad small: the rows sit 2 px apart from the column header above them
    x0 = min(w["x"] for w in ws) - pad
    y0 = min(w["y"] for w in ws) - vpad
    x1 = max(w["x"] + w["w"] for w in ws) + pad
    y1 = max(w["y"] + w["h"] for w in ws) + vpad
    return x0, y0, x1, y1


def row(doc, label, x_max):
    """Every widget on the row of the point label `label`, left of x_max (the settings column)."""
    lab = find(doc, "QLabel", label)
    cy = lab["y"] + lab["h"] / 2
    ws = [w for w in doc["widgets"]
          if w["class"] in ("QLabel", "QPushButton", "QDoubleSpinBox")
          and abs(w["y"] + w["h"] / 2 - cy) < 6 and w["x"] + w["w"] <= x_max and w["w"] < 400]
    return union(ws)


def box(im, r, t=2):
    cv2.rectangle(im, (int(r[0]), int(r[1])), (int(r[2]), int(r[3])), RED, t, lineType=cv2.LINE_AA)


def calib(name, out, boxes):
    """The Calibration And Points group, up to the settings column's unit labels, with red boxes."""
    im, doc = load(name)
    g = group(doc, "Calibration")
    units = [w for w in doc["widgets"] if w["class"] == "QLabel" and w["text"] == "m"]
    x1 = max(w["x"] + w["w"] for w in units) + 16
    for b in boxes(doc):
        box(im, b)
    cv2.imwrite(os.path.join(PAGE, out), im[0:g["y"] + g["h"] + 2, 0:x1])
    print("wrote", out)


def settings_col(doc):
    return min(w["x"] for w in doc["widgets"] if w["class"] == "QPushButton" and w["text"] == "Vertical Margin")


def mission(name, out):
    """The Mission group's body (Plan row and the step buttons), without its title line."""
    im, doc = load(name)
    g = group(doc, "Mission")
    plan = find(doc, "QPushButton", "Plan")
    cv2.imwrite(os.path.join(PAGE, out), im[plan["y"] - 8:g["y"] + g["h"] + 1, 0:doc["width"]])
    print("wrote", out)


# the overview: both groups at READY
im, doc = load("05_ready")
g = group(doc, "Mission")
cv2.imwrite(os.path.join(PAGE, "gs_panel.png"), im[0:g["y"] + g["h"] + 2, 0:doc["width"]])
print("wrote gs_panel.png")

calib("01_drop_get", "gs_drop_row.png", lambda d: [row(d, "Drop", settings_col(d) - 8)])
calib("02_pick_get", "gs_pick_row.png", lambda d: [row(d, "Pick (obj_0)", settings_col(d) - 8)])
calib("03_adjust", "gs_adjust.png", lambda d: [
    union([find(d, "QPushButton", "Adjust")] +
          [w for w in d["widgets"] if w["class"] == "QLabel" and abs(w["y"] - find(d, "QPushButton", "Adjust")["y"]) < 4
           and w["x"] < find(d, "QPushButton", "EE Offset")["x"]]),
    row(d, "Start", settings_col(d) - 8)])


def settings(d):
    ee = find(d, "QPushButton", "EE Offset")
    ee_boxes = [w for w in d["widgets"] if w["class"] == "QDoubleSpinBox" and abs(w["y"] - ee["y"]) < 6
                and w["x"] > ee["x"]]
    col = [w for w in d["widgets"] if w["class"] == "QPushButton"
           and w["text"] in ("Vertical Margin", "Side Margin", "Drop Margin", "Back Margin")]
    col += [w for w in d["widgets"] if w["class"] == "QDoubleSpinBox" and w["x"] >= min(c["x"] for c in col)]
    col += [w for w in d["widgets"] if w["class"] == "QLabel" and w["text"] == "m"]
    return [union([ee] + ee_boxes), union(col)]


calib("05_ready", "gs_settings.png", settings)
mission("06_ready_to_pick", "gs_ready_to_pick.png")
mission("07_exit_pick", "gs_exit_pick.png")
mission("10_drop_hold", "gs_drop_hold.png")
mission("12_exit_drop", "gs_exit_drop.png")

if ISAAC:
    a = cv2.imread(os.path.join(ISAAC, "isaac_1.png"))       # 1280 x 1440: the six waypoints, both pillars
    a = a[300:1350, :]
    cv2.imwrite(os.path.join(PAGE, "isaac_overview.png"), cv2.resize(a, (960, 788), interpolation=cv2.INTER_AREA))
    b = cv2.imread(os.path.join(ISAAC, "isaac_75.png"))      # holding over the container, basket in the jaws
    b = b[170:598, 600:1280]                                 # (above the Start waypoint's label)
    cv2.imwrite(os.path.join(PAGE, "isaac_drop.png"), b)
    print("wrote isaac_overview.png, isaac_drop.png")
