#!/usr/bin/env python3
"""Give the arm GS's tab page the window's height: find the page/log splitter by its
grip dots (four 1-px dots on one row, blank rows around them) and drag it down to
row TARGET, so the log keeps a few lines. Holds until the window is resized or the
tab changes (ControlPanel::applyLogSplit). Run with PYTHONNOUSERSITE=1 /usr/bin/python3.

    split.py [TARGET]        (default 1290, window client coordinates)
"""
import os
import subprocess
import sys
import tempfile
import time

import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import xwin  # noqa: E402

TITLE = "Indoor Robotic Arm"
GRAB = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grab.sh")


def dots_row():
    with tempfile.NamedTemporaryFile(suffix=".png") as f:
        subprocess.run([GRAB, TITLE, f.name], check=True)
        g = cv2.imread(f.name).mean(axis=2)
    for y in range(400, g.shape[0] - 3):
        row = g[y, 290:335] < 200
        if 3 <= row.sum() <= 6 and not (g[y - 1, 290:335] < 235).any() and not (g[y + 1, 290:335] < 235).any():
            xs = [290 + i for i in range(row.size) if row[i]]
            if xs[-1] - xs[0] <= 12:
                return y, (xs[0] + xs[-1]) // 2
    return None, None


target = int(sys.argv[1]) if len(sys.argv) > 1 else 1290
w = xwin.find(TITLE)
y, x = dots_row()
if y is None:
    sys.exit("splitter grip not found (already moved? its dots are not drawn near the bottom)")
xwin.drag(w, x, y, x, target, steps=60)
time.sleep(1.0)
y2, _ = dots_row()          # the grip is not drawn once the handle sits low: gone = moved
ok = y2 is None or y2 > target - 20
print(f"splitter {y} -> {'moved' if ok else y2} (target {target})")
sys.exit(0 if ok else 1)
