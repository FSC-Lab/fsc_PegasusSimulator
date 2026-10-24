#!/usr/bin/env python3
"""How far off does the claw arrive at the end of Ready To Push, and does it decide the outcome?

For every Isaac box push with the GS's automatic steps (this campaign + the 10-22 contact-stability
campaign) and the two hardware pushes: the claw's error to the point above the handle when Ready
starts (push_pull/arrival_error, step go_to_start), its error to the grasp point when Ready ends
(step ready: |e|, across the fin), how much it grew during the 3.2 s descent, and the outcome.
    /usr/bin/python3 arrival_errors.py"""
import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.join(HERE, "..", "..")
RUNS = [os.path.join(HERE, "..", "runs"), os.path.join(ARCH, "box_pushing_contact_stability_tune_20261021", "runs")]


def outcome(p):
    z = np.load(p, allow_pickle=True)
    names = [str(n) for n in z["marks_name"]]
    if not bool(z["aborted"]) and "go_to_land_start:end" in names:
        return "complete"
    txt = open(p[:-4] + ".log", errors="replace").read() if os.path.exists(p[:-4] + ".log") else ""
    m = re.search(r"SAFETY GUARD: ([^(]*)", txt)
    if m:
        return "guard: " + m.group(1).strip()[:28]
    if "timeout" in txt and "automatic Push" in txt:
        return "no grasp (timeout)"
    if "ABORTING --" in txt:
        return "abort (GS / driver)"
    return "aborted"


def sample(A, step, t, before):
    """the claw error row of `step` (claw target) nearest t: the last one before t, or the first after"""
    r = A[(A[:, 1] == step) & (A[:, 2] == 1)]
    if not len(r):
        return None
    k = np.searchsorted(r[:, 0], t)
    k = k - 1 if before else k
    return r[min(max(k, 0), len(r) - 1)]


rows = []
for d in RUNS:
    for p in sorted(glob.glob(os.path.join(d, "*.npz"))):
        z = np.load(p, allow_pickle=True)
        mk = {str(n): float(t) for n, t in zip(z["marks_name"], z["marks_t"])}
        if "ready:start" not in mk or "ready:end" not in mk or "arrival" not in z.files:
            continue
        args = open(p[:-4] + ".args").read() if os.path.exists(p[:-4] + ".args") else ""
        if "ARM_GS_AUTO_STEPS=true" not in args:      # the GS's automatic steps only (the hardware sequence)
            continue
        A = z["arrival"]
        a0 = sample(A, 0, mk["ready:start"], True)
        a1 = sample(A, 1, mk["ready:end"] + 0.05, False)
        if a0 is None or a1 is None:
            continue
        rows.append((os.path.basename(p)[:-4], a0[3] * 1e3, a1[3] * 1e3, a1[10] * 1e3, outcome(p)))

print(f"{len(rows)} Isaac runs with the GS's automatic steps; error at Ready start (to the point above "
      f"the handle) and at Ready's end (to the grasp point) [mm]")
print(f"{'run':14s} {'start':>6s} {'arrive':>6s} {'across':>7s}  outcome")
for r in sorted(rows, key=lambda r: -r[2]):
    print(f"{r[0]:14s} {r[1]:6.1f} {r[2]:6.1f} {r[3]:+7.1f}  {r[4]}")
e1 = np.array([r[2] for r in rows])
ok = np.array([r[4] == "complete" for r in rows])
for lo, hi in ((0, 20), (20, 30), (30, 40), (40, 1e9)):
    m = (e1 >= lo) & (e1 < hi)
    if m.any():
        print(f"arrival {lo:>2.0f}-{hi if hi < 1e9 else 'inf':>3} mm: {m.sum():2d} runs, {ok[m].sum():2d} complete")
e0 = np.array([r[1] for r in rows])
print(f"corr(start error, arrival error) = {np.corrcoef(e0, e1)[0, 1]:.2f}; arrival - start: median "
      f"{np.median(e1 - e0):+.1f} mm")

# the hardware pushes (the 10-21 / 10-23 bags, through the 10-23 analysis loader)
sys.path.insert(0, os.path.join(ARCH, "wb_box_pushing_flight_20261023", "tools"))
try:
    from bp_common import Bag
    for tag, (t_rs, t_re) in (("170123", (33.3, 36.5)), ("172853", (41.81, 45.00))):
        b = Bag(tag)
        ta = b.t("pl_arr")
        A = np.column_stack([ta, np.vstack(b.f("pl_arr", "data")).astype(float)])
        a0, a1 = sample(A, 0, t_rs, True), sample(A, 1, t_re + 0.05, False)
        print(f"HW {tag}: start {a0[3]*1e3:5.1f}, arrival {a1[3]*1e3:5.1f}, across {a1[10]*1e3:+5.1f} mm")
except Exception as e:  # noqa: BLE001
    print("hardware part skipped:", e)
