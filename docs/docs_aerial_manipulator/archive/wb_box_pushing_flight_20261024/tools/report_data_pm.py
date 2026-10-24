#!/usr/bin/env python3
"""Chart data for the afternoon section of report.html (flights A 162850, B 163413, C 164419).
handover: aligned at the end of Ready (t = 0), every 0.04 s from -3 to +1.4 s:
  short  the planner's claw error to the grasp point ALONG the push [mm] (push_pull/arrival_error,
         + = the claw has not reached it), cut at the guard trip (the planner stops updating it)
  q2/q3  measured joints [deg]
  trip   the q3 guard's trip time [s] (None if it did not trip at the grasp)
scatter: pitch_offset_fit.json (CoM short-along vs attitude error about model x).
Usage: /usr/bin/python3 report_data_pm.py > ../report_data_pm.json"""
import json, os
import numpy as np
from bp_common import Bag, yaw_deg

HERE = os.path.dirname(os.path.abspath(__file__))
T = np.round(np.arange(-3.0, 1.4001, 0.04), 2)
out = {"t": [float(v) for v in T], "flights": {}}
for tag, name in (("162850", "A"), ("163413", "B"), ("164419", "C")):
    b = Bag(tag)
    st = b.edges("pl_pstatus")
    tr = [t for t, s in st if s.startswith("DONE ready")][0]
    trip = [t for t, s in st if s.startswith("ABORTING") and t < tr + 3]
    trip = trip[0] - tr if trip else None
    oy = np.radians(np.median(yaw_deg(*[b.f("obj", f"pose.orientation.{k}") for k in "wxyz"])[b.t("obj") < tr]))
    ta, A = b.t("pl_arr"), b.f("pl_arr", "data")
    m = A[:, 0] == 1                                   # Ready's target = the grasp point
    along = -(-np.sin(oy) * A[m, 3] + np.cos(oy) * A[m, 4]) * 1e3
    ts = ta[m] - tr
    short = np.interp(T, ts, along, left=np.nan, right=np.nan)
    if trip is not None:
        short[T > trip] = np.nan
    tj, Q = b.t("js"), b.f("js", "position")
    q2 = np.degrees(np.interp(T + tr, tj, Q[:, 0]))
    q3 = np.degrees(np.interp(T + tr, tj, Q[:, 1]))
    r = lambda a: [None if not np.isfinite(v) else round(float(v), 1) for v in a]  # noqa: E731
    out["flights"][name] = {"tag": tag, "short": r(short), "q2": r(q2), "q3": r(q3),
                            "trip": None if trip is None else round(trip, 2)}
out["scatter"] = json.load(open(os.path.join(HERE, "..", "pitch_offset_fit.json")))
print(json.dumps(out))
