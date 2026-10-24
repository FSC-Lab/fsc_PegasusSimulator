#!/usr/bin/env python3
"""One flight's grasp (or release) window, sample by sample.
Columns: planner step/state; the planner's claw error to its target (push_pull/arrival_error: |e|,
across = along the box x = the jaws' closing axis, along = box y), the claw's speed; the base (mocap)
displacement and the law's CoM error x_cd - x_c (wb_control_debug [45..50]) in the BOX frame
(across / along) and up; the free-axis anchor's shift (debug [117]); q1..q4; the gripper finger;
the box (obj_0) displacement. Positions in mm relative to T0.
Usage: /usr/bin/python3 grasp_window.py <tag> <T0> <t0> <t1> [dt]"""
import sys
import numpy as np
from bp_common import Bag, yaw_deg

tag, T0, t0, t1 = sys.argv[1], *(float(x) for x in sys.argv[2:5])
dt = float(sys.argv[5]) if len(sys.argv) > 5 else 0.1
b = Bag(tag)
P = ["pose.position.x", "pose.position.y", "pose.position.z"]


def interp(key, cols_or_fields, t):
    tt = b.t(key)
    if isinstance(cols_or_fields, list):
        return np.array([np.interp(t, tt, b.f(key, f)) for f in cols_or_fields])
    return np.interp(t, tt, cols_or_fields)


oy = np.radians(np.median(yaw_deg(*[b.f("obj", f"pose.orientation.{k}") for k in "wxyz"])[b.t("obj") < T0]))
R2 = np.array([[np.cos(oy), np.sin(oy)], [-np.sin(oy), np.cos(oy)]])   # world xy -> box (across, along)
ta, A = b.t("pl_arr"), b.f("pl_arr", "data")
tw, W = b.t("wb"), b.f("wb", "data")
tj, Q = b.t("js"), b.f("js", "position")
q = lambda t: np.degrees([np.interp(t, tj, Q[:, i]) for i in (2, 0, 1, 3)])  # names j2,j3,j1,j4,gl,gr
grip = lambda t: 1e3 * np.interp(t, tj, Q[:, 4])
base0, box0 = interp("mocap", P, T0), interp("obj", P, T0)
st = b.edges("pl_pstatus")
tc = b.t("pl_current_ee")
C = np.column_stack([b.f("pl_current_ee", k) for k in P])
print(f"{tag}: box yaw {np.degrees(oy):+.1f} deg; across = box x (jaws), along = box y; T0 = {T0}")
print(f"{'t':>6s} {'step':10s}| {'|e|':>4s} {'acr':>5s} {'alg':>5s} {'vcl':>4s} | {'base a':>6s} {'l':>5s} {'z':>4s} |"
      f" {'eCoM a':>6s} {'l':>5s} | {'fax':>4s} | {'q1':>5s} {'q2':>5s} {'q3':>5s} {'q4':>5s} | {'grip':>5s} | {'box a':>5s} {'l':>5s}")
for t in np.arange(t0, t1 + 1e-9, dt):
    s = [x for x in st if x[0] <= t]
    lab = s[-1][1].split("(")[0].replace("FLYING ", "F:").replace("DONE ", "D:")[:10] if s else ""
    i = min(np.searchsorted(ta, t), len(ta) - 1)
    e = A[i]
    ec = R2 @ e[3:5] * 1e3
    v = np.linalg.norm(np.gradient(np.array([[np.interp(tt, tc, C[:, k]) for k in range(2)] for tt in (t - 0.05, t + 0.05)]), axis=0)[0]) / 0.1 * 1e3
    bs = interp("mocap", P, t) - base0
    ba = R2 @ bs[:2] * 1e3
    k = min(np.searchsorted(tw, t), len(tw) - 1)
    ecom = R2 @ (W[k, 45:47] - W[k, 48:50]) * 1e3
    bx = R2 @ (interp("obj", P, t) - box0)[:2] * 1e3
    qq = q(t)
    print(f"{t:6.2f} {lab:10s}| {e[2]*1e3:4.0f} {ec[0]:+5.0f} {ec[1]:+5.0f} {v:4.0f} | {ba[0]:+6.0f} {ba[1]:+5.0f} {bs[2]*1e3:+4.0f} |"
          f" {ecom[0]:+6.0f} {ecom[1]:+5.0f} | {W[k,117]*1e3:+4.0f} | {qq[0]:+5.1f} {qq[1]:+5.1f} {qq[2]:+5.1f} {qq[3]:+5.1f} |"
          f" {grip(t):+5.1f} | {bx[0]:+5.1f} {bx[1]:+5.1f}")
