#!/usr/bin/env python3
"""Per-phase summary of one 2026-10-27 box push (needs ../npz/push_<tag>.npz from push_series.py):
the slide (box, arm, guards, true tilt), the window between the push end and the abort, and the
abort itself (jaws, arm, vehicle, box).  Usage: summary.py <tag> <slide t0> <slide t1> <trip t>"""
import sys
import numpy as np
from bp_common import Bag

tag, s0, s1, ttrip = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
b = Bag(tag)
S = np.load(f"../npz/push_{tag}.npz")
T = S["T"]
dt = T[1] - T[0]
box0 = S["box_al"][np.argmin(np.abs(T - (s0 - 2.0)))]       # before the clamp
veh0 = S["veh_al"][np.argmin(np.abs(T - (s0 - 2.0)))]


def w(a, c):
    return (T >= a) & (T <= c)


def arm(m, name):
    q2, q3 = S["q2"][m], S["q3"][m]
    print(f"  {name}: q2 {q2.min():5.1f}..{q2.max():5.1f} deg (>44.5: {np.mean(q2 > 44.5):4.0%}), "
          f"q3 {q3.min():5.1f}..{q3.max():5.1f} deg (<15: {np.mean(q3 < 15):4.0%}, <0: {np.sum(q3 < 0) * dt:.2f} s), "
          f"|tau2| max {np.nanmax(np.abs(S['tau2'][m])):.2f}, |tau3| max {np.nanmax(np.abs(S['tau3'][m])):.2f} N.m")


def guards(m, name):
    print(f"  {name}: planner box tilt max {np.nanmax(S['gtilt'][m]):.2f} deg (guard 2.0), |box lag| max "
          f"{1e3 * np.nanmax(np.abs(S['lag'][m])):.0f} mm (guard 40), force reading max {np.nanmax(S['fguard'][m]):.2f} N (guard 5.0); "
          f"mocap box tilt max {np.nanmax(S['box_tilt'][m]):.2f} deg, vehicle tilt max {np.nanmax(S['veh_tilt'][m]):.2f} deg")


print(f"===== {tag}")
m = w(s0 - 1.5, s0)
print(f"clamped hold {s0 - 1.5:.2f}..{s0:.2f}: box moved {1e3 * (S['box_al'][m][-1] - box0):+.0f} mm along, "
      f"vehicle {1e3 * (S['veh_al'][m].min() - veh0):+.0f}..{1e3 * (S['veh_al'][m].max() - veh0):+.0f} mm, "
      f"box along min {1e3 * (S['box_al'][m].min() - box0):+.0f} mm")
arm(m, "arm  ")
m = w(s0, s1)
print(f"slide {s0:.2f}..{s1:.2f}: box travel {1e3 * (S['box_al'][m][-1] - box0):.0f} mm at the end "
      f"(monitor plan {1e3 * np.nanmax(S['mon_plan'][m]):.0f} mm), across drift "
      f"{1e3 * (S['box_ac'][m].max() - S['box_ac'][m].min()):.0f} mm p-p, box yaw "
      f"{S['box_yaw'][m].min():.1f}..{S['box_yaw'][m].max():.1f} deg")
print(f"  CoM error: along rms {1e3 * np.nanstd(S['ecom_al'][m]):.0f} mm, across rms {1e3 * np.nanstd(S['ecom_ac'][m]):.0f} mm, "
      f"vertical rms {1e3 * np.nanstd(S['ecom_z'][m]):.0f} mm; |e_R| max {1e3 * np.nanmax(np.hypot(S['eR0'][m], S['eR1'][m])):.0f} mrad; "
      f"EE task error |e_y[0:3]| rms {1e3 * np.sqrt(np.nanmean(S['ey0'][m]**2 + S['ey1'][m]**2 + S['ey2'][m]**2)):.1f} mm; "
      f"free-axis weight {np.nanmin(S['free_w'][m]):.2f}..{np.nanmax(S['free_w'][m]):.2f}")
arm(m, "arm  ")
guards(m, "guard")
u = S["mot0"][m], S["mot1"][m], S["mot2"][m], S["mot3"][m]
print(f"  motors min {min(np.nanmin(x) for x in u):.3f} max {max(np.nanmax(x) for x in u):.3f}")

if s1 < ttrip - 0.1:
    m = w(s1, ttrip)
    g = S["grip"][m] / np.degrees(1.0)
    vb = np.gradient(S["veh_al"], T)[m]
    print(f"push end -> trip {s1:.2f}..{ttrip:.2f} ({ttrip - s1:.2f} s, jaws closed: grip {g.min()*1e3:.1f}..{g.max()*1e3:.1f} mm): "
          f"box {1e3 * (S['box_al'][m].min() - box0):.0f}..{1e3 * (S['box_al'][m].max() - box0):.0f} mm, "
          f"vehicle speed along {1e3 * vb.min():.0f}..{1e3 * vb.max():.0f} mm/s, free-axis weight "
          f"{np.nanmin(S['free_w'][m]):.2f}..{np.nanmax(S['free_w'][m]):.2f}")
    arm(m, "arm  ")
    guards(m, "guard")

# the abort
m = w(ttrip, ttrip + 8.0)
g = S["grip"] / np.degrees(1.0)            # push_series stores degrees(position); the gripper is metres
k = np.argmin(np.abs(T - ttrip))
open_i = k + np.argmax(g[k:] > g[k] + 0.010)                  # jaws 10 mm open from the clamped gap
print(f"abort at {ttrip:.2f}: grip {g[k] * 1e3:+.1f} mm; jaws 10 mm open at +{T[open_i] - ttrip:.2f} s; "
      f"grip reaches {g[m].max() * 1e3:+.1f} mm")
mm = w(ttrip, T[open_i])
arm(mm, "arm until open")
arm(m, "arm 8 s       ")
q3 = S["q3"][m]; tq = T[m]
if (q3 < 0).any():
    print(f"  q3 crosses 0 at +{tq[np.argmax(q3 < 0)] - ttrip:.2f} s, min {q3.min():.1f} deg at +{tq[np.argmin(q3)] - ttrip:.2f} s")
else:
    print(f"  q3 min {q3.min():.1f} deg at +{tq[np.argmin(q3)] - ttrip:.2f} s (never below 0)")
guards(m, "guard")
z = S["veh_z"]
mrel = w(ttrip, ttrip + 1.6)
print(f"  vehicle along {1e3 * (S['veh_al'][mrel].min() - S['veh_al'][k]):+.0f}..{1e3 * (S['veh_al'][mrel].max() - S['veh_al'][k]):+.0f} mm "
      f"in the 1.5 s release hold; climb {1e3 * (z[m].max() - z[k]):.0f} mm in 8 s")
mb = w(ttrip, T[-1])
print(f"  box: {1e3 * (S['box_al'][k] - box0):.0f} mm at the trip, {1e3 * (S['box_al'][mb][-1] - box0):.0f} mm at the end; "
      f"moved {1e3 * (S['box_al'][mb].max() - S['box_al'][mb].min()):.1f} mm along / "
      f"{1e3 * (S['box_ac'][mb].max() - S['box_ac'][mb].min()):.1f} mm across after the trip; "
      f"tilt max {S['box_tilt'][mb].max():.2f} deg, z {S['box_z'][mb][-1]:.3f} m")
