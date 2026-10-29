#!/usr/bin/env python3
"""The turn's arm pose, re-screened on the CURRENT config (free-axis anchor 0.75, room-held, 90 deg / 18 s).
The arm absorbs the airframe's offset from its circle; here that offset is MEASURED as the cloud of claw
displacements in the body frame -- FK of the recorded joints minus FK of the flown pose [0, 5, 25, 0] --
over the turn of every completed Isaac run of tonight's config (+ the two hardware turns), then carried to
every candidate pose (q1 = q4 = 0) by 2-D IK on (q2, q3) (the 10-18 pose_fin_screen machinery). Per pose:
the worst margin to the stops over the whole cloud (q2 [-20, 45]; q3 above the 10 deg soft floor -- the
guard is 3, the elbow branch below 0 -- and under its +50 stop), the claw's lateral reach (q1 swing per mm
of the lateral sway), the skid clearance over the beam top AND over the tallest marker ball, sigma_nd.
A ranking to fly, not a prediction: it assumes the airframe's offset is the same at every pose."""
import os, sys, glob, importlib.util
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("pfs", os.path.join(HERE, "..", "..", "valve_turning_20261015", "tools", "pose_fin_screen.py"))
pfs = importlib.util.module_from_spec(spec); spec.loader.exec_module(pfs)
TP = pfs.TP; D = np.pi / 180
P0 = np.array([0.0, 5.0, 25.0, 0.0])
GRASP, BEAM_TOP, MARKER_TOP, GEAR = 0.179, 0.054, 0.1405, 0.2425      # above the plate bottom / skid below body
Q2L, Q3L = (-20.0, 45.0), (10.0, 50.0)
p = TP.make_params_t650()
c0 = pfs.claw(P0, p)
runs = sorted(glob.glob(os.path.join(HERE, "..", "sim", "runs", "*.npz")) +
              glob.glob(os.path.join(HERE, "..", "..", "valve_turning_v2_20261027", "runs", "t18_hw_*.npz")) +
              glob.glob(os.path.join(HERE, "..", "..", "valve_turning_v2_20261027", "runs", "mk_full_*.npz")))
cloud = []
for f in runs:
    z = np.load(f, allow_pickle=True)
    if bool(z["aborted"]) and "exit" not in str(z["reason"]): continue        # completed turns (exit aborts kept: the turn completed)
    if "fa05" in f or "kv15" in f or "nz_" in f: continue                   # other configs
    mk = {str(n): float(x) for n, x in zip(z["marks_name"], z["marks_t"])}
    if "push:start" not in mk or "push:end" not in mk: continue
    J = z["joints"]; w = (J[:, 0] >= mk["push:start"]) & (J[:, 0] <= mk["push:end"])
    for row in J[w][::25]:
        q = np.degrees(row[1:5]); q[0] = q[3] = 0.0
        cloud.append(pfs.claw(q, p) - c0)
cloud = np.array(cloud)
print(f"{len(runs)} run files, {len(cloud)} samples of the claw displacement absorbed by q2/q3 over completed turns at {P0[1:3]}:")
for pc in (0.5, 5, 50, 95, 99.5):
    print(f"   p{pc:<4}: fwd {np.percentile(cloud[:,0], pc)*1e3:+6.1f}  down {np.percentile(cloud[:,1], pc)*1e3:+6.1f} mm")
sub = cloud[np.linspace(0, len(cloud)-1, 400).astype(int)]
rows = []
for q2 in np.arange(-15.0, 40.1, 2.5):
    for q3 in np.arange(10.0, 45.1, 2.5):
        q = np.array([0.0, q2, q3, 0.0]); c = pfs.claw(q, p)
        worst = 1e9; qmin = [1e9, 1e9]; qmax = [-1e9, -1e9]; bad = False
        for d in sub:
            qs = pfs.solve(q.copy(), c + d, p)
            if qs is None: bad = True; break
            m = min(qs[1] - Q2L[0], Q2L[1] - qs[1], qs[2] - Q3L[0], Q3L[1] - qs[2])
            worst = min(worst, m); qmin = np.minimum(qmin, qs[1:3]); qmax = np.maximum(qmax, qs[1:3])
        if bad: continue
        body_above_grasp = c[1]                        # the claw's drop below the body
        skid_beam = GRASP + body_above_grasp - GEAR - BEAM_TOP
        skid_marker = GRASP + body_above_grasp - GEAR - MARKER_TOP
        reach = c[0]
        rows.append((worst, q2, q3, reach, c[1], skid_beam, skid_marker, TP._sigma_nd(q * D, p), qmin, qmax))
rows.sort(key=lambda r: -r[0])
print("\nposes by the worst stop margin over the measured cloud (q1 = q4 = 0):")
print("  q2    q3  | worst margin | reach  drop  | skids over beam / tallest marker | sig_nd | q2 range      q3 range")
for r in rows[:14]:
    worst, q2, q3, reach, drop, sb, sm, sig, qmin, qmax = r
    print(f" {q2:+5.1f} {q3:+5.1f} |   {worst:5.1f} deg  | {reach:.3f} {drop:.3f} |   {sb*1e3:4.0f} / {sm*1e3:4.0f} mm             | {sig:.3f}  | {qmin[0]:+5.1f}..{qmax[0]:+5.1f}  {qmin[1]:+5.1f}..{qmax[1]:+5.1f}")
r0 = [r for r in rows if abs(r[1] - 5) < 1e-6 and abs(r[2] - 25) < 1e-6][0]
print(f"\nthe flown pose [0, 5, 25, 0]: worst margin {r0[0]:.1f}, skids {r0[5]*1e3:.0f} / {r0[6]*1e3:.0f} mm, reach {r0[3]:.3f}, sig {r0[7]:.3f}")
