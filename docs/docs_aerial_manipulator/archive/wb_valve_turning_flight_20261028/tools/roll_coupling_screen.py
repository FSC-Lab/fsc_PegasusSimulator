#!/usr/bin/env python3
"""Which grasp pose couples the vehicle's sideways sway LEAST into the clamped claw's roll?

The 10-28 mechanism (fin_roll.py, observer_interaction_study_20261029): the base sways sideways, the
arm keeps the claw pinned on the fin (q1 takes up the sideways offset, q4 holds the heading), and at the
flown pose that q1/q4 pair also ROLLS the claw about the nose. The fin resists the roll and the couple
lands on the base, where the attitude loop answers it ~90 deg late. The roll moment is therefore
proportional to  d(claw roll)/d(sideways offset)  at the pose.  Per candidate pose (q1 = q4 = 0):
  * that coupling, from the planner's own IK (claw position + heading azimuth held, base moved
    sideways by +-5 mm), in deg of claw roll per mm, plus q1 and q4 per mm;
  * the worst stop margin over the claw displacements measured in tonight's completed Isaac turns and
    the two hardware turns (pose_screen_now.py's cloud), the skids' clearance over the beam top and the
    tallest marker ball, sigma_nd.
Calibration: the flown pose must give ~0.10-0.15 deg/mm (hardware: claw roll 3.8 deg rms over a 37 mm
rms sway; FK of the flown q1/q4 swing ~9 deg over ~61 mm)."""
import os, glob, importlib.util
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("pfs", os.path.join(HERE, "..", "..", "valve_turning_20261015", "tools", "pose_fin_screen.py"))
pfs = importlib.util.module_from_spec(spec); spec.loader.exec_module(pfs)
TP = pfs.TP; D = np.pi / 180
P0 = np.array([0.0, 5.0, 25.0, 0.0])
GRASP, BEAM_TOP, MARKER_TOP, GEAR = 0.179, 0.054, 0.1405, 0.2425
Q2L, Q3L = (-20.0, 45.0), (10.0, 50.0)
p = TP.make_params_t650()


def rotvec(E):
    th = np.arccos(np.clip((np.trace(E) - 1) / 2, -1, 1))
    v = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    return th * v / (np.linalg.norm(v) + 1e-15)


def coupling(qdeg, axis=0, d=0.005):
    """d/d(base offset along model axis `axis`) of (claw rotation vector, q) with the claw pinned."""
    q0 = qdeg * D
    _, r0e, R0 = TP.arm_fk_model(q0, p)
    az0 = np.arctan2(R0[1, 0], R0[0, 0])
    out = []
    for s in (+1, -1):
        tgt = r0e.copy(); tgt[axis] -= s * d                 # base moves +d -> the claw sits -d in the base frame
        q, info = TP.ik_position_azimuth(p, tgt, az0, q0)
        if info["residual"] > 1e-8:
            return None
        _, _, R = TP.arm_fk_model(q, p)
        out.append((rotvec(R @ R0.T), q))
    dr = (out[0][0] - out[1][0]) / (2 * d * 1e3) / D           # deg per mm, model axes (right, nose, up)
    dq = (out[0][1] - out[1][1]) / (2 * d * 1e3) / D
    return dr, dq


# the measured claw-displacement cloud (as pose_screen_now.py)
runs = sorted(glob.glob(os.path.join(HERE, "..", "sim", "runs", "*.npz")) +
              glob.glob(os.path.join(HERE, "..", "..", "valve_turning_v2_20261027", "runs", "t18_hw_*.npz")) +
              glob.glob(os.path.join(HERE, "..", "..", "valve_turning_v2_20261027", "runs", "mk_full_*.npz")))
c0 = pfs.claw(P0, p); cloud = []
for f in runs:
    z = np.load(f, allow_pickle=True)
    if bool(z["aborted"]) and "exit" not in str(z["reason"]): continue
    if "fa05" in f or "kv15" in f or "nz_" in f: continue
    mk = {str(n): float(x) for n, x in zip(z["marks_name"], z["marks_t"])}
    if "push:start" not in mk or "push:end" not in mk: continue
    J = z["joints"]; w = (J[:, 0] >= mk["push:start"]) & (J[:, 0] <= mk["push:end"])
    for row in J[w][::25]:
        q = np.degrees(row[1:5]); q[0] = q[3] = 0.0
        cloud.append(pfs.claw(q, p) - c0)
sub = np.array(cloud)[np.linspace(0, len(cloud) - 1, 300).astype(int)]

rows = []
for q2 in np.arange(-15.0, 40.1, 2.5):
    for q3 in np.arange(10.0, 45.1, 2.5):
        q = np.array([0.0, q2, q3, 0.0]); c = pfs.claw(q, p)
        skid_beam = GRASP + c[1] - GEAR - BEAM_TOP; skid_marker = GRASP + c[1] - GEAR - MARKER_TOP
        if skid_beam < 0.06 or skid_marker < 0.0:
            continue
        cp = coupling(q)
        if cp is None:
            continue
        worst = 1e9
        for dd in sub:
            qs = pfs.solve(q.copy(), c + dd, p)
            if qs is None: worst = -1e9; break
            worst = min(worst, qs[1] - Q2L[0], Q2L[1] - qs[1], qs[2] - Q3L[0], Q3L[1] - qs[2])
        rows.append(dict(q2=q2, q3=q3, roll=cp[0][1], pitch=cp[0][0], yaw=cp[0][2], dq1=cp[1][0], dq4=cp[1][3],
                         worst=worst, sb=skid_beam, sm=skid_marker, sig=TP._sigma_nd(q * D, p), reach=c[0], drop=c[1]))

flown = [r for r in rows if r["q2"] == 5.0 and r["q3"] == 25.0][0]
print(f"{len(sub)} claw-displacement samples from {len(runs)} runs; flown pose [0, 5, 25, 0]: claw roll "
      f"{flown['roll']:+.3f} deg per mm of sideways sway (q1 {flown['dq1']:+.3f}, q4 {flown['dq4']:+.3f} deg/mm), "
      f"worst stop margin {flown['worst']:.1f} deg")
print("\nposes with a stop margin >= 0 over the measured cloud, by |roll coupling| (relative to the flown pose):")
print("   q2    q3  fold | roll deg/mm  rel | q1/mm  q4/mm | worst margin | skids beam/marker mm | reach drop m | sig_nd")
ok = sorted([r for r in rows if r["worst"] >= 0.0], key=lambda r: abs(r["roll"]))
for r in ok[:12] + [flown]:
    print(f"  {r['q2']:+5.1f} {r['q3']:+5.1f} {r['q2']+r['q3']:5.1f} | {r['roll']:+.3f}   {abs(r['roll'])/abs(flown['roll']):4.2f} |"
          f" {r['dq1']:+.3f} {r['dq4']:+.3f} | {r['worst']:6.1f} deg   |  {r['sb']*1e3:4.0f} / {r['sm']*1e3:4.0f}        |"
          f" {r['reach']:.3f} {r['drop']:.3f} | {r['sig']:.3f}")
print("\nby fold, the best roll coupling at any split (all margins):")
for fold in sorted({r["q2"] + r["q3"] for r in rows}):
    rs = [r for r in rows if r["q2"] + r["q3"] == fold]
    b = min(rs, key=lambda r: abs(r["roll"]))
    print(f"  fold {fold:5.1f}: roll {b['roll']:+.3f} deg/mm at [{b['q2']:+.1f}, {b['q3']:+.1f}], worst margin {b['worst']:6.1f}")
