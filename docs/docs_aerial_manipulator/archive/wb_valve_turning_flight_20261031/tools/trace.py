#!/usr/bin/env python3
"""Sampled trace of the wrist-roll channel: trace.py <tag> <t0> <t1> [dt]."""
import sys, numpy as np
from vt_common import Bag
tag, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]); dt = float(sys.argv[4]) if len(sys.argv) > 4 else 0.2
b = Bag(tag)
names = [x.strip(" '[]") for x in str(b.f("js","name")[0]).split(",")]
tj = b.t("js"); P = b.f("js","position"); V = b.f("js","velocity"); E = b.f("js","effort")
i4 = names.index("joint4"); ig = names.index("gripper_left_joint"); i1 = names.index("joint1")
tw = b.t("wb"); W = b.f("wb","data"); ok = np.array([len(r) >= 120 for r in W])
tl = b.t("law"); LD = b.f("law","data")
tv = b.t("velobs"); VO = b.f("velobs","velocity") if "velobs__velocity" in b.z.files else None
vnames = str(b.f("velobs","name")[0]).split(",") if VO is not None else None
ta = b.t("armref"); AR = b.f("armref","points[0].positions"); AV = b.f("armref","points[0].velocities"); anames = str(b.f("armref","joint_names")[0]).split(",")
def at(t, ts, arr):
    i = min(np.searchsorted(ts, t), len(ts)-1); return arr[i]
print(" t     q4   qd4ref q4dot(js) v_obs | q1   grip mm | tau4_law  eh4deg wfree off_mm hdg_deg | law13..16 cmd4 | aux4 err4 | t_ext4")
for t in np.arange(t0, t1, dt):
    q4 = np.degrees(at(t, tj, P)[i4]); qd = np.degrees(at(t, tj, V)[i4]); g = 1e3*at(t, tj, P)[ig]; q1 = np.degrees(at(t, tj, P)[i1])
    ref = at(t, ta, AR); rv = at(t, ta, AV); r4 = np.degrees(ref[anames.index("joint4")]) if len(ref) else np.nan
    vo = np.degrees(at(t, tv, VO)[vnames.index("joint4")]) if VO is not None else np.nan
    j = min(np.searchsorted(tw, t), len(tw)-1)
    while j > 0 and not ok[j]: j -= 1
    w = W[j]
    ld = at(t, tl, LD)
    print(f"{t:5.1f} {q4:+6.1f} {r4:+6.1f} {qd:+7.1f} {vo:+7.1f} | {q1:+5.1f} {g:+5.1f} | {w[16]:+7.3f} {np.degrees(np.arcsin(np.clip(w[27],-1,1))):+6.1f} {w[116]:4.2f} {1e3*w[117]:+6.1f} {np.degrees(w[119]):+6.1f} | {ld[15]:+6.1f} | {ld[21]:+5.1f} {ld[31]:+6.1f} | {ld[12]:+6.3f}")
