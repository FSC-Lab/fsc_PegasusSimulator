#!/usr/bin/env python3
"""Relative rotation between the clamped claw (planner current_ee pose: FK on the measured base pose +
encoders) and the fin (obj_0, rigid on the rotor) over the contact, referred to the grasp instant and
split in obj_0's own axes: x = along the beam (the jaws' closing axis = the fin's normal), y = the tangent
(the fin's roll axis), z = the shaft.  A rigid grip would read zero on all three."""
import sys
import numpy as np
from vt_common import Bag
from valve_axis import obj, step_times, quat_R, rotvec
tag = sys.argv[1] if len(sys.argv) > 1 else "145526"
b = Bag(tag); st = step_times(b)
t0, t1 = st["push"] + 0.05, st.get("abort", st["push"] + 20)
g = np.arange(t0, t1, 0.02)
to, Ro, p = obj(b)
te = b.t("pl_current_ee"); Re = quat_R(*[b.f("pl_current_ee", f"pose.orientation.{c}") for c in "xyzw"])
io = np.clip(np.searchsorted(to, g), 0, len(to)-1); ie = np.clip(np.searchsorted(te, g), 0, len(te)-1)
Rrel = np.einsum("nji,njk->nik", Ro[io], Re[ie])          # Ro^T Re
dR = np.einsum("ji,njk->nik", Rrel[0], Rrel)                # in the claw frame at the grasp
dRf = np.einsum("ij,njk,lk->nil", Rrel[0], dR, Rrel[0])     # expressed in obj_0 axes
th, a = rotvec(dRf); v = np.degrees(th)[:, None] * a
tw = b.t("wb"); D = b.f("wb", "data"); ok = np.isfinite(D[:, 50]) & (D[:, 0] == 1); tw, D = tw[ok], D[ok]
es = None
print(f"=== {tag}: claw-vs-fin rotation since the grasp, deg (obj_0 axes): x = fin normal / jaw axis, y = tangent (fin roll), z = shaft")
print(f"  rms  x {v[:,0].std():5.1f}  y {v[:,1].std():5.1f}  z {v[:,2].std():5.1f} ; range x {v[:,0].min():+.1f}..{v[:,0].max():+.1f}  y {v[:,1].min():+.1f}..{v[:,1].max():+.1f}  z {v[:,2].min():+.1f}..{v[:,2].max():+.1f}")
q = np.degrees(np.stack([np.interp(g, tw, D[:, 5+i]) for i in range(4)], 1))
for k, n in enumerate("xyz"):
    print(f"  corr({n}, q1) {np.corrcoef(v[:,k], q[:,0])[0,1]:+.2f}  corr({n}, q4) {np.corrcoef(v[:,k], q[:,3])[0,1]:+.2f}")
for tt in np.arange(t0, t1, 0.5):
    i = np.searchsorted(g, tt)
    print(f"   {g[i]:6.2f}  x {v[i,0]:+6.1f}  y {v[i,1]:+6.1f}  z {v[i,2]:+6.1f}   q1 {q[i,0]:+5.1f} q4 {q[i,3]:+5.1f}")
