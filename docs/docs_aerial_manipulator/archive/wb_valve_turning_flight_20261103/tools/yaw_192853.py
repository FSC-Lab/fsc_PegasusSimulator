#!/usr/bin/env python3
"""192853: where the yaw demand came from (user question, 2026-11-03: "why does the push use up the yaw authority? is
that caused by the reference?"). 10 Hz trace from Ready through SAFETY:
- the references the law flew: platform heading psi_d (from b1_d) and the EE heading psi_ed (from b1_de);
- the measured base yaw (mocap) and the valve angle;
- the law: e_R z [30], EE heading error [27] (sin), tau_body FLU z [23], the observer's yaw estimate d_hat_r z [36]
  (filtered) and [67] (unfiltered), the arm joint torques [13..16];
- the allocator: unallocated yaw (FRD, desired - achieved) [54], thrust [55]; the four motors [41..44];
- the raw contact-force reading F_hat [97..100] (task x y z + heading)."""
import sys
import numpy as np
from vt_common import Bag, yaw_deg
t0, t1 = (float(sys.argv[1]), float(sys.argv[2])) if len(sys.argv) > 2 else (31.0, 35.2)
b = Bag("192853")
tw = b.t("wb"); W = np.asarray(b.f("wb", "data"), float)
tr = b.t("wbref")
psi_d = np.degrees(np.unwrap(np.arctan2(b.f("wbref", "b1_d.y"), b.f("wbref", "b1_d.x"))))
psi_ed = np.degrees(np.unwrap(np.arctan2(b.f("wbref", "b1_de.y"), b.f("wbref", "b1_de.x"))))
qd = np.degrees(np.asarray(b.f("wbref", "q_d"), float))
tv = b.t("vrpn"); yv = np.degrees(np.unwrap(np.radians(yaw_deg(*[b.f("vrpn", f"pose.orientation.{c}") for c in "wxyz"]))))
to = b.t("obj"); yo = np.degrees(np.unwrap(np.radians(yaw_deg(*[b.f("obj", f"pose.orientation.{c}") for c in "wxyz"]))))
y_get = np.median(yo[(to > 9) & (to < 11)])
# the frame of the model heading: the law's model frame is the actual body frame turned -90 deg about z
print("    t  | psi_d psi_ed  yaw   | valve | e_Rz  eEEpsi | tauZ  dRz  dRz_c | tau_j1 j2   j3   j4 | unalYaw unalT | "
      "m1(FR,ccw) m2(RL,ccw) m3(FL,cw) m4(RR,cw) | u1 | Fhat x y z psi | q_d1 q_d4")
for t in np.arange(t0, t1, 0.1):
    i = min(np.searchsorted(tw, t), len(tw) - 1); w = W[i]
    k = min(np.searchsorted(tr, t), len(tr) - 1)
    iv = min(np.searchsorted(tv, t), len(tv) - 1); io = min(np.searchsorted(to, t), len(to) - 1)
    m = w[41:45]
    print(f" {t:5.1f} | {psi_d[k]:+6.1f} {psi_ed[k]:+6.1f} {yv[iv]:+6.1f} | {yo[io]-y_get:+5.1f} | "
          f"{w[30]:+.3f} {np.degrees(np.arcsin(np.clip(w[27],-1,1))):+5.1f} | {w[23]:+.3f} {w[36]:+.3f} {w[67]:+.3f} | "
          f"{w[13]:+.2f} {w[14]:+.2f} {w[15]:+.2f} {w[16]:+.2f} | {w[54]:+.3f} {w[55]:+5.1f} | "
          f"{m[0]:.2f} {m[1]:.2f} {m[2]:.2f} {m[3]:.2f} | {w[17]:4.1f} | "
          f"{w[97]:+5.2f} {w[98]:+5.2f} {w[99]:+5.2f} {w[100]:+5.2f} | {qd[k,0]:+5.1f} {qd[k,3]:+5.1f}")
