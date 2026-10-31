#!/usr/bin/env python3
"""The contact-only path: F_hat_y as the law consumed it (debug [58..61]) and the term it injects into
u3, -(Lambda_y M_y^-1 - I) F_hat_y, with the flown M_y = diag(1,1,1,0.05) and Lambda_y at the grasp pose
(nose frame: along 1.472, lateral 0.471, vertical 0.700 kg) -- its size, and its phase against the
lateral sway (a force in phase with the sway velocity feeds it)."""
import numpy as np
from vt_common import Bag, BAGS
from valve_axis import step_times
LAM = {"nose": 1.472, "lat": 0.471, "z": 0.700}
for tag in BAGS:
    b = Bag(tag); st = step_times(b); T = np.load(f"../npz/turn_{tag}.npz"); g = T["g"]
    tw = b.t("wb"); W = b.f("wb", "data"); ok = np.isfinite(W[:, 61]); tw, W = tw[ok], W[ok]
    Fy = np.stack([np.interp(g, tw, W[:, 58+i]) for i in range(3)], 1)
    tv = b.t("vatt"); qv = b.f("vatt", "q")
    # the nose direction from the base yaw (PX4 NED quaternion -> ENU yaw)
    yaw_ned = np.arctan2(2*(qv[:,0]*qv[:,3]+qv[:,1]*qv[:,2]), 1-2*(qv[:,2]**2+qv[:,3]**2))
    psi = np.interp(g, tv, np.unwrap(np.pi/2 - yaw_ned))
    nose = np.stack([np.cos(psi), np.sin(psi)], 1); lat = np.stack([-np.sin(psi), np.cos(psi)], 1)
    Fn, Fl = np.sum(Fy[:, :2]*nose, 1), np.sum(Fy[:, :2]*lat, 1)
    inj_n, inj_l, inj_z = -(LAM["nose"]-1)*Fn, -(LAM["lat"]-1)*Fl, -(LAM["z"]-1)*Fy[:, 2]
    w = (g >= st["push"] + 0.3) & (g <= st["abort"])
    # lateral CoM error and velocity in the same nose frame
    ec = T["ec"]; el = np.sum(ec[:, :2]*lat, 1); vl = np.gradient(el, g)
    P = inj_l[w] * vl[w]
    print(f"===== {tag}: F_hat_y consumed rms (nose/lat/z) {Fn[w].std():.2f}/{Fl[w].std():.2f}/{Fy[w,2].std():.2f} N;  injected rms {inj_n[w].std():.2f}/{inj_l[w].std():.2f}/{inj_z[w].std():.2f} N")
    print(f"  corr(injected lateral, lateral CoM error) {np.corrcoef(inj_l[w], el[w])[0,1]:+.2f}, with its velocity {np.corrcoef(inj_l[w], vl[w])[0,1]:+.2f};"
          f"  mean power into the sway {P.mean()*1e3:+.2f} mW (sign only meaningful if |corr| is large)")
