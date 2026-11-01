#!/usr/bin/env python3
"""Attitude / arm-torque statistics in windows that separate the evening's two changes (the grip schedule's stiff
heading before the grasp; the j1/j4 error damping raised at Go To Start) within each flight, against the afternoon.
Per window: |e_R| rms and the roll/pitch components' std (model axes: x = pitch, y = roll), the law's u3_1 / u3_4 rms,
K_psi in force [123], the arm's error-damping duty j1/j4 rms, and the rotational observer's roll/pitch estimate drift."""
import numpy as np
from vt_common import Bag
def stats(b, a, z, lab):
    tw = b.t("wb"); W = b.f("wb", "data"); ok = np.array([len(r) >= 120 for r in W]); tw = tw[ok]
    Wm = np.array([np.r_[r[:124], [np.nan]*(124-len(r[:124]))] if len(r) < 124 else r[:124] for r in W[ok]])
    m = (tw >= a) & (tw <= z); Wk = Wm[m]
    tl = b.t("law"); LD = np.array([r if len(r) >= 32 else np.full(32, np.nan) for r in b.f("law", "data")]); ml = (tl >= a) & (tl <= z)
    eR = Wk[:, 28:31]; kpsi = np.nanmean(Wk[:, 123]) if Wm.shape[1] > 123 else float("nan")
    print(f"  {lab:38s} {z-a:5.1f}s | |e_R| rms {1e3*np.sqrt((eR**2).sum(1).mean()):4.0f} mrad, pitch std {1e3*eR[:,0].std():3.0f} roll std {1e3*eR[:,1].std():3.0f} | "
          f"u3_1 rms {np.sqrt((Wk[:,13]**2).mean()):.3f} u3_4 rms {np.sqrt((Wk[:,16]**2).mean()):.3f} | K_psi {kpsi:4.2f} | errdamp duty j1 {np.nanstd(LD[ml,28]):5.2f} j4 {np.nanstd(LD[ml,31]):5.2f} | "
          f"d_r pitch {Wk[0,34]:+.3f}->{Wk[-1,34]:+.3f} roll {Wk[0,35]:+.3f}->{Wk[-1,35]:+.3f}")
def ed(b, pref):
    return [t for t, s in b.edges("pl_pstatus") if s.startswith(pref)]
for tag in ("174943", "175725", "180423", "211808"):
    b = Bag(tag); tdir = [t for t, s in b.edges("wbmode") if s == "DIRECT"][0]
    tg = ed(b, "FLYING go_to_start")[0]; th = ed(b, "DONE go_to_start")[0]; tr = ed(b, "FLYING ready")[0]
    print(f"== {tag}: DIRECT {tdir:.2f}, Go To Start {tg:.2f}, room hold {th:.2f}, Ready {tr:.2f}")
    if tag == "211808":
        stats(b, tdir + 1.5, 7.4, "hover, jaws CLOSED: soft heading, d 0.05")
        stats(b, 8.8, tg - 0.05 if tg > 9.0 else 9.9, "hover, jaws OPEN: STIFF heading, d 0.05")
    else:
        stats(b, tdir + 1.5, tg - 0.1, "hover before Go To Start (soft, d 0.05)")
    stats(b, tg + 0.5, th, "Go To Start (moving)")
    stats(b, th - 3.0, th, "last 3 s of Go To Start")
    stats(b, th, tr, "room hold (arm still)")
