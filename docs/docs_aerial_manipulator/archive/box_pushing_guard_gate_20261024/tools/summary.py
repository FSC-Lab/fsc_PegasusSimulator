#!/usr/bin/env python3
"""Pool the hardware-like (force-model) Isaac flights per configuration: completion, the worst q3
(guard 10) and q2 (limit 45) per window over every seed, and the means of the accuracy metrics.
    PYTHONNOUSERSITE=1 /usr/bin/python3 summary.py"""
import glob
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import phase_metrics as PMx   # noqa: E402

RUNS = os.path.join(HERE, "..", "runs")
GROUPS = [
    ("shipped (anchor 0.75 from the grasp)", ["g_ship_*", "h_ship_*"]),
    ("+ free axis from the room hold (FW)", ["g_fw_[0-9]", "h_fw_[0-9]"]),
    ("+ omega_c_t 2.0", ["g_oc2_*"]),
    ("FW + omega_c_t 2.0", ["g_fwoc2_*", "h_fwoc2_*"]),
    ("FW + settle 1.0 s (CANDIDATE)", ["i_fws10_*", "l_w_*"]),
    ("FW + exit 1.5 s", ["j_fwx15_*"]),
    ("FW + weight 0.9", ["k_w09_*"]),
    ("FW + settle 1.0 + blend 0.5 s", ["l_wb05_*"]),
    ("anchor OFF (the 10-21 hardware config)", ["g_cal0_*"]),
]


def main():
    print(f"{'configuration':42s} {'done':>5s} | {'Ready q3':>8s} {'push q3':>7s} {'rel q3':>6s} {'q2 max':>6s} | "
          f"{'Rdy sw':>6s} {'sld sw':>6s} {'e_imp':>5s} {'CoM':>5s} | {'travel 0.10 / 0.50 m':>22s} | failures")
    for name, pats in GROUPS:
        paths = sorted(set(p for pat in pats for p in glob.glob(os.path.join(RUNS, pat + ".npz"))))
        if not paths:
            continue
        rows = [PMx.one(p) for p in paths]
        ok = [r for r in rows if r["ok"]]
        def mn(k):
            x = [r[k] for r in rows if np.isfinite(r.get(k, np.nan))]
            return min(x) if x else np.nan
        def mx(k):
            x = [r[k] for r in rows if np.isfinite(r.get(k, np.nan))]
            return max(x) if x else np.nan
        def me(k):
            x = [r[k] for r in ok if np.isfinite(r.get(k, np.nan))]
            return np.mean(x) if x else np.nan
        tr_s = [r["trav"] for r in ok if "trav" in r and r["trav"] < 200]
        tr_l = [r["trav"] for r in ok if "trav" in r and r["trav"] >= 200]
        trv = (f"{min(tr_s):.0f}-{max(tr_s):.0f}" if tr_s else "-") + " / " + (f"{min(tr_l):.0f}-{max(tr_l):.0f}" if tr_l else "-")
        fails = "; ".join(f"{r['tag']}: {r['reason'][:48]}" for r in rows if not r["ok"])
        print(f"{name:42s} {len(ok)}/{len(rows):<3d} | {mn('rdy_q3'):8.1f} {mn('psh_q3'):7.1f} {mn('rel_q3'):6.1f} "
              f"{max(mx('rdy_q2'), mx('psh_q2'), mx('rel_q2')):6.1f} | {me('rdy_sw'):6.1f} {me('sld_sw'):6.2f} "
              f"{me('eimp'):5.2f} {me('com'):5.1f} | {trv:>22s} | {fails}")


if __name__ == "__main__":
    main()
