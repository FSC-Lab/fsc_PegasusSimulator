#!/usr/bin/env python3
"""Where to hold the U (2026-11-02, archive/valve_upush_20261102): for a claw pitch, search the grasp depth d
(the claw point d below the fin top) and the shift s (the claw point s AHEAD of the fin's centre along the turn)
on the CAD bracket (link5: bracket + hub crank + rods). For each (d, s):
  - the descent (Ready: a straight slide from 0.12 m behind, 0.10 m above) must keep every bracket point in the
    fin's slab (|y| < 2.5 mm + TOL) out of the fin -- the minimum clearance along the slide is reported;
  - the push: moving forward along the turn, the first bracket point (|y| < 2.5 mm) the fin's back edge meets --
    the slack before it, and how far below the fin top it is.
Run with PYTHONNOUSERSITE=1 /usr/bin/python3 -I."""
import os, sys, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import push_geom_screen as G

FIN_L, FIN_H, HALF_T = 0.040, 0.130, 0.0025
def run(pitch_deg, tol=0.003, depths=range(35, 91, 5), shifts=range(-20, 11, 5), back=0.120, above=0.100):
    p = np.radians(pitch_deg)
    m_col = np.abs(G.S[:, 1]) < HALF_T + tol
    m_push = np.abs(G.S[:, 1]) < HALF_T
    fc, uc = G.side(G.S[m_col], p); fp, upp = G.side(G.S[m_push], p)
    lam = np.linspace(0.0, 1.0, 121)
    print(f"claw {pitch_deg} deg below level: rows d (mm under the fin top), columns s (claw point ahead of the fin centre, mm)")
    print("   each cell: descent clearance mm / push contact depth under the fin top mm / slack mm   (x = the bracket hits the fin)")
    print("     d \\ s " + "".join(f"{s:>16d}" for s in shifts))
    best = []
    for dmm in depths:
        d = dmm / 1e3; cells = []
        for smm in shifts:
            s = smm / 1e3
            b_edge, f_edge, top, bot = -FIN_L / 2 - s, FIN_L / 2 - s, d, d - FIN_H
            # descent: claw at (-back*(1-l), +above*(1-l)); clearance = signed distance outside the fin's rectangle
            clear = np.inf
            for l in lam:
                fa = fc - back * (1 - l); ua = uc + above * (1 - l)
                inside = (fa > b_edge) & (fa < f_edge) & (ua < top) & (ua > bot)
                if inside.any():
                    clear = -1.0; break
                dx = np.maximum(np.maximum(b_edge - fa, fa - f_edge), 0.0)
                dz = np.maximum(np.maximum(bot - ua, ua - top), 0.0)
                clear = min(clear, float(np.min(np.hypot(dx, dz))))
            if clear < 0:
                cells.append(f"{'x':>16s}"); continue
            sel = (upp <= top) & (upp >= bot) & (fp <= b_edge)
            if not sel.any():
                cells.append(f"{clear*1e3:5.0f}/   none   "); continue
            k = np.argmax(np.where(sel, fp, -1e9))
            depth, slack = (top - upp[k]) * 1e3, (b_edge - fp[k]) * 1e3
            cells.append(f"{clear*1e3:5.0f}/{depth:4.0f}/{slack:4.0f}  ")
            best.append((depth, -slack, clear * 1e3, dmm, smm, slack))
        print(f"    {dmm:4d}   " + "".join(cells))
    good = [b for b in best if b[2] >= 4 and b[5] <= 20 and b[0] >= 12]
    good.sort(reverse=True)
    print("candidates (contact >= 12 mm under the top, slack <= 20 mm, descent clearance >= 4 mm):")
    for g in good[:8]:
        print(f"   d {g[3]} mm, s {g[4]:+d} mm: contact {g[0]:.0f} mm under the top, slack {g[5]:.0f} mm, descent clearance {g[2]:.0f} mm")

if __name__ == "__main__":
    for pd in (60, 50):
        run(pd); print()
