"""q2/q3 tracking and joint-torque delivery over the circle run -- the 0924 report's section-3.1 statistics
(wb_l1_4d_flight_20260924/tools/arm_stats_table.py), same definitions, extended to today's flights.

  span realised   (max q - min q) / (max q_d - min q_d)
  err             q - q_d [deg] (whole-body flights: the law's own q, q_d = wb_control_debug[5..12];
                  decoupled flights: joint_states vs the planner stream, model convention)
  hold err        the same over the start hold before the run (static reference): mean = offset, std = ripple
  stuck           share of the moving-reference time (|q_d'| > 1/4 of its peak) with |q'| < 1 deg/s
  peak speed      99th percentile of the measured |q'| against the same percentile of the reference's |q_d'|
                  (a max is set by single interpolation spikes across feedback gaps; the planned peak is 15.6 deg/s)
  overshoot       how far q went past the reference's extremes (negative = fell short)
  FF opposing     share of the moving time (|q'| > 1 deg/s) where the low-passed friction feed-forward opposes the
                  measured velocity (whole-body flights only; the decoupled arm is in position mode)
  torques         10 Hz-filtered law command / intended (law + gravity correction + friction FF) / applied (Present
                  Current / kappa_tau); caps 2.44 N.m (j2), 1.42 N.m (j3)
Velocity q' = 0.1 s central difference of the measured angle on a uniform 250 Hz grid (same on every flight; the
0924 table used the arm's velocity observer, which today's whole-body law also closes on).
Writes ../analysis/arm_stats.json.
"""
import json
from common import *

CAP = {1: 2.44, 2: 1.42}


def lp(x, k=25):
    return np.convolve(x, np.ones(k) / k, "same")


def hold_window(nm, c0):
    d, t0 = load(nm)
    st = [e for e in edges(d, t0, "pl_status") if e[0] < c0]
    h0 = [t for t, v in st if v.startswith("HOLD")][-1]
    return h0, c0


def arm_rows(nm):
    d, t0 = load(nm); a, b, c0, c1 = windows(nm); h0, h1 = hold_window(nm, c0)
    dt = 0.004; tu = np.arange(c0, c1, dt); th = np.arange(h0 + 0.3, h1 - 0.1, dt)
    wb = "wb__recv" in d.files
    if wb:
        tw = d["wb__recv"] - t0; D = d["wb__data"]; tq, q, qd = tw, D[:, 5:9], D[:, 9:13]
    else:
        tj, qj, _ = joints_model(d, t0); tr, ref = wbref(d, t0)
        tq, q = tj, qj; qd = interp(tj, tr, ref["q_d"])
    Q = np.degrees(interp(tu, tq, q)); QD = np.degrees(interp(tu, tq, qd))
    QH = np.degrees(interp(th, tq, q)); QDH = np.degrees(interp(th, tq, qd))
    cd = lambda X, h=12: np.vstack([np.repeat(((X[2*h:] - X[:-2*h]) / (2 * h * dt))[:1], h, 0), (X[2*h:] - X[:-2*h]) / (2 * h * dt), np.repeat(((X[2*h:] - X[:-2*h]) / (2 * h * dt))[-1:], h, 0)])
    V = cd(Q); VD = cd(QD)
    rows = []
    if wb:
        tl = d["law__recv"] - t0; L = d["law__data"]; ml = (tl > c0) & (tl < c1); L = L[ml]; tl = tl[ml]
        tj = d["js__recv"] - t0; app = d["js__effort"][:, :4][:, JS_IDX] / KT; tc = d["tcmd__recv"] - t0; cmd = d["tcmd__effort"][:, :4]
        cmd_l = interp(tl, tc, cmd); app_l = interp(tl, tj, app); aux = L[:, 18:22] / KPWM; gc = L[:, 22:26] / KPWM
        vl = interp(tl, tu, V)
    else:
        tj = d["js__recv"] - t0; app = d["js__effort"][:, :4][:, JS_IDX] / KT
        mj = (tj > c0) & (tj < c1); app_l = app[mj]
    for j in (1, 2):
        e = Q[:, j] - QD[:, j]; eh = QH[:, j] - QDH[:, j]; pk_ref = np.percentile(np.abs(VD[:, j]), 99)
        span = (Q[:, j].max() - Q[:, j].min()) / (QD[:, j].max() - QD[:, j].min())
        x = Q[:, j] - Q[:, j].mean(); y = QD[:, j] - QD[:, j].mean(); best = (0, -2)
        for Lg in range(0, 500, 2):
            c = np.corrcoef(x[Lg:], y[:len(y) - Lg])[0, 1]
            if c > best[1]: best = (Lg, c)
        refmov = np.abs(VD[:, j]) > 0.25 * pk_ref
        r = dict(flight=FLIGHTS[nm], nm=nm, joint=f"j{j+1}", qd_min=QD[:, j].min(), qd_max=QD[:, j].max(), q_min=Q[:, j].min(), q_max=Q[:, j].max(),
                 span=100 * span, e_mean=e.mean(), e_rms=np.sqrt((e**2).mean()), e_max=np.abs(e).max(), hold_mean=eh.mean(), hold_std=eh.std(),
                 lag_s=best[0] * dt, lag_corr=best[1], stuck=100 * (refmov & (np.abs(V[:, j]) < 1.0)).sum() / max(refmov.sum(), 1),
                 pk_ref=pk_ref, pk_meas=np.percentile(np.abs(V[:, j]), 99), over_hi=Q[:, j].max() - QD[:, j].max(), over_lo=QD[:, j].min() - Q[:, j].min(),
                 at_stop=100 * (Q[:, j] >= 49.5).mean())
        if wb:
            ffl = lp(aux[:, j]); mov = np.abs(vl[:, j]) > 1.0
            r["ff_opp"] = 100 * ((np.sign(ffl[mov]) * np.sign(vl[mov, j])) < 0).mean()
            r["ff_rms"] = float(np.sqrt((ffl**2).mean()))
            c_ = lp(cmd_l[:, j]); it = lp(cmd_l[:, j] + gc[:, j] + aux[:, j]); ap = lp(app_l[:, j])
            r.update(law_rms=np.sqrt((c_**2).mean()), app_rms=np.sqrt((ap**2).mean()), app_law_rms=np.sqrt(((ap - c_)**2).mean()),
                     app_int_mean=(ap - it).mean(), app_int_rms=np.sqrt(((ap - it)**2).mean()), corr=np.corrcoef(ap, it)[0, 1],
                     app_peak=np.abs(ap).max(), cap_pct=100 * np.abs(ap).max() / CAP[j])
        else:
            ap = lp(app_l[:, j]); r.update(app_rms=np.sqrt((ap**2).mean()), app_peak=np.abs(ap).max(), cap_pct=100 * np.abs(ap).max() / CAP[j])
        rows.append({k: (float(v) if isinstance(v, (np.floating, float, int)) else v) for k, v in r.items()})
    return rows


if __name__ == "__main__":
    import sys
    names = sys.argv[1:] or ["a1", "a2", "a3", "a4", "w1", "w2", "d1", "d2"]
    allr = []
    for nm in names:
        for r in arm_rows(nm):
            allr.append(r)
            s = (f"{r['flight']:14s} {r['joint']}: q_d {r['qd_min']:.1f}..{r['qd_max']:.1f} q {r['q_min']:.1f}..{r['q_max']:.1f} span {r['span']:.0f}% | "
                 f"err {r['e_mean']:+.2f}/{r['e_rms']:.2f}/{r['e_max']:.2f} hold {r['hold_mean']:+.2f}±{r['hold_std']:.2f} | lag {r['lag_s']:.2f}s(c{r['lag_corr']:.2f}) "
                 f"stuck {r['stuck']:.0f}% speed {r['pk_meas']:.1f}/{r['pk_ref']:.1f} over {r['over_hi']:+.1f}/{r['over_lo']:+.1f} stop {r['at_stop']:.1f}%")
            if "ff_opp" in r:
                s += (f" | FFopp {r['ff_opp']:.0f}% FFrms {r['ff_rms']:.3f} | tau law {r['law_rms']:.2f} app {r['app_rms']:.2f} app-law {r['app_law_rms']:.3f} "
                      f"app-int {r['app_int_mean']:+.3f}/{r['app_int_rms']:.3f} corr {r['corr']:.3f} peak {r['app_peak']:.2f}={r['cap_pct']:.0f}%")
            else:
                s += f" | tau app {r['app_rms']:.2f} peak {r['app_peak']:.2f}={r['cap_pct']:.0f}%"
            print(s)
    json.dump(allr, open("../analysis/arm_stats.json", "w"), indent=1)
