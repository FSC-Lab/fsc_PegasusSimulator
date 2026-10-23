"""One statistics table for section 3.1: q2/q3 tracking and joint-torque delivery over the circle run, all four flights.

Writes arm_stats.html (the two tables the report embeds between <!--armstats:start--> and <!--armstats:end-->) and arm_stats.json into the current directory, and prints the rows.
Definitions (circle run only, law timeline unless stated):
  span realised   = (max q - min q) / (max q_d - min q_d)
  err             = q - q_d [deg]
  hold err        = the same over the start hold before the run (a static reference): mean = standing offset, std = ripple
  lag             = cross-correlation lag of detrended q against q_d (slow flights only; meaningless once the joint
                    stick-slips, printed as "-")
  stuck           = share of the moving-reference time (|q_d'| above a quarter of its own peak) with the measured joint
                    speed below 1 deg/s
  peak speed      = 99th percentile of the measured |q'| (velocity observer) against the reference's peak
  overshoot       = how far q went past the reference's own extremes (negative = it fell short of them)
  FF opposing     = share of the moving time (|q'| > 1 deg/s) where the low-passed friction feed-forward has the
                    opposite sign to the measured velocity, i.e. is pushing against the real motion
  torques         = torque_lp.py's 10 Hz-filtered law command / intended (law + gravity corr. + friction FF) / applied
                    (Present Current / kappa_tau); caps 2.44 N.m (j2) and 1.42 N.m (j3) = the servo's max_effort
"""
import sys, json, html
sys.path.insert(0, "<scratch>")
from common import *
KT = np.array([162.4, 154.0, 150.5, 153.4]); KPWM = np.array([169.47, 149.70, 135.25, 148.51])
JS = [2, 3, 1, 4]; idx = [JS.index(j) for j in (1, 2, 3, 4)]
CAP = {1: 2.44, 2: 1.42}
RUNS = [("a1", "F1", (35.9, 75.5), (30.84, 35.9)), ("a2", "F2", (22.94, 50.94), (20.27, 22.94)),
        ("a3", "F3", (20.62, 48.62), (18.60, 20.62)), ("a4", "F4", (20.09, 48.08), (17.41, 20.09))]


def lp(x, k=25):
    return np.convolve(x, np.ones(k) / k, "same")


rows = []
for nm, lab, (r0, r1), (h0, h1) in RUNS:
    d, t0, a, b, ex, hold = load(nm)
    t = d["wb__recv"] - t0; D = d["wb__data"]; m = (t > r0) & (t < r1); mh = (t > h0 + 0.1) & (t < h1 - 0.1)
    q = np.degrees(D[:, 5:9]); qd = np.degrees(D[:, 9:13]); tt = t[m]
    # reference rate on a uniform 250 Hz grid with a 0.1 s central difference: the receive stamps jitter, and a raw
    # gradient on them reads hundreds of deg/s on a reference that never exceeds 16
    tu = np.arange(r0, r1, 0.004); qdu = np.column_stack([np.interp(tu, tt, qd[m, k]) for k in range(4)])
    vdu = (np.roll(qdu, -12, 0) - np.roll(qdu, 12, 0)) / (24 * 0.004); vdu[:12] = vdu[12]; vdu[-12:] = vdu[-13]
    vd = np.column_stack([np.interp(tt, tu, vdu[:, k]) for k in range(4)])
    tv = d["velobs__recv"] - t0; vob = np.degrees(d["velobs__velocity"][:, :4]); v = np.column_stack([np.interp(tt, tv, vob[:, k]) for k in range(4)])
    tl = d["law__recv"] - t0; L = d["law__data"]; ml = (tl > r0) & (tl < r1); L = L[ml]; tl = tl[ml]
    tj = d["js__recv"] - t0; app = d["js__effort"][:, :4][:, idx] / KT; tc = d["tcmd__recv"] - t0; cmd = d["tcmd__effort"][:, :4]
    cmd_l = np.column_stack([np.interp(tl, tc, cmd[:, k]) for k in range(4)]); app_l = np.column_stack([np.interp(tl, tj, app[:, k]) for k in range(4)])
    aux = L[:, 18:22] / KPWM; gc = L[:, 22:26] / KPWM
    vl = np.column_stack([np.interp(tl, tv, vob[:, k]) for k in range(4)])
    for j in (1, 2):
        e = q[m, j] - qd[m, j]; eh = q[mh, j] - qd[mh, j]; pk_ref = np.abs(vd[:, j]).max()
        span = (q[m, j].max() - q[m, j].min()) / (qd[m, j].max() - qd[m, j].min())
        x = q[m, j] - q[m, j].mean(); y = qd[m, j] - qd[m, j].mean(); best = (0, -2)
        for Lg in range(0, 1000, 2):
            c = np.corrcoef(x[Lg:], y[:len(y) - Lg])[0, 1]
            if c > best[1]: best = (Lg, c)
        refmov = np.abs(vd[:, j]) > 0.25 * pk_ref; stuck = 100 * (refmov & (np.abs(v[:, j]) < 1.0)).sum() / max(refmov.sum(), 1)
        pk = np.percentile(np.abs(v[:, j]), 99)
        over_hi = q[m, j].max() - qd[m, j].max(); over_lo = qd[m, j].min() - q[m, j].min()
        at_stop = 100 * (q[m, j] >= 49.5).mean()
        ffl = lp(aux[:, j]); mov = np.abs(vl[:, j]) > 1.0
        opp = 100 * ((np.sign(ffl[mov]) * np.sign(vl[mov, j])) < 0).mean() if mov.any() else float("nan")
        c_ = lp(cmd_l[:, j]); it = lp(cmd_l[:, j] + gc[:, j] + aux[:, j]); ap = lp(app_l[:, j])
        rows.append(dict(flight=lab, joint=f"j{j+1}", qd_min=qd[m, j].min(), qd_max=qd[m, j].max(), q_min=q[m, j].min(), q_max=q[m, j].max(),
                         span=100 * span, e_mean=e.mean(), e_rms=np.sqrt((e**2).mean()), e_max=np.abs(e).max(), hold_mean=eh.mean(), hold_std=eh.std(),
                         lag_s=best[0] * 0.004, lag_corr=best[1], stuck=stuck, pk_ref=pk_ref, pk_meas=pk, over_hi=over_hi, over_lo=over_lo, at_stop=at_stop,
                         ff_opp=opp, law_rms=np.sqrt((c_**2).mean()), app_rms=np.sqrt((ap**2).mean()), app_law_rms=np.sqrt(((ap - c_)**2).mean()),
                         app_int_mean=(ap - it).mean(), app_int_rms=np.sqrt(((ap - it)**2).mean()), corr=np.corrcoef(ap, it)[0, 1],
                         app_peak=np.abs(ap).max(), cap_pct=100 * np.abs(ap).max() / CAP[j]))
        r = rows[-1]
        print(f"{lab:36s} {r['joint']}: q_d {r['qd_min']:.1f}..{r['qd_max']:.1f} q {r['q_min']:.1f}..{r['q_max']:.1f} span {r['span']:.0f}% | err {r['e_mean']:+.1f}/{r['e_rms']:.1f}/{r['e_max']:.1f} hold {r['hold_mean']:+.2f}±{r['hold_std']:.2f} | "
              f"lag {r['lag_s']:.2f}s(c{r['lag_corr']:.2f}) stuck {r['stuck']:.0f}% speed {r['pk_meas']:.1f}/{r['pk_ref']:.1f} over +{r['over_hi']:.1f}/-{r['over_lo']:.1f} stop {r['at_stop']:.1f}% FFopp {r['ff_opp']:.0f}% | "
              f"tau law {r['law_rms']:.2f} app {r['app_rms']:.2f} app-law {r['app_law_rms']:.3f} app-int {r['app_int_mean']:+.3f}/{r['app_int_rms']:.3f} corr {r['corr']:.3f} peak {r['app_peak']:.2f}={r['cap_pct']:.0f}%")

json.dump(rows, open("arm_stats.json", "w"), indent=1)
slow = lambda r: r["flight"].startswith(("F1", "F2"))
def n(v, f="{:.1f}"): return f'<td class="n">{f.format(v)}</td>'
def n(v): return f'<td class="n">{v}</td>'
A = ['<p><strong>Joint tracking</strong></p>', '<div class="tscroll"><table>',
     '<tr><th>flight</th><th>joint</th><th>q_d range [°]</th><th>q measured [°]</th><th>span realised</th><th>err mean / rms / max [°]</th><th>hold err mean ± std [°]</th>'
     '<th>lag</th><th>stuck</th><th>peak speed, meas / ref [°/s]</th><th>past ref max / min [°]</th><th>at +50° stop</th><th>FF against motion</th></tr>']
B = ['<p><strong>Torque delivery</strong> (law = the whole-body law\'s command; intended = law + gravity correction + friction feed-forward; applied = from the servo current)</p>', '<div class="tscroll"><table>',
     '<tr><th>flight</th><th>joint</th><th>τ law rms [N·m]</th><th>τ applied rms [N·m]</th><th>applied − law rms [N·m]</th><th>applied − intended mean / rms [N·m]</th><th>corr(applied, intended)</th><th>peak applied [N·m], of cap</th></tr>']
for r in rows:
    head = f"<td class=\"nb\">{html.escape(r['flight'])}</td><td>{r['joint']}</td>"
    A.append("<tr>" + head + n(f"{r['qd_min']:.1f} – {r['qd_max']:.1f}") + n(f"{r['q_min']:.1f} – {r['q_max']:.1f}") + n(f"{r['span']:.0f} %")
             + n(f"{r['e_mean']:+.1f} / {r['e_rms']:.1f} / {r['e_max']:.1f}") + n(f"{r['hold_mean']:+.2f} ± {r['hold_std']:.2f}")
             + n(f"{r['lag_s']:.1f} s" if slow(r) else "—") + n("—" if slow(r) else f"{r['stuck']:.0f} %") + n(f"{r['pk_meas']:.1f} / {r['pk_ref']:.1f}")
             + n(f"{r['over_hi']:+.1f} / {r['over_lo']:+.1f}") + n(f"{r['at_stop']:.1f} %") + n(f"{r['ff_opp']:.0f} %") + "</tr>")
    B.append("<tr>" + head + n(f"{r['law_rms']:.2f}") + n(f"{r['app_rms']:.2f}") + n(f"{r['app_law_rms']:.3f}") + n(f"{r['app_int_mean']:+.3f} / {r['app_int_rms']:.3f}")
             + n(f"{r['corr']:.3f}") + n(f"{r['app_peak']:.2f} = {r['cap_pct']:.0f} %") + "</tr>")
H = A + ["</table></div>"] + B + ["</table></div>"]
open("arm_stats.html", "w", encoding="utf-8").write("\n".join(H) + "\n")
print("wrote arm_stats.html / arm_stats.json")
