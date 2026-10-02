"""q2/q3 tracking and joint-2/3 torques, all four flights -> arm_trk.json for the report's interactive figure in 3.1.

Angles: the law's own q and q_d (wb_control_debug[5..12], deg).
Torques, the same processing as torque_lp.py (0.1 s moving average = its 10 Hz low-pass, on the law_debug timeline):
  law      = the whole-body law's streamed torque command (external_torque_controller/joint_torque_command)
  intended = law + gravity-scale correction + friction feed-forward/dither (law_debug[22..25] and [18..21], / kappa_pwm)
  applied  = Present Current (joint_states effort, broadcaster order [j2,j3,j1,j4]) / kappa_tau
Everything is then binned to 0.04 s means (25 Hz). Run from the directory that should receive arm_trk.json.
"""
import sys, json; sys.path.insert(0, "<scratch>")
from common import *
KT = np.array([162.4, 154.0, 150.5, 153.4]); KPWM = np.array([169.47, 149.70, 135.25, 148.51])
JS = [2, 3, 1, 4]; idx = [JS.index(j) for j in (1, 2, 3, 4)]
BIN = 0.04
FL = [("a1", "Flight 1 · 30° ± 10°, 48 s · stopped at 1.57 laps", 24.8, 78.05,
       [("go-to-start", 24.81, 30.84, "goto"), ("", 30.84, 35.9, "hold"), ("circle run", 35.9, 75.51, "run"), ("mocap frozen", 75.51, 78.05, "frozen")]),
      ("a2", "Flight 2 · 30° ± 10°, 48 s · one lap = half a cycle", 16.2, 57.7,
       [("go-to-start", 16.17, 20.27, "goto"), ("", 20.27, 22.94, "hold"), ("circle run", 22.94, 50.94, "run"), ("", 50.94, 57.7, "hold")]),
      ("a3", "Flight 3 · 25° ± 15°, 12 s · one lap", 12.2, 52.1,
       [("go-to-start", 12.24, 18.60, "goto"), ("", 18.60, 20.62, "hold"), ("circle run", 20.62, 48.62, "run"), ("", 48.62, 52.1, "hold")]),
      ("a4", "Flight 4 · 25° ± 15°, 6 s · one lap", 12.7, 55.5,
       [("go-to-start", 12.74, 17.41, "goto"), ("", 17.41, 20.09, "hold"), ("circle run", 20.09, 48.08, "run"), ("", 48.08, 55.5, "hold")])]


def lp(x, k=25):
    return np.convolve(x, np.ones(k) / k, "same")


out = []
for nm, title, w0, w1, ph in FL:
    d, t0, a, b, ex, hold = load(nm)
    t = d["wb__recv"] - t0; D = d["wb__data"]
    tl = d["law__recv"] - t0; L = d["law__data"]; ml = (tl > w0 - 0.5) & (tl < w1 + 0.5); L = L[ml]; tl = tl[ml]
    tj = d["js__recv"] - t0; app = d["js__effort"][:, :4][:, idx] / KT
    tc = d["tcmd__recv"] - t0; cmd = d["tcmd__effort"][:, :4]
    cmd_l = np.column_stack([np.interp(tl, tc, cmd[:, k]) for k in range(4)])
    app_l = np.column_stack([np.interp(tl, tj, app[:, k]) for k in range(4)])
    aux = L[:, 18:22] / KPWM; gc = L[:, 22:26] / KPWM
    law_f = np.column_stack([lp(cmd_l[:, k]) for k in range(4)])
    int_f = np.column_stack([lp(cmd_l[:, k] + gc[:, k] + aux[:, k]) for k in range(4)])
    app_f = np.column_stack([lp(app_l[:, k]) for k in range(4)])
    n = int(np.ceil((w1 - w0) / BIN))

    def binned(tt, v):
        m = (tt > w0) & (tt < w1); k = np.floor((tt[m] - w0) / BIN).astype(int); c = np.bincount(k, minlength=n)[:n]
        s = np.bincount(k, weights=v[m], minlength=n)[:n]; r = np.full(n, np.nan); r[c > 0] = s[c > 0] / c[c > 0]; return r

    T = binned(t, t); ok = ~np.isnan(T)
    q = np.degrees(D[:, 5:9]); qd = np.degrees(D[:, 9:13])
    r2 = lambda v: [None if np.isnan(x) else round(float(x), 2) for x in v[ok]]
    r3 = lambda v: [None if np.isnan(x) else round(float(x), 3) for x in v[ok]]
    rec = dict(title=title, t=[round(float(x), 2) for x in T[ok]], phases=[[l, p0, p1, k] for l, p0, p1, k in ph],
               q=[r2(binned(t, q[:, j])) for j in (1, 2)], qd=[r2(binned(t, qd[:, j])) for j in (1, 2)],
               law=[r3(binned(tl, law_f[:, j])) for j in (1, 2)], intended=[r3(binned(tl, int_f[:, j])) for j in (1, 2)],
               applied=[r3(binned(tl, app_f[:, j])) for j in (1, 2)])
    out.append(rec)
    run = [p for p in ph if p[3] == "run"][0]; mr = (tl > run[1]) & (tl < run[2]); e = app_f[mr] - int_f[mr]; e0 = app_f[mr] - law_f[mr]
    vals = lambda v: [x for x in v if x is not None]
    print(f"{nm}: {len(rec['t'])} bins, law_debug {1/np.median(np.diff(tl)):.0f} Hz; q2 {min(vals(rec['q'][0])):.1f}..{max(vals(rec['q'][0])):.1f}, "
          f"q3 {min(vals(rec['q'][1])):.1f}..{max(vals(rec['q'][1])):.1f} deg; circle run applied-intended j2 {e[:,1].mean():+.3f}/{np.sqrt((e[:,1]**2).mean()):.3f}, "
          f"j3 {e[:,2].mean():+.3f}/{np.sqrt((e[:,2]**2).mean()):.3f} N.m; applied-law j2 rms {np.sqrt((e0[:,1]**2).mean()):.3f}, j3 {np.sqrt((e0[:,2]**2).mean()):.3f}")
json.dump(out, open("arm_trk.json", "w"), separators=(",", ":"), ensure_ascii=False)
import os; print("json bytes", os.path.getsize("arm_trk.json"))
