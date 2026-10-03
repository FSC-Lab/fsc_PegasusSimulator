"""Every number in ../README.md. Run: PYTHONNOUSERSITE=1 /usr/bin/python3 analyze.py > ../analysis/metrics.txt
Needs p1.npz (this flight) and, for section 7, x_*.npz (0918/0921/0924 whole-body flights) in $AM_NPZ."""
import math
from scipy.signal import lfilter, welch
from common import *

d, t0 = load("p1")
a, b, c, e = windows(d, t0)
tw, D = wbdebug(d, t0)
T = d["tel_state__data"]; tt = d["tel_state__recv"] - t0; ok = np.isfinite(T[:, 0]); T = T[ok]; tt = tt[ok]
tj, ch = pad(d, t0)
tv = d["vatt__recv"] - t0; tilt = tilt_deg(*d["vatt__q"].T)
tsc = d["sc__recv"] - t0; gyro = d["sc__gyro_rad"]


def hdr(s):
    print(f"\n=== {s}")


hdr("1. Timeline")
print("DIRECT %.2f - %.2f s (%.1f s); TELEOP %.2f - %.2f s (%.1f s); exit = operator 'activate Baseline (Safety)'" % (a, b, b - a, c, e, e - c))
for k in ("wbmode", "pl_status", "actlaw"):
    print(" ", k, [(round(t, 2), v) for t, v in edges(d, t0, k)])

hdr("2. Pad feed and usage (TELEOP window)")
dt = np.diff(tj); m = (tj > c) & (tj < e)
print("rc/input: median rate %.1f Hz, max gap %.1f ms (timeout 300 ms); stale/disarmed samples in teleop/state: %d"
      % (1 / np.median(dt), 1e3 * dt.max(), int(np.sum(T[:, TS['joy_fresh']] < 1) + np.sum(T[:, TS['armed']] < 1))))
dtj = np.diff(np.append(tj, tj[-1])); anyact = np.zeros(len(tj), bool)
for k, v in ch.items():
    act = (np.abs(v) > 0) & m; anyact |= act
    print(f"  {k:9s} active {np.sum(dtj[act]):5.1f} s in {len(runs(act)):2d} presses")
print("  any input %.1f s of %.1f s" % (np.sum(dtj[anyact]), e - c))
print("  rates (teleop/state): com_xy %.2f m/s, com_z %.2f m/s, yaw %.0f deg/s, ee %.3f m/s, roll %.0f deg/s, time scale %.2f"
      % tuple(T[0, TS[k]] for k in ("com_speed_xy", "com_speed_z", "yaw_rate", "ee_speed", "roll_rate", "time_scale")))
com = T[:, TS["com"]]; print("  CoM target span xyz [m]", com.max(0) - com.min(0), " yaw target span [deg]", np.degrees(np.ptp(T[:, TS['yaw']])))
print("  walls: arm_blocked bits", sorted(set(T[:, TS['arm_blocked']].astype(int))), "com_blocked", sorted(set(T[:, TS['com_blocked']].astype(int))),
      "; leash active %.1f %%; sigma_nd min %.3f" % (100 * T[:, TS['leash']].mean(), T[:, TS['sigma_nd']].min()))
print("  notes:", [(round(t, 2), v) for t, v in edges(d, t0, "tel_note") if v])

hdr("3. Stability over DIRECT")
dtw = np.diff(tw)
print("law ticks %d at %.1f Hz, max gap %.1f ms; reference stream fresh %.3f %%; arm-reference fresh %.1f %%; observer qdot %.1f %%; chi free %.1f %%"
      % (len(tw), len(tw) / (tw[-1] - tw[0]), 1e3 * dtw.max(), 100 * D[:, 56].mean(), 100 * D[:, 3].mean(), 100 * D[:, 106].mean(), 100 * D[:, 105].mean()))
tr = d["wbref__recv"] - t0; dtr = np.diff(tr)
print("WholeBodyReference %.1f Hz, max gap %.1f ms; peak |x_cd_dot| %s m/s, peak |qdot_d| %s deg/s"
      % (1 / np.median(dtr), 1e3 * dtr.max(), np.round(np.abs(np.column_stack([d[f'wbref__x_cd_dot.{k}'] for k in 'xyz'])).max(0), 3),
         np.round(np.degrees(np.abs(d['wbref__qdot_d']).max(0)), 1)))
M = D[:, 41:45]; tau = D[:, 13:17]
print("rotor saturation ticks %d, unallocated wrench max %s; motors min %s max %s"
      % (int(np.sum(D[:, 51] > 0)), np.abs(D[:, 52:56]).max(0), M.min(0).round(3), M.max(0).round(3)))
print("arm torque |max| %s N.m (clamp 3.0), L1 clamped ticks %d" % (np.abs(tau).max(0).round(3), int(np.sum(D[:, 88] > 0))))
eR = np.linalg.norm(D[:, 28:31], axis=1); mv = (tv > a) & (tv < b); ms = (tsc > a) & (tsc < b)
print("|e_R| rms %.3f max %.3f; tilt max %.2f deg; |gyro| max %.1f deg/s; watchdog/abort messages: none (rosout)"
      % (rms(eR), eR.max(), tilt[mv].max(), np.degrees(np.linalg.norm(gyro[ms], axis=1).max())))

hdr("4. Tracking by phase (law's own errors; EE abs = e_y + e_x, the CoM-anchored EE)")
ex = D[:, 48:51] - D[:, 45:48]; ey = D[:, 24:28]; eq = np.degrees(D[:, 5:9] - D[:, 9:13])
for nm, p0, p1 in (("hold before teleop", a + 0.5, c), ("teleop", c, e), ("hold after teleop", e, b)):
    mm = (tw > p0) & (tw < p1); mvv = (tv > p0) & (tv < p1)
    en = np.linalg.norm(ex[mm], axis=1); ea = np.linalg.norm(ey[mm, :3] + ex[mm], axis=1)
    print(f"{nm:18s} {p1 - p0:5.1f}s | CoM rms {1e3 * rms(en):5.1f} max {1e3 * en.max():5.1f} mm | EE task rms {1e3 * rms(np.linalg.norm(ey[mm, :3], axis=1)):4.1f} mm,"
          f" heading rms {np.degrees(rms(np.arcsin(ey[mm, 3]))):.2f} deg | EE abs rms {1e3 * rms(ea):5.1f} max {1e3 * ea.max():5.1f} mm |"
          f" joint err rms {np.array2string(rms(eq[mm]), precision=2)} deg | tilt max {tilt[mvv].max():.2f}")
tu_ = np.arange(c, e, 0.004)
for ax, nm in enumerate("xyz"):
    u = np.interp(tu_, tw, ex[:, ax]); f, P = welch(u - u.mean(), fs=250, nperseg=250 * 32)
    band = lambda lo, hi: 1e3 * np.sqrt(np.trapz(P[(f >= lo) & (f < hi)], f[(f >= lo) & (f < hi)]))
    k = np.argmax(P[(f > 0.02) & (f < 5)])
    print(f"  CoM error {nm}: spectral peak {f[(f > 0.02) & (f < 5)][k]:.3f} Hz; rms by band <0.05 {band(0, .05):.1f} | 0.05-0.3 {band(.05, .3):.1f} | 0.3-1 {band(.3, 1):.1f} | >1 Hz {band(1, 125):.1f} mm")

hdr("5. Pad -> reference -> vehicle (D-pad window 16-62 s)")
mm = (tw > 16) & (tw < 62)
tgt = np.column_stack([np.interp(tw[mm], tt, com[:, i]) for i in range(3)]); xcd = D[mm, 45:48]; xc = D[mm, 48:51]
tu2 = np.arange(tw[mm][0], tw[mm][-1], 0.004)


def lag(u, v):
    uu = np.interp(tu2, tw[mm], u); vv = np.interp(tu2, tw[mm], v)
    cc = np.correlate(vv - vv.mean(), uu - uu.mean(), "full"); return (np.argmax(cc) - (len(uu) - 1)) * 0.004


for ax in (0, 1):
    g = lambda X: np.gradient(X[:, ax], tw[mm])
    print("  axis %s: lag target->reference %.2f s, reference->measured %.2f s" % ("xy"[ax], lag(g(tgt), g(xcd)), lag(g(xcd), g(xc))))
print("  per tap: 0.11-0.49 s presses at 0.20 m/s -> 28-96 mm target steps; measured - reference rms %s mm" % np.round(1e3 * rms(xc - xcd), 1))

hdr("6. Arm channel, gripper, observer")
S = 1e3 * T[:, TS["s"]]; print("EE target (body FLU, mm) span fwd %.1f, left %.1f, up %.1f; q target range min %s max %s deg"
                               % (np.ptp(S[:, 0]), np.ptp(S[:, 1]), np.ptp(S[:, 2]), np.degrees(T[:, TS['q']].min(0)).round(1), np.degrees(T[:, TS['q']].max(0)).round(1)))
tq, qm, _ = joints_model(d, t0)
for t_ in (62.0, 91.0, 112.0, 118.0):
    i = np.searchsorted(tq, t_); w = np.searchsorted(tw, t_)
    print(f"  t={t_:5.1f}  measured {np.degrees(qm[i]).round(1)}  law reference {np.degrees(D[w, 9:13]).round(1)} deg")
P6 = d["js__position"]; tjs = d["js__recv"] - t0
for t_ in (107.0, 108.5, 110.5):
    print(f"  gripper t={t_}: pos {P6[np.searchsorted(tjs, t_), 4]:+.4f} rad, effort {d['js__effort'][np.searchsorted(tjs, t_), 4]:+.0f}")
mg = (tw > 106) & (tw < 112); print("  CoM error around the gripper presses (106-112 s): max %.1f mm" % (1e3 * np.linalg.norm(ex[mg], axis=1).max()))
tb = d["batt__recv"] - t0; V = d["batt__voltage_v"]
for p0, p1 in ((11, 14), (60, 63), (113, 118)):
    mm2 = (tw > p0) & (tw < p1)
    print(f"  {p0}-{p1}s: d_hat_z {D[mm2, 33].mean():+.2f} N, u1 {D[mm2, 17].mean():.2f} N, battery {V[(tb > p0) & (tb < p1)].mean():.2f} V")
F = np.column_stack([np.interp(tu_, tw, D[:, 97 + i]) for i in range(3)])
for wc in (2.0, 0.25):
    al = math.exp(-wc * 0.004); y = lfilter([1 - al], [1, -al], F, axis=0); n = np.linalg.norm(y[1250:], axis=1)
    print(f"  raw F_hat (collision reading) low-passed at {wc} rad/s: p99 {np.percentile(n, 99):.2f} N, max {n.max():.2f} N")
print("  consumed F_hat_y max %.4f N (free flight: must be 0)" % np.abs(D[:, 58:62]).max())

hdr("7. Feedback")
tv2 = d["vrpn__recv"] - t0; dv = np.diff(tv2); Pv = np.column_stack([d[f'vrpn__pose.position.{k}'] for k in 'xyz'])
print("VRPN %.1f Hz, max gap %.1f ms, repeated poses %d; EKF cs_ev_pos/vel/yaw always %s/%s/%s"
      % (1 / np.median(dv), 1e3 * dv.max(), int(np.all(np.diff(Pv, axis=0) == 0, axis=1).sum()),
         np.unique(d['esf__cs_ev_pos']), np.unique(d['esf__cs_ev_vel']), np.unique(d['esf__cs_ev_yaw'])))

hdr("8. DIRECT -> SAFETY revert, every 4-D hardware flight")
KA, KB = 4.260431e-05, 4.540431e-05
for nm in ("x_20260918_144828", "x_20260921_112328", "x_20260921_115207", "x_20260924_120546", "x_20260924_165654", "p1"):
    try:
        dd, tz = load(nm)
    except FileNotFoundError:
        print("  missing", nm); continue
    md = edges(dd, tz, "wbmode"); rv = [t for t, v in md if v == "SAFETY" and t > 5][-1]
    to = dd["odom__recv"] - tz; z = dd["odom__pose.pose.position.z"]
    mz = (to > rv) & (to < rv + 4.5); z0 = np.interp(rv, to, z)
    tu = dd["ude__recv"] - tz; U = dd["ude__disturbance_estimate.z"]
    tww, DD = wbdebug(dd, tz); mw = (tww > rv - 3) & (tww < rv)
    ta = dd["attsp__recv"] - tz; th = -dd["attsp__thrust_body"][:, 2]
    print(f"  {nm:18s} dip {1e3 * (z[mz].min() - z0):+5.0f} mm at +{to[mz][np.argmin(z[mz])] - rv:.1f} s | motor mean DIRECT {DD[mw, 41:45].mean():.3f} -> SAFETY {np.interp(rv + 0.2, ta, th):.3f}"
          f" | UDE z change by +6 s {np.interp(rv + 6, tu, U) - np.interp(rv, tu, U):+.2f} N (predicted {-DD[mw, 17].mean() * (1 - KA / KB):+.2f})")
