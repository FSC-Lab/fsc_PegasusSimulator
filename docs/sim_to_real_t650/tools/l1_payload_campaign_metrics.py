#!/usr/bin/env python3
"""Score a 769 g loaded L1 campaign run recorded by l1_769g_driver.py."""
import sys
import numpy as np

f = np.load(sys.argv[1], allow_pickle=True)
# l1adapt_max_thrust_n of the run being scored -- pass it if the yaml has moved,
# otherwise the "% of clamp" line quietly reports against the wrong ceiling.
BOUND = float(sys.argv[2]) if len(sys.argv) > 2 else 18.0
log, dbg, ev = f["log"], f["dbg"], f["events"]
print("=" * 78)
for e in ev:
    print(e)
print("=" * 78)
if len(log) == 0:
    sys.exit("no odom log")

t, x, y, z, vx, vy, vz, tilt = (log[:, i] for i in range(8))
rx, ry, rz = log[:, 8], log[:, 9], log[:, 10]

# phase boundaries from the event log
ph = {}
for e in ev:
    s = str(e)
    if "PHASE " in s:
        ph[s.split("PHASE ")[1].split()[0]] = float(s.split("[")[1].split("s]")[0])
order = [k for k in ["SOAK", "STEPX", "STEPX_RET", "STEPZ", "STEPZ_RET", "SOAK2",
                     "SAFETY_SETTLE", "LAND_WAIT"] if k in ph]

MASS, G, KF_P, KF_C = 3.802921, 9.81, 4.679931e-05, 5.6159172e-05


def win(a, b):
    return (t >= a) & (t < b)


def dwin(a, b):
    return (dbg[:, 0] >= a) & (dbg[:, 0] < b) if len(dbg) else np.zeros(0, bool)


print("\n--- PHASE SUMMARY (pos error vs streamed ref, tilt, u_L1) ---")
print(f"{'phase':<14}{'dur':>6}{'|e|rms':>9}{'|e|max':>9}{'ez':>8}{'tilt':>7}"
      f"{'uL1_f':>8}{'uL1_sd':>8}{'|uL1_M|':>9}{'motors':>8}")
for i, k in enumerate(order):
    a = ph[k]
    b = ph[order[i + 1]] if i + 1 < len(order) else t[-1]
    m = win(a, b)
    if m.sum() < 5:
        continue
    e = np.stack([x[m] - rx[m], y[m] - ry[m], z[m] - rz[m]])
    en = np.linalg.norm(e, axis=0)
    dm = dwin(a, b)
    if len(dbg) and dm.sum() > 5:
        u = dbg[dm, 1 + 16]
        uM = np.linalg.norm(dbg[dm][:, 1 + 17:1 + 20], axis=1)
        mot = dbg[dm][:, 1 + 32:1 + 36].mean()
        s = f"{u.mean():8.3f}{u.std():8.3f}{uM.max():9.3f}{mot:8.4f}"
    else:
        s = " " * 33
    print(f"{k:<14}{b-a:6.1f}{en.mean()*1000:9.1f}{en.max()*1000:9.1f}"
          f"{(z[m]-rz[m]).mean()*1000:8.1f}{tilt[m].max():7.2f}{s}")

if len(dbg):
    # settled-DIRECT window = last 20 s of SOAK
    a = ph.get("SOAK", t[0])
    b = ph.get("STEPX", t[-1])
    m = dwin(max(a, b - 20.0), b)
    if m.sum() > 10:
        u = dbg[m, 1 + 16]
        fb = dbg[m, 1 + 12]
        mot = dbg[m][:, 1 + 32:1 + 36]
        print("\n--- SETTLED DIRECT HOVER (last 20 s of soak) ---")
        print(f"  u_L1 thrust        {u.mean():8.3f} N  (sd {u.std():.3f}, "
              f"min {u.min():.3f}, max {u.max():.3f})")
        print(f"  predicted demand   {MASS*G*0.2:8.3f} N   [T_cmd*(1-kf_p/kf_c)]")
        print(f"  bound              {BOUND:8.3f} N   -> {u.mean()/BOUND*100:.1f}% of clamp")
        print(f"  samples at >=99% of bound: "
              f"{(u >= 0.99*BOUND).mean()*100:.1f}%   (hardware loaded flight: 82.2% on a 10 N bound)")
        print(f"  baseline f_b       {fb.mean():8.3f} N  (mg = {MASS*G:.3f} N)")
        print(f"  commanded total    {fb.mean()+u.mean():8.3f} N  "
              f"-> physical {(fb.mean()+u.mean())*KF_P/KF_C:8.3f} N")
        print(f"  motors             {mot.mean():8.4f} mean, per-rotor "
              f"{np.round(mot.mean(axis=0),4).tolist()}")
        uM = dbg[m][:, 1 + 17:1 + 20]
        print(f"  u_L1 torque        max |xy| {np.abs(uM[:, :2]).max():.4f} N.m "
              f"(clamp 1.5), |z| {np.abs(uM[:, 2]).max():.4f} (clamp 0.8)")
        gu = dbg[m][:, 1 + 24:1 + 26]
        print(f"  gamma_unmatched    {np.round(gu.mean(axis=0),4).tolist()} N")
        er = np.linalg.norm(dbg[m][:, 1 + 6:1 + 9], axis=1)
        print(f"  |e_R|              max {er.max():.4f}")

# steps
def step_metrics(name, a, b, axis, amp):
    m = win(a, b)
    if m.sum() < 10:
        return
    sig = {"x": x, "y": y, "z": z}[axis][m]
    ref = {"x": rx, "y": ry, "z": rz}[axis][m]
    tt = t[m] - a
    start, target = sig[0], ref[-1]
    d = target - start
    if abs(d) < 1e-6:
        return
    nrm = (sig - start) / d
    try:
        i10 = np.argmax(nrm >= 0.1)
        i90 = np.argmax(nrm >= 0.9)
        rise = tt[i90] - tt[i10] if i90 > i10 else float("nan")
    except Exception:
        rise = float("nan")
    # nrm is already normalised by the SIGNED step, so overshoot is always
    # nrm.max()-1 -- the earlier sign-split reported a degenerate -100% on every
    # return leg (nrm never dips below its own start).
    ovs = (nrm.max() - 1.0) * 100
    fin = sig[-int(len(sig) * 0.2):].mean() - target
    print(f"  {name:<12} rise {rise:5.2f} s  overshoot {ovs:6.1f} %  "
          f"final err {fin*1000:7.1f} mm")

print("\n--- STEPS ---")
if "STEPX" in ph:
    step_metrics("X +0.5", ph["STEPX"], ph.get("STEPX_RET", t[-1]), "x", 0.5)
if "STEPX_RET" in ph:
    step_metrics("X return", ph["STEPX_RET"], ph.get("STEPZ", t[-1]), "x", -0.5)
if "STEPZ" in ph:
    step_metrics("Z +0.25", ph["STEPZ"], ph.get("STEPZ_RET", t[-1]), "z", 0.25)
if "STEPZ_RET" in ph:
    step_metrics("Z return", ph["STEPZ_RET"], ph.get("SOAK2", t[-1]), "z", -0.25)

if "SAFETY_SETTLE" in ph:
    m = win(ph["SAFETY_SETTLE"], ph["SAFETY_SETTLE"] + 8)
    if m.sum() > 5:
        print(f"\n--- DIRECT->SAFETY abort ---\n  z peak {z[m].max():.3f} m "
              f"(ref {rz[m][0]:.3f}), balloon {(z[m].max()-rz[m][0])*1000:.0f} mm, "
              f"tilt max {tilt[m].max():.2f} deg")
print(f"\ntouchdown z = {z[-1]:.3f} m, final tilt {tilt[-1]:.2f} deg, "
      f"run length {t[-1]:.1f} s")
