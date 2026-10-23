#!/usr/bin/env python3
"""Score an AM-T650 GEOMETRIC+L1 robustness run recorded by l1_payload_campaign_driver.py.

Same mission and the same recorder as the bare-T650 769 g campaign
(l1_payload_campaign_metrics.py), so the two are directly comparable -- but this
one knows the AM plant's numbers and the AM node's LONGER debug array:

  * mass 3.746170 kg (AM_xfwd on T650 motors, arm at home), not the bare+payload
    3.802921 kg (l1adapt_max_thrust_n is 18 N on both since 2026-08-24);
  * l1_control_debug is 44 elements, not 37 -- [36..38] r_os, [39] r_os source
    (0 none / 1 static home fallback / 2 LIVE arm encoders), [40] L1 active,
    [41..43] the INJECTED r_os model mismatch (armff_mismatch_*).

The last three are the whole point of an AM robustness run: they let the score
separate the deliberate CoM-model error from the true r_os the plant flies, and
prove the live-arm path never silently fell back to the home pose.

  /usr/bin/python3 am_l1_robustness_metrics.py runA_am_l1.npz [bound_N]
"""
import sys
import numpy as np

f = np.load(sys.argv[1], allow_pickle=True)
BOUND = float(sys.argv[2]) if len(sys.argv) > 2 else 18.0
log, dbg, ev = f["log"], f["dbg"], f["events"]

# AM-T650 plant, and the two deliberate injections under test.
MASS, G = 3.746170, 9.81
KF_PLANT, KF_CTRL = 4.679931e-05, 5.6159172e-05      # +20% believed by the allocator

print("=" * 78)
for e in ev:
    print(e)
print("=" * 78)
if len(log) == 0:
    sys.exit("no odom log")

t, x, y, z, vx, vy, vz, tilt = (log[:, i] for i in range(8))
rx, ry, rz = log[:, 8], log[:, 9], log[:, 10]

ph = {}
for e in ev:
    s = str(e)
    if "PHASE " in s:
        ph[s.split("PHASE ")[1].split()[0]] = float(s.split("[")[1].split("s]")[0])
order = [k for k in ["SOAK", "STEPX", "STEPX_RET", "STEPZ", "STEPZ_RET", "SOAK2",
                     "SAFETY_SETTLE", "LAND_WAIT"] if k in ph]

win = lambda a, b: (t >= a) & (t < b)                                  # noqa: E731
dwin = lambda a, b: ((dbg[:, 0] >= a) & (dbg[:, 0] < b)                # noqa: E731
                     if len(dbg) else np.zeros(0, bool))

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
    a, b = ph.get("SOAK", t[0]), ph.get("STEPX", t[-1])
    m = dwin(max(a, b - 20.0), b)
    if m.sum() > 10:
        d = dbg[m]
        u, fb = d[:, 1 + 16], d[:, 1 + 12]
        mot = d[:, 1 + 32:1 + 36]
        print("\n--- SETTLED DIRECT HOVER (last 20 s of soak) ---")
        print(f"  u_L1 thrust        {u.mean():8.3f} N  (sd {u.std():.3f}, "
              f"min {u.min():.3f}, max {u.max():.3f})")
        # The allocator believes kf is KF_CTRL, so it delivers kf_p/kf_c of what it
        # commands. To land mg on the airframe the COMMANDED collective must be
        # mg*kf_c/kf_p, and the baseline only ever asks for mg -- the whole excess
        # is u_L1's to find. Writing this as mg*(1 - kf_p/kf_c) is the common slip
        # and under-states it (6.125 N here against a true 7.350 N).
        print(f"  predicted demand   {MASS*G*(KF_CTRL/KF_PLANT-1):8.3f} N   "
              f"[mg*(kf_ctrl/kf_plant - 1)]")
        print(f"  bound              {BOUND:8.3f} N   -> {u.mean()/BOUND*100:.1f}% of clamp")
        print(f"  samples at >=99% of bound: {(u >= 0.99*BOUND).mean()*100:.1f}%")
        print(f"  baseline f_b       {fb.mean():8.3f} N  (mg = {MASS*G:.3f} N)")
        print(f"  commanded total    {fb.mean()+u.mean():8.3f} N  -> physical "
              f"{(fb.mean()+u.mean())*KF_PLANT/KF_CTRL:8.3f} N  "
              f"(hover {MASS*G:.3f} N)")
        print(f"  motors             {mot.mean():8.4f} mean, per-rotor "
              f"{np.round(mot.mean(axis=0),4).tolist()}")
        uM = d[:, 1 + 17:1 + 20]
        print(f"  u_L1 torque        mean {np.round(uM.mean(axis=0),4).tolist()} N.m, "
              f"max |xy| {np.abs(uM[:, :2]).max():.4f} (clamp 1.5), "
              f"|z| {np.abs(uM[:, 2]).max():.4f} (clamp 0.8)")
        Mb = d[:, 1 + 13:1 + 16]
        print(f"  baseline M_b       {np.round(Mb.mean(axis=0),4).tolist()} N.m")
        gm = d[:, 1 + 20:1 + 24]
        print(f"  gamma_matched      {np.round(gm.mean(axis=0),4).tolist()}")
        gu = d[:, 1 + 24:1 + 26]
        print(f"  gamma_unmatched    {np.round(gu.mean(axis=0),4).tolist()} N")
        er = np.linalg.norm(d[:, 1 + 6:1 + 9], axis=1)
        print(f"  |e_R|              max {er.max():.4f}")

        # ---- AM-specific: the CoM-model injection under test ----
        r_os = d[:, 1 + 36:1 + 39]
        src = d[:, 1 + 39]
        inj = d[:, 1 + 41:1 + 44]
        print("\n--- ARM CoM MODEL (AM-only debug fields) ---")
        print(f"  r_os believed      {np.round(r_os.mean(axis=0),5).tolist()} m (FLU)")
        print(f"  injected mismatch  {np.round(inj.mean(axis=0),5).tolist()} m")
        print(f"  r_os TRUE (plant)  {np.round((r_os-inj).mean(axis=0),5).tolist()} m")
        srcs = {0: "none", 1: "static home fallback", 2: "LIVE arm encoders"}
        uniq = sorted(set(np.unique(src).astype(int).tolist()))
        print("  r_os source        "
              + ", ".join(f"{srcs.get(s, s)} ({(src==s).mean()*100:.1f}%)" for s in uniq))
        # What u_L1 must supply on pitch is NOT simply dx_err*f_b. The baseline
        # commands -dx_believed*f_b, but the allocator ALSO carries the kf error,
        # so what the plant actually needs COMMANDED is -dx_true*T*(kf_c/kf_p).
        # The two injections partly offset on this channel; scoring against the
        # raw dx_err*f_b makes a correct cancellation look like a 64% one.
        dx_err = inj[:, 0].mean()
        dx_true = (r_os - inj)[:, 0].mean()
        T_phys = (fb.mean() + u.mean()) * KF_PLANT / KF_CTRL
        need = -dx_true * T_phys * (KF_CTRL / KF_PLANT) - Mb[:, 1].mean()
        print(f"  raw pitch bias      {dx_err*fb.mean():+7.4f} N.m  (dx_err * f_b)")
        print(f"  u_L1 pitch REQUIRED {need:+7.4f} N.m  "
              f"[-dx_true*T*kf_c/kf_p - M_b_y, i.e. after the kf error offsets part]")
        print(f"  u_L1 pitch supplied {uM[:,1].mean():+7.4f} N.m  "
              f"({uM[:,1].mean()/need*100:.0f}% of required)")
        print(f"  residual moment     {(Mb[:,1].mean()+uM[:,1].mean())*KF_PLANT/KF_CTRL + dx_true*T_phys:+7.4f} N.m "
              f"-> parks the integrator-free attitude AND position loops")

def step_metrics(name, a, b, axis):
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
    i10, i90 = np.argmax(nrm >= 0.1), np.argmax(nrm >= 0.9)
    rise = tt[i90] - tt[i10] if i90 > i10 else float("nan")
    ovs = (nrm.max() - 1.0) * 100
    fin = sig[-int(len(sig) * 0.2):].mean() - target
    print(f"  {name:<12} rise {rise:5.2f} s  overshoot {ovs:6.1f} %  "
          f"final err {fin*1000:7.1f} mm")

print("\n--- STEPS ---")
for nm, k0, k1, ax in (("X +0.5", "STEPX", "STEPX_RET", "x"),
                       ("X return", "STEPX_RET", "STEPZ", "x"),
                       ("Z +0.25", "STEPZ", "STEPZ_RET", "z"),
                       ("Z return", "STEPZ_RET", "SOAK2", "z")):
    if k0 in ph:
        step_metrics(nm, ph[k0], ph.get(k1, t[-1]), ax)

if "SAFETY_SETTLE" in ph:
    m = win(ph["SAFETY_SETTLE"], ph["SAFETY_SETTLE"] + 8)
    if m.sum() > 5:
        print(f"\n--- DIRECT->SAFETY abort ---\n  z peak {z[m].max():.3f} m "
              f"(ref {rz[m][0]:.3f}), balloon {(z[m].max()-rz[m][0])*1000:.0f} mm, "
              f"tilt max {tilt[m].max():.2f} deg")
print(f"\ntouchdown z = {z[-1]:.3f} m, final tilt {tilt[-1]:.2f} deg, "
      f"run length {t[-1]:.1f} s")
