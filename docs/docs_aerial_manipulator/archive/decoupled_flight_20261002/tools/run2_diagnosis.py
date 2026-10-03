"""Why circle run 2 of the 10-02 13:42 flight looks worse than run 1 (NOT in the report, by request; recorded here).

Prints / writes ../analysis/run2_diagnosis.json:
 1. per-run headline (EE xy / z rms, roll-pitch rms, worst tilt) with and without the 14.9-21 s window
 2. the clock-step chain: every Orin-stamped topic's header stamp jumps ~0.6 s at 73.57 s (recorder receive times stay
    continuous); the PX4-clock fused odometry stamps jump 0.65 s at 84.58 s; the L1 client then re-seeds (u_L1 := 0)
 3. u_L1 / altitude / |e_R| around 84.58 s
 4. the ~0.6 Hz roll-pitch mode: 0.4-1 Hz |e_R| per 6 s window (both 10-02 flights and the 09-28 decoupled flights),
    the external-disturbance proxies outside that band, and the decay-rate damping estimate
Model side: lateral_mode_model.py (least-damped lateral mode of each gain set).
    AM_NPZ=<npz dir> PYTHONNOUSERSITE=1 /usr/bin/python3 run2_diagnosis.py
"""
import json
from c1002 import *
import metrics as M
from scipy.signal import butter, filtfilt, welch, hilbert

R = {}
for nm in ("r1", "r2"):
    o, S = M.analyse(nm); t = S["t"] - S["t"][0]; e = S["re"] - S["red"]; att = S["e"]["att"]; m = o["metrics"]
    R[nm] = {}
    for wn, k in (("all", t >= 0), ("excl_14.9-21", (t < 14.9) | (t > 21))):
        R[nm][wn] = dict(ee_xy=float(1e3 * rms(np.linalg.norm(e[k, :2], axis=1))), ee_z=float(1e3 * rms(e[k, 2])),
                         roll=float(rms(att[k, 0])), pitch=float(rms(att[k, 1])))
    R[nm]["tilt_max"] = m["tilt_max_deg"]
    print(nm, R[nm])

d, t0 = load("r1")
steps = {}
for k in ("js", "parmref", "wbref", "pcref_dir", "pl_current_base", "odom"):
    t = d[f"{k}__recv"] - t0; h = d[f"{k}__hdr"]; m = (t > 69.48) & (t < 96.8); dh = np.diff(h[m]); i = int(np.argmax(dh))
    steps[k] = dict(t=float(t[m][1:][i]), stamp_step_s=float(dh[i]), recv_gap_max_s=float(np.diff(t[m]).max()))
print("stamp steps in run 2:", steps)

tl = d["l1__recv"] - t0; L = d["l1__data"]
pre = (tl > 84.0) & (tl < 84.57); post = (tl > 84.59) & (tl < 84.8)
reset = dict(uL1_thrust_before=float(L[pre, 16].mean()), uL1_thrust_after=float(L[post, 16].min()),
             uL1_pitch_before=float(L[pre, 18].mean()), uL1_yaw_before=float(L[pre, 19].mean()),
             ep_z_min_mm=float(1e3 * L[(tl > 84.5) & (tl < 88), 2].min()), eR_max=float(np.linalg.norm(L[(tl > 84.5) & (tl < 88), 6:9], axis=1).max()),
             uL1_thrust_run1_mean=float(L[(tl > 28.6) & (tl < 55.8), 16].mean()))
print("L1 reset at 84.58 s:", reset)

b, a = butter(2, [0.4 / 125, 1.0 / 125], btype="band")
mode = {}
for nm, (A, B) in (("r1", (12, 96.8)), ("tp", (15.2, 96.8)), ("d1", (10.7, 70.7)), ("d2", (9.1, 57.1))):
    dd, tt0 = load(nm); tq = dd["l1__recv"] - tt0; Lq = dd["l1__data"]; tu = np.arange(A, B, 0.004)
    eR = np.column_stack([filtfilt(b, a, interp(tu, tq, Lq[:, 6 + i])) for i in (0, 1)])
    mode[nm] = [[float(a0), float(1e3 * rms(np.linalg.norm(eR[(tu >= a0) & (tu < a0 + 6)], axis=1)))] for a0 in np.arange(A, B - 3, 6)]
    print(nm, "0.4-1 Hz |e_R| [mrad] per 6 s:", [round(v, 1) for _, v in mode[nm]])
tu = np.arange(55, 70, 0.004); ey = filtfilt(b, a, interp(tu, tl, L[:, 7])); env = np.abs(hilbert(ey))
i0 = np.argmax(env * ((tu > 59) & (tu < 63))); j = (tu > tu[i0] + 0.2) & (tu < tu[i0] + 3.2)
sig = -np.polyfit(tu[j] - tu[i0], np.log(env[j]), 1)[0]
damp = dict(peak_t=float(tu[i0]), decay_rate_1_s=float(sig), f_hz=0.62, zeta=float(sig / (2 * np.pi * 0.62)))
print("hold-2 burst decay:", damp)

out = dict(runs=R, stamp_steps=steps, l1_reset=reset, mode_band_mrad=mode, damping_from_decay=damp)
json.dump(out, open("../analysis/run2_diagnosis.json", "w"), indent=1)
