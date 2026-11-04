#!/usr/bin/env python3
"""Per lap of every matched pair (the scorer's windows, read from ../cache/scores.json), in BODY axes (roll = about the
nose, pitch = about the side), controller-agnostic:
  gyro     roll / pitch rate, 0.5-5 Hz band rms [deg/s]       -- the attitude shaking
  F_imu    body x / y force (m x accelerometer), 0.1-1 Hz [N] -- a sideways push in the body frame
  dm       the motor commands' roll / pitch differential (actuator_motors, PX4 Quad-X order FR RL FL RR):
           mean (the standing trim) and 0.1-3 Hz rms (the effort spent fighting disturbances) [command]
and, from the planner holds of each flight, the standing roll / pitch trim of the motors."""
import os, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); CACHE = os.path.join(HERE, "..", "cache")
M, DT = 3.746, 0.01
SC = json.load(open(os.path.join(CACHE, "scores.json")))
LAB = {"w13": "WB circle 10-28", "w14": "WB circle 10-28", "w15": "WB circle 10-28", "w16": "WB circle 10-28",
       "n1a": "WB circle 11-02", "n1b": "WB circle 11-02", "w17": "WB fig-8 10-28", "w18": "WB fig-8 10-28",
       "x183342_a": "WB fig-8 11-02", "x183342_b": "WB fig-8 11-02", "r9": "DEC circle 10-28", "r10": "DEC circle 10-28",
       "x184129_a": "DEC circle 11-02", "x184129_b": "DEC circle 11-02", "r11": "DEC fig-8 10-28", "r12": "DEC fig-8 10-28",
       "x184743_a": "DEC fig-8 11-02", "x184743_b": "DEC fig-8 11-02"}
def bp(x, lo, hi):
    n = np.arange(len(x)); y = x - np.polyval(np.polyfit(n, x, 1), n)
    F = np.fft.rfft(y); f = np.fft.rfftfreq(len(y), DT); F[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(F, len(y))
cache = {}
def load(key):
    p = os.path.realpath(os.path.join(CACHE, key + ".npz"))
    if p not in cache:
        d = np.load(p, allow_pickle=True); t0 = min(d[k][0] for k in d.files if k.endswith("__recv"))
        ts = d["sc__recv"] - t0; g = np.stack(d["sc__gyro_rad"]).astype(float); a = np.stack(d["sc__accelerometer_m_s2"]).astype(float)
        tm = d["act_motors__recv"] - t0; mc = np.stack(d["act_motors__control"]).astype(float)[:, :4]
        T = np.arange(max(ts[0], tm[0]) + 1, min(ts[-1], tm[-1]) - 1, DT)
        G = np.stack([np.interp(T, ts, g[:, k]) for k in range(2)], 1); A = np.stack([np.interp(T, ts, a[:, k]) for k in range(2)], 1)
        Mo = np.stack([np.interp(T, tm, mc[:, k]) for k in range(4)], 1)
        roll = (Mo[:, 2] + Mo[:, 1]) - (Mo[:, 0] + Mo[:, 3]); pitch = (Mo[:, 0] + Mo[:, 2]) - (Mo[:, 1] + Mo[:, 3])
        sig = dict(gr=bp(G[:, 0], 0.5, 5), gp=bp(G[:, 1], 0.5, 5), fx=bp(A[:, 0] * M, 0.1, 1), fy=bp(A[:, 1] * M, 0.1, 1),
                   dr=bp(roll, 0.1, 3), dp=bp(pitch, 0.1, 3), dr_m=roll, dp_m=pitch)
        cache[p] = (T, sig)
    return cache[p]
rows = []
for key, lab in LAB.items():
    if key not in SC: continue
    a, b = SC[key]["window"]
    T, s = load(key); m = (T >= a) & (T <= b)
    rms = lambda k: np.sqrt(np.mean(s[k][m] ** 2))
    rows.append((lab, key, np.degrees(rms("gr")), np.degrees(rms("gp")), rms("fx"), rms("fy"), s["dr_m"][m].mean(), s["dp_m"][m].mean(), rms("dr"), rms("dp"),
                 SC[key]["platform_roll_deg"], SC[key]["platform_pitch_deg"], SC[key]["ee_pos_mm"]))
print(f"{'lap':24s} | gyro 0.5-5 Hz roll/pitch | F_imu body x/y | motor roll/pitch diff: mean      rms   | track err roll/pitch  EE")
for r in rows:
    print(f"{r[0]:17s} {r[1]:9s} |   {r[2]:4.2f} / {r[3]:4.2f} deg/s    | {r[4]:4.2f} / {r[5]:4.2f} N  |   {r[6]:+6.3f} / {r[7]:+6.3f}   {r[8]:5.3f} / {r[9]:5.3f} | {r[10]:4.2f} / {r[11]:4.2f} deg  {r[12]:5.1f}")
