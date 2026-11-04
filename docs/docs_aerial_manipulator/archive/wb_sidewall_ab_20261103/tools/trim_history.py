#!/usr/bin/env python3
"""The vehicle's standing motor trim and attitude shaking across every flight 10-20 .. 11-03, in time order, from the
free still holds at the HOME arm pose (../../wb_hover_wander_history_20261103/holds.csv, + the 11-02 flights):
each motor's mean command minus the four-motor mean (PX4 Quad-X order: 1 FR, 2 RL, 3 FL, 4 RR), the roll (left-right),
pitch (front-rear) and yaw (CW FL+RR minus CCW FR+RL) differentials, the gyro's 0.5-5 Hz roll/pitch shaking, and the
accelerometer's > 20 Hz vibration."""
import os, sys, csv
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "wb_hover_wander_history_20261103", "tools"))
import history as H
ARCH = os.path.join(HERE, "..", "..")
rows = list(csv.DictReader(open(os.path.join(ARCH, "wb_hover_wander_history_20261103", "holds.csv"))))
def bp(x, lo, hi, dt):
    n = np.arange(len(x)); y = x - np.polyval(np.polyfit(n, x, 1), n)
    F = np.fft.rfft(y); f = np.fft.rfftfreq(len(y), dt); F[(f < lo) | (f > hi)] = 0
    return np.fft.irfft(F, len(y))
def holds_of(b, t, D):
    out = []
    for a, c in H.static_segments(t, D):
        w = (t >= a) & (t <= c)
        if w.sum() < 50 or D[w, 47].mean() < 0.6: continue
        q = np.degrees(D[w, 5:9].mean(0))
        if abs(q[1] - 40) < 5 and abs(q[2] - 40) < 5: out.append((a, c))
    return out
flights = []
seen = set()
for r in rows:
    if r["ts"] in seen: continue
    seen.add(r["ts"]); flights.append((r["ts"], r["task"], r["wall"]))
flights += [("182844", "trk 11-02", "1"), ("183342", "trk 11-02", "1"), ("192323", "PnD 11-02", "1")]
NPZ2 = os.path.join(HERE, "..", "npz")
print(f"{'flight':18s} {'wall':4s} {'home s':>6s} | motor - mean (FR RL FL RR)          | roll(L-R) pitch(F-R) yaw(CW-CCW) | gyro 0.5-5 Hz r/p deg/s | acc >20 Hz m/s2")
for ts, task, wall in flights:
    p = H.find_npz(ts) or (os.path.join(NPZ2, f"b{ts}.npz") if os.path.exists(os.path.join(NPZ2, f"b{ts}.npz")) else None)
    if p is None:
        if ts == "183342": print(f"{task:11s} {ts}  (not extracted)"); continue
        continue
    H._c.NPZ = os.path.dirname(p); b = H._c.Bag(ts)
    try: t, D = H.wb(b)
    except Exception: continue
    hs = holds_of(b, t, D)
    if not hs or not b.has("act_motors"): print(f"{task:11s} {ts}  no home-pose hold"); continue
    tm = b.t("act_motors"); mc = np.stack(b.f("act_motors", "control")).astype(float)[:, :4]
    ts_ = b.t("sc"); g = np.stack(b.f("sc", "gyro_rad")).astype(float); ac = np.stack(b.f("sc", "accelerometer_m_s2")).astype(float)
    m_all, gr, ar, dur = [], [], [], 0.0
    for a, c in hs:
        w = (tm >= a) & (tm <= c); m_all.append(mc[w]); dur += c - a
        ws = (ts_ >= a) & (ts_ <= c)
        if ws.sum() > 200:
            dt = np.median(np.diff(ts_[ws]))
            gr.append([bp(g[ws, k], 0.5, 5, dt).std() for k in (0, 1)])
            ar.append([bp(ac[ws, k], 20, 0.45 / dt, dt).std() for k in (0, 1)])
    mm = np.vstack(m_all).mean(0); rel = mm - mm.mean()
    roll = (mm[2] + mm[1]) - (mm[0] + mm[3]); pitch = (mm[0] + mm[2]) - (mm[1] + mm[3]); yaw = (mm[2] + mm[3]) - (mm[0] + mm[1])
    gr = np.degrees(np.mean(gr, 0)) if gr else [np.nan] * 2; ar = np.mean(ar, 0) if ar else [np.nan] * 2
    print(f"{task:11s} {ts} {'W' if wall == '1' else ' ':4s} {dur:6.1f} | {rel[0]:+.3f} {rel[1]:+.3f} {rel[2]:+.3f} {rel[3]:+.3f}  (mean {mm.mean():.3f}) | "
          f"  {roll:+.3f}    {pitch:+.3f}     {yaw:+.3f}  |   {gr[0]:4.2f} / {gr[1]:4.2f}          | {ar[0]:4.2f} / {ar[1]:4.2f}")
