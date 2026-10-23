"""Data for the report's three interactive figures -> ../analysis/{ee3d,err_ts,arm_trk}.json.

ee3d     today's four circle runs in 3-D (world): EE planned / measured (FK), airframe planned (the bridge conversion)
         / measured (odometry), arm whiskers every 2 s; 10 Hz.
err_ts   EE error in the circle frame (radial = outward from the circle centre, tangential = along the direction of
         travel, vertical) and EE heading error, 25 Hz bin means, time since the run started.
arm_trk  q2/q3 against the reference and the joint-2/3 torques at three points of the chain (0924 report fig. 3's
         processing: 0.1 s moving average, 25 Hz bins) for 0924 F3, 0924 F4 and today's two whole-body flights.
"""
import json
from common import *
import metrics as M

OUT = "../analysis"
r4 = lambda A: np.round(np.asarray(A), 4).tolist()

# ---------------- 3-D -----------------------------------------------------------------------------
ee3d = {}
for nm in ("w1", "w2", "d1", "d2"):
    o, S = M.analyse(nm)
    k = slice(None, None, 10)
    err = 1e3 * np.linalg.norm(S["re"] - S["red"], axis=1)
    ee3d[nm] = dict(label=FLIGHTS[nm], rig="wb" if nm[0] == "w" else "dec", t=np.round(S["t"][k] - S["t"][0], 2).tolist(),
                    ee=r4(S["re"][k]), ee_ref=r4(S["red"][k]), base=r4(S["Pu"][k]), base_ref=r4(S["xb"][k]), err=np.round(err[k], 1).tolist(),
                    rms=round(o["metrics"]["ee_pos"]["rms_norm"], 1))
json.dump(ee3d, open(f"{OUT}/ee3d.json", "w"), separators=(",", ":"))

# ---------------- error time series ---------------------------------------------------------------
ts = {}
for nm in ("w1", "w2", "d1", "d2"):
    o, S = M.analyse(nm); t = S["t"] - S["t"][0]
    rf = S["red"]; rad = rf[:, :2] / np.linalg.norm(rf[:, :2], axis=1)[:, None]; tan = np.column_stack([-rad[:, 1], rad[:, 0]])
    e = S["re"] - rf
    ch = dict(rad=1e3 * np.sum(e[:, :2] * rad, 1), tan=1e3 * np.sum(e[:, :2] * tan, 1), ver=1e3 * e[:, 2], head=S["e"]["ee_head"])
    rb = S["xb"]; radb = rb[:, :2] / np.linalg.norm(rb[:, :2], axis=1)[:, None]; tanb = np.column_stack([-radb[:, 1], radb[:, 0]])
    eb = S["Pu"] - rb
    ch.update(brad=1e3 * np.sum(eb[:, :2] * radb, 1), btan=1e3 * np.sum(eb[:, :2] * tanb, 1), bver=1e3 * eb[:, 2])
    n = int(np.ceil(t[-1] / 0.04)); kb = np.minimum((t / 0.04).astype(int), n - 1)
    cnt = np.bincount(kb, minlength=n)
    bm = lambda v: np.round(np.bincount(kb, weights=v, minlength=n) / np.maximum(cnt, 1), 2).tolist()
    ts[nm] = dict(label=FLIGHTS[nm], rig="wb" if nm[0] == "w" else "dec", t=np.round(np.bincount(kb, weights=t, minlength=n) / np.maximum(cnt, 1), 2).tolist(),
                  **{c: bm(v) for c, v in ch.items()})
    print(nm, {c: (round(float(np.mean(v)), 1), round(float(np.sqrt(np.mean(v**2))), 1)) for c, v in ch.items()})
json.dump(ts, open(f"{OUT}/err_ts.json", "w"), separators=(",", ":"))

# ---------------- arm tracking + torques ----------------------------------------------------------
def lp(x, k=25):
    return np.convolve(x, np.ones(k) / k, "same")

BIN = 0.04
arm = []
for nm, title in (("a3", "0924 F3 · 25° ± 15°, 12 s sweep · old tune"), ("a4", "0924 F4 · 25° ± 15°, 6 s sweep · old tune"),
                  ("w1", "WB-1 today · 25° ± 15°, 6 s sweep · new tune"), ("w2", "WB-2 today · 25° ± 15°, 6 s sweep · new tune")):
    d, t0 = load(nm); a, b, c0, c1 = windows(nm)
    st = [e for e in edges(d, t0, "pl_status")]
    g0 = [t for t, v in st if v.startswith("EXECUTING") and t < c0][-1]           # go-to-start
    h0 = [t for t, v in st if v.startswith("HOLD") and t < c0][-1]
    after = [t for t, v in st if t > c1]
    w0, w1 = g0 - 0.5, min(c1 + 4.0, after[0] if after else c1 + 4.0)
    ph = [["go-to-start", g0, h0, "goto"], ["", h0, c0, "hold"], ["circle run", c0, c1, "run"], ["", c1, w1, "hold"]]
    t = d["wb__recv"] - t0; D = d["wb__data"]
    tl = d["law__recv"] - t0; L = d["law__data"]; ml = (tl > w0 - 0.5) & (tl < w1 + 0.5); L = L[ml]; tl = tl[ml]
    tj = d["js__recv"] - t0; app = d["js__effort"][:, :4][:, JS_IDX] / KT
    tc = d["tcmd__recv"] - t0; cmd = d["tcmd__effort"][:, :4]
    cmd_l = interp(tl, tc, cmd); app_l = interp(tl, tj, app); aux = L[:, 18:22] / KPWM; gc = L[:, 22:26] / KPWM
    law_f = np.column_stack([lp(cmd_l[:, k]) for k in range(4)]); int_f = np.column_stack([lp(cmd_l[:, k] + gc[:, k] + aux[:, k]) for k in range(4)])
    app_f = np.column_stack([lp(app_l[:, k]) for k in range(4)])
    n = int(np.ceil((w1 - w0) / BIN))

    def binned(tt, v):
        m = (tt > w0) & (tt < w1); k = np.floor((tt[m] - w0) / BIN).astype(int); c = np.bincount(k, minlength=n)[:n]
        s = np.bincount(k, weights=v[m], minlength=n)[:n]; r = np.full(n, np.nan); r[c > 0] = s[c > 0] / c[c > 0]; return r

    T = binned(t, t); ok = ~np.isnan(T)
    q = np.degrees(D[:, 5:9]); qd = np.degrees(D[:, 9:13])
    r2 = lambda v: [None if np.isnan(x) else round(float(x), 2) for x in v[ok]]
    r3 = lambda v: [None if np.isnan(x) else round(float(x), 3) for x in v[ok]]
    arm.append(dict(title=title, t=[round(float(x), 2) for x in T[ok]], phases=[[l, round(p0, 2), round(p1, 2), k] for l, p0, p1, k in ph],
                    q=[r2(binned(t, q[:, j])) for j in (1, 2)], qd=[r2(binned(t, qd[:, j])) for j in (1, 2)],
                    law=[r3(binned(tl, law_f[:, j])) for j in (1, 2)], intended=[r3(binned(tl, int_f[:, j])) for j in (1, 2)],
                    applied=[r3(binned(tl, app_f[:, j])) for j in (1, 2)]))
    print(nm, "arm window", round(w0, 2), round(w1, 2), "bins", int(ok.sum()))
json.dump(arm, open(f"{OUT}/arm_trk.json", "w"), separators=(",", ":"), ensure_ascii=False)
import os
print({f: os.path.getsize(f"{OUT}/{f}") for f in ("ee3d.json", "err_ts.json", "arm_trk.json")})
