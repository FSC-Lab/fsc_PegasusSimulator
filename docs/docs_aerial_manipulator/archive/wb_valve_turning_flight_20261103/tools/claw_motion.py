#!/usr/bin/env python3
"""What moves the claw while the vehicle hovers above the handle (11-03 valve flights, + older ones for comparison):
the planner's claw (current_ee, 15 Hz) = base (current_base position) + R0 (current_base attitude) . ee_body (current_ee_body,
the arm's FK in the body frame). The claw's speed over the planner's 0.3 s window, split into
  base   : the base's own translation
  tilt   : the attitude changing with the arm still (dR0 . ee_body)
  arm    : the joints moving (R0 . d ee_body)
Windows: HOVER (DIRECT, arm home, before Go To Start), ABOVE (Go To Start done -> Ready start), ON (Ready done -> abort / turn).
Usage: claw_motion.py <npz>... (default: this folder's four)"""
import sys, os, re, glob
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
import importlib.util
_spec = importlib.util.spec_from_file_location("bpc", os.path.join(HERE, "..", "..", "wb_box_pushing_flight_20261021", "tools", "bp_common.py"))
_c = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_c)

def R(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])

def pose(b, key):
    t = b.t(key)
    p = np.stack([b.f(key, f"pose.position.{c}") for c in "xyz"], 1)
    q = np.stack([b.f(key, f"pose.orientation.{c}") for c in "wxyz"], 1)
    return t, p, q

def phases(b):
    ed = b.edges("pl_pstatus")
    def first(pat, after=0.0):
        for t, s in ed:
            if t >= after and re.search(pat, s): return t
        return None
    d = {}
    d["gts"] = first(r"^FLYING go_to_start")
    d["gts_done"] = first(r"^DONE go_to_start", d["gts"] or 0)
    d["ready"] = first(r"^FLYING ready", d["gts_done"] or 0) if d["gts_done"] else None
    d["ready_done"] = first(r"^DONE ready", d["ready"] or 1e9) if d["ready"] else None
    d["abort"] = first(r"^ABORTING", d["gts"] or 0)
    d["push"] = first(r"^FLYING push", d["gts"] or 0)
    md = b.edges("wbmode")
    d["direct"] = next((t for t, s in md if s == "DIRECT"), None)
    return d

def speed(t, x, win=0.3):
    j = np.searchsorted(t, t - win)
    ok = (t - t[np.clip(j, 0, len(t) - 1)]) >= win - 0.05
    v = np.full(len(t), np.nan)
    v[ok] = np.linalg.norm(x[ok] - x[j[ok]], axis=1) / (t[ok] - t[j[ok]])
    return v

def analyse(path):
    _c.NPZ = os.path.dirname(path)
    tag = os.path.basename(path)[1:-4]
    b = _c.Bag(tag)
    if not b.has("pl_current_ee_body"):
        print(f"{tag}: no current_ee_body"); return
    te, pe, _ = pose(b, "pl_current_ee")
    tb, pb, qb = pose(b, "pl_current_base")
    tk, pk, _ = pose(b, "pl_current_ee_body")
    n = min(len(te), len(tb), len(tk))
    # align by nearest time
    def near(t_src, x_src, t):
        i = np.clip(np.searchsorted(t_src, t), 1, len(t_src) - 1)
        i = np.where(np.abs(t_src[i - 1] - t) < np.abs(t_src[i] - t), i - 1, i)
        return x_src[i]
    t = te
    base = near(tb, pb, t); q = near(tb, qb, t); body = near(tk, pk, t)
    Rs = np.array([R(*qq) for qq in q])
    recon = base + np.einsum("nij,nj->ni", Rs, body)
    err = np.linalg.norm(recon - pe, axis=1)
    win = 0.3
    j = np.searchsorted(t, t - win)
    ok = (t - t[np.clip(j, 0, len(t) - 1)]) >= win - 0.05
    dt = np.where(ok, t - t[j], np.nan)
    v_claw = np.linalg.norm(pe - pe[j], axis=1) / dt
    v_base = np.linalg.norm(base - base[j], axis=1) / dt
    v_tilt = np.linalg.norm(np.einsum("nij,nj->ni", Rs - Rs[j], body[j]), axis=1) / dt
    v_arm = np.linalg.norm(np.einsum("nij,nj->ni", Rs, body - body[j]), axis=1) / dt
    ph = phases(b)
    lev = np.linalg.norm(body, axis=1)
    print(f"===== {tag}  (claw recon vs planner: median {np.nanmedian(err)*1e3:.1f} mm)  phases " +
          ", ".join(f"{k} {v:.1f}" for k, v in ph.items() if v is not None))
    ws = []
    if ph["direct"] and ph["gts"]: ws.append(("HOVER", ph["direct"] + 3, ph["gts"]))
    if ph["gts_done"]: ws.append(("ABOVE", ph["gts_done"], ph["ready"] or ph["abort"] or ph["gts_done"] + 15))
    if ph["ready_done"]: ws.append(("ON", ph["ready_done"], ph["push"] or ph["abort"] or ph["ready_done"] + 10))
    for lab, a, c in ws:
        w = (t >= a) & (t <= c) & ok
        if w.sum() < 3: continue
        f = lambda x: f"{np.nanmedian(x[w])*1e3:5.0f}/{np.nanpercentile(x[w],90)*1e3:5.0f}"
        slow = np.mean(v_claw[w] <= 0.030) * 100
        print(f"  {lab:6s} {a:6.1f}-{c:6.1f} s  claw-from-body {lev[w].mean()*1e3:4.0f} mm | speed med/p90 mm/s: claw {f(v_claw)}  "
              f"base {f(v_base)}  tilt {f(v_tilt)}  arm {f(v_arm)} | claw <=30 mm/s {slow:4.0f}% of samples")
    return

if __name__ == "__main__":
    for p in (sys.argv[1:] or sorted(glob.glob(os.path.join(HERE, "..", "npz", "b*.npz")))):
        analyse(p)
