"""Shared loaders for the claw hover-precision study (mail-slot sizing, 2026-10-23).

Reuses the 0928 model / loaders (../../../wb_vs_decoupled_flight_20260928/tools/common.py: planner FK on the T650
model with the hardware base_com, joint_states order [j2, j3, j1, j4], hardware->model sign [-1, 1, 1, -1]) and the
existing npz extractions of each flight (extract_bag.py + np1_compat.py). Only the 10-20 flight was extracted here
(tools/run_extract.sh -> ../npz/s1.npz).

Run with PYTHONNOUSERSITE=1 /usr/bin/python3 (numpy 1.21 + apt scipy).

Time base: receive time on the ground-station laptop (the recorder), per bag t0 = first receive stamp of any topic.
The fused odometry's header is on PX4's clock and is NOT used.
"""
import os, sys, math
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARCH = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
OUT = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(ARCH, "wb_vs_decoupled_flight_20260928", "tools"))
import common as C  # noqa: E402
from common import TP, PARAMS, RZM90, JS_IDX, HW_SIGN, quat_R, quat_cont, interp, wrap  # noqa: E402,F401

# id: (label, npz, stack, carried the basket, note, set, extra excluded intervals [s])
#   set "main" = the flights asked for (10-14..10-21); "supp" = the 10-09/10-10 set-down pick-and-place flights
#   (k_R/k_w 1.6/1.2 as on 10-14/15; the 10-16+ pick-and-drop yaml flies 2.134/1.567), added for their LONG holds
#   (PP-1 waited 38 s at the pick) and reported apart.
FLIGHTS = {
    "pd14a": ("1014 PD-1 17:28", "wb_pick_drop_flight_20261014/npz/d1.npz", "fused", False,
              "no payload; drop hold over the FULL bin (aero-disturbed, reported apart), abort, transit, land", "main", []),
    "pd14b": ("1014 PD-2 18:39", "wb_pick_drop_flight_20261014/npz/d2.npz", "fused", False,
              "no payload; reverted by the operator during the pick approach", "main", []),
    "e15":   ("1015 E-1 17:23", "wb_pick_drop_flight_20261015/npz/e1.npz", "fused", False, "no payload, full mission", "main", []),
    "d15":   ("1015 D-1 17:50", "wb_pick_drop_flight_20261015/npz/d1.npz", "fused", True,
              "190 g basket; loaded wrist (q4) swing +-18 deg at 0.5 Hz (fixed 10-16)", "main", []),
    "e16":   ("1016 E-1 13:44", "wb_pick_drop_flight_20261016/npz/e1.npz", "fused", False, "no payload, full mission", "main", []),
    "d16a":  ("1016 D-1 13:51", "wb_pick_drop_flight_20261016/npz/d1.npz", "fused", True, "basket (wrong grip)", "main", []),
    "d16b":  ("1016 D-2 14:00", "wb_pick_drop_flight_20261016/npz/d2.npz", "fused", True, "basket", "main", []),
    "s20":   ("1020 PD 18:22", "mail_delivery_20261023/hover_error/npz/s1.npz", "fused", True, "basket, full mission", "main", []),
    "ff21":  ("1021 FF 16:43", "wb_box_pushing_flight_20261021/npz/b164338.npz", "raw", False,
              "box-pushing free flight: 16 s hover above the (typed) handle", "main", []),
    "bp21":  ("1021 BP 17:01", "wb_box_pushing_flight_20261021/npz/b170123.npz", "raw", False,
              "box push; contact (jaws closed on the fin -> abort open) excluded", "main", []),
    "pp1":   ("1009 PP-1 17:25", "wb_pick_place_flight_20261009/npz/p1.npz", "fused", False,
              "set-down mission, no payload; 38 s wait behind the pick", "supp", []),
    "pp2":   ("1009 PP-2 18:02", "wb_pick_place_flight_20261009/npz/p2.npz", "fused", True,
              "basket; place missed, basket hung on the OPEN fingers 89.5-118.3 s (excluded); PX4 link froze 144.7 s",
              "supp", [(89.5, 118.3)]),
    "pp3":   ("1009 PP-3 18:18", "wb_pick_place_flight_20261009/npz/p3.npz", "fused", False,
              "gripper closed above the arch, nothing lifted; reset and flown home", "supp", []),
    "pp4":   ("1010 PP-4 19:07", "wb_pick_place_flight_20261010/npz/p4.npz", "fused", True,
              "basket; set-down drag, pilot STAB 74.8 s", "supp", []),
}
GRID_DT = 0.02
WB_EY = slice(24, 28)          # wb_control_debug: EE task error (law), [27] = heading row (sine)
WB_XC, WB_XCD = slice(48, 51), slice(45, 48)


def load(fid):
    d = np.load(os.path.join(ARCH, FLIGHTS[fid][1]))
    t0 = min(d[k][0] for k in d.files if k.endswith("__recv") and len(d[k]))
    return d, t0


def edges(d, t0, key):
    out, last = [], None
    if f"{key}__recv" not in d.files:
        return out
    for ti, v in zip(d[f"{key}__recv"] - t0, d[f"{key}__data"]):
        v = str(v)
        if v != last:
            out.append((float(ti), v)); last = v
    return out


def phases(d, t0):
    """[(ta, tb, label)] from the mission status (pick_place or push_pull): FLYING x / WAITING x / DONE x / ..."""
    key = "pp_status" if "pp_status__recv" in d.files else "pl_pstatus"
    ev = []
    for t, v in edges(d, t0, key):
        tok = v.split(" T=")[0]
        if v.startswith("WAITING"):
            tok = "WAITING " + v.split()[1].rstrip(":")
        elif v.startswith("DONE"):
            tok = "DONE " + v.split()[1]
        elif v.startswith("ABORTING"):
            tok = "ABORTING"
        elif v.startswith("ABORTED"):
            tok = "ABORTED"
        elif v.startswith("FLYING"):
            tok = " ".join(v.split()[:2])
        if not ev or ev[-1][1] != tok:
            ev.append((t, tok))
    tend = float(d[f"{key}__recv"][-1] - t0)
    return [(ev[i][0], ev[i + 1][0] if i + 1 < len(ev) else tend, ev[i][1]) for i in range(len(ev))]


def phase_at(ph, t):
    for a, b, l in ph:
        if a <= t < b:
            return l
    return ""


def direct_span(d, t0):
    md = edges(d, t0, "wbmode")
    a = next(t for t, v in md if v == "DIRECT")
    b = next((t for t, v in md if v == "SAFETY" and t > a), None)
    tw = d["wb__recv"] - t0
    ok = d["wb__data"][:, 0] == 1.0
    tw = tw[ok & (tw >= a)]
    end = float(tw[-1]) if b is None else min(b, float(tw[-1]))
    to = d["odom__recv"] - t0
    gap = np.where((np.diff(to) > 0.5) & (to[:-1] > a) & (to[:-1] < end))[0]
    if len(gap):
        end = float(to[gap[0]]) - 0.05
    return a, end


def vatt_R(d, t0):
    """PX4 vehicle_attitude (NED/FRD) -> t, R_world<-body in ENU/FLU (the planner's / law's attitude)."""
    t = d["vatt__recv"] - t0
    w, x, y, z = d["vatt__q"].T
    A = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1.0]]); B = np.diag([1.0, -1, -1])
    R = np.einsum("ij,njk,kl->nil", A, np.stack([quat_R(*q) for q in zip(w, x, y, z)]), B)
    return t, R


def R_from_quats(Q):
    return np.stack([quat_R(*q) for q in Q])


def interp_R(tq, t, R):
    """Interpolate rotation matrices by interpolating their quaternion (continuity-fixed) and re-normalising."""
    from scipy.spatial.transform import Rotation as Rot
    q = Rot.from_matrix(R).as_quat()          # xyzw
    q = quat_cont(q)
    qi = interp(tq, t, q); qi /= np.linalg.norm(qi, axis=1)[:, None]
    return Rot.from_quat(qi).as_matrix()


def mocap_clean(d, t0, to, Po):
    """/uav_0/mocap without frozen republished samples, flips and jumps: t, P, R. Jump = > 50 mm from the
    odometry at the same time (fused EKF2 or the raw stack's guarded copy)."""
    t = d["mocap__recv"] - t0
    P = np.column_stack([d[f"mocap__pose.position.{k}"] for k in "xyz"])
    Q = np.column_stack([d[f"mocap__pose.orientation.{k}"] for k in "wxyz"])
    ok = np.all(np.isfinite(P), axis=1) & (np.linalg.norm(Q, axis=1) > 0.5)
    rep = np.zeros(len(P), bool); rep[1:] = np.all(P[1:] == P[:-1], axis=1) & np.all(Q[1:] == Q[:-1], axis=1)
    w, x, y, z = Q.T
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
    dev = np.linalg.norm(P - interp(t, to, Po), axis=1)
    keep = ok & ~rep & (tilt < 30) & (dev < 0.05)
    stats = dict(n=int(len(P)), frozen=int((ok & rep).sum()), flipped=int((ok & (tilt >= 30)).sum()),
                 jump=int((ok & ~rep & (tilt < 30) & (dev >= 0.05)).sum()))
    Qk = quat_cont(Q[keep])
    return t[keep], P[keep], R_from_quats(Qk), stats


def gripper_events(d, t0):
    """Gripper motion starts from joint_states (col 4, + = open): [(t, 'open'|'close')]. The motion follows the GS
    command by the GS->arm latency (tens of ms); window ends are taken 0.1 s before the motion start."""
    t = d["js__recv"] - t0; g = d["js__position"][:, 4]
    ok = np.isfinite(g); t, g = t[ok], g[ok]
    ev = []; s = g[0]; i = 0; n = len(g)
    while i < n:
        if abs(g[i] - s) > 0.0015:
            k = max(i - 1, 0)
            direction = "open" if g[i] > s else "close"
            ev.append((float(t[k]), direction))
            # wait until steady again (< 0.5 mm change over 0.3 s)
            j = i
            while j < n:
                jj = np.searchsorted(t, t[j] + 0.3)
                if jj >= n or abs(g[min(jj, n - 1)] - g[j]) < 0.0005:
                    break
                j += 1
            s = g[min(np.searchsorted(t, t[j] + 0.3), n - 1)]
            i = np.searchsorted(t, t[j] + 0.3)
            continue
        i += 1
    return ev


def build(fid):
    """50 Hz grid over DIRECT with: the reference (stream), the measured claw by two estimates, CoM, heading, tilt.
    M = mocap pose (/uav_0/mocap, physical) + encoders; C = odometry position + PX4 attitude + encoders (what the
    planner's arrival_error and the law use)."""
    d, t0 = load(fid)
    a, b = direct_span(d, t0)
    t = np.arange(a + 0.1, b - 0.05, GRID_DT)
    to = d["odom__recv"] - t0
    Po = np.column_stack([d[f"odom__pose.pose.position.{k}"] for k in "xyz"])
    tm, Pm, Rm, mstats = mocap_clean(d, t0, to, Po)
    ta, Ra = vatt_R(d, t0)
    tj = d["js__recv"] - t0
    qj = d["js__position"][:, :4][:, JS_IDX] * HW_SIGN
    okj = np.all(np.isfinite(qj), axis=1); tj, qj = tj[okj], qj[okj]
    tr = d["wbref__recv"] - t0
    g = lambda f: np.column_stack([d[f"wbref__{f}.{k}"] for k in "xyz"])
    ref = dict(x_cd=g("x_cd"), r_ed=g("r_ed"), b1_de=g("b1_de"), b1_d=g("b1_d"))
    S = dict(t=t, a=a, b=b, fid=fid, mocap_stats=mstats, z_body=interp(t, to, Po[:, 2]))
    for k, v in ref.items():
        S[k] = interp(t, tr, v)
    # reference stream freshness (a gap > 0.25 s makes the grid sample invalid)
    gi = np.searchsorted(tr, t); gi = np.clip(gi, 1, len(tr) - 1)
    S["ref_ok"] = (t - tr[gi - 1]) < 0.25
    q = interp(t, tj, qj)
    jgap = np.searchsorted(tj, t); jgap = np.clip(jgap, 1, len(tj) - 1)
    S["q_ok"] = (t - tj[jgap - 1]) < 0.15
    # arm FK once per sample (model frame)
    n = len(t); r0e = np.zeros((n, 3)); r0c = np.zeros((n, 3)); Re = np.zeros((n, 3, 3))
    for k in range(n):
        r0c[k], r0e[k], Re[k] = TP.arm_fk_model(q[k], PARAMS)
    S["q"] = q
    for tag, (tp, P, tR, R) in (("M", (tm, Pm, tm, Rm)), ("C", (to, Po, ta, Ra))):
        Pi = interp(t, tp, P); Ri = interp_R(t, tR, R)
        Rmod = Ri @ RZM90
        claw = Pi + np.einsum("nij,nj->ni", Rmod, r0e)
        com = Pi + np.einsum("nij,nj->ni", Rmod, r0c)
        b1e = np.einsum("nij,nj->ni", Rmod, Re[:, :, 0])
        psi = np.arctan2(Ri[:, 1, 0], Ri[:, 0, 0])            # nose (body +x) heading
        e = claw - S["r_ed"]
        c, s = np.cos(psi), np.sin(psi)
        S[f"e_{tag}"] = e
        S[f"eh_{tag}"] = np.column_stack([c * e[:, 0] + s * e[:, 1], -s * e[:, 0] + c * e[:, 1], e[:, 2]])
        ec = com - S["x_cd"]
        S[f"ecom_{tag}"] = np.column_stack([c * ec[:, 0] + s * ec[:, 1], -s * ec[:, 0] + c * ec[:, 1], ec[:, 2]])
        S[f"head_{tag}"] = np.degrees(wrap(np.arctan2(b1e[:, 1], b1e[:, 0]) - np.arctan2(S["b1_de"][:, 1], S["b1_de"][:, 0])))
        S[f"tilt_{tag}"] = np.degrees(np.arccos(np.clip(Ri[:, 2, 2], -1, 1)))
        S[f"psi_{tag}"] = psi
        if tag == "M":
            gi = np.searchsorted(tp, t); gi = np.clip(gi, 1, len(tp) - 1)
            S["M_ok"] = (t - tp[gi - 1]) < 0.06
        else:
            S["C_ok"] = np.ones(n, bool)
    # law task error (claw vs the law's own, CoM-anchored or world, target), world frame -> heading frame (C psi)
    tw = d["wb__recv"] - t0; D = d["wb__data"]; ok = D[:, 0] == 1.0
    ey = interp(t, tw[ok], D[ok][:, WB_EY])
    c, s = np.cos(S["psi_C"]), np.sin(S["psi_C"])
    S["ey_law"] = np.column_stack([c * ey[:, 0] + s * ey[:, 1], -s * ey[:, 0] + c * ey[:, 1], ey[:, 2]])
    # reference speed (static detection)
    for k in ("x_cd", "r_ed"):
        S[f"v_{k}"] = np.linalg.norm(np.gradient(S[k], t, axis=0), axis=1)
    az = lambda v: np.unwrap(np.arctan2(v[:, 1], v[:, 0]))
    S["w_ref"] = np.degrees(np.maximum(np.abs(np.gradient(az(S["b1_de"]), t)), np.abs(np.gradient(az(S["b1_d"]), t))))
    # the whole-body planner's own state: HOLD = no leg in flight, the streamed reference is a rest (the leg end is
    # the HOLD edge; a min-snap leg's tail is below 5 mm/s for up to ~1.3 s before it, so speed alone is not used)
    ev = edges(d, t0, "pl_status")
    hold = np.zeros(len(t), bool)
    for i, (te, v) in enumerate(ev):
        if v.startswith("HOLD"):
            tb = ev[i + 1][0] if i + 1 < len(ev) else np.inf
            hold |= (t >= te) & (t < tb)
    S["hold"] = hold
    S["phases"] = phases(d, t0)
    S["grip"] = gripper_events(d, t0)
    return d, t0, S
