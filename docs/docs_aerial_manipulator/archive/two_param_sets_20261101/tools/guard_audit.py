#!/usr/bin/env python3
"""Replay the SHIPPED contact-task guards on every recorded hardware contact flight (2026-11-01, the
safety audit before the next flight test): which guard would have fired, when, against what the flight
actually did -- and whether any would have fired on a phase that completed normally (a FALSE trip).

Signals the planner uses, from the extracted bags (archive/<flight>/npz/b<tag>.npz, the 10-21 Bag helper):
  tilt         PX4 vehicle_attitude (vatt q)                       planner tilt guard, node soft / hard watchdog
  q2, q3       the arm's joint_states (js)                         q3 static + rate guards (q2 wall credit)
  CoM error    wb_control_debug [48..49] - [45..46]                sway guard, Ready gate, leash, descent guard
  F_hat        wb_control_debug [97..99], low-passed 3 rad/s       force guard
  obj_0 tilt   /obj_0/mocap orientation vs its value at the grasp  rotor / box tilt guard
Thresholds are READ from the hardware yamls (valve_turning / box_pushing). Phases from the planner's
push_pull status edges. Guards the replay cannot do: box lag / cross (need the plan stream), stale /
frozen feeds (need the processor), the emergency-abort escalation (node internal).
    PYTHONNOUSERSITE=1 /usr/bin/python3 guard_audit.py [--md]"""
import glob
import importlib.util
import os
import re
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
A = os.path.abspath(os.path.join(HERE, "..", ".."))
CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
_spec = importlib.util.spec_from_file_location(
    "bp_common_1021", os.path.join(A, "wb_box_pushing_flight_20261021", "tools", "bp_common.py"))
bp = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bp)

FLIGHTS = [  # (folder, tag, task, what happened -- from the flight records)
    ("wb_valve_turning_flight_20261028", "180652", "valve", "turn cut by a FALSE 2 deg rotor-tilt trip (+7.7 s)"),
    ("wb_valve_turning_flight_20261028", "181833", "valve", "turn cut by a FALSE 2 deg rotor-tilt trip (+2.1 s)"),
    ("wb_valve_turning_flight_20261029", "145526", "valve", "contact sway -> arm limits -> q3 rate guard -> watchdog, crash"),
    ("wb_valve_turning_flight_20261031", "174943", "valve", "static hold attempt, wrist swing, no grasp"),
    ("wb_valve_turning_flight_20261031", "175725", "valve", "static hold attempt, wrist swing, no grasp"),
    ("wb_valve_turning_flight_20261031", "180423", "valve", "30 deg turn attempt, wrist swing, no grasp"),
    ("wb_valve_turning_flight_20261031", "211808", "valve", "base ran away in the room hold, q3 through 0, operator abort, SAFETY"),
    ("wb_box_pushing_flight_20261021", "170123", "box", "pushed 0.10 m; q3 rate guard in the release"),
    ("wb_box_pushing_flight_20261023", "172853", "box", "q3 rate guard at the grasp (a stretch onto q2's wall)"),
    ("wb_box_pushing_flight_20261024", "150423", "box", "#1 pushed ~70 mm, box-tilt guard abort"),
    ("wb_box_pushing_flight_20261024", "151159", "box", "#2 missed the fin, q3 rate guard"),
    ("wb_box_pushing_flight_20261024", "151634", "box", "#3 box blown off the table in Go To Start"),
    ("wb_box_pushing_flight_20261024", "162850", "box", "A: pushed, q3 rate guard in the release"),
    ("wb_box_pushing_flight_20261024", "163413", "box", "B: q3 rate guard at the grasp"),
    ("wb_box_pushing_flight_20261024", "164419", "box", "C: q3 rate guard at the grasp"),
    ("wb_box_pushing_flight_20261024_evening", "194310", "box", "280 g: box reached its plan, box-tilt guard 4 s before the end"),
    ("wb_box_pushing_flight_20261024_evening", "195657", "box", "510 g: q3 rate guard at the grasp, q3 < 0, watchdog, SAFETY"),
    ("wb_box_pushing_flight_20261027", "171114", "box", "A: push complete, q3 rate guard after the release (jaws open)"),
    ("wb_box_pushing_flight_20261027", "172227", "box", "B: q3 static guard 1.5 s before the push end"),
]


def yaml_vals(task):
    f = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_%s.yaml"
                     % ("valve_turning" if task == "valve" else "box_pushing"))
    s = open(f).read()
    out = {}
    for k in ("push_pull_guard_tilt_deg", "push_pull_guard_persist_s", "push_pull_guard_q3_min_deg",
              "push_pull_guard_box_persist_s", "push_pull_guard_q3_lookahead_s", "push_pull_guard_q3_rate_min_deg_s",
              "push_pull_guard_q3_rate_persist_s", "push_pull_guard_q3_rate_filter_s", "push_pull_guard_q2_max_deg",
              "push_pull_guard_force_n", "push_pull_guard_force_persist_s", "push_pull_guard_sway_speed",
              "push_pull_guard_sway_filter_s", "push_pull_guard_sway_persist_s", "push_pull_guard_arm_before_handle",
              "push_pull_guard_ready_com_err", "push_pull_ready_max_com_err", "push_pull_ready_max_com_speed",
              "push_pull_ready_com_settle_s", "push_pull_room_hold_leash", "push_pull_guard_box_tilt_deg",
              "system_wd_max_tilt_deg", "system_wd_hard_tilt_deg", "wb_ee_anchor_blend_s", "wb_ee_anchor_fast_blend_s"):
        m = re.search(rf"^    {k}:\s*([^\s#]+)", s, re.M)
        v = m.group(1) if m else None
        out[k] = (None if v is None else (v == "true" if v in ("true", "false") else float(v)))
    # a key the yaml does not write = the node's / planner's declared default
    defaults = {"push_pull_guard_tilt_deg": 15.0, "push_pull_guard_persist_s": 0.10, "push_pull_guard_box_persist_s": 0.20,
                "push_pull_guard_q3_rate_min_deg_s": 5.0, "push_pull_guard_q3_rate_persist_s": 0.05,
                "push_pull_guard_q3_rate_filter_s": 0.02, "push_pull_guard_force_n": 5.0, "push_pull_guard_force_persist_s": 0.30,
                "push_pull_guard_sway_filter_s": 0.25, "push_pull_guard_sway_persist_s": 0.20,
                "push_pull_guard_arm_before_handle": False, "push_pull_ready_max_com_speed": 0.0,
                "push_pull_ready_com_settle_s": 0.3, "system_wd_max_tilt_deg": 15.0, "system_wd_hard_tilt_deg": 0.0,
                "wb_ee_anchor_blend_s": 1.0, "wb_ee_anchor_fast_blend_s": 0.0}
    for k, d in defaults.items():
        if out.get(k) is None:
            out[k] = d
    return out


def quat_tilt(qw, qx, qy, qz):
    return np.degrees(np.arccos(np.clip(1 - 2 * (qx * qx + qy * qy), -1, 1)))


def lp(t, x, tau):
    y = np.array(x, float).copy()
    for k in range(1, len(t)):
        dt = min(max(t[k] - t[k - 1], 0.0), 0.1)
        a = 1 - np.exp(-dt / tau) if tau > 0 else 1.0
        y[k] = y[k - 1] + a * (x[k] - y[k - 1])
    return y


def first_persist(t, cond, persist):
    """first time cond has held for persist s (None if never)"""
    since = None
    for k in range(len(t)):
        if cond[k]:
            if since is None:
                since = t[k]
            if t[k] - since >= persist - 1e-9:
                return float(t[k])
        else:
            since = None
    return None


def phases(b):
    ed = b.edges("pl_pstatus")
    P = {}
    first = lambda pred: next((t for t, s in ed if pred(s)), None)   # noqa: E731
    P["gts"] = first(lambda s: s.startswith("FLYING go_to_start"))
    P["hold"] = first(lambda s: s.startswith("DONE go_to_start"))
    P["ready"] = first(lambda s: s.startswith("FLYING ready"))
    P["ready_end"] = first(lambda s: s.startswith("DONE ready"))
    P["push"] = first(lambda s: s.startswith("FLYING push"))
    P["push_end"] = first(lambda s: s.startswith("DONE push"))
    P["exit"] = first(lambda s: s.startswith("FLYING exit"))
    P["exit_end"] = first(lambda s: s.startswith("DONE exit"))
    P["abort"] = first(lambda s: s.startswith("ABORTING"))
    ab = next((s for t, s in ed if s.startswith("ABORTING")), "")
    P["abort_why"] = ab[:110]
    md = b.edges("wbmode") if b.has("wbmode") else []
    td = next((t for t, s in md if s == "DIRECT"), None)
    P["direct"] = td
    P["safety"] = next((t for t, s in md if s == "SAFETY" and td is not None and t > td), None)
    return P


def audit(folder, tag, task, note, md=False):
    bp.NPZ = os.path.join(A, folder, "npz")
    b = bp.Bag(tag)
    Y = yaml_vals(task)
    P = phases(b)
    hold, ready, ready_end = P["hold"], P["ready"], P["ready_end"]
    on_handle0 = ready_end if ready_end is not None else None          # the claw on the handle from DONE ready
    on_handle1 = P["exit"] if P["exit"] is not None else (P["abort"] if P["abort"] else None)
    end = P["safety"] or (b.t("wb")[-1] if b.has("wb") else None)
    rows = []

    def in_(t, a, z):
        return (t >= a) & (t <= z) if a is not None and z is not None else np.zeros(len(t), bool)

    # ---- tilt (PX4 attitude) ---------------------------------------------------------------
    tv = b.t("vatt"); q = np.array(list(b.f("vatt", "q")), float)
    tilt = quat_tilt(q[:, 0], q[:, 1], q[:, 2], q[:, 3])
    eng = (tv >= (P["gts"] or np.inf)) & (tv <= (end or np.inf))
    lim = Y["push_pull_guard_tilt_deg"]
    t_tilt = first_persist(tv, eng & (tilt > lim), Y["push_pull_guard_persist_s"])
    t_soft = next((float(x) for x, c in zip(tv, (tv >= (P["direct"] or np.inf)) & (tilt > Y["system_wd_max_tilt_deg"])) if c), None)
    t_hard = next((float(x) for x, c in zip(tv, (tv >= (P["direct"] or np.inf)) & (tilt > Y["system_wd_hard_tilt_deg"])) if c), None) \
        if Y["system_wd_hard_tilt_deg"] > 0 else None
    rows.append(("planner tilt %g deg / %.1f s" % (lim, Y["push_pull_guard_persist_s"]), t_tilt))
    rows.append(("node soft tilt %g deg (-> emergency abort)" % Y["system_wd_max_tilt_deg"], t_soft))
    rows.append(("node hard tilt %g deg (-> SAFETY)" % Y["system_wd_hard_tilt_deg"], t_hard))

    # ---- arm joints --------------------------------------------------------------------------
    tj = b.t("js"); names = [s.strip(" '[]") for s in str(b.f("js", "name")[0]).split(",")]
    Pj = np.degrees(np.array(list(b.f("js", "position")), float))
    q2, q3 = Pj[:, names.index("joint2")], Pj[:, names.index("joint3")]
    before = bool(Y["push_pull_guard_arm_before_handle"])
    arm_a = hold if before else on_handle0
    arm_z = on_handle1 if on_handle1 is not None else end
    armed = in_(tj, arm_a, arm_z)
    q3lim = Y["push_pull_guard_q3_min_deg"] or 0.0
    t_q3s = first_persist(tj, armed & (q3 < q3lim), Y["push_pull_guard_box_persist_s"]) if q3lim > 0 else None
    rows.append(("q3 static < %g deg / %.2f s%s" % (q3lim, Y["push_pull_guard_box_persist_s"], " (from the room hold)" if before else " (on the handle)"), t_q3s))
    tau = Y["push_pull_guard_q3_lookahead_s"] or 0.0
    if tau > 0:
        fil = Y["push_pull_guard_q3_rate_filter_s"]
        v3 = np.zeros(len(tj)); v2 = np.zeros(len(tj))
        for k in range(1, len(tj)):
            dt = tj[k] - tj[k - 1]
            if 0 < dt < 0.1:
                a = 1 - np.exp(-dt / fil)
                v3[k] = v3[k - 1] + a * ((q3[k] - q3[k - 1]) / dt - v3[k - 1])
                v2[k] = v2[k - 1] + a * ((q2[k] - q2[k - 1]) / dt - v2[k - 1])
        q2max = Y["push_pull_guard_q2_max_deg"] or 0.0
        credit = np.where((q2max > 0) & (q2 <= q2max + 1.0), np.maximum(0.0, q2 + tau * v2 - q2max), 0.0)
        pred = q3 + tau * v3 + credit
        cond = armed & (pred < q3lim) & (v3 < -Y["push_pull_guard_q3_rate_min_deg_s"])
        t_q3r = first_persist(tj, cond, Y["push_pull_guard_q3_rate_persist_s"])
        rows.append(("q3 rate: pred(%.1f s) < %g deg, falling > %g deg/s, q2 wall %g" % (tau, q3lim, Y["push_pull_guard_q3_rate_min_deg_s"], q2max), t_q3r))
    # ---- the law's CoM error ------------------------------------------------------------------
    tw = b.t("wb"); W = b.f("wb", "data")
    n = len(tw)
    if W.dtype == object:
        Wm = np.full((n, 131), np.nan)
        for i, r in enumerate(W):
            r = np.asarray(r, float); Wm[i, :min(len(r), 131)] = r[:131]
    else:
        Wm = np.full((n, max(131, W.shape[1])), np.nan); Wm[:, :W.shape[1]] = W
    dm = Wm[:, 0] == 1.0
    e = Wm[:, 48:50] - Wm[:, 45:47]
    en = np.linalg.norm(e, axis=1)
    en[~dm] = np.nan
    # the sway rate: the planner's 0.25 s filter on the finite difference of e
    sf = Y["push_pull_guard_sway_filter_s"] or 0.25
    v = np.zeros(2); vn = np.full(n, np.nan)
    for k in range(1, n):
        dt = tw[k] - tw[k - 1]
        if 0 < dt < 0.1 and np.all(np.isfinite(e[k])) and np.all(np.isfinite(e[k - 1])):
            v += (1 - np.exp(-dt / sf)) * ((e[k] - e[k - 1]) / dt - v)
            vn[k] = np.linalg.norm(v)
    sway = Y["push_pull_guard_sway_speed"] or 0.0
    if sway > 0:
        t_sway = first_persist(tw, in_(tw, arm_a, arm_z) & (np.nan_to_num(vn) > sway), Y["push_pull_guard_sway_persist_s"])
        rows.append(("sway: CoM-error rate > %g m/s / %.2f s" % (sway, Y["push_pull_guard_sway_persist_s"]), t_sway))
    gate = Y["push_pull_ready_max_com_err"] or 0.0
    gate_txt = None
    if gate > 0 and ready is not None:
        k = min(np.searchsorted(tw, ready), n - 1)
        er, vr = en[k] * 1e3, np.nan_to_num(vn[k]) * 1e3
        gate_txt = "at the Ready press: CoM err %.0f mm (gate %.0f), rate %.0f mm/s (gate %.0f) -> %s" % (
            er, gate * 1e3, vr, Y["push_pull_ready_max_com_speed"] * 1e3,
            "REFUSED" if (er > gate * 1e3 or vr > Y["push_pull_ready_max_com_speed"] * 1e3) else "accepted")
    leash = Y["push_pull_room_hold_leash"] or 0.0
    if leash > 0:
        t_leash = first_persist(tw, in_(tw, hold, ready if ready else arm_z) & (np.nan_to_num(en) > leash), 0.1)
        rows.append(("leash: CoM err > %g mm / 0.1 s in the hold (claw back to the vehicle, no abort)" % (leash * 1e3), t_leash))
    dg = Y["push_pull_guard_ready_com_err"] or 0.0
    if dg > 0:
        t_dg = first_persist(tw, in_(tw, ready, ready_end if ready_end else arm_z) & (np.nan_to_num(en) > dg), Y["push_pull_guard_sway_persist_s"])
        rows.append(("descent guard: CoM err > %g mm / %.2f s on the Ready descent" % (dg * 1e3, Y["push_pull_guard_sway_persist_s"]), t_dg))
    # ---- force ----------------------------------------------------------------------------
    F = Wm[:, 97:100]
    Ff = np.zeros(3); Fn = np.full(n, np.nan)
    for k in range(1, n):
        dt = min(max(tw[k] - tw[k - 1], 0.0), 0.1)
        if np.all(np.isfinite(F[k])):
            a = np.exp(-3.0 * dt); Ff = a * Ff + (1 - a) * F[k]; Fn[k] = np.linalg.norm(Ff)
    pa = on_handle0; pz = on_handle1 if on_handle1 is not None else end
    t_force = first_persist(tw, in_(tw, pa, pz) & (np.nan_to_num(Fn) > Y["push_pull_guard_force_n"]), Y["push_pull_guard_force_persist_s"])
    rows.append(("force: |F_hat| LP3 > %g N / %.1f s on the handle" % (Y["push_pull_guard_force_n"], Y["push_pull_guard_force_persist_s"]), t_force))
    # ---- the rotor / box tilt (obj_0 z vs its value at the grasp) -------------------------
    bt = Y["push_pull_guard_box_tilt_deg"] or 0.0
    if bt > 0 and b.has("obj") and on_handle0 is not None:
        to = b.t("obj"); g = lambda k: b.f("obj", k)   # noqa: E731
        ow, ox, oy, oz = (g(f"pose.orientation.{c}") for c in "wxyz")
        # the body z axis in the world
        zx = 2 * (ox * oz + ow * oy); zy = 2 * (oy * oz - ow * ox); zz = 1 - 2 * (ox * ox + oy * oy)
        k0 = min(np.searchsorted(to, on_handle0), len(to) - 1)
        z0 = np.array([zx[k0], zy[k0], zz[k0]])
        ang = np.degrees(np.arccos(np.clip(zx * z0[0] + zy * z0[1] + zz * z0[2], -1, 1)))
        t_bt = first_persist(to, in_(to, on_handle0, pz) & (ang > bt), Y["push_pull_guard_box_persist_s"])
        rows.append(("%s tilt > %g deg / %.2f s vs the grasp" % ("rotor" if task == "valve" else "box", bt, Y["push_pull_guard_box_persist_s"]), t_bt))

    # ---- report ----------------------------------------------------------------------------
    rel = lambda t: ("-" if t is None else ("%+.2f s" % (t - hold) if hold is not None else "%.1f s" % t))   # noqa: E731
    print(f"\n### {folder.replace('wb_', '')} / {tag}  [{task}]  -- {note}")
    ph = "  phases (s from the room hold): " + ", ".join(
        f"{k} {rel(P[k])}" for k in ("gts", "hold", "ready", "ready_end", "push", "push_end", "exit", "abort", "safety") if P[k] is not None)
    print(ph if hold is not None else "  no room hold reached (phases: %s)" % {k: round(v, 1) for k, v in P.items() if isinstance(v, float)})
    if P["abort"]:
        print(f"  recorded abort: {rel(P['abort'])}  {P['abort_why']}")
    if gate_txt:
        print("  " + gate_txt)
    fired = []
    for name, t in rows:
        print(f"  {'FIRES ' + rel(t) if t is not None else '  -   ':>14s}  {name}")
        if t is not None:
            fired.append((t, name))
    # the verdict: before / after the recorded abort or SAFETY, false trips on completed phases
    ref = P["abort"] if P["abort"] is not None else P["safety"]
    completed = P["abort"] is None and P["safety"] is not None and P["exit_end"] is not None
    if fired:
        t1, n1 = min(fired)
        if ref is not None:
            print(f"  -> first replayed trip {rel(t1)} ({n1}); recorded {'abort' if P['abort'] else 'SAFETY'} {rel(ref)}: "
                  f"the guard is {'%.2f s EARLIER' % (ref - t1) if t1 < ref else '%.2f s LATER' % (t1 - ref)}")
        else:
            print(f"  -> first replayed trip {rel(t1)} ({n1}) on a flight with no recorded abort: FALSE TRIP?")
    else:
        print("  -> no replayed guard fires" + (" (the flight ended without an abort)" if ref is None else ""))
    return P, rows


if __name__ == "__main__":
    for fl in FLIGHTS:
        try:
            audit(*fl)
        except Exception as ex:  # a bag without the signal
            print(f"\n### {fl[0]} / {fl[1]}: skip -- {type(ex).__name__}: {ex}")
