#!/usr/bin/env python3
"""Score a C++ FSC-autopilot flight from its bag (scripts/start_vtol_fsc_hil.sh records -a).

    source scripts/rosenv.sh; ~/envs/isaac_env/bin/python3 scripts/ana_fsc_bag.py ~/flight_bags/fsc_cpp/<bag_dir>

Truth is the mocap (Isaac's exact state, ENU); the reference is what scripts/fly_fsc_cpp.py streamed on
/<ns>/fsc_autopilot_ros2/position_controller/reference (the same frame). Each ARMED window of at least
--min_flight s is scored as one flight (a bag can hold several). The reference is held (zero-order) at
each mocap sample, and the flight is split where the reference moves: a MOVE segment (the reference
ramping) is scored as tracking error -- it includes the loop's lag, by design -- and a HOLD segment (the
reference still for >= --min_hold s, above the ground) is scored from the vehicle's ARRIVAL (its first
sample within --tol of the point; the time it took is printed): the steady error. Also: attitude, the yaw drift from the armed
heading, the climb overshoot, the landing point against the arming point, and PX4's arming/nav state.
"""
import argparse, glob, math, os, sqlite3, sys
import numpy as np
from rclpy.serialization import deserialize_message
from fsc_autopilot_ros2_msgs.msg import Mocap, PositionControllerReference
from px4_msgs.msg import VehicleStatus


def rpy(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag")
    ap.add_argument("--ns", default="uav_0")
    ap.add_argument("--tol", type=float, default=0.3, help="m: a hold is scored from the vehicle's first sample this close")
    ap.add_argument("--min_hold", type=float, default=5.0)
    ap.add_argument("--min_flight", type=float, default=20.0, help="s armed below which a window is not scored")
    a = ap.parse_args()
    db = a.bag if a.bag.endswith(".db3") else sorted(glob.glob(os.path.join(a.bag, "*.db3")))[0]
    cur = sqlite3.connect(db).cursor()
    tid = {n: i for i, n in cur.execute("select id, name from topics")}

    def rows(name, cls):
        if name not in tid:
            sys.exit(f"[ana_fsc_bag] {db}: no topic {name}")
        return [(ts / 1e9, deserialize_message(d, cls)) for ts, d in
                cur.execute("select timestamp, data from messages where topic_id=? order by timestamp", (tid[name],))]

    ns = a.ns
    moc = rows(f"/{ns}/mocap", Mocap)
    ref = rows(f"/{ns}/fsc_autopilot_ros2/position_controller/reference", PositionControllerReference)
    st = rows(f"/{ns}/fmu/out/vehicle_status_v1", VehicleStatus)
    t0 = moc[0][0]
    tm = np.array([t - t0 for t, _ in moc])
    P = np.array([[m.pose.position.x, m.pose.position.y, m.pose.position.z] for _, m in moc])
    V = np.array([[m.twist.linear.x, m.twist.linear.y, m.twist.linear.z] for _, m in moc])
    E = np.array([rpy(m.pose.orientation) for _, m in moc])
    tr = np.array([t - t0 for t, _ in ref])
    R = np.array([[r.position.x, r.position.y, r.position.z] for _, r in ref])
    ts = np.array([t - t0 for t, _ in st]); armed = np.array([s.arming_state == 2 for _, s in st])
    navs = np.array([s.nav_state for _, s in st])

    # every armed window (arm -> the disarm after it); a short one (a refused or aborted start) is listed only
    edges = np.flatnonzero(np.diff(np.r_[0, armed.astype(int), 0]))
    wins = [(ts[edges[k]], ts[edges[k + 1]] if edges[k + 1] < len(ts) else ts[-1], edges[k + 1] < len(ts))
            for k in range(0, len(edges), 2)]
    if not wins:
        sys.exit("[ana_fsc_bag] never armed")
    print(f"[ana_fsc_bag] {db}")
    print(f"  mocap {len(moc)} samples ({len(moc) / max(tm[-1], 1e-9):.0f} Hz), reference {len(ref)}, "
          f"{len(wins)} armed window(s): " + ", ".join(f"{t1 - t0_:.0f} s" for t0_, t1, _ in wins))
    for k, (t_arm, t_dis, disarmed) in enumerate(wins, 1):
        if t_dis - t_arm < a.min_flight:
            print(f"  flight {k}: armed {t_dis - t_arm:.1f} s -- not a flight, skipped")
            continue
        score(k, t_arm, t_dis, disarmed, a, tm, P, V, E, tr, R, ts, navs)


def score(k, t_arm, t_dis, disarmed, a, tm, P, V, E, tr, R, ts, navs):
    nav_armed = sorted(set(navs[(ts >= t_arm) & (ts <= t_dis)].tolist()))
    w = (tm >= t_arm) & (tm <= t_dis)
    if len(tr) == 0 or tr[0] > t_dis:
        print(f"  flight {k}: no reference inside the armed window"); return
    idx = np.searchsorted(tr, tm, side="right") - 1
    have = idx >= 0
    w &= have
    Rm = R[np.clip(idx, 0, None)]
    err = np.linalg.norm(P - Rm, axis=1); eh = np.linalg.norm((P - Rm)[:, :2], axis=1); ez = (P - Rm)[:, 2]

    # segments: the reference "moves" where it changed in the last 0.5 s
    moving_ref = np.zeros(len(tr), bool)
    for i in range(1, len(tr)):
        j = np.searchsorted(tr, tr[i] - 0.5)
        moving_ref[i] = np.linalg.norm(R[i] - R[j]) > 1e-3
    mv = moving_ref[np.clip(idx, 0, None)]
    segs = []; i = np.flatnonzero(w)[0]; last = np.flatnonzero(w)[-1]
    while i <= last:
        j = i
        while j + 1 <= last and mv[j + 1] == mv[i]:
            j += 1
        segs.append((bool(mv[i]), i, j)); i = j + 1

    p_arm = P[np.flatnonzero(w)[0]]; yaw0 = E[np.flatnonzero(w)[0], 2]
    print(f"  FLIGHT {k}: armed {t_arm:.1f} -> {t_dis:.1f} s ({t_dis - t_arm:.0f} s), nav states while armed {nav_armed} (14 = OFFBOARD)")
    print(f"  {'segment':<26}{'dur s':>7}{'arrive':>8}{'mean':>7}{'rms':>7}{'max':>7}{'horiz':>7}{'z mean':>8}   (3-D error, m)")
    holds, moves = [], []
    for moving, i, j in segs:
        sl = np.arange(i, j + 1)
        dur = tm[j] - tm[i]
        r_end = Rm[j]
        arr = ""
        if moving:
            lab = f"MOVE to ({r_end[0]:+.1f},{r_end[1]:+.1f},{r_end[2]:.1f})"
        else:
            if dur < a.min_hold or r_end[2] < p_arm[2] + 0.3:      # a short pause, or a reference on/below the ground
                continue
            lab = f"HOLD ({r_end[0]:+.1f},{r_end[1]:+.1f},{r_end[2]:.1f})"
            inside = np.flatnonzero(err[sl] < a.tol)                # score from the ARRIVAL: the catch-up is the move's lag
            if len(inside):
                arr = f"{tm[sl[inside[0]]] - tm[i]:+7.1f}s"; sl = sl[inside[0]:]
            else:
                arr = "  never"
        if not len(sl):
            continue
        e = err[sl]
        row = (lab, dur, e.mean(), math.sqrt((e ** 2).mean()), e.max(), eh[sl].max(), ez[sl].mean())
        (moves if moving else holds).append(row)
        print(f"  {lab:<26}{dur:7.1f}{arr:>8}{row[2]:7.3f}{row[3]:7.3f}{row[4]:7.3f}{row[5]:7.3f}{row[6]:+8.3f}")

    air = w & (P[:, 2] > p_arm[2] + 0.3)
    att = np.abs(E[air, :2]).max(axis=0) if air.any() else np.zeros(2)
    ydr = np.abs((E[w, 2] - yaw0 + 180) % 360 - 180).max()
    zmax_hold = max((Rm[j][2] for mvg, i, j in segs if not mvg), default=0.0)
    over = P[w, 2].max() - zmax_hold
    land = P[np.flatnonzero(w)[-1]]
    vland = np.linalg.norm(V[np.flatnonzero(w)[-1]])
    print(f"    -- flight {k} summary")
    print(f"    hold steady error: mean of means {np.mean([h[2] for h in holds]):.3f} m, worst {max(h[4] for h in holds):.3f} m "
          f"({len(holds)} holds, scored from the arrival within {a.tol:g} m)" if holds else "    no holds")
    if moves:
        print(f"    tracking while the reference moves: rms {math.sqrt(np.mean([m[3] ** 2 for m in moves])):.3f} m, "
              f"worst {max(m[4] for m in moves):.3f} m (includes the loop's lag)")
    print(f"    attitude airborne: |roll| max {att[0]:.1f} deg, |pitch| max {att[1]:.1f} deg; yaw drift max {ydr:.1f} deg")
    print(f"    climb overshoot above the highest hold {over:+.2f} m")
    print(f"    landing {np.linalg.norm((land - p_arm)[:2]):.2f} m from the arming point (z {land[2]:.2f}, |v| {vland:.2f} m/s), "
          f"disarmed {'yes' if disarmed else 'NO (bag ended armed)'}")


if __name__ == "__main__":
    main()
