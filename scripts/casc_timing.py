#!/usr/bin/env python3
"""Measure the CASCADE's realised loop rates and its true actuation latency, from a bag.

The cascade has no single rate: its INNER loop runs on every mocap sample and its OUTER NMPC runs
in a background thread. So `check_timing.py` (which assumes one nominal rate and parses "FULL "
status lines) does not apply.

  MOCAP rate       how fast state arrives  -> the inner loop's ceiling
  ACTUATOR rate    how fast the inner loop actually published
  TILT LATENCY     commanded tilt vs the tilt the vehicle realised, by cross-correlation.
                   This is the documented way to calibrate `latency_s` for this stack, and it
                   matters more than any other parameter: under-compensating is fatal.

Usage: casc_timing.py <bagdir>
"""
import glob
import math
import sqlite3
import sys


def pct(xs, p):
    if not xs:
        return float("nan")
    s = sorted(xs)
    return s[min(len(s) - 1, int(len(s) * p))]


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    bag = sys.argv[1].rstrip("/")
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    db = sorted(glob.glob(bag + "/*.db3"))
    if not db:
        print(f"  no .db3 in {bag}")
        return 1
    con = sqlite3.connect(db[0])
    tp = {r[0]: (r[1], r[2]) for r in con.execute("select id,name,type from topics")}

    def rows(name):
        ids = [k for k, v in tp.items() if v[0] == name]
        if not ids:
            return []
        T = get_message(tp[ids[0]][1])
        return [(t, deserialize_message(d, T)) for t, d in con.execute(
            f"select timestamp,data from messages where topic_id={ids[0]} order by timestamp")]

    # --- window: while the cascade was the active controller ---
    t0 = t1 = None
    for t, m in rows("/uav_0/ground_station/active_controller"):
        # any member of the cascade FAMILY (cascade, cascade_adaptive, ...; 2026-09-15)
        d = str(getattr(m, "data", ""))
        if d.startswith("cascade") and t0 is None:
            t0 = t
        elif t0 is not None and not d.startswith("cascade") and t1 is None:
            t1 = t
    if t0 is None:
        print("  cascade was never active in this bag")
        return 1
    if t1 is None:
        t1 = 1 << 62
    print(f"  cascade active window: {(min(t1, 1<<62)-t0)/1e9:.1f} s")

    def rate(name):
        ts = [t for t, _ in rows(name) if t0 <= t <= t1]
        if len(ts) < 3:
            return None, []
        g = [(ts[i + 1] - ts[i]) / 1e6 for i in range(len(ts) - 1)]
        g = [x for x in g if x > 0.05]
        return (1000.0 / (sum(g) / len(g)) if g else None), g

    for nm, lbl in (("/uav_0/mocap", "MOCAP   "),
                    ("/uav_0/fmu/in/actuator_motors", "ACTUATOR")):
        r, g = rate(nm)
        if r:
            print(f"  {lbl} {r:6.1f} Hz   period median {pct(g,0.5):5.1f}  p95 {pct(g,0.95):5.1f}  "
                  f"max {max(g):6.1f} ms   n={len(g)}")

    # --- true actuation latency: commanded tilt (servo 0) vs realised body response ---
    # The servo command and the vehicle's realised tilt are not both in the bag, but the COMMAND
    # and the resulting pitch-rate response are. Cross-correlate the commanded tilt RATE against
    # the measured forward-acceleration rate: the delay that maximises correlation is the
    # command->effect latency of the whole chain (DDS -> PX4 -> Isaac -> physics -> mocap).
    srv = [(t, m.control[0]) for t, m in rows("/uav_0/fmu/in/actuator_servos") if t0 <= t <= t1]
    moc = [(t, m.twist.linear.x, m.twist.linear.y) for t, m in rows("/uav_0/mocap") if t0 <= t <= t1]
    if len(srv) > 200 and len(moc) > 200:
        import bisect
        st = [t for t, _ in srv]
        best, bl = None, None
        for lag_ms in range(0, 401, 10):
            num = n = 0
            for i in range(1, len(moc) - 1):
                tm, vx, vy = moc[i]
                dt = (moc[i + 1][0] - moc[i - 1][0]) / 1e9
                if dt <= 0:
                    continue
                ax = (moc[i + 1][1] - moc[i - 1][1]) / dt          # forward accel
                j = bisect.bisect_left(st, tm - lag_ms * 1_000_000)
                if j <= 0 or j >= len(srv):
                    continue
                num += ax * srv[j][1]
                n += 1
            if n:
                c = num / n
                if best is None or c > best:
                    best, bl = c, lag_ms
        print(f"  TILT->ACCEL latency  ~{bl} ms   (peak correlation)")
        print(f"  -> latency_s should be >= this; UNDER-compensating is fatal, over is safe")
    # --- ELEVATOR -> PITCH-RATE-DERIVATIVE latency (2026-09-15): the direct moment actuator, so the
    # lag that maximises the NORMALISED cross-correlation of the elevator command against the pitch
    # angular acceleration is the command->effect delay of the chain. Measured against the mocap's
    # rates (the physical chain) and, when the bag has it, against EKF2's odometry rates (what a
    # state_source=px4 node sees -- the number latency_s has to cover on that source).
    try:
        import numpy as np
        ele = [(t, m.control[3]) for t, m in rows("/uav_0/fmu/in/actuator_servos") if t0 <= t <= t1]
        srcs = {"mocap": [(t, m.twist.angular.y) for t, m in rows("/uav_0/mocap") if t0 <= t <= t1]}
        try:
            srcs["odometry"] = [(t, -m.angular_velocity[1]) for t, m in rows("/uav_0/fmu/out/vehicle_odometry") if t0 <= t <= t1]   # FRD -> FLU
        except Exception:      # noqa: BLE001
            pass
        if len(ele) > 200:
            te = np.array([t for t, _ in ele]) / 1e9; ue = np.array([u for _, u in ele])
            # the FIRST STRAIGHT LEG only (engage + 12..37 s): in the turns the elevator follows the
            # bank and the pitch acceleration is everyone's; and band-pass 0.5-8 Hz (a 1 s moving mean
            # removed, a 25 ms one kept) so the correlation is the servo's transient, not the trim
            fs = 200.0; ta = te[0] + 12.0; tb = min(te[-1] - 1.0, te[0] + 37.0)
            tg = np.arange(ta, tb, 1.0 / fs)
            def bp(x):
                k1 = int(fs * 1.0); k2 = max(1, int(fs * 0.025))
                lo = np.convolve(x, np.ones(k1) / k1, mode="same"); hi = np.convolve(x - lo, np.ones(k2) / k2, mode="same")
                return hi - hi.mean()
            e = bp(np.interp(tg, te, ue))
            for name, qs in srcs.items():
                if len(qs) < 200:
                    continue
                tq = np.array([t for t, _ in qs]) / 1e9; q = np.array([v for _, v in qs])
                qd = bp(np.gradient(np.interp(tg, tq, q), 1.0 / fs))
                lags = np.arange(0, 61); cc = []
                for k in lags:
                    a = e[:len(e) - k] if k else e; b = qd[k:]
                    cc.append(float(np.corrcoef(a, b)[0, 1]) if a.std() > 0 and b.std() > 0 else 0.0)
                k = int(lags[int(np.argmax(np.abs(cc)))])
                verdict = "" if abs(cc[k]) >= 0.5 else "  -- |corr| < 0.5: NOT a measurement (closed-loop cross-correlation; inject an elevator doublet in a straight leg to measure it)"
                print(f"  ELEVATOR->PITCH-ACCEL latency ~{1e3 * k / fs:.0f} ms vs the {name} (|corr| {abs(cc[k]):.2f}; first straight leg, 0.5-8 Hz, 5 ms steps){verdict}")
    except Exception as ex:     # noqa: BLE001
        print(f"  (elevator latency not computed: {ex})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
