#!/usr/bin/env python3
"""Write the C++ guard's parity fixture from the Python reference.

For excerpts of recorded hardware flights (the freezes of 09-18 and 09-24, the
flips of 09-21, a clean 09-28 circle) writes test/data/guard_<name>.csv (the
samples, times relative to the first) and guard_<name>_events.csv (the events
guard_reference finds on THOSE ROUNDED values, so the C++ sees exactly the same
input). The estimator's gtest test_mocap_guard replays every stream on the same
4 ms tick grid and must find the same events. The synthetic cases (the
self-test's) are generated inside the gtest itself and only printed here.

Run: PYTHONNOUSERSITE=1 /usr/bin/python3 -I export_guard_fixture.py [out_dir]
(default out_dir: ~/ros2_ws/src/fsc_drone_state_estimator_ros2/test/data)
"""

import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import guard_reference as G  # noqa: E402
import replay_flights as R  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/ros2_ws/src/fsc_drone_state_estimator_ros2/test/data")
HIST = os.path.join(R.ARCH, "wb_pick_drop_flight_20261014", "npz", "hist")


def synthetic():
    rate = 120.0
    t = np.arange(0.0, 20.0, 1.0 / rate)
    p = np.c_[0.2 * np.sin(0.5 * t), 0.1 * t, 1.0 + 0.0 * t]
    q = np.tile([1.0, 0.0, 0.0, 0.0], (len(t), 1))
    out = {"clean": (t, p, q)}
    p1, q1 = p.copy(), q.copy()
    p1[600] += [0.18, 0.0, 0.0]
    q1[600] = [np.cos(np.radians(55)), 0.0, 0.0, np.sin(np.radians(55))]
    out["flip"] = (t, p1, q1)
    p2 = p.copy()
    a, b = 1200, 1200 + int(2.84 * rate)
    p2[a:b] = p2[a - 1]
    out["freeze"] = (t, p2, q)
    for gap in (0.10, 0.40, 1.50):
        keep = ~((t > 5.0) & (t < 5.0 + gap))
        out[f"gap{int(gap * 100):03d}"] = (t[keep], p[keep], q[keep])
    p4 = p.copy()
    p4[1::2] = p4[0:-1:2]
    out["pairs"] = (t, p4, q)
    return out


def recorded():
    want = {   # name: (flight file, window [s] after its first airborne sample)
        "rec_0918_freeze": ("flight_wb_l1_4d_20260918_144828.npz", (30.0, 50.0)),
        "rec_0921_flips": ("flight_wb_l1_4d_circle_20260921_123637.npz", (18.0, 34.0)),
        "rec_0924_freeze": ("flight_wb_l1_4d_circle_20260924_120228.npz", (62.0, 78.0)),
        "rec_0928_clean": ("flight_wb_l1_4d_circle_20260928_171154.npz", (10.0, 25.0)),
    }
    out = {}
    for name, (f, (a, b)) in want.items():
        t, p, v, q = R.load(os.path.join(HIST, f))
        air = np.where(p[:, 2] > R.AIR_Z)[0]
        t0 = max(t[air[0]], t[0] + R.SKIP_START)
        k = (t >= t0 + a) & (t <= t0 + b)
        out[name] = (t[k] - t[k][0], p[k], q[k])
    return out


def write(name, t, p, q):
    path = os.path.join(OUT, f"guard_{name}.csv")
    with open(path, "w") as f:
        f.write("# t,px,py,pz,qw,qx,qy,qz -- times relative [s]\n")
        for i in range(len(t)):
            f.write(f"{t[i]:.6f},{p[i, 0]:.6f},{p[i, 1]:.6f},{p[i, 2]:.6f},"
                    f"{q[i, 0]:.6f},{q[i, 1]:.6f},{q[i, 2]:.6f},{q[i, 3]:.6f}\n")
    # the reference runs on the ROUNDED values, exactly what the C++ reads
    d = np.loadtxt(path, delimiter=",", comments="#")
    tt, pp, qq = d[:, 0], d[:, 1:4], d[:, 4:8]
    ev, chk, _ = G.run_ladder(tt, pp, np.zeros_like(pp), qq, t_start=tt[0], t_end=tt[-1])
    with open(os.path.join(OUT, f"guard_{name}_events.csv"), "w") as f:
        f.write(f"# t_start,t_end,tick = {tt[0]:.6f},{tt[-1]:.6f},0.004; rejected = {chk.n_jump + chk.n_frozen}\n")
        for e in ev:
            f.write(f"{e.kind},{e.t0:.9f},{e.t1:.9f},{e.reason}\n")
    return len(tt), [(e.kind, e.reason) for e in ev], chk.n_jump + chk.n_frozen


def main():
    os.makedirs(OUT, exist_ok=True)
    for name, (t, p, q) in synthetic().items():   # printed only (the gtest builds them)
        ev, chk, _ = G.run_ladder(t, p, np.zeros_like(p), q)
        print(f"{name:18s} synthetic, events: {', '.join(f'{e.kind}/{e.reason}' for e in ev) or 'none'}")
    for name, (t, p, q) in recorded().items():
        n, ev, rej = write(name, t, p, q)
        print(f"{name:18s} {n:6d} samples, {rej:4d} rejected, events: "
              f"{', '.join(f'{k}/{r}' for k, r in ev) or 'none'}")


if __name__ == "__main__":
    main()
