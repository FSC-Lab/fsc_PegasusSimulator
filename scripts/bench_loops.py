#!/usr/bin/env python3
"""The compute budget of the flown loops, per component, on THIS machine (hardware-transfer piece 6,
2026-09-16) -- the number to scale for the Jetson Orin (roughly half the single-thread speed of this
desktop) before promising 150-200 Hz for the inner loop.

  ~/envs/isaac_env/bin/python3 scripts/bench_loops.py [--ticks 2000] [--outer 40] [--profile] [--load] [--pin N] [--json F]

  --load   time the inner step WHILE a background thread solves the outer NMPC back to back (what the
           node does: CasADi releases the GIL, the two share the machine) -- the number that matters
  --pin N  restrict the process to N cores first (a load proxy for a small companion computer)
  --json F write every median/p95 to F (tests/test_bench_budget.py reads it)

Times, in the recommended configuration (imu-mdob + EKF predictor + L1, the offset 100 mm tank):
  inner   base.inner_step per tick (the geometric law, the allocation with the prop inflow, the L1
          adapt = moment observer + IMU force residual + EKF predictor + cancellation)
  parts   the L1 adapt alone; the EKF predictor step alone; the allocation inverse alone
  outer   OuterNMPC.solve warm (the JIT solver), median and p95 -- the 20 Hz thread's budget
and prints each as a fraction of the 5 ms tick (200 Hz) and the 6.7 ms tick (150 Hz), plus the
x2 Orin estimate. --profile adds a cProfile top-15 of the inner step."""
import argparse
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
sys.path.insert(0, ROOT)
from controller_path import controller_dir     # noqa: E402

ap = argparse.ArgumentParser()
ap.add_argument("--ticks", type=int, default=2000)
ap.add_argument("--outer", type=int, default=40)
ap.add_argument("--profile", action="store_true")
ap.add_argument("--load", action="store_true")
ap.add_argument("--pin", type=int, default=0)
ap.add_argument("--json", default="")
a = ap.parse_args()
if a.pin > 0:
    os.sched_setaffinity(0, set(range(a.pin)))

os.environ.setdefault("VTOL_TANK_KG", "0.502"); os.environ.setdefault("VTOL_TANK_R", "0.10"); os.environ.setdefault("VTOL_TANK_DX", "0.3")
sys.argv = ["bench", "--est", "mdob", "--slosh_pred", "on", "--test"]        # what the variant modules read at import
sys.path.insert(0, controller_dir())
# the PACKAGE modules: the variants re-point `base.L1Aug` / `base.inner_step` on
# px4_vtol_direct_actuator.nmpc_cascade_node, so a top-level import of the same file would benchmark the base L1
from px4_vtol_direct_actuator import nmpc_cascade_node as base            # noqa: E402
from px4_vtol_direct_actuator import nmpc_cascade_steso_node as sv        # noqa: E402
from px4_vtol_direct_actuator import nmpc_cascade_imu_node as iv          # noqa: E402,F401
so3 = base.so3
if so3._AERO_NUM is None:          # the node builds it through the schedule; the bench builds it directly
    so3._AERO_NUM = so3._aero_numeric()

print(f"loops benchmark on {os.uname().nodename} ({os.cpu_count()} cpus{', pinned to ' + str(a.pin) if a.pin else ''}), python {sys.version.split()[0]}, numpy {np.__version__}")
RESULT = {"host": os.uname().nodename, "cpus": os.cpu_count(), "pin": a.pin, "load": a.load, "numpy": np.__version__, "python": sys.version.split()[0]}
print(f"  L1 class {base.L1Aug.__name__}, inner_step {base.inner_step.__module__}")

# a wing-borne state: 16 m/s, tilt 90, pitch 7 deg, small rates; the airframe's own constants
th = math.radians(7.0); c, s_ = math.cos(th), math.sin(th)
R = np.array([[c, 0.0, -s_], [0.0, 1.0, 0.0], [s_, 0.0, c]])
w = np.array([0.01, -0.02, 0.005]); v = np.array([16.0, 0.0, 0.0]); vb = R.T @ v
alloc = base.RotorAlloc()
l1 = base.L1Aug(wc=2.0, ks=6.0, delay=0.049, tau_spin=0.12, tau_surf=0.03)
if hasattr(l1, "R"):
    l1.R = R
g = dict(kR_roll=3.0, kW_roll=1.8, kR_pitch=3.0, kW_pitch=1.8, kR_yaw=2.0, kW_yaw=0.5, kR_hover=0.0, latency=0.049,
         k_bank=2.0, k_bank_d=1.0, bank_ff=0.0, turn_rate_ff=0.0, k_head=1.0, k_head_d=0.3, turn_rate_max=math.radians(40.0),
         kI_roll=0.8, kI_yaw=0.4, i_band=math.radians(12.0), i_max=1.5, fx_budget=4.0, pitch_ratio=1.0, pitch_zeta=0.7,
         bank_max=math.radians(25.0))
Tf, Tr, beta, de = 4.0, 2.0, math.pi / 2, 0.02
base.IMU_ACCEL = np.array([0.5, 0.0, 9.9]); base.IMU_ACCEL_T = time.monotonic()
ei = np.zeros(3)


def tick():
    global ei
    base.IMU_ACCEL_T = time.monotonic()
    mot, ail, elev, rud, ei = base.inner_step(R, w, th, 0.0, beta, 16.0, Tf, Tr, de, g, alloc, vb=vb, ei=ei, dt=0.005, l1=l1)
    return mot


for _ in range(200):          # warm up (numba/casadi caches, allocations)
    tick()
t = []
for _ in range(a.ticks):
    t0 = time.perf_counter(); tick(); t.append(time.perf_counter() - t0)
t = np.array(t) * 1e3


def row(name, arr):
    med, p95, mx = float(np.median(arr)), float(np.percentile(arr, 95)), float(arr.max())
    RESULT[name.strip()] = {"median_ms": med, "p95_ms": p95, "max_ms": mx}
    print(f"  {name:34s} median {med:6.3f} ms  p95 {p95:6.3f}  max {mx:6.3f}   = {100 * med / 5.0:4.1f}% of a 200 Hz tick, {100 * med / 6.667:4.1f}% at 150 Hz;  Orin x2: {100 * 2 * med / 5.0:4.1f}% / {100 * 2 * med / 6.667:4.1f}%")


nmpc = base.OuterNMPC()
sched = base.n6.Schedule(V_cruise=16.0, tilt_cap_deg=90.0, alpha_max_deg=8.0)
refs, _, _ = base.REFS_FN(sched, nmpc.N, nmpc.DT, 0.05, 16.0, 16.0, 10.0, 30.0)
x0 = np.array([0.0, 30.0, 16.0, 0.0, th, 0.0])
for _ in range(5):
    nmpc.solve(x0, refs)

print(f"\nINNER STEP ({a.ticks} ticks, wing-borne state):")
row("inner_step (all)", t)
if a.load:
    import threading
    stop = threading.Event(); n_solve = [0]

    def _outer_load():
        while not stop.is_set():
            nmpc.solve(x0, refs); n_solve[0] += 1
    th_ = threading.Thread(target=_outer_load, daemon=True); th_.start()
    time.sleep(0.2)
    tl = []
    for _ in range(a.ticks):
        t0 = time.perf_counter(); tick(); tl.append(time.perf_counter() - t0)
    stop.set(); th_.join()
    row("inner_step (all) UNDER OUTER LOAD", np.array(tl) * 1e3)
    print(f"    ({n_solve[0]} outer solves ran back to back meanwhile, {n_solve[0] / (sum(tl) + 0.2):.0f} Hz)")
# parts
tt = []
for _ in range(a.ticks):
    t0 = time.perf_counter(); l1.adapt(w, 0.005); tt.append(time.perf_counter() - t0)
row("  L1 adapt (the law + observer)", np.array(tt) * 1e3)
tt = []
T4 = np.array([Tf / 2, Tf / 2, Tr / 2, Tr / 2])
for _ in range(a.ticks):
    base.IMU_ACCEL_T = time.monotonic()
    t0 = time.perf_counter(); l1.applied_moment(beta, T4, vb, w, de, 0.0, 0.0, 0.005); tt.append(time.perf_counter() - t0)
row("  L1 applied_moment (model F/M, IMU residual, EKF)", np.array(tt) * 1e3)
tt = []
for _ in range(a.ticks):
    t0 = time.perf_counter(); alloc.inv(beta); tt.append(time.perf_counter() - t0)
row("  allocation inverse", np.array(tt) * 1e3)
pred = getattr(l1, "pred", None)
if pred is not None:
    tt = []
    for _ in range(a.ticks):
        t0 = time.perf_counter(); pred.step(np.zeros(2), np.zeros(3), np.array([0.0, 0.0, 9.81]), 0.005); tt.append(time.perf_counter() - t0)
    row("  EKF slosh predictor step", np.array(tt) * 1e3)
tt = []
for _ in range(a.ticks):
    t0 = time.perf_counter(); base.model_moment(beta, np.array([Tf / 2, Tf / 2, Tr / 2, Tr / 2]), vb, w, de, 0.0, 0.0); tt.append(time.perf_counter() - t0)
row("  model_moment (casadi aero call)", np.array(tt) * 1e3)

print(f"\nOUTER SOLVE ({a.outer} warm solves):")
ts = []
for _ in range(a.outer):
    t0 = time.perf_counter(); nmpc.solve(x0, refs); ts.append(time.perf_counter() - t0)
ts = np.array(ts) * 1e3
RESULT["outer_solve"] = {"median_ms": float(np.median(ts)), "p95_ms": float(np.percentile(ts, 95)), "max_ms": float(ts.max())}
print(f"  solve median {np.median(ts):5.1f} ms  p95 {np.percentile(ts, 95):5.1f}  max {ts.max():5.1f}   of the 50 ms period at 20 Hz: {100 * np.median(ts) / 50:4.1f}% (Orin x2: {100 * 2 * np.median(ts) / 50:4.1f}%)")
if a.load:
    # the outer solving while the inner ticks at 200 Hz in another thread -- its own budget under load
    import threading
    stop = threading.Event()

    def _inner_load():           # a tick every 5 ms, idle (GIL released) in between, as a callback-driven loop is
        while not stop.is_set():
            t0 = time.perf_counter(); tick()
            time.sleep(max(0.0, 0.005 - (time.perf_counter() - t0)))
    th_ = threading.Thread(target=_inner_load, daemon=True); th_.start()
    time.sleep(0.2)
    tl = []
    for _ in range(a.outer):
        t0 = time.perf_counter(); nmpc.solve(x0, refs); tl.append(time.perf_counter() - t0)
    stop.set(); th_.join()
    tl = np.array(tl) * 1e3
    RESULT["outer_solve UNDER INNER LOAD"] = {"median_ms": float(np.median(tl)), "p95_ms": float(np.percentile(tl, 95)), "max_ms": float(tl.max())}
    print(f"  UNDER a 200 Hz inner loop: median {np.median(tl):5.1f} ms  p95 {np.percentile(tl, 95):5.1f}  max {tl.max():5.1f}")
if a.json:
    import json
    with open(a.json, "w") as fh:
        json.dump(RESULT, fh, indent=1)
    print(f"  -> {a.json}")

if a.profile:
    import cProfile, pstats
    pr = cProfile.Profile(); pr.enable()
    for _ in range(500):
        tick()
    pr.disable()
    print("\ncProfile, inner_step x500 (top 15 by cumulative time):")
    pstats.Stats(pr).sort_stats("cumulative").print_stats(15)
