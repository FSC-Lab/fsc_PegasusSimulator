#!/usr/bin/env python3
"""The compute budget of the flown loops, per component, on THIS machine (hardware-transfer piece 6,
2026-09-16) -- the number to scale for the Jetson Orin (roughly half the single-thread speed of this
desktop) before promising 150-200 Hz for the inner loop.

  ~/envs/isaac_env/bin/python3 scripts/bench_loops.py [--ticks 2000] [--outer 40] [--profile]

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
a = ap.parse_args()

os.environ.setdefault("VTOL_TANK_KG", "0.502"); os.environ.setdefault("VTOL_TANK_R", "0.10"); os.environ.setdefault("VTOL_TANK_DX", "0.3")
sys.argv = ["bench", "--est", "mdob", "--slosh_pred", "on", "--test"]        # what the variant modules read at import
sys.path.insert(0, os.path.join(controller_dir(), "px4_vtol_direct_actuator"))
sys.path.insert(0, controller_dir())
import nmpc_cascade_node as base                     # noqa: E402
import nmpc_cascade_steso_node as sv                 # noqa: E402
import nmpc_cascade_imu_node as iv                   # noqa: E402,F401   (re-points base.L1Aug / base.inner_step)
so3 = base.so3

print(f"loops benchmark on {os.uname().nodename} ({os.cpu_count()} cpus), python {sys.version.split()[0]}, numpy {np.__version__}")
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
    med, p95, mx = np.median(arr), np.percentile(arr, 95), arr.max()
    print(f"  {name:34s} median {med:6.3f} ms  p95 {p95:6.3f}  max {mx:6.3f}   = {100 * med / 5.0:4.1f}% of a 200 Hz tick, {100 * med / 6.667:4.1f}% at 150 Hz;  Orin x2: {100 * 2 * med / 5.0:4.1f}% / {100 * 2 * med / 6.667:4.1f}%")


print(f"\nINNER STEP ({a.ticks} ticks, wing-borne state):")
row("inner_step (all)", t)
# parts
tt = []
for _ in range(a.ticks):
    t0 = time.perf_counter(); l1.adapt(w, 0.005); tt.append(time.perf_counter() - t0)
row("  L1 adapt (obs + force + EKF)", np.array(tt) * 1e3)
tt = []
for _ in range(a.ticks):
    t0 = time.perf_counter(); alloc.inv(beta); tt.append(time.perf_counter() - t0)
row("  allocation inverse", np.array(tt) * 1e3)
pred = getattr(l1, "pred", None)
if pred is not None:
    tt = []
    for _ in range(a.ticks):
        t0 = time.perf_counter(); pred.step(np.zeros(3), np.zeros(3), np.array([0.0, 0.0, 9.81]), 0.005); tt.append(time.perf_counter() - t0)
    row("  EKF slosh predictor step", np.array(tt) * 1e3)
tt = []
for _ in range(a.ticks):
    t0 = time.perf_counter(); base.model_moment(beta, np.array([Tf / 2, Tf / 2, Tr / 2, Tr / 2]), vb, w, de, 0.0, 0.0); tt.append(time.perf_counter() - t0)
row("  model_moment (casadi aero call)", np.array(tt) * 1e3)

print(f"\nOUTER SOLVE ({a.outer} warm solves):")
nmpc = base.OuterNMPC()
sched = base.n6.Schedule(V_cruise=16.0, tilt_cap_deg=90.0, alpha_max_deg=8.0)
refs, _, _ = base.REFS_FN(sched, nmpc.N, nmpc.DT, 0.05, 16.0, 16.0, 10.0, 30.0)
x0 = np.array([0.0, 30.0, 16.0, 0.0, th, 0.0])
for _ in range(5):
    nmpc.solve(x0, refs)
ts = []
for _ in range(a.outer):
    t0 = time.perf_counter(); nmpc.solve(x0, refs); ts.append(time.perf_counter() - t0)
ts = np.array(ts) * 1e3
print(f"  solve median {np.median(ts):5.1f} ms  p95 {np.percentile(ts, 95):5.1f}  max {ts.max():5.1f}   of the 50 ms period at 20 Hz: {100 * np.median(ts) / 50:4.1f}% (Orin x2: {100 * 2 * np.median(ts) / 50:4.1f}%)")

if a.profile:
    import cProfile, pstats
    pr = cProfile.Profile(); pr.enable()
    for _ in range(500):
        tick()
    pr.disable()
    print("\ncProfile, inner_step x500 (top 15 by cumulative time):")
    pstats.Stats(pr).sort_stats("cumulative").print_stats(15)
