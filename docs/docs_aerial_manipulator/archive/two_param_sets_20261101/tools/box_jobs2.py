#!/usr/bin/env python3
"""The box follow-up (2026-11-01): the regenerated box yaml with wb_grip_closed_below -0.05 (the jaws creep on the
sim's grip shim; the set switched 1.7 s INTO the push at -0.1) and wb_grip_release_delay_s 6 (the set holds
through the exit climb; tsB_s5 tipped its box with the set blending back there). Same condition as box_jobs.py."""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
Q = os.path.join(HERE, "..", "queue")
os.makedirs(Q, exist_ok=True)
ENV = "MASS=0.280 DIST=0.40 SPEED=0.021 ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 PL_DRIVER_ARGS=--gs-auto"
GUARDS = "push_pull_guard_tilt_deg=10.0 push_pull_abort_release_wait_s=1.5"


def noise(seed):
    return f"sim_plant_force_noise_n=0.32 sim_plant_force_noise_bw_hz=0.12 sim_plant_force_noise_seed={seed}"


jobs = [("tsB2_nom", ""), ("tsB2_s5", noise(5)), ("tsB2_s2", noise(2))]
for k, (tag, args) in enumerate(jobs, 11):
    with open(os.path.join(Q, f"{k:02d}__{tag}.job"), "w") as f:
        f.write(f'ENV="{ENV}"\nARGS="{(GUARDS + " " + args).strip()}"\n')
    print(tag, args)
