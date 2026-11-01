#!/usr/bin/env python3
"""Queue the box-pushing A/B for runner.sh (this folder's copy of the 10-25 pose/gain campaign's runner:
flies queue/*.job oldest-name first through tools/fly.sh, a clean slate before each flight).
Condition = the 10-25 hardware-like condition: 280 g, 0.40 m at 21 mm/s, the hardware guard values
(tilt 10 deg, abort release hold 1.5 s), the drift model 0.32 N @ 0.12 Hz per seed, the arm GS
flying Ready / Push / Exit itself.
  B = the regenerated box sim yaml (the two sets: free flight 211.9 / 347.5 / 211.9 + omega_c_t 2.927 +
      the stiff heading, the push's contact set from the grasp, held until the exit is clear).
  A = the same yaml with every free-flight key off (the one-set configuration every box campaign flew).
    python3 box_jobs.py            # writes queue/<nn>__<tag>.job; then: setsid nohup bash runner.sh > ../runner.log 2>&1 &
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
Q = os.path.join(HERE, "..", "queue")
os.makedirs(Q, exist_ok=True)
ENV = "MASS=0.280 DIST=0.40 SPEED=0.021 ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 PL_DRIVER_ARGS=--gs-auto"
GUARDS = "push_pull_guard_tilt_deg=10.0 push_pull_abort_release_wait_s=1.5"
ONESET = ("wb_ky_x_before_grip=0.0 wb_ky_y_before_grip=0.0 wb_ky_z_before_grip=0.0 wb_dy_x_before_grip=0.0 "
          "wb_dy_y_before_grip=0.0 wb_dy_z_before_grip=0.0 wb_l1_omega_c_t_before_grip=0.0 "
          "wb_ky_psi_before_grip=0.0 wb_dy_psi_before_grip=0.0 wb_stuck_hold_j4_before_grip=false "
          "wb_grip_hold_set_while_free_axis=false")


def noise(seed):
    return f"sim_plant_force_noise_n=0.32 sim_plant_force_noise_bw_hz=0.12 sim_plant_force_noise_seed={seed}"


jobs = [
    ("tsB_nom", ""), ("tsB_s2", noise(2)), ("tsB_s3", noise(3)), ("tsB_s5", noise(5)),
    ("tsA_nom", ONESET), ("tsA_s5", f"{noise(5)} {ONESET}"),
    ("tsB_s1", noise(1)), ("tsA_s2", f"{noise(2)} {ONESET}"),
]
for k, (tag, args) in enumerate(jobs, 1):
    with open(os.path.join(Q, f"{k:02d}__{tag}.job"), "w") as f:
        f.write(f'ENV="{ENV}"\nARGS="{(GUARDS + " " + args).strip()}"\n')
    print(tag, args)
