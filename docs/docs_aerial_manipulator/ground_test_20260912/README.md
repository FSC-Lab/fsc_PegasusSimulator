# Whole-body + L1 on the ground, props inert — 2026-09-12

What the coupled law does when DIRECT is engaged on a seated vehicle and the
arm is then asked to move. Plant = the shipped `_sim` configuration with
`PEGASUS_PLANT_KF_SCALE = PEGASUS_PLANT_KM_SCALE = 0`, so the rotors turn at
the commanded speed and produce no force and no yaw moment.

Launcher `scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_ground_test_sitl.sh`
(headless) against the UNCHANGED controller stack
`start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh`.

| file | what it is |
|---|---|
| `ground_A.npz` | run A — 77 s of DIRECT, arm out + back |
| `ground_B_soak.npz` | run B — 359 s of DIRECT, arm out + back |
| `ground_score.py` | the scorer (reads either; needs `/usr/bin/python3` WITHOUT `PYTHONNOUSERSITE`) |

    /usr/bin/python3 ground_score.py ground_B_soak.npz

Driver: `application/robotic_arm/utils/wb_ground_test_driver.py`.
Full findings: `../Command.md` §7.16.1.

## The result in one line

    u1 + d_hat_z = 36.755 N, constant to 9-12 mN = the controller's own m*g

The law sits in a self-consistent hover equilibrium. The momentum observer
measures `dp/dt`; seated, that is zero and the commanded `h + u` is already
~zero, so there is no residual to find — the floor supplies the reaction and
nothing in the loop can tell. Neither the +15% allocator kf nor the x1.10 plant
mass is ever detected: the plant weighs 40.41 N, the law commands 36.75 N, and
in six minutes the observer accumulates 0.90 N of a 20 N bound.

Both runs completed with no abort, 0.00% saturation/clamp, the vehicle immobile
to the micron, and the arm tracking its compatible trajectory to 0.9-2.1 mm.

## Two traps this campaign paid for

* The hold node originally LATCHED its reference after N odometry samples. The
  estimator publishes (0, 0, 0) before it has a fix, so it latched the origin
  and the DIRECT gate then refused on a 0.305 m error. It now tracks.
* PX4 stays armed after a ground run (the land detector needs low thrust and
  the law commands 57% motor), so **one launch is one run** — full clean and
  relaunch in between.
