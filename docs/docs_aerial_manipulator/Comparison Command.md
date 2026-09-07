# Comparison campaign — whole-body vs geometric + L1 adaptive

Command reference for the AM-T650 controller comparison. Three tasks, two
controllers, one shared trajectory table.

**Nothing in this campaign touches the flight-validation path.** The production
launchers, the production yamls and the Isaac entrypoints `05` and `06` are
unmodified; every file below is new. `05`/`06` are *reused unmodified* rather
than copied, deliberately: the plant is the one thing that must be identical
between the two runs, and two copies of a 700-line plant script is the easiest
way to lose that.

---

## 1. What is being compared, and what is held fixed

| | whole-body (`wb`) | geometric + L1 (`l1`) |
|---|---|---|
| control node | `autopilot_whole_body_direct_actuation_node` | `autopilot_geometric_l1_direct_actuation_node` |
| law in DIRECT | coupled airframe+arm Cartesian impedance + GMO; commands 4 rotors **and** 4 joint torques | geometric SE(3) + L1 adaptive augmentation; commands 4 rotors only |
| arm actuation | torque, from the same law | position servo (`05`'s Dynamixel emulation, PD + gravity comp) |
| Isaac entrypoint | `06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py` | `05_px4_direct_t650_aerial_manipulator_ros2_arm_hold.py` |
| config | `params_single_aerial_manipulator_whole_body_direct_actuation_t650_comparison.yaml` | `params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_comparison.yaml` |

Held identical, and checked rather than assumed:

* **Plant.** `AM_xfwd.usda`, the T650 motor model (`t650_params`: idle/max omega,
  `k_f`, `c`, the measured rotor lag `lambda = 10.0265 1/s`), the 3.746170 kg
  total (both launchers set `PEGASUS_EXPECTED_TOTAL_MASS`, and both Isaac
  scripts abort before physics if it drifts), the arm model, the spawn pose and
  the PX4 profile `rootfs_fsc_indoor_am_t650`.
* **Allocator belief.** `alloc_thrust_coeff = 4.0412832584460006e-05` in BOTH
  configs = `t650_params.ROTOR_CONSTANT`, i.e. **matched to the plant**. The
  shipped `_sim.yaml`s carry deliberate robustness injections (+15% on the
  whole-body side, +20% **and** a +10 mm `armff_mismatch_x` r_os error on the
  L1 side) which would by themselves decide the comparison; both are OFF here.
  `start_comparison_am_stack.sh` greps both files and refuses to launch if
  either has drifted.
* **Everything else in the shared key set.** The two comparison yamls have 49
  keys in common and 46 of them are byte-identical: mass, thrust scaling, idle
  thrust, all four rotor positions and `km`, the SAFETY cascade gains, the UDE
  gain and bounds, the watchdog limits. The only intended differences are
  `vehicle_name` and each law's own gain block.
* **Commanded motion.** One table, `utils_comparison/comparison_tasks.py`,
  produces the base path, the joint path and the EE path; the driver publishes
  the base half to both stacks as a `PositionControllerReference` and the joint
  half to both as the same numbers. The whole-body node additionally receives
  the full `WholeBodyReference` (its law's actual reference), built from that
  same table.

One asymmetry worth naming, because it is in the interface rather than in
anything this campaign chose: `PositionControllerReference` carries position,
velocity and acceleration — the L1 geometric law reads all three
(`l1_geometric_controller.cpp` uses `ref.velocity` and `ref.acceleration`) — but
it has no jerk/snap or yaw-rate field, so that law runs with
`omega_d = omega_dot_d = 0`. `WholeBodyReference` carries the CoM chain through
snap, which the coupled law needs. Both sides therefore get the same
position/velocity/acceleration command; the whole-body side additionally gets
the higher derivatives its own law is written against. This only bites where
the commanded yaw actually moves, i.e. comparison 2 (peak 15.5 deg/s).

Not identical, by design — this is the difference under test:

* the two laws and their gains (tuned separately; a shared tune would be a
  handicap, not a control);
* the arm's actuation channel (position servo vs direct torque). The whole-body
  law *requires* torque mode; the geometric+L1 law is built to fly with a
  position-mode servo arm. Both runs log the measured joints, so the realised
  arm motion is comparable too.

**Not in the loop on either side:** the `fsc_open_manipulator` ros2_control
stack, both ground stations, and the whole-body planner. The driver
owns every reference. `start_comparison_am_stack.sh` refuses to start if a
whole-body planner is running — it publishes the same `WholeBodyReference` topic, and two
publishers there is exactly the failure the logs could not reveal.

---

## 2. The three tasks

Defined once in `application/robotic_arm/utils_comparison/comparison_tasks.py`.
Run that file directly for the offline self-test (pure numpy, no Isaac, no ROS):

```bash
cd ~/fsc_PegasusSimulator
/usr/bin/python3 application/robotic_arm/utils_comparison/comparison_tasks.py
```

It checks joint limits, the certified singularity margin, the commanded joint
rates against `05`'s 0.5 rad/s servo slew limit, that every derivative is
really the derivative of the field below it (recomputed with a different
stencil), that the headings are unit, and that every task starts at the folded
HOME pose, at the hover setpoint, at rest. **A task that fails this is refused
by the driver before it arms.**

### `hover_arm_swing` — comparison 1

Base pinned at `(0, 0, 1)`; the arm sweeps its fold `beta = q2 + q3` from the
home 80 deg down to 40 deg and back, twice, over 24 s (`q1 = q4 = 0`, so the
disturbance is a clean in-plane one). Metric: **base position error** — how far
the floating base moves while the arm swings. Ball figure: base position.

Measured on this model: over that sweep the system CoM travels 10 mm
vertically and 0.7 mm horizontally while the EE travels 150 mm vertically. The
arm is light next to 3.75 kg, so most of what the base feels is the arm's
inertial reaction rather than the static CoM shift.

### `circle_ee_hold` — comparison 2

The base flies a small circle **and** yaws while the end-effector holds one
world pose (3 position + 1 heading). Unfolds from home to `beta = 60 deg`
first, pins there for 40 s, refolds at the end; only the **pinned** segment is
scored. Ball figure: end-effector position.

**Why the circle is only 25 mm.** Measured with the real IK, not assumed, and
this is itself a result. Pinning all four EE DOFs leaves the base only the
arm's null space, and for this manipulator that is a thin set:

* **radially** (along the arm) the reach is a *fold*, so extending it also
  drops the EE; at a fixed base height only ~5 cm of radial travel keeps the
  pinned point inside the joint limits and the singularity margin. Circles
  above R = 25 mm leave it — verified by running the IK around the whole
  circle at every candidate radius.
* **laterally** it is much freer (`q1` gives +-35 deg of azimuth), which is why
  a circle fits at all rather than a slit.
* **in yaw** the cap is +-20..25 deg, and it is not the airframe: locking the
  EE heading forces `q1 ~ -psi`, and `q1`'s stop is +-35 deg. `q4` cannot take
  over, because its authority over the EE azimuth scales with `cos(beta)` — at
  the folded home pose (`beta = 80 deg`) that is 0.17, so undoing 1 rad of
  `q1` would need ~5.7 rad of wrist. This is the same bound `poly_whole`'s
  pinned phase reported, measured again here from the other direction.

That last point is why the task does **not** run at home: it unfolds to
`beta = 60 deg`, where `cos(beta) = 0.5` and the wrist has real authority.
Shipped parameters give `sigma_nd >= 0.206` (margin 0.10), `q` inside every
limit with >= 6 deg to spare, and a peak commanded joint rate of 16 deg/s.

### `figure8_ee_updown` — comparison 3

Both halves move. The base flies a Gerono figure-8 (1.0 m x 0.3 m, one lap over
32 s, constant heading) while the arm bobs the EE up and down relative to the
drone (fold 80 -> 50 deg, three cycles). The EE reference is forward kinematics
of the commanded base path and the commanded fold, which is exactly what makes
the up-and-down "compatible" — no IK and no feasibility question. Metrics:
base **and** EE tracking error. Ball figure: end-effector position.

Constant heading on purpose: yaw is this simulator's known-weak axis (the
missing `I_rotor * omega_dot` term), and the comparison should not be
additionally separated by plant fidelity.

### The reference is attitude-consistent, not level

Both `x_cd` and `r_ed` are built with the attitude the reference actually
demands (`b3` from the commanded CoM acceleration, `b1` from the commanded
heading), not with `Rz(phi)`. Taking level would put a systematic
`|r_0e| * tilt` bias into the EE reference — 4 mm at comparison 3's peak
acceleration. One correction step, not a fixed point: iterating
`x_cd -> R0(d2x_cd/dt2) -> x_cd` is unstable at sample resolution (measured:
it diverges to a 180 deg "tilt" in 12 iterations), and it is unnecessary
because `|r_0c|` is only 24 mm, so the feedback is second order.

---

## 3. Running a case

### Everything at once

```bash
cd ~/fsc_PegasusSimulator
./scripts/comparison/run_comparison_case.sh wb hover_arm_swing shiqi_machine
./scripts/comparison/run_comparison_case.sh l1 hover_arm_swing shiqi_machine
```

Clean slate -> controller stack -> Isaac/PX4 -> wait for odometry -> wait for
**EKF alignment** -> fly -> `results/<task>/<controller>.npz`.

Three things that harness does which a hand-run must also do:

* **A full relaunch per data point.** Parameters are read at controller startup
  and PX4 never disarms this rig, so a second mission on one launch finds PX4
  latched "in flight" and the SAFETY UDE integrating the ground reaction; the
  takeoff then produces zero lift with no error message. The driver refuses to
  start against an armed vehicle for the same reason.
* **Gate on the EKF flags, not on `vehicle_status.pre_flight_checks_pass`.**
  That field is false on this rig even while it is armed and flying. Gating on
  it fails every run for a reason that has nothing to do with the vehicle.
  Gate on `estimator_status_flags`: `cs_yaw_align`, `cs_ev_pos`, `cs_ev_yaw`.
* **Source `~/ros2_ws/install/setup.bash`.** This machine has two workspaces
  carrying `fsc_autopilot_ros2_msgs`, and the older `~/workspaces/isaacsim` one
  has no `WholeBodyReference`. Inheriting it surfaces only as an `ImportError`
  in the driver *after* the whole sim is up. The harness proves the import
  before launching anything.

Everything in the harness runs from a script file on purpose: the launcher
guards use `pgrep -f` on the controller node name, which also matches any
**shell** whose command line contains it — never inline these commands in a
terminal.

### By hand, three terminals

```bash
# 1. controller stack (owns MicroXRCEAgent) — 'wb' or 'l1'
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_comparison_am_stack.sh \
    shiqi_machine wb uav_0

# 2. Isaac + PX4
~/fsc_PegasusSimulator/scripts/comparison/start_comparison_am_sitl.sh shiqi_machine wb

# 3. the mission (in a shell with ~/ros2_ws/install/setup.bash sourced)
/usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/comparison_driver.py \
    --controller wb --task hover_arm_swing
```

The mission is fully scripted: OFFBOARD -> arm -> SAFETY takeoff to the task's
own start point -> settle -> DIRECT -> 15 s settle -> the task -> hold ->
SAFETY -> reference landing -> disarm. It reverts to SAFETY and lands on its
own if the vehicle leaves the abort envelope (35 deg tilt, 2.5 m from the hover
point, 2.5 m/s), and records that in the npz.

Useful flags: `--direct-settle` (default 15 s), `--land-z` (default 0.35 —
land **to 0.35 or above**; commanding below the 0.305 m resting height pushes
the vehicle into the ground and tips it), `--abort-tilt`, `--out`.

### Scoring and figures

```bash
PYTHONNOUSERSITE=1 /usr/bin/python3 \
    application/robotic_arm/comparison_plots.py --task hover_arm_swing
```

`PYTHONNOUSERSITE=1` is required on this machine: a user-site numpy 2.2.6
shadows the apt numpy 1.x that the apt matplotlib 3.5.1 was compiled against,
so `import matplotlib` fails outright. Setting it selects the apt numpy and
matplotlib works. (The **driver** must NOT use it — it needs the user-site
numpy alongside rclpy.)

Writes into `results/<task>/`: `metrics.json`, `summary.md`,
`tracking_error.png`, `error_ball.png`, `paths.png`.

---

## 4. How the numbers are produced

The measured end-effector is **forward kinematics of the measured base pose and
the measured joint angles**, on `transition_planner.make_params_t650()` — the
same chain both laws are built on, with the EE at the gripper grasp point
(`GRIPPER_OFF_WRIST = 0.108`). Neither controller is scored against its own
idea of where the EE is.

Every number is taken over a scored window that is intersected with DIRECT and
with the task actually running, so the mode-switch transient and the SAFETY
takeoff never enter a metric. Comparison 2 narrows further to its `pinned`
segment. The shaded band in `tracking_error.png` is that window.

The **ball** is the error cloud over the scored window with a sphere at its RMS
norm — smaller ball, tighter hold. Both clouds go in one 3-D axes so "which is
smaller" is a direct comparison, with the three orthogonal projections beside
it because 3-D scatter depth is exactly what the eye is bad at.

---

## 4b. Reading the numbers: base yaw dominates the EE error

`1 deg of base yaw error IS 4.4 mm of end-effector position error`, because the
EE sits ~0.25 m out on the arm. In comparison 2 that term is the whole story:
the pinned-phase base yaw error measured 4.0 deg rms (whole-body) and 8.7 deg
rms (geometric+L1), which alone accounts for 18 mm and 38 mm of lateral EE
error against measured y-axis EE rms of 36 mm and 58 mm. The error cloud in
`error_ball.png` is correspondingly a flat, tilted sheet — the signature of an
in-plane rotation error, not of the arm mis-tracking.

So `base_yaw_err_deg` is reported as a first-class metric, not a diagnostic. A
reader who only saw the EE number would attribute it to the manipulator.

Two consequences worth stating plainly:

* **Comparison 2 is, in large part, a yaw-tracking comparison.** That was not
  avoidable: the pinned-EE workspace is so thin that the +-20 deg yaw sweep is
  the only motion of any amplitude available (section 2), so any pinning test
  on this arm is a yaw test. The whole-body law tracked it 2.2x better, which
  is a real result, but it is not the same claim as "the arm compensates
  better".
* **Yaw is this simulator's known-weak axis** — the thrust curve applies only
  the steady drag torque `c*omega^2` and never the rotors' own
  `I_rotor*omega_dot`, which `t650_params` stands in for with
  `YAW_TORQUE_FIT_FACTOR = 3.0`. Both controllers fly the same wrong yaw
  plant, so the comparison is fair, but the absolute yaw numbers should not be
  carried to hardware. Comparisons 1 and 3 hold the yaw constant on purpose
  and are free of this.

---

## 5. Files

New, in `fsc_PegasusSimulator`:

| path | what |
|---|---|
| `application/robotic_arm/utils_comparison/comparison_tasks.py` | the three task definitions + the offline self-test |
| `application/robotic_arm/comparison_driver.py` | mission state machine, reference streaming, logging |
| `application/robotic_arm/comparison_plots.py` | metrics + figures |
| `scripts/comparison/start_comparison_am_sitl.sh` | Isaac/PX4 side |
| `scripts/comparison/run_comparison_case.sh` | clean slate -> npz, one case |
| `results/<task>/` | npz, metrics, figures, logs |

New, in `fsc_autopilot_ros2` (`dev_CCM`):

| path | what |
|---|---|
| `config/params_single_aerial_manipulator_whole_body_direct_actuation_t650_comparison.yaml` | |
| `config/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_comparison.yaml` | |
| `scripts/isaacsim/start_comparison_am_stack.sh` | one script for both controllers, with the config-parity check |

---

## 6. First campaign, 2026-09-02 — results

Six flights, one per (task, controller), each from a clean slate. No aborts;
every run completed its task, reverted to SAFETY and landed. Data, figures and
per-run console logs under `results/`; the roll-up is `results/SUMMARY.md`.

| comparison | headline | whole-body | geometric+L1 | ratio |
|---|---|---|---|---|
| 1. hover + arm fold sweep | base position RMS | **5.26 mm** | 10.98 mm | 2.09x |
| 2. circle + yaw, EE pinned | EE position RMS | **40.84 mm** | 60.99 mm | 1.49x |
| 3. figure-8 + EE up/down | EE position RMS | **87.71 mm** | 128.57 mm | 1.47x |

**Comparison 1 — the cleanest of the three.** The base is commanded to stand
still, so there is no tracking-lag term and the number is pure disturbance
rejection. The whole-body law holds the base 2.1x tighter (p95 9.7 vs 18.2 mm)
and flies 0.10 vs 0.15 deg rms of tilt. Per-axis, the error is almost entirely
along the arm (x: 4.2 vs 10.5 mm; y: 2.5 vs 4.6; z: 0.6 vs 0.5) — which is what
the mechanism predicts: the fold moves the system CoM fore-and-aft, the
whole-body law is *told* that through `x_cd`, and the geometric law has to
discover it. The one metric the L1 side wins is joint tracking (0.79 vs
0.91 deg rms), which is what a dedicated position servo is for.

**Comparison 2 — the whole-body law wins the pinned EE, and loses the base.**
EE position 40.8 vs 61.0 mm and EE heading 2.80 vs 7.79 deg go to the
whole-body law; base position (33.3 vs 29.7 mm) goes to geometric+L1. That
split is the whole point of a whole-body controller rather than an anomaly: it
is allowed to move the base to hold the task variable, and it did. But see
4b — most of both numbers is base *yaw* error (4.0 vs 8.7 deg rms), so this is
substantially a yaw-tracking comparison.

**Comparison 3 — the raw number is mostly LAG, and the split matters.** Both
laws run behind a 0.11 m/s reference:

| | whole-body | geometric+L1 |
|---|---|---|
| fitted lag | **1.04 s** | 1.57 s |
| along-path rms | 115.0 mm | 163.1 mm |
| cross-track rms | **49.9 mm** | 86.6 mm |
| rms with the lag removed | **26.2 mm** | 35.7 mm |

So the honest reading of comparison 3 is *"the whole-body law runs 0.5 s less
behind, and once the lag is taken out its residual path error is 1.4x
smaller"* — not "it tracks 1.5x better". `paths.png` shows both flying the
figure-8 shape cleanly, slightly inside the commanded one, which is the
amplitude deficit that a lag of this size produces.

**A ~1 s lag despite correct velocity and acceleration feedforward is
unexpected and is the obvious next thing to chase.** It is not a scheduling
artifact, and that was checked rather than assumed:

* the driver published at 100.0 Hz with a 12.9 ms worst-case tick in every run;
* the feedforward is right — least-squares fit of the published `ref_v` against
  a finite difference of the published `ref_p` gives a scale of 1.0014
  (an earlier peak-to-peak comparison suggested 0.83 and was simply
  noise-dominated; peaks are the wrong statistic for this check);
* and the **simulator is running in real time**. This is the trap worth
  recording: `docs/sim_to_real_t650` measures 0.84–0.90x for other scenarios,
  and a wall-clock-scheduled reference under a slow sim produces a *growing*
  lag that looks exactly like this. It is ruled out by the arm: comparison 1's
  fold sweep is scheduled on the same wall clock, and its joint-2 error is
  1.34 deg rms in the first half of the sweep and 1.19 deg in the second — flat,
  not growing. At 0.85x it would have ended 45 deg out.

## 7. Traps met while building this

* **The pinned-EE workspace is much smaller than the arm's reach suggests**, and
  in a direction that is easy to get wrong: radial travel, not lateral, is the
  binder (section 2).
* **`q4` cannot pin the EE heading at the folded pose.** Its azimuth authority
  is `cos(beta)`; at `beta = 80 deg` an IK that is asked to do it drives the
  wrist at 900 deg/s and leaves the joint limits. Unfold first.
* **The commanded joint rate must stay under 0.5 rad/s**, `05`'s servo
  reference slew limit — above it the L1 side is silently rate-limited and the
  two runs no longer share a command. The self-test checks it.
* **The 4th difference of an IK-derived signal is noise-limited.** At a 20 ms
  stencil the CoM snap carried 2.4e-4 m/s^4 of IK jitter; 48 ms drops it ~33x
  while the truncation error stays negligible for these <0.2 Hz tasks.
* **Peaks are the wrong statistic for verifying a derivative.** Comparing the
  peak of a published velocity against the peak of a finite difference of the
  published position suggested a 17% error that did not exist; a least-squares
  scale over the whole record gave 1.0014. A finite difference taken across
  real timer jitter has noise spikes above the true peak.
* **A relative FD-consistency check is meaningless on a channel whose true
  derivative is zero** (a constant heading, a pinned base) — it reads 100%
  wrong at a 1e-12 residual. Judge those on the absolute number.
