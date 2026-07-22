# PegasusSimulator — Developer Checklist

## Repository layout

This is the FSC Lab fork of [Pegasus Simulator](https://github.com/PegasusSimulator/PegasusSimulator) (NVIDIA Isaac Sim framework for PX4/ArduPilot multirotor simulation).

- `extensions/pegasus.simulator/` — core simulator (upstream): vehicles, backends, sensors, dynamics, thrusters, graphs, UI
  - `pegasus/simulator/logic/vehicles/` — vehicle base class (`vehicle.py`), `multirotor.py`, and concrete models under `multirotors/` (e.g. `iris.py`, `ideal_quadrotor.py`)
  - `pegasus/simulator/logic/backends/` — control/telemetry backends (PX4 mavlink, ArduPilot mavlink, ROS 2)
- `extensions/fsc_aerial_manipulation/` — FSC Lab's own library: `aerodynamics/`, `constraints/`, `robotic_arm/`, `rotorcraft/`, `slung_load/`, `utils/`
- `application/` — runnable example scripts, one subfolder per scenario: `ideal_quadrotor/`, `px4_base/`, `slungload/`, `robotic_arm/` (X650 quadrotor + arm, see below). `aerial_manipulation/` and `VTOL/` still exist as empty placeholders
- `examples/`, `docs/`, `scripts/`, `tools/` — upstream examples, docs, and Isaac Sim tooling

## Aerial manipulator (X650 quadrotor + robotic arm)

Merged from teammate Shiqi Gao's `dev_robotic_arm` branch (`380d297`, 2026-07-10) — a quadrotor
("X650" airframe) with a 4-DOF robotic arm rigidly coupled to it, controlled by a Python port of
a validated MATLAB whole-body geometric+impedance controller. Its original/default `direct`
mode is pure ROS 2. The launcher now also provides a `px4-offboard` mode where the external
controller streams PX4 `OffboardControlMode(direct_actuator=True)` + `ActuatorMotors`, PX4
applies its Offboard/arming/saturation gate, and the resulting `HIL_ACTUATOR_CONTROLS` drives
Isaac through Pegasus's primary MAVLink backend.

**Related paths — treat these three as one unit for future work on this scenario:**
- `application/robotic_arm/`
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/`
- `scripts/start_aerial_manipulator.sh`

- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/x650_vehicle.py` —
  `VehicleMod(Vehicle)`, thin subclass adding `usd_prim_path`/configurable `body_path` for a
  custom USD asset.
- `.../robotic_arm/x650_multirotor.py` — `MultirotorMod(BaseMultirotor, VehicleMod)`, adds
  configurable rotor prim paths/joint names. Compliant with this file's vehicle-model checklist
  below (`config=None` resolved inside `__init__`, no mutable defaults).
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/x650_rotorcraft_utils.py`
  — spawn helper (`spawn_rotorcraft_with_mavlink` with `px4_primary`/`include_px4` toggles for
  ROS2-only vs PX4-primary operation) plus asset-authoring helpers used to extract USD-derived
  mass/inertia/rotor-position params for the controller.
- `.../robotic_arm/controller.py` (1008 lines) — `MatlabController` (thrust + body torque +
  joint torques from the full coupled airframe+arm dynamics, not the arm alone) wrapped in a
  ROS2 `MatlabControllerNode`: subscribes `state/pose`/`state/twist*`/`joint_states`, publishes
  `rotor_velocity_command`/`joint_torque_cmd`. Run standalone (`python3 controller.py --ros-args
  -r __ns:=/uav_0`), not launched by the Isaac Sim script itself.
- `.../robotic_arm/postprocessor.py` (1227 lines) — **not** a runtime component. A one-shot
  OnShape→USD asset-cleanup pipeline (reparent/rename/realign frames/set mass+inertia/add
  colliders/apply joint drives/export), meant to be pasted into Isaac Sim's Script Editor after
  CAD import, not imported by anything else.
- `.../robotic_arm/__init__.py` is empty — import submodules directly
  (`fsc_aerial_manipulation.robotic_arm.x650_vehicle`, etc.), not via the package.
- `application/robotic_arm/01_aerial_manipulator_hover.py` (761 lines) — the working demo
  entrypoint: DC-interface joint control, ROS2 pub/sub for torque/rotor commands, arm
  position-hold→effort-control handoff.
- `application/robotic_arm/22_x650_with_manip_minimal.py` (109 lines) — **currently broken as
  committed**: inserts `application/robotic_arm/utils/` onto `sys.path` to import
  `x650_rotorcraft_utils`, but that `utils/` directory doesn't exist in this repo (line 30), and
  it hardcodes an absolute USD path under `/home/fsc-jupiter/...` (line 73) rather than
  resolving relative to `FSC_PEGASUS_ROOT`. Don't use as a template until fixed — `01` is the
  correct reference.
- `scripts/start_aerial_manipulator.sh` + `scripts/config/shiqi_machine.conf` — follows the
  same `common_config.sh`/`terminal_utils.sh`/per-machine-config convention as the slung-load
  launch scripts. `direct` starts the original two panes (Isaac + controller).
  `px4-offboard` starts PX4 SITL, Micro XRCE-DDS Agent, Isaac, and the controller; it sources a
  configurable `PX4_MSGS_SETUP`, applies `UXRCE_DDS_SYNCT=0` plus disabled HIL auto-disarm for
  the run, prestreams for one second, confirms Offboard, then auto-arms. The controller runs
  under `--ros-args -r __ns:=/uav_0`, matching `PX4_UXRCE_DDS_NS=uav_0`. The actual Isaac
  entrypoint is `01_aerial_manipulator_hover.py`.

**Known checklist deviations, not yet fixed** (see "Checklist for adding a new vehicle model"
below — `x650_vehicle.py`/`x650_multirotor.py` themselves are compliant, only `controller.py`
deviates):
- `controller.py:774` — ROS2 node name is hardcoded (`"matlab_aerial_manipulator_controller"`),
  not parameterized by vehicle/uav id. Fine for today's single-vehicle demo; will collide if two
  arm instances ever run in the same ROS2 domain.
- `controller.py:961` — `rclpy.init(args=args)` is called bare, not wrapped in try/except per
  this repo's "already initialised is not an error" rule; will raise if `rclpy` is already
  initialized by a host process.

## X650 airframe reused standalone (no arm) for a stock PX4 slung-load scenario

`application/slungload/px4_single_drone_payload_x650.py` (new) — a **different, simpler** use
of the X650 platform than the aerial-manipulator scenario above: the bare X650 airframe (no
robotic arm) flown as a plain PX4 SITL quadrotor via the **stock** `pegasus.simulator`
`Multirotor`/`MultirotorConfig` (no `VehicleMod`/`MultirotorMod` subclassing), carrying a
fixed-length slung-load payload — structurally the same pattern as
`application/slungload/01_px4_single_drone_payload.py`, just with the X650 asset swapped in for
Iris. This works unmodified because the new asset follows the stock naming convention the
`Multirotor` class hardcodes (`/body`, `/rotor0`..`/rotor3`, `joint0`..`joint3`).

- Asset: `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/x650.usd`
  (~54MB) — **not** the same file as the aerial-manipulator's
  `AM_realign.usda` (~255MB) in that same `assets/` directory; `x650.usd` is the bare frame only
  (stock rotor/body naming, no arm links), while `AM_realign.usda` is the full quadrotor+arm
  asset with the custom prim structure `VehicleMod`/`MultirotorMod` are built to reference. Both
  are covered by the `extensions/fsc_aerial_manipulation/**/assets/` `.gitignore` rule (large
  binaries, distributed out-of-band via Google Drive, not committed).
- PX4 is the primary backend here (motor authority), with a ROS2 backend for state-publishing
  only (`sub_control: False`) — same division as every other `application/slungload/` scenario,
  and the opposite of the aerial-manipulator scenario (pure ROS2, no PX4).
- Note the filename breaks the `NN_px4_...` numbering convention every other file in
  `application/slungload/` follows (`01_`..`06_`) — not fixed here since it wasn't asked for,
  just worth knowing if you're looking for it by number.

**MN4014+15x5" thrust-curve calibration applied (2026-07-13)** — `x650.usd`'s structure was
verified first (standalone `pxr` inspection, no Isaac Sim launch needed — see
`docs/propeller_testing/` below for the bench data this uses): naming/articulation pattern
matches the known-working `iris.usd` exactly (`ArticulationRootAPI` on the non-rigid-body root
Xform, `RigidBodyAPI` on `/body` and each `/rotorN`), motor-to-motor **diagonal** spacing is
exactly 650.3mm (matches the "X650" name), and rotor0/1 (`prop_clock` mesh) vs. rotor2/3
(`prop_counter_clock` mesh) are correctly paired by diagonal for yaw balance. Vehicle mass was
then set to 3.5kg total (`/x650/body/body`'s authored mass 1.467kg → 3.416kg, so the 4 rotors'
existing ~0.021kg each bring the total to exactly 3.5kg; diagonal inertia scaled by the same
mass ratio, ×2.328, assuming similar mass distribution). A backup
(`x650.usd.bak_before_mass_edit`) was left alongside the asset since it's gitignored with no
git history to fall back on.

**Inertia halved again (2026-07-13), mass unchanged** — user's own follow-up correction: scaling
inertia by the same ratio as mass assumed the added ~2kg is distributed like the rest of the
body, but if it's more centrally concentrated (e.g. batteries mounted near the CG rather than
spread over the frame's full extent), actual inertia wouldn't scale as much as mass did. Diagonal
inertia halved from the mass-ratio-scaled value: `(0.1345, 0.1492, 0.1513) → (0.0673, 0.0746,
0.0757)` kg·m² (mass stays 3.416kg). A second backup (`x650.usd.bak_before_inertia_half`) was
made of the pre-halving (fully mass-scaled) state, in addition to the original
`x650.usd.bak_before_mass_edit` - both available to revert to. This was part of diagnosing the
rotor-lag-induced PX4 oscillation (see below): testing whether the earlier proportional-scaling
assumption on inertia, not just the actuator lag itself, was contributing to the instability,
before considering retuning PX4's rate/attitude gains directly.

The bench-fit throttle→RPM/thrust/torque polynomials (`docs/propeller_testing/` below) were
then converted into the sim's actual thrust-curve interface:
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/x650_bare_frame_utils.py`
  (new) — factored `spawn_x650_with_mavlink(...)` (bare X650, stock `Multirotor`) out of
  `px4_single_drone_payload_x650.py` into a shared module so the calibration constants live in
  exactly one place; that script now just imports it. Distinct from the arm-scenario's
  `x650_rotorcraft_utils.py` (spawns the *arm-equipped* X650 on `MultirotorMod`/`VehicleMod` —
  different USD asset, different vehicle classes).
- Calibration derivation: `QuadraticThrustCurve` takes rotor angular velocity **ω (rad/s)** and
  computes `force = rotor_constant·ω²`, yaw torque via `rolling_moment_coefficient·ω²·rot_dir` —
  an instantaneous model, **no rotor spin-up lag** (`quadratic_thrust_curve.py`'s own comment:
  "no delay introduced"). Only the Step 1 throttle→RPM/thrust/torque polynomials were used at
  first; **the report's Step 3 λ (spin-up bandwidth) model was added later** (2026-07-13, user
  report: it "significantly affects the dynamics") — see below.
  - `rpm(x) = 70.2593x + 781.49` (x = throttle 0–100%) is linear, and so is PX4MavlinkBackend's
    own `omega = (controls + input_offset)·input_scaling + zero_position_armed` map (`controls`
    = PX4's normalized 0–1 motor command) — so the two were matched **exactly** (not just at the
    endpoints): `zero_position_armed = rpm(0)·2π/60 = 81.8374 rad/s`,
    `input_scaling = (rpm(100) − rpm(0))·2π/60 = 735.7537 rad/s`, replacing the stock
    Iris-tuned defaults (`input_scaling=1000`, `zero_position_armed=100`).
  - `thrust(x)`/`torque(x)` are quadratic/cubic in throttle, but `QuadraticThrustCurve` only
    supports a pure `k·ω²` form (through the origin) — so `rotor_constant`/
    `rolling_moment_coefficient` were derived via an **unweighted zero-intercept least-squares
    fit**, sampling the bench polynomials across throttle 0–100% and regressing against ω²
    (no raw CSV data available in this repo to fit directly against ω² — see the propeller
    bench-test section below). Fit quality: thrust RMSE 0.31N (range 0–30N, ~1%), torque RMSE
    0.0136 N·m (range 0.01–0.52 N·m, ~2.6%) — the fixed-through-origin sim model can't capture
    the bench fit's small nonzero throttle=0 intercept, but tracks it closely elsewhere.
  - **Bug found on first live test (2026-07-13) and fixed**: props were visibly spinning on the
    ground before arming. Root cause: `QuadraticThrustCurve.update()` applies `min_rotor_velocity`
    as an **unconditional floor** (`velocity = np.maximum(min_rotor_velocity, ...)`), regardless
    of armed state — unlike `zero_position_armed` (`PX4MavlinkBackend`), which is correctly
    armed-gated (`handle_control()` calls `zero_input_reference()`, forcing true zero, whenever
    PX4 reports disarmed). The first version of this calibration reused the same 81.8374 rad/s
    idle value for *both* — forcing rotors to spin at that floor even disarmed. Fixed by
    splitting them: `zero_position_armed=81.8374` (unchanged, armed-idle-throttle behavior is
    physically correct and desired) but `min_rotor_velocity=0.0` (matches stock Iris's own
    default, restores the sim's existing disarmed→true-zero gating). Not a curve-fitting
    problem — the bench-measured nonzero idle RPM at 0% throttle is real hardware behavior and
    didn't need to be altered; it was being applied through the wrong one of two similar-looking
    but differently-gated parameters.
  - Resulting total max thrust (4 rotors) ≈118.8N against the new 3.5kg vehicle weight (34.3N)
    → thrust/weight ≈3.46:1, a physically sensible margin for this motor class.
- **Rotor spin-up lag added (2026-07-13)** —
  `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/lagged_thrust_curve.py`
  (new): `LaggedQuadraticThrustCurve` subclasses the stock `QuadraticThrustCurve` (not modified —
  Iris and every other vehicle stay on the instantaneous stock class) and replaces its
  `velocity = clip(input_reference, min, max)` instantaneous jump with a first-order lag,
  `omega_dot = rotor_lambda*(omega_cmd - omega)`, directly implementing the bench report's Step 3
  model (`RPM_rate = λ·(RPM_cmd − RPM)`), previously left unapplied. Force is always computed
  from the lagged `omega`, never from the raw commanded/target velocity directly.
  - **Per-step update is the exact discrete (zero-order-hold) solution, not Euler integration**
    (user-requested fix, 2026-07-14): `omega[k+1] = omega_cmd[k] + (omega[k] -
    omega_cmd[k])·exp(-rotor_lambda[k]·dt)`. Euler integration is only *conditionally* stable
    (diverges into growing oscillation once `rotor_lambda·dt` exceeds ~1–2 — confirmed
    numerically: at `rotor_lambda·dt=2.4`, Euler oscillates ±4500 rad/s around an 800 rad/s
    target while the exact update converges smoothly); the exact form is unconditionally stable
    for any `rotor_lambda, dt > 0` and stays exact even if `rotor_lambda` later varies with
    operating point (as long as it's treated as constant within one step — the usual
    zero-order-hold assumption). Not just theoretical here: the exact update also matches the
    true continuous-time analytic solution much more closely at normal operating `rotor_lambda`
    (confirmed numerically: ~3.4x smaller error than Euler after the same number of steps).
  - `rotor_lambda` default `10.51 (1/s)` is the mean of the report's `lag_all`/`lag_filtered`
    variants (10.519/10.501) — not `direct_all`/`direct_filtered` (noisier, up to ~24%
    unphysical negative λ values, NRMSE>1 — see the propeller bench-test section below).
    `τ=1/λ≈95ms`, inside the report's own physically-expected 70–200ms range.
  - Wired into `x650_bare_frame_utils.py` in place of `QuadraticThrustCurve`
    (`X650_ROTOR_LAMBDA=10.51`), so all three X650 scenarios that share this spawn helper (bare
    drone, fixed-length-cable payload, variable-length-cable payload) pick it up automatically.
  - **Resolved (2026-07-14): the drone couldn't fly normally after this was added — root cause
    confirmed as PX4's untouched generic `none_iris` rate/attitude gains, not a bug in the lag
    model itself** (separately verified correct via standalone step-response checks, and the
    Euler-vs-exact-discrete-update fix below). User reported diverging rate/attitude
    oscillation at the bench-measured `λ=10.51`.
    - **Per-step lag update switched from Euler to the exact discrete (zero-order-hold)
      solution** (user-requested, 2026-07-14): `omega[k+1] = omega_cmd + (omega[k] -
      omega_cmd)·exp(-λ·dt)`, unconditionally stable for any `λ,dt>0` (Euler integration is
      only conditionally stable — confirmed numerically: at `λ·dt=2.4`, Euler diverges into a
      growing ±4500 rad/s oscillation around an 800 rad/s target while the exact update
      converges smoothly). Applied in `lagged_thrust_curve.py`; force is still always computed
      from the lagged `omega`, never from the raw commanded velocity directly.
    - **Diagnostic λ sweep** (against PX4's then-still-untouched default gains,
      `MC_*RATE_K=1.0`): `10.51` (bench value) diverged, `12.0` crashed outright, `13.5`/`14.5`
      both oscillated (marginal), `15.0`/`15.51` flew cleanly. This confirmed a lag-vs-
      control-loop-tuning mismatch, not a model bug — but `15.51` isn't the physically accurate
      value, just a stability-driven workaround.
    - **X650 CAD inertia correction**: the user provided the actual CAD (OnShape) "Mass and
      section properties" panel for the X650 frame part. Its `Lxx/Lyy/Lzz` (kg·mm², ÷1e6 for
      kg·m²) — `(0.057775, 0.065004, 0.064082)` — matched `x650.usd`'s **original, pre-edit**
      authored inertia `(0.057775, 0.064082, 0.065005)` for `(Ixx,Iyy,Izz)` almost exactly (Y/Z
      swapped between CAD's and USD's axis labeling) — confirming the asset's original inertia
      was already correctly CAD-sourced, and the earlier `×2.328` mass-ratio scaling (done when
      total mass was set to 3.5kg) was the wrong move, not a refinement. A same-ratio "halve the
      scaled inertia" diagnostic (an intermediate, still-not-CAD-accurate value) was tried first
      and made flight *worse*, which in hindsight makes sense — it wasn't testing the real CAD
      value. Fixed by setting `/x650/body/body`'s diagonal inertia to the exact original
      CAD-derived values (`0.05777498, 0.06408172, 0.065004565`) while **keeping mass at
      3.416kg** (ignoring CAD's own 1.467kg reference mass, per explicit user instruction — the
      extra ~2kg is assumed not to significantly change the rotational inertia distribution). A
      second backup, `x650.usd.bak_before_inertia_half`, and the original
      `x650.usd.bak_before_mass_edit` are both still available to revert to if needed.
    - **X650 PX4 gain tuning — final working configuration**, confirmed live against the
      corrected CAD inertia and the true bench λ=10.51:
      | Parameter | Stock default | X650 tuned value |
      |---|---|---|
      | `MC_ROLLRATE_K` / `MC_PITCHRATE_K` / `MC_YAWRATE_K` | 1.0 | **0.3** |
      | `MC_ROLL_P` / `MC_PITCH_P` | 6.5 | **3.25** |
      | `MC_YAW_P` | 2.8 | **1.4** |

      Reasoning: `*RATE_K` is an overall multiplier on that axis's whole rate-loop P/I/D/FF
      stack — the cleanest lever to uniformly soften the loop without rebalancing P/I/D ratios
      individually, needed because the added actuator lag reduces the loop's available phase
      margin. The attitude-loop `P` gains needed reducing too, found empirically after rate-only
      softening (down to `0.2`) stopped helping — with the inner (rate) loop now much slower,
      the outer (attitude) loop's unchanged gain was commanding rate corrections faster than the
      softened inner loop could actually deliver, a classic cascade-control mismatch (outer loop
      bandwidth must stay meaningfully below inner-loop bandwidth). Halving both attitude `P`
      gains alongside the rate gains resolved this.
      - `scripts/apply_x650_px4_gains.sh` (new) — applies and `param save`s this exact
        configuration to a running PX4 SITL instance via `tmux send-keys`, so it doesn't have
        to be re-typed by hand every session. Called (backgrounded, `&`) from all three X650
        launch scripts (`start_x650_single_drone.sh`,
        `start_single_drone_sitl_payload_x650.sh`,
        `start_single_drone_sitl_payload_variable_cable_x650.sh`) a fixed 8s after PX4 is
        expected to have booted to its `pxh>` prompt — `tmux send-keys` has no way to detect
        readiness, so keys sent too early are silently dropped.
- `application/px4_base/03_px4_single_drone_x650.py` (new) — bare X650 (no payload), mirrors
  `01_px4_single_drone.py`'s exact structure with Iris swapped for the calibrated X650 via
  `spawn_x650_with_mavlink`. **`01_px4_single_drone.py`/Iris itself was not touched** by any of
  this work — verified via `git status` on the upstream `extensions/pegasus.simulator/` tree.
- `scripts/start_x650_single_drone.sh` (new) — mirrors `start_single_drone_sitl.sh` exactly,
  pointing at the new bare-drone script above; now also calls `apply_x650_px4_gains.sh` (see
  "X650 PX4 gain tuning" above) 8s after launch.
- Note: `scripts/start_single_drone_sitl_payload_x650.sh` already existed (pre-dates this
  session's work, launches the payload variant) but lacks the tmux kill-pane cleanup pattern the
  other launch scripts in this repo have — not fixed here since it wasn't asked for. It does now
  also call `apply_x650_px4_gains.sh`, same as the other two X650 launch scripts.

**X650 variable-length cable variant added and confirmed working (2026-07-13)** —
`application/slungload/px4_single_drone_payload_variable_length_cable_x650.py` (new): same
winch mechanism as `02_px4_single_drone_payload_variable_length_cable.py` (rod_a/rod_b +
prismatic joint + `ROS2CableWinchBackend`), with Iris swapped for the calibrated X650 via
`spawn_x650_with_mavlink` — same swap pattern as the fixed-length-cable variant above. Payload
mass (`rod_b_mass+payload_mass=0.565kg`) is unchanged from the Iris version — that constraint
comes from `cable_torque_ctrl_node`'s deployed mass param (AK40-10 side), not the carrying
vehicle's lift capacity, so it doesn't change just because X650 has far more thrust margin.
`scripts/start_single_drone_sitl_payload_variable_cable_x650.sh` (new) mirrors
`start_single_drone_sitl_payload_variable_cable.sh` exactly, pointing at the new script, and also
calls `apply_x650_px4_gains.sh`. Neither the Iris variant nor its launch script were touched
(verified via `git status`).

## Propeller bench test data (`docs/propeller_testing/`)

Real motor+propeller bench-test reports — reference data, not code. `MN_4014_15x5` is now wired
into the X650 vehicle's thrust curve (see below); `MT_2216_10x4.5` is not yet connected to any
vehicle config.

- `MT_2216_10x4.5_report.pdf` (generated 2026-06-05) and `MN_4014_15x5_report.pdf` (generated
  2026-07-12) — one report per motor+prop combo tested.
- Each report gives: **Step 1** polynomial fits of throttle → RPM/Thrust/Torque (with R², all
  ≥0.996 in both reports); **Step 2** throttle→RPM response lag lookup tables; **Step 3** a
  first-order RPM-response bandwidth model, `RPM_rate = λ·(RPM_cmd − RPM)`, comparing 4
  estimation variants (`direct_all`/`direct_filtered`: λ from the model equation at each sample;
  `lag_all`/`lag_filtered`: λ = 1/τ from the measured step-response lag directly — a
  system-ID estimate independent of the model equation).
- The methodology page lists companion CSV outputs (`global_fit_coeffs.csv`,
  `lag_vs_throttle_rate.csv`, `lambda_lookup_<variant>.csv`, etc.) — **only the summary PDF
  reports are currently in this repo, not the underlying CSVs.**
- **`MN_4014_15x5` is now fully connected**: its Step 1 throttle→RPM/thrust/torque polynomials
  *and* its Step 3 spin-up-lag (λ) model are both applied to the X650 vehicle — see the X650
  section above for the full derivation, `x650_bare_frame_utils.py`'s calibration, and
  `lagged_thrust_curve.py`'s `LaggedQuadraticThrustCurve`. `MT_2216_10x4.5` remains unconnected
  to any vehicle config as of this writing.

## Normalized swing state (r, v) — JGCD paper data feed

`extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/utils/swing_state_backend_utils.py`
(new) — `ROS2SwingStateBackend`, publishes the normalized swing state `r`/`v` used by the JGCD
paper at `~/source/JGCD-paper-longhao/main.tex` ("Robust Neural Contraction Slung Load
Manipulation With Active Cable Length Control"), for that paper's still-empty "Simulation
Verification" section.

- **Definitions** (derived from `main.tex`, not guessed): cable vector
  `l_c = x_q - x_p` (quadrotor position minus payload position, main.tex:793), cable length
  `l = ||l_c||`, unit direction vector `n = l_c/l = [r; sqrt(1-r^T r)]` (`eq:def_cable_vector`) —
  so **`r` is the horizontal (x,y) position difference between drone and payload, divided by the
  total 3-D cable length `l`** (`r = (x_q-x_p)_{xy} / l`, i.e. the horizontal components of the
  unit cable-direction vector `n`, equivalently) — normalized by the actual straight-line cable
  length, **not** by the horizontal distance itself or any fixed reference length. Matches
  main.tex's own description: "the 2-D offset of the quadrotor when the cable is 1m long"
  (main.tex:130); dimensionless, bounded by `||r||<=1`. **Not** the winch's own
  extension/extension-rate (already published separately by `ROS2CableWinchBackend` as
  `cable_winch_0/state/cable`) — a different physical quantity (swing geometry vs. cable length
  itself).
- `v = rdot` is computed **analytically**, not by finite-differencing `r` — deliberately, per
  this repo's own established lesson (differentiating a noisy/irregularly-sampled quantity
  amplifies noise; see the AK40-10 emulator's `on_state_cable()` timing bug in the WIP section
  below) and because the closed form is directly available here: `l_dot = n·(v_q-v_p)`,
  `n_dot = ((v_q-v_p) - n*l_dot)/l` (`eq: B_property`'s `B` matrix, whose top block is `I_2`, so
  `n_dot[:2] == rdot` by construction), `v = n_dot[:2]`. `v_q`/`v_p` come directly from
  `dynamic_control.get_rigid_body_linear_velocity` (drone body + payload), never differentiated.
  Verified numerically (standalone, no Isaac Sim needed): analytic `v` matches a finite-
  difference of `r` at `dt=1e-6` to ~1e-7 agreement.
- Publishes two `geometry_msgs/Vector3Stamped` topics (`z` always 0, `r`/`v` are 2-D):
  `<topic_prefix>r` and `<topic_prefix>v`, default prefix `swing_state_{swing_id}/`. Follows the
  same per-instance-unique-node-name pattern as `ROS2CableWinchBackend`
  (`simulator_swing_state_{swing_id}`).
- Wired into **both** variable-length-cable scenarios as `swing_state_0/{r,v}`:
  `application/slungload/02_px4_single_drone_payload_variable_length_cable.py` (Iris) and
  `application/slungload/px4_single_drone_payload_variable_length_cable_x650.py` (X650) — not
  wired into the fixed-length-cable scenarios, since this is specifically paper-verification data
  for the active-cable-length-control system.
- **Bug found on first live test (2026-07-14) and fixed**: `TypeError: Can't instantiate abstract
  class ROS2SwingStateBackend with abstract method update_state`. `Backend`'s ABC only checks
  that a method *named* `update_state` exists on the subclass (not its signature - e.g.
  `ROS2CableWinchBackend`'s own `update_state(self, extension, extension_vel)` has a completely
  different signature from the base class's `update_state(self, state)` and still satisfies it
  fine), and this class simply never defined one. Fixed by adding a no-op `update_state(self,
  state)` — this backend doesn't need it since it independently reads both the drone's and
  payload's rigid bodies via `dynamic_control` in its own `update_sim_state()` physics callback,
  the same pattern `ROS2CableWinchBackend` uses. Affects both scenarios it's wired into (single
  shared class, one fix covers both).
- **Known simplification, flagged to the user, not yet confirmed**: `x_q`/`v_q` are read from the
  drone's own rigid-body prim (`uav_path`, the same one the spherical joint attaches to), i.e.
  the drone's own COM — the small `uav_hook_local` cable-attachment offset from that COM is
  ignored. This matches `main.tex`'s idealized model (`x_q` is just "quadrotor inertial
  position," no hook offset anywhere in the math), so it's the more faithful choice of the two
  options, not an arbitrary shortcut — but if the paper's verification actually wants the real
  physical attachment point instead, this needs revisiting.

## Work in progress: variable-length slung-load cable + AK40-10 Isaac Sim emulator

Design doc: `docs/design_requirements/design_requirements.txt`. Full implementation plan (context,
rationale, verification steps): `/home/longhao/.claude/plans/gentle-crunching-treehouse.md`.

Goal: a slung-load variant whose cable has variable length (2 rigid rods + a prismatic joint,
joint force = cable tension), bridged over ROS 2 to the real AK40-10 winch-actuator stack in
`fsc_autopilot_ws` (`AK40-10-ROS2-Bridge` driver + `fsc_ak_actuator_ground_control` GUI) via a new
"emulator" ROS 2 package that impersonates the real driver's `ak_motor_cable_control_node` so
those two packages work against it **unmodified**.

**Status — Part 1 (Isaac Sim side, this repo) built and force→extension response CONFIRMED WORKING under manual flight test:**
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/constraints/constraints_utils.py` — added `create_prismatic_joint` (no DriveAPI, pure kinematic constraint)
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/slung_load/variable_length_cable_utils.py` (new) — `create_prismatic_rod_pair`, `setup_variable_length_cable_geometry`
- `extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/utils/cable_winch_backend_utils.py` (new) — `ROS2CableWinchBackend`: publishes `cable_winch_0/state/cable` (extension, extension velocity) + `state/force`, subscribes `cable_winch_0/command/force`, applies it as a force pair each physics step
- `application/slungload/02_px4_single_drone_payload_variable_length_cable.py` — populated (was an empty placeholder): drone ↔(spherical)↔ rod_a ↔(prismatic)↔ rod_b ↔(spherical)↔ payload, wired to the new backend
- `scripts/start_single_drone_sitl_payload_variable_cable.sh` (new) — launch script for this scenario, same tmux/PX4 pattern as `start_single_drone_sitl_payload.sh`

**Runtime test log:**
1. Full stack launches cleanly: PX4 SITL connects over mavlink, Isaac Sim runs with no exceptions/tracebacks, visually the rods/payload behaved correctly (confirmed by user).
2. ROS2 topics (`/cable_winch_0/{state/cable,state/force,command/force}`) all present and publishing; `ros2 topic info --verbose` confirms the subscriber is connected.
3. **Bug found** (ground/static test, no pilot input): commanding a force via `ros2 topic pub .../command/force` had **no effect on extension** — `state/force` correctly echoed the commanded value (proves the ROS2 subscribe/callback path works), but `state/cable` position stayed pinned at ~0.004 regardless of force magnitude/sign/duration (tested sustained pubs up to 8s). Isolated to the physics application step, not ROS2.
4. Ruled out: joint limits sitting exactly on the flush starting position (`lower_limit=0.0` == initial translation) — widened to `[-max_cable_extension, +max_cable_extension]` as a diagnostic, no change.
5. Applied fix: `update_sim_state` was calling `dc.apply_body_force(..., global=True)` with a world-frame force vector — the only other precedent in this codebase (`vehicle.py`'s `apply_force`/`apply_torque`) always uses `global=False` (local frame). Changed to apply the force in each rod's **local** frame (`global=False`) using `self._axis_local` directly (valid since rod_a/rod_b never rotate relative to each other). Also added a throttled debug `print(...)` every 60 physics steps in `update_sim_state` showing `rb_a`/`rb_b` handles, `pos_a`/`pos_b`, `axis_world`, `extension`, and the applied local force.
6. **Re-verified with the sim still on the ground/no pilot input (office machine, `fsc_lab_machine` config)**: still no measurable response to a sustained ±1 N force over 6s+ (`state/cable` position drift ~1e-6 m, well below the ~meters of displacement 1 N should produce on the 0.02 kg rod). Looked like the fix hadn't worked.
7. **Root cause of steps 3/6, resolved**: it was never a frame/API bug — the drone was sitting static/on the ground (not actually hovering), so there was no real gravity-loaded cable, and the test forces (±1 N) were below the weight the joint needs to overcome to move at all. With the drone actually hovering (piloted via QGroundControl) and the payload hanging freely, **contracting the cable requires the commanded force to exceed the combined weight of everything below the joint**: `(rod_b_mass + payload_mass) × g = (0.02 + 0.3) × 9.81 ≈ 3.14 N` with default masses. Manually tested: 1 N and 3 N → no contraction; 3.14 N and 3.5 N → contracts. The cutover landed almost exactly on the computed equilibrium value, confirming this is real physics (gravity/tension equilibrium), not an engine artifact — **force→extension path is working correctly**. `rod_a`'s weight doesn't factor in (drone-side segment, reacted through its own spherical joint).

**Part 1 cleanup: done** — removed the throttled debug `print(...)` from `update_sim_state` (`cable_winch_backend_utils.py`), and reverted the prismatic joint's limits from the symmetric `±max_cable_extension` diagnostic back to the physically-correct `lower_limit=0.0` (can't retract past flush) / `upper_limit=max_cable_extension` in `application/slungload/02_px4_single_drone_payload_variable_length_cable.py`, now that the diagnostic is confirmed to have been a red herring (real cause was the static/grounded-drone test setup, see step 7 above).

**Status — Part 2 (`AK40-10-Isaac-Sim-Emulator`, own repo at `fsc_autopilot_ws/src/AK40-10-Isaac-Sim-Emulator`, pushed to `github.com/FSC-Lab/AK40-10-Isaac-Sim-Emulator`) built, SPEED mode + EXTERNAL mode + requirement #5 all verified working; requirement #6 next:**
- Package scaffolded (own git repo, cloned the layout of `isaacsim_optitrack_ros2_emulator`): `CMakeLists.txt` (+ `src/CMakeLists.txt`), `package.xml` (ROS2 package name `ak_motor_cable_control_emulator`), `config/cable_control_emulator_params.yaml`, `launch/cable_control_emulator.launch.py`, `src/ak_motor_cable_control_node.cpp` (node name `ak_motor_cable_control_node`, the drop-in name the real GUI/`cable_torque_ctrl_node` expect), `README.md`, `CLAUDE.md`. Ported from `AK40-10-ROS2-Bridge/src/ak_motor_cable_control_node.cpp`: full ROS2 interface (all topics/services/params except CAN/MIT-protocol-specific ones), external-mode 3-state machine + authority hierarchy, command/heartbeat watchdog *detection* (fallback behavior changed, see below).
- **SPEED mode required real debugging, not a direct port** — three iterations, in order:
  1. First attempt kept the real driver's convention (`~/command.velocity` = rad/s, pure damper `force=kd_speed*(v_des-v)`, `kd_speed=0.5` via `effective_radius=0.036`). Produced ~0.009 N of force for a typical command — three orders of magnitude below the ~3.14 N gravity load on the default payload. No visible response ("no response in the cable" — user report).
  2. Switched `~/command.velocity` to m/s directly (dropped `effective_radius` from this path) and added genuine PD against a position setpoint integrated from the velocity command, so a `kp` term could reject the constant gravity disturbance (a pure damper fundamentally cannot). First gain guess (`kp_speed=2000, kd_speed=50`) implied a ~79 rad/s natural frequency — too stiff for the ROS2↔Isaac-Sim (GUI-rendering, sub-100Hz) control loop, destabilized the physics solver ("payload moves very fast, simulation hit a glitch" — user report).
  3. Added an explicit `gravity_feedforward_n` term (mirrors `cable_torque_ctrl_node`'s own `tau_p=-m*g*r` feedforward) so `kp_speed`/`kd_speed` only handle tracking dynamics, not disturbance rejection — dropped gains ~100x (`kp_speed=10, kd_speed=4`, ~5.6 rad/s). **User confirmed this works.**
  4. One more bug found after that: the real driver's `kd_watchdog` fallback (weak pure damper, no feedforward) is safe on real hardware because the winch's mechanical friction prevents free-fall — this sim's cable has none, so a stale command (e.g. the GUI's Stop button publishes one command then stops republishing) let the payload drop to the ground ~500ms later. Fixed by removing `kd_watchdog` entirely: any time SPEED isn't actively tracking a fresh command, the same PD+feedforward law now freezes the setpoint at the current position and holds it — "stale" means hold, not fall back to passive damping.
- Full design rationale, the sign-convention table, and all current parameter defaults are documented in the emulator's own `CLAUDE.md` (`fsc_autopilot_ws/src/AK40-10-Isaac-Sim-Emulator/CLAUDE.md`) — read that before touching SPEED-mode gains or units.
- **EXTERNAL mode: verified working, including requirement #5.** Getting here took two real bug fixes (both user-caught from live plots):
  1. `~/arm`-ing `cable_torque_ctrl_node` (no `~/reference`, its own default gravity-hold-at-arm-position law) with the drone hovering produced growing oscillation and repeated torque-limit clamping — even plain SPEED-mode hold was fine, isolating this to the EXTERNAL-mode path. **Fixed**: the emulator's shaft-dynamics low-pass filter (`omega_filtered_retract_`) was built for its own internal force computation but never applied to what actually gets *published* — `~/cable_state`/`~/joint_state` were still sending raw, noisier Isaac Sim velocity straight to `cable_torque_ctrl_node`'s sliding-mode/L1-adaptive law, which is tuned for real hardware's much cleaner encoder feedback and chattered on the noise. Both topics now publish the same filtered value.
  2. `cable_torque_ctrl_params.yaml`'s deployed `mass: 0.565` kg (real hardware's payload, baked into gravity feedforward and the L1 adaptive law's nominal init at startup, no override path or live-update support) didn't match this scenario's actual hanging load (`rod_b_mass+payload_mass` was `0.32` kg — nearly half). **Fixed** by changing the sim's own default masses to match (`application/slungload/02_px4_single_drone_payload_variable_length_cable.py`'s `payload_mass` default `0.3→0.545`, so `rod_b_mass(0.02)+payload_mass(0.545)=0.565` exactly), and updating the emulator's `gravity_feedforward_n` default to match (`3.14→5.54` N, SPEED mode only).
  3. Also checked and ruled out as contributors: Iris lift capacity (1.5 kg body, ~41 N max thrust vs ~20.5 N loaded weight — ~2:1 margin, confirmed by directly querying the USD asset's mass) and PX4 attitude-gain mismatch (SPEED mode, which bypasses `cable_torque_ctrl_node` entirely, stayed clean with the same heavier payload).
  - **After both fixes**: `cable_square_ref_node` + `cable_torque_ctrl_node` (both unmodified) correctly track a real commanded reference against the sim cable — **requirement #5 confirmed working**. A small bounded oscillation (chatter) persists, traced to the controller's own `smc_phi=0.15` sliding-mode boundary layer width interacting with this sim's noise profile — not a bug, not growing, effort stays within limits; not chased further since the controller can't be modified. Full write-up in the emulator's own `CLAUDE.md`.

**Open investigation (2026-07-11): why does the SMC controller chatter in sim but run cleanly on real hardware?** User reports `cable_torque_ctrl_node` (BLSMC/L1) works perfectly on the real AK40-10 rig; in this sim it's the bounded chatter described above. Working theory: Isaac Sim isn't running this scenario in true real-time, so the controller's fixed-100Hz wall-clock polling sometimes sees stale/jittery feedback that a real CAN-bus-fed control loop wouldn't. Findings so far:
- Directly measured `cable_winch_0/state/cable`'s actual wall-clock publish rate (`ros2 topic hz`) against the nominal `physics_dt=1/250s`: **~150–190 Hz actual, both with rendering on and in headless mode (`PEGASUS_HEADLESS=1`, added to `02_px4_..._variable_length_cable.py` this session)** — only ~60–76% of nominal, and headless gave **no meaningful improvement** over rendered, which was initially surprising.
- Root cause of that non-result: the comparison wasn't controlled for GPU contention. `nvidia-smi` showed the GPU (RTX 3080) at 90% utilization, shared across many concurrent desktop processes (Xorg, kwin_x11, several VSCode/Chrome renderer processes, QGroundControl, `qgroundnode`) — Isaac Sim was the largest single consumer (2865MiB) but far from the only one, and **Isaac Sim uses the GPU heavily even in headless mode** (no visible window ≠ no GPU usage — likely still doing offscreen RTX/Hydra passes). CPU was not the bottleneck (load average 4.55 of 20 cores, plenty of headroom).
- Also found, while trying to get `qgroundnode`'s 3D view working as a lightweight (non-Isaac-Sim) way to watch the drone during headless runs: PX4's uXRCE-DDS bridge (`/fmu/out/*` topics) shows **Publisher count 0** even after starting `MicroXRCEAgent udp4 -p 8888` and seeing a full connection handshake in its log — classic symptom of the agent needing to be listening *before* PX4 boots, not attached after. Not yet fixed (deprioritized — orthogonal to the real-time investigation; Pegasus's own `/uav_0/state/*` topics are unaffected, they come from a separate rclpy backend, not PX4's DDS bridge).
- **Controlled re-test, same day**: closed Chrome and Slack (the two identifiable unrelated GPU consumers) and re-measured headless. Result: GPU utilization dropped only 90%→77%, **Isaac Sim's own GPU footprint was unchanged (2865 MiB both times)**, and the measured rate was unchanged too (~140–190 Hz). This ruled out contention from unrelated desktop apps as the cause — but the real cause turned out to be simpler and fixable (see below), so treat the "not rendering" framing at this point in the investigation as superseded.
- **Root cause found via direct profiling**: added `PEGASUS_PROFILE=1` (wall-clock timing around `world.step()` and `ROS2CableWinchBackend.update_sim_state()`, printed every 250 steps) to `02_px4_..._variable_length_cable.py`. With rendering on (even headless — the script was still unconditionally calling `world.step(render=True)`): outer loop only ~41–47 Hz (~22ms/call, max ~44–49ms); our own callback averaged under 1ms (~0.8–0.9ms, max ~3ms) — clearly not the bottleneck. The `ros2 topic hz`-measured ~140–190 Hz was reconciled as ~4 physics substeps firing per outer `world.step()` call (`physics_dt=1/250s` vs `rendering_dt=1/60s` ≈ 4.2:1 ratio) — i.e. **Isaac Sim was still running a full render pass every call even with no window to show** (consistent with the GPU-usage finding above: headless ≠ no rendering work).
- **Fix**: made the step loop's `render` argument conditional — `render=False` whenever `PEGASUS_HEADLESS=1` (there's nothing to display anyway), instead of always `render=True`. Result: outer loop jumped to **~163–267 Hz** (often at or above the nominal 250 Hz), max per-step time dropped from ~44–49ms to ~10–18ms, and the actual `cable_winch_0/state/cable` rate (`ros2 topic hz`) rose to **~195–242 Hz** (78–97% of nominal, up from ~140–190 Hz) with much tighter jitter (max ~11–15ms, down from ~21–42ms). **This is now the permanent default** (tied to `PEGASUS_HEADLESS`, not gated behind the profiling flag) — headless runs skip rendering unconditionally.
- **Residual gap** (250 Hz nominal vs. ~195–242 Hz actual, plus the still-present ~11–15ms jitter) is smaller now but not zero — a genuinely faster/dedicated machine (the shelved remote-server-over-ZeroTier test) would be the next lever if this still matters for the `smc_phi` boundary-layer chatter accepted above. Not pursued further for now given the size of the improvement already achieved. Real-time factor plateaued at **~80%** after the `render=False` fix (measured again after the SMC-cause discussion below, unchanged) — the remaining gap is attributed to PhysX's own CPU solve cost plus the PX4 lockstep mavlink/TCP round-trip each physics step, both effectively fixed costs for this scene on this hardware, not something with an obvious further toggle.
- **Conclusion on what caused the chatter**: after both bug fixes (velocity filtering, mass matching) *and* the real-time improvement above, torque/external-mode chatter still persisted essentially unchanged. Since improving sim timing didn't reduce it, **`cable_torque_ctrl_node`'s own sliding-mode boundary-layer behavior (`smc_phi`) is the dominant cause**, not sim timing — sliding-mode chattering near the boundary layer is a well-known, textbook SMC characteristic in any discretized loop, not a sim-specific bug. Likely compounded somewhat by this sim's simulated plant being *less damped* than real hardware (the shaft-dynamics model has viscous drag only, no Coulomb/static friction, a deliberate scope simplification — see the emulator's own `CLAUDE.md`), since real mechanical friction would naturally suppress chattering amplitude more than our simplified model does.

**Methodological conclusion — when to use this sim vs. a custom solver** (recorded 2026-07-11, useful for scoping future work in this repo): this Isaac Sim + emulator setup is **not** the right tool for verifying a controller's theoretical properties (stability margins, exact tracking-error bounds, chattering amplitude as a tuning metric) — for that, a simplified deterministic solver (e.g. a Python RK4 integrator against the nominal plant model) is the right approach, since it's free of rendering cost, PX4 lockstep IPC, and OS scheduling jitter entirely. What this sim setup *is* good for: integration-testing the actual unmodified controller binary against a realistic 3D physics scene and the real ROS2 topology it will face on real hardware, before real experiments — exactly how the two genuine bugs this session (unfiltered velocity feeding the SMC/L1 law, the payload-mass mismatch) were caught, neither of which a pure math simulation would ever have surfaced. This class of validation is well-scoped to **slow-dynamics applications** (aerial manipulation, slung loads — pendulum/cable timescales of seconds), where the measured ~10–20ms timing jitter is 2+ orders of magnitude smaller than the dynamics being validated; it would matter far more for fast/high-bandwidth control problems (attitude loops, flexible-structure vibration) where the sim's own timing noise could rival the phenomena under study.

**Lightweight visualization tooling added this session** (`scripts/`, for watching the drone/payload during headless runs without Isaac Sim's own render cost):
- `view_drone_3d.sh` — opens RViz2 (`config/drone_view.rviz`) with a Grid + TF display (drone, via Pegasus's existing `map`→`uav__base_link` broadcast) + a Pose display for the payload (`/payload/state/pose`, published by a `ROS2RigidBodyBackend` newly wired onto the payload in `02_px4_..._variable_length_cable.py` — it had no pose output before). Also starts `cable_line_marker_node.py` in the background (kills it on exit via an `EXIT` trap) — a small standalone rclpy node with no Isaac Sim dependency that subscribes to both poses and publishes a `visualization_msgs/Marker` line between them, since RViz has no built-in way to connect two independently-published poses.
- Tuning notes if the framing ever looks off again: RViz2's Pose display has *separate* size properties per `Shape` — `Shaft Length`/`Head Length` only apply to `Shape: Arrow`; `Shape: Axes` (what's used here) needs `Axes Length`/`Axes Radius` instead, silently ignoring the Arrow-only properties if set instead (this was a real bug hit this session — don't reintroduce it).
- Separately, `qgroundnode`'s own PX4-telemetry-based 3D view was **not** fixed (deprioritized, see above) — the tooling above is the reliable path since it doesn't depend on PX4's uXRCE-DDS bridge at all.

**PD+L1 chatter A/B test node (2026-07-11, in `fsc_autopilot_ws`'s `AK40-10-Isaac-Sim-Emulator` package, not this repo)**: to empirically test the `smc_phi`-boundary-layer conclusion above, built a second controller node, `cable_torque_ctrl_pd_l1_node`, as a near-verbatim copy of `cable_torque_ctrl_node.cpp` with the same public ROS2 interface (drop-in swap in `cable_torque_ctrl.launch.py`) but with the BLSMC sliding-surface/switching term (`s`, `tau_sw`, `smc_eta`, `smc_phi`) replaced by a plain PD term (new `kp_pd`/`kd_pd` params) on top of the same feedforward. The L1 adaptive estimator and UDE monitor are unchanged — valid because the L1 predictor/adaptation law only reads back the previously *applied* torque, not how it was computed.

**First live test surfaced a bigger, unrelated bug**: holding position with zero reference produced a sustained **~23cm limit-cycle oscillation** (confirmed via the node's own `~/debug` topic: `e_p` amplitude was 0.228m in the first quarter of a 12s window and 0.227m in the last — not decaying, a true limit cycle, not settling ringing). Traced to the `coulomb_friction` feedforward term, copied unchanged from `cable_torque_ctrl_node`'s deployed value (real hardware's calibrated static friction) — but this sim's shaft-dynamics model has **no Coulomb friction at all** (viscous drag only, an explicit scope decision). At the oscillation's velocities, `tanh(omega/deadband)` saturates to essentially `sign(velocity)`, i.e. a relay force always in phase with velocity that does net positive work every cycle since there's no real friction for it to cancel — a textbook friction-overcompensation limit cycle. **Fixed** by removing the Coulomb term from this node entirely (not zeroing it — no physical friction here for it to ever compensate), keeping viscous drag only. **User confirmed live: controller is smoother, no chattering, holding position cleanly.**

**Second bug, found getting the live square-wave reference test running**: `cable_square_ref.launch.py` (real driver package, must stay unmodified) hardcodes its reference remap to `/cable_torque_ctrl_node/reference`, not parameterized the way `cable_torque_ctrl.launch.py` is — so it never reached `cable_torque_ctrl_pd_l1_node`. Fixed by running the same unmodified `cable_square_ref_node` executable directly (`ros2 run`, not the launch file) with a manual remap. **Third, self-inflicted bug** on the first attempt at that fix: an unquoted `-r ~/reference:=...` gets tilde-expanded by bash itself into the user's home directory path before `ros2` ever sees it, so the remap silently failed with no error. Fixed by quoting the whole remap argument.

With all three fixed, the live A/B test (real square-wave reference driving `cable_torque_ctrl_pd_l1_node`) is now running. **This meaningfully updates the smc_phi conclusion above**: the Coulomb-friction-feedforward mismatch (present in the real `cable_torque_ctrl_node` too, against this same frictionless-in-Coulomb-terms sim plant) is at least a major contributor to the originally observed chatter — the SMC switching term may have been masking/absorbing this effect as small bounded chatter rather than being its root cause. Not yet isolated exactly how much of the original chatter was `smc_phi` vs. this friction mismatch — full write-up (including a proposed follow-up test: re-running the original `cable_torque_ctrl_node` with `coulomb_friction` diagnostically overridden to 0) in the emulator package's own `CLAUDE.md`.

**Next steps (in order — see the plan file for full detail):**
1. Optional follow-up to fully separate the two chatter causes: re-run the original `cable_torque_ctrl_node` (BLSMC) with `coulomb_friction` overridden to `0` via a command-line param override (diagnostic only, not a permanent change to its deployed config) and see whether its chatter also disappears — would show how much of the original chatter was `smc_phi` vs. the friction mismatch.
2. Validate requirement #6: bring up the `ak_motor_ground_station` GUI against the same running stack (emulator + `cable_torque_ctrl_node`, external mode active) and confirm Stop/Emergency-Stop/Exit-External correctly override/interrupt the active external controller.
3. Optionally: explicitly verify the `off→standby→running` state transitions and `RUNNING→STANDBY` fallback in isolation (implicitly exercised during the above testing, but not checked step-by-step on their own).
4. Optionally: fix the uXRCE-DDS agent/PX4 ordering issue so `qgroundnode`'s 3D view works (restart PX4 with the agent already running first) — lower priority now that `view_drone_3d.sh` covers the same need without that dependency.

## Checklist for adding a new vehicle model

When creating a new vehicle (config class + vehicle class), verify each item below.

### Config class
- [ ] All fields are **pure data** — no live resources (ROS nodes, sockets, processes) instantiated in `__init__`
- [ ] `self.backends = []` or `self.backends = None` — never `[SomeBackend(...)]`
- [ ] Same rule applies to `self.sensors`, `self.graphical_sensors`, `self.graphs` if they carry live state

### Vehicle class `__init__` signature
- [ ] Default `config` argument is `None`, **not** `config=MyConfig()` — Python evaluates default arguments once at import time, creating shared mutable state across all calls
  ```python
  # Wrong
  def __init__(self, ..., config=MyVehicleConfig()):

  # Correct
  def __init__(self, ..., config=None):
      if config is None:
          config = MyVehicleConfig()
  ```
- [ ] Default backend (if any) is created **inside** `__init__` after resolving `None`:
  ```python
  if config.backends is None:
      config.backends = [MyDefaultBackend()]
  ```

### ROS 2 backend
- [ ] The backend creates **exactly one** `rclpy` node per instance — verify the node name is unique (include `vehicle_id`)
- [ ] `rclpy.init()` is wrapped in `try/except` (already initialised is not an error)
- [ ] Publisher/subscriber QoS matches what the counterpart node expects (`qos_profile_sensor_data` = BEST_EFFORT)

### Single-link floating articulation (no external USD)
- [ ] `ArticulationRootAPI` and `RigidBodyAPI` are applied to the **same** prim — PhysX 5 does not form a valid articulation when they are on separate prims
- [ ] Override `update_state` to read from `self._stage_prefix` (not `+ "/body"`)
- [ ] Call `apply_force` / `apply_torque` with `body_part=""`
