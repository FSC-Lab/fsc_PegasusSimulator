# Aerial-Manipulator Simulation — Command Reference

Running record of the commands used to launch, batch-run, and post-process the
**aerial-manipulator** (X650 quadrotor + 4-DOF arm, `AM_realign.usda`) scenarios,
plus (§7) the **T650 DIRECT-actuation** stack, which is a different vehicle and a
different repo pairing but is run from this machine alongside them.
The bare-X650 / slung-load launchers under `scripts/indoor_sim/` and
`scripts/outdoor_sim/` are still not covered here.

All commands are run from the repository root:

```bash
cd /home/shiqi/fsc_PegasusSimulator
```

Every launcher takes a **machine config name** (no `.conf` suffix), resolved from
`scripts/config/<name>.conf`. On this machine that is **`shiqi_machine`**
(available: `shiqi_machine`, `fsc_lab_machine`, `longhao_machine`, `maxwell_machine`).
The config supplies `ISAAC_PY` (`/home/shiqi/isaacsim/python_r_fsc.sh`),
`FSC_PEGASUS_ROOT`, `PX4_DIR`, and `PX4_MSGS_SETUP`.

The plotting commands below use two shorthands. Paste these into your shell once
per session (the launchers do **not** export them — they are only for the
copy-paste commands in this document):

```bash
export ISAAC_PY=/home/shiqi/isaacsim/python_r_fsc.sh
export AM=extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm
```

---

## 1. The six aerial-manipulator rigs

| Rig | Launcher | Isaac entrypoint | Controller | Rotor path |
|---|---|---|---|---|
| **Free-flying in-process** *(current default)* | `start_aerial_manipulator_free.sh` | `02_aerial_manipulator_free.py` | `controller.py` + `config/free.yaml` (no posture anchor, GMO observer) | direct `input_ref` |
| **Pick-and-place in-process** | `start_aerial_manipulator_pick.sh` | `02_aerial_manipulator_pick.py` | `controller.py` + `config/pick.yaml` (same law, `pickplace*` plans in utils_planner) | direct `input_ref` |
| **Push in-process** | `start_aerial_manipulator_push.sh` | `02_aerial_manipulator_push.py` | `controller.py` + `config/push.yaml` (same law, `push_home` plan) | direct `input_ref` |
| Track in-process | `start_aerial_manipulator_track.sh` | `01_aerial_manipulator_track.py` | `controller_track.py` (posture anchor, no observer) | direct `input_ref` |
| Hover, ROS 2 two-process | `start_aerial_manipulator_hover.sh` | `01_aerial_manipulator_hover.py` | `controller_hover.py` (separate node) | ROS 2 topics |
| Free flight + PX4 direct-actuator | `start_px4_direct_aerial_manipulator_free.sh` | `03_px4_direct_aerial_manipulator_free.py` | `controller.py` + `config/px4_direct_free.yaml` in-process, rotors gated by PX4, lagged motors | PX4 `ActuatorMotors` via bridge |

Each launcher opens a **new terminal window** and takes the machine config name as
its first argument.

**`free` and `px4_direct` also take an optional second argument — the trajectory:**

```bash
./scripts/start_aerial_manipulator_free.sh shiqi_machine circle_drone
./scripts/start_px4_direct_aerial_manipulator_free.sh shiqi_machine hover_drone
```

It overrides the in-script `MODE` (§2). `AM_SWEEP`'s `traj_type` in turn wins over
the argument — that is the batch path (§3). `pick`, `push`, `track` and `hover`
take the config name only; select their task with the in-script `MODE` or
`AM_SWEEP`.

### 1.1 Free-flying in-process demo — the usual run

This is the rig currently configured for the **poly_whole showcase flight**
(`MODE = "poly_whole"` in `02_aerial_manipulator_free.py`):

```bash
./scripts/start_aerial_manipulator_free.sh shiqi_machine
```

Single Isaac Sim process, no PX4, no ROS 2. The whole-body law runs as a physics
callback at the 250 Hz physics rate. Log is teed to `/tmp/aerial_manip_gmo.log`;
the run writes `…/robotic_arm/results/log/gmo_log.npz` on exit (Ctrl+C or auto-stop).
Plot it with §5:

```bash
$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/gmo_log.npz --tag compatible
```

### 1.2 Pick-and-place in-process demo (physical interaction)

```bash
./scripts/start_aerial_manipulator_pick.sh shiqi_machine
```

Same single-process rig as §1.1 and the same whole-body law, but driving
`controller.py` under `config/pick.yaml`, with the `pickplace*` plans from
utils_planner and real physics props
spawned in the scene: a **0.30 m table at each end** and a tall graspable
**block** standing on the pick table. This is the rig where the controller meets
**contact forces and an unmodelled 0.2 kg payload** — the disturbance the GMO
exists to reject.

The default `MODE = "pickplace_home"` is a full **ground-to-ground** mission
along **world +y** — the direction the arm faces — with the drone doing all the
aiming. 39 s of trajectory after the 4 s takeoff, plus a 2 s rotor ramp-off:

| phase | arm | what happens |
|---|---|---|
| takeoff | **folded HOME** [0,40,40,0]° | climb to 1.5 m on the software PD arm hold |
| settle | home | 3 s at the anchor |
| fly-out | **home → grasp** | translate to the pick table *while the arm unfolds* |
| pick | grasp (q = 0) | descend alongside the block → close → climb out |
| carry | grasp | translate to the place table |
| place | grasp | descend → release → climb out |
| fly-land | **grasp → home** | translate one more leg **past** the place table *while the arm folds* |
| land | home | vertical descent onto the legs, then rotors ramp off |

The flight **ends past its own work** rather than returning to launch: the last
transit is one more +y leg to `land_wp` (default `[0, 3.5, 0]`, 1.0 m beyond the
place table) and the vehicle sets down there. Measured clearance at touchdown —
the place table spans y 2.511…2.811 and the nearest rotor sits at y 3.29, so
**0.46 m** of margin, with the folded arm pointing away from the table; the
transit passes over the table with 0.46 m between the landing legs and the
placed block's top, and the block is confirmed undisturbed. Set
`land_wp = [0, 0, 0]` for a return-to-launch finish instead.

The arm has exactly **two setpoints** and the schedule min-jerks between them
during the two transit flights — nothing is planned, and the arm only moves
while the vehicle translates, never while it is in contact with the block. The
jaws are driven by a **constant closing torque**, not a position servo — a servo
relaxes and drops the block.

**The folded home pose is required, not decorative.** At q = 0 the finger tips
reach 0.356 m below the base — 51 mm **below ground** at the measured 0.305 m
resting body height — so the vehicle cannot sit down with the arm hanging.
Folded (β = q₂+q₃ = 80°) the lowest arm point (the elbow) keeps 0.140 m of
clearance and the resting CoM is 0.288 m. It is also the best-conditioned fold
direction, and the same pose `poly_whole` lands on — which is why
`land_drop` is the same 1.17 m: with the arm at the same pose at both ends, the
EE drop equals the CoM drop.

Ground contact is **not** in the control model, so both ground phases run the
software PD arm hold (`TAKEOFF_ARM_HOLD` for the climb, the `LAND_*` block below
`LAND_HOLD_Z` = 1.0 m for touchdown, which also freezes the GMO). The landing
hold tracks the pose the trajectory is *already* commanding — falling back to the
takeoff `q_hold` there would swing the arm just as the legs touch.

`MODE = "pickplace_static"` is the same mission without the ground phases: arm
fixed at q = 0 throughout, ends hovering.

**Why the block is tall and gripped near its top** (§2.3 has the measurements):
the fingers reach **64 mm below** the band where they actually pinch, so a
mid-height grasp on a table would drive the finger tips through the tabletop.
The old thin pedestals existed only to give those tips somewhere to hang; with
the grasp moved up near the block's top face, the support can be a real table.

Log is teed to `/tmp/aerial_manip_pick.log`; npz: `…/robotic_arm/results/log/pick_log.npz`.

```bash
$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/pick_log.npz --tag pick
```

The npz records the block's own world position each step (`obj_p`), so the carry
is measurable rather than only visual — check it lifted with the vehicle and
stayed on the place table after release.

### 1.3 Push in-process demo (physical interaction, no grasp)

```bash
./scripts/start_aerial_manipulator_push.sh shiqi_machine
```

Sibling of §1.2 — same rig, same law, same GMO, selected by `config/push.yaml` —
but the interaction is **pushing, not grasping**: the **closed** jaws shove a box
across a table. Nothing is ever held between the fingers; the closed fingers are
simply the pusher, and **the drone does the pushing**.

Why this is the cleaner demonstration of the two: the EE gets a straight **line**
command at **fixed yaw**, and with the arm held at a fixed pose the
dynamically-compatible CoM reference of that line degenerates to the line minus a
constant CoM→EE offset — exactly what the FK-consistent schedule reference already
produces. So the reference model contains no contact at all, and the contact force
is carried entirely by the task impedance plus the GMO. That is the point of the
run.

Default `MODE = "push_home"`, a ground-to-ground mission along **world +y** (the
direction the arm faces):

| phase | arm | what happens |
|---|---|---|
| takeoff | **folded HOME** [0,40,40,0]° | fly the body to the task start on the software PD arm hold |
| fly-out | **home → push** | translate above the push start while the arm unfolds to q = 0 (tool vertical) and the jaws **close** |
| approach | push pose | vertical descent so the hanging fingers sit **beside** the box |
| push | push pose | base flies a **straight +y line** at constant height and fixed heading; fingers meet the box after a small spawn gap and slide it ~0.32 m against sliding friction |
| retreat | push pose | back off along −y (contact ends), climb out |
| fly-land | **push → home** | translate past the table while the arm folds back |
| land | home | vertical descent onto the legs, rotors ramp off |

The arm has exactly **two setpoints** (home and push) and min-jerks between them
during the transit flights only — it is **fixed at the push pose from approach
until after retreat**, so the push geometry never changes while there is contact.

Props (`PUSH_PROPS` block): a 0.10 × 0.12 × 0.064 m, **0.20 kg** box —
deliberately unmodelled — with friction (0.25 static / 0.20 dynamic) on both box
and tabletop, spawned `BOX_GAP_Y` = 0.03 m ahead of the fingers so the approach
is contact-free and the contact transient happens *during* the push. Finger tips
clear the tabletop by `PUSH_TIP_CLEAR` = 0.030 m.

The folded home pose is **required, not decorative** — same reason as §1.2: at
q = 0 the finger tips reach 0.356 m below the base, 51 mm *below ground* at the
0.305 m resting body height, so the vehicle cannot sit down with the arm hanging.

`EE_FORCE_ENABLE` (default `False`) swaps the whole task for the simpler
`const_wrench` check: takeoff → hover → apply a constant 1 N force at the EE and
watch `d_t_hat` converge to it while the vehicle holds position.

Log is teed to `/tmp/aerial_manip_push.log`; npz: `…/robotic_arm/results/log/push_log.npz`.

```bash
$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/push_log.npz --tag push
```

### 1.4 Track in-process demo (posture-anchor baseline)

```bash
./scripts/start_aerial_manipulator_track.sh shiqi_machine
```

Log: `/tmp/aerial_manip_inproc.log`; npz: `…/robotic_arm/results/log/track_log.npz`.

```bash
$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/track_log.npz --tag track
```

### 1.5 Hover demo (pure ROS 2, two processes)

```bash
./scripts/start_aerial_manipulator_hover.sh shiqi_machine
```

Two tmux panes: Isaac Sim + the `controller_hover.py` node under the `/uav_0`
namespace. Log: `/tmp/aerial_manip_isaac.log`. This demo does **not** write an
npz — it predates the history logging, so there is nothing to plot.

### 1.6 Free flight with the PX4 direct-actuator rotor path

```bash
./scripts/start_px4_direct_aerial_manipulator_free.sh shiqi_machine
./scripts/start_px4_direct_aerial_manipulator_free.sh shiqi_machine hover_drone   # pick the trajectory
```

Three tmux panes in session **`am_free_px4`** (the script attaches automatically):
PX4 SITL `none_iris` + `MicroXRCEAgent udp4 -p 8888`, Isaac Sim, and the rotor
bridge. PX4 runs **no** control loops — it is only the arming/offboard gate.
`apply_aerial_manipulator_px4_offboard_params.sh` fires 15 s in.
Logs: `/tmp/am_free_px4_{isaac,px4,bridge,xrce}.log`; npz: `…/robotic_arm/results/log/gmo_px4_log.npz`.

This rig runs the **same full flight machine as §1.1** — setpoint takeoff at the
plan's own start point, hover-error-gated handover, tracked task, hover-gated
setpoint landing, rotor ramp-off — over the PX4 `ActuatorMotors` path instead of
direct `input_ref`. What stays PX4-specific: the `LaggedQuadraticThrustCurve`
motor-delay model (λ = 10.51), the armed+OFFBOARD engagement hold before phase 1,
the 3.416 kg body override mirrored into the control model, and the
`omega_cmd`/`omega_plant`/`omega_real` diagnostics.

> **Its gains are not `free.yaml`'s, and that is a plant property, not a
> preference.** `config/px4_direct_free.yaml` carries `k_R` = 4 / `k_w` = 1.5,
> **half** the in-process rig's `k_R` = 8. At `k_R` = 8 the attitude loop's
> `wn = sqrt(k_R/0.065) = 11.1 rad/s` sits right on the rotor-lag pole (10.51), so
> the moment commanded to cancel the arm's reaction arrives ~90° out of phase and
> pumps a ~2.7 Hz body–arm mode. Also `D_y` = 24 is **not** optional here —
> `free.yaml`'s `D_y` = 6 and 12 both crash; 24 and 48 fly.

To reattach after detaching (`Ctrl+b d`):

```bash
tmux attach -t am_free_px4
```

Run `./scripts/kill_stale_sim_processes.sh` before every launch of this rig (§6).

---

## 2. Choosing the trajectory

The in-script knob is `MODE` near the top of the demo file; it maps 1:1 onto
`utils_planner.TRAJ_CONFIG` (renamed 2026-08-08 to the `<shape>_<who>` scheme:
`<shape>` = the flown path, `<who>` = `drone` for arm-locked / `whole` for
whole-body). Parameters live in `utils_planner.TRAJ_CONFIG[...]`:

| MODE | What it does |
|---|---|
| `hover_drone` | hold the takeoff setpoint, arm locked (ex `hover`) |
| `hover_whole` | **showcase_drone_gimbal**: base hovers pinned while the EE sweeps a closed loop (fold + arm-yaw in quadrature) — the drone is the gimbal |
| `circle_drone` | one rest-to-rest circle, arm locked (ex `circle`; the old `circle_bent` is its optional `q_hold` knob) |
| `circle_whole` | the same circle while the arm folds/unfolds (β 50…80°, FK-exact EE reference) |
| `figure8_drone` | one rest-to-rest figure-8 (Gerono lemniscate), heading on the tangent, arm locked |
| `figure8_whole` | the figure-8 with the same fold sweep as `circle_whole` |
| `poly_drone` | the showcase waypoints flown by the base alone (min-snap segments, arm locked at home, mid-phase yaw cycle swings the EE) — the A/B against `poly_whole`; **lands** |
| `poly_whole` | **showcase_end_effector_gimbal**: 4-phase whole-body demo flight on the offline compatible plan (ex `compatible_showcase`, see §2.1/§2.2) — current default; **lands** |

The classic single-phase `compatible` mode was dropped in the rename; its
offline planner lives on as `poly_whole`'s engine.

The two physical-interaction rigs have their own mission plans in the same
catalogue: **`pickplace_home`** (default, §1.2), `pickplace_static`, `pickplace`,
and **`push_home`** (default, §1.3).

> **The `MODE` comments inside `02_..._pick.py` and `02_..._push.py` are stale.**
> They still offer `"hover"`, `"circle"` and `"circle_bent"`, which were renamed
> away in the 2026-08-08 catalogue rename and are **not** keys of
> `utils_planner.TRAJ_CONFIG` any more. The full authoritative list is the twelve
> keys above (`hover_drone` … `push_home`); check it directly with:
>
> ```bash
> python3 -c "
> import sys; sys.path.insert(0,'extensions/fsc_aerial_manipulation')
> from fsc_aerial_manipulation.robotic_arm import utils_planner as P
> print(sorted(P.TRAJ_CONFIG))"
> ```

To change it without editing the file, pass it as the launcher's second argument
(`free` / `px4_direct` only, §1) or override per run through `AM_SWEEP`:

```bash
AM_SWEEP='{"traj_type":"circle_drone"}' ./scripts/start_aerial_manipulator_free.sh shiqi_machine
```

### 2.0 Scene visualizations

The demo draws four things into the viewport, all pure `UsdGeom.BasisCurves` —
no colliders, no per-step cost, and they persist into screenshots and USD
exports (unlike `debug_draw`, which must be re-issued every frame):

| What | Where | Colour |
|---|---|---|
| **Mocap field edges** | `/World/FlightVolume` — 4.5 × 4.5 × 2.0 m box | green (floor ring a deeper green so the footprint reads from above) |
| **Fly-in start** | `/World/Waypoints/StartFlyIn` | **blue** |
| **Gimbal point** (EE pose held through the pinned phase) | `/World/Waypoints/GimbalPoint` | **red** |
| **Fly-out end** | `/World/Waypoints/EndFlyOut` | **blue** |

Each waypoint is drawn the way the EE command is actually structured: a
**point** for the 3-D position (`UsdGeom.Points` dot) and an **arrow** for the
1-D yaw (`b1_de`) — the only two things the end-effector task controls, so the
marker carries exactly the four commanded numbers and nothing else. The
arrowhead is four barbs in a cross around the shaft so the direction reads the
same from any camera angle. Blue marks the two transit endpoints; red marks the
pinned pose, the one held frozen.

> **Arrow yaw offset.** The EE task's yaw reference `b1_de` is the EE's
> *x-axis*, which on this asset sits 90° off the **mechanical front** (body +y)
> — the same frame quirk `path_yaw_deg` exists for. Drawing the raw `b1_de`
> puts the arrow along world +x while the vehicle faces and travels along +y,
> which reads as wrong. `WP_ARROW_YAW_OFFSET_DEG` (90°) rotates only the
> *drawn* arrow about z, so it still turns exactly with the commanded yaw — the
> same 1-D DOF, referenced to the front instead of the x-axis. Set it back to
> `0.0` alongside `path_yaw_deg` once the USDA has the front on body +x.

The markers are placed from the **anchored** trajectory, so they are drawn when
it is built and redrawn at the climb→track re-anchor — they end up exactly where
the vehicle will fly, not at the nominal spawn estimate. Knobs live in the
`SHOW_WAYPOINT_MARKERS` / `WP_*` and `SHOW_FLIGHT_VOLUME` / `FLIGHT_VOLUME_*`
blocks of `02_aerial_manipulator_free.py`.

### 2.1 The offline compatible planner (behind `poly_whole`; added 2026-08-06)

`MODE = "poly_whole"` runs the offline plan solved by
`extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/utils_planner/compatible_trajectory.py`
(Python port of `refs_matlab/utils/utils_planning/plan_compatible_trajectory.m`).
The planner runs once at `build_traj` (~0.3 s) and prints a summary before the
sim starts — check it before trusting a run:

```
=== compatible-CoM planning (z-x-z port) ===
Picard iterations          : 7  (final step 6.05e-12 m)
MAX dynamic defect |e_dyn| : 3.650e-06 m   <-- should be ~0
min |sin(beta)|            : 7.072e-01     (~0 => wrist singularity)
cond(J_3y^0): start 15 | max 15 | ...
recovered q range [deg]    : min [-0.1 22.5 22.5 -0.] max [25. 34.9 34.9 55.]
```

Keep `TAKEOFF_ARM_HOLD = True` for this mode — the plan starts with the arm at
its consistent `q0` (exported as `tr["q_hold"]`) and the takeoff hold
pre-positions it there so the trajectory begins with zero EE error. Task
amplitudes live in
`TRAJ_CONFIG["poly_whole"]` and are sized to this asset's authored joint limits
(q1 ±35°, q2/q3 −90…+50°); the planner warns if the recovered joint trajectory
would exceed them.

### 2.2 The 4-phase showcase flight, ending in a landing (added 2026-08-06)

`MODE = "poly_whole"` runs the whole-body demonstration flight through
the same planner (phased task, one polynomial per phase). Total 4 s takeoff +
23 s trajectory:

| Phase | Duration | Motion | Arm |
|---|---|---|---|
| takeoff | 4 s | climb to 1.5 m | held **FOLDED at home** β=86° (q = [0, 40°, 46°, 0]) |
| 1 fly-in | 6 s | **straight** +2.0 m y, arcing **up 0.15 m then down 0.35 m** | **unfolds** β 86°→50° |
| 2 pinned | 6 s | **gimbal**: EE position **and** yaw exactly fixed; platform yaws a full cycle 0→+30°→0→−30°→0 | folds/unfolds β 50→80→50° |
| 3 fly-out | 6 s | **straight** on to +4.0 m y, climbing back over the same 0.35 m | **folds back to home** β 50°→86° |
| 4 land | 5 s | pure **vertical descent** of `land_drop` (1.17 m) | held at home through touchdown |

**The home pose is singularity-certified and compactness-optimised** — full
treatment, including why the vehicle now *spawns* in it, in
[Key Items.md](<Key Items.md>) §1. β=86° with an
**asymmetric split of 0.465** (q₂=40°, q₃=46° — *not* an equal fold) was chosen from the MATLAB
singularity analysis (`refs_matlab/utils/utils_singularity/singularity_sweep.m`)
mapped onto this asset: the elbow/yaw singular branches live at **negative**
q₂/q₃ in the asset's sign convention (deepest valley q₃ ≈ −52°; the previously
measured cond≈880 point at q₂=q₃=−43° is that branch), while on the positive
fold line the non-dimensional σ_min *rises* with β — 0.52 at home, 5.2× the
analysis' 0.10 keep-out margin, and 0.48 worst-case over the whole flight
envelope. The planner certifies every generated plan (`min sigma_nd` in its
startup summary; warns below 0.10).

The **split is asymmetric because the two joints have unequal usable headroom**
against their shared +50° limit: the PD-hold→GMO-law handover transient
overshoots q₂ by ~8° but q₃ by ~1° (measured), so an equal split wastes ~7° of
q₃ travel. Biasing to 0.465 tightens the fold from β=80° to 86° — tool reach
0.152 → 0.139 m — while holding q₂ at the same 40° that already flew safely, and
σ_min even improves (0.521 → 0.529). Measured in flight: q₂ peaks 48.3°
(**1.7° margin**), q₃ peaks 47.1° (2.9° margin). Going to β=88° would leave q₃
only 1°. If you want more margin back at a small compactness cost, β=84° with
the same split gives ≈2.9°/4.0° margins for 0.143 m reach.

A **Z-fold** (counter-rotating the elbow, q₃ < 0) is the wrong direction on this
asset: link 3 is a forward +x offset while links 2/4 run down −z, so opposite
signs *straighten* the chain — q₂=+50°/q₃=−30° gives 0.273 m reach (vs 0.139)
and σ_min 0.23, heading into the elbow branch (singular by q₃ ≈ −60°).

**Task shape — a dip, straight from above.** The pinned point sits **0.35 m
below** both the fly-in start and the fly-out end, and each transit arcs ~0.15 m
*over* before descending, so the side view (y-z) shows a clear
climb-over → dip → climb-over → descend profile (EE z: 1.48 → 1.62 → **1.13** →
1.62 → 1.48 → land). `Ay = 0` keeps both transits **dead straight** in the bird
view — measured lateral deviation 0.0 mm — so all the shape lives in the side
view.

**Pinned-phase style — `pin_mode` (`"yaw"` default, or `"vertical"`).** Two
selectable showcases of the same gimbal, both holding the EE to ~0.3 mm:

| `pin_mode` | What the drone does | Vertical travel |
|---|---|---|
| **`"combo"`** *(default)* | **bobs up and down _while_ yawing** — fold runs a FULL cycle (β 50→80→50→20→50) 90° out of phase with the ±30° yaw, so **(yaw, height) traces a circle** | **0.118 m** |
| `"yaw"` | platform sweeps 0→+30°→0→−30°→0; arm counter-rotates 1:1; one-sided fold | 0.051 m |
| `"vertical"` | heading held; arm sweeps β 50→86° so the drone swings up and down | 0.059 m |

The combo mode's gain comes from cycling the fold **both ways**: the drone's
height above the pinned tip is monotonic in β (0.147 m at β=20° down to 0.029 m
at β=80°), so a full cycle more than doubles the vertical travel a one-sided
sweep manages. `pin_phase_deg` (90°) sets the quadrature; 0° would couple yaw
and height into a diagonal instead of a circle.

> **A circle around the target is impossible in either mode.** The offset
> `x_c − r_e = R_0·(r_0c − r_0e)` has **v_z > 0 for every reachable q** — the arm
> can only place the tool *below* its own base — so the drone is always **above**
> the EE. Its reachable set is a patch on the upper side of a ~0.24 m sphere: an
> arc, never a loop. Measured: β=0 → 0.237 m at 36° from vertical; β=50 → 0.171 m
> at 62°; β=86 → 0.133 m at 80°. Closing the loop would need the arm to reach up
> past its base, or the quadrotor to fly inverted. A **large** circle around a
> target needs the position pin dropped — fly the circle and keep the tool
> *aimed* at the point instead.

**Pinned phase is a true gimbal, and its yaw range is kinematically capped.**
The EE position *and* heading are both frozen inertially while the platform
sweeps a full sine cycle 0→+30°→0→−30°→0, returning to forward so the fly-out
continues straight ahead. The cap is hard: with the EE yaw locked,
`R_e^0 = R_0ᵀ R_e` forces **q₁ = −ψ**, so the platform yaw is limited by q₁'s
±35° (30° leaves 5°). A **full 360° spin is impossible** with the yaw lock — it
would need q₁ = −360°, and q₄ cannot help because the two z-rotations in
`Rz(q₁)Rx(β)Rz(q₄)` only trade when β = 0, the wrist singularity. Dropping the
yaw lock (letting the EE heading follow the platform) *would* allow a full
circle, at the cost of the gimbal property.

> **Why the drone barely translates while pinned.** q₁ counter-rotating the yaw
> also counter-rotates the arm's own offset, so `R_0·r_0e` is invariant and a
> pure yaw moves the base **not at all**. All pinned-phase translation comes
> from the arm's *fold* changing `r_0e`, which over β 50–80° is only ~0.03 m
> horizontally but ~0.06 m vertically. Measured in flight: CoM moves 0.059 m
> total. The yaw is the visible motion, not the translation — this is a
> workspace bound, not a tuning choice.

**Travel direction — `path_yaw_deg` (currently 90°).** The controller's heading
reference `b1_d` steers **body +x**, but on this asset the mechanical front (the
arm side) is **body +y**. `path_yaw_deg` rotates only the prescribed EE
*position* about z, leaving every *orientation* signal untouched — so the vehicle
**travels** along world +y (face-first) while its attitude reference stays on +x.
It never yaws to follow the path; the only yaw in the flight is the deliberate
±15° counterpoint during the pinned phase.

> This is a **temporary workaround** for the asset's frame convention. Once the
> USDA is re-authored so the mechanical front is body +x, set `path_yaw_deg` back
> to `0.0` and the demo flies along +x natively. Note a USDA-only rotation is not
> sufficient on its own — `make_params` (both controllers), `RotorMixer`
> positions, `M_r_d`, and the planner's z-x-z joint recovery all encode the body
> frame and must be updated together.

**Arm fold direction**: β = q₂+q₃, and *larger* β folds the elbow and tucks the
EE **up** toward the body (base→EE reach 0.279 m at β=0, 0.205 m at 50°, 0.16 m
at 80°). Folded home = compact, ground-safe, *and* best-conditioned — all three
align, which is why the same pose serves takeoff and landing.

**Landing is outside the control model** (ground contact isn't modelled — the
same reason `TAKEOFF_ARM_HOLD` exists for the climb), so the demo adds two
protections, both in the `LAND_*` block of `02_aerial_manipulator_free.py`:
below `LAND_HOLD_Z` (1.0 m) the arm returns to the software PD hold *tracking the
home pose* and the GMO freezes; once the plan is over the rotors ramp off over
`LAND_DISARM_TIME` (2 s) so the vehicle settles on its legs instead of the
position loop fighting the ground.

Knobs in `utils_planner.TRAJ_CONFIG["poly_whole"]`. Headless-validated
2026-08-06: pinned-phase EE held to **0.28 mm** / yaw error 1e-5 while the base
flew a 11.7 cm path; touchdown settles 3.9 cm at ≤0.34 m/s onto the measured
0.305 m resting height, final thrust back at hover (32.7 N ≈ mg), zero drift
after. Same `TAKEOFF_ARM_HOLD = True` requirement as §2.1.

> If you retune the descent: the vehicle's **resting body height is 0.305 m**
> (CoM 0.290 m), measured from a landing run — *not* the 0.613 m spawn height,
> because the drone lifts off at t=0 and is never seen resting at spawn. Sizing
> `land_drop` against the spawn value leaves the reference 27 cm high and the
> rotor ramp turns that into a drop.

To validate planner changes **without launching Isaac** (pure numpy, system python):

```bash
python3 -c "
import sys; sys.path.insert(0, 'extensions/fsc_aerial_manipulation')
import numpy as np
from fsc_aerial_manipulation.robotic_arm import utils_planner as P
P.set_traj_type('poly_whole')
tr = P.build_traj(np.array([0., 0., 1.5]), np.zeros(3), np.array([1., 0., 0.]))
print('T =', tr['T'], ' q_hold =', tr['q_hold'].round(4))
"
```

### 2.3 The pick-and-place task geometry (measured 2026-08-07)

`MODE = "pickplace_static"` (§1.2) is the only mode that touches the world, so
its props are sized from **measured gripper geometry**, not from the CAD comments
that preceded it. All offsets below are in the EE frame at arm q = 0, +z up the
tool axis, x = the jaw closing axis; they come from a headless `gripper_joint`
sweep (fingers' mesh points transformed into the EE frame).

| Feature | Offset from the EE | Note |
|---|---|---|
| `manip_base` plate underside | −0.046 m | **±64 mm wide** — the real obstruction |
| hub underside | −0.058 m | ±11 mm x, ±20 mm y |
| **pinch band** (narrowest jaw) | **−0.060 m** | where the grip actually happens |
| finger tips | −0.124 m | **64 mm BELOW the pinch** |

Jaw gap at the pinch band vs `gripper_joint` (≈0.55 mm/deg):

| angle | +50° (open stop) | +30° | +15° | 0° | −25° | −50° |
|---|---|---|---|---|---|---|
| gap | **41.9 mm** | 31.0 | 22.7 | 14.6 | 3.0 | fingers interfere |

The jaws are effort-driven, so "open" is the authored +50° limit the constant
opening effort parks them against — 41.9 mm. Note the old
"pad gap ~7 cm open / ~3.5 cm closed" comment measured the grip-link **origin**
spacing, not the jaw; the real jaw is less than half that.

Two constraints follow, and they pull in opposite directions:

1. **The tips hang 64 mm below the pinch**, so a mid-height grasp drives them
   through the tabletop. This is why the old props were thin pedestals — they
   gave the tips somewhere to hang. Grip near the object's top instead and the
   support can be a real table.
2. **The object's top must stay below the hub underside (EE−0.058)**, because
   directly above the pinch band sits the gripper's own body — and `manip_base`
   is a 128 mm-wide plate. The object may poke at most ~2 mm above the pinch.

Violating (2) is not a cosmetic problem. Measured, same trajectory, only the
block's top height changed:

| block top | outcome | arm fold when the jaws close | peak arm torque | max \|e_R\| |
|---|---|---|---|---|
| EE−0.020 | completed, but gripping the top **edge** | q2 → +27°, jaws ride +40 mm up | 1.07 N·m | 0.028 |
| EE+0.010 | **crashed at t = 29.8 s** | q2 → **+50°, its joint limit** | **1.50 = clamp** | 1.00 |
| **EE−0.062** | **completed cleanly** | q2 → 7° | 0.92 N·m | 0.045 |

In every run the EE sat exactly on its reference at t = 14 s — descent complete,
jaws still open — and only moved when the grip **closed**. So this is contact
with the gripper's own body, not a tracking or trajectory problem, and no amount
of gain tuning would have fixed it.

Current props (`PICKPLACE_PROPS` block of the demo): 0.30 m tables at both ends
with the tops at z ≈ 1.146 m, and a 0.03 × 0.06 × 0.10 m, 0.20 kg block whose
top face is placed at EE−0.062. That leaves 5.9 mm of approach clearance per
side on the jaw axis and 38 mm of finger-tip clearance above the tabletop.
Headless-validated over the full 34 s mission: block lifted 492 mm, carried
1.5 m, released 11 mm from the nominal place point and resting exactly on the
table, CoM error ≤0.9 cm, EE error ≤1.3 cm, arm within ±10 mm of its reference
throughout.

---

## 3. Headless / batch runs (`AM_SWEEP`)

`AM_SWEEP` is a JSON dict read from the environment by every GMO demo. It is the
only supported way to change a run without editing the source.

| Key | Type | Meaning |
|---|---|---|
| `headless` | bool | run with no window (also disables rendering in the step loop) |
| `t_end` | float | auto-stop and save after this many seconds of **sim** time |
| `log_path` | str | npz destination (default `…/robotic_arm/results/log/gmo_log.npz`) |
| `traj_type` | str | overrides the in-script `MODE` |
| `k_x`, `k_v`, `k_R`, `k_w` | float | body-loop gains |
| `K_y`, `D_y` | 4-list | EE impedance stiffness / damping (diagonal) |
| `K_o` | 10-list | GMO observer bandwidth per channel (diagonal) |
| `M_r_d` | 3-list | desired attitude inertia (diagonal) |
| `M_Y` | 4-list | shaped-impedance task inertia |
| `DLS_LAMBDA`, `TAU_MAX` | float | J_3y damped-least-squares λ, arm torque clamp |
| `USE_GMO` | bool | master on/off for the observer (GMO-vs-no-GMO A/B) |
| `ARM_ALWAYS_PD_HOLD` | bool | *PX4 demo only* — fly the arm on joint-space PD the whole time |

A crashed run (on the ground or flipped after takeoff) ends early and the npz
carries `crashed=True`.

**Headless run, bypassing the launcher** (what was used to validate the
compatible trajectory — calls Isaac's python directly, so it blocks in the
current terminal and needs no config name):

```bash
AM_SWEEP='{"headless":true,"t_end":40.0,"traj_type":"poly_whole","log_path":"/tmp/compat.npz"}' \
  /home/shiqi/isaacsim/python_r_fsc.sh application/robotic_arm/02_aerial_manipulator_free.py \
  2>&1 | tee /tmp/compat.log
```

**Headless run through the launcher** (new terminal, config-driven):

```bash
AM_SWEEP='{"headless":true,"t_end":60.0,"traj_type":"hover"}' \
  ./scripts/start_aerial_manipulator_free.sh shiqi_machine
```

**Gain override example** (single trial, GMO off for an A/B):

```bash
AM_SWEEP='{"headless":true,"t_end":30.0,"USE_GMO":false,"K_y":[8,8,8,1],"D_y":[6,6,6,1]}' \
  ./scripts/start_aerial_manipulator_free.sh shiqi_machine
```

> **Headless is not the final gate.** A configuration that is clean for 90 s
> headless has crashed at 8.2 s rendered — rendered stepping is bursty (≈4
> physics substeps per render frame) and excites a ~2 Hz coupled body–arm mode
> that headless timing never wakes. Always confirm with a rendered run.
> See `Hover Gain Tuning.md`.

---

## 4. Gain sweeps (PX4 direct-actuator rig)

`application/robotic_arm/utils/px4_gmo_gain_sweep.py` runs the **complete** stack
(PX4 SITL + agent + bridge + headless Isaac) once per trial, tears it down, and
computes hover metrics. Run under **system python3** — it imports numpy only, no
Isaac or ROS in its own process.

```bash
cat > /tmp/trials.json <<'JSON'
[{"name": "baseline", "sweep": {}},
 {"name": "kx24",     "sweep": {"k_x": 24.0, "k_v": 12.0}},
 {"name": "arm_ko_0", "sweep": {"K_o": [1,1,1,1,1,1,0,0,0,0]}}]
JSON

python3 application/robotic_arm/utils/px4_gmo_gain_sweep.py \
  --trials /tmp/trials.json --t-end 25 \
  --out /tmp/sweep_results.jsonl --results-dir /tmp/sweep_npz --retry 1
```

Only one trial stack runs at a time (single GPU; PX4 ports 4560/8888/14540 are
hardcoded). The harness aborts if a foreign controller is publishing on `/uav_0` —
a stale X650 node once cost a full debug cycle because its Iris-convention
allocation is roll-mirrored on this asset's rotor indexing.

---

## 5. Plotting a run

`plot_results.py` renders the MATLAB-style figure set from an npz into
**`…/robotic_arm/results/`**. It must run under **Isaac's python** (system
python3's matplotlib is broken here by a numpy 1.x/2.x conflict).

**Plot the run you just finished** — with no path argument it picks the newest
`.npz` in `log/`, so this always plots the most recent run of any rig:

```bash
$ISAAC_PY $AM/utils_plot/plot_results.py --tag compatible
```

It prints which log it chose — `[plot_results] newest log: results/log/gmo_log.npz` —
**read that line**. `log/` accumulates one npz per rig, so if the run you just
did crashed early or never saved, "newest" is a stale log from a different rig
and the plots will silently be of the wrong run. Pass the path explicitly when
in doubt.

**Plot a specific rig's log** (from the repo root, using the shorthands defined
at the top of this document):

| Rig | Command | Tag if `--tag` omitted |
|---|---|---|
| Free-flying in-process (§1.1) | `$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/gmo_log.npz` | `gmo` |
| Pick-and-place in-process (§1.2) | `$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/pick_log.npz` | `pick` |
| Push in-process (§1.3) | `$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/push_log.npz` | `push` |
| Track in-process (§1.4) | `$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/track_log.npz` | `track` |
| Free flight + PX4 direct-actuator (§1.6) | `$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/gmo_px4_log.npz` | `gmo_px4` |
| A sweep trial (§4) | `$ISAAC_PY $AM/utils_plot/plot_results.py /tmp/sweep_npz/<trial>.npz` | the file's basename |

Without the shorthands, the fully-spelled form is:

```bash
/home/shiqi/isaacsim/python_r_fsc.sh \
  extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/utils_plot/plot_results.py \
  extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/results/log/gmo_log.npz \
  --tag compatible
```

Use `--tag` to keep runs side by side in `results/` (e.g. `--tag compatible`
vs `--tag circle`); reusing a tag overwrites that run's five PNGs.

Five figures are written, each suffixed with the tag:

| File | Contents |
|---|---|
| `trajectories_<tag>.png` | **actual vs desired** time histories — CoM `x_c/x_cd`, EE `r_e/r_ed`, joints `q/q_d`, plus an x-y top view. Solid = actual, dashed = desired. |
| `tracking_errors_<tag>.png` | CoM position error, attitude error `e_R`, EE position error, EE yaw error |
| `control_<tag>.png` | thrust `u1`, base moment `τ(4:6)`, applied joint torques `τ_j`, arm task input `u3` |
| `states_<tag>.png` | base position, joint angles, base linear/angular velocity, joint rates, and the GMO estimate `d̂_t` (with the applied EE force dotted, when the log has one) |
| `trajectory3d_<tag>.png` | 3D base / CoM / EE paths, actual vs desired; the takeoff climb is faded so the tracking phase stands out |

**Flight phases are shaded automatically.** The demo records the phase
boundaries in the npz (`phase_t` / `phase_names`), so every time-axis panel gets
one tinted band per phase with its name along the top edge — takeoff (grey),
fly-in (blue), **pinned (amber)**, fly-out (green), land (mauve), hold (grey).
The two spatial panels (top view, 3D) get phase-transition dots instead, since
they have no time axis; boundaries that land on the same point are merged into
one label (e.g. "pinned / fly-out", because the pinned phase barely moves the
CoM). Logs recorded before this was added simply fall back to the single
takeoff line.

Options: `--tag TAG` (filename tag, default derived from the npz name),
`--takeoff T` (marks the takeoff→track handover; defaults to the controller's
`TAKEOFF_TIME`), `--show` (open windows instead of only writing files).

The script also prints an RMSE table over the tracking phase and a **commanded
travel** summary — the span of `x_cd`/`r_ed`/`q_d` — which is the quickest way to
answer "was the vehicle supposed to move that little?" from the log itself.

Generated PNGs are ignored by git (`results/.gitignore`); the folder itself is
tracked so the path always exists.

Quick numeric check without plotting (system python3 is fine — numpy only):

```bash
python3 -c "
import numpy as np
d = np.load('extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/results/log/gmo_log.npz')
t = d['t']; m = t > 5.0
print('crashed:', bool(d['crashed']), ' t_end:', round(float(t[-1]), 1))
print('EE  err max [cm]:', round(float(np.linalg.norm(d['r_e'][m]-d['r_ed'][m], axis=1).max()*100), 2))
print('CoM err max [cm]:', round(float(np.linalg.norm(d['x_c'][m]-d['x_cd'][m], axis=1).max()*100), 2))
print('|tau_j| max [Nm]:', round(float(np.abs(d['tau_j'][m]).max()), 2))
"
```

---

## 6. Cleanup between runs

Kills both halves of any stale stack (Isaac/PX4 side and the ROS 2 side) plus
every tmux session the launchers create:

```bash
./scripts/kill_stale_sim_processes.sh          # scan, then ask before killing
./scripts/kill_stale_sim_processes.sh -y       # kill without asking
./scripts/kill_stale_sim_processes.sh --dry-run # scan only
```

Worth running before every PX4-rig launch. The failure it prevents is silent: a
mocap emulator that outlives Isaac keeps republishing the last pose forever on a
fixed-rate timer, so the ground station shows a plausible but completely fake
position instead of going blank.

---

## 7. T650 DIRECT-actuation stack (two repos)

A **different vehicle and a different architecture** from everything above: the
bare **T650** (Tarot 650 — the X650 airframe with MN4010 + 15x5" motors at
2.95 kg, no arm, sharing `x650_new.usd`), flown by the external
**`fsc_autopilot_ros2`** controller. In DIRECT mode that node owns attitude, rate
**and** mixing; PX4 runs no control at all and is only the arming/OFFBOARD gate.
Contrast §1.6, where the *in-process* whole-body law drives the PX4 actuator path.

Two repos are involved:

| Side | Path on this machine |
|---|---|
| ROS 2 controller stack | `~/ros2_ws/src/fsc_autopilot_ros2` |
| Pegasus / PX4 SITL | `/home/shiqi/fsc_PegasusSimulator` |

> **The upstream workflow's paths are the lab machine's, not this one.** It says
> `~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2` and
> `~/Source/fsc_PegasusSimulator` and passes `fsc_lab_machine`. On shiqi-desktop
> use the paths above and **`shiqi_machine`** — Pegasus's `fsc_lab_machine.conf`
> points at `/home/fsc-jupiter/…`. The config name only matters on the Pegasus
> side; all four confs on the autopilot side are identical
> (`ROS_DISTRO=humble`, `MICROXRCE_AGENT=MicroXRCEAgent`).

### 7.0 Prerequisite: rebuild after pulling `fsc_autopilot_ros2`

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select fsc_autopilot_ros2
```

One package is enough — the root `CMakeLists.txt` pulls in `fsc_autopilot_lib`
via `add_subdirectory`, so it is not a separate colcon package.

**This is not optional after a pull, and it fails silently if skipped.** The
launch file installs unchanged, so a stale build *looks* like it starts, while the
node quietly lacks whatever the new config sets. Verify against the running node
rather than the build tree:

```bash
ros2 param list /uav_0/fsc_autopilot_ros2 | grep -E 'system_fb_|system_wd_|ratectl_trim'
```

(Observed 2026-08-10: an 8-day-old build was missing `system_fb_timeout_s`,
`system_fb_land_speed`, `system_wd_max_rate_dps`, `system_wd_max_tilt_deg` and
`ratectl_trim_{x,y,z}` — i.e. the feedback-loss failsafe and the watchdog were
absent while everything appeared to launch normally.)

### 7.1 Clean slate

```bash
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/stop_isaacsim_stack.sh
```

### 7.2 ROS 2 stack — terminal 1, maximized

```bash
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_direct_actuation_t650_stack.sh shiqi_machine uav_0
```

Six panes: `microXRCE`, `emulator`, `autopilot`, `estimator[raw-mocap]`, `gui`,
`vrc`. The node **starts in SAFETY mode** (position control + UDE publishing an
attitude setpoint — behaviourally the baseline controller).

> **Detach with `Ctrl-b d`.** Not Ctrl-C, not Ctrl-D, not closing the window. The
> script leaves the `microXRCE` pane selected on attach, so Ctrl-C + Ctrl-D there
> kills the agent *and* closes its pane — after which the Pegasus launcher in §7.3
> refuses to start. Observed 2026-08-10, with the `vrc` pane lost the same way,
> which also removes the `rc/arm` and `rc/offboard` services §7.4 needs.
>
> Run it from a **maximized** terminal: six even-horizontal panes on an
> 80-column window are 12 characters wide and unreadable.

Verify before continuing:

```bash
tmux list-panes -t fsc_direct_actuation_t650_stack:stack -F '#{pane_index} #{pane_title}'  # expect 6
pgrep -x MicroXRCEAgent && ss -lunp | grep 8888                                            # expect a pid + a listener
```

**The `vrc` pane is `virtual_rc`** (`ros2 run fsc_autopilot_ros2 virtual_rc`,
2026-09-25; before that `px4_offboard_control virtual_remote` from
fsc_virtual_remote_controller's retired `dev_robotic_arm` branch): it does
nothing on its own — arming, disarming, OFFBOARD and RTL happen only through
its `/uav_0/rc/{arm,disarm,offboard,rtl}` services, §7.4's path. It exists only
because there is no real transmitter in the loop; hardware scripts never launch it.

### 7.3 Pegasus / PX4 SITL — terminal 2

```bash
cd /home/shiqi/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_direct_actuator_sitl.sh shiqi_machine
```

**Order is load-bearing, not stylistic.** This launcher hard-exits if
`MicroXRCEAgent` is not already running (the ROS 2 stack owns it), and it pushes
`PEGASUS_PX4_LOCKSTEP=0` onto the **tmux server** that stack created. Confirm it
prints:

```
Pegasus PX4 lockstep: pushed to the tmux server environment
```

If it instead says *"no tmux server yet"*, the ordering is wrong. A new tmux
session inherits the environment of the already-running **server**, not of the
calling shell, so an `export` alone is silently discarded — Isaac then comes up
with lockstep enabled and entering DIRECT deadlocks the PX4/Isaac HIL link
(`ERROR [simulator_mavlink] poll timeout 0, 111`).

First run initializes a fresh `rootfs_fsc_indoor_t650` PX4 profile (separate from
the X650's, because PX4 `param save`s into it), so expect a slower boot; the gain
script fires 8 s after the `pxh>` prompt.

### 7.4 OFFBOARD, arm, then DIRECT — terminal 3

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**OFFBOARD before arm is required in sim** — arming first is denied with
`arming_check_error_flags = 16777216` (no global position indoors).

Get a stable hover in SAFETY first, then hand over attitude, rate and mixing:

```bash
ros2 service call /uav_0/fsc_autopilot_ros2/direct_actuation/set_direct_mode \
  std_srvs/srv/SetBool "{data: true}"
```

Abort path — keep it ready before entering DIRECT:

```bash
ros2 service call /uav_0/fsc_autopilot_ros2/direct_actuation/set_direct_mode \
  std_srvs/srv/SetBool "{data: false}"
```

### 7.5 What to watch on a T650 DIRECT flight

- **Hover command ≈ 0.503**, not the X650's 0.480 — the MN4010's top rotor speed
  is lower (730.05 vs 817.59 rad/s) at a near-identical thrust constant, so the
  same weight sits higher on the stick. Far off ⇒ suspect the mass/thrust model,
  not the tune.
- **Watch yaw first.** This config is **UNFLOWN in DIRECT**. Its rate-loop gains
  are the X650 carry-over, and against this plant the yaw axis runs at ~2.45× the
  loop gain they were tuned for (`t650_params.py`'s `YAW_TORQUE_FIT_FACTOR` = 3.0).
- Rotor lag is **larger** than the X650's: λ = 10.0265 vs 10.51 1/s (τ 99.7 vs
  95.1 ms), costing phase margin in every inner loop.
- Feedback is **raw mocap**, not EKF2-fused — deliberate, since this node takes
  the loops PX4's estimator would otherwise sit inside. The trade: velocity is a
  finite difference with no IMU fusion, and EKF2's reset/dropout handling is
  bypassed.

### 7.6 Shutdown, in this order

```bash
cd ~/ros2_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh   # ROS 2 first
/home/shiqi/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh               # then the simulator
```

ROS 2 first, so the control node is not streaming setpoints into a dying PX4.
`stop_isaacsim_stack.sh` deliberately does **not** touch PX4/Isaac — those have
their own lifecycle on the Pegasus side.

> Duplicate `/uav_0/uav_0_node` entries in `ros2 node list` after a stop are the
> known graph-ghost artifact (a hard-killed participant never deregisters). Count
> **processes**, not graph entries: `pgrep -af indoor_mocap_feedback_node`.

### 7.7 AM-T650 variant: the aerial manipulator under the DIRECT stack (2026-08-10)

The same two-repo workflow with the **aerial manipulator as the plant**: the
**`AM_xfwd.usda`** X650+arm asset spawned PX4-primary with the **T650 motor
calibration** (MN4010, λ = 10.0265) and **T650 body mass** (2.95 kg re-authored
onto `/body` at spawn; total flying mass **3.746 kg**), while the Isaac process
(`application/robotic_arm/04_px4_direct_t650_aerial_manipulator_hold.py`) does exactly one thing
in-process — PD + gravity-comp holds the arm at its home pose
**[0, 40°, 40°, 0]** (KP 3.0 / KD 0.25 / clamp 3.0 N·m, the flight-validated
02/03 hold, running unconditionally: ground, SAFETY and DIRECT). No whole-body
law, no mixer, no flight machine — the T650 DIRECT-actuation controller flies
the vehicle as a plain quadrotor and never sees the arm. This is the
**safe-fallback integration step** for the future whole-body controller: T650 is
the airframe the real aerial manipulator will be built on.

**`AM_xfwd.usda` is the x-forward re-authoring of `AM_realign.usda`** (which is
untouched and still serves every whole-body demo), generated by
`utils_model/make_x_forward_asset.py` — re-run that script if the file is
missing (assets are gitignored, distributed out-of-band). It exists because the
first flight of this rig, on the original asset, **flipped on lift-off**:
AM_realign's mechanical front is body +y and its rotor channel indexing is
mirrored vs `x650_new.usd` (ch0 front-LEFT…), which **exactly negates the roll
row** of any standard Quad-X allocation — PX4's own SAFETY-mode mixer included
(observed ω = [427,467,466,427]: PX4 pushed what it believed was the right pair
low; on that asset it was the left pair). In AM_xfwd the arm side is body +x
and, together with the demo's channel 2↔3 prim remap
(`ROTOR_PATHS = [rotor0, rotor1, rotor3, rotor2]`), the plant presents the exact
PX4 Quad-X convention — so the standard T650 allocation and PX4's internal mixer
are both correct without modification.

**SIM-VALIDATED 2026-08-10** (headless, full stack, flown by Claude):
SAFETY takeoff to a streamed z = 1.2 reference → hover held to **4 mm** →
**DIRECT entered in-flight, 50+ s, zero drift, no watchdog trip** → SAFETY
reversion → 0.2 m/s descent to touchdown. Hover command measured **0.5687** vs
0.569 predicted; arm within **0.6°** of home throughout, including ground
contact; standing pitch trim from the forward arm CoM measured at ~37 rad/s
front-over-rear and absorbed cleanly. A rendered confirmation run is still
worth doing (headless timing is the gentler regime).

#### 7.7.1 Run sequence — copy-paste, per machine

> **Script names changed 2026-08-11** (`am_t650` → `t650_aerial_manipulator`).
> The old `start_direct_actuation_am_t650_stack.sh` and
> `start_am_t650_direct_actuator_sitl.sh` no longer exist — if a saved command
> fails with "No such file or directory", this is why.

**fsc_lab_machine** (the lab desktop, user `fsc-jupiter`):

```bash
# 0. clean slate            (any terminal)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_direct_actuation_am_t650_stack.sh fsc_lab_machine uav_0
#   ^ the OLD name — this is the one that RUNS on this machine today. Once the
#     rename is pushed from shiqi-desktop, switch to:
#     ./scripts/isaacsim/start_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 2. Pegasus / PX4 SITL     (terminal 2)
cd ~/Source/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_direct_actuator_sitl.sh fsc_lab_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**shiqi_machine** (shiqi-desktop):

```bash
# 0. clean slate            (any terminal)
cd ~/ros2_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. Pegasus / PX4 SITL     (terminal 2)
cd ~/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_direct_actuator_sitl.sh shiqi_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

Detach a stack terminal with **`Ctrl-b d`** — never Ctrl-C/Ctrl-D. Step 0 is
also the shutdown: run it again to tear everything down.

**In flight** (machine-independent). Takeoff is driven from the ground station:
after arming, SAFETY holds the *ground* position until a position reference
arrives, and nothing lifts off on its own. Once the hover is settled:

```bash
# enter DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/direct_actuation/set_direct_mode \
  std_srvs/srv/SetBool "{data: true}"

# abort back to SAFETY — keep this ready before entering DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/direct_actuation/set_direct_mode \
  std_srvs/srv/SetBool "{data: false}"

# PX4 refuses an in-air disarm, so land by reference FIRST, then:
ros2 service call /uav_0/rc/disarm std_srvs/srv/Trigger {}
```

> The standalone Python reference-streamer and descent scripts used for the
> 2026-08-10 headless validation (plus the pre-arm verification checks and the
> `PEGASUS_HEADLESS` tmux-server flag) were trimmed from this section on
> 2026-08-11 for brevity — recover them from git history if a headless run is
> needed again. Their one trap is worth repeating here: **exactly one publisher
> on the reference topic**, or the vehicle chases both streams.

Since 2026-08-11 step 1 launches the **aerial-manipulator fork** of the
direct-actuation node (`autopilot_aerial_manipulator_direct_actuation_node`) —
the fork carries the arm torque feedforward. Node name, topics and services are
unchanged, so the commands above are unaffected. The controller runs
`config/params_single_drone_direct_actuation_t650_aerial_manipulator.yaml`, a
copy of the T650 tune whose only value changes are the four `vehicle_*` plant
numbers.

What to watch, beyond §7.5's list (which still applies):

- **Hover command ≈ 0.569**, not the bare T650's 0.503 — same motors, +23%
  mass. The Isaac spawn prints the exact TOTAL the yaml's `vehicle_mass` must
  equal (`T650 MASS OVERRIDE … TOTAL → … kg`); if they disagree, the printout
  is the truth.
- **A standing PITCH torque exists and is now cancelled by a TRUE feedforward**
  (2026-08-11; supersedes the earlier `ratectl_trim_y` integrator seed): the
  folded arm sits 19.5 mm forward on body +x while `alloc_rotor*_px/py` stay
  geometric, so the front rotor pair runs ~37 rad/s hot — hover ω ≈
  `[462, 424, 461, 424]`. Left uncompensated this cost a **97.7 cm X excursion
  taking 23.1 s to settle** at DIRECT engagement. The AM yaml now sets
  `system_ff_tau_y_per_coll: -0.060` / `system_ff_tau_y_const: -0.0058` — a
  thrust-proportional torque added every inner-loop tick (`ratectl_trim_y` back
  to 0.0). Verify in flight: settled `rate_control_debug[4]` ≈ **0.000** (with
  the old trim it sat at −0.0394; with nothing, the vehicle flew a metre
  sideways). Derivation, the flight validation, and the trim history:
  [Feedforward Compensation for Home-Pose Arm.md](<Feedforward Compensation for Home-Pose Arm.md>) §8.
- **Arm status** is printed by the Isaac pane every ~5 s (`q_err`, hold torque,
  realized rotor ω: 0 = disarmed, ~64 = armed idle, ~443 = hover). The arm
  should stay within ~2° of home throughout; a growing q_err or hold torque
  pinned at 3.0 N·m is a plant-side problem, not a controller tune.
- First run initializes the fresh PX4 profile `rootfs_fsc_indoor_am_t650`
  (deliberately separate — PX4 `param save`s into it).

### 7.8 AM-T650 BASELINE (SAFETY-only) stack — added 2026-08-11

The same AM plant flown by the **baseline** controller instead of the
direct-actuation one. This node publishes a `VehicleAttitudeSetpoint` and **PX4
runs attitude + rate + mixing**; there is no `set_direct_mode` service on this
path at all. It is the gentler of the two rigs and the one to reach for when you
want the AM airborne without DIRECT in the loop.

**Two differences from §7.7, both load-bearing:**

1. **Lockstep is ON.** The baseline path wants it (PX4 owns the inner loops);
   the DIRECT path must disable it. That is the entire reason there are two
   Pegasus launchers. `start_t650_aerial_manipulator_baseline_sitl.sh` asserts
   `PEGASUS_PX4_LOCKSTEP=1` and clears the tmux-server global, so a leftover `0`
   from a previous DIRECT run cannot leak in.
2. **Different config, different node name.** The baseline node is
   `/uav_0/autopilot_sv_baseline_node` (not `/uav_0/fsc_autopilot_ros2`), reading
   `params_single_vehicle_baseline_t650_aerial_manipulator.yaml`.

Same shape as §7.7.1, only the two launcher names change. **fsc_lab_machine:**

```bash
# terminal 1 — ROS 2 baseline stack (owns the agent; detach with Ctrl-b d)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_baseline_am_t650_stack_fused.sh fsc_lab_machine uav_0
#   ^ the OLD name — this is the one that RUNS on this machine today. Once the
#     rename is pushed from shiqi-desktop, switch to:
#     ./scripts/isaacsim/start_baseline_t650_aerial_manipulator_stack_fused.sh fsc_lab_machine uav_0

# terminal 2 — Pegasus / PX4 SITL, lockstep ON
cd ~/Source/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_baseline_sitl.sh fsc_lab_machine
```

**shiqi_machine:**

```bash
# terminal 1 — ROS 2 baseline stack (owns the agent; detach with Ctrl-b d)
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_baseline_t650_aerial_manipulator_stack_fused.sh shiqi_machine uav_0

# terminal 2 — Pegasus / PX4 SITL, lockstep ON
cd ~/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_baseline_sitl.sh shiqi_machine
```

Verify before arming — note the **baseline** node name:

```bash
ros2 param get /uav_0/autopilot_sv_baseline_node vehicle_mass       # 3.74617
ros2 param get /uav_0/autopilot_sv_baseline_node posctl_k_vel_x     # 3.7
ros2 topic hz /uav_0/mocap                                          # ~250 Hz
```

Then OFFBOARD → arm exactly as §7.7.1 step 3, and fly from the ground station.
There is **no `set_direct_mode` service** on this path. Shutdown is §7.7.1
step 0; `stop_isaacsim_stack.sh` picks up the
`fsc_baseline_t650_aerial_manipulator_stack` session automatically.

**The one number that proves the arm's feedforward is working:**

```bash
ros2 topic echo --once /uav_0/fsc_autopilot_ros2/position_controller/ude
```

`disturbance_estimate.z` should sit **near zero** in a settled hover (measured
**+0.05 N**). If it parks near **−8 N**, the bare-T650 yaml got loaded instead of
the AM one — that is the 0.71 kg arm being carried by the UDE instead of by
`vehicle_mass`.

**Measured 2026-08-11** (0.6 m steps in x/y, 0.4 m in z): flies the full sequence
— takeoff, hover, six steps, landing, disarm — with the UDE at 0.05 N throughout.
Step character is the T650's validated (underdamped) response by design, since
this path's `k_vel` 3.70 is a pure mass restore: x/y overshoot 42–50% and do not
settle to 2% within 20 s; z 10–11% overshoot, 9–14 s. The DIRECT rig is
noticeably tighter (§7.7) because it runs `k_vel` 7.0. Raising the baseline gain
on simulation evidence is explicitly warned against — see
[Feedforward Compensation for Home-Pose Arm.md](<Feedforward Compensation for Home-Pose Arm.md>) §7.

### 7.9 AM-T650 DIRECT with the ROS 2 position-mode ARM STACK (fsc_open_manipulator) — added 2026-08-13

The §7.7 rig with the arm commanded **over ROS 2 by the real
`fsc_open_manipulator` position-mode stack** instead of 04's in-process hold —
the first integration step toward driving the simulated arm through the same
software that drives the real OM-X (torque mode comes later). Incremental: 04
and its launcher are untouched and still work.

```
fsc_open_manipulator PositionController      ← the SAME compiled ros2_control
  (arm_position_controller, AERIAL config:     plugin the Gazebo and hardware
   home [0, 40°, 40°, 0], min-jerk moves)      bring-ups load
    ↕ position command/state interfaces
open_manipulator_x_isaac_bridge/IsaacTopicSystem     (new hardware plugin)
    → /uav_0/isaacsim_manipulator/position_commands (JointState)
    ← /uav_0/isaacsim_manipulator/joint_states
Isaac (05_px4_direct_t650_aerial_manipulator_ros2_arm_hold.py, 250 Hz):
    Dynamixel-servo EMULATION — 04's flight-validated PD + gravity comp +
    3 N·m clamp, tracking the streamed reference; latches it if the stack dies
```

The plant spawns at home, the PositionController homes on activation (a no-op
move) and holds — so this rig's flight behaviour is 04's, with the arm
reference arriving over DDS. Only the **inverted/aerial** configuration exists
(on the drone the arm always hangs inverted). The run adds a second tmux
window `arm` (switch with **`Ctrl-b n`**) holding the arm ros2_control stack
and the **ARM GROUND STATION** (`custom_gui joint_plot_inverted`) — together
with the drone ground station from step 1 the simulation runs **two ground
stations, one per subsystem, like the real rig**.

**Every arm topic sits under the namespace of the PACKAGE that owns it**
(2026-08-15, user request — superseding the 2026-08-13 plain `/uav_0`
namespacing), mirroring how the flight stack owns `/uav_0/fsc_autopilot_ros2/…`
so provenance is readable straight off `ros2 topic list`:

| prefix | owner | on real hardware |
|---|---|---|
| `/uav_0/fsc_open_manipulator/…` | the real arm stack (ros2_control) | **identical** |
| `/uav_0/isaacsim_manipulator/…` | the Isaac servo emulation | **absent** |

so `/uav_0/fsc_open_manipulator/{controller_manager, joint_states,
joint_desired_states, dynamic_joint_states, arm_position_controller/…}` and
`/uav_0/isaacsim_manipulator/{joint_states, position_commands}` (the Isaac
bridge pair — the emulated Dynamixel/U2D2 wire; **never subscribe to it from an
application**, it vanishes on the real arm).

The ground station's ROS names are namespace-relative (`custom_gui`,
2026-08-13) — bench launches without a namespace resolve to the root exactly as
before; the launcher starts the sim's station with
`-r __ns:=/uav_0/fsc_open_manipulator`. **The launcher's single `ARM_NS`
variable drives the stack namespace, the ground station and the readiness
gate**, so they cannot drift, and `position_controller_isaac_aerial.yaml` no
longer has to be edited in step — its node keys use the `/**/<node>:` wildcard
and apply at any namespace. That wildcard must be **flat**: `/**:` with the
node nested under it parses fine and matches nothing (params silently stop
applying, controllers fail on empty `joints`). Full convention:
`docs/docs_aerial_manipulator/Arm Topic Naming.md`.

**One-time build on shiqi-desktop** (already done 2026-08-13; repeat after
pulling the arm repo). Since the same-day repo consolidation there is ONE arm
repo — the fsc_open_manipulator GitHub repo, cloned at
`~/ros2_ws/src/fsc_open_manipulator` with its pinned Dynamixel deps beside
it (`cd ~/ros2_ws && vcs import src < src/fsc_open_manipulator/workspace.repos`;
`vcs` is not installed on this machine, so clone the three by hand at the
manifest's pins). It moved out of `~/colcon_ws` on 2026-08-23 — the arm now
shares ONE workspace with the flight stack, and `~/colcon_ws` is gone.
This machine has no system ros2_control/pinocchio and no sudo, so a root-less
deb extract provides them (`~/ros2_ws/rosdeps/local_setup.bash` — its header
explains how to replace it with a real `sudo apt install` later):

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash && source ~/ros2_ws/rosdeps/local_setup.bash
colcon build --packages-select \
  dynamixel_interfaces open_manipulator_x_description open_manipulator_x_bringup \
  open_manipulator_x_custom_controller open_manipulator_x_isaac_bridge utils_custom_ground_station \
  --symlink-install
```

**Run sequence — shiqi_machine (shiqi-desktop):**

```bash
# 0. clean slate            (any terminal)
cd ~/ros2_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2)
cd ~/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_geometric_direct_actuator_sitl.sh shiqi_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**Run sequence — fsc_lab_machine (the lab desktop, user `fsc-jupiter`):**

Same shape; only the two repo roots differ (`~/Workspaces/fsc_autopilot_ws` and
`~/Source/fsc_PegasusSimulator`, see §7.7.1) and the config name.

```bash
# 0. clean slate            (any terminal)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_direct_actuation_am_t650_stack.sh fsc_lab_machine uav_0
#   ^ the OLD name — this is the one that RUNS on this machine today. Once the
#     rename is pushed from shiqi-desktop, switch to:
#     ./scripts/isaacsim/start_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 2. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2)
cd ~/Source/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_geometric_direct_actuator_sitl.sh fsc_lab_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

> **The `t650_aerial_manipulator` RENAME OF THE AUTOPILOT-SIDE SCRIPT IS NOT ON
> `origin/dev_CCM`** (checked 2026-08-13: no remote branch of fsc_autopilot_ros2
> contains it, and fsc_lab_machine's checkout is 0/0 against `origin/dev_CCM`).
> It is unpushed on shiqi-desktop. **This does not block a run** — everything
> functional IS on `dev_CCM` under the old name:
> `start_direct_actuation_am_t650_stack.sh` takes the identical
> `<machine_config> [uav_prefix]` arguments, launches the same
> `single_drone_direct_actuation_launch.py`, and points at
> `config/params_single_drone_direct_actuation_am_t650.yaml`, which already
> carries the 2026-08-11 tune (`posctl_k_pos_x/y` 0.6, `posctl_k_vel_*` 7.0,
> `system_ff_tau_y_per_coll` −0.060 / `_const` −0.0058, `ratectl_trim_y` 0.0).
> The `system_ff_tau_*` feedforward is implemented on `dev_CCM` in
> `fsc_autopilot_ros2_node/single_drone_direct_actuation/client_lib/`.
>
> One documentation drift to be aware of: §7.7.1 above says step 1 launches a
> FORK, `autopilot_aerial_manipulator_direct_actuation_node`. **No such fork
> exists on `dev_CCM`** — there the feedforward lives in the shared
> direct-actuation node, gated off by 0.0 defaults. That refactor is unpushed
> too. Functionally the two are equivalent for this rig.

**One-time build on fsc_lab_machine** — the arm workspace is
`~/Source/Shiqi/fsc_om_ws`, **not** `~/colcon_ws`: this machine's `~/colcon_ws`
already holds the UPSTREAM `ROBOTIS-GIT/open_manipulator` checkout, which is a
different repo (no `open_manipulator_x_custom_controller`, no `_isaac_bridge`,
no `custom_gui`) and must not be conflated with the fork. Unlike shiqi-desktop
this machine has **ros2_control installed system-wide from apt**, so there is no
root-less deb overlay — `FSC_OM_ARM_ROSDEPS_SETUP` is `/dev/null`.

```bash
# one-time apt (needs the sudo password). pinocchio is a hard BUILD dep of
# open_manipulator_x_custom_controller; Qt6 Widgets+Charts of custom_gui.
sudo apt install ros-humble-pinocchio qt6-base-dev libqt6charts6-dev \
                 ros-humble-robot-state-publisher

mkdir -p ~/Source/Shiqi/fsc_om_ws/src
cd ~/Source/Shiqi/fsc_om_ws
# NOTE the Gao907@ in the URL — see the credential warning below
git clone https://Gao907@github.com/Gao907/fsc_open_manipulator.git src/fsc_open_manipulator
vcs import src < src/fsc_open_manipulator/workspace.repos

# MANDATORY on this machine — see the python warning below
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v fsc_isaac_env | paste -sd:)
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  dynamixel_interfaces open_manipulator_x_description open_manipulator_x_bringup \
  open_manipulator_x_custom_controller open_manipulator_x_isaac_bridge utils_custom_ground_station \
  --symlink-install
```

> **The build FAILS on this machine unless the `fsc_isaac_env` venv is off
> `PATH` first.** This login shell's `python3` resolves to
> `~/envs/fsc_isaac_env/bin/python3`, CMake hands that interpreter to
> `ament_cmake_core`'s `package_xml_2_cmake.py`, and it dies with
> `ModuleNotFoundError: No module named 'catkin_pkg'` on the very first
> package. The error names a python module, so it reads like a missing pip
> dependency; it is the wrong interpreter. Verify with `which python3` →
> `/usr/bin/python3` before building. The same applies to `custom_gui`'s
> Python-side imports (PyQt5/pyqtgraph/matplotlib exist under `/usr/bin/python3`
> only).
>
> **`gazebo_ros` / `gazebo_ros2_control` are NOT needed** even though
> `open_manipulator_x_bringup` and `_custom_controller` declare them in
> `package.xml`: neither is ever `find_package`d (the build lists are the
> `THIS_PACKAGE_INCLUDE_DEPENDS` blocks), and this rig runs Isaac, not Gazebo.
> Skipping them avoids pulling the whole Gazebo stack.
>
> `open_manipulator_x_isaac_bridge` must be built **after**
> `open_manipulator_x_custom_controller` — colcon looks for the latter's
> `package.sh` and fails outright if it is missing, so a `--packages-select`
> that omits the controller cannot build the bridge either.

> **`fsc_open_manipulator` is PRIVATE to the `Gao907` account, and this shared
> lab desktop's stored GitHub credential belongs to `LonghaoQian`.** A plain
> `git clone https://github.com/Gao907/...` silently authenticates as Longhao
> and fails with a misleading **"Repository not found"** (GitHub returns 404,
> not 403, for private repos an identity cannot see) — it is an access problem,
> not a typo. Putting `Gao907@` in the URL makes git-credential-manager key the
> lookup on host + username, so it prompts for the right identity instead of
> reusing Longhao's and leaves that credential untouched.
>
> The GUI dependencies (PyQt5/pyqtgraph/matplotlib) are already present under
> this machine's `/usr/bin/python3`; note an interactive shell's `python3`
> resolves to `~/envs/fsc_isaac_env`, which does **not** have them, so check
> with the absolute path if `custom_gui` fails to import.

Flight (takeoff by streamed reference, DIRECT in/out, land, disarm) is
identical to §7.7.1. The arm stack's pane waits for Isaac's
`/uav_0/isaacsim_manipulator/joint_states` before launching `controller_manager`, so the
ordering inside terminal 2 is automatic.

**Commanding the arm** — identical to the Gazebo/hardware bring-ups apart
from the `/uav_0` prefix, plus the ground station's Setpoints/Sine/Demos tabs:

```bash
ros2 topic pub --once /uav_0/fsc_open_manipulator/arm_position_controller/target_joint_positions \
    std_msgs/msg/Float64MultiArray '{data: [0.0, 0.698, 0.698, 0.0]}'
ros2 service call /uav_0/fsc_open_manipulator/arm_position_controller/go_home std_srvs/srv/Trigger
```

(For this integration step the mission is just: home commanded and held for
the whole flight. Targets are clamped to the validated working range; the
aerial config's max_velocity is a deliberate 0.2 rad/s.)

What to watch:

- The Isaac pane's status line gained `cmds: n=…, age …s` — n growing and age
  ~0 s means the position stack owns the reference; `none yet (holding spawn
  pose)` means the arm window hasn't come up (the rig is then exactly 04).
- The `arm` window: hardware activation logs `Activated on the measured pose
  [0.000, 0.698, 0.698, 0.000]`, then `uav_0.arm_position_controller` logs
  `Activated; homing over 1.50 s`.
- In the arm ground station the commanded-torque curves are flat zero — the
  position controller publishes no commanded effort (its caption says so);
  executed torque is live (the Isaac servo emulation's applied τ). The
  gripper button has no action server in this sim (Isaac pins the gripper);
  ignore it.
- **VALIDATED 2026-08-13** (headless, ground-seated plant, no PX4 — the drone
  path is §7.7's and was not re-flown): real Isaac plant + real stack held
  home exactly, tracked a commanded 4-joint target move and a `go_home`
  through the real topic/service interface, all with < 0.05° steady error;
  hold torque ≈ 0.7 N·m (gravity), commands flowing continuously. After the
  `/uav_0` namespacing (same day) the identical hold/move/go_home sequence
  was re-run loopback (fake plant) through the namespaced interfaces — all
  PASS — and the namespaced ground station verified on the wire: it
  publishes/subscribes `/uav_0/fsc_open_manipulator/arm_position_controller/target_joint_positions`,
  follows `/uav_0/fsc_open_manipulator/joint_states`, and adopts the controller's working range
  through the namespaced parameter service. First full flight with
  PX4/DIRECT in the loop was run by the user 2026-08-13.

### 7.10 AM-T650 GEOMETRIC direct actuation — added 2026-08-13

The §7.7 rig flown by a **different controller**: a new node in
`fsc_autopilot_ros2` (`single_aerial_manipulator_geometric_direct_actuation`,
installed as `autopilot_geometric_direct_actuation_node`) whose DIRECT mode replaces the
apm attitude stage + normalized-torque rate PID + `NormalizedMix` with **one
geometric SO(3) law computed in physical units**. The plant, the Pegasus
launcher and the arm hold are **exactly §7.7's** — only the ROS 2 side changes.

```
outer loop (UNCHANGED, shared by both modes, 100 Hz)
  robust position controller + UDE  →  attitude setpoint + thrust [N]
    │
    ├─ SAFETY : thrust → vehicle thrust model → normalized thrust_body
    │           + attitude setpoint → VehicleAttitudeSetpoint → PX4 runs
    │           attitude + rate + mixer          (geometric law DORMANT)
    │
    └─ DIRECT : attitude setpoint → GEOMETRIC SO(3) LAW, 250 Hz
                  τ = −kR∘e_R − kΩ∘ω + ω×Jω          [N·m]
                + CoM torque feedforward             [N·m]
                → FLU→FRD → WRENCH ALLOCATOR
                  [τ; T] → per-rotor thrust [N] → ω² → ω [rad/s]
                  → normalized throttle via the MN4010 bench curve
                → ActuatorMotors                     (PX4 runs NOTHING)
```

**The point of the redesign is where normalization happens.** There is no
vehicle thrust model on the DIRECT path: collective thrust stays in newtons all
the way down, and the only `→ normalized` step is the last one, after per-rotor
speeds exist. `vehicle_thrust_scaling` / `vehicle_idle_thrust` in the yaml
therefore serve **SAFETY only** (PX4 needs a dimensionless `thrust_body`).

**Three consequences that differ from §7.7 and will bite if forgotten:**

1. ~~**No integral state anywhere in the inner loop.**~~ **REVISED 2026-08-15
   (see §7.10.2): the node now runs the geometric PID of §7.11** (the Goodarzi
   integral term the bare-T650 fork validated in sim AND hardware on
   2026-08-14), and the CoM torque feedforward is recomputed **live from the
   arm encoders** each joint-state message (`armff_*`), with the flown static
   `system_ff_tau_y_per_newton: -0.0195` pair as the stale-arm fallback (= −dx
   at home, N·m per newton — exact zero-intercept form, −0.717 N·m at hover,
   the same moment §7.7 cancels). Division of labour: the feedforward carries
   the arm's computable moment, the integral only the residual — expect the
   settled integral ≈ 0, like §7.7's flown I_y. A persistent attitude offset
   in DIRECT hover now means the *arm model* is wrong (and the integral will
   mask it slowly — watch debug [24..26], not just the attitude).
   **RE-REVISED 2026-08-18: the ROLL/PITCH integral was DISABLED
   (`geoctl_ki_x/y: 0.0`; yaw keeps 0.35) — SUPERSEDED 2026-08-19, the
   shipped value is now `0.15`, see §7.10.4. The diagnosis below stands
   as the description of the OLD (`kR` 2.6, ζ 0.376) loop; with the
   attitude poles fixed the integral is no longer the trigger.** Flown A/B in sim on
   fsc_lab_machine (live armff active and verified, debug[30]=2): the hover
   diverges in a growing ~0.5–1.3 Hz roll+pitch whirl at the attitude loop's
   own natural frequency, and the time-to-flip scales hard with kI — in SIM
   time, ki 1.08 flips ~5 s after takeoff (4/4 flights), ki 0.3 at ~25 s,
   ki 0 at ~108 s (one run; a second ki-0 run was still clean when torn down
   at 107 sim-s). **The underlying cause is machine speed, measured**: this
   full rig (Isaac + 2 ground stations + both stacks on one box) runs at
   REAL-TIME FACTOR 0.36–0.39 on fsc_lab_machine — PX4-SITL is a lockstep
   build, so PX4/plant time is Isaac sim time, while the 250 Hz law, the DDS
   latency and the integral's dt all run on WALL clock — every effective
   loop delay and integration rate is ~2.6× off. A faster desktop (higher
   RTF) flies ki 1.08; do not tune this rig's marginal gains on one machine.
   ki_xy 0 is the robust setting (the live feedforward already carries the
   arm's moment; a residual bias now shows as a small standing e_R instead
   of winding an integrator), but at RTF≈0.38 a slow residual whirl remains
   (~108 sim-s to flip) — keep lab-machine DIRECT hovers short, raise RTF
   (close the GS plot windows, lighten rendering), or — the principled fix —
   derive the node's dt/rates from the PX4 (sim) clock instead of wall time,
   the same lesson `tools/stack_driver.py::sim_now` already encodes.
2. **`alloc_rotor*_km` is NOT inert here.** §7.7's allocator re-normalizes the
   yaw column so a common km factor cancels; this one does no column scaling, so
   km sets yaw torque in N·m directly. The shipped 0.052867 is the **sim-plant
   effective** value (bench × `YAW_TORQUE_FIT_FACTOR` 3.0); for hardware use the
   bench 0.018164 and kf 4.540431e-05.
3. **Gains live in `geoctl_*`, in physical units** (kR N·m/rad, kΩ N·m/(rad/s),
   full inertia tensor, torque clamps) — not `attctl_*`/`ratectl_*`. They are
   **derived, not tuned**: authority-matched to §7.7's flown cascade at hover
   (1.0 normalized roll/pitch ≙ 17.96 N·m, yaw ≙ 5.84 N·m — cross-checked
   against the flown hover rotor split). **RETUNED 2026-08-18 and again 2026-08-19 — see §7.10.3 and §7.10.4 (the shipped set).**

Higher-order references (jerk/snap → ω_ref, ω̇_ref) are **zero** for now, so the
J-weighted tracking-feedforward terms of the full Lee law vanish; the
gyroscopic ω×Jω term remains.

**STATUS: bench-checked only, NEVER FLOWN — not even in sim — as of
2026-08-13.** A synthetic closed-loop harness (faked PX4/estimator topics, real
service switch) drove the full chain and reproduced the AM-T650 hover solution:
mean motor command **0.567** vs 0.569 derived, front/rear split
**0.595 / 0.539** vs 0.597 / 0.540 physical. The feedback-loss failsafe was
observed tripping at +1.0 s and reverting DIRECT → SAFETY. Everything past that
is unflown. **2026-08-15: the node was then upgraded (integral term + live arm
feedforward, §7.10.2) before ever flying, so the first sim flight will fly the
upgraded form.** The FK core of the live feedforward is cross-validated against
this repo's `utils_controller/controller.py` to 1e-9 m over six arm poses
(home reproduces the flown −0.0195 to 0.02%), and a clean `colcon build`
passed; the closed loop is unflown.

#### 7.10.1 Run sequence — copy-paste, per machine

Identical to §7.7.1 except **step 1's launcher** and the **service namespace**
(`geometric_direct_actuation`, not `direct_actuation`). **Step 2 changed
2026-08-15: this rig now pairs with the §7.9 ROS2-ARM launcher** — the live
arm feedforward consumes `/uav_0/fsc_open_manipulator/joint_states`, which only the
fsc_open_manipulator ros2_control stack publishes — so the sim runs **two
ground stations** (drone GS from step 1's gui pane, arm GS from step 2's `arm`
tmux window), like the real rig. The plant itself is unchanged (§7.7's, arm
servo emulation and all). Pairing with the plain §7.7 hold launcher still
works but leaves the feedforward permanently on its static home-pose fallback
(warn throttled in the autopilot pane).

> **Renamed 2026-08-14, twice.** First `am_t650` → `t650_aerial_manipulator`,
> completing on the geometric stack the rename the other AM stacks got on
> 2026-08-11. Then `single_drone_` → `single_aerial_manipulator_` on the node
> itself, since this variant flies the manipulator, not a bare drone. Current
> names:
>
> | | name |
> |---|---|
> | launcher | `start_geometric_direct_actuation_t650_aerial_manipulator_stack.sh` |
> | params | `params_single_aerial_manipulator_geometric_direct_actuation_t650.yaml` |
> | launch file | `single_aerial_manipulator_geometric_direct_actuation_launch.py` |
> | tmux session | `fsc_geometric_direct_actuation_t650_aerial_manipulator_stack` |
>
> **Unchanged on purpose:** the executable `autopilot_geometric_direct_actuation_node`
> and the service namespace `geometric_direct_actuation`, so the `set_direct_mode`
> calls below still work verbatim. Any `am_t650` or `single_drone_geometric`
> spelling will now fail with "No such file or directory".

**fsc_lab_machine** (the lab desktop, user `fsc-jupiter`):

```bash
# 0. clean slate            (any terminal)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/Workspaces/fsc_autopilot_ws                                                 # workspace root, NOT the repo
colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF  # after any pull
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_geometric_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 2. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2 — the §7.9
#    ros2_arm launcher, REQUIRED since 2026-08-15 for the live arm feedforward)
cd ~/Source/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_geometric_direct_actuator_sitl.sh fsc_lab_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**shiqi_machine** (shiqi-desktop) — the node arrived here with the
`fsc_autopilot_ros2` `dev_CCM` pull of 2026-08-13:

```bash
# 0. clean slate            (any terminal)
cd ~/ros2_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/ros2_ws                                                                     # workspace root, NOT the repo
colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF  # after any pull
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_geometric_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 2. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2 — the §7.9
#    ros2_arm launcher, REQUIRED since 2026-08-15 for the live arm feedforward)
cd ~/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_aerial_manipulator_geometric_direct_actuator_sitl.sh shiqi_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**Only the paths differ between the two blocks** — on shiqi-desktop the
workspace is `~/ros2_ws` and Pegasus sits at `~/fsc_PegasusSimulator` rather
than `~/Source/…`. In particular `colcon build` runs from the **workspace
root on both** (corrected for fsc_lab_machine 2026-08-18): the stack script
derives `WS_ROOT` from its own path, refuses to start unless
`$WS_ROOT/install/setup.bash` exists, and sources that overlay itself — so no
manual `source` is needed, but a build launched from the *repo* directory
writes a nested `build/`+`install/` the script never reads, and the stack then
runs the PREVIOUS binaries with no complaint.

**Build after every pull, not once.** `dev_CCM` changed this node's sources on
2026-08-15 (§7.10.2), and a stale binary against the current params file is the
quiet failure mode: rclcpp ignores overrides the running node never declared,
so the rig flies the OLD law under the NEW gains and logs nothing about it.
One command settles it — zero means the binary predates the live feedforward:

```bash
strings $WS/install/fsc_autopilot_ros2/lib/fsc_autopilot_ros2/autopilot_geometric_direct_actuation_node \
  | grep -c armff        # 0 → pre-2026-08-15 build, rebuild before flying
```

**Why step 2 is the ROS 2 arm launcher (changed 2026-08-15).** It used to be
§7.7's `04_..._hold.py` (arm held in-process), with the §7.9 rig listed as an
optional, untried swap. ~~The two sides are independent (the arm stack talks
only to `/uav_0/arm/*`, the flight controller never sees it)~~ — **no longer
true, and no longer optional.** The flight controller now subscribes to
`/uav_0/fsc_open_manipulator/joint_states` for its live CoM feedforward, and only the
fsc_open_manipulator ros2_control stack publishes that topic; it also brings
the arm ground station, which is what makes this a two-ground-station rig.

Running the plain `04` hold launcher instead still flies — the controller logs
a throttled warn and falls back to the flown static home-pose coefficients, so
you get the pre-2026-08-15 behaviour — but the live feedforward never
activates (`geometric_control_debug[30]` stays 1), so it is a drone-only
check, not this rig. The same fallback is the safety net if the arm stack dies
mid-flight: degraded, not dangerous (the arm parks near home, and the integral
absorbs the bounded difference slowly).

**Prerequisite for step 2 — the ARM repo is a THIRD repo to pull and rebuild,
in lockstep with the other two.** Pulling `fsc_PegasusSimulator` +
`fsc_autopilot_ros2` alone leaves the arm stack on the pre-2026-08-15 topic
names, and the rig degrades with no error at the flight-controller level (hit
on fsc_lab_machine 2026-08-18: the local `fsc_open_manipulator` was 10 commits
behind — before the owner-prefix topic rename — so its bridge waited forever
on the old `/uav_0/arm/joint_states`, the ros2_control stack never activated,
the arm ground station had nothing to drive, and the live arm feedforward sat
on its static fallback, `geometric_control_debug[30]` stuck at 1). Pull and
rebuild from the **workspace root**, then re-run the whole sequence:

```bash
# fsc_lab_machine (workspace ~/Source/Shiqi/fsc_om_ws; no rosdeps overlay)
cd ~/Source/Shiqi/fsc_om_ws/src/fsc_open_manipulator && git pull --ff-only
cd ~/Source/Shiqi/fsc_om_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select dynamixel_interfaces open_manipulator_x_description \
  open_manipulator_x_bringup open_manipulator_x_custom_controller \
  open_manipulator_x_isaac_bridge utils_custom_ground_station --symlink-install

# shiqi_machine (workspace ~/ros2_ws since 2026-08-23, was ~/colcon_ws —
# same pull+build there after any arm-repo change, plus the ~/ros2_ws/rosdeps
# overlay sourced before building, see §7.9's "One-time build" block)
```

Detach a stack terminal with **`Ctrl-b d`** — never Ctrl-C/Ctrl-D. Step 0 is
also the shutdown (`stop_isaacsim_stack.sh` auto-discovers this stack's session
`fsc_geometric_direct_actuation_t650_aerial_manipulator_stack` by grepping its
sibling `start_*.sh` files, so it cannot drift).

**In flight** (machine-independent). Takeoff is driven from the ground station
exactly as in §7.7 — SAFETY holds the *ground* position until a reference
arrives, and **exactly one publisher** on the reference topic. Once settled:

```bash
# enter GEOMETRIC DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_direct_actuation/set_direct_mode \
  std_srvs/srv/SetBool "{data: true}"

# abort back to SAFETY — keep this ready before entering DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_direct_actuation/set_direct_mode \
  std_srvs/srv/SetBool "{data: false}"

# PX4 refuses an in-air disarm, so land by reference FIRST, then:
ros2 service call /uav_0/rc/disarm std_srvs/srv/Trigger {}
```

**The launcher now enters DIRECT for you (2026-08-15) — this rig only.** The
node still *boots* in SAFETY, which is the right power-on state for a fallback
controller, but the stack script's `vrc` pane calls `set_direct_mode` as soon
as the service appears, sequenced before `virtual_rc` starts. So the switch
always lands while DISARMED, and step 3's arm command starts the flight already
in the geometric law — there is no SAFETY takeoff to switch out of, and the
manual call above is now the recovery path, not the normal one. The tell is
printed directly above the arm prompt:

```
AUTO-DIRECT: geometric DIRECT entered while disarmed …   (green — good)
AUTO-DIRECT FAILED — the node is still in SAFETY          (red — use the call above)
```

Every other stack keeps the old behaviour: it boots in SAFETY and
`innerLoop()` returns on its first line unless the mode is DIRECT, so the
geometric law computes nothing until the service call above.
Confirm which law is live:

```bash
ros2 topic echo --once /uav_0/fsc_autopilot_ros2/controller_type
#   → "Baseline (Safety)"  |  "Geometric Direct Actuation"      (latched)
ros2 topic echo /uav_0/fsc_autopilot_ros2/geometric_direct_actuation/geometric_control_debug
#   → SILENT in SAFETY; publishes at 250 Hz in DIRECT
```

`geometric_control_debug` is a flat `Float32MultiArray`, **31 elements since
2026-08-15** ([24..30] appended so earlier indices stay stable — bags recorded
before that date have 24):

| index | contents | units |
|---|---|---|
| 0–2 | `e_R` attitude error | — |
| 3–5 | `e_ω` (= ω, reference zeroed) | rad/s |
| 6–8 | commanded torque, feedforward included | N·m |
| 9–11 | CoM torque feedforward (live or static) | N·m |
| 12–15 | per-rotor thrust | N |
| 16–19 | rotor speed | rad/s |
| 20–23 | normalized motor command | — |
| 24–26 | integral torque kI∘e_I (the RESIDUAL the FF misses) | N·m |
| 27–29 | live CoM offset r_com(q), FLU (zeros unless live) | m |
| 30 | feedforward source: 0 none, 1 static fallback, 2 live | — |

What to watch, beyond §7.5 and §7.7's lists (both still apply):

- **Motor commands ≈ `[0.597, 0.540, 0.597, 0.540]`** in a settled DIRECT hover
  (elements 20–23), and element 10 (τ_ff pitch) ≈ **−0.717 N·m**. Same hover
  point as §7.7 — the plant is identical, only the law computing it changed.
- **Element 30 must read 2** with the ros2_arm rig up — a 1 means the joint
  stream is not reaching the controller (arm stack down, wrong namespace), and
  the FF is frozen at home-pose values. Elements 27–29 should sit near
  `[0.0195, −0.0001, −0.0145]` with the arm at home and MOVE when you command
  the arm from the arm ground station — that motion, with the vehicle holding
  station, is the whole point of the live feedforward.
- **Integral torque [24..26] should settle near ZERO on all three axes.**
  All three integrate again since 2026-08-19 (`geoctl_ki_x/y: 0.15`, `ki_z
  0.13` — §7.10.4; they were pinned at 0 between 08-18 and 08-19). Near zero
  is the *expected* value, not a sign the term is inert: the live arm
  feedforward carries the arm's moment, so only residual model error is left
  — measured ≤0.007 N·m settled. A value parked near `geoctl_i_max_*`
  (1.62 xy / 0.53 z) means a broken arm model or a §7.7-class airframe fault.
- **Yaw first.** The gains are derived, unflown, and the sim yaw axis runs at
  ~2.45× its nominal loop gain from `YAW_TORQUE_FIT_FACTOR = 3.0` — and unlike
  §7.7, km feeds yaw torque directly here (consequence 2 above).
- **Allocator saturation** is logged (throttled, 1 Hz) with the unallocated
  wrench in N·m/N. On this airframe it should never fire in hover: peak
  authority is 11.38 N·m roll/pitch, 2.62 N·m yaw against clamps of 8.0 / 2.0.
- The SAFETY path here is the baseline *architecture* but **not** numerically
  §7.8's tune: this config runs the DIRECT-tuned position gains
  (`k_pos_x/y 0.6`, `k_vel 7.0`) in **both** modes, where the AM baseline stack
  runs 1.0 / 3.70. Don't compare step responses across the two rigs.

#### 7.10.2 The 2026-08-15 upgrade: geometric PID + live arm feedforward

Two changes landed together in `fsc_autopilot_ros2` (branch `dev_CCM`) after
§7.11's bare-T650 framework was validated in sim and hardware experiment:

1. **The Goodarzi integral term was ported from the §7.11 fork** — verbatim
   law, same arming-edge-only reset, preserved across SAFETY↔DIRECT, frozen
   per-axis on allocator saturation. Gains are the bit-faithful classic
   mapping at the AM's authority (17.96/5.84 N·m): `geoctl_ki` **1.08 xy /
   0.35 z**, `c2` **3.25/1.4**, `i_max` **1.62/0.53** — products kI·c2
   3.51/0.49, Ti 0.74 s/1.0 s, the classic kp/ki. §7.11's kI/c2 split warning
   applies unchanged: never retune kI alone.
2. **The CoM torque feedforward went live** (`armff_*` params + a new
   `arm_state_feedforward` module): the controller subscribes to
   `/uav_0/fsc_open_manipulator/joint_states` (names `joint1..4`), runs the 4-link FK **from this
   repo's `utils_controller/controller.py` link table** (make_params(), T650
   base bump; vectors in the model frame, fixed model→FLU rotation), and feeds
   τ_ff = r_com(q)×(T·e_z) each 250 Hz tick. q matters: τ_ff_y/T is −0.0195 at
   home but −0.0145 at `[0,0,0,0]` and −0.0215 at `[0,90,0,0]`. The C++ FK is
   cross-validated against the python to 1e-9 m over six poses.
   **Force level, settled by physics:** a quasi-static arm adds exactly its
   weight, which `vehicle_mass` already carries (§7.8 measured the UDE at
   +0.05 N on the total-mass yaml) — so the force channel deliberately adds
   nothing, and the node instead startup-checks base+links ≡ vehicle_mass
   (3.7462 vs 3.74617 ✓) and home-τ_ff ≡ the static pair (both logged in the
   autopilot pane; warns at 1% / 5%). Reaction wrenches from a *fast* arm need
   q̈ and belong to the whole-body controller.

Suggested first sim campaign, in order: (a) hover in SAFETY, check the two
startup anchors and element 30 = 2; (b) enter DIRECT at home pose — expect
§7.7's hover split and integral ≈ 0; (c) command an arm move from the arm GS
mid-hover and watch elements 27–29 track q while the vehicle holds station —
compare against the same move with `armff_enable: false` (static FF only),
which should show a transient attitude/position excursion the integral cleans
up slowly. That A/B is the deliverable measurement of this upgrade.

#### 7.10.3 Gain tuning against the harsh step (2026-08-18)

> **Read §7.10.4 first — it supersedes this section's shipped gains and
> retracts the A/Bs whose gains were never actually live.** Kept here as
> the record of how the attitude-damping finding was reached.

Tuned on fsc_lab_machine against the standard acceptance test — **from a
settled 1 m hover, step to x=1, y=1, yaw=90 in one command** (zero velocity and
zero acceleration in the reference, so every feedforward term is zero by
construction). Six full-stack flights, one gain set each; metrics measured in
SIM time from the PX4 ulog, not from the ground station's wall-clock plot.

| # | change | result on the step |
|---|---|---|
| 1 | as-found (ki_xy 0.25, ude 2.0, kR 2.6) | **flipped 2.5 s after the step** — already diverging in hover |
| 2 | ude_gain 2.0 → **0.74** | survived 35 s; rise 1.63 s, overshoot 3.8% |
| 3 | + ki_xy → **0** | no divergence in 36 s; ss err ±6 mm; one big excursion that recovered |
| 4 | + kR 2.6 → 1.6, kΩ 0.81 → 0.9 | no divergence in 61 s; ss err ±0.3 mm; still one excursion (peak tilt 44.8°) |
| 5 | ude_gain 0.74 → 0.30 | **diverged 14.7 s after the step** — too slow, so 0.74 is a real optimum |
| 6 | **kR → 1.2, kΩ → 0.75** (shipped) | **clean**: overshoot 3.4%, settling 6.9 s, peak tilt 3.5°, ss err ≤1.8 mm, yaw exact |
| 7 | repeat of 6, no change | no divergence, tilt 4.7° — but x/y **wandered ±0.3 m** at ~1.3 rad/s |
| 8 | + ki_xy 0 → 0.15 | worse: overshoot 99%, wander ±0.55 m. Reverted — ki_xy stays 0 |
| 9 | + k_pos_xy 0.6 → 0.4 | wander unchanged (±0.4 m, 1.27 rad/s). Reverted — k_pos stays 0.6 |

**The one finding that matters most: the yaml header's attitude sanity check
omits the rotor-lag pole**, and that is where the ~1 Hz whirl came from. The
header solves `wn = sqrt(kR/J) = 4.7 rad/s, zeta = 0.73`, but the plant (and the
real motors) carry the MN4010 spin-up lag `lambda = 10.0265 1/s`. Solving the
true third-order loop `J s^2 (s + lambda) + lambda (kOmega s + kR) = 0` gives
**wn 6.61 rad/s, zeta 0.376** at the shipped 2.6/0.81 — half the claimed
damping, at exactly the frequency every failed flight oscillated at. Softening
to 1.2/0.75 gives zeta 0.574 at the same bandwidth. This is the same lesson the
classic AM rig learned when it took `k_R` 8 → 4 (CLAUDE.md, 2026-08-10).

**Which of these transfer to hardware — this matters, the two halves differ:**

- **`geoctl_kr_*` / `geoctl_komega_*` DO transfer.** The rotor lag is a property
  of the real MN4010s, not of the simulator, so the damping deficit is real on
  hardware too. Carry 1.2 / 0.75.
- **`ude_gain` and `geoctl_ki_*` DO NOT transfer** — they were scaled by the
  measured RTF 0.38 purely to undo this rig's wall-clock-`dt`-vs-sim-time
  mismatch (§7.10.1's note), and on hardware RTF is 1. Restore
  `ude_gain: 2.0`, and **start `ki_xy` at 0.15–0.25** — zero is right for THIS
  rig only; hardware's real biases (wind, CoM error, motor asymmetry) need
  integral action. **The 2026-08-19 "`ki_xy` 0 / 0.15 / 0.41 are
  indistinguishable" measurement that stood here is RETRACTED** — all three
  runs actually flew `ki_xy` 0, so their identity was an artifact, not a
  result, and the "0–1.08 hardware-equivalent range is stable" conclusion
  drawn from it does not follow. See §7.10.4 for the cause (the harness
  edited an install-tree yaml the launcher never reads) and for the first
  verified-live `ki_xy` flights, which do support starting at 0.15.

Harness (session scratchpad, not committed): `runtest.sh` drives one full
launch → step → teardown cycle per gain set, `step_driver.py` is the sole
reference publisher, `step_metrics.py` reports rise/overshoot/settling/steady
state in sim time and truncates at loss of control so a late crash cannot
flatter the step numbers. Two gotchas it encodes: PX4 opens its ulog at BOOT
(so "newest log after teardown" is this run's — a before/after comparison never
sees a new file), and `kill_stale_sim_processes.sh` pattern-kills any shell
whose command line mentions a launcher, so it must never be chained with one.

**Still open — this is a large improvement, not a finished tune.** Across the
four runs carrying the attitude fix there were **zero divergences** (vs a flip
2.5 s after the step on the as-found config) and peak tilt stayed 3.5–8.5°, but
the *position* result is not yet repeatable: run 6 settled to ±1 mm, while runs
7–9 wandered ±0.3–0.6 m at ~1.3 rad/s on identical or near-identical gains.
That is run-to-run scatter of a still-marginal loop — the same trap
CLAUDE.md records for the classic AM rig ("a marginal system's run-to-run
scatter … which is why every candidate is now repeat-tested"), so treat any
single good flight here as unproven.

**ROOT CAUSE OF THE WANDER FOUND — IT IS THE ROS 2 ARM STACK, NOT A DRONE
GAIN.** The wander frequency would not move for ANY drone gain (`ude_gain`
0.74/1.1, `k_pos` 0.6/0.4, `ki_xy` 0/0.15 all left it at 1.26–1.36 rad/s),
which already ruled out every loop being tuned. Recording
`/uav_0/isaacsim_manipulator/joint_states` alongside the flight showed the ARM
JOINTS oscillating at 0.079–0.085 Hz on the wall clock = **1.31–1.41 rad/s in
sim time — the same mode**. The A/B is unambiguous: same drone gains, arm held
by §7.7's in-process PD instead of the fsc_open_manipulator ros2_control stack:

| | ros2_control arm stack | in-process arm PD |
|---|---|---|
| position noise (sigma) | 300–610 mm | **3–4 mm** |
| overshoot | 52–104 % | **30 / 28 %** |
| tail ringing | 449–936 mm | **189 mm** |
| peak tilt | 4.7–8.2 deg | **3.3 / 4.0 deg** |

A ~100x reduction, far larger than the 0.38 -> 0.42 RTF change that comes with
running fewer processes. The arm stack's own loop is wall-clock-driven at
250 Hz against a sim-time plant — the same class of bug as §7.10.1's, but on a
subsystem whose gains are NOT in this file. **Consequence for tuning: any
drone-gain result measured on the §7.9 ros2-arm rig is confounded** — the
`ki_xy` 0.15 and `k_pos` 0.4 rejections above were both measured through this
disturbance and should be re-judged on the §7.7 hold rig before being trusted.

Re-judged on the §7.7 hold rig (arm confound removed), BOTH rejections hold and
the ceiling is now measured: `ki_xy` 0 vs 0.15 is **identical** (ss err
+35/+23 vs +39/+21 mm, overshoot 30.4/27.6 vs 30.0/27.9 %), and `k_pos` 0.6 vs
0.4 moves overshoot only 30.4 -> 28.9 %. So on the clean rig the residual is
~30 % overshoot, a ~1.5 rad/s ringing of ~180 mm, and a **steady +35/+20 mm
offset that no gain touches**. That offset is almost certainly not a control
error at all: the controller closes on `state_estimator/local_position/odom`
while these metrics come from PX4's own EKF in the ulog, so the two estimates
need not agree. **Check `position_controller/state`'s `position_error` before
tuning anything further** — if it reads ~0 while the ulog reads 35 mm, the loop
is already on target and the gap is estimator disagreement.

The real fixes, in order of value: (1) make the arm stack's control loop use
the sim clock, or slow it until it stops exciting this mode; (2) do the same
for the flight node's `dt` (§7.10.1), which removes the RTF dependence
entirely and lets the integrators run at their designed, hardware-valid values.
Neither is a gain change, and no further gain search is worthwhile until at
least (1) is done.

#### 7.10.4 The install-yaml no-op, and the 2026-08-19 retune (SUPERSEDES parts of §7.10.3)

**A methodology bug invalidated part of §7.10.3.** The sim stack launcher
passes the **source-tree** params yaml to the node
(`start_geometric_direct_actuation_t650_aerial_manipulator_stack.sh`,
"Source-tree path" comment) — the §7.10.3 harness edited the
**install/share** copy, which nothing reads. Consequences, verified
2026-08-19 by re-running with `ros2 param get` confirmation of the live node:

- The "ki_xy 0 / 0.15 / 0.41 indistinguishable" sweep (§7.10.3's
  hardware-transfer bullet) is an artifact — all three runs flew the source
  yaml's `ki_xy 0`. **ki_xy had never actually flown nonzero** before
  2026-08-19. Any §7.10.3 row whose label disagrees with what the source
  yaml held at that moment is unverifiable; the attitude-damping finding
  (kR 2.6 → 1.2) is real (it matches the source-yaml evolution and
  reproduced), the fine A/Bs are not.
- The harness now edits the source yaml and prints the live node's
  parameters after every launch.

**Retune, validated on the §7.9 ros2-arm rig** (fsc_lab_machine, RTF 0.38,
x=1/y=1/yaw=90° step from a settled 1 m hover, live armff, all configs
verified live):

| gain | 08-18 value | shipped | why |
|---|---|---|---|
| `geoctl_kr_x/y` | 1.2 | **1.0** | with `komega` 0.55: pair ζ 0.574→0.661, wn 6.82→4.86 |
| `geoctl_komega_x/y` | 0.75 (source had drifted to an undocumented 1.25) | **0.55** | 0.75 left a residual whirl at exactly 6.8 rad/s that flipped the vehicle ~50 s after the step; 1.25 diverged 11.6 s after the step (slow real pole lands on the position loop — never raise kΩ for damping against a lag pole) |
| `geoctl_ki_x/y` | 0 | **0.15** | first live integral flights; settled integral ≤0.007 N·m |
| `ude_gain` | 0.74 | **0.45** | 0.74 (=2.0 rad/s effective) sits above the position loop; wound ±6 N and drove a growing ~1.1 rad/s position ring on this rig |
| `posctl_k_pos_x/y` | 0.6 | **0.45** | position loop wn 1.06→0.92 rad/s, ζ 0.88→1.02 |

Result, 3/3 clean flights (two 210 s-wall, one 476 s-wall = 180 sim-s):
overshoot **0.8–2.4 %** (was ~30 % clean-rig / 50–300 % arm-rig), settle to
±2 cm in **~8 s**, steady-state error **±2 mm** (the old +35 mm x offset is
gone), yaw exact in <1 s, peak tilt ≤3.1°, hover σ 3–5 mm, **no divergence
over 144 sim-s of post-step hold** — the §7.10.3 "wander" no longer appears
even with the ros2_control arm stack running, so it was the marginal drone
loop being excited, with the arm stack as the exciter, not an independent
fault. Sim-only vs hardware split: `ude_gain` 0.45 and roughly half of the
kr/komega softening compensate this rig's RTF-inflated delays — on hardware
restore `ude_gain 2.0` and start the attitude pair at §7.10.3's 1.2/0.75;
`ki_xy 0.15`, `k_pos 0.45` and the kI/c2 split carry as-is. Full per-gain
rationale lives in the params yaml's comments.

#### 7.10.5 r_os model-mismatch injection (2026-08-20)

`armff_mismatch_{x,y,z}` [m, FLU] were added to THIS node too, so it can be
A/B'd against the §7.12 geometric+L1 fork under identical CoM uncertainty. A
constant offset is added to every `r_com` the arm model emits, making the
controller's CoM model wrong by a known amount while the plant keeps the
truth. Here the resulting standing moment must be absorbed by the **Goodarzi
attitude integral** (`geoctl_ki_*` with `c2`, clamped at `geoctl_i_max_xy`
1.62 N·m) — where the L1 fork uses its matched-channel estimate instead.

**Shipped 0.0, so every existing config and flight on this node is
bit-identical to before.** Nonzero only for a deliberate robustness
experiment, and NEVER on hardware. The node prints a magenta startup warning
whenever it is nonzero. Comparison results: §7.12.4.

#### 7.12.4 CoM-mismatch sweep: L1 vs integral (2026-08-20)

Both nodes flown against an IDENTICAL closed-loop plant (rigid body + MN4010
rotor lag + the exact allocator effectiveness, wrapping the REAL C++ binaries
over ROS 2; harness `sweep.py`, scratchpad). The plant holds the true CoM; the
node boots with `armff_mismatch_x`. Protocol: arm → 8 s settled hover → engage
DIRECT → 34 s hold; transient = first 8 s, steady = last 12 s. 16 runs + 6
boundary repeats, all `clean_teardown`.

| dx err | moment | **L1** | **integral (§7.10)** |
|---|---|---|---|
| 0 mm | 0 | stable, ss 0.0 mm | stable, ss 1.8 mm |
| 5 mm | 0.184 N·m | stable, **ss 191 mm**, peak 0.66 m, u_L1 +0.163 | stable, **ss 2 mm**, peak 1.49 m, τ_i −0.184 |
| 7.5 mm | 0.276 N·m | — | stable, **ss 2 mm**, peak 2.89 m, τ_i −0.276 |
| 10 mm | 0.367 N·m | stable, ss 382 mm, peak 1.36 m, u_L1 +0.326 | **DIVERGED t=3.3 s** |
| 20 mm | 0.735 N·m | stable, ss 766 mm, peak 3.43 m, u_L1 +0.652 | DIVERGED t=1.8 s |
| 25 mm | 0.919 N·m | **DIVERGED t=2.5 s** | — |
| ≥30 mm | ≥1.10 N·m | DIVERGED | DIVERGED |

**Boundaries: L1 20–25 mm, integral 7.5–10 mm — L1 tolerates ~2.7× more.**

**The two mechanisms fail differently, and that is the whole story:**

- **L1 cancels only 88.7% of a constant moment, at every level** (0.163/0.184,
  0.326/0.367, 0.652/0.735). That is exactly `e^{A_s·T}` = e^(−30×0.004) =
  0.887 — the PWC one-sample attenuation of §7.12.2, now confirmed a third
  time, in the live binary. The uncancelled 11.3% becomes a standing attitude
  offset and hence a position offset of **38 mm per mm of CoM error**, dead
  linear. So L1 stays *stable* to 20 mm but is *accurate* nowhere near it.
- **The integral cancels 100%** (τ_i = −0.184 at 5 mm, −0.276 at 7.5 mm — the
  needed value to 3 digits), so its steady error stays 2 mm no matter the
  mismatch. But it is slow (Ti ≈ 0.74 s), so the transient grows until the
  marginal attitude loop lets go: peak excursion 1.49 m at 5 mm, 2.89 m at
  7.5 mm, divergence at 10 mm.

**HARDWARE CORROBORATION — the sim reproduces the real failure.** The
2026-08-19 first hardware flight of the §7.10 geometric node diverged in 1.3 s
with a measured **+0.33 N·m pitch feedforward overcompensation**
([[am-geometric-first-hardware-flight]]). 0.33 N·m ÷ 36.74 N = **8.98 mm of
equivalent CoM error — which falls exactly between this sweep's last stable
point (7.5 mm) and first divergence (10 mm)** for that same controller. The
memory note for that flight says the cause was "invisible in sim by
construction (sim plant IS the model)"; the mismatch injection is what closes
that blind spot, and the first thing it did was predict the crash.

**Caveats.** Python plant, so no PX4, no DDS jitter, no RTF-0.38 effect —
absolute boundaries will differ in Isaac; the *ratio* and the mechanisms are
the transferable result. `armff_base_com_*` was zeroed on the L1 node for the
sweep so both compute r_com identically. Not yet repeated in Isaac.

### 7.11 Bare-T650 GEOMETRIC direct actuation — added 2026-08-14

§7.10's geometric controller on §7's **bare T650 plant** (no arm —
`x650_new.usd` with the T650 motor calibration, 3.034 kg total). The ROS 2 side
runs a **separate parallel fork** of the geometric node (split out 2026-08-14):
its own sub-package `single_drone_geometric_direct_actuation`, its own
executable, sharing no source file with the AM node — but the **runtime
interface is identical on purpose** (same node name `fsc_autopilot_ros2`, same
topics, same `geometric_direct_actuation/set_direct_mode` service), so every
command in this section is §7.10's verbatim. The Pegasus side gets its **own
named launcher** (below, added 2026-08-14), a functionally identical twin of
§7.3's classic one — same plant, same lockstep-off setup — so every stack
pairs 1:1 with its own launcher.

| | name |
|---|---|
| launcher | `start_geometric_direct_actuation_t650_stack.sh` |
| executable | `autopilot_drone_geometric_direct_actuation_node` (fork, NOT §7.10's) |
| params | `params_single_drone_geometric_direct_actuation_t650.yaml` |
| tmux session | `fsc_geometric_direct_actuation_t650_stack` |
| Pegasus launcher | `start_t650_geometric_direct_actuator_sitl.sh` (added 2026-08-14) |

The Pegasus launcher is a **functionally identical twin of §7.3's**
`start_t650_direct_actuator_sitl.sh` — same plant, same lockstep-off setup,
its own name so every stack pairs 1:1 with its own launcher (its param-delay
override is `T650_GEOMETRIC_DIRECT_ACTUATOR_PARAM_DELAY`). Keep the two
scripts functionally in lockstep; a plant change in one belongs in both.

The params file is the §7.10 config re-derived about the bare hover point:
`vehicle_mass` 3.033921, the bare-body inertia tensor, `geoctl_*` gains
authority-matched at the bare hover (`kR` 2.4 / 0.44, `kΩ` 0.73 / 0.32 N·m
units), and the classic bare-T650 outer loop (`k_pos` 1.0, `k_vel` 3.0 —
**not** the AM's 0.6/7.0). The `system_ff_tau_*` keys **do not exist on this
node** (not "present but zero" — setting them is silently ignored).

**§7.10's consequence 1 does NOT apply here: this fork's law is geometric PID,
not PD.** An integral term (`geoctl_ki_*`, Goodarzi & Lee's form) was added
2026-08-14 after the first sim hover measured what pure PD does with a standing
torque bias — see the status paragraph below. Consequences 2 (km not inert)
and 3 (`geoctl_*` physical units) apply unchanged.

**STATUS — three sim lessons on 2026-08-14, one flight each.** (1) First
hover (pure PD law): stable, but with a steady 3–6 cm x/y offset in DIRECT —
the plant's true CoM sits ~0.5 mm off the rotor centroid (mesh-derived mass
properties of `x650_new.usd`), a ~0.015 N·m standing torque that PD can only
absorb as a 0.36° standing attitude error, bought by the P-P position loop as
position error (`e_pos = m·g·τ/(kR·k_pos·k_vel)`). The same bias exists under
PX4 and the classic node — their rate integrators hide it. (2) An integral
term was added (Goodarzi & Lee geometric PID) — and the first integrator
build **oscillated in DIRECT**: the integral of e_ω *is* the attitude, so a
mis-split that put the whole gain into kI (3.15, c2=1.0) acted as phantom
proportional stiffness, cutting the roll/pitch phase margin against the
99.7 ms rotor lag from 29° to 6°. (3) Corrected same day to the bit-faithful
classic mapping — `ki_xy 0.97 / ki_z 0.32` with `c2_xy 3.25 / c2_z 1.4`
(products unchanged, phase margin 20°, the flown classic loop shape). The
corrected split is **unflown** as of writing. Hardware unflown entirely.

> **UPDATE 2026-08-15: superseded — the corrected split has now flown, and so
> has hardware.** The bare-T650 geometric rig was validated end-to-end in both
> the sim stack and a real indoor experiment (2026-08-14, user-confirmed
> 2026-08-15), with no config or code changes coming back from the campaign —
> the committed configuration is the flown one. This validation is what
> qualified porting the integral term to the AM geometric node (§7.10.2).

**Run sequence — shiqi_machine (shiqi-desktop):**

```bash
# 0. clean slate            (any terminal)
cd ~/ros2_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/ros2_ws                                                                     # workspace root, NOT the repo
colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF  # after any pull
cd ~/ros2_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_geometric_direct_actuation_t650_stack.sh shiqi_machine uav_0

# 2. Pegasus / PX4 SITL     (terminal 2 — the geometric stack's own launcher, twin of §7.3's)
cd ~/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_geometric_direct_actuator_sitl.sh shiqi_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**Run sequence — fsc_lab_machine (the lab desktop, user `fsc-jupiter`):**

Same four steps, lab paths. Both new files arrived here on 2026-08-14 — the
Pegasus launcher with `dev_robotic_arm` `b07f426`, the ROS 2 fork with
`fsc_autopilot_ros2` `dev_CCM` `d8d64e1`.

```bash
# 0. clean slate            (any terminal)
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2 && ./scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
cd ~/Workspaces/fsc_autopilot_ws                                                 # workspace root, NOT the repo
colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF  # after any pull
cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2
./scripts/isaacsim/start_geometric_direct_actuation_t650_stack.sh fsc_lab_machine uav_0

# 2. Pegasus / PX4 SITL     (terminal 2 — the geometric stack's own launcher, twin of §7.3's)
cd ~/Source/fsc_PegasusSimulator
./scripts/indoor_sim/start_t650_geometric_direct_actuator_sitl.sh fsc_lab_machine

# 3. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

Only the paths differ from the shiqi_machine block: the autopilot workspace is
`~/Workspaces/fsc_autopilot_ws` (repo under `src/fsc_autopilot_ros2`) and
Pegasus is at `~/Source/fsc_PegasusSimulator`. **`colcon build` must run from
the workspace root on this machine too** — the stack script derives
`WS_ROOT` as its own path's great-grandparent (`~/Workspaces/fsc_autopilot_ws`)
and sources `$WS_ROOT/install/setup.bash`, so a build launched from the repo
directory writes a nested `build/`+`install/` the script never reads and the
stack silently runs the previous binaries. (§7.10's fsc_lab_machine block still
shows the build in the repo directory — treat that as stale; nothing has ever
built there, there is no in-repo `build/`.)

Detach with **`Ctrl-b d`** — never Ctrl-C/Ctrl-D. Step 0 is also the shutdown
(`stop_isaacsim_stack.sh` auto-discovers the session by grepping its sibling
`start_*.sh` files).

**In flight**: identical to §7.10 — takeoff by streamed reference from the
ground station (exactly one publisher), then the same
`geometric_direct_actuation/set_direct_mode` service calls to enter/abort
DIRECT, land by reference, disarm on the ground. The `controller_type` /
`geometric_control_debug` verification topics are §7.10's, but this fork's
debug array has its **own 24-element layout** (no feedforward slot; integral
torque appended): e_R [0..2], e_ω [3..5], torque N·m [6..8], rotor thrust
[9..12], rotor speed [13..16], motor command [17..20], **integral torque
kI∘e_I N·m [21..23]**.

What to watch, beyond §7.5's list (which still applies):

- **Motor commands ≈ `[0.5025, 0.5025, 0.5025, 0.5025]`** in a settled DIRECT
  hover (debug elements **17–20**) — the symmetric plant's signature. A
  front/rear split here would mean the AM stack is running (its hover is
  0.597/0.540).
- **Integral torque [21..23] should settle near `[±0.015, ∓0.008, ·]` N·m** —
  the measured CoM-offset bias — and the commanded-vs-actual attitude gap and
  the 3–6 cm x/y offset should be gone with it. An integral parked at
  `geoctl_i_max_*` (1.45 xy / 0.47 z) means a real airframe-class fault, the
  X650 motor-cant kind — land and measure, don't retune.
- **Yaw first**, same as every T650 rig: the sim yaw axis carries
  `YAW_TORQUE_FIT_FACTOR` = 3.0, and on this allocator km feeds yaw torque in
  N·m directly.
- The integral resets on the **arming edge only** and survives SAFETY↔DIRECT
  switches, so the first DIRECT entry of a flight re-learns the bias in ~1–2 s
  (Ti = 0.76 s xy) and every later entry starts pre-charged.

#### 7.11.1 Reference for the integral term

The integral term added on 2026-08-14 is **not** an ad-hoc PID bolt-on; it is
the attitude integral of:

> F. Goodarzi, D. Lee, and T. Lee, **"Geometric Nonlinear PID Control of a
> Quadrotor UAV on SE(3),"** *Proceedings of the European Control Conference
> (ECC)*, Zürich, Switzerland, July 17–19 2013, IEEE, New York, pp. 3845–3850.
> Preprint: [arXiv:1304.6765](https://arxiv.org/abs/1304.6765).

It extends the PD law this node already ran — Lee, Leok & McClamroch,
*"Geometric tracking control of a quadrotor UAV on SE(3)"*, CDC 2010 — whose
attitude channel is what §7.10 and this section call "the geometric law".

**What we took, verbatim.** The paper's attitude moment, its equations (13)–(14):

```
M   = −kR·eR − kΩ·eΩ − kI·eI + (RᵀR_d Ω_d)^ J RᵀR_d Ω_d + J RᵀR_d Ω̇_d      (13)
eI  = ∫₀ᵗ [ eΩ(τ) + c₂·eR(τ) ] dτ                                            (14)
```

Our implementation is (13)–(14) with `Ω_d ≡ Ω̇_d ≡ 0` (no jerk/snap references,
so both feedforward terms vanish and only `ω×Jω` survives — §7.10's note), plus
two engineering additions the paper does not specify: a per-axis clamp on the
integral and a freeze while the allocator reports that axis saturated.

| paper | our param | shipped (bare T650) |
|---|---|---|
| `kR` | `geoctl_kr_*` | 2.4 / 2.4 / 0.44 N·m/rad |
| `kΩ` | `geoctl_komega_*` | 0.73 / 0.73 / 0.32 N·m/(rad/s) |
| `kI` | `geoctl_ki_*` | 0.97 / 0.97 / 0.32 N·m/rad |
| `c₂` | `geoctl_c2_xy`, `geoctl_c2_z` | 3.25 / 1.4 s⁻¹ (paper uses one scalar) |
| — | `geoctl_i_max_*` | 1.45 / 0.47 N·m (not in the paper) |

**The paper predicts the oscillation we hit.** Integrating `eΩ` as well as `eR`
is the paper's deliberate choice (it is what buys exponential stability under
disturbance), and §III-B spells out the side effect:

> "Unlike common integral control terms where the attitude error is integrated
> only, here the angular velocity error is also integrated at (14). […] From
> (12), it essentially increases the proportional term. **The corresponding
> effective controller gains for the proportional term and the integral term
> are given by kR + kI and c₂kI, respectively.**"

That is exactly the trap the first integrator build fell into: putting the whole
desired `c₂kI` into `kI` (3.15, c₂ = 1) raised the *effective* proportional gain
to `kR + kI` = 5.55 N·m/rad and cut phase margin against the 99.7 ms rotor lag
to 6°. The shipped split keeps `kI` small and `c₂` large — same `c₂kI` = 3.15,
effective proportional 3.37, margin 20°.

**One deliberate deviation, recorded.** The paper's Proposition 2 requires

```
c₂ < min{ √(kR·λm/λM),  4kΩ / (8kR·λM + (kΩ+B₂)²) }
```

which at our tensor (λm 0.0595, λM 0.0815) and `Ω_d = 0` (so `B₂ = 0`) evaluates
to **c₂ < 1.324 s⁻¹**. Our `c2_xy` = 3.25 exceeds that by 2.45×, so
**Proposition 2's almost-global exponential-stability guarantee does not cover
this tune.** We ship it anyway, deliberately: the paper's model is a rigid body
with instantaneous torque and **no actuator dynamics**, while this plant's
binding constraint is the rotor lag. Satisfying the bound at fixed integral
authority forces `kI` up (`c₂` 1.324 → `kI` 2.38 → effective P 4.78) and drops
the measured phase margin to **10.4°** — i.e. obeying the sufficient condition
would make the real loop *worse*. Values were instead authority-matched to the
flown classic cascade and confirmed in sim. Revisit this if jerk/snap references
are ever added (`Ω_d ≠ 0` makes `B₂ > 0` and tightens the bound further).

### 7.12 AM-T650 GEOMETRIC + L1-ADAPTIVE direct actuation — added 2026-08-20

The §7.10 rig flown by a **third controller**: the method of *Cai, Yu, Zhang,
Liang, Fang, Han, "An experiment study for unmanned aerial manipulator systems
with L1 adaptive augmentation of geometric control", Control Engineering
Practice 164 (2025) 106418* (PDF in `docs/docs_aerial_manipulator/`),
implemented as a new PARALLEL FORK in `fsc_autopilot_ros2`
(`single_aerial_manipulator_geometric_l1_direct_actuation`, installed as
`autopilot_geometric_l1_direct_actuation_node`). The plant, the Pegasus/PX4
side and the §7.9 arm stack remain §7.10's architecture. The post-review mass
and timing corrections below also changed the shared T650 plant constant and
added fail-fast checks to its launch path.

```
SAFETY : unchanged baseline (robust position ctrl + UDE → attitude setpoint →
         PX4 runs attitude+rate+mixer). Takeoff, abort and failsafe path.
DIRECT : u = u_b + u_L1, 250 Hz, straight from the POSITION reference:
  u_b  (paper eqs 17/18, ENU/FLU): F_d = -Kp e_p - Kv e_v + m g e3 + m a_d
         + m R(ω×(ω×r_os));  f_b = F_d·Re3;  R_d from F_d + ref yaw;
         M_b = -KR e_R - Kω ω + ω×Iω + r_os×(f_b e3 - m ω×(ω×r_os))
  u_L1 (paper eqs 19-23): predictor on ζ=[v;ω] → piecewise-constant
         adaptation (Ḡ=[G,G⊥] inverted per state sample, T = 4 ms in this
         simulation and capture-stamp delta on hardware) → LPF;
         only the MATCHED estimate [f; M] enters the control channel.
  → same physical wrench allocator as §7.10 → ActuatorMotors.
```

Key differences from §7.10 worth having in hand:

- **No attitude integral, no UDE in DIRECT** — the L1 augmentation is the
  paper's replacement for both. The residual-bias display moved from
  "integral torque" to `u_L1` (`l1_control_debug` [16..19]).
- **The arm's wrench is compensated INSIDE the law** through `r_os` (live
  from `/uav_0/fsc_open_manipulator/joint_states`, home-pose fallback). At
  home the thrust-moment term reproduces the flown −0.0195 N·m/N to the
  reported precision.
- **The DIRECT position loop is the paper's** (`l1geo_kp/kv` in newtons), not
  the robust controller; `posctl_*` serve SAFETY only.
- **Scope is hover/constant yaw.** The reference message has acceleration but
  no jerk, snap, yaw rate, or yaw acceleration, so the implementation sets
  `omega_d = omega_d_dot = 0`. It is not the paper's complete
  time-varying-trajectory attitude feedforward.
- The predictor is advanced with the ACHIEVED wrench (allocator saturation
  subtracted) — the L1 equivalent of conditional integration — and `u_L1` is
  clamped (±10 N, ±1.5/0.8 N·m); the estimate gets 2× that range. The 10 N
  thrust bound is the 2026-08-21 +20% allocation-mismatch stress-test value.
- `l1adapt_enable: false` in the yaml = pure baseline geometric law (the
  paper's own A/B).

**STATUS: full SITL flight and gain sweep completed 2026-08-21.** The earlier
synthetic closed-loop harness (faked PX4/estimator/arm topics, real service
switch — §7.10's pre-flight standard) was rerun after the mass, sample-clock,
and CoM-origin corrections. The live C++ node agreed with an independent
Python reference at hover and three non-trivial states (including ω = 1.1
rad/s and an off-home arm) to **1.34e-6 N**, **3.14e-8 N·m**, and **1.79e-9 m**
in arm FK. Its measured sample period was **4.000 ms**. The later live SITL
stress test and selected gains are recorded in §7.12.4.

#### 7.12.2 Verification pass (2026-08-20) — what was proven, and three fixes

Scripts live in the session scratchpad (not committed): `verify_derivation.py`,
`verify_node.py`, `verify_l1.py`, `soak.py`.

**Proven, all to machine precision:**

| check | result |
|---|---|
| Paper's constant-position/constant-yaw specialization in NED/FRD vs this implementation in ENU/FLU, 400 random states with ω up to 1.5 rad/s | f_b agree to **7e-15 N**, M_b to **4e-16 N·m**, R_d to **6e-16** |
| Closed loop reproduces the paper's eq (27), `I ė_ω = −K_R e_R − K_ω e_ω` | residual **4e-16 N·m** — the r_os feedforward cancels d_im1 and ω×Iω *exactly* |
| Translational loop reproduces eq (30), Δ → 0 at perfect attitude | residual **1e-14 N** |
| LIVE C++ node vs independent Python, rerun after the post-review corrections over 4 states incl. **ω = 1.1 rad/s** and an off-home arm | f_b **1.34e-6 N**, M_b **3.14e-8 N·m**, arm FK **1.79e-9 m** |
| eq (23) LPF recursion measured in the running binary | α = 0.992032 vs 0.992032 expected, implied dt **4.000 ms** |
| PWC adaptation recovers a known injected wrench (closed-loop sim) | matches theory to **2e-5** |
| eq (16) `F(ζ)` and `Ḡ = [G, G⊥]` under the same frame map | **0.000e+00** |

**Correction to the earlier reading of the paper's `G(R)`: there is no sign
typo.** The apparent minus is the exponent in `I⁻¹`, not a minus on
`ᴮr_os × e₃`. The paper's NED thrust→moment block is
`+I⁻¹(ᴮr_os × e₃)`. Its ENU/FLU form in this implementation is
`−I⁻¹(r_os × e₃)` because positive collective acts along body `+e₃` instead
of NED/FRD `−e₃`. Equations (4), (7), (12), the displayed matrix, and the
implemented frame conversion therefore agree.

The ω ≠ 0 cases matter: **the centripetal feedforward terms are identically
zero at hover**, so the original bench run never tested them at all.

**Three defects found and fixed:**

1. **The UDE was integrating a fictitious input in DIRECT.** `VelocityBasedUDE`
   integrates `−R e_z · input_.thrust + m g`, and `outerLoop()` fills `input_`
   with the *robust controller's* command — which this node never applies
   (the paper's law computes its own thrust). The UDE was therefore booking
   the difference between the two controllers as a disturbance and handing it
   to SAFETY on the abort edge. Now fed the achieved collective.
2. **Predictor integration was forward Euler.** Fine at the shipped poles, but
   the dt gate admits 50 ms and the paper's own hardware poles are A_s = −65/−80,
   giving |a|·dt = 4.0 — **measured to diverge to inf**. Replaced with the exact
   zero-order-hold update (unconditionally stable, reduces to Euler as dt → 0),
   the same fix `lagged_thrust_curve.py` records for the rotor-lag pole. This
   is what makes restoring the paper's A_s safe on hardware.
3. **The law and the L1 predictor used DIFFERENT gravity constants** (9.80665
   vs 9.81) — caught by a 30 s hover soak, where u_L1 settled at **−0.119 N**
   instead of zero. The two form one loop, so a constant model disagreement is
   indistinguishable from a real matched disturbance and is amplified by
   `e^{A_s T}/(1 − e^{A_s T}) ≈ 1/(|A_s|·T)` — **10× here**, turning a 0.0126 N
   modelling slip into 0.119 N of thrust and ~1.5 cm of phantom altitude bias.
   Predicted ζ̃ = −1.340e-4 vs measured −1.34e-4; after the fix the soak sits
   at exactly zero. **General rule: every constant shared by the law and the
   predictor must come from one place.**

**One characteristic, not a defect, worth knowing before reading logs:** the
PWC estimate steady-states at `Ḡ⁻¹ e^{A_s T} Ḡ σ`, i.e. it **under-reads** a
constant disturbance by one sample of the predictor dynamics — 10% at the
shipped sim poles, but `e^{-0.65}` ≈ **0.52** at the paper's hardware poles
with T = 10 ms. So `u_L1` cancels only about half of a constant disturbance in
one pass and the baseline loop carries the remainder; do not read a
`γ̂` smaller than the disturbance you injected as a bug.

#### 7.12.3 r_os model-mismatch injection (2026-08-20, user request)

In sim the controller's r_os is otherwise EXACT (the `armff_*` table is the
plant's own USDA extraction — verified to 0.0 µm at q=0), so a clean hover
proves plumbing but never exercises what the L1 augmentation is *for*. On
hardware the mismatch is real and large: the model's home-pose
**dx = +19.5 mm** against a true value of **~5–11 mm** measured two ways from
the 2026-08-19 flights — ~11 mm from the geometric rig's over-feedforward
(+0.33 N·m ÷ ~36 N), ~5 mm back-solved from the baseline ulog's hover rotor
split (`0819 - T650-AM baseline/log_244`, mean motors [0.558 0.549 0.690
0.668]; that estimate is contaminated by per-motor thrust-constant spread —
the same asymmetry behind the km÷2.9 yaw finding, so treat it as a lower
bound). The model **over-reads dx by roughly 2×**.

New `armff_mismatch_{x,y,z}` [m, FLU] on the L1 node's arm model: a constant
offset added to EVERY r_os the model emits — live FK, home-pose fallback,
baseline law, L1 predictor, startup anchors — so the controller is
**structurally blind** to it; only the USDA plant knows the truth. Exactly the
hardware situation. The sim yaml ships **+0.0085 x** (the 19.5-vs-11 datum,
same sign relationship as hardware: controller over-reads). The node WARNS
loudly (magenta) at startup when nonzero; debug **[41..43]** publish the
injected value so `[36..38] − [41..43]` recovers the true r_os. **MUST be 0
in any hardware config** (hardware carries its own mismatch; more would
compound, not emulate) and 0 for a clean-model sim baseline.

Hover-test expectations WITH the injection: baseline pitch moment
−1.03 N·m (over-compensating by 0.31), `γ̂_My` (debug [22]) ≈ −0.28 N·m,
**u_L1 pitch (debug [18]) settling near +0.31 N·m — not near zero**, position
hold unchanged. That last point IS the robustness claim under test; the A/B is
the same injection with `l1adapt_enable: false`, where the integrator-less
baseline turns 0.31 N·m into a standing offset instead. Note the §7.12.2
synthetic harness CANNOT see the injection (its faked plant has no physics and
model/predictor stay self-consistent — verified: u_L1 stays 0 on the bench
with the injection active); only a real Isaac flight exposes it. Plumbing
verified 2026-08-20 against the live node: M_b pitch −1.0304 N·m, r_os belief
0.02805, `[36]−[41]` = +0.01955 = the plant truth.

#### 7.12.3 Post-review corrections (2026-08-20)

- Restored `t650_params.BODY_MASS = 2.95 kg` as its own comments and every
  AM-T650 YAML require. The accidental rotor-mass subtraction made the live
  UAM 3.662249 kg while the controller modeled 3.746170 kg, a 0.823 N hover
  error. The L1 launcher now passes the expected total into Isaac and the app
  aborts on any future mismatch.
- L1 no longer advances on every 250 Hz wall-timer callback. It ignores held
  duplicate mocap samples and advances once per distinct capture stamp using
  the 4 ms Isaac physics step. The actuator message still publishes at 250 Hz
  wall time for PX4. This removes RTF-dependent fictitious disturbances during
  acceleration.
- `r_os` now starts at the bare T650 CoM, not the asset's geometric/model
  origin. The configured model-frame base CoM is
  `[0.00001019, -0.00030900, 0.04178889] m`.
- The external launcher rejects a stale instance of its own controller, and
  the Pegasus launcher requires the matching controller process rather than
  accepting any running Micro XRCE-DDS agent.

#### 7.12.4 +20% thrust-coefficient stress-test tuning (2026-08-21)

The controller allocator was set to `5.6159172e-05 N/(rad/s)^2`, +20% from
Pegasus's computed `4.679931202e-05`; Pegasus itself was not changed. Because
allocation divides by the controller coefficient, this makes physical thrust
16.67% lower than the controller predicts. The pre-existing
`armff_mismatch_x=0.0085 m` injection remained active, so the sweep covered
the thrust error and the named CoM-model error together.

The original `l1adapt_max_thrust_n=6` was authority-limited: `u_L1` stayed at
6 N for 93–100% of DIRECT and hover settled near `x=-0.605 m`, `z=0.830 m`
for the `[0,0,1] m` reference. Raising only the bound to 10 N removed the rail
but left the known PWC one-sample under-read. Sweeps over `A_s` and `omega_c`
selected:

```yaml
l1adapt_as_v: 2.0
l1adapt_as_omega: 2.0
l1adapt_omega_c: 6.0
l1adapt_max_thrust_n: 10.0
```

At the 4 ms sample time the constant-disturbance factor is
`exp(-2*0.004)=0.992`. The final full trial (SAFETY takeoff, DIRECT handover,
0.5 m X step/return and 0.25 m Z step/return) measured 2.4–3.0 cm settled X
error, 7.3 mm Z error, 5.32–5.44 s X settling with 7.6–8.5% overshoot, and
2.53 s Z settling with less than 0.1% overshoot. Peak L1 thrust was 8.27 N,
peak adaptive torque 0.23 N·m, peak tilt 4.80 degrees, and peak motor command
0.619; no adaptive/motor rail or watchdog event occurred. DIRECT handover L1
settling improved from 2.58 s at the original `omega_c=2` to 0.88 s, with the
minimum altitude improving from 0.688 m to about 0.895 m.

The geometric position pair remains `Kp=[4,4,8]`, `Kv=[6,6,10]`. A candidate
`Kp=[6,6,10]`, `Kv=[8,8,12]` shortened X rise time but increased overshoot to
12–13%, settling to about 9.1 s, and later crossed the 3 m lateral test bound
during the Z-return segment. The harness reverted to SAFETY and the candidate
was rejected.

#### 7.12.1 Run sequence — copy-paste, per machine

Same shape as §7.10.1 (clean slate → build → ROS 2 stack → Pegasus/PX4 + arm →
OFFBOARD/arm → DIRECT), with only the two launcher names changed. **Every
command below is absolute-path and self-contained** — each one runs on its own
from ANY directory, including `~`. The only `cd` left is the one *inside* the
build command, because `colcon` writes `build/`/`install/` into the current
directory and so genuinely must run from the workspace root.

**fsc_lab_machine** (the lab desktop, user `fsc-jupiter`). Its `.bashrc`
already sources ROS 2 and the workspace overlay, so a fresh terminal needs no
manual `source`:

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. build after every pull (any terminal — the cd IS part of the command)
cd ~/Workspaces/fsc_autopilot_ws && colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 3. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2)
~/Source/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh fsc_lab_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}

# 5. take off in SAFETY from the ground station, settle at the hover
#    reference, THEN hand the vehicle to the geometric+L1 law (terminal 3)
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: true}"

# ABORT back to SAFETY — have this line ready BEFORE entering DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: false}"

# 6. PX4 refuses an in-air disarm: land by reference first, then
ros2 service call /uav_0/rc/disarm std_srvs/srv/Trigger {}
```

**shiqi_machine** (shiqi-desktop) — identical apart from the two repo roots
(`~/ros2_ws` and `~/fsc_PegasusSimulator`). If `ros2` is not on the PATH in a
fresh terminal there, prefix the service calls with
`source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash &&`:

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. build after every pull (any terminal — the cd IS part of the command)
cd ~/ros2_ws && colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 3. Pegasus / PX4 SITL + ARM STACK + ARM GROUND STATION   (terminal 2)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh shiqi_machine

# steps 4-6 are machine-independent — use the fsc_lab_machine block above
```

**Never chain step 0's two lines with a launcher on the same line.**
`kill_stale_sim_processes.sh` pattern-kills any shell whose command line
mentions a launcher, so `kill_stale… && start_…` kills the very stack it just
started (the §7.10.3 harness trap; the same pattern-matching bit me during
this node's bench-check, killing my own shell twice).

Session name `fsc_geometric_l1_direct_actuation_t650_aerial_manipulator_stack`
(step 0's `stop_isaacsim_stack.sh` auto-discovers it). Prefix step 2 with
`AUTO_DIRECT=1` to enter DIRECT pre-arm instead of taking off in SAFETY
(§7.10's semantics) — but for a first flight of this law, prefer the default:
take off on the proven baseline and switch in the air, so step 5's abort line
is a live escape route. All of §7.10.1's other traps apply verbatim (the
source-tree-yaml rule, the arm repo as a third repo to pull and rebuild, two
ground stations, `Ctrl-b d` to detach — never Ctrl-C).

`l1_control_debug` (**44** elements, FLU): [0..2] e_p, [3..5] e_v, [6..8] e_R,
[9..11] ω, [12] f_b N, [13..15] M_b N·m, [16] u_L1 thrust, [17..19] u_L1
torque, [20..23] γ̂_m, [24..25] γ̂_um, [26..31] ζ̃, [32..35] motor commands,
[36..38] r_os, [39] r_os source (0 none / 1 static / 2 live), [40] L1 active,
**[41..43] the INJECTED r_os model mismatch** (`armff_mismatch_*`, zero unless a
sim robustness config sets it). This line said 41 until 2026-08-24 — the last
three were appended with the injection on 2026-08-20 and the doc was not
updated. They are what lets an analysis recover the TRUE r_os the plant flies
([36..38] minus [41..43]) instead of the corrupted one the law believes.

#### 7.12.5 FLIGHT TEST — +20% kf AND +10 mm CoM, two matched runs (2026-08-24)

**The AM geometric+L1 node had never flown.** It has now, twice, end to end, with
BOTH robustness injections active at once and no gain touched. The CoM injection
was raised `armff_mismatch_x` 0.0085 → **0.010 m** (user request: the round top of
the 8–14 mm band the 0819 hardware flights bracket, rather than its mid-point);
`alloc_thrust_coeff` stays **5.6159172e-05** against the plant's 4.679931202e-05,
i.e. **+20%** exactly.

Mission = §7.13.2's, so the numbers sit beside the bare-T650 campaign's: SAFETY
takeoff to z = 1.2 → settle → DIRECT → 45 s soak → 0.5 m X step/return → 0.25 m Z
step/return → 10 s soak → SAFETY → reference landing. Driven by the SAME recorder
the bare campaign uses (`l1_payload_campaign_driver.py`, AM args below); scored by
a new `am_l1_robustness_metrics.py` that knows this plant's mass/bound and reads
the AM-only debug fields [36..43]. Data: `docs/sim_to_real_t650/am_l1_robustness_20260824/`.

```bash
# after this section's steps 0-3, instead of driving by hand:
cd ~/Source/fsc_PegasusSimulator/docs/sim_to_real_t650/tools
source /opt/ros/humble/setup.bash && source ~/Workspaces/fsc_autopilot_ws/install/setup.bash
/usr/bin/python3 l1_payload_campaign_driver.py --namespace /uav_0 \
  --hover-z 1.2 --land-z 0.35 --soak 45 --step 15 --out ../am_l1_robustness_20260824/runA_am_l1.npz
/usr/bin/python3 am_l1_robustness_metrics.py ../am_l1_robustness_20260824/runA_am_l1.npz 10.0
```

**Settled DIRECT hover, runs A / B — reproducible to the third decimal:**

| quantity | run A | run B | note |
|---|---|---|---|
| `u_L1` thrust | 7.303 N (sd 0.004) | 7.303 N (sd 0.003) | predicted 7.350; **73% of the 10 N clamp, 0 samples railed** |
| `f_b` | 36.796 N | 36.796 N | mg = 36.750 N — the baseline never notices the kf error |
| commanded → delivered | 44.099 → 36.749 N | 44.099 → 36.750 N | ×(kf_plant/kf_ctrl); **exactly hover, so the mismatch is demonstrably active** |
| motors | .5969/.5409/.5958/.5404 | .5969/.5409/.5959/.5404 | the documented 0.597/0.540 split |
| `u_L1` pitch | +0.2354 N·m | +0.2354 N·m | 16% of the 1.5 N·m clamp |
| residual pitch moment | +0.0115 N·m | +0.0114 N·m | see the accounting trap below |
| \|e_R\| max | 0.0073 | 0.0073 | tilt ≤ 0.14° in the settled soak |
| settled \|e\| rms (SOAK2) | 49.0 mm | 33.9 mm | dominated by the standing X offset |
| Z step final error | −7.4 / −7.2 mm | −7.6 / −7.1 mm | out / return |
| touchdown | 0.339 m, 0.18° | 0.339 m, 0.18° | seats at the 0.305 m resting height |

**THE ACCOUNTING TRAP, and it inverts the conclusion.** The obvious way to score
the CoM injection is `dx_err * f_b` = 0.010 × 36.8 = **0.368 N·m**, and against
that L1's +0.2354 N·m looks like a 64% cancellation — a failure. It is not. The
moment channel carries the **+20% allocation error too**, and on this channel the
two injections partly OFFSET: what the plant needs COMMANDED is
`-dx_true * T * (kf_c/kf_p)` = −0.8622 N·m, the baseline commands −1.0837, so the
required `u_L1` pitch is **+0.2217 N·m** and the measured +0.2354 is **106% of
it**. L1 cancels the injected moment essentially exactly. Never score one
injection without the other when both are on.

**Where the position error comes from — and it is the paper's structure, not a
tune.** The ~0.011 N·m that L1 leaves uncancelled has nowhere to go: the paper's
attitude loop is pure P (no integral, §7.12's whole point) so it parks at
`e_R ≈ 0.0039` rad, and the position loop is pure P/D so it parks at
`T·e_R / Kp ≈ 0.14/4.0 ≈ 3.5 cm`. Measured settled X offset 3.4–4.9 cm. Same
mechanism as §7.13's finding (b), different source (there a thrust-axis
misalignment, here the injected CoM error).

**Do not read the X-STEP "final error" as a settled value.** X shows a slow
residual mode of tens of seconds — at the end of the 15 s step window x was still
moving 0.04 m per 6 s — so the −80/−99 mm "final error" on the +0.5 m X step is a
window artifact. The settled hover offset (45 s soak) is the 3.4–4.9 cm above.
Z, which does not close through the attitude loop, settles inside its window
(−7.1 to −7.6 mm, both runs, both directions).

**Live arm state held for 100% of samples in both runs** (debug [39] = 2), r_os
believed [0.02955, −0.00008, −0.02159] m against a TRUE [0.01955, …] — the
injection is visible and separable in the log, which is what [41..43] is for.
The arm stayed at HOME [0, 40.00, 39.97, 0]° throughout DIRECT.

**Known, expected, not a fault:** the DIRECT→SAFETY abort carries a one-off
~560 mm vertical balloon (A 558, B 567) — §7.13's finding (c): the SAFETY-path
UDE books the DIRECT-only mismatch as a disturbance and hands it to a loop whose
thrust model is honest. An artifact of the ASYMMETRIC sim injection; it does not
transfer to hardware, where a real kf error lives in both models.

**PX4 never disarms this rig** (`Disarming denied: not landed`, forever, even
seated at exactly 0.305 m with the reference pushed below it). Identical to the
bare-T650 campaign — runs A and D there end the same way at `--land-z 0.28`, so
it is NOT the AM and NOT the land target. Consequence is unchanged: **full step-0
clean and relaunch between missions**, which is what was done between A and B.


### 7.13 Bare-T650 GEOMETRIC + L1-ADAPTIVE direct actuation — added 2026-08-21

The NO-ARM parallel of §7.12, on the bare T650 plant of §7.11 — same paper
(Cai et al., CEP 164 (2025) 106418), same SAFETY/DIRECT split, same mode
service NAME, but a separate single-drone node fork
(`fsc_autopilot_ros2_node/single_drone_geometric_l1_direct_actuation`,
executable `autopilot_drone_geometric_l1_direct_actuation_node`) with NO arm
model anywhere: r_os ≡ 0 on a bare airframe, no `armff_*` keys, and the debug
array is 37 elements ([0..35] identical to §7.12's, [36] = L1 active; no r_os
block). Params:
`config/params_single_drone_geometric_l1_direct_actuation_t650.yaml`.

**The executable name is DECORATED, and the decoration is INFIXED**
("autopilot_" + "drone_" + the rest). So `autopilot_geometric_l1_direct_actuation_node`
is **NOT** a substring of `autopilot_drone_geometric_l1_direct_actuation_node`:
neither name matches the other under `pgrep -f`, and each pattern selects
exactly ONE fork. That is what lets each Pegasus launcher refuse the wrong
controller — but it also means **every stale-node guard must list BOTH names
explicitly**. All three L1 scripts shipped on 2026-08-21 with the decorated
name missing from their guard lists, i.e. unable to detect a stale instance of
their own controller; found and fixed the same day by direct string test. Do
not "simplify" those lists back down.

**The shipped yaml is the +20% thrust-coefficient ROBUSTNESS config**
(`alloc_thrust_coeff 5.6159172e-05` vs the plant's 4.679931e-05, §7.12.4's
protocol without the r_os injection — there is no arm model to corrupt).
Physical thrust runs 16.67% below what the controller predicts, and closing
that gap is u_L1's job: in a settled DIRECT hover expect **u_L1 thrust (debug
[16]) near +5.9 N, NOT near zero** (predicted +5.95: 4.96 N of missing
physical thrust × the same wrong coefficient on the way back out), motors
~0.5025 symmetric, f_b ≈ mg = 29.8 N. Restore the matched kf in the yaml for
a clean-model baseline.

#### 7.13.1 Run sequence — copy-paste (fsc_lab_machine)

Same shape as §7.12.1; only the two launcher names change. Every §7.12.1 trap
applies (never chain step 0's lines with a launcher; `Ctrl-b d`, never
Ctrl-C; source-tree yaml rule). There is no arm repo and no arm ground
station in this rig, so step 3 brings up two panes where §7.12.1 brings four.

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. build after every pull (any terminal — the cd IS part of the command)
cd ~/Workspaces/fsc_autopilot_ws && colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_geometric_l1_direct_actuation_t650_stack.sh fsc_lab_machine uav_0

# 3. Pegasus / PX4 SITL     (terminal 2)
~/Source/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_geometric_L1_adaptive_direct_actuation_sitl.sh fsc_lab_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}

# 5. take off in SAFETY from the ground station, settle at the hover
#    reference, THEN hand the vehicle to the geometric+L1 law (terminal 3)
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: true}"

# ABORT back to SAFETY — have this line ready BEFORE entering DIRECT
ros2 service call /uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: false}"

# 6. PX4 refuses an in-air disarm: land by reference first, then
ros2 service call /uav_0/rc/disarm std_srvs/srv/Trigger {}
```

Session name `fsc_geometric_l1_direct_actuation_t650_stack` (step 0
auto-discovers it — `stop_isaacsim_stack.sh` builds its known-session list by
grepping `^SESSION=` out of `scripts/isaacsim/start_*.sh`, verified to list
this one). `AUTO_DIRECT=1` on step 2 enters DIRECT pre-arm, §7.12's semantics.
shiqi_machine: swap the two repo roots as in §7.12.1.

Every path, argument and service name in the block above was re-verified
against the scripts and the node source on 2026-08-21: both step-0 scripts are
executable; step 2/3 take `<machine_config> [uav_prefix]` and `<machine_config>`
respectively, with `fsc_lab_machine.conf` present; and the mode service resolves
to `/uav_0/fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode`
(node name `fsc_autopilot_ros2`, launch namespace `uav_0`, relative service
`fsc_autopilot_ros2/geometric_l1_direct_actuation/set_direct_mode`).

#### 7.13.2 Campaign results (2026-08-21, +20% kf, full SITL)

Mission: SAFETY takeoff to z 1.0 → DIRECT → 25 s soak → 0.5 m X step/return
→ 0.25 m Z step/return → SAFETY → reference landing → disarm.

- **The geometric sibling's attitude pair (kr 2.4/kΩ 0.73) is NOT safe on
  this node.** Its rate-loop crossover (kΩ/I = 9.5 roll / 11.4 pitch rad/s)
  sits on the MN4010 rotor-lag pole (10.03 1/s); the pure geometric law
  tolerates that, but the L1 torque channel's ωc = 6 LPF + one-sample delay
  eat the rest of the margin. Measured: a ~2.4 Hz roll/pitch mode excited by
  the first X step decayed in run 1 but GREW +3%/s for 55 s in run 2 (u_L1
  torque envelope 0.12 → 0.62 N·m) until the vehicle flipped — the §7.12.4
  marginal-system signature, decided by run-to-run scatter. **Shipped kr
  1.2/kΩ 0.45** (crossover 5.8/7.0 rad/s, wn 3.9/4.3, ζ 0.74/0.81):
  repeat-tested 3/3, step transient decaying every run.
- **Settled numbers on the shipped config** (3/3 complete flights, spread
  under 1%): u_L1 thrust settles **+5.92 N** in every run (predicted +5.95),
  never railed (bound 10 N, peak 6.6); u_L1 torque ≤ 0.053 N·m vs the 1.5
  clamp; motors 0.502 mean, symmetric; soak tilt ≤ 0.5°. Z step: rise 2.4 s,
  settle 3.5 s, overshoot ≤ 2.6%, final error 6 mm. X step: rise 3.9–4.4 s,
  return overshoot 7–10%, both parking on the standing offset below. DIRECT
  handover min z = 0.907 m (ref 1.0) in all three runs; X-step transient
  peak 3.6° tilt, fully decayed within 4 s every run.
- **A constant ~4–5 cm −x/−y hover offset is the PLANT, not a gain
  problem.** γ̂_um (debug [24..25], the unmatched estimate) reads a real
  ~0.20 N lateral force on x650_new.usd (the known built-in thrust-axis/
  body-frame misalignment class, ~0.4°), and the paper's pure-P position
  loop parks exactly at e_p = F_lat/Kp — measured Kp·e_p = γ̂_um to a few
  percent in all three runs. The law has no integrator and the unmatched
  channel is estimated but physically unactuatable at fixed attitude. It
  also biases the X-step metrics (the step "undershoots" by the same 4 cm).
  Raising Kp would shrink it; the §7.12.4-rejected stiffer pair says don't.
- **DIRECT→SAFETY handover carries a one-off ~6 N vertical surge — an
  artifact of the ASYMMETRIC injection, expected and bounded.** During
  DIRECT the SAFETY-path UDE (fed the allocator's believed collective) books
  the mismatch as a ~6 N disturbance; on the abort edge SAFETY compensates
  through its own CORRECT thrust model, ballooning z by ~0.5 m for ~2 s
  until the UDE re-learns (gain 2.0). On hardware a real kf error lives in
  BOTH models, so this artifact does not transfer. Do not "fix" it by
  softening ude_gain — that slows the real abort path.

#### 7.13.3 769 g PAYLOAD campaign (2026-08-24) — the config the rig ships today

The 2026-08-21 HARDWARE loaded flight (report *L1 Against Battery Fade*, bag
`debug_recording_l1_direct_..._769g20260821_192638`) saturated: `u_L1` sat on
its 10 N clamp for **82.2%** of DIRECT while the requirement grew 12.8 -> 15.3 N,
and the vehicle sagged 0.35 -> 0.65 m. The report's split of that demand is the
whole story -- **8.28 N was `vehicle_mass` never being told about the payload**
and only 4.4-6.8 N was the battery fade the campaign was actually testing.

**Three changes, no gain touched.** Plant and controller move TOGETHER:

| | bare | 769 g loaded |
|---|---|---|
| Isaac plant (`PEGASUS_PAYLOAD_MASS`) | 0 | **0.769** -> total **3.802921 kg** |
| `vehicle_mass` | 3.033921 | **3.802921** |
| `vehicle_thrust_scaling` / `vehicle_idle_thrust` | 0.040232 / 0.203169 | **0.035935 / 0.238967** |
| `l1adapt_max_thrust_n` | 10.0 | **18.0** |
| hover command | 0.5025 | **0.5741** |
| `alloc_thrust_coeff` | 5.6159172e-05 | **unchanged (+20%)** |

`thrust_scaling`/`idle_thrust` are the SAFETY path's tangent linearization and
had to be **re-derived, not copied** -- +25.3% of mass moves the hover point far
off the bare anchor (the AM-T650's +23% lesson). The payload is added to the
Isaac body mass at the CoM with the inertia tensor UNCHANGED, so the sim does
**not** reproduce the hardware payload's ~4 mm forward CoM shift (0.5 -> 0.8 N.m
of standing pitch moment); expect a much smaller `u_L1` torque here than on
hardware.

**`4.679931e-05` IS STILL THE SIM PLANT TRUTH, so +20% is still 5.6159172e-05.**
The report identified the REAL vehicle's coefficient as the MN4010 **bench**
value `4.540431e-05` at a fresh pack, i.e. `t650_params.THRUST_FIT_FACTOR`
(1.030724) is a sim-only hover-command match with no hardware counterpart -- but
it changed nothing in the simulator ("Fits only -- nothing has been wired into
the simulator"), and Isaac still builds its thrust curve from
`t650_params.ROTOR_CONSTANT`. In sim the plant truth is whatever Isaac applies.
The finding is a **hardware**-yaml item: the flown allocator was 3.0% optimistic
at the start of that flight and 10.3% by the end.

**Flown 2026-08-24, full SITL, mission = section 7.13.2's** (SAFETY takeoff to
z 1.0 -> DIRECT -> 45 s soak -> 0.5 m X step/return -> 0.25 m Z step/return ->
SAFETY -> reference landing). **Two matched runs, A and B, agree to the third
decimal:**

| | run A | run B | predicted |
|---|---|---|---|
| `u_L1` thrust, settled | **7.416 N** | **7.416 N** | 7.461 |
| -- as a fraction of the clamp | 49.4% of 15 | 49.4% of 15 | -- |
| samples on the rail | **0 of 28750** | 0 | -- |
| `u_L1` over the whole 115 s of DIRECT | 6.79-8.23 N | 6.80-8.23 N | -- |
| baseline `f_b` | 37.353 N | 37.352 N | mg = 37.307 |
| commanded -> physical | 44.769 -> **37.307 N** | same | = mg exactly |
| motors | 0.5741 symmetric | 0.5741 | 0.5741 |
| `u_L1` torque, max abs xy | 0.047 N.m of 1.5 | 0.046 | -- |
| soak tilt p-p | 0.15 deg | 0.15 deg | -- |
| X step rise / return overshoot | 3.84 s / 10.9% | 3.30 s / 13.0% | bare: 3.9-4.4 s / 7-10% |
| Z step rise / overshoot / final err | 2.60 s / 2.0% / 7.3 mm | 2.40 s / 1.5% / 7.2 mm | bare: 2.4 s / <=2.6% / 6 mm |
| touchdown | 0.305 m, 0.00 deg | 0.305 m | resting height |

`f_b` sitting at mg while the commanded collective is 44.77 N is the report's
Flight-A signature reproduced: **the baseline law never notices the kf error;
the augmentation absorbs all of it.** The mismatch is demonstrably ACTIVE --
44.769 * (4.679931/5.6159172) = 37.307 N delivered = exactly hover.

Two things that are the PLANT, not the tune, both matching 7.13.2's bare
findings: a constant ~5 cm -x/-y hover offset (gamma_um = [-0.21, -0.11] N, the
x650_new.usd thrust-axis misalignment; the integrator-less pure-P loop parks at
F_lat/Kp, which is also the -44 to -49 mm the X step "undershoots" by), and a
one-off DIRECT->SAFETY vertical surge. The surge is **bigger loaded -- 622 mm vs
the bare ~500 mm** -- because the SAFETY-path UDE books a larger mismatch (7.4 N
vs 5.9 N) and hands it to a SAFETY loop whose thrust model is honest. Artifact
of the ASYMMETRIC sim injection; it does not transfer to hardware.

**Run C -- last week's configuration on the loaded plant -- cannot even take off,
which is a stronger result than the expected sag.** With `vehicle_mass` at the
bare 3.033921 and the 10 N bound restored, the vehicle sat at 0.305 m for 60 s
with a z = 1.5 m reference. Measured commanded collective **32.1 N against a
37.31 N weight**: the bare-mass gravity feedforward is 29.76 N and the position
term adds ~2.3 N, so it is 5.2 N short -- and the UDE that would supply the rest
is gated off by `ude_height_threshold: 0.4` below 0.4 m, which the vehicle
cannot reach without it. A ground deadlock. (Predicted sag had it flown:
626 mm at the 10 N bound, ~1 mm at 15 N -- i.e. with the mass wrong the demand
is 15.005 N and the *raised* bound is consumed exactly, which is why BOTH
changes were needed, not either one.)

**BOUND RAISED AGAIN, 15 -> 18 N (same day, user request).** 15 N covers the sim's
kf mismatch with margin, but a real loaded sortie carries TWO independent error sources and
only the first exists in simulation:

| source | worst case | where measured |
|---|---|---|
| allocator kf error | 7.42 N | this campaign, sim, at the shipped +20% |
| battery fade | 6.8 N, **growing** 4.4 -> 6.8 over 2.5 min | 2026-08-21 hardware |
| **sum** | **14.2 N** | -> 18 N leaves ~21% margin |

Even railed, the commanded collective is 62.8 N against the allocator's own 99.8 N ceiling;
the DIRECT watchdog (40 deg tilt / 360 dps) is the real envelope guard, not this clamp. The
matched estimate automatically gets 2x this value as its own anti-windup range
(`l1_adaptive_augmentation.cpp:152`, `2.0 * max_thrust_n`), so there is no separate estimate
bound to move.

**The 15.3 N peak in the hardware flight is NOT what this bound is sized against.** 8.28 N
of that was `vehicle_mass` left at the bare airframe; with the mass fixed the same flight
would have asked for ~7.0 N. **If `u_L1` heads for 15 N again with the mass correct, the
mass or the kf assumption is wrong** -- raising the bound further would only hide it.
Re-flown at 18 N (run D, 90 s soak): `u_L1` **7.415 N**, identical to runs A and B to the
third decimal, **41.2% of clamp, 0 samples on the rail**, touchdown 0.305 m.

**GROUND-STATION FIX, same day: the N/T rotor pies read 0.0% for this rig.** Same defect as
the whole-body fork's on 2026-08-23 and the same one-line fix -- the drone GS's
`motors_debug` subscription list in
`ros2_ground_station_gui/src/ROS_Node/ros_single_drone_control.py` is PER-NAMESPACE and did
not carry `geometric_l1_direct_actuation`. There are exactly FOUR such namespaces across
the seven forks, and forks SHARE them in pairs (AM + bare-drone):
`direct_actuation`, `geometric_direct_actuation`, `geometric_l1_direct_actuation`,
`whole_body_direct_actuation`. **All four are now in the list; add the namespace whenever a
new fork appears.** Diagnose in one command --
`ros2 topic info /uav_0/fsc_autopilot_ros2/<ns>/motors_debug` showed
`Publisher count: 1, Subscription count: 0`, and reads `1 / 1` after the fix. The widget's
other two gates were already fine: the controller-name test is a SUBSTRING match on
"Direct Actuation" (so "Geometric+L1 Direct Actuation" passes) and `connected` is really
PX4 `pre_flight_checks_pass`. **Needs a ground-station restart to take effect.** Verified
live in DIRECT: M1 57.6 / M2 57.3 / M3 57.4 / M4 57.4%, matching the 0.5741 loaded hover
command; screenshot in `l1_payload_campaign_20260824/gs_direct_18N.png`. The change is in
a shared repo on `dev_robotic_arm`, left UNCOMMITTED.

**On this machine the GS pane of every stack script never starts at all** -- `python3` is
`~/envs/fsc_isaac_env/bin/python3` (3.11) in every shell, and PyQt5 is installed only for
`/usr/bin/python3` (3.10). Start it by hand:
`cd ~/Workspaces/fsc_autopilot_ws/src/ros2_ground_station_gui && PYTHONPATH=src /usr/bin/python3 src/single_drone_ground_control.py`.
The 13 stack scripts were left alone -- it is an environment issue, not a script bug.
Convenient side effect while it is down: `position_controller/reference` has ZERO
publishers, so a driver script is unambiguously the only one. The GUI only publishes a
reference when its Send button is pressed, so the two can coexist.

**Three operating traps, all hit live:**
- **`PEGASUS_PAYLOAD_MASS` and `vehicle_mass` must move together.** The launcher
  defaults the payload to 0.769 to match the shipped yaml. Running the launcher
  bare against this yaml gives the controller a 769 g phantom.
- **Never fly two missions off one launch.** PX4's land detector never sees this
  rig land (`Disarming denied: not landed` forever), so the vehicle stays armed;
  a second run then finds PX4 latched "in flight" AND the SAFETY UDE integrating
  the ground reaction, and the takeoff produces **zero lift with no error
  message** (measured: 60 s at ref z = 1.0, altitude constant to 3 decimals).
  Full step-0 clean and relaunch between flights. The driver now refuses to
  start against an armed vehicle for exactly this reason.
- **Land to z = 0.28, not lower.** The bare T650's resting height is **0.305 m**
  (measured, not the 0.07 m spawn).

Tooling, so the flown configuration is reproducible from the repo rather than
reconstructed from a bag (the report's own complaint):
`docs/sim_to_real_t650/tools/l1_payload_campaign_driver.py` (drives the whole
mission, records odom + the 37-element `l1_control_debug` + the reference
actually streamed, aborts to SAFETY on a 35 deg / 3.5 m envelope) and
`l1_payload_campaign_metrics.py`. Runs A/B/C are in
`docs/sim_to_real_t650/l1_payload_campaign_20260824/`. **Both need the SYSTEM
python** (`/usr/bin/python3`) -- the default `python3` on this machine is a uv
3.11 and rclpy is built for 3.10.

```bash
# after the section 7.13.1 steps 0-3, instead of driving by hand:
cd ~/Source/fsc_PegasusSimulator/docs/sim_to_real_t650/tools
source /opt/ros/humble/setup.bash && source ~/Workspaces/fsc_autopilot_ws/install/setup.bash
/usr/bin/python3 l1_payload_campaign_driver.py --namespace /uav_0 \
  --hover-z 1.0 --land-z 0.28 --soak 45 --step 15 --out runA_769g.npz
/usr/bin/python3 l1_payload_campaign_metrics.py runA_769g.npz
```

Not covered by this campaign: battery fade (the sim has none -- the demand here
is constant, where hardware's grew 4.4 -> 6.8 N over 2.5 minutes), and the
payload's CoM offset. On a real loaded sortie budget for BOTH on top of the
7.5 N measured here, which is what the 15 N bound is sized for.

#### 7.13.4 HARDWARE config split out (2026-08-24)

`params_single_drone_geometric_l1_direct_actuation_t650.yaml` **is now the
HARDWARE config.** The simulation one it came from is preserved verbatim as
**`..._t650_sim.yaml`** -- that is the file the three matched SITL runs of
7.13.3 flew -- and `scripts/isaacsim/start_geometric_l1_direct_actuation_t650_stack.sh`
now points there. The hardware script
(`scripts/indoor_exp/start_geometric_l1_direct_actuation_stack_t650.sh`) keeps
pointing at the unsuffixed name.

**Why the split.** Until now ONE file served both stacks, and on 2026-08-19 the
AM geometric node's first hardware flight took off on the unmodified simulation
yaml and diverged in **1.3 s** -- pilot rescue at z = 0.246 m. It was verified
from the bag that the "FOR HARDWARE swap in the bench pair" header notes had
never been applied, and two of the three measured causes were the two allocator
constants below.

**Exactly 10 keys differ, verified by loading both files; all 81 keys present in
both and EVERY GAIN IDENTICAL:**

| key | sim | hardware | why |
|---|---|---|---|
| `alloc_thrust_coeff` | 5.6159172e-05 | **4.540431e-05** | bench. Drops the +20% injection AND the sim's x1.030724 hover-match factor. The 2026-08-21 hover balance put the real coefficient on the bench value to **-0.08%** at a fresh pack, so the sim number starts a flight 3.0% optimistic and ends 10.3% optimistic |
| `alloc_rotor*_km` | +-0.052867 | **+-0.018164** | bench (= c/kf exactly). The sim value carries `t650_params`' x3.0 stand-in for the unmodelled `I_rotor*omega_dot` reaction. **2.91x**, and NOT inert -- on 2026-08-19 it gave a measured yaw response of 0.35x commanded, exactly 1/2.91 |
| `l1adapt_fixed_sample_time_s` | 0.004 | **0.0** | measured capture-stamp deltas. 0.004 is Isaac's fixed step. The report recovered 0.0 as the flown hardware value from the LPF recursion (implied dt 3.44/13.05/16.62 ms = the bimodal mocap cadence) |
| `vehicle_mass` | 3.802921 | **3.878000** | as-flown. **INFERRED from the 2026-08-21 hover balance, NOT weighed** -- 75 g more than bare + payload, likely mounting hardware. See below |
| `vehicle_thrust_scaling` / `idle_thrust` | 0.035935 / 0.238967 | **0.036128 / 0.247419** | SAFETY tangent linearization, re-derived at the bench kf AND that mass (hover command 0.5910, T/W 2.54) |
| `vehicle_name` | T650-L1-769g | T650-L1-769g-HW | so the GS says which one is loaded |

**`vehicle_mass` IS THE ONE FIELD THAT STILL NEEDS A SCALE.** 3.878 is an
inference from an assumed bare mass and a thrust model; the report says plainly
"weigh the airframe as flown". It is preferred over the arithmetic 3.802921
because erring 75 g low is 0.74 N the augmentation has to find -- the exact
error class this revision removes. If the scale says 3.802921, the matching
linearization pair is **0.036483 / 0.244076** (both alternatives are written
into the yaml next to the value).

**Kept deliberately, with reasons in the file:**
- **`l1adapt_as_v/as_omega` 2.0/2.0, `omega_c` 6.0** -- these ARE the hardware
  values. The report read A_s = -2 straight out of the 2026-08-21 logs; net
  delivery was 94-95% with a flat 0.09-0.17 N residual, and the measured
  under-read (2.5-3%) matched the `e^{A_s*T}` prediction (1.6%). The paper's
  faster 65/80 would chase the last 5%, but that is its own step, not this
  flight's.
- **`l1adapt_max_thrust_n` 18.0** -- with the bench kf the allocator error
  largely goes away, so the dominant remaining demand is battery fade
  (4.4 -> 6.8 N measured). Expect `u_L1` to settle in the **4-7 N band and walk
  upward** with the pack. Heading for 15 N means the mass or kf is wrong.
- **`l1geo_kr/komega` 1.2/0.45 -- the ONE block carried over UNTESTED.** The
  geometric sibling's hardware guidance is "start 1.2/0.75" (its further
  softening was an RTF artifact), but this node closes the L1 torque channel
  around the same axes and its omega_c = 6 LPF + one-sample delay eat margin
  the geometric node never spends: at 2.4/0.73 in sim a ~2.4 Hz mode decayed in
  one run and grew +3%/s to a flip in the repeat. The softer pair is kept for a
  first flight. **Never raise komega for damping against the lag pole.**

**The hardware launcher's pre-flight guard was widened** from the single +20%
string to all five sim values, and now also PRINTS POSITIVE CONFIRMATION of the
three that must be right plus the mass -- a silent pass was previously
indistinguishable from a grep that stopped matching. Tested both ways: it
passes the hardware file clean and catches the sim file on three counts. One
trap found while testing: the `0.0` check must be **anchored to end-of-line**,
because unanchored it is a substring of the sim's `0.004` and reports OK on the
wrong file.

Verified by launching the node on the hardware yaml under `uav_test` and reading
every changed parameter back with `ros2 param get`. **STATUS: never flown in
this form** -- geometric+L1 is unflown on hardware on any airframe.

### 7.14 AM-T650 WHOLE-BODY direct actuation (torque-mode arm) — added 2026-08-22

The whole-body coupled airframe+arm law — `utils_controller/controller.py`'s
EE impedance + shaped task inertia + generalized-momentum disturbance observer
(GMO) + DLS arm inverse + saturation-consistent coupling — running as a C++
fsc_autopilot_ros2 fork (`single_aerial_manipulator_whole_body_direct_actuation`,
node `autopilot_whole_body_direct_actuation_node`), commanding BOTH the four
rotors (ActuatorMotors, `direct_actuator`) and the FOUR ARM JOINT TORQUES.
The §7.9 arm stack moved from position to TORQUE mode for this rig; both
ground stations are live (drone GS = position/yaw reference, arm GS = the
goal the torque controller smooths into the joint reference the law tracks).
Derived from the §7.12 rig (same
SAFETY/DIRECT split, watchdog, feedback-loss failsafe, allocator). The
controller, plant and torque bridge are isolated additions; the shared ground
stations and launcher guards are extended only to recognize this new mode.

```
drone GS ── position_controller/reference ─────────────┐
arm GS ── target_joint_setpoint ─► ExternalTorqueController min-jerk generator ─┐
              smoothed_reference_joint_trajectory (q_d, q̇_d) ──────────────┤
raw-mocap estimator odom + fmu/out/* ──────────────────┤
/uav_0/fsc_open_manipulator/joint_states (BY NAME) ────┤
                                                        ▼
        autopilot_whole_body_direct_actuation_node
        ├ SAFETY: baseline robust_ctrl+UDE → VehicleAttitudeSetpoint
        │         + THIS NODE's PD+gravity ARM HOLD (kp 3.0/kd 0.25,
        │         controller-smoothed reference, clamp 3.0)
        │         — arm torques stream in BOTH modes
        └ DIRECT: wb law (250 Hz, model frame + Rz(-90) boundary adapter)
              u1,tau_body → WrenchAllocator → fmu/in/actuator_motors
              tau_joint ──► external_torque_controller/joint_torque_command
                              ▼
        ExternalTorqueController (clamp/limit-pullback/stale-PD-fallback/
        zero-on-deactivate) → IsaacTopicEffortSystem
              → /uav_0/isaacsim_manipulator/effort_commands → 06 plant
              (fresh → apply clipped ±3.0; stale → PD+gravity hold at the
               latched pose — a torque stream must never latch)
```

Key things worth having in hand:

- **PARITY-LOCKED PORT.** `wb_model.cpp`/`wb_controller.cpp` reproduce the
  Python law to ≤1e-8 over 220 random cases + 10-step GMO rollouts
  (`fsc_autopilot_tests --gtest_filter='WbParityTest.*'`; fixture
  `tests/data/wb_truth_t650.json`, regenerated by fsc_PegasusSimulator's
  `application/robotic_arm/utils/generate_wb_truth.py`). The MODEL lives in
  code (`WholeBodyParams::t650Defaults` — the T650 body override applied
  CORRECTLY in the model frame, xx/yy swapped + Ixy negated, NOT 05's
  un-rotated delta); the yaml carries only gains — the exact
  make_params-in-code / gains-in-yaml split the Python has.
- **DIRECT entry is GATED** (the sequencing rule made mechanical): armed +
  OFFBOARD, odom + arm states fresh, pos err < 0.15 m, speed < 0.20 m/s,
  controller-smoothed arm reference fresh, arm err < 0.05 rad — a refusal
  names every red gate. On entry the node
  captures the CoM anchor (measured CoM minus GS reference) so the switch
  instant has exactly zero position error; GS steps then translate 1:1.
- **DIRECT THROTTLE IS LIVE IN THE DRONE GS.** The whole-body node publishes
  `whole_body_direct_actuation/motors_debug`; the four rotor pies show those
  normalized commands and the total normalized-throttle bar shows their mean.
  Rendering is keyed to the active direct controller (a SUBSTRING test on
  "Direct Actuation", so "Whole-Body Direct Actuation" passes) AND on the
  GUI's `connected` field — which is actually PX4 `pre_flight_checks_pass`,
  so if the pies ever blank mid-flight while DIRECT is healthy, that gate is
  why (an earlier revision of this section claimed the gate had been removed;
  it has not been, verified in the GUI checkout 2026-08-23).
  **The GUI must also SUBSCRIBE to this fork's topic.** Its subscription list
  is per-fork and named `whole_body_direct_actuation/motors_debug` here;
  until 2026-08-23 the list held only `direct_actuation` and
  `geometric_direct_actuation`, so the four pies read 0.0% for an entire
  whole-body DIRECT flight while the node published normally (`ros2 topic
  info ...` showed `Subscription count: 0` — the one-command diagnosis).
  Fixed in `ros2_ground_station_gui/src/ROS_Node/ros_single_drone_control.py`;
  a GUI restart is required to pick it up.
  The same mean is mirrored to PX4's `vehicle_thrust_setpoint` (with body -z
  sign) so PX4's land detector also sees the collective while
  `direct_actuator=true`.
- **THE ARM GS HAS A THIRD `WB-TORQUE` STATE.** Controller discovery recognizes
  the active `external_torque_controller`, keeps the panel and its command
  curves scoped to that controller, and publishes joint-space or solved
  end-effector targets to its `target_joint_setpoint`. The torque controller is
  the single trajectory owner: it generates activation homing, **Home-button**,
  joint-target and end-effector-target motion, then publishes the named
  position/velocity curve on `smoothed_reference_joint_trajectory`. Both the
  GUI dashed lines and the whole-body reference builder consume that exact
  curve. The Isaac arm therefore follows the plotted minimum-jerk trajectory
  instead of a second, independently slewed approximation.
- **HOME IS `[0, 40, 40, 0]` degrees.** The torque controller stores
  `[0, 0.698132, 0.698132, 0]` radians, moves there on activation, and the Home
  buttons call its `go_home` service. SAFETY continues publishing torque, so
  initialization and Home work before DIRECT is selected as well as during
  whole-body flight.
- **TORQUE MODE IS A BRING-UP CHOICE, NOT AN IN-FLIGHT SWITCH.** The
  [ROBOTIS XM430-W350 control table](https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
  defines Operating Mode in EEPROM, writable only with Torque Enable off, and
  warns that changing Operating Mode/Torque Enable can reset Present Position.
  The whole-body launcher therefore starts the arm on effort/current interfaces
  and keeps it there; POSITION is not loaded and is greyed out in this rig.
- **ATTITUDE PAIR IS NOT THE PYTHON TUNE.** The in-process 03 rig flew
  k_R=4/k_w=1.5/M_r_d=AM_realign; over DDS (mocap→estimator→node→PX4→Isaac
  transport latency the in-process loop never had) that tune grew a rate
  oscillation that tripped the 360 dps watchdog 20 s after the first-ever
  engagement (tilt only 14° — a rate mode, 03's crash signature). Shipped:
  k_R=2.0/k_w=1.1 with M_r_d = the ACTUAL T650 M_r diag at home
  [0.0777, 0.0907, 0.0834] (natural inertia — the old numbers under-stated
  roll/pitch inertia 20–40%, silently amplifying the loop). That matched-model
  tune has wn 4.9 rad/s and ζ 1.34 against the 10.0 rad/s rotor-lag pole. The
  current +15% hover stress configuration raises only `k_w` to 1.5, as listed
  in §7.14.2.
- **tau_max = 3.0 N·m appears three times** (wb yaml, ExternalTorqueController
  max_effort, the 06 plant's TAU_MAX) — keep them one number.
- Three-layer arm fallback: WBC SAFETY hold → controller stale-PD (0.3 s
  timeout, re-arm after 3 fresh) → plant PD+gravity hold (0.3 s). All three
  verified live (kill any stage and the next takes over).
- Matched-baseline debug: `wb_control_debug` (56 floats, append-only; layout
  in the client header). Watch [17] u1 ≈ 36.75 N, [13..16] arm torques ≈
  gravity terms, [51] n_sat = 0, motors [41..44] ≈ 0.597/0.540. The current
  +15% hover-only robustness injection is documented separately in §7.14.2.

**MATCHED-MODEL SIM-VALIDATED 2026-08-22 (headless full stack, two flights
after the retune):** SAFETY takeoff on a streamed z=1.2 reference → gated DIRECT
engagement → 60 s + 30 s hover soaks; over 179 s total of DIRECT: EE task
error rms 2.36 mm (max 11.3 mm, during the arm step), CoM err rms 3.2 mm,
u1 mean exactly mg, peak joint torque 0.87 of 3.0 N·m, zero saturations,
|e_R| max 0.037, no resonant mode in the e_R spectrum. In-flight arm GS step
(q2/q3/q4 to [0, 0.5, 0.6, 0.3] rad) tracked within 1° in 4 s with the base
held to 5 mm; SAFETY abort bumpless; re-entry gated + clean; staged landing
touched down at z = 0.307/0.311 (resting 0.305), arm back at home.

#### 7.14.1 Run sequence — copy-paste, per machine

Same shape as §7.12.1 (clean slate → build → ROS 2 stack → Pegasus/PX4 + arm →
OFFBOARD/arm → DIRECT), with the launcher names changed and **one extra build**:
this rig is the first to need the ARM repo rebuilt too (the new
`ExternalTorqueController` + `IsaacTopicEffortSystem` and the ground station's
`WB-TORQUE` mode). Every command is
absolute-path and self-contained; the only `cd`s are the ones *inside* build
commands, where `colcon` genuinely must run from the workspace root.

**fsc_lab_machine** (the lab desktop, user `fsc-jupiter`). Arm workspace is
`~/Source/Shiqi/fsc_om_ws`, **not** `~/colcon_ws` (§7.9's warning: that holds
the unrelated upstream ROBOTIS checkout). ros2_control is apt-installed here,
so there is no rosdeps overlay:

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 3. Pegasus / PX4 SITL + TORQUE-MODE ARM STACK + ARM GROUND STATION (terminal 2)
~/Source/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_direct_actuation_sitl.sh fsc_lab_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**shiqi_machine** (shiqi-desktop) — identical apart from the three repo roots
(`~/ros2_ws`, `~/fsc_PegasusSimulator`, arm also in `~/ros2_ws` since
2026-08-23) and the root-less
rosdeps overlay this machine needs for ros2_control:

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 3. Pegasus / PX4 SITL + TORQUE-MODE ARM STACK + ARM GROUND STATION (terminal 2)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_direct_actuation_sitl.sh shiqi_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**Never chain step 0's two lines with a launcher on the same line.**
`kill_stale_sim_processes.sh` pattern-kills any shell whose command line
mentions a launcher. The same pattern-matching bit this rig's own bring-up
from the other side: the stack scripts' `pgrep -f` stale-node guards also
match a SHELL whose command line merely CONTAINS a node name, so a monitoring
command like `pgrep -f autopilot_whole_body…` run in the foreground makes the
launcher refuse to start. Use a bracketed pattern (`[a]utopilot_…`) when
checking by hand.

**Which trajectory planner the whole-body planner uses (2026-09-04).** `planner` selects
the transition-planner backend and **the default is now `bspline`** — so the
sequence above flies the **flat B-spline planner**, not the straight-line one
every earlier flight used. Section 5 of
`config/params_single_aerial_manipulator_whole_body_direct_actuation_t650_sim.yaml`
states it explicitly anyway, so the choice is visible where the gains are. The whole-body planner
reads that yaml through a new `--params-file` on its pane; the launcher prints
the choice before opening the window and the whole-body planner logs it
(`transition planner: BSPLINE …`).

| | `straight_line` | `bspline` |
|---|---|---|
| module | `utils_planner/transition_planner.py` | `utils_planner/flat_bspline_planner.py` |
| EE path | **straight line**, prescribed | whatever a min-snap CoM + min-jerk joints give (~2 cm different) |
| CoM | solved — Picard fixed point | a flat output; compatibility is an identity |
| defect / endpoint | 3e-7 m / 4e-5 m | 2e-16 m / exact |
| `qdot_d` | 1 ms central difference | exact (a spline derivative) |
| constraints | joint box + σ_nd verified, then refuse | rest, joint box, v/a, rotor ω, τ_joint **enforced** |
| plan time | ~300 ms | ~45 ms |
| flown | yes, 2026-08-23 | **no** |

To fall back, change that one key and restart the `whole-body planner` window — the
whole-body planner reads parameters only at startup, the same rule the node has. **The
end-effector path is not the same between the two**: only `straight_line`
holds the EE to a straight line, so a transit that has to clear something is a
reason to switch back.

**Hardware opts OUT, explicitly.** Because the default changed, "says nothing"
no longer means "keeps the flown planner", so
`params_single_aerial_manipulator_whole_body_direct_actuation_t650.yaml` now
carries its own whole-body planner section with `planner: "straight_line"`, and
`scripts/indoor_exp/start_whole_body_direct_actuation_stack_t650_aerial_manipulator.sh`
passes that yaml to the whole-body planner with `--params-file` (it did not before — the
key alone would have done nothing). **Those two halves only work together**:
delete either and hardware silently inherits an unflown planner. Verified by
launching the whole-body planner three ways — bare (BSPLINE), hardware yaml
(STRAIGHT_LINE), sim yaml (BSPLINE).

**Arm ground station says "no known arm controller is loaded" (shiqi_machine,
fixed 2026-09-04).** Symptom is two red lines in the arm GS log — that one, plus
"Controller 'position_controller' has not reported its limits" — while
`joint_states` still ticks at 100 Hz. It looks like a GUI fault and is not:
`external_torque_controller` failed to LOAD, so the panel found no known
controller, retargeted to `position_controller`, and then complained about
*that* one's limits. One root cause, two messages.

The cause is the no-sudo rosdeps overlay: ros-humble-pinocchio 4.0 installs
into `rosdeps/root/opt/ros/humble/lib/`**`x86_64-linux-gnu/`** and only the
parent `lib/` was on `LD_LIBRARY_PATH`, so the controller built fine and then
died at runtime with

```
Failed to load library libopen_manipulator_x_custom_controller.so
dlopen error: libpinocchio_parsers.so.4.0.0: cannot open shared object file
```

Confirm it in the `arm` window's left pane, never from the GUI. Fixed by adding
that arch subdirectory in `~/ros2_ws/rosdeps/local_setup.bash` — **re-add it if
the overlay is ever regenerated**, and note the controller_manager and
joint_state_broadcaster come up normally either way, which is what makes this
look like something else.

**Planned trajectory drawn in Isaac (2026-09-04, user request).** On by
default; `PEGASUS_TRAJ_VIZ=0` baked into the Isaac pane turns it off, and it is
skipped under `PEGASUS_HEADLESS=1` anyway.

* **Blue** — the drone reference path (`x_cd`, the system CoM the law tracks)
  and a heading arrow on the vehicle.
* **Red** — the end-effector reference path (`r_ed`) and its heading arrow.

The curve appears on PLANNED, survives Send, and is cleared when the
transition completes, when a goal is refused, and when the vehicle leaves
DIRECT — a stale curve left on screen after an abort is worse than no curve.
The arrows stream at 20 Hz and are visible while merely holding, before any
plan exists.

The data is the whole-body planner's own reference, already world-frame, on two new
plain `Float64MultiArray` topics — `whole_body_planner/viz_path` (latched,
12 doubles per sample: `x_cd`, `b1_d`, `r_ed`, `b1_de`) and
`whole_body_planner/viz_pose` (the current sample). Deliberately NOT
`WholeBodyReference`: the Isaac process has core message packages only, and
re-deriving the model↔actual frame conversion there would put a second copy of
the whole-body planner's frame boundary somewhere with no business owning one.
`06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py` subscribes and
redraws through `debug_draw` from the RENDER loop (every 8 steps), acquiring
the extension under either of its two namespaces and degrading to no drawing
rather than taking the sim down.

Each arrow is a shaft plus two head strokes; sizes at the top of the file
(`VIZ_ARROW_DRONE`/`_EE` lengths, `VIZ_ARROW_WIDTH`, `VIZ_CURVE_WIDTH`).

**The arrows show the NOSE and the GRIPPER AXIS, not the law's `b1_d`/`b1_de`
(corrected 2026-09-04 after the first look at it).** Publishing the law's own
heading vectors drew the drone's arrow **90° off its nose** — measured
`[0, -1, 0]` at yaw 0 while the arm reaches along `+x`. `b1_d` is
`R0_model·e1`, and `R0_model = R0_actual·Rz(-90)`, so it is minus the ACTUAL
body y: 90° to the vehicle's right, not its front.

The whole-body planner now converts before publishing, on the side where the frame
boundary already lives:

* **nose** = `Rz(+90)·b1_d`. **Exact, not an approximation** — `b1_d` is
  `[cos, sin, 0]` by construction, so the conversion is a pure yaw and the
  model↔actual boundary is a pure −90° yaw. Gives `[1, 0, 0]` at yaw 0, which
  on this airframe is the arm side.
* **claw** = `-R_e·e3`, the gripper axis. `l_i[3] = [0, 0, -0.108]` puts the
  grasp point 108 mm along the EE frame's −z, so that axis IS where the claw
  aims — and unlike any x-axis it carries no frame convention at all. Measured
  `[0.985, 0, -0.174]` at the home pose: forward and 10° down, which is what
  you see. The task heading `b1_de` cannot show that tilt.

Both are asserted in `test_traj_viz.py` against the ARM'S OWN measured reach
direction rather than a hard-coded `[1,0,0]` — a constant would still pass on a
vehicle whose arm had moved, and the reach is the invariant that actually
failed.

Regression: `test_traj_viz.py` covers the part that can silently be wrong —
array layout, unit headings, the curve starting ON the hold, the 20 Hz arrow
rate, the arrows tracking the drawn curve, and both topics clearing on SAFETY.
Passes on both backends; the endpoint difference shows through it, with the
curve starting `0.0e+00` m from the hold under `bspline` and `2.8e-05` m under
`straight_line`.

Regression, both backends: `WB_GOV_PLANNER=bspline` (or `straight_line`)
selects the planner in the three whole-body planner loopback tests
(`test_planner_loopback.py`, `test_go_home.py`, `test_replan_stream.py`);
unset now exercises the new default. **9/9 pass** across default /
straight_line / bspline, 2026-09-04. That suite earned its keep immediately: the B-spline
solve holds the GIL for ~45 ms in one block and `test_replan_stream.py` caught
the reference stream sagging to **60 Hz with an 83 ms gap**. The planner's
input-bound sweep now takes the same `yield_hook` the Picard loop has, and
both backends stream at **98 Hz, 18-19 ms worst gap**.

Session name `fsc_whole_body_direct_actuation_t650_aerial_manipulator_stack`
(step 0's `stop_isaacsim_stack.sh` auto-discovers it). `AUTO_DIRECT=1` exists
on step 2 for parity with the siblings, but on THIS fork the entry gates
refuse a ground-side switch by design — a flight always starts in SAFETY.
All of §7.10.1's other traps apply verbatim (the source-tree-yaml rule — note
the NODE reads params only at startup, so restart the autopilot pane after a
yaml edit; the arm repo as a third repo to pull and rebuild; two ground
stations; `Ctrl-b d` to detach, never Ctrl-C). The position-mode §7.9/§7.12
bring-up and this torque bring-up are separate bring-ups, never a runtime
switch (different command interfaces and, on XM430 hardware, a torque-off
Operating Mode change). The active arm-GS controller indicator must read
`WB-TORQUE`; if it does not, do not send an end-effector command.

`wb_control_debug` (57 elements; DIRECT ticks unless noted): [0] mode (0
SAFETY / 1 DIRECT), [1] law hold, [2] gmo_active, [3] controller-smoothed arm
reference fresh, [4] arm state fresh, [5..8] q, [9..12] controller-smoothed
q_d, [13..16] tau_joint published N·m,
[17] u1 N, [18..20] tau_body model frame, [21..23] tau_body actual FLU,
[24..27] e_y (EE task error), [28..30] e_R, [31..40] d_e_hat (GMO, 10),
[41..44] motor commands, [45..47] x_cd, [48..50] x_c measured, [51] n_sat,
[52..54] unallocated torque FRD, [55] unallocated thrust N, [56] streamed
WholeBodyReference fresh (1 = the whole-body planner's compatible trajectory is the
law's reference; 0 = internal builder fallback / SAFETY).

#### 7.14.4 DIRECT-hold instability — measured 2026-08-23, NOT yet fixed

Flown end to end on shiqi_machine (the §7.14.1 sequence, three matched runs,
settled DIRECT hold 35-72 s, arm reference CONSTANT at home throughout):

| run | EE | alloc_thrust_coeff | pitch range | |v| max | outcome |
|---|---|---|---|---|---|
| A | gripper | 5.38192065e-05 (shipped, +15%) | -9.2 .. +9.1 deg | 0.11 m/s | bounded limit cycle, 0.88 Hz |
| B | gripper | 4.679931e-05 (calibrated) | -25 .. +31 deg | 3.4 m/s | DIVERGED, watchdog -> SAFETY at ~30 s |
| C | wrist  | 4.679931e-05 (calibrated) | -42 .. +90 deg | 18.4 m/s | DIVERGED, flew 34 m |
| D | gripper | 4.679931e-05, **whole-body planner KILLED** (node on its INTERNAL BUILDER) | -23 .. +28 deg | 4.0 m/s | DIVERGED, flipped at ~26 s |

What this rules OUT, with evidence:
- **Not the reference path — proven twice.** In the whole-body planner runs the streamed
  reference was static to **0.0 mm** (`x_cd` spread) with
  `wb_control_debug[56] = 1` throughout and a single plan event, and the ARM
  swung +-10 deg against a CONSTANT `q_d`. Run **D** then removes the whole-body planner
  entirely (killed; `Publisher count: 0` on the reference topic, so the node
  falls back to its own builder — the historical path) and it diverges just the
  same, growing steadily: pitch +-0.9 deg at t = 8-16 s, +-2.1 at 16-20,
  +-8.6 at 20-26, flipped by 26. A textbook growing unstable mode, with the
  whole-body planner not in the loop at all.
- **Not the end-effector definition.** Parking the EE back at the wrist (run C)
  made it strictly worse, so the 2026-08-23 `r_e` = gripper change is not the
  trigger.
- **0.88 Hz is the attitude loop's own frequency**: `sqrt(k_R/I) =
  sqrt(2.0/0.065) = 5.55 rad/s`. The limit cycle sits exactly on it.

What it points AT: **the +15% "stress injection" is the only reason this
configuration flies.** Believing a kf 15% higher than truth makes the allocator
command ~13% LESS motor for a given torque, i.e. it is a quiet loop-gain
reduction. Removing it (the "matched-model baseline" §7.14 recommends)
un-stabilises the hold. So the shipped yaml is left at 5.38192065e-05, and
§7.14.2's framing of that number as a robustness result is misleading — it is
load-bearing tune, not margin.

**Next step is a gain re-tune, not another fix to the whole-body planner** (run D is the
evidence for that sentence — without it the whole-body planner could not have been ruled
out): k_R/k_w and
K_y/D_y against the MATCHED kf, on the ground rig first (`utils/px4_gmo_gain_sweep.py`
is the harness, and the 2026-08-10 lesson applies — every candidate repeat-tested,
since a marginal stack's run-to-run scatter reads as success).

#### 7.14.5 The fix — M_r_d retune, and the kf injection removed (2026-08-23)

**Root cause.** `wb_mrd_x/y/z` was the T650 **I0** diagonal
`[0.077681, 0.090738, 0.083401]` — the BODY-ONLY inertia. The COUPLED
rotational inertia with the arm is `~[0.132, 0.112, 0.116]` (read straight off
`dynamics()`'s M). `M_r_d` is the inertia the law IMPOSES, so the old setting
asked the vehicle to rotate as if it were about half its real weight. That is a
~2x bandwidth demand the 99.7 ms rotor lag plus the DDS/HIL transport delay
cannot honour, and the +15% allocator-kf injection was the only thing holding
it up (it cuts delivered wrench ~13%, i.e. loop gain).

**Fix: `M_r_d` -> 1.5x I0 = `[0.116522, 0.136107, 0.125102]`, and
`alloc_thrust_coeff` back to the CALIBRATED `4.679931e-05`.** `k_R`, `k_w`,
`K_y`, `D_y`, the GMO and the model are untouched.

Screened offline first with `application/robotic_arm/utils/wb_hover_stability.py`,
which closes the same loop (exact law + exact model + allocator with the
believed-vs-true kf split + rotor lag + a transport-delay FIFO) in ~5 s per
candidate. It is scored on **DELAY MARGIN**, because that is what the flight
showed to be binding:

| M_r_d | delay margin at the calibrated kf |
|---|---|
| 1.0x I0 (old) | 24 ms |
| 1.25x | 28 ms |
| **1.5x - 2.0x** | **32 ms** |
| 2.5x | 28 ms (turns over) |

`k_R` moves it barely at all (20-24 ms across 0.5-2.0) and raising `k_w` makes
it WORSE (12 ms) — damping gain amplifies lag. `K_y`/`D_y` do not affect it.
**Read that tool's absolute verdicts with suspicion** — it does NOT reproduce
the flown +15% run (it calls it unstable; the flight held), so it is trusted
only for the RELATIVE ordering above, and the flight is the gate.

**Flown result** (matched kf, whole-body planner streaming, 78 s hold then a
0.5 m transition):

| | before (I0, +15% kf) | after (1.5x I0, calibrated kf) |
|---|---|---|
| pitch | +-9.2 deg limit cycle | **0.14 deg peak-to-peak** |
| CoM error | 18 mm mean / 55 mm max | **2.6 mm mean / 5.2 mm max** |
| EE error | 53 mm | **5.1 mm** |
| arm vs its reference | +-10 deg | **0.11 deg** |
| position spread | 55 x 31 x 36 mm | **4 x 9 x 0.3 mm** |
| thrust | - | 36.74-36.76 N (hover = 36.75) |

and at the CALIBRATED kf the old tune DIVERGED outright, so this is not a
marginal improvement — it is the difference between flying and not. The
transition (drone-GS target -> Trajectory Planning -> Send) then executed for
the first time: pitch stayed inside +-0.07 deg, CoM error <= 6 mm, and the
vehicle arrived at x = 0.480 for a 0.5 m CoM command (the ~20 mm is the
CoM-vs-base offset of the forward arm) and returned to HOLD.

ONE flight each. Per the 2026-08-10 lesson a marginal stack's run-to-run
scatter can read as success, so **repeat-test before trusting this on
hardware** — though the margin here (a decaying transient vs a divergence) is
far outside that noise band.

**THE +15% INJECTION IS KEPT (user, 2026-08-23), now as a real test.** kf is
never known exactly on hardware and ~15% is near worst case, so the rig is
REQUIRED to fly with the allocator believing the wrong number — a tune that
only works at zero mismatch is not a usable result. Before the retune the
injection was a CRUTCH (removing it made the hold diverge); after it, the rig
flies on the true model, so the same number now measures robustness instead of
providing it. `alloc_thrust_coeff` is therefore back at 5.38192065e-05 with
the retuned M_r_d.

**FLOWN AND PASSED (2026-08-23).** The retuned `M_r_d` needed NO further
tuning to carry the injection — the same value is stable at both. Steady state
over a continuous 53 s window with the allocator believing the wrong kf:

| | old M_r_d + 15% | **retuned M_r_d + 15%** | retuned, matched kf |
|---|---|---|---|
| pitch peak-to-peak | 18.4 deg (limit cycle) | **0.197 deg** | 0.14 deg |
| roll peak-to-peak | ~3 deg | **0.157 deg** | - |
| CoM error mean | 18 mm, oscillating | **2.85 mm** | 2.6 mm |
| EE error | 53 mm | **6.1 mm** | 5.1 mm |
| position spread | 55 x 31 x 36 mm | **8 x 12 x 0.5 mm** | 4 x 9 x 0.3 mm |
| joint saturation | - | none | none |

i.e. ~93x less attitude motion than the same mismatch produced before the
retune, and within a whisker of the zero-mismatch case. **The mismatch is
demonstrably ACTIVE, not bypassed**: `u1` sits at 42.26 N, and
42.26 x (4.679931/5.38192) = 36.74 N delivered — exactly hover. The allocator
is commanding 15% high and the loop absorbs it.

The entry transient is bigger than the matched case and worth expecting: CoM
error starts at ~119 mm and decays 119 -> 26 -> 5 mm over ~40 s as the observer
learns the ~13% thrust deficit. That is the disturbance estimator doing its
job, not a fault.

`wb_hover_stability.py` could NOT have predicted this: it reports 0 ms margin
at +15% for EVERY M_r_d while both flights plainly held, so its +15% branch is
wrong (the thrust-sag transient trips its divergence test). Screen with it at
the MATCHED kf only; the mismatch case is a flight question.

#### 7.14.3 The whole-body WHOLE-BODY PLANNER — paper-faithful DIRECT commanding (2026-08-23)

> **2026-09-14: the planner window now runs the C++ rclcpp node
> `whole_body_trajectory_planner` from the `fsc_trajectory_planner` package**
> (`~/Workspaces/fsc_autopilot_ws/src/fsc_trajectory_planner`; build it with
> `colcon build --packages-select fsc_trajectory_planner`). Same topics,
> services, states and operator flow as described below — only the process
> behind them changed. It needs no Pegasus checkout and plans in ~3-4 ms.
> `planner/whole_body_planner.py` no longer exists in the autopilot repo.
> See §7.15.11 for the flight that validated it on this rig.


In DIRECT the law now always receives the paper's FULL compatible reference
set — system-CoM chain through snap, base heading, EE position+heading chains,
consistent q_d — streamed as `fsc_autopilot_ros2_msgs/WholeBodyReference` on
`whole_body_direct_actuation/reference` by the **whole-body reference
whole-body planner** (`planner/whole_body_planner.py`, its own
WINDOW `whole-body planner` in the step-2 stack session — `Ctrl-b n` / `Ctrl-b 1` to
reach it; the arm-GS lamp is the primary operator surface, the window carries
the planning diagnostics). Raw GS steps never reach the law.

**Six panes is the ceiling of the stack row.** The whole-body planner was first added as
a 7th pane and tmux refused it — `no space for new pane` in the 80-col
DETACHED window the launcher creates — which under `set -e` aborted the script
*before* the vrc pane, the pane titles and the attach: a stack that looked
half-started with no way to arm. Every sibling launcher in
`scripts/isaacsim/` is at exactly 6 for the same reason. Add a WINDOW, never a
7th pane (or pass `tmux new-session -x/-y` to widen the detached window).

The operator flow after entering DIRECT (unchanged: SAFETY takeoff, settle,
gated `set_direct_mode`):

1. **HOLD** — on DIRECT entry the whole-body planner captures the current rest set
   (odometry base pose + the controller-smoothed arm reference; with the arm
   at home this IS the home-pose EE command, computed by FK in the inertial
   frame) and streams it. The vehicle holds; drone-GS sends stop executing.
2. **Drone GS send → PENDING** — the base target is captured, NOT executed,
   and republished on `whole_body_planner/pending_base` for the arm GS.
   A ride-along plan (arm pose kept) is prepared automatically.
3. **Arm GS "EE Whole-Body" tab** (NEW second tab of the inverted station;
   it resolves the whole-body planner's topics against the VEHICLE namespace by
   stripping its own last namespace component — the station runs at
   `/uav_0/fsc_open_manipulator` while the whole-body planner, a flight-stack node,
   sits at `/uav_0`. Plain relative names left the tab subscribed to
   `/uav_0/fsc_open_manipulator/whole_body_planner/*`, i.e. permanently on
   "Not in whole-body DIRECT" while the whole-body planner was three states further on;
   fixed 2026-08-23, override with `-p planner_namespace:=`). Top to bottom
   (the 2026-08-23 layout):
   * **`Joint Space`** — the Setpoints tab's table verbatim (row name + unit
     column, one column per joint) with rows **Range / Home / Solved Target**;
     Home comes from the controller's `home_position`, Solved Target is drawn
     as plain boxes with GREEN text (red when out of range) so a read-only
     value never looks like an editable field. Watch **J4**: with the position inside the drawn region,
     the usual remaining refusal is the WRIST ROLL running past ±90° to make
     the commanded heading (measured 92-99° on the failures). Rotate the
     heading dial to bring it back. Showing the pose the whole-body planner's IK SOLVED for the current
     target, green in range and **red out of it**, from the whole-body planner's
     `whole_body_planner/target_joints` (published on every plan attempt
     including the infeasible ones — that is exactly when it is needed, and
     it defaults to the HOLD pose when no target is assigned). It answers
     "which joint made this infeasible" at a glance instead of as a sentence.
   * **`Drone Target (From the Drone Ground Station)`** — 4 READ-ONLY cells
     (X/Y/Z/Yaw) mirroring what the drone GS sent, **amber while PENDING**
     (captured, not yet executed) and plain once it is the hold. The drone station owns this value; every
     relative coordinate below is anchored on it.
   * **The three selectors** — side view + bearing dial (relative to the
     drone target) + the heading dial (INERTIAL EE heading, with the drone
     target's heading as a blue rim tick). The side/top views draw the
     **whole-body usable workspace**, published by the whole-body planner on
     `whole_body_planner/workspace_rz` (a filled 192x192 (r,z) occupancy
     grid, latched, computed once at startup) — NOT the Setpoints tab's
     raster. Two reasons that raster is wrong here, both measured 2026-08-23:
     it models a PITCH wrist with a 126 mm bracket while the flying asset has
     a coaxial ROLL wrist (~10 cm of disagreement), and it knows nothing of
     the planner's fold guard, so it drew a region of which only ~2.6% could
     ever be planned. The published set is a correct NECESSARY bound —
     measured 0/40 points outside it plan — but position-only, so a target
     inside can still fail on HEADING. That was ~55% at the original ±90° q4;
     since q4 was widened to ±120° it is **98%** (measured, same samples).
     (r, z) is swept over (q2, q3) ALONE: the whole chain is left-multiplied
     by Rz(q1), so q1 cannot change radius or height (exact to 9 decimals),
     and q4 is a roll about the EE axis.
   * **`End-effector Space`, TWO rows, both editable** — `Drone Frame` (the
     drone target's yaw frame; Yaw 0 = gripper points the way the drone does)
     and `Inertial Frame` (what is actually published). Editing either rewrites the other through the anchor, so
     they cannot disagree; the views drive the relative row.
   * **Buttons** (2026-08-23 naming): `Trajectory Planning` (publishes
     `whole_body_planner/ee_target`; the whole-body planner solves IK on the
     controller's own model — limits + the certified sigma_nd >= 0.10 margin
     — and plans the COMPATIBLE transition), `Send Compatible Trajectory`
     (enabled only on PLANNED), `Clear`, and **`Go Home`** — which calls the
     whole-body planner's `whole_body_planner/go_home` and PLANS a compatible
     transition to the folded home pose. It deliberately does NOT call the arm
     controller's own Go Home: in whole-body DIRECT the law tracks the
     whole-body planner's stream, not that reference, so the arm would not move. The
     whole-body planner's `home_pose` parameter and the controller's `home_position`
     must be moved together (both `[0, 40, 40, 0]` deg today).
   * **`Status` strip at the BOTTOM** — orange `Calculating` -> green
     `Planned T=..s` / red `Infeasible` + reason. It sits under the buttons
     because it reports on what they just did.
4. **Send Compatible Trajectory** — `whole_body_planner/send`
   (std_srvs/Trigger; also the button). The planned transition streams
   sample-by-sample; on completion the goal becomes the new HOLD. `Clear`
   drops targets back to HOLD.

The transition planner (`utils_planner/transition_planner.py`, Pegasus repo)
prescribes every task channel A->B on ONE min-snap phase (min-snap is
REQUIRED: the compatible CoM's velocity depends on the prescribed jerk, so
min-jerk endpoints would step the CoM velocity at the hold joins) and solves
the CoM by the same Picard fixed point as `compatible_trajectory.py`.
Validated offline: dynamic defect 4.4e-8 m, hold-handover mismatch 0.028 mm,
all derivative chains FD-consistent to ~5e-11, IK round-trip 1e-14 rad.
Pace knobs are whole-body planner parameters (`v_max` 0.30 m/s, `a_max` 0.15 m/s^2,
`w_max` 0.30 rad/s, `t_min` 3 s).

Safety net unchanged: the whole-body planner is silent in SAFETY (takeoff/landing
byte-identical); if it dies in DIRECT the WB node falls back to its internal
setpoint builder after `wb_streamed_ref_timeout_s` (0.25 s default) — the
validated hover-campaign behaviour. While EXECUTING the whole-body planner also streams
the planned q_d to the ExternalTorqueController's `target_joint_setpoint`, so
a SAFETY abort lands on the CURRENT arm pose, not a stale one. Reverting to
SAFETY at any moment aborts everything instantly.

Build note: the message is new, so build `fsc_autopilot_ros2_msgs`
BEFORE `fsc_autopilot_ros2` after this pull, and rebuild
`utils_custom_ground_station` for the new tab. The whole-body planner itself is a plain python script — no build.
In whole-body DIRECT, tab 1 joint/EE commands are superseded by this flow
(the law does not consume the smoothed joint reference while the stream is
fresh); use tab 2.

#### 7.14.2 Thrust-coefficient mismatch: +20% fails, retuned +15% hovers

On 2026-08-22 the whole-body yaml was changed, on user request, to the same
controller-side robustness injection used by the §7.13 L1 test:

```yaml
# Isaac plant remains 4.679931e-05 with lambda = 10.0265 1/s (tau = 99.7 ms)
alloc_thrust_coeff: 5.6159172e-05  # controller belief, exactly +20%
```

The plant was not changed. At the 3.746170 kg system mass, DIRECT therefore
starts with only `mg/1.2 = 30.625 N` of physical thrust against
`mg = 36.750 N`. Successful rejection would require the controller's `u1` to settle near
`1.2 mg = 44.100 N`, with the GMO vertical estimate near `-7.350 N`.

**Guarded hover result: FAIL.** SAFETY first settled at z = 1.148 m and the
normal DIRECT gates passed; the entry anchor made the initial CoM error zero.
Within 7.68 s of DIRECT:

- altitude fell to 0.796 m before climbing through 1.385 m;
- `d_hat_z` reached -9.12 N and `u1` reached 49.06 N instead of settling;
- the observer also grew a -4.89 N x estimate, and CoM error reached 1.504 m
  (the vehicle moved to x = -1.445 m, y = -0.442 m);
- two arm joints reached the ±3.0 N·m clamp; saturation was present on 52.9%
  of DIRECT debug samples, while allocator unallocated thrust stayed below
  1.5e-14 N (the rotors themselves were not railed);
- tilt reached 30.45 degrees, where the independent test guard switched back
  to SAFETY before the controller's 40-degree watchdog. The vehicle then
  landed and disarmed normally.

The plant log confirmed `kf=4.6799e-05`, `lambda=10.0265`; the controller log
confirmed `alloc_thrust_coeff=5.61592e-05`, GMO on with `K_o=0.5`, and live
arm effort transport. This is therefore a whole-body/GMO robustness failure,
not a missing motor lag, coefficient-injection error, DDS dropout, or rotor
allocation rail. That value was a historical test point and is no longer the
shipped configuration.

**Retuned +15% result: PASS for hover only.** The controller now uses
`alloc_thrust_coeff=5.38192065e-05` while Isaac remains at the calibrated
`4.679931e-05` with the measured `10.0265 1/s` motor pole. The physical thrust
deficit on DIRECT entry is 13.04% (`mg/1.15`); successful rejection requires
`u1=42.262 N` and `d_hat_z=-5.512 N`. The hover tune is:

```yaml
wb_k_x: 16.0
wb_k_v: 12.0
wb_k_r: 2.0
wb_k_w: 1.5
wb_ky_x: 2.0
wb_ky_y: 2.0
wb_ky_z: 2.0
wb_dy_x: 4.0
wb_dy_y: 4.0
wb_dy_z: 4.0
wb_ko_t: 0.5
wb_ko_r: 0.1
wb_ko_q: 0.1
wb_dls_lambda: 0.3
wb_posture_kp: 2.0
wb_posture_kd: 0.25
wb_posture_ki: 0.05
wb_posture_i_max: 0.8
```

> **The four `wb_posture_*` keys above no longer exist (removed 2026-09-05).**
> They are kept in this block because they are what this campaign flew. The
> joint-posture PID was an implementation addition, not part of the published
> law, and it was deleted from `wb_controller.cpp` along with its parameters —
> see §7.14.6. Copying this block into a current yaml is harmless (rclcpp
> ignores undeclared keys) but they will do nothing.

The Cartesian EE objective alone admitted an alternate IK branch during the
large entry sag. The optional joint-posture PID objective uses the same
controller-smoothed `q_d/qdot_d`, is transformed through `u3` so the base gets
the matching arm-reaction compensation, and clamps its integral torque. Zero
posture gains preserve the parity-locked original Python law. A fixed
`Kp=8, Kd=0.6` and a 20 s ramp to that gain were both rejected because the
world-fixed EE task and home posture fought during the entry sag, causing
rotor/joint saturation; the slow integral removes the steady branch error
without adding that stiffness.

Two independent clean-start runs completed 90 s DIRECT soaks, then returned
to SAFETY, landed, and disarmed:

| metric | run 1 | run 2 |
|---|---:|---:|
| steady CoM error RMS (last 20 s) | 2.03 mm | 3.27 mm |
| maximum tilt | 1.20 deg | 1.55 deg |
| steady `u1` | 42.2619 N | 42.2627 N |
| steady `d_hat_z` | -5.5119 N | -5.5123 N |
| peak arm torque | 0.778 N.m | 0.780 N.m |
| motor / arm saturation | none | none |
| final arm q [deg] | [0.05, 37.28, 39.16, 0.02] | [0.04, 37.64, 38.85, 0.02] |
| final-20-s worst home error | 4.24 deg | 4.31 deg |

The arm error decreased monotonically after the transient and had not quite
finished integrating at 90 s; it stayed on the requested `[0, 40, 40, 0] deg`
branch. This validation covers a constant hover reference only. Do not infer
position-step or end-effector-trajectory robustness under the +15% mismatch.
Restore `4.679931e-05` for the matched baseline and for any hardware-oriented
configuration.

---

#### 7.14.6 Deleting the joint-posture PID breaks DIRECT — measured, then reverted (2026-09-05)

The term is an implementation addition, not part of the published law:

```cpp
posture_i_torque_ += posture_ki * (ref.q_d - x.q) * dt;          // clamped
u3 += ramp * (posture_kp * (ref.q_d - x.q)
              + posture_kd * (ref.qdot_d - x.qdot)) + posture_i_torque_;
```

It was removed, flown, and **put back**, because without it this rig does not
hold DIRECT. Seven 75–90 s soaks on the §7.14.1 sequence, one variable at a
time, nothing else touched:

| # | thrust mismatch | posture PID | K_y / D_y | outcome |
|---|---|---|---|---|
| A | +15% | removed | 2 / 4 | **diverged**, abort 32 s |
| B | +15% | **present** | 2 / 4 | **stable, 75 s** |
| C | matched | removed | 2 / 4 | stable, 75 s |
| D | +7.5% | removed | 2 / 4 | **diverged**, abort 15 s |
| E | +15% | removed | 20 / 9 | **diverged**, abort 3.8 s |
| F | matched | removed | 2 / 4 | **diverged**, abort 17 s — *repeat of C* |
| G | matched | removed | 2 / 4 | **diverged**, abort 3.7 s — *repeat of C* |

**C, F and G are the same configuration.** One survived, two did not. Matched-kf
without the term is **1 of 3**, i.e. marginal, not stable — the 2026-08-10
run-to-run-scatter lesson exactly (this stack is wall-clock PX4 + DDS, not
deterministic; a single completed run proves nothing). Reading C alone would
have shipped an unstable configuration, and briefly did.

| metric, DIRECT soak | A (+15%, no PID) | B (+15%, PID) |
|---|---|---|
| duration [s] | 32.3 (abort) | 75.0 |
| CoM err, last 20% [mm] | 530.4 | **2.9** |
| CoM err max [mm] | 1494.7 | 197.8 |
| EE err, last 20% [mm] | 708.4 | **6.5** |
| \|e_R\| max | 0.9599 | **0.0664** |
| tilt, last 20% [deg] | 12.843 | **0.040** |
| tilt max [deg] | 32.12 | **2.37** |
| peak \|tau_j\| [N·m] | 3.00 (clamp) | **0.74** |
| samples at the clamp [%] | **51.1** | **0.0** |
| final q [deg] | [−6.4, −18.0, −6.2, 26.6] | [0.0, 38.8, 39.7, 0.0] |

**Mechanism, read off run A's first five seconds.** The arm leaves home
immediately: q2 spans −10.8…50.0° and q3 −77.6…50.0°, hitting the +50° stop at
t = 0.8 s and crossing into negative q2/q3, this asset's **elbow-singular
branch**. J_3y degrades, the DLS solve asks for torque the servos cannot give,
all four joints rail, and the arm's reaction takes the base. Run B's same
window: q2 31.0…42.7°, q3 38.8…43.2°. The 51% clamp figure is the same
signature the +20% kf failure produced on 2026-08-22.

**Why the EE task cannot substitute, and why stiffening it is worse.** At
`K_y` = 2 N/m a 200 mm task error is 0.4 N of restoring force — the arm is
effectively free. `K_y` was softened to 2 in the same +15% campaign that added
the PID, so **the two are a compensating pair and were only ever validated
together**; every configuration of the Pegasus in-process law (`controller.py`)
that flies with no posture anchor uses `K_y` 8–200. But undoing both together
does not work: run E, `K_y` 2→20 / `D_y` 4→9, was the **worst** run of the
seven. A stiff task on this laggy plant is its own instability.

**State of the tree: the term is back, and so is the +15% injection.** Both
yamls carry a DO-NOT-DELETE note at `wb_dls_lambda`, and the sim yaml's
`alloc_thrust_coeff` block records that the two must move together. What
removal actually needs is a retune with mismatch margin — `M_r_d`, the GMO
bandwidths, `K_y`/`D_y`, and probably a gentler DIRECT-entry — not a one-line
edit. `WbParityTest` 4/4, `WbReferenceBuilderTest` 2/2 both before and after.

**If the published law is what a result claims, this is a declared deviation,
not a hidden one.** In steady flight the term is worth 0.004 N·m against a
0.77 N·m gravity load, 0.5%, at the flown 0.11° arm error — small, but it is
what keeps q on its branch through the entry transient, and it is added
straight to `u3` with no null-space projection (there is no null space: the
4-DOF task has isolated but non-unique solutions, `[0, 40, 40, 0]°` and
`[0, 128.7, −108.4, 0]°` agreeing on the EE pose to 4.7e-16).

**Reproducing.** `docs/docs_aerial_manipulator/posture_removal_20260905/` holds
the driver and the per-run metrics. The driver publishes ONLY
`PositionControllerReference` — the whole-body planner owns `WholeBodyReference`, and a
second publisher on it invalidates the test. Full step-0 clean between every
run; PX4 stays armed otherwise and the next takeoff produces no lift.

---

#### 7.14.7 One arm reference, one planner per mode (2026-09-05)

The torque controller no longer plans. `external_torque_controller` tracks a
single reference topic and generates nothing of its own; exactly one node
publishes that topic at any instant, chosen by flight mode.

| mode | publisher | silent |
|---|---|---|
| SAFETY (and the bench, no flight stack) | `arm_planner` (new) | `whole_body_planner` |
| whole-body DIRECT | `whole_body_planner` | `arm_planner` |

Topic: `…/external_torque_controller/reference_joint_trajectory`,
`trajectory_msgs/JointTrajectory`, 100 Hz. Live check —

```bash
ros2 topic info -v /uav_0/fsc_open_manipulator/external_torque_controller/reference_joint_trajectory
# Publisher count: 2   (arm_planner @ /uav_0/fsc_open_manipulator,
#                       whole_body_planner @ /uav_0)
# Subscription count: 1 (external_torque_controller)
ros2 topic hz  <same topic>     # 100 Hz in either mode, from one source
```

**What it replaced.** The min-jerk generator lived in `TorqueControllerBase`,
so in DIRECT it ran in series with the whole-body planner's B-spline plan: the whole-body planner
poked `target_joint_setpoint` and the controller re-profiled it. The 09-02 and
09-03 flights measured the arm replaying the whole-body planner's move **3–4 s late**
with the endpoints agreeing exactly.

**The ground station is unchanged.** It still solves end-effector targets to
joint angles itself and drives `<ctrl>/target_joint_setpoint` and
`<ctrl>/go_home` — the PLANNER now serves those at the same names, because the
controller no longer advertises them. In DIRECT the planner **refuses** both
(it does not queue them) and says why.

**Blast radius is one config.** `TorqueControllerBase` gained
`external_reference_topic` (default `""`) and `external_reference_timeout`
(0.2 s); only `torque_controller_isaac_aerial.yaml` sets them. Empty keeps the
internal generator, so `ComputedTorqueController`, `PositionController` and
every bench/hardware workflow are byte-identical and need no re-validation. In
pure-tracking mode there is no activation homing (homing is a plan) and a stale
stream **freezes** the last reference — nothing here may invent a trajectory.

**The reference fans out; it does not loop through the controller.** The torque
controller and the whole-body node both subscribe to `reference_joint_trajectory`
directly. The flight node used to read `smoothed_reference_joint_trajectory`
instead, i.e. a controller emitting a reference and a flight node consuming a
controller's output — the wrong direction, and what hid the second generator.
That topic keeps its name and type and is now **controller telemetry only**: its
`effort` is the commanded torque the GS plots; its position/velocity is an echo.
Check it with `ros2 topic info -v`: the reference has 2 publishers (one always
quiet) and 2 subscribers; the echo has 1 and 1.

**The sine sweep is not available on this rig** — its generator stayed in the controller
deliberately (its ramp continuity guard is subtle, and copying it would put a
second generator back). The planner logs a throttled refusal rather than
swallowing the command; run sweeps on a bench/hardware config.

**Validated.** Loopback 9/9
(`open_manipulator_x_custom_controller/test/test_arm_planner.py` —
100 Hz in SAFETY, seeds on the measured pose, tracks a GS target at ≤3.75 mrad
per sample, 0 messages in DIRECT, target and go_home refused there, resumes on
the measured pose after revert). Planner tests 3/3, with `arm syncs 550`
against 546 reference samples — the full stream rate, where it used to be a
separate 20 Hz timer. Full stack, 60 s DIRECT soak, against the §7.14.6
baseline:

| | this split | baseline |
|---|---|---|
| CoM err, last 20% | 3.26 mm | 2.98 |
| EE err, last 20% | 8.84 mm | 5.57 |
| \|e_R\| max | 0.0671 | 0.0682 |
| tilt, last 20% | 0.034° | 0.042 |
| peak \|tau_j\| | 0.740 N·m | 0.745 |
| at clamp | 0.00% | 0.00 |
| u1, last 20% | 42.26 N | 42.263 |
| stream fresh | 100% | 100 |

and the planner logged `going SILENT` on DIRECT entry and `owns the arm
reference again` on the revert. One flight each — repeat before hardware.

---

## Notes

- `CLAUDE.md` refers to `scripts/indoor_sim/start_aerial_manipulator.sh` with
  `direct` / `px4-offboard` modes. **That file does not exist in this repo** —
  the six launchers in §1 are the real entrypoints.
- `02_aerial_manipulator_free.py`'s header mentions `utils/gain_sweep.py`; the
  only sweep harness actually present is `utils/px4_gmo_gain_sweep.py` (§4),
  which targets the PX4 rig.
- Isaac's `python.sh` resets `PYTHONPATH`, so a bare-module import plus
  `PYTHONPATH` does **not** survive into the process. The demos use
  fully-qualified `fsc_aerial_manipulation.*` imports for this reason — keep any
  new module inside the editable-installed package rather than adding a path hack.

### 7.15 AM-T650 WHOLE-BODY + L1-ADAPTIVE augmented observer — added 2026-09-06

> **THE STANDING SIM PLANT CHANGED ON 2026-09-14 (user request).** This rig now
> flies the mismatch campaign's **config A** by default: the calibration
> report's **gearbox friction at ×1.05** against an arm-side compensation that
> pays ×1.00, and **arm link masses ×1.05** while every model keeps the nominal
> arm — a 5 % under-compensation of the two corrections the report calls
> dominant, for robustness validation before the flight test. It is set in both
> `_sim` yamls (`sim_arm_friction_scale` / `_width` / `sim_arm_mass_scale`) and
> the launcher prints it in red at every start. Flown 3× and stable: §7.15.14.
>
> **Every flight number in §7.15.4 – §7.15.12 was flown on a FRICTIONLESS arm and
> is not comparable with a run from here on** — that includes the C++-planner and
> EE-trajectory flights of §7.15.11/12 (2026-09-14/15), flown from a checkout that
> did not yet carry the friction plant. (§7.15.13 is a mock-hardware bench test,
> no flight numbers.) The frictionless baseline is
> `posture_ablation_20260909/l1_mission_kx32.npz`;
> `PEGASUS_ARM_FRICTION_SCALE=0 PEGASUS_ARM_MASS_SCALE=1.0 <launcher>`
> reproduces that plant for an A/B.

**This is §7.14's rig with one thing swapped: the disturbance observer.** Same
coupled airframe+arm law, same gains, same plant, same SAFETY/DIRECT split,
same gates and watchdog, same torque-mode arm stack, same two ground stations,
same service namespace. The augmentation lives **entirely inside whole-body
DIRECT** — in SAFETY the law is held (`hold = true`) and the observer is held
reset exactly as the GMO is, so takeoff and abort are byte-identical to §7.14.

What that means concretely, and how to check it rather than take it on trust:

| | §7.14 (GMO) | §7.15 (L1) |
|---|---|---|
| client class | `WholeBodyDirectActuationClient` | **the same object**, second `main()` |
| the law | `WholeBodyController::step()` | **the same function** |
| DIRECT controller name | "Whole-Body Direct Actuation" | identical |
| mode service | `fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode` | identical |
| executable | `autopilot_whole_body_direct_actuation_node` | `autopilot_whole_body_l1_direct_actuation_node` |
| yaml | `..._whole_body_direct_actuation_t650_sim.yaml` | `..._whole_body_l1_direct_actuation_t650_sim.yaml` |
| estimator | `d_hat = K_o (p − p_hat)` | deadbeat PWC + `C(s)` + attribution |

The yamls differ by **`vehicle_name` and the `wb_l1_*` block, nothing else** —
diff them before every campaign or the comparison is not one:

```bash
cd ~/ros2_ws/src/fsc_autopilot_ros2/config && diff \
  <(grep -vE '^\s*#|^\s*$' params_single_aerial_manipulator_whole_body_direct_actuation_t650_sim.yaml) \
  <(grep -vE '^\s*#|^\s*$' params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml)
```

And in flight, `wb_control_debug[57]` says which observer owns the tick.
Measured over the four 2026-09-06 flights: L1 active on **27540 of 27540**
DIRECT samples and **0** SAFETY samples in every L1 run, 0 everywhere in the
GMO run. The estimator cannot run outside whole-body DIRECT by construction —
it is gated by the same `gmo_active = use_gmo && !hold && !gmo_inhibit` the
GMO is.

**Why swap the observer at all.** The whole-body law consumes a disturbance
estimate in exactly three places — `−d_hat_t` in `f_d`, `−d_hat_r` in `u_2`,
and `F_hat_y` in `u_3`. The GMO supplies all three from one proportional law,
and that costs two things:

- **One gain for two jobs.** `d_hat = K_o(p − p_hat)` closed on its own
  predictor IS `K_o/(s+K_o)` applied to the true residual, so `K_o` sets
  estimate accuracy AND loop robustness together. That is why §7.14's `K_o` is
  pinned at 0.5/0.1/0.1 — 1.0 on the body channels crashed the rig. The L1 law
  splits them: a **deadbeat** piecewise-constant inversion whose accuracy is the
  sample period alone, then an explicit `C(s) = ω_c/(s+ω_c)` that alone decides
  robustness. `ω_c` is the direct analogue of `K_o`.
- **No attribution.** A momentum residual measures only the SUM `d + d_e`, so
  the GMO's lumped `F_hat_y = (J_y^#)^T d_hat` hands `u_3` the internal
  disturbance as a **phantom contact force** — the impedance law renders
  compliance against a force the environment never applied. The repair is the
  four directions no wrench can reach (`N(J_e)`, the arm self-motions, where
  `Z_0^T J_e^T = 0` exactly, so no contact flag is needed anywhere).

Design note: `disturbance_observer_draft.tex` ("Decompose the lumped
disturbances into end-effector and orthogonal components", 2026-08-27).
Implementation and every measured number: **`docs/docs_aerial_manipulator/
L1 Augmented Disturbance Observer.md`**, Python reference
`extensions/.../utils_controller/l1_observer.py` (run it for its self-test),
parity-locked to 1e-8 by `WbL1ParityTest`.

#### 7.15.1 Run sequence — copy-paste, per machine

> **Since 2026-09-14 step 1 must also build `fsc_trajectory_planner`** (the
> C++ whole-body planner the step-2 stack launches in its `planner` window):
> `cd ~/Workspaces/fsc_autopilot_ws && colcon build --packages-select
> fsc_trajectory_planner fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF`.
> Nothing else in this sequence changes.


Identical to §7.14.1 except the two launcher names. Every trap in §7.14.1
applies verbatim: never chain step 0's lines with a launcher, use bracketed
`pgrep` patterns when checking by hand, the NODE reads params only at startup
(restart the autopilot pane after a yaml edit), two ground stations, `Ctrl-b d`
to detach.

**fsc_lab_machine** (lab desktop, user `fsc-jupiter`):

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. build after every pull (any terminal — the cd IS part of the command)
cd ~/Workspaces/fsc_autopilot_ws && colcon build --packages-select fsc_autopilot_ros2 --cmake-args -DBUILD_TESTING=OFF
#    ...and the arm stack (arm_planner carries the SAFETY guard's arm fold)
cd ~/Source/Shiqi/fsc_om_ws && colcon build --packages-select open_manipulator_x_custom_controller

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 3. Pegasus / PX4 SITL + TORQUE-MODE ARM STACK + ARM GROUND STATION (terminal 2)
~/Source/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_direct_actuation_sitl.sh fsc_lab_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}

# 5. take off in SAFETY from the ground station, settle at the hover
#    reference, THEN hand the vehicle to the whole-body law (terminal 3).
#    SAME service as §7.14 — the mode namespace is shared on purpose.
ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: true}"

# ABORT back to SAFETY — have this line ready BEFORE entering DIRECT.
# Same effect as the drone GS's "Return to baseline" and the automatic
# 20 deg tilt watchdog: hover where it is, fold the arm home (SAFETY GUARD below)
ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: false}"

# 6. PX4 refuses an in-air disarm: land by reference first, then
ros2 service call /uav_0/rc/disarm std_srvs/srv/Trigger {}
```

**shiqi_machine** (shiqi-desktop) — identical apart from the repo roots:

```bash
# 0. clean slate            (any terminal — run BOTH lines, in this order)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 2. ROS 2 stack            (terminal 1 — must start FIRST, owns the agent)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 3. Pegasus / PX4 SITL + TORQUE-MODE ARM STACK + ARM GROUND STATION (terminal 2)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_direct_actuation_sitl.sh shiqi_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

Add `cd ~/ros2_ws && colcon build --packages-select fsc_autopilot_ros2
--cmake-args -DBUILD_TESTING=OFF` and `cd ~/colcon_ws && colcon build
--packages-select open_manipulator_x_custom_controller` as step 1 after a pull.
Steps 5-6 (DIRECT entry, the abort line, disarm) are pure service calls with no
paths in them — use the fsc_lab_machine block above verbatim.

**SAFETY GUARD — every way out of whole-body DIRECT hovers in place and folds
the arm home (2026-09-11).** Four things revert DIRECT → SAFETY, and all four now
end in the same state:

| trigger | how |
|---|---|
| **tilt watchdog** — automatic | body tilt > **20°** (`system_wd_max_tilt_deg`, was 40) |
| body-rate watchdog — automatic | \|ω\| > 360 °/s (`system_wd_max_rate_dps`, unchanged) |
| **sideways-drift watchdog** — automatic | horizontal CoM tracking error > **0.75 m** (`system_wd_max_drift_m`, new) |
| feedback-loss failsafe — automatic | odometry / EV fusion lost > 1 s (then it also LANDS) |
| drone GS **"Return to baseline"** / Controller tab | `activate_controller("Baseline (Safety)")` |
| the ABORT line above | `set_direct_mode false` |

The tilt is `acos(R22)`, the angle of the body z axis off vertical, and that is
never less than `max(|roll|, |pitch|)`: it trips when **roll OR pitch** passes
20°, and a little earlier when both are large together. Nominal flights on this
rig peak at 6–10° (the DIRECT entry); every divergence recorded here went past
20° on its way out.

**The drift it measures is `|x_c − x_cd|_xy`** — the law's own horizontal CoM
tracking error, against **the reference the law is actually flying** (the
whole-body planner's `x_cd`, or the anchored builder reference), NOT against a
fixed point and NOT against the raw ground-station setpoint. That distinction is
what makes the limit usable: the planner moves the base by half a metre on a
normal step leg, and none of that counts as drift — only the law failing to
follow does. Measured peaks on this rig: **410 mm** at the DIRECT entry (the
handover transient, §7.15.8) and **349 mm** through the step and trajectory
legs, so 0.75 m is ~1.8× the worst nominal excursion. It shares the tilt/rate
watchdog's latch, so one trip is one revert.

What happens, in order:

1. **The flight node** switches to SAFETY, i.e. PX4 attitude + rate + mixer, and
   re-seeds the position reference to **where the vehicle is at that instant**,
   with the yaw it has then. The vehicle hovers there. No ground-station click is
   needed, and none is re-sent: the drone GS publishes a setpoint only when you
   press send, so it cannot drag the vehicle back to an old target.
2. **The node publishes the new mode immediately** (it used to wait for its
   500 ms info tick), so the whole-body planner goes silent at once.
3. **`arm_planner`** takes the arm reference back, holds the **measured** pose for
   `home_on_revert_delay_s` = 1.0 s so PX4 has regained attitude before the arm's
   reaction torque joins in, then folds the arm to home `[0, 40, 40, 0]°` on its
   ordinary min-jerk profile (≤ 0.2 rad/s per joint, ≥ 1.5 s). Any arm command
   sent in that 1 s (arm GS joint target, `go_home`) cancels the automatic fold.

What you see:

```
autopilot pane:  DIRECT WATCHDOG TRIPPED: excess tilt (tilt 20.3/20.0 deg, rate …). Reverting to SAFETY.
   (or)          DIRECT WATCHDOG TRIPPED: excess drift (horizontal CoM error 0.78/0.75 m, [+0.12 -0.77]). Reverting to SAFETY.
                 Reference re-seeded to current pose: [x y z] m, yaw … deg
arm stack pane:  SAFETY revert: folding the arm HOME in 1.0 s (home_on_safety_revert). …
                 Folding the arm HOME over 2.6 s.
```

The first line appears only for a watchdog trip. The other three show for every
revert. A watchdog trip **latches**: the vehicle stays in SAFETY until you
deliberately call `set_direct_mode true` again, and that call still has to pass
the DIRECT-entry gates.

Where the settings live, so you can check them rather than take them on trust:

- `system_wd_max_tilt_deg: 20.0` and `system_wd_max_drift_m: 0.75` —
  `params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml`.
  The node reads them only at startup, so restart the autopilot pane after
  editing. **The GMO twin (§7.14) has neither** (40°, and the drift check
  defaults to 0 = off) — deliberately: this work is L1-only.
- `system_arm_go_home_service` / `system_arm_go_home_delay_s` — the node's own
  way to fold the arm, for a stack with **no `arm_planner`** (i.e. hardware).
  Empty in the sim yaml, because `arm_planner` owns that edge there, so the arm
  is never commanded twice.
- `home_on_safety_revert: true`, `home_on_revert_delay_s: 1.0` —
  `open_manipulator_x_isaac_bridge/config/torque_controller_isaac_aerial.yaml`,
  the `arm_planner` block. That yaml is shared by the §7.14 and §7.15 rigs, so
  the fold applies to both. It is **off by default** in the node, so bench and
  hardware configs are unchanged.

**Validated 2026-09-11, 3 Isaac flights on this rig** (full §7.15 `_sim` plant,
fsc_lab_machine). Before DIRECT the arm was moved to `[15, 26, 32, 23]°`, off
home, so the fold would show. **After the revert the driver published NO
reference**, so every metre of hover-in-place below is the node's own
re-seed:

| flight | revert | vertical, peak | horizontal, peak → last 5 s | tilt after | arm within 2° of home |
|---|---|---|---|---|---|
| A — GS `activate_controller("Baseline (Safety)")` | 20 ms after the call | +454 mm | 394 → **28 mm** | 6.97° | **2.40 s** (final err 0.41°) |
| control — A with `home_on_safety_revert: false` | 20 ms | +468 mm | 72 → **8 mm** | 2.36° | never (stays at the pre-revert pose, as designed) |
| B — tilt watchdog, temporarily set to 3° | **at 3.13°**, 1.44 s into DIRECT | +146 mm | 632 → **30 mm** | 11.2° | **2.44 s** (final err 0.43°) |

What the three flights separate:

- **The vertical climb is not the guard.** It is the same with and without the
  fold (454 vs 468 mm). This is the DIRECT→SAFETY handover surge that §7.13 (c)
  and the AM-L1 retune already document. The yaml's allocator believes kf is
  +17.6% (`alloc_thrust_coeff` vs the plant). In DIRECT the SAFETY UDE is fed the
  allocator's *believed* collective, so it books that gap as a ~7 N
  disturbance, and SAFETY then acts on it. This is **sim-only**: on hardware a
  real kf error lives in the SAFETY model too. It is smaller in flight B only
  because B spent 1.4 s in DIRECT, which gave the UDE less time to wind up.
- **The fold does add a sideways drift, and that part is real physics.**
  Horizontal peak goes 72 → 394 mm, along world −y. It starts with a +6° roll
  spike exactly inside the fold window, 1.0–3.4 s. Swinging the arm-yaw and
  wrist joints back to 0 moves the CoM sideways; PX4's integrator re-learns
  the new trim and the vehicle walks back. It is back inside 50 mm by 15 s,
  without any reference from anyone.
  In flight B the larger +x motion (597 mm) is mostly the DIRECT-entry
  transient's own momentum: it was already pitched 8° when the watchdog
  fired.
- Nominal DIRECT stayed at **6.9–7.3° peak tilt**, so 20° leaves ~3× margin.
  Set the limit below ~8° and the normal DIRECT entry will trip it, which is
  what flight B did on purpose.

Knobs, if the sideways walk matters more than getting the arm in fast:
`home_on_revert_delay_s` (fold later, after SAFETY has settled) and the
`arm_planner` `max_velocity` (fold slower, so PX4's integrator keeps up).
Neither has been flown yet. The loopback test
(`open_manipulator_x_custom_controller/test/test_arm_planner.py`, no Isaac)
checks the fold, its 1 s delay, its smoothness, and that an operator target
cancels it.

**The drift guard, validated 2026-09-11, 2 more flights:**

| flight | result |
|---|---|
| the FULL 10-leg standard mission on the shipped guard (20°, 0.75 m) | **no false trip**: all 10 legs flew in DIRECT and it left only when asked, 170 s of DIRECT, peak CoM error **410 mm** (entry) / 349 mm (legs) / 207 mm late, tilt 6.0° p-p, 0 saturations |
| drift limit temporarily **0.15 m** to force a trip | tripped **1.44 s into DIRECT at only 7.0° tilt** — so tilt cannot explain it — then hovered (settled 21 mm) and the arm folded home in **2.40 s**, final error 0.46° |

That pair is the point of the 0.75 m number: the mission's own worst excursion
is 410 mm, and a deliberately tight limit proves the path fires, reverts, hovers
and folds. A trip prints `excess drift` with both components, so a log says
which way it went.

**§7.15.1's HARDWARE twin (created 2026-09-11, NEVER FLOWN).** The guard also
ships on the experiment side, as a new pair beside the GMO hardware stack:

```bash
# hardware (companion computer) — the L1 whole-body stack, guard included
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/indoor_exp/start_whole_body_l1_direct_actuation_stack_t650_aerial_manipulator.sh uav_0
```

- yaml `params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650.yaml`
  takes its **plant, allocator and SAFETY map from the GMO hardware file**
  (bench kf 4.540431e-05, bench km, the re-derived thrust pair, the measured
  `wb_base_com_y`, the arm sign map, `straight_line`) and its **law + observer
  from the L1 sim file** (k_x 32/k_v 20, K_y 20/D_y 12, psi 0.3, no posture PID
  but `wb_ee_anchor_com` + `wb_u3_internal_ff`, the whole `wb_l1_*` block), plus
  the guard at 20° / 0.75 m. Every `sim_plant_*`/`sim_arm_*` key and the kf
  injection are dropped.
- The launcher greps for the **L1 sim file's** values (its own kf injection, the
  sim plant keys, `ude_height_threshold 0.35`, all three vehicle_name variants)
  and refuses to start on a hit; it then prints positive confirmation of the
  bench kf, the thrust pair, the observer type, the two compensation keys and
  both guard limits. It keeps the shared hardware tmux session name on purpose,
  so starting it kills any other hardware stack.
- Verified by loading that yaml on the node under `/uav_test`: all four guard
  params read back, `AM-T650-WB-L1-HW`, the magenta L1-observer banner, a green
  `joint-posture PID is OFF`, `wb_posture_kp` "Parameter not set", and the
  model-vs-`vehicle_mass` cross-check passed.
- **The arm fold on hardware goes through the node, not `arm_planner`**:
  `system_arm_go_home_service` calls the arm controller's own `go_home` one
  second after the revert. **BLOCKER, pre-existing and not introduced here:**
  `wb_arm_reference_topic` expects `reference_joint_trajectory`, which on
  hardware nobody publishes in SAFETY (the 2026-09-05 split moved generation
  into `arm_planner`, and the hardware arm launch does not start it — the
  controller publishes `~/smoothed_reference_joint_trajectory` instead). The
  DIRECT-entry gate checks that topic, so entry should be refused until either
  `arm_planner` runs with the hardware arm stack (set `external_reference_topic`
  in `external_torque_controller_hardware_aerial_pwm.yaml` — the sim topology,
  which also gives the fold) or the key is repointed. Both the yaml header and
  the launcher header say so.

**CONFIRM THE PLANT TOO (2026-09-14).** The Pegasus launcher prints, in red,
`GEARBOX FRICTION ACTIVE on the plant: x1.05 ... arm-side friction
compensation true` and `ARM GRAVITY MISMATCH ACTIVE: arm link masses x1.05`,
and Isaac repeats both with the per-joint numbers. That pair is the standing
config A; their absence means you are flying the pre-2026-09-14 frictionless
arm, which is a different plant (§7.15.14).

**CONFIRM THE OBSERVER BEFORE YOU FLY.** The autopilot pane prints a magenta
banner at startup; **if it is absent you are flying the GMO**, because the
yaml, the node name and the service are otherwise indistinguishable:

```
DISTURBANCE OBSERVER: L1 ADAPTIVE (not the GMO). A_s=-[2.00 2.00 2.00]
Ts=0.0000s omega_c=[2.00 0.50 0.50] omega_i=2.00 omega_x=20.0
L_c var=[100 0.25 0.0025] attribution=ON (attributed F_y)
bounds=[20.0 N, 2.00 N.m, 1.50 N.m | 15.0 N, 3.00 N.m]
```

The drone GS shows Vehicle **`AM-T650-WB-L1`**. The two launchers refuse to
start against each other's node (the executable names are substring-disjoint,
so each `pgrep -f` guard matches exactly one fork), and the Pegasus launcher
says so by name when it finds the wrong one.

Session name `fsc_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack`
(step 0's `stop_isaacsim_stack.sh` auto-discovers it).

#### 7.15.2 Debug array — §7.14's 57 elements plus 32

`wb_control_debug` is append-only, so **[0..56] are exactly §7.14's** and every
existing analysis still works. **Read by index, never by length** — SAFETY
publishes a shorter prefix. In DIRECT it is 89 elements:

```
[57]      L1 active (0 while the GMO runs, or in SAFETY)
[58..61]  F_hat_y CONSUMED by u3 — lumped under the GMO, attributed under L1.
          ONE SLOT, BOTH PATHS: this is what makes the phantom-force
          comparison like for like. In free flight the true interaction
          wrench is exactly zero, so every newton here is fictitious.
[62..71]  d_hat^c, the UNFILTERED deadbeat estimate (10). This is what the
          note's 2*L_Sigma*T_s bound applies to and what Steps 2-4 consume;
          [31..40] above is the FILTERED signal the control loops see.
[72..77]  w_hat_e, the estimated interaction wrench (EE frame, N / N.m)
[78..87]  w_hat, the estimated internal disturbance (original coordinates)
[88]      estimator outputs on a bound this tick — should read 0
```

Healthy L1 hover on the shipped config, from the 2026-09-06 flights:
`[57] = 1`, `[88] = 0`, `[17] u1 = 47.56 N` (against a 36.75 N nominal hover —
the injections are real), `[31..40] d_hat_z = −10.81 N`,
`[78..87] w_hat_thrust = −10.75 N` (**99.5 %** of the residual booked to the
collective), `[72..77] w_e = 0.07 N / 0.02 N·m` (correct — nothing is touching
the arm), `[58..61] |F_hat_y| = 0.05 N`.

#### 7.15.3 The two places the design note leaves a choice

Both were measured, both changed the implementation, both are recorded here
because someone reading the note will otherwise reimplement the wrong one.

**(a) The note's `Φ` is not the deadbeat gain of the predictor that can
actually be run.** The `(h + u)` term must be plain Euler — the PLANT
integrates it exactly and any other weight leaves a `(weight − dt)(h + u)`
bias, ~0.15 N on the z channel at hover. Over the `N = Ts/dt` sub-steps of one
adaptation interval the exact gain is therefore

```
Phi_d = dt (I − e^{A_s Ts}) (I − e^{A_s dt})^-1        (diagonal)
```

which equals the note's `Φ` in the fine-sub-step limit and `dt` at one-tick
adaptation. Using `Φ` instead measures a REAL DC error of `a·Ts`: **0.8 % at
`A_s = 2`, 6.9 % at `A_s = 20`**; with `Phi_d` it is 2.5e-13.

*Corollary that saves a wasted sweep:* at one-tick adaptation the deadbeat
cancellation is exact and **`A_s` drops out of the estimate entirely** — it
only weights the average within an interval. `wb_l1_a_*` and
`wb_l1_adapt_period_s` are **one trade, not two**, and in noise-free simulation
`A_s` is inert. Sweeping it alone will measure nothing.

**(b) The note's minimum-norm `L_c = B_a G^+` makes the phantom force WORSE on
this plant.** The collective's coupling into the wrench-free rows is
`a_3 = [0, −0.002, −0.103, −0.001]` — metres of EE lift per joint radian —
against the joint rows' exact `I_4`. Minimum norm therefore compares **newtons
of collective against newton-metres of joint torque**, and books a 5.5 N thrust
deficit as joint torque:

| `L_c` metric | `theta_hat_f` (true −5.50 N) | phantom `\|F_hat_y\|` |
|---|---|---|
| no attribution at all | — | 0.890 |
| minimum-norm (the note) | **−0.041** | **4.000** — 4.5x WORSE |
| prior-variance (100, 0.25, 0.0025) | **−4.903** | **0.440** |

Generalized to `L_c = B_a W G^T (G W G^T)^-1` with a diagonal prior variance.
**Every property the note's proof uses survives for any `W ≻ 0`**:
`Z_0^T L_c = I_4`, the range stays inside `R(B_a)` so the lateral rows stay
frozen, and the error dynamics stay stable with the Lyapunov function taken in
the `W^-1` metric. `W = 1,1,1` recovers the note's own choice and is the CODE
default; the yaml ships `(100, 0.25, 0.0025)` — "a 10 N thrust error is as
plausible as a 0.5 N·m body moment or a 0.05 N·m joint torque".

**(c) The note's persistency-of-excitation remedy does not work here** — worth
knowing before anyone designs an excitation trajectory for it. A vigorous 60 s
arm sweep (q1 ±30°, fold ±25°, wrist ±60°) gives an averaged projector with
eigenvalues `[4e-4, 0.017, 0.051, 0.097, 0.89, 0.95, 1.00, 1.00]`, so the weak
directions converge 10–2500x slower than `ω_i`; **400 s of sweeping moved the
collective estimate from −0.04 to −0.30 N** of a true −5.5. The metric in (b)
is what actually fixes it. `ω_i` changes only how fast the fixed point is
reached, not where it is (identical from 0.2 to 10 rad/s).

#### 7.15.4 The ω_c sweep — 4 flights, 2026-09-06, none aborted

Same plant and same injections as §7.14.2's stress configuration, so the
estimator has real work: **thrust loss** (+15 % allocator kf), **model
uncertainty** (plant mass and inertia ×1.10, CoM shift 10/10/5 mm) and **motor
delay** (rotor lag λ = 10.0265 1/s) — together **10.8 N** it must find before
the vehicle holds station. `u1 = 47.559 N` and `d_hat_z = −10.809 N` are
identical to three decimals across runs, which is the check that the plant
really was the same in each.

| `ω_c` | rise90 | entry sag | recovery | steady CoM | \|e_R\| | tilt p-p | **phantom F_y** |
|---|---|---|---|---|---|---|---|
| **GMO** `K_o` .5/.1/.1 | 2.86 s | 463 mm | 49.3 s | 4.59 mm | .0885 | 0.108° | **1.884 N** |
| L1, = `K_o` | 3.28 s | 428 mm | 49.2 s | 3.17 mm | .0934 | 0.337° | **0.065 N** |
| **L1, 2 / 0.5 / 0.5** | 1.38 s | 187 mm | 21.0 s | **1.65 mm** | **.0059** | 0.564° | **0.055 N** |
| L1, 6 / 2 / 1 | 0.73 s | 69 mm | never | 52.0 mm | .2615 | 22.24° | 2.045 N |

**At matched bandwidth the two estimators are indistinguishable, and must be** —
both reach `f_d` and `u_2` through the same first-order filter. That is the
sanity check that the swap changed nothing it should not have, not a result.

**The phantom force is a separate axis and it collapses 29x at no cost.** Three
numbers show it is the mechanism rather than a coincidence: `w_hat_thrust` =
−10.753 of −10.81 (the null channel booked **99.5 %** of the residual to the
COLLECTIVE, which is what the (b) metric was for), `w_e_hat` = 0.074 N
(correct — nothing is touching the arm), and `σ(d_hat^c)` 0.657 N against
`σ(d_hat_f)` 0.0021 N (the deadbeat estimate really is a noisy 250 Hz momentum
difference and the filter really does remove it — the two-layer split working).

**`ω_c` = 2 / 0.5 / 0.5 is the shipped winner**, 4–5x the GMO's ceiling, better
on every metric with zero saturation.

**`ω_c` = 6 / 2 / 1 is past the limit and fails as the theory says it should**:
the best ENTRY of the four (0.73 s rise, 69 mm sag — the estimate takes up the
mismatch almost immediately), then a **22° peak-to-peak limit cycle** with
`d_hat_z` swinging −5..−20 N onto its own bound and 10.4 % of samples on the
joint clamp. It did not abort (the tilt stayed under the 35° envelope and the
bounds held it) but it is not a flying configuration. 6 rad/s sits on the
10.03 rad/s MN4010 rotor-lag pole with DDS transport on top — the same
mechanism that capped the GMO at `K_o = 1.0`.

**Not resolved, and the obvious next sweep:** that run raised all three channel
groups together, so which one binds is unknown. The limit is between
(2, 0.5, 0.5) and (6, 2, 1), and the translational channel — the one carrying
the 10.8 N — may well tolerate more than the rotational one.

**Not covered by this campaign**, stated rather than implied:

- **No contact in any flight.** That is what makes the phantom force
  measurable, but it means the TRUE wrench estimate was never exercised against
  a real one. `w_e_hat` reading ~0.07 N when the answer is 0 is necessary, not
  sufficient.
- **No arm motion**, so the PE question was not flown — though §7.15.3(c)
  measured offline that it would not have helped.
- **`sim_arm_backemf_enable` was false**, so there was no joint-space
  disturbance at all. **`wb_l1_lc_var_q = 0.0025` is the one shipped number
  this campaign did not test**: it tells the observer a joint-torque error is
  implausible, which is exactly wrong once the back-EMF droop is on. RAISE IT
  with the droop.
- **One flight per configuration.** Near a stability boundary on this rig that
  proves nothing (the 2026-08-10 run-to-run-scatter lesson). Repeat the
  2 / 0.5 / 0.5 point before trusting it on hardware.
- **Never flown on hardware.** There is no hardware yaml for this rig yet, on
  purpose.

#### 7.15.5 The standard test — 1 m hover, x/y/yaw steps, compatible trajectory

The §7.15.4 sweep flew a HOVER SOAK only, because the phantom force is
measurable in free flight and a hold is the cleanest place to read it. The
rig's normal test is more than that: hover at **1 m**, step **x**, **y** and
**yaw**, then a **compatible-trajectory** leg. `wb_l1_campaign_driver.py` flies
it by default (`--no-steps` reproduces the sweep's hover-only mission):

```
offboard -> arm -> SAFETY climb to 1 m -> settle -> DIRECT -> 20 s hold
  -> step x +/-0.5 m, out and back      (drone-GS target)
  -> step y +/-0.5 m, out and back
  -> step yaw +/-30 deg, out and back
  -> compatible trajectory: an inertial EE target, +0.15 m x / -0.15 m z
     and back                            (arm GS tab-2 path)
  -> hold -> SAFETY abort -> descend
```

**A STEP IS NOT A REFERENCE PUBLISH IN WHOLE-BODY DIRECT.** The whole-body
planner captures a drone-GS target as PENDING and never executes it; the
transition planner solves a compatible trajectory, reports PLANNED, and only an
explicit Send starts it. Every leg therefore runs
`target -> PLANNED -> Send -> EXECUTING -> HOLD`, and an INFEASIBLE goal is
logged with the planner's reason and skipped rather than hung on.

**Two traps in driving that handshake, both of which produce a
plausible-looking log with the steps silently not happening** (found on the
first flight of this mission, 2026-09-06):

- **Do not stream `position_controller/reference` at full rate in DIRECT.**
  The planner's unchanged-target guard is only active while it is already
  PENDING/CALCULATING/PLANNED/EXECUTING — **from HOLD an unchanged target is
  accepted and re-plans.** A 50 Hz stream of the same setpoint therefore drags
  it back out of HOLD on the very next tick after any leg completes. Publish
  on CHANGE in DIRECT; SAFETY still needs the full-rate stream.
- **Wait for a FRESH `PLANNED`, and for `EXECUTING` before `HOLD`.** The
  planner is already PLANNED when a leg starts (from the above, or from the
  previous leg), and HOLD is the state the leg starts in. Without both checks
  a leg Sends the previous leg's plan, sees it finish instantly, and records
  success with the vehicle never moving.

**FLOWN 2026-09-06 — five attempts, four completed, and the last one flew every
leg.** The x/y/yaw steps are the transition planner's own min-snap profile, so
these are TRACKING numbers for a compatible trajectory, not step responses.
Run E (`l1_stepsE.npz`), all eight legs, no refusals, no abort:

| leg | peak CoM err | settled | max tilt | duration |
|---|---|---|---|---|
| step x +0.5 m | 248 mm | 55.2 mm | 2.42° | 11.7 s |
| step x −0.5 m | 268 mm | 62.5 mm | 2.96° | 11.5 s |
| step y +0.5 m | 203 mm | 49.8 mm | 2.21° | 11.7 s |
| step y −0.5 m | 222 mm | 52.8 mm | 2.28° | 11.6 s |
| step yaw +30° | 30 mm | 6.7 mm | 0.80° | 9.4 s |
| step yaw −30° | 9.4 mm | 0.9 mm | 0.13° | 9.5 s |
| **compatible trajectory, EE −6 cm** | **9.8 mm** | **0.74 mm** | **0.17°** | 9.5 s |
| compatible trajectory, back | 7.2 mm | 2.7 mm | 0.17° | 9.5 s |

`u1` 47.57 N, `d_hat_z` −10.814 N, `w_hat_thrust` −10.738 N, zero allocator
saturation, `tau_max` 0.74 of 3.0 N·m, nothing on an estimator bound, phantom
`|F_hat_y|` 0.107 N. Runs C, D and E agree on `u1` / `d_hat_z` /
`w_hat_thrust` to three decimals, which is the check that the plant and the
injections were identical across them.

**THE COMPATIBLE-TRAJECTORY LEG IS THE BEST-TRACKED MOTION IN THE MISSION** —
9.8 mm peak, 0.74 mm settled, 0.17° of tilt while the arm moves 6 cm and the
base counter-moves. That is the expected ordering, not luck: it is the only
leg whose reference is dynamically consistent by construction, so the law is
not fighting anything.

**Translation is the expensive axis; yaw is nearly free.** 0.5 m peaks at
~250 mm of CoM error and settles to ~55 mm; a 30° yaw peaks at 30 mm and
settles to 6.7 mm. The ~55 mm residual after a translation is the same
structural `T·e_R/K_p` offset the hover case has — the attitude loop is pure P
and the position loop pure P/D — not a tuning failure.

**THE EE WORKSPACE AROUND THE FOLDED HOME IS SMALL AND LOPSIDED, and both
obvious step directions are outside it.** Two flights were spent finding this;
the planner refused each in ~0.1 s with a readable reason, which is the system
working:

| EE step from home | \|target\| | planner's verdict |
|---|---|---|
| out + down 0.15 m | 0.480 m | `INFEASIBLE: IK did not converge` |
| in + up 0.05 m | 0.215 m | `INFEASIBLE: joint limits: q3 = 63.0 deg vs 50` |
| **down 0.06 m** | 0.280 m | **q = [0, 27.1, 38.8, 0] deg — flies** |

Reaching OUT exceeds the arm: at the folded home the gripper already sits
0.26 m from the body origin, which is its own reach (0.16 m wrist at β = 80°
plus the 0.108 m gripper offset). Retracting FOLDS it further, and q3 is
already at 40° of a 50° stop. **Down is the direction with room**, because it
unfolds. Map it before guessing —
`transition_planner.ik_position_azimuth` from `q_home` over a grid takes
seconds and the planner also publishes the true reachable set on
`whole_body_planner/workspace_rz`.

**ONE ATTEMPT IN FIVE ABORTED 8.4 s AFTER DIRECT ENTRY, and it is the LAW —
not the observer, the altitude or the mission.** Run A, same
`ω_c` = 2/0.5/0.5, same 1.0 m:

| t−t0 | x_c | err | u1 | d_hat_z | tilt |
|---|---|---|---|---|---|
| 0.0 s | [0.041, 0.004, 0.967] | 0 mm | 36.6 N | 0.0 N | 0.1° |
| 2.0 s | [0.133, 0.102, **0.777**] | 232 mm | 53.0 N | −12.7 N | 7.2° |
| 4.0 s | [0.659, 0.495, 0.971] | 790 mm | 47.0 N | −10.1 N | 9.2° |
| 8.3 s | [**−1.368**, −0.287, 0.836] | 1444 mm | 41.7 N | −12.5 N | 16.1° |

The ALTITUDE recovered (0.777 → 0.971 m by t = 4 s — the 19 cm entry sag
`ω_c` = 2 gives). What diverged is a **growing lateral oscillation**, with the
arm pinned: `|tau_joint| = 3.000 N·m` (the clamp) and **21.4 % of DIRECT
samples with the allocator saturated**, against 0.74 N·m and zero saturation
on every completed run. Runs B–E then entered DIRECT at the same 1.0 m and
flew, so **the 1 m hover is not the cause** — this is the run-to-run scatter
§7.14.6 records for this rig (same config, 1 pass in 3 there). The estimator
was live for 100 % of DIRECT and the streamed reference fresh 96.6 %, so
neither the observer swap nor the mission logic is implicated. Data:
`l1_steps_A_aborted.npz`.

#### 7.15.6 Automated campaign — one command per data point

Params are read at controller STARTUP and PX4 never disarms this rig, so every
gain change needs a full relaunch. That is what the harness is for:

```bash
# one data point, clean slate to npz  (gmo | l1)
~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_tune_cycle.sh l1 mytag shiqi_machine

# edit a gain between runs — unknown keys are an error, comments preserved
/usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_set_gains.py --show
/usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_set_gains.py omega_c_t=4.0 decompose=false

# the whole sweep back to back (~7 min per flight; restores the yaml on exit)
~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_campaign.sh shiqi_machine

# score and plot
/usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_metrics.py \
    ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/l1_observer_20260906/*.npz
PYTHONNOUSERSITE=1 /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_plot.py \
    out.png ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/l1_observer_20260906/*.npz
```

The cycle script cleans up at the **start** of each run, not the end, so the
last run's stack is left up — that is normal, and step 0 above clears it.

Three traps the harness encodes, each of which cost a run to find:

- **`vehicle_status` never publishes on this PX4 v1.16 / px4_msgs release/1.16
  pairing — it is `vehicle_status_v1`.** Subscribing to the un-suffixed name
  strands a driver in its WAIT phase with no error at all.
- **Gate arming on `estimator_status_flags`, never on
  `vehicle_status.pre_flight_checks_pass`** — that field is false on this rig
  even while it is armed and flying.
- **`PYTHONNOUSERSITE=1` for plotting**: `~/.local` carries numpy 2.2.6 against
  an apt matplotlib built for numpy 1.x, so a plain `python3` dies at import
  with "numpy.core.multiarray failed to import".

#### 7.15.7 Law audit, the posture term, and the arm's back-EMF — 2026-09-06

Three questions, in the order they have to be answered: is the shipped L1 law
the manuscript's law, does it still fly if the one extra term is removed, and
what happens when the arm stops being an ideal torque source.

##### (a) What is actually in `u3` — a line-by-line audit

`WholeBodyController::step()` is shared by both observers, so the audit covers
§7.14 as well. Against the manuscript:

| term | in the law | status |
|---|---|---|
| `f_d = −k_x e_x − k_v e_vx + m ẍ_cd + mg e_3 − d̂_t`, `u1 = f_d·(R_0e_3)` | yes | published |
| `u2 = M_r(…) + C_r ω_0 + C_rp ρ − d̂_r` | yes | published |
| geometric `R_0c` chain with two levels of feedforward | yes | published |
| impedance inner `Λ_y(J̇_y ξ − ÿ_d) + ΛM_y^{-1}(D_y e_vy + K_y e_y) − (ΛM_y^{-1} − I)F̂_y` | yes | published |
| coupling feedforward `J_1y F_trans + J_2y τ_rot` | yes | published |
| `u3 = −J_3y^† (inner) − C_rp^T ω_0 + C_p ρ` | yes | published |
| DLS regularisation of `J_3y^†`, `λ = 0.3` | — | numerics, not a force term; `λ = 0` recovers the plain solve |
| saturation-consistent rebuild of `u2` after the `τ_max` clamp | — | actuator bookkeeping; a no-op when nothing saturates |
| **joint-posture PID added to `u3`** | **no** | **the one extra term** |

So there is exactly one. It is gain-gated, and the node now says which way it
is configured at startup rather than leaving it to be inferred:

```
LAW CHECK: joint-posture PID is OFF (all gains 0) -- u3 carries no term
           outside the published law.
```

Zero `wb_posture_kp/kd/ki/i_max` and that line appears in green; leave any of
them non-zero and it prints in yellow naming the values. Grep the controller
pane for `LAW CHECK` before trusting any run's provenance.

##### (b) Removing it — flown twice, aborted twice

| run | outcome | peak CoM err | peak tilt | joints on the 3.0 N·m clamp |
|---|---|---|---|---|
| L1, `wb_posture_* = 0`, A | **abort 7.7 s** | 1313 mm | 13.2° | 32.4% |
| L1, `wb_posture_* = 0`, B | **abort 8.0 s** | 1408 mm | 36.1° | 46.9% |
| L1, term restored | completed 121 s, 8 legs | — | 8.7° | 0.0% |

**The failure is kinematic, not estimation.** Arm trace from run A, from the
DIRECT edge (the joint-state broadcaster's order on this rig is `[q2, q3, q1,
q4]`, not `joint1..4` — at the ground pose it reads `(40, 40, 0, 0)` for the
home `[0, 40, 40, 0]`):

| t | q1 | q2 | q3 | q4 | tilt |
|---|---|---|---|---|---|
| 0.0 s | 0.0 | 40.1 | 39.9 | 0.0 | 0.07° |
| 1.0 s | −5.0 | 43.1 | **50.0** | −4.7 | 7.31° |
| 3.0 s | **−35.0** | −25.4 | 46.9 | 87.2 | 8.45° |
| 4.0 s | **−35.0** | 50.0 | **−64.7** | 57.9 | 8.14° |
| 7.0 s | 19.2 | 49.6 | −55.0 | −41.3 | 15.18° |

q3 reaches its +50° stop at t = 1 s, then crosses zero into **negative q3**,
this asset's elbow-singular branch, and q1 rails at −35°. `J_3y` degrades, the
DLS solve asks for torque the servos cannot deliver, all four rail, and the
reaction takes the base.

**This is the same failure §7.14.6 recorded for the GMO, and the L1's
attribution does not prevent it — which is the informative part.** The two
observers fail for different reasons and only one of them is an observer
problem:

- *The GMO's* is attribution. It reports the lumped residual, `u3` receives it
  as a phantom contact force (**1.79 N** measured in free flight today), the
  impedance law yields to it, and the arm drifts off home.
- *What remains after that is fixed* is that the 4-DOF EE task has **isolated
  but non-unique** solutions — `[0, 40, 40, 0]` and `[0, 128.7, −108.4, 0]`
  deg agree on the EE pose to 4.7e-16 — so no task-space law selects a branch
  at all. The L1 cuts the phantom force to **0.06–0.16 N**, a 11–30x
  reduction, and still cannot select a branch, because nothing in the task
  can.

**Verdict: keep it, declare it.** In steady flight it contributes 0.004 of
0.77 N·m (0.5%) at the flown 0.11° arm error, added straight to `u3` with no
null-space projection (there is none to project onto). It is a branch guard,
not a control gain. The proper fix is an explicit branch guard or a joint
limit in the planner, not a bigger gain — stiffening the EE task instead was
already measured to be *worse* (§7.14.6, abort at 3.8 s). Both yamls carry
this reasoning at `wb_posture_kp`.

##### (c) The arm's back-EMF droop — both observers fail, 4 runs of 4

`sim_arm_backemf_enable: true` puts the OM-X servos' real PWM-mode behaviour
into the plant: `τ_app = clip(τ_cmd) − b·q̇`, `b = [0, 0.9337, 1.4934, 0]` N·m
per rad/s (the identification is in CLAUDE.md; joints 1 and 4 sit below the
current sensor's noise floor and are left at 0). The controller does not model
it — that mismatch **is** the test.

| observer | droop | runs | outcome |
|---|---|---|---|
| GMO | off | 2 | completed, 122.6 s and 138.0 s of DIRECT, 8 legs |
| L1 | off | 2 | completed, 121.3 s and 136.4 s of DIRECT, 8 legs |
| **GMO** | **on** | **2** | **abort 9.7 s / 10.8 s** |
| **L1** | **on** | **2** | **abort 5.8 s / 5.8 s** |

| run | DIRECT | peak tilt | peak CoM err | peak τ | clamp % |
|---|---|---|---|---|---|
| gmo, droop on, A | 9.7 s | 20.6° | 2126 mm | 3.00 | 4.9% |
| gmo, droop on, B | 10.8 s | 24.5° | 2407 mm | 3.00 | 9.2% |
| l1, droop on, A | 5.8 s | 32.3° | 1450 mm | 2.45 | 0.0% |
| l1, droop on, B | 5.8 s | 31.0° | 1387 mm | 2.45 | 0.0% |
| gmo, droop off | 122.6 s | 8.4° | 1307 mm | 0.73 | 0.0% |
| l1, droop off | 121.3 s | 8.7° | 796 mm | 0.83 | 0.0% |

Repeats agree to a few percent in every column, so this is not the run-to-run
scatter §7.14.6 records.

**Neither observer compensates it, and neither should be expected to.** Three
reasons, in increasing order of how fundamental they are:

1. **It is a gain error, not an additive disturbance.** `−b q̇` is a feedback
   path, so an additive estimate would have to track it at the arm's own
   bandwidth. `b/I ≈ 75 1/s` at the 0.02 kg·m² armature; the observers' arm
   channel runs at `ω_c = 0.5` rad/s (L1) or `K_o = 0.1` (GMO), 150–750x
   slower.
2. **The damage is on the BASE, not the arm.** The law commands `τ = T^T u`,
   so the rotors pre-compensate the arm's reaction through `N_1^T u_3`. If the
   arm delivers less than `u_3`, the base is being compensated for a reaction
   that never arrives — this is the failure CLAUDE.md predicted in the servo
   model's own entry ("the law's `N_1^T u_3` arm-reaction pre-compensation is
   exact only if the arm delivers the modelled torque").
3. **It is a positive feedback around the entry transient.** DIRECT entry on
   this rig has a large, normal excursion (peak CoM error 0.80 m on L1, 1.31 m
   on GMO, both recovering in ~8 s with the droop off). That transient moves
   the arm; the moving arm loses torque; the lost torque disturbs the base;
   the base moves the arm more. Measured on `gmo_emf1`: the droop reaches
   0.09 N·m at t = 1 s when the tilt is already 5.5°, then **1.70 N·m** by
   t = 7 s — over half the joint clamp — as the loop winds up.

**The two observers fail differently, and the difference is diagnostic.** The
GMO takes 10 s and rails the joint clamp (4.9–9.2% of samples) before drifting
2.1–2.4 m. The L1 takes 5.8 s, reaches a *higher* tilt (31–32°), and **never
touches the clamp** (peak τ 2.45 N·m). The L1's arm channel is the faster of
the two, so it responds to the growing residual sooner and drives the base
harder before the joints saturate. Faster estimation is not helping here — the
residual it is chasing is not a disturbance it can cancel.

**What this does NOT say.** It does not say the whole-body law cannot fly a
PWM-mode arm; it says the *present tune* cannot, at this droop, entering DIRECT
through this transient. Untested and worth trying before concluding anything
stronger: a gentler DIRECT entry (the transient is the trigger); modelling `b`
in the law's arm channel, which is a one-line feedforward `+b q̇` given that
`b` is identified to ±5% with no fitted parameter; or the real fix on hardware,
which is **current-control mode (Mode 0)** — the servo then closes its own
current loop, `R` and `K_e` drop out, `b → 0`, and this model should be deleted
rather than compensated.

##### (d) GMO vs L1, leg by leg — the comparison table

Both observers, same plant, same mission, same driver, ideal arm (droop off),
**16 s hold after each leg**. Figure: `l1_final_20260906/compare_gmo_l1.png`
(CoM tracking, CoM error, EE task error, phantom force — each run against its
own reference). Data: `gmo_settle.npz` / `l1_settle.npz`. Scored by
`application/robotic_arm/utils/wb_compare_metrics.py`.

**READ THE HOLD LENGTH BEFORE READING THE NUMBERS.** An earlier pair of runs
used the driver's default 6 s hold and made the L1 look *worse* — 65–72 mm
"settled" against the GMO's 6–21 mm. It was an artifact: this rig has a slow,
lightly damped position mode, so at 6 s neither observer has settled and the
2 s average lands at whatever phase it lands at. At 16 s both settle and the
ordering inverts. Do not compare settled errors between runs with different
`--hold-between`.

| leg | GMO peak | **L1 peak** | GMO rms | **L1 rms** | GMO settled | **L1 settled** |
|---|---|---|---|---|---|---|
| step x +0.5 m | 296.4 | **192.7** | 96.4 | **60.7** | 6.8 | **2.6** |
| step x −0.5 m | 250.5 | **209.4** | 80.1 | **66.1** | 5.0 | **3.0** |
| step y +0.5 m | 266.5 | **237.6** | 85.9 | **76.4** | 2.5 | 5.1 |
| step y −0.5 m | 267.5 | **222.3** | 84.6 | **69.8** | 4.9 | 4.9 |
| step yaw +30° | 7.6 | 7.8 | 4.5 | **4.0** | 1.4 | 3.2 |
| step yaw −30° | 9.2 | **8.1** | 4.4 | **4.0** | 2.8 | **1.8** |
| **compatible traj, out** | 63.8 | **47.1** | 38.7 | **17.7** | 14.7 | **2.6** |
| **compatible traj, back** | 56.9 | **34.6** | 33.4 | **13.2** | 18.9 | **1.4** |

All figures are CoM tracking error in mm. End-effector task error, same runs:

| leg | GMO peak / settled | **L1 peak / settled** |
|---|---|---|
| step x +0.5 m | 297.8 / 16.3 | **195.8 / 15.9** |
| step y +0.5 m | 270.9 / 5.7 | **244.9 / 8.4** |
| step yaw +30° | 62.6 / 3.0 | **50.1 / 4.6** |
| **compatible traj, out** | 91.9 / 20.6 | 110.4 / **10.7** |
| **compatible traj, back** | 110.8 / 19.8 | 122.0 / **5.1** |

And the two numbers that are not close:

| | GMO | L1 | ratio |
|---|---|---|---|
| phantom `\|F̂_y\|`, per leg | 1.35 – 1.89 N | **0.044 – 0.132 N** | **13–40x** |
| DIRECT-entry peak CoM error (3 runs each) | 1273 / 1273 / 1305 mm | **796 / 800 / 854 mm** | **1.6x** |
| entry recovery to < 50 mm | 37.9 / 38.6 / 39.3 s | **7.1 / 7.6 / 7.7 s** | **5x** |

**What the table says, stated no more strongly than it supports.**

1. **The compatible trajectory is where the L1 wins clearly** — 2.4–2.9x lower
   rms and 6–13x lower settled error, on both legs, with the EE task error
   settling 2–4x tighter. That is the leg where the arm and the base move
   together, i.e. where a *lumped* estimate is most wrong, so this is the
   expected ordering rather than a surprise. It is also the only leg whose
   reference is dynamically consistent by construction.
2. **The entry transient is the most reproducible difference** — three runs
   each, agreeing to 4%, and the L1 recovers **5x faster**. This is the `ω_c`
   vs `K_o` bandwidth split doing exactly what it is for: the L1's translational
   channel runs at `ω_c = 2` where the GMO's `K_o = 0.5` is the most it can
   carry, because in the GMO one gain buys both accuracy and robustness.
3. **On x/y steps the L1 is 6–30% better on peak and rms and the settled
   errors are a wash** (2.5–6.8 mm GMO, 2.6–5.1 mm L1, i.e. inside the spread).
   Do not claim more than that from single runs.
4. **On yaw the two are indistinguishable** — peaks under 10 mm either way. A
   30° yaw barely moves the CoM on this airframe, so the leg has little to
   discriminate.
5. **The phantom force is not a tracking metric and should not be read as one.**
   It is the correctness of the *estimate*: in free flight the true contact
   wrench is exactly zero, so the GMO's steady 1.35–1.89 N is entirely
   fictitious force that the impedance law renders compliance against. The L1's
   0.044–0.132 N is the same quantity computed correctly. Its practical
   consequence is §(b): it is why the GMO drifts off the home posture, and it
   is *not* sufficient to make the posture term unnecessary.

**Two honest caveats.** One run per configuration at 16 s hold, on a rig whose
run-to-run scatter §7.14.6 documents — the entry numbers are backed by three
runs each and are safe; individual leg numbers are not. And the L1's entry is
the *noisier* of the two in this particular pair (a ±0.4 m oscillation during
the first 30 s, visible in the figure, with phantom-force chatter to 3 N)
even though its peak and recovery are better; that oscillation was absent in
`l1_v2`, so it is scatter in the transient, not a property.

#### 7.15.8 Tuning campaign — the J1/J4 ripple and the slow takeoff, 2026-09-06

Two user-reported symptoms, 16 flights, data
`docs/docs_aerial_manipulator/l1_tune_20260906/`. Both fixed; **two parameters
changed, nothing else, all three disturbance sources left active** (+15%
allocator kf, plant mass/inertia x1.10 with a 10/10/5 mm CoM shift, MN4010
rotor lag). Back-EMF stays off per the request.

| | before | after |
|---|---|---|
| `ude_height_threshold` | 0.4 | **0.35** |
| `wb_ky_psi` / `wb_dy_psi` | 1.0 / 1.0 | **0.3 / 0.3** |

##### (a) The slow SAFETY takeoff — a UDE gate below the resting height

Commanding 1 m and arming, the vehicle took ~28 s to get there. Measured
altitude trace: it creeps at **0.003 m/s for 22 s**, crosses 0.40 m, and only
then climbs at 0.08-0.22 m/s.

The cause is arithmetic. `sim_plant_mass_scale: 1.10` makes the plant
4.120787 kg while the controller keeps `vehicle_mass: 3.746170`, so the
gravity feedforward is **3.67 N short**. A constant force error can only be
supplied by the UDE, `ude_height_threshold` gates the UDE off below 0.40 m,
and the vehicle **rests at 0.305 m** — it cannot reach the gate that enables
the thing it needs to climb. This is the bare-T650 L1 deadlock (§7.13.3 run C)
in slow motion, and the `ude_height_threshold` comment in the yaml already
warned about it; the value had simply never been checked against this rig's
resting height.

**Not fixed by correcting the mass** — that would delete the model-uncertainty
disturbance under test.

**The value is bounded on both sides, and the obvious choice is unsafe.** Above
0.305 m or the UDE integrates the ground reaction while seated (the failure
that produces "zero lift with no error message"). 0.32 m flies, but across
**33 recorded runs the seated peak reaches 0.3338 m** during the arming
transient, so 0.32 is crossed while still on the ground in **29 of 33** — a
one-run test would have shipped it. 0.35 clears that peak by 16 mm and is
crossed while seated in **0 of 33**.

| | reach 0.95 m |
|---|---|
| gate 0.40 (before) | 28.2 s |
| gate 0.35 (after, run A / run B) | **9.6 s / 9.2 s** |

##### (b) The J1/J4 torque ripple — the EE heading channel's loop gain

In the settled hold q1 and q4 carry **~0 mean torque** (-0.000, -0.004 N·m)
while q2/q3 hold the gravity load (0.707, 0.242) — yet q1/q4 carry all the
ripple, ~70% of it in a **3.90 Hz** line. Traced into the task: the EE
**heading** row is the channel oscillating at 3.90 Hz, and heading is exactly
the (q1, q4) pair, which is why those two joints and no others.

**On the full mission the shipped gains drove q4 across its entire ±3.0 N·m
clamp** — 6.00 N·m peak to peak. The hover-only test never showed that,
because only the trajectory leg's 60° heading sweep exercises the channel.

| | τ₁ std | τ₄ std | τ₄ p-p | peak arm τ | clamped |
|---|---|---|---|---|---|
| before (`l1_settle`) | 0.2242 | 0.4833 | **6.0000** | 3.00 | 0.3% |
| after, run A | 0.0039 | 0.0041 | 0.0623 | 0.85 | **0.0%** |
| after, run B | 0.0035 | 0.0039 | 0.0623 | 0.83 | **0.0%** |

A **96x** reduction in q4 p-p, and the arm comes off the clamp entirely.
**Heading tracking improved** (`e_psi` std 1.17e-3 → 5.6e-4 rad), so this was
a self-excited oscillation, not stiffness worth having.

**The ripple is L1-SPECIFIC, which is why the GMO yaml does NOT carry this
change.** Measured both ways: at `ky/dy_psi` 1.0 the GMO's own q4 ripple is
0.0042 N·m std against the L1's 0.4833, and lowering it to 0.3 moves the GMO
to 0.0046 — nothing. What differs is what feeds that task row. The GMO's
`F_hat_y` is the smooth lumped `(J_y^#)^T d_hat` off a heavily filtered
estimate; the L1's is the attributed `Λ_y S_e Λ_e⁻¹ ŵ_e`, rebuilt every tick
from a deadbeat estimate, and the heading row is the least-filtered path it
takes. The best-against-best pair was flown with the GMO at 0.3 for symmetry;
since the change is inert on that rig its numbers stand either way.
**`ude_height_threshold` 0.35 IS mirrored into the GMO yaml** — that one helps
both (GMO takeoff 29.7 → 15.5 s), because the mass mismatch and the resting
height are shared.

**It is NOT an anomalous `M_Y_psi`.** The first reading of this — that
`wb_my_psi: 0.05` amplifies the heading row 20x against the 1.0 of x/y/z — is
a units error, kg·m² against kg. The amplification each row actually receives
is `Λ_y M_y⁻¹ = [0.485, 1.320, 0.778, 0.404]`, so the heading row is
unremarkable. What is too high is that channel's **loop gain** against the arm
dynamics plus the external 250 Hz DDS loop. Four independent routes all remove
the line; `ky/dy_psi = 0.3` measured best on the trajectory legs and peak tilt:

| route | τ₄ std | e_psi std | tail CoM |
|---|---|---|---|
| shipped 1.0 | 0.0379 | 1.17e-3 | 2.5 mm |
| `my_psi` 0.3 | 0.0028 | 5.47e-4 | 2.0 mm |
| `my_psi` 1.0 | 0.0023 | 6.66e-4 | 1.8 mm |
| **`ky/dy_psi` 0.3** | **0.0014** | 5.60e-4 | 2.3 mm |
| `dls_lambda` 0.6 | 0.0026 | 6.17e-4 | 2.4 mm |

##### (c) Observer bandwidth is NOT the lever, and the rotational channel binds

The 2026-09-06 sweep moved all three `ω_c` groups together (2/0.5/0.5 →
6/2/1) and recorded "6 is past the limit" without knowing which channel did
it. Moving one group at a time settles it — **it is the rotational channel,
and 0.5 is already its ceiling**:

| ω_c (t / r / q) | entry peak | tail | tilt | clamp |
|---|---|---|---|---|
| **2 / 0.5 / 0.5** (shipped) | 841 mm | 2.7 mm | 8.5° | 0% |
| 2 / **1.5** / 0.5 | 1656 mm | **279 mm** | 34.0° | 4.5% |
| 2 / **3.0** / 0.5 | — | — | 42.3° | 24.8%, **abort 12 s** |
| **4** / 0.5 / 0.5 | 854 mm | 2.1 mm | 10.3° | 0% |
| 2 / 0.5 / **1.5** | 818 mm | 2.2 mm | 9.7° | 0% |

Translational and arm changes do essentially nothing, so **the entry transient
is not observer-limited** and no `ω_c` value will fix it. It is the
SAFETY→DIRECT handover: SAFETY's UDE has already found the ~10.8 N of
mismatch and DIRECT's observer restarts from zero. Closing that is a
**seeding** change (hand the L1 the UDE's estimate at the mode switch), not a
gain change — untried, and it is a code change, not a parameter.

##### (d) What did not change

Entry peak 854 → 847/844 mm, tail CoM 2.1 → 2.0/3.5 mm, |e_R| 0.325 →
0.334/0.329, per-leg step and trajectory tracking within run-to-run spread.
Both fixes are additive wins; neither trades anything measurable.

#### 7.15.9 The joint-posture term — mechanism and two paper-consistent fixes (2026-09-09)

Full write-up: `Removing the Joint-Posture Term.md`. Data: `posture_ablation_20260909/`.
Offline reproduction: `application/robotic_arm/utils/wb_entry_sim.py`
(`--validate --ablate --sweep --fixes`, `--case posture=0,ee_ref=relative,int_ff=true,g.ky=20,g.dy=12`).

Two mechanisms, both from the manuscript's `d_e` = contact wrench only: (1) the
DIRECT-entry base transient (0.8–1.3 m) makes the world-anchored EE reference
unreachable (workspace ~5 cm), the arm hits its stops and crosses the elbow
branch; (2) an internal disturbance is rendered as a phantom task force,
0.22 N per N of vertical deficit at home, and the arm walks off home by
`F/K_y` (5% model: 214 mm measured, 203 predicted). No gain group fixes either.

Two default-off options in the whole-body fork, parity 9/9 unchanged:
`wb_ee_anchor_com` (EE reference held about the actual CoM in free flight) and
`wb_u3_internal_ff` (coupling feedforward carries `u + d_hat`). Flown at the
full injection with `wb_posture_* = 0`, `K_y` 20 / `D_y` 12:

| run | vehicle | peak e_x | peak e_y | settled e_y | clamp | arm end |
|---|---|---|---|---|---|---|
| fix_ab_A/B/C | 3/3 completed | 848–919 mm | 29–31 mm | 1.2 mm | 0% | [0, 40, 41, −1]° |
| fix_ab50_A (K_y 50/20) | completed | 858 | 15 | 0.5 | 0% | [0, 40, 40, −1]° |
| fix_a_A (anchor only) | completed | 798 | 114 | 102.7 (predicted 102.7) | 0% | [−5, 18, 37, 1]° |
| fix_b_A (ff only, K_y 2) | completed, arm lost | 903 | 880 | 221 | 68% | at the stops |
| 2026-09-06, PID off | abort 7.7 / 8.0 s | 1313 / 1408 | 1.2 m | — | 32 / 47% | negative branch |
| ffw_A/B (source = `T^-T w_hat`) | 2/2 completed | 855–894 | 30–31 | 3.1 / 3.4 | 0% | [0, 39, 42, 1]° |

The feedforward has two sources. `wb_u3_internal_ff` alone feeds the filtered LUMPED
estimate, exact in free flight (`d_e = 0`); `wb_u3_internal_ff_use_w_hat` feeds the
design note's INTERNAL `T^-T w_hat`, the only form admissible under contact. The
internal source costs 1.2 -> 3.1-3.4 mm settled and 0.8 -> 1.3 N.m peak arm torque,
because `w_hat` is partial (stationary arm, alpha = 0 in the note's PE condition, 4 of
10 directions); it works here because `L_c` books 99.3% of the residual to the
collective (measured `w_hat_z` = -10.73 of -10.81 N). NO CONTACT has been simulated,
so the source distinction is derivation, not measurement.

```bash
# reproduce (every run is a clean relaunch; ~5 min each; yaml restored on exit)
docs/docs_aerial_manipulator/posture_ablation_20260909/run_ablation.sh   # Q1, PID off, plant-side kf
docs/docs_aerial_manipulator/posture_ablation_20260909/run_fix.sh        # Q3, allocator-side kf
/usr/bin/python3 docs/docs_aerial_manipulator/posture_ablation_20260909/score.py docs/docs_aerial_manipulator/posture_ablation_20260909/l1_*.npz
```

Traps: plant-side `PEGASUS_PLANT_KF_SCALE` ≤ 0.90 never takes off (SAFETY
gravity feedforward short + UDE height gate, the 7.13.3 run C deadlock) — the
driver still exits 0 with an empty event list, so check `t_direct` is finite
before trusting a "completed". And `pkill -f` on a campaign script kills the
shell that names the same script later on its own command line (the 7.15
bracket-trick caveat) — issue the kill and the edit as separate commands.

#### 7.15.10 The arm's current loop — the droop is gone, the residual is the model (2026-09-09)

`fsc_open_manipulator` now closes a **software current loop** around Dynamixel
Mode 16 (`fsc_open_manipulator/doc/Current Loop Design.md`), calibrated on J2
and J3. It removes the back-EMF droop the simulator had modelled since
2026-09-03, so the plant model was rebuilt around what the loop leaves behind.

| bench, loop OFF → 1.5 Hz | joint 2 | joint 3 |
|---|---|---|
| torque delivered, 20 °/s | 95.0 % → **99.5 %** | 87.5 % → **91.8 %** |
| current error | 13–15 → **4.2–4.6 mA rms** | 14–16 → **6.7–6.9 mA rms** |

The plant is now `tau_app = clip(tau_cmd, ±tau_cap) + Kt·i_err`, with `i_err`
**zero-mean, 7 mA rms, first-order band-limited at 5 Hz** (= J3, the worse
calibrated joint, on all four — J1/J4 are uncharacterised), i.e.
**[16.3, 15.0, 17.7, 16.3] mN·m rms**. The *shape* is the result: loop off, the
error was a one-sided ≈ −20 mA bias that never crossed zero; loop closed, it is
centred scatter. A bias corrupts a torque-controlled flight; scatter does not.

```bash
# knobs — env > yaml > built-in.   PEGASUS_ARM_SERVO_MODEL is now current|ideal
#   'pwm' / 'pwm_0903' are REFUSED with a message (they modelled the droop)
sim_arm_current_noise_enable: true     # in both whole-body _sim yamls
sim_arm_current_noise_a: 0.007         # A rms
sim_arm_current_noise_bw_hz: 5.0       # first-order corner
sim_arm_current_noise_seed: 0

PEGASUS_ARM_SERVO_MODEL=ideal \
  scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_direct_actuation_sitl.sh shiqi_machine

# the campaign (two matched 16 s-hold missions; ~9 min each, clean relaunch)
docs/docs_aerial_manipulator/arm_current_noise_20260909/run_noise.sh shiqi_machine
/usr/bin/python3 application/robotic_arm/utils/wb_l1_metrics.py \
    docs/docs_aerial_manipulator/arm_current_noise_20260909/*.npz \
    docs/docs_aerial_manipulator/posture_ablation_20260909/l1_mission_kx32.npz

# the model on its own (no Isaac needed)
/usr/bin/python3 extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/servo_model.py
```

**Result — the L1 whole-body law handles it, no gain touched.** Same `_sim`
plant (+15 % allocator kf, mass/inertia ×1.10, 10/10/5 mm CoM shift, MN4010
rotor lag), same 2026-09-09 law (posture absent, `wb_ee_anchor_com` +
`wb_u3_internal_ff`, `K_y` 20 / `D_y` 12, `k_x` 32). Run B vs the matched
ideal-arm baseline:

| | noise B | noise C | ideal arm |
|---|---|---|---|
| legs / aborted | 10 of 10, no | 9 of 10*, no | 10 of 10, no |
| CoM err mean / late | 21.0 / 9.6 mm | 16.9 / 5.4 mm | 18.8 / 5.0 mm |
| EE err mean / max | 3.5 / 33.4 mm | 3.2 / 28.5 mm | 2.9 / 30.4 mm |
| tilt p-p, \|e_R\| max | 3.08°, 0.181 | 2.95°, 0.160 | 2.78°, 0.170 |
| peak arm torque | 0.798 of 3.0 N·m | 0.785 | 0.751 |
| joint clamp / rotor sat | **0.00 / 0.00 %** | **0.00 / 0.00 %** | 0.00 / 0.00 % |
| q2 / q3 abs err | 0.81 / 1.21° | 0.76 / 1.19° | 0.58 / 0.97° |
| phantom \|F̂_y\| | 0.123 N | 0.120 N | 0.077 N |

\* C lost `traj_both_back` to the §7.15.9 **driver race** — the planner went
CALCULATING→PLANNED inside one sample, `send` hit CALCULATING, and the driver
waited out its 45 s exec timeout while the vehicle held. Not a control failure;
the fix belongs in `wb_l1_campaign_driver.py`.

**THE MECHANISM IS IN THE OBSERVER'S ARM CHANNELS, and it reproduces 3/3** —
std over the quiet soak hold, N·m:

| | d̂ᶜ UNFILTERED (deadbeat) q1/q2/q3/q4 | d̂ FILTERED (what the law sees) |
|---|---|---|
| noise, 3 runs | .0247–.0254 / .0389–.0496 / .0229–.0266 / .0199–.0206 | .0029 / .0031–.0050 / .0039–.0042 / .0038–.0040 |
| ideal arm | .0138 / .0330 / .0143 / .0108 | .0003 / .0047 / .0025 / .0006 |

The deadbeat estimate picks the injection up almost exactly — quadrature excess
over the ideal run is **16.7 mN·m on q4 and 17.9 on q3 against the 16.3 / 17.7
injected** (q1/q2 read ~21 because the law's own reaction adds there) — and
`C(s) = ω_c/(s+ω_c)` then cuts it ~5×, to 3–5 mN·m against the arm's 700 mN·m
hold torque. That bandwidth/accuracy split is what the observer exists for, and
it is why 7 mA costs single-digit millimetres.

**Do NOT read this off the COLLECTIVE channel.** `dc_z_std` is 0.73 / 0.23 /
0.29 N on noise-B / noise-C / the **ideal** run: the arm's joint-space noise is
not what dominates it and the run-to-run spread swamps the effect. Scored that
way it looks 2.5× worse than ideal on one run and identical on the next.

Five things worth knowing before touching this:

- **Do NOT turn on `current_loop_bandwidth_hz` in
  `torque_controller_isaac_aerial.yaml`.** The plant already models the loop's
  *outcome*, and `IsaacTopicEffortSystem` does export an effort state interface
  (06 republishes the applied torque), so a non-zero bandwidth closes a second
  loop around an already-post-loop signal and cancels the disturbance under
  test. It defaults to 0.0; that file now says why.
- **The controller cannot see the noise**, which is what makes the test fair —
  the whole-body node never subscribes to `joint_states.effort`, it only
  publishes its own commanded torque there (grep-verified).
- **`wb_l1_lc_var_q` 0.0025 no longer "must be RAISED".** That instruction was
  written for the droop: a systematic joint-torque error the observer *should*
  find. Zero-mean noise is one it should NOT chase, so a low prior is now
  correct.
- **The applied-torque log columns are in MODEL order**, unlike the position
  columns, which need `LOG_Q_OF_JOINT = (2,0,1,3)`. Mixing them up scores
  joint 2 against joint 1 and invents ~0.7 N·m of error.
- **Read `wb_hover_stability.py --servo current` for ordering only** — the noise
  is seeded but it is still scatter; use `--servo ideal` for delay margins.

Measured end-to-end from telemetry over a 33 s quiet hold: `applied − commanded`
has mean **0.1–0.8 mN·m** (the bias really is gone), at 21–24 mN·m rms against
15–18 injected. The excess is the law *reacting* — its own 250 Hz commanded
torque carries 13–19 mN·m of ripple in the same window — plus the asynchronous
50 Hz log sampling of two streams.

`l1_noise_A.npz` was flown first at the driver's default 6 s holds. It completed
the whole mission too, but **its settled numbers are not comparable** with the
16 s baseline (§7.15.7's hold-length trap).


#### 7.15.11 The C++ whole-body trajectory planner — full §7.15.1 flight, 2026-09-14

The planner behind the `planner` window is no longer `whole_body_planner.py`
importing this repo's `utils_planner`; it is the rclcpp node
`whole_body_trajectory_planner` of the new, self-contained package
`fsc_trajectory_planner` (`~/Workspaces/fsc_autopilot_ws/src/fsc_trajectory_planner`,
see its CLAUDE.md). Same topics, services and states. Its maths is locked to
this repo's Python by gtests (kinematics 1e-12, straight-line plan samples
≤3.5e-9, flat B-spline 1e-9) and it plans in ~3-4 ms.

**One full §7.15.1 mission on this rig, standard test via
`wb_l1_tune_cycle.sh l1 cpp_planner fsc_lab_machine`** (L1 stack, `_sim`
yaml, backend `bspline`, plant as configured on 2026-09-11 incl. the per-joint
current-loop residual). The steps went through the drone-GS reference topic and
the compatible-trajectory legs through the arm-GS `whole_body_planner/ee_target`
path — the ground stations' own ROS interface, driven by the campaign driver.
**Ten legs, ten PLANNED verdicts within 10-20 ms of the target, no refusal, no
abort, no watchdog**; SAFETY revert, landing and disarm normal. Data:
`trajectory_planner_cpp_20260914/l1_cpp_planner.npz`, score in
`l1_cpp_planner_metrics.txt`, logs in `logs/`.

| leg | peak CoM err | settled | max tilt |
|---|---|---|---|
| step x +0.5 / −0.5 m | 352 / 386 mm | 61 / 50 mm | 4.6 / 5.5° |
| step y +0.5 / −0.5 m | 356 / 389 mm | 94 / 73 mm | 4.7 / 5.7° |
| step yaw +30 / −30° | 166 / 70 mm | 45 / 40 mm | 4.3 / 2.3° |
| **compatible trajectory, EE (down 6 cm, +8 cm y, +60° heading)** | **34 mm** | **11 mm** | 1.2° |
| compatible trajectory, back | 15 mm | 5 mm | 0.8° |
| whole-system move (base + arm at once) / back | 211 / 199 mm | 69 / 71 mm | 2.5 / 2.6° |

`u1` 47.59 N soak mean (run E: 47.57 N — same injections), `w_hat_thrust` −10.65 N, `d_hat_z` −10.81 N, zero joint clamping, debug[56]
(streamed reference fresh) = 1 throughout DIRECT at 100.0 Hz. The ordering of
§7.15.5 holds — the compatible-trajectory legs are the best-tracked motions
by an order of magnitude over the translations. The absolute numbers are NOT
comparable with the 2026-09-06 run E table (the ARM-side plant moved after it:
the count↔torque register path and the per-joint current-loop residual of
§7.15.10 were not in run E), and no
same-day Python-planner A/B was flown; the reference streams of the two
planners are identical to 1e-9 by the gtests, so a flight A/B would measure
run-to-run scatter, not the port.

One capture warning, as with the Python planner: `arm is NOT at rest at
capture: max |qdot| = 0.057 rad/s (> 0.05)` — the servo jitter of the
current-loop residual, harmless.

**Second flight the same evening, after the package was re-based on the flight
stack's exported `wb_law` library** (the model and flat planner are now linked
from `fsc_autopilot_ros2`, not copied; only the IK and the straight-line Picard
planner are the package's own code). Same rig, same mission, tag
`cpp_planner_wblaw`: 10/10 legs, no refusal, no abort, stream fresh 100 %,
`u1` 47.60 N, zero clamping. Per-leg peak / settled: x 318-356 / 79-84 mm,
y 328-389 / 79-89 mm, yaw 77-167 / 46-53 mm, compatible EE trajectory 46 / 21 mm
(back 35 / 15 mm), whole-system move 209-220 / 71-76 mm — the same profile as
the first flight within this rig's run-to-run scatter. Data
`l1_cpp_planner_wblaw.npz`, score `l1_cpp_planner_wblaw_metrics.txt`.

#### 7.15.12 END-EFFECTOR TRAJECTORY mode — two circle flights, 2026-09-15

`fsc_trajectory_planner`'s second mode (see its CLAUDE.md, "End-effector
trajectory mode"): a periodic circle / figure-8 EE trajectory with yaw along
the tangent, planned as ONE compatible whole-body run (min-snap ramp-in, laps,
ramp-out, rest at both ends) — the MATLAB task-space planner's redundant
recovery adapted to the z-x-x-z arm (q1 = 0, q2 an assigned sinusoid, drone
yaw the first z angle), the thrust feasibility residual solved as a relaxed
Picard fixed point on B-spline flat outputs, every bound of the constraint set
checked over the run, the largest feasible time scale found by bisection.
Commanded from the arm GS's new **EE trajectory** tab (Sine Test / Demos tabs
removed) or, as here, from `fsc_trajectory_planner/test/ee_trajectory_sim_driver.py`
through the same topics/services:
`ee_trajectory_sim_cycle.sh <tag> fsc_lab_machine circle <fraction of s_max>`.

Two flights on this rig (L1 stack, `_sim` yaml), both complete: SAFETY to 1 m
→ DIRECT → select circle (0.5 m, 2 laps, s_max 1.13 limited by the 0.30 rad/s
yaw rate) → time scale → Go-to-start (compatible transition, executed) → at
start → Start → run → HOLD at the start → SAFETY → land → disarm. The planner's
own FK round trip of the reference was 0.0-0.1 mm / 0.0° on both.

| flight | s | lap | EE speed | raw EE err mean / max | lag | residual after lag | circle radius flown |
|---|---|---|---|---|---|---|---|
| `ee_circle_eecircle` | 0.90 | 26.5 s | 0.118 m/s | 214 / 267 mm | 1.90 s | 102 mm | 0.398 m of 0.508 |
| `ee_circle_eecircle_slow` | 0.45 | 53.0 s | 0.059 m/s | 138 / 166 mm | 2.35 s | 56 mm | 0.447 m of 0.502 |

(EE error = the planner's `current_ee`, i.e. measured joints on measured
odometry, against its `reference_pose`; z error 1-2 mm throughout.)

**Third flight, 2026-09-15, after the circle was re-centred on the WORLD
ORIGIN** (`ee_traj_center_origin`, the default; the two above were anchored on
the held EE point). Same rig, same 0.5 m radius, 2 laps, s = 0.905 of the 1.131
maximum — `ee_circle_origin_circle.npz`:

| | reference | flown |
|---|---|---|
| distance from the world origin | 0.500 m, sd **0 mm** | 0.387 m, sd 14 mm |
| path centre (x, y) | — | (+0.047, +0.005) m |
| raw EE error mean / max | — | 215 / 262 mm |
| lag / residual after it | — | 1.90 s / 104 mm |

The reference is exactly the commanded circle about the origin; the flown one
is concentric with it to ~5 cm and 77 % of its radius, which is the same
first-order lag with gain below one as the two flights above, at the same
speed. Re-centring changed where the run is, not how well it is tracked.

**What this flight tested that the first two could not: Go-to-start as a real
transition.** With the circle anchored on the held EE point the start rest was
where the vehicle already was, so the button was a null move. Centred on the
origin it is a **0.56 m translation with a 90° yaw change** — planned by the
transition planner, executed without a Send, complete in 15 s, and it settled
inside the Start gate's 5 cm / 5° / 3° tolerance on the first try. That gate
was the open question when the circle moved: §7.15.5 measures 50-90 mm of
settled CoM error after a 0.5 m step leg, which is the same order as the
tolerance. It passed here; on a vehicle that settles worse, the gate is the
thing that will refuse, and its three numbers are `ee_traj_start_*_tol`.

**The error is the whole-body law's tracking bandwidth, not the reference.**
It is a first-order lag of ~2 s with a gain below one (the flown circle is
78 % / 89 % of the commanded radius at the two speeds), and it halves when the
speed halves. That is the same structural P/D position-loop behaviour §7.15.5
measured on the step legs (55-90 mm settled offsets, 200-390 mm peaks) now seen
on a continuously moving reference; the reference itself is dynamically
consistent to 0.1 mm. Raising the loop's bandwidth (or adding the missing
velocity/acceleration feedforward path in the law) is the lever; slowing the
time scale is the operator's. Data and logs under
`trajectory_planner_cpp_20260914/` (`ee_circle_*.npz`).

Operational note for cleanup on this machine: `stop_isaacsim_stack.sh` now
`pkill -f`s `whole_body_trajectory_planner`, so — as the cycle script already
warns for the controller names — never type that string on the command line
that runs it; run the stop scripts from a script file.

#### 7.15.13 The PD+ calibration corrections, applied to the streamed u3 (2026-09-14)

**The request.** The PD+ campaign (`fsc_open_manipulator/doc/Calibration
Result for PWM Torque Control.pdf`) does two things after its nominal law:
the torque→count transformation and a set of compensations/corrections for
a more faithful torque. The whole-body law's arm channel now gets the same
treatment, **after `u3` is transformed to a joint torque and on the arm
side**: PD+ and the streamed whole-body torque are both "a nominal torque
law" feeding one bottom-level hardware correction. **Nothing in
`fsc_autopilot_ros2`'s law, observer or node changed** — the flight node
still asks for N·m on `joint_torque_command` and the arm controller still
maps them; what changed is that the servo now delivers closer to what was
asked, so the observer has less of the gearbox to find. The report's own
§2.2 draws it this way already: everything below "THE MAPPING" is actuator
and plant, not law.

**Where it lives:** `fsc_open_manipulator`'s `ExternalTorqueController`, whose
shared base already ran the PD+ pipeline and gated it off on the pass-through
(`auxiliary_terms_enabled() = !passthrough_`). Three keys in
`external_torque_controller_hardware_aerial_pwm.yaml`, all LIVE, all default
false (every existing config byte-identical):

| report stage | PD+ | pass-through before | pass-through now |
|---|---|---|---|
| (3) `κ_pwm` mapping | yes | yes | yes |
| (4) friction `[fc + µ\|τ\|κ]·tanh(q̇_d/w)` + viscous | yes | no | **yes** — `passthrough_auxiliary_terms: true` |
| (5) dither, gated on a moving reference + stuck joint | yes | no | **yes** — same key |
| (5) position-error integral | yes | no | **no** — `passthrough_integral: false`, deliberately |
| (2) gravity scale `(S−1)·g(q)` | yes (in `τ_ff`) | no | **yes** — `passthrough_gravity_correction: true` |
| (6) `max_effort` clamp, (7) back-EMF ff + current trim, (8) PWM bound | yes | yes | yes |
| (1) reference lead / observer velocity inside the law | yes | — | not reachable from the arm side (this node closes on Present Velocity) |

Three judgement calls, stated so they can be reversed:

- **The integral stays OFF on this path.** It is feedback on `q_d − q`, not a
  post-correction, and this law's L1 arm channel plus `wb_u3_internal_ff`
  already integrate the joint residual. A second integrator at the servo would
  quietly take part of what the observer must estimate and corrupt its `ŵ`
  attribution. The key exists for a bench A/B only; the hardware stack script
  prints a red warning if it is ever true.
- **The gravity-scale correction is valid on a torque the URDF did not
  produce** because the two models agree: `wb_arm_model_check` with this yaml's
  sign map `[-1 1 1 -1]` puts `WholeBodyParams::t650Defaults`' `g(q)` within
  **0.0016 N·m** of the Pinocchio URDF's over 10 poses (run 2026-09-14). So
  the law feeds forward `g_urdf` to 0.2 %, the calibrated arm needs
  `S·g_urdf` (`S = [1, 0.897, 0.969, 1]`), and `(S−1)·g` is the same
  plant-model error whichever law computed the nominal. On j2 at home that is
  −0.072 N·m — the "handover step the observer absorbs over a few seconds"
  the arm CLAUDE.md records, now paid up front.
- **The friction load term is the (corrected) streamed torque**, eq. (5)'s
  `µ|τ_load|`; at a hold it equals the `|S·g|` the coefficients were fitted
  against and in motion it carries the dynamic load too.

**Every added term is exactly zero while the arm reference holds** — the
`tanh(q̇_d/w)` and the dither gate are driven by the whole-body planner's
`q̇_d` through `external_reference_topic`, so a hovering DIRECT with the arm
at rest streams `u3` untouched apart from the gravity term.

**In simulation nothing changes.** The Isaac yaml
(`torque_controller_isaac_aerial.yaml`) leaves the keys unset and now says
why: the Pegasus plant models no gearbox friction (only the current loop's
residual, §7.15.10), so a friction feed-forward would inject torque the plant
never loses, and no URDF gravity model is loaded there. The §7.15.1 stack
script and launcher are untouched.

**Validated 2026-09-14 on mock hardware, no arm** (the real controller on
`fake_components/GenericSystem` with the real hardware yaml under `/uav_t`;
24 checks, ~25 s):

```bash
cd ~/ros2_ws/src/fsc_open_manipulator/open_manipulator_x_custom_controller
test/run_passthrough_corrections_test.sh      # ALL PASS 2026-09-14
```

Against the installed yaml's own constants (`κ_pwm = [169.47, 149.70, 135.25,
148.51]` counts/N·m, the report's numbers): pure pass-through `duty = κτ` to
0.01 counts; gravity correction `[0, −8.05, −1.75, 0]` counts exact; friction
`[2.90, 17.46, 14.03, 7.78]` to 0.02 counts with the reference's sign; dither
rms `[0, 33.95, 13.58, 10.18]` = `A/√2` exact, none on j1; aux term **exactly
0** at rest before and after moving; integral 0 when off, winds to `i_clamp`
57.61 on j2 when asked, 0 again when released; stream stale → the PD+ hold
takes the arm back. **Two things the test found on the way:**
`current_loop_bandwidth_hz_joints` was live in PD+ but NOT in the
pass-through's copy of the callback (fixed — both copies match again), and
mock hardware mirrors the written duty into Present Current, so the trim must
be off (scalar AND list) or it integrates a fictitious error into every reading.

**Reading a flight bag.** `law_debug` grew 18 → 28, append-only: `[18..21]`
what friction/dither added per tick, `[22..25]` the gravity correction, both
in duty counts, `[26]`/`[27]` the gravity/integral flags. The old invariant
"`law_debug[3]` is the complement of `[0]`" holds only with
`passthrough_auxiliary_terms` false; with it true, `[3]` is 1 in both modes by
design. `current_trim_report.py` reads `[13:17]` and is unaffected. The
hardware stack script's arm-side check now prints both keys either way.

**Live A/B on the bench, no relaunch:**

```bash
A=/uav_0/fsc_open_manipulator/external_torque_controller
ros2 param set $A passthrough_auxiliary_terms false      # friction + dither off
ros2 param set $A passthrough_gravity_correction false   # (S-1)*g off
ros2 topic echo $A/law_debug --field data | head        # [18..25] read 0 within a tick
```

**NOT flown.** First-flight watch list: `law_debug[18..25]` against the
node's own `u3` (`wb_control_debug[13..16]`), the observer's arm-channel
estimate (it should shrink — the gearbox is now paid at the servo), and
whether this yaml's `wb_u3_estimate_max_j4: 0.06` stiction cap is still
needed with the dither acting on the stream.

#### 7.15.14 Imperfect friction + gravity compensation on the arm — 5 % mismatch, 3 flights (2026-09-14)

**The request.** §7.15.13 gave the whole-body torque the calibration report's
corrections. The report also says friction and gravity compensation are the
*dominant* corrections on the real arm, so: put both into the simulated plant
with a controlled **5 % mismatch** against what the arm controller compensates
and see whether the whole system still stabilizes. No back-EMF — the arm's
1.5 Hz current loop removes it on hardware (§7.15.10); only its residual is
simulated, as before.

**What the plant now carries** (`servo_model.py` + `06`, read from the `_sim`
yaml by the launcher like every other `sim_*` key):

- `sim_arm_friction_scale` — the report's eq. (5) gearbox friction
  `[fc + µ|τ|]·tanh(q̇/w)` on the **measured** joint velocity and the
  transmitted torque, `fc` = [17.1, 31.4, 57.5, 52.4] mN·m (the yaml's duty
  counts through κ_pwm), `µ` = [0, 0.246, 0.161, 0], `w` = 0.015 rad/s. It is
  applied explicitly at 250 Hz, so it is **momentum-clamped** to
  `I_min·|q̇|/dt` (I_min = the 0.02 kg·m² armature) — a Coulomb term steeper
  than `I/dt` would flip a joint's sign inside one step; the self-test spins an
  inertia down at the 3 N·m clamp and asserts it never reverses. 1.05 = 5 %
  more than compensated; 0 = the frictionless plant every earlier campaign flew.
- `sim_arm_mass_scale` — arm link masses and inertias ×1.05 while **every**
  model (the law's `t650Defaults`, the arm controller, 06's own hold) keeps the
  nominal arm: a 5 % gravity-compensation error on every joint. Distinct from
  `sim_plant_mass_scale`, which moves the BODY and leaves the arm exact
  (checked in `06`: the total is hit by adjusting `/body` only).
- **Compensation in sim:** `torque_controller_isaac_aerial.yaml` carries the
  same `fc`/`µ` in N·m with `passthrough_auxiliary_terms: true`, driven by the
  whole-body planner's `q̇_d`; the launcher passes
  `passthrough_corrections:=true|false` to `torque_control_isaac.launch.py`
  (a second parameter file, verified to reach the controller in all three
  states) so compensation is on **iff** the plant has friction —
  `PEGASUS_ARM_FRICTION_COMP=0` is the uncompensated A/B. Dither stays 0: the
  tanh plant has no stiction to break.

**Three flights, same mission, driver and 16 s holds as §7.15.7's baseline
(`posture_ablation_20260909/l1_mission_kx32.npz`, ideal arm):**

```bash
docs/docs_aerial_manipulator/arm_mismatch_20260914/run_mismatch.sh shiqi_machine   # A, B, C
```

| | **A** friction ×1.05, mass ×1.05, comp ON | **B** ×1.00 / ×1.00, comp ON | **C** ×1.05 / ×1.05, comp OFF | ideal arm |
|---|---|---|---|---|
| aborted / legs | no, 10/10 | no, 10/10 | no, 10/10 | no, 10/10 |
| joint clamp / rotor sat | **0 % / 0 %** | 0 / 0 | 0 / 0 | 0 / 0 |
| peak arm τ [N·m of 3] | 1.00 | 0.95 | 1.06 | 0.75 |
| tilt p-p [°] | 3.35 | 3.58 | 3.19 | 2.78 |
| `d̂_z` [N] | −11.18 | −10.82 | −11.18 | −10.81 |
| `τ_q2` mean [N·m] | 0.733 | 0.715 | 0.728 | 0.705 |
| q2 / q3 abs err [°] | 0.89 / 2.61 | 0.44 / 0.61 | 1.46 / 3.03 | 0.58 / 0.97 |
| `late_com` (mean \|e\|, last 30 s) [mm] | 24.0 | 17.8 | 35.8 | 5.0 |
| final hold, `e_com,y` mean / **std** [mm] | −4.6 / **25.1** | −7.4 / **16.2** | −11.7 / 17.2 | −0.4 / 1.2 |
| final hold, `e_y` std [mm] | 0.9 | 1.6 | 1.0 | 0.0 |
| phantom `F̂_y,z` mean / std [N] | −0.16 / **0.44** | −0.01 / **0.42** | **−0.28** / 0.10 | −0.04 / 0.01 |
| final hold `τ_q2` std [N·m] @ f | **0.106 @ 0.88 Hz** | **0.105 @ 0.94 Hz** | 0.025 @ 0.06 Hz | 0.003 |
| `traj_ee` peak / settle [mm] | 75 / 28 | 79 / 25 | 90 / 44 | 11 / 2 |

**1. It stabilizes.** Three of three, every leg, nothing on the joint clamp
or the allocator, peak arm torque a third of the clamp, tilt within 0.8° of
the ideal arm. The 5 % gravity error is found and paid: `d̂_z` carries the
extra 0.31 N of arm weight (−10.82 → −11.18) and `τ_q2` the extra 5 % of
hold torque (+0.019 N·m).

**2. The 5 % itself is cheap; the *presence* of friction is what costs.**
A vs B differ by the mismatch only: j3 error 0.6 → 2.6°, CoM wander std
16 → 25 mm, `late_com` 18 → 24 mm. B vs the ideal arm differ by friction
only, and that is the bigger step on every row.

**3. `late_com` is a WANDER, not an offset — read the per-axis numbers.**
`late_com` is the mean of |e|, so a slow oscillation reads as a standing
error. Per axis the means are 1–7 mm; what is 16–25 mm is the **std of
`e_com,y`** (world y = along the arm), a ~0.1 Hz wander of the base while the
EE task error stays at 1–2 mm std. The arm holds its task; the base drifts
under it.

**4. The compensation buys the standing accuracy and costs a marginal
0.88 Hz cycle — and the cycle is the ATTITUDE LOOP's, not the arm's.** C vs A:
without compensation the joints park short (q2/q3 1.46/3.03°), the phantom
force has a −0.28 N **bias** (the observer booking the unpaid friction on the
EE) and `late_com` is worst at 36 mm; with it the bias is gone. But A and B
carry a **0.9 Hz line in the j2 torque at 0.105 N·m rms** that C does not —
its frequency is `√(k_R/I)` = 5.55 rad/s = 0.88 Hz, the attitude mode
recorded on 2026-08-23, and `F̂_y,z` chatters at the same frequency with
0.42 N rms (a *jitter*, not a bias — mean ≈ 0). `hold_chatter.py` scores every
hold of the mission separately (`hold_chatter.txt`) and shows it is
**metastable**: in C the `traj_ee` leg kicks it to 97 % of the torque's power
and it then **decays** over the next holds (53 → 13 → 11 → 6 → 5 %); in A it
is absent in the first hold (13 %) and locks in from the third leg (84–96 %);
in B it is present from the first. Since the friction feed-forward is exactly
zero at a hold (the streamed `q_d` is bit-constant there — checked), the
compensation cannot *drive* the cycle at a hold; what it does is re-excite it
at every leg end, where the feed-forward switches off on `q̇_d` while the
plant's friction switches off on the lagging `q̇`. Gearbox friction makes the
0.88 Hz attitude mode marginally damped; compensation keeps kicking it.

**What this does and does not say.** Stable, bounded, no saturation: yes.
The near-rest friction is a numerical construction (the momentum clamp is an
effective stick below ~0.04 rad/s on a loaded j2), so the cycle's *damping* is
model-dependent even though the mode itself is the airframe's. One run per
configuration except that A and B agree; C is one run. Not tried: repeating
C, damping the 0.88 Hz mode (k_w / M_r_d are the levers, §7.14.5), or a
rate-limited switch-off of the friction feed-forward at leg ends. The
`wb_l1_lc_var_q` prior is **not** implicated — the phantom force is jitter,
and C's bias is the unpaid friction, not attribution.

Data + tools: `docs/docs_aerial_manipulator/arm_mismatch_20260914/`
(`run_mismatch.sh`, `hold_chatter.py`, `metrics.txt`, `hold_chatter.txt`,
`compare_mismatch.png`, `logs/`). Knobs, all env > yaml > built-in:
`PEGASUS_ARM_FRICTION_SCALE`, `PEGASUS_ARM_FRICTION_WIDTH`,
`PEGASUS_ARM_MASS_SCALE`, `PEGASUS_ARM_FRICTION_COMP`. **CONFIG A IS THE
STANDING SIM PLANT** (user request, 2026-09-14 — robustness validation ahead of
the flight test): both `_sim` twins carry 1.05 / 0.015 / 1.05, so every
whole-body sim flight from here on flies the friction plant with the
compensation on. The GMO twin gets the same values on purpose — the repo's rule
is that the two sim yamls must not describe different plants. To fly the old
frictionless arm for an A/B:
`PEGASUS_ARM_FRICTION_SCALE=0 PEGASUS_ARM_MASS_SCALE=1.0 <launcher> <config>`.

### 7.17 AM-T650 WHOLE-BODY + L1 with the FOUR-DIMENSIONAL attribution — added 2026-09-16

**§7.15's rig with one thing swapped: how the lumped residual is ATTRIBUTED.**
Same coupled law, same gains, same plant (the standing config A of the
2026-09-14 note at the top of §7.15), same SAFETY/DIRECT split, same gates,
watchdogs and torque-mode arm stack, same two ground stations, **and the same
executable** — `autopilot_whole_body_l1_direct_actuation_node`. Layer 1 of the L1
observer (the deadbeat piecewise-constant estimate and the `C(s)` filter feeding
`f_d` and `u_2`) is untouched. What changes is Layer 2, per the September 2026
revision of the working note (`disturbance_observer_draft.tex`, *"the
four-dimensional interaction wrench"*; the six-dimensional design it replaces is
archived in `disturbance_observer_draft_backup.tex`):

| | §7.15 (6-D) | §7.17 (4-D) |
|---|---|---|
| wrench model | `w_e ∈ R^6`, split by a metric-weighted projector onto `N(J_e)` + a dynamic identifier (`ω_i`, `L_c`, `lc_var_*`) | `w_e = S_e^T F + k`, `k ∈ N(J_{e,q}^T)` joint-invisible; `F ∈ R^4` read off the **joint rows** |
| task force | `F_hat_y = Λ_y S_e Λ_e^{-1} w_hat_e` | `F_hat = J_{y,q}^{-T}([w_hat_Σ]_q − w_hat_q)`, `F_hat_y = (1−χ) C_x F_hat` |
| free flight (`χ = 1`) | `F_hat_y` ≈ 0.08–0.4 N of phantom | **`F_hat_y ≡ 0` by construction** |
| joint rows | modelled by the identifier's prior | **trimmed** by `w_hat_q` (`ω_q`, free flight only), frozen in contact |
| `u_3` feedforward (`wb_u3_internal_ff`) | filtered lumped `d_f` (per-group `ω_c`) | `d_int^f = C_x T^{-T} w_hat_int` (eq. u3_4), one scalar `ω_x` |
| identifiability | needs excitation + the `W` prior | at every instant iff the four joint rows are known (Property ident) |
| contact flag | none needed (`Z_0^T J_e^T = 0`) | **exogenous** task flag `wb_l1_contact`, + threshold fallback |
| yaml | `..._whole_body_l1_direct_actuation_t650_sim.yaml` | `..._whole_body_l1_4d_direct_actuation_t650_sim.yaml` |
| executable | `autopilot_whole_body_l1_direct_actuation_node` | **identical** |

Implementation: the `four_d` branch of the fork's `wb_l1_observer.cpp` (Python
reference `extensions/.../utils_controller/l1_observer.py`, section 6 of its
self-test: exact attribution in contact to 2e-15, where the 6-D identifier had a
0.59 N error on the same wrench), selected by `wb_l1_four_d`. Parity-locked to 1e-8
by `WbL1ParityTest` over 9 rollouts (three new `four-d-*` sets cover free flight, a
task-flagged contact and the collision-threshold latch; one covers the per-block
filter); the five 6-D rollouts are byte-identical to before, `WbParityTest` 4/4
still holds. The yamls differ by **`vehicle_name`, `wb_u3_internal_ff_use_w_hat`
(true — the note's own `u_3` form), `wb_l1_omega_x` and the `wb_l1_four_d` block**
— diff them before every campaign:

```bash
cd ~/ros2_ws/src/fsc_autopilot_ros2/config && diff \
  <(grep -vE '^\s*#|^\s*$' params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml) \
  <(grep -vE '^\s*#|^\s*$' params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml)
```

**Two things the port had to decide that the note leaves open**, both recorded
in the code and reproduced in the Python reference:

- **`Φ_d`, not `Φ`.** Unchanged from §7.15.3 — Layer 1 is the same code.
- **The order of the discrete Layer-2 operations.** Per tick: joint rows of
  `T^T d_hat^c` → phase flag (task flag, then the threshold fallback evaluated on
  the **previous** tick's filtered reading, so the decision never depends on the
  estimate it gates) → trim `w_hat_q` (exact ZOH of the first-order filter, only if
  free) → `F_hat` → `C_x` → gate → `w_hat_int` (with the **raw** `F_hat`, as eq.
  step_int4 writes it) → `C_x T^{-T}` → bounds. The bounds are the existing
  `wb_l1_max_*` (force/heading rows of `F_hat_y` by `max_wrench_*`, `d_int^f` by
  `max_force/torque/joint`). `wb_l1_omega_x_{t,r,q}` (default 0 = the note's
  scalar) let `d_int^f`'s three blocks carry their own `C_x` bandwidth — added as a
  diagnostic during tuning (7.17.3) and kept.

#### 7.17.1 Run sequence — copy-paste

Same as §7.15.1 with the 4-D script names; step numbers match it (1 = build and
5 = DIRECT are not needed here). Run each step in its own terminal, and never
chain step 0 with a launcher.

**shiqi_machine** (`shiqi-desktop`):

```bash
# 0. clean slate            (any terminal — BOTH lines, this order)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 2. ROS 2 stack            (terminal 1 — FIRST, owns the agent)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 3. Pegasus / PX4 SITL     (terminal 2 — also starts the arm stack + arm ground station)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh shiqi_machine

# 3'. INSTEAD OF 3, WITH THE EE MARKER CUBE (terminal 2) — a mocap cube welded
#     into the gripper; the arm GS's EE Trajectory tab draws it as "EE (Meas)".
#     Step 2 needs nothing extra (its emulator already lists obj_0). 30 g is the
#     flown mass — do NOT take the yaml's 0.2 kg default, see below.
PEGASUS_EE_MARKER_CUBE=1 PEGASUS_EE_MARKER_CUBE_MASS=0.03 \
  ~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh shiqi_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

**fsc_lab_machine** (`fsc-jupiter`) — the same four steps against this machine's
roots (`~/Workspaces/fsc_autopilot_ws`, `~/Source/fsc_PegasusSimulator`); step 1
is shiqi's with those two substituted, and step 5 is identical.

```bash
# 0. clean slate            (any terminal — BOTH lines, this order, and as TWO
#    SEPARATE calls: kill_stale_sim_processes.sh -y kills its own invoking shell
#    (exit 144), so anything chained after it in the same call never runs)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/Source/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 2. ROS 2 stack            (terminal 1 — FIRST, owns the agent)
~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh fsc_lab_machine uav_0

# 3. Pegasus / PX4 SITL + TORQUE-MODE ARM STACK + ARM GROUND STATION (terminal 2)
~/Source/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh fsc_lab_machine

# 4. OFFBOARD, then arm     (terminal 3 — order is mandatory)
source /opt/ros/humble/setup.bash && source ~/Workspaces/fsc_autopilot_ws/install/setup.bash
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}
```

Two things specific to this box, both measured rather than assumed:

- **The ground station comes up unaided.** `python3` here is Isaac Sim's 3.11,
  which has neither PyQt5 nor an rclpy built for it, but this stack script
  resolves the interpreter through `scripts/python_interpreter.sh` and picks
  `/usr/bin/python3` (3.10.12) on its own — verified 2026-09-20. `FSC_GS_PYTHON`
  overrides it. The older "launch the GS by hand" workaround is NOT needed here.
- **Real-time factor is ~0.34, so do not compare numbers from this machine with
  shiqi_machine's.** The estimator/controller chain runs on the wall clock while
  the plant advances at 250 Hz × RTF, so ~65% of consecutive odometry samples are
  byte-identical repeats and the law acts on a staircase. That is far outside
  what this rig's attitude tune and `M_r_d` delay margin were chosen against; it
  aborted an 8-flight validation set here in 2026-09-11, including with an
  `ideal` arm. Use this box for bring-up, wiring and build checks — not for
  flight-quality metrics.

**Three banners to read before entering DIRECT, in the autopilot pane:** the
magenta `DISTURBANCE OBSERVER: L1 ADAPTIVE` (else you are on the GMO), the second
magenta line `ATTRIBUTION: FOUR-DIMENSIONAL, on the joint rows` (else you are on
§7.15's 6-D rig), and the cyan `LAW CHECK: u3 coupling feedforward ... source 4D
INTERNAL d_int^f`. **The Pegasus launcher checks the second one for you**: because
the two rigs share an executable, a process-name gate cannot tell them apart, so
it reads `wb_l1_four_d` off the RUNNING node (`ros2 param get
/uav_0/fsc_autopilot_ros2 wb_l1_four_d`) and refuses to start against the 6-D stack
rather than mislabel a flight. In DIRECT the pane also prints, once a second, the
magenta `4D attribution:` watch line — `chi`, the raw `F_hat`, `w_hat_q` and the
consumed `F_hat_y`.

**EE marker cube (step 3', added 2026-09-18).** Off unless asked for: the yaml
carries `sim_ee_marker_cube: false`, and the environment overrides the yaml, which
is all step 3' does. Chain: 06 welds the cube to the wrist-roll link at the model's
grasp point → publishes it as mocap body `obj_0` → the step-2 emulator turns it
into `/obj_0/mocap` → the arm GS draws EE (Meas) with a black line to EE (FK).
**Confirm before arming**: the Pegasus pane prints magenta
`EE MARKER CUBE ACTIVE: 0.03 kg welded into the gripper ...` (off prints
`EE marker cube off`), `ros2 topic hz /obj_0/mocap` streams, and the tab's overlay
reads `EE (FK) → EE (Meas) Rigid Offset: … mm, …°`. The two triads should COINCIDE,
axes included: the cube's body frame is the planner's EE convention (x along the
claw, z the gripper's up). Seated at five arm poses (2026-09-20): **0.1–0.3°, 0.3–1.0 mm**;
in flight 8.6–10.5 mm was seen at the circle's start rest (2026-09-18). That offset
is the model's grasp frame against the USD gripper, not a tracking error: EE (FK)
is computed from the MEASURED joints. **~120° means the rigid body is on the wrist's
own axes** — what 06 did before 2026-09-20, and what a hardware rigid body defined
any other way will show.
**The cube is an UNMODELLED end-effector payload — no controller knows its mass —
so keep it light** (measured 2026-09-18, `ee_marker_cube_20260918/`):
- 200 g (the yaml default) cannot take off. It noses over at spin-up in SAFETY.
- 100 g takes off, but the SAFETY arm hold sags 5.5°, and the DIRECT gate
  (`wb_gate_arm_rad` 0.05 rad = 2.9°) refuses. The sag is ~4.7°/100 g + 0.8°, so
  ≤ ~44 g passes the gate.
- 30 g flew the whole circle: DIRECT, go-to-start, 2 laps, back to HOLD.

A heavier cube needs a wider gate or an integral/feed-forward in the SAFETY arm
hold. That is a design decision, not a launch flag.

**Automated campaign** (one data point = one full relaunch, ~7 min):

```bash
# one run of the 4-D rig (or `l1` for the 6-D one, `gmo` for §7.14's):
application/robotic_arm/utils/wb_l1_tune_cycle.sh l1_4d <tag> shiqi_machine
# edit the 4-D yaml between runs (unknown key = error; writes a float where the
# file holds one -- an integer literal for a double parameter kills the node at
# startup with no log line, 2026-09-17):
/usr/bin/python3 application/robotic_arm/utils/wb_l1_set_gains.py --four-d omega_x=0.25
# the 2026-09-16/17 campaign, back to back (restores the yaml on exit; a running
# copy must NOT be edited -- bash reads a script by byte offset):
docs/docs_aerial_manipulator/l1_4d_20260916/run_4d.sh shiqi_machine
# score:
/usr/bin/python3 application/robotic_arm/utils/wb_l1_metrics.py     docs/docs_aerial_manipulator/l1_4d_20260916/*.npz
/usr/bin/python3 application/robotic_arm/utils/wb_compare_metrics.py docs/docs_aerial_manipulator/l1_4d_20260916/*.npz
cd docs/docs_aerial_manipulator/l1_4d_20260916 && /usr/bin/python3 summarize_4d.py && /usr/bin/python3 ripple_4d.py && /usr/bin/python3 ripple_4d.py --soak
# EVERY reference-tracked state, per leg, peak / rms / settled-rms (7.17.4):
/usr/bin/python3 traj_errors.py --csv traj_errors.csv
PYTHONNOUSERSITE=1 /usr/bin/python3 plot_4d.py --six l1_6d_A.npz l1_6d_B.npz --four l1_4d_4d_best_A.npz l1_4d_4d_best_B.npz --out compare_6d_4d.png
```

#### 7.17.2 Debug array — §7.15.2's 97 elements plus 9

Append-only, so **[0..96] are exactly §7.15.2's** (the L1 block ends at [88],
`wb_u3_estimate_max_j*` raw/applied at [89..96]). In DIRECT it is now 106:

```
[97..100]  F_hat RAW, the 4-D joint-row reading J_yq^-T([w_Sigma]_q - w_hat_q)
           (N, N, N, N.m). This is what the collision test sees and what the
           impedance WOULD render without the (1 - chi) gate; on this rig it
           is the honest free-flight phantom number, since [58..61] is gated
           to exactly 0. Zero on the 6-D path.
[101..104] w_hat_q, the joint-row trim (N.m), FREE-FLIGHT ONLY: it converges
           toward the joint-level calibration residual (friction, gravity
           mismatch, current-loop residual) and freezes in contact.
[105]      chi: 1 = free flight, 0 = contact (task flag OR threshold latch)
```

`wb_l1_metrics.py` reports the new slots as `phantom_Fraw_*`, `wq_hat_late_Nm`
and `chi_free_frac`; `wb_compare_metrics.py` adds an `|Fraw|` column per leg.
**Do not compare `phantom_Fy_*` across the two rigs** — on the 4-D one it is 0 by
construction, which is the design, not a measurement.

#### 7.17.3 Tuning the 4-D design — the one knob is `ω_x`, and the answer is 0.25 rad/s

Data `docs/docs_aerial_manipulator/l1_4d_20260916/` (`run_4d.sh`, `summarize_4d.py`,
`ripple_4d.py`, `plot_4d.py`; tables `metrics.txt`, `legs.txt`, `ripple.txt`,
`hover.txt`, `summary.json`; figure `compare_6d_4d.png`; launcher/driver logs in
`logs/`; the `.npz` stay out of git). **Ten flights, 2026-09-16/17, none aborted, 0 %
joint clamp, 0 % rotor saturation, 0 % estimator bound in every one.** Every run: the
standing **config A plant** (+15 % allocator kf, body mass/inertia ×1.10 + 10/10/5 mm
CoM shift, MN4010 rotor lag, current-loop residual, gearbox friction ×1.05, arm mass
×1.05), the rig's standard mission with `--hold-between 16` (SAFETY takeoff → DIRECT →
20 s hover soak → x/y/yaw steps ±0.5 m / ±30° → the two compatible-trajectory legs →
abort → land), the same driver, the same law gains, the same Layer-1 observer. The
two 6-D runs reproduce the 2026-09-14 config-A flight (phantom 0.40 / 0.24 vs 0.40 N,
tilt p-p 2.95 / 3.06 vs 3.35°), so the baseline is not the scatter.

**Why `ω_x` is the whole tuning problem.** In free flight (`χ = 1`) the 4-D law differs
from the flown 6-D one in exactly two places: `F_hat_y ≡ 0` (no knob), and `u_3`'s
feedforward is `C_x` of the WHOLE residual at ONE bandwidth, where the 6-D path fed
the per-group-filtered `d_f` (2 / 0.5 / 0.5 rad/s). `ω_q` moves only the RAW `F_hat`
reading, never a torque, so it is not a flight knob. The 6-D yaml's `ω_x = 20` (a
wrench-reading smoother there) would hand `u_3` a nearly raw 250 Hz momentum
difference, so the sweep started at 2.

| run | `ω_x` [rad/s] | CoM err mean / late [mm] | EE pos mean [mm] | tilt p-p [deg] | τ peak [N·m] | j2 / j3 ripple [mN·m] | tilt ripple [deg] |
|---|---|---|---|---|---|---|---|
| **6-D A** | (20, 6-D) | 31.6 / 18.6 | 7.2 | 2.95 | 0.96 | 70 / 50 (the §7.15.14 cycle locked in) | 0.256 |
| **6-D B** | (20, 6-D) | 29.4 / 16.0 | 6.2 | 3.06 | 0.83 | 7.8 / 5.7 | 0.104 |
| 4-D | 6 | 65.1 / 24.3 | 13.2 | 14.3 | 2.31 | 339 / 363 | 2.11 |
| 4-D | 2 | 42.3 / 25.5 | 8.7 | 6.04 | 1.65 | 259 / 133 | 1.18 |
| 4-D | 0.5 | 30.4 / 18.0 | 7.7 | 6.97 | 1.26 | 196 / 91 | 0.88 |
| 4-D | per-block 2 / 0.5 / 0.5 | 29.8 / 10.5 | 7.0 | 4.81 | 1.23 | 167 / 86 | 0.74 |
| 4-D | per-block 2 / 0.25 / 0.25 | 29.6 / 27.0 | 7.9 | 6.52 | 1.18 | 182 / 93 | 1.04 |
| **4-D** | **0.25** | 27.0 / 18.0 | 5.5 | 2.78 | 0.84 | **5.0 / 2.5** | **0.075** |
| **4-D** | **0.25**, repeat A | **23.1 / 3.7** | **5.0** | **2.69** | 0.88 | **6.3 / 3.6** | **0.075** |
| **4-D** | **0.25**, repeat B | **22.1 / 3.3** | **4.7** | 2.92 | 0.85 | **5.1 / 3.0** | **0.083** |

(ripple = std of the 0.5–3 Hz band-passed signal over all of DIRECT after the 40 s
entry allowance, `ripple_4d.py`; CoM/EE from `wb_l1_metrics.py`.)

- **The mode that binds is the airframe's ~0.9 Hz attitude mode** (`√(k_R/I)`,
  §7.14.4; the metastable j2-torque cycle of §7.15.14). At `ω_x = 2` the arm
  feedforward drives it to 259 mN·m of j2 ripple at 0.94 Hz; at 0.5 still 196.
- **It is NOT a bandwidth-mismatch artefact, and it is the TRANSLATIONAL block that
  excites it.** A per-block `Ω_x` was added (`wb_l1_omega_x_{t,r,q}`, default 0 = the
  note's scalar, parity-locked) and set to exactly the 6-D path's 2 / 0.5 / 0.5, i.e.
  `u_3` fed the SAME filtered residual the 6-D flew: still 167 / 86. 2 / 0.25 / 0.25:
  182 / 93. Scalar 0.25: 5.0 / 2.5. So the boundary sits between 0.25 and 0.5 on the
  translational block, and what the 6-D design had — the rendered
  `−(Λ_y M_y⁻¹ − I) F_hat_y`, a ~0.3–0.4 N force read off a 20 rad/s wrench filter,
  acting as fast momentum-residual feedback into `u_3` — was damping that mode by
  accident. The 4-D removes that term on purpose, so its feedforward bandwidth has to
  come down to where the mode is not fed.
- **At 0.25 the mode is gone**, 3/3: j2/j3 ripple 5–6 / 2.5–3.6 mN·m, below BOTH 6-D
  runs, and the best CoM, EE and tilt numbers of the campaign. That is the note's own
  scalar form; the per-block keys stay in the code as the diagnostic they were.
  **Shipped: `wb_l1_omega_x: 0.25`** in the 4-D yaml.

#### 7.17.4 6-D vs 4-D on the same plant — what the attribution buys

**EVERY LEG OF THIS MISSION IS A COMPATIBLE TRAJECTORY**, steps included: the
whole-body planner solves a min-snap transition and streams the full reference set
(`x_cd` + derivatives, `b1_d`, `r_ed`, `b1_de`, `q_d`), so every state has a
reference and each is scored below on its own, in its own unit —
`traj_errors.py` in the campaign folder, `traj_errors.txt` / `.csv` for the full
per-run output.

> **CORRECTION, 2026-09-17.** The per-leg table first published here reported one
> translation number (CoM) plus an "EE" column that was
> `norm(e_y)` over ALL FOUR components — three in METRES plus
> `sin(heading error)`, which is DIMENSIONLESS. An 8.7° heading error contributes
> 0.151 to that norm and was printed as "151 mm" of EE position error. **The EE
> position error was never 110–233 mm; it is 4–7 mm settled on both designs.**
> `wb_compare_metrics.py` now reports `peakEE`/`settEE` (position, mm) and
> `pkHead`/`setHead` (heading, deg) as separate columns; the numbers below are
> recomputed per channel and cross-checked between two independent
> implementations (`traj_errors.py` and the fixed `wb_compare_metrics.py`).

**Conventions, and they matter.** `‖e_R‖ = |sin θ|` exactly for the angle θ between
`R_0` and the commanded `R_0c`, and `e_y[3] = −(b1ec·b2e)` is the SINE of the EE
heading error — both are inverted through `asin`, never read as radians. The joint
channel is `max_j |q_j − q_dj|` with the log's broadcaster order `[q2,q3,q1,q4]`
mapped back through `LOG_Q_OF_JOINT`. Every window is masked to DIRECT: the last
leg runs past the abort into the SAFETY descent, where the node publishes the
shorter debug prefix and the recorder zero-pads it, which reads as a 0 mm CoM
"error" and a 65° joint "error".

**Hover** (the 20 s soak, DIRECT +26…+43 s, arm at rest; **rms over the window**):

| | CoM pos [mm] | EE pos [mm] | EE heading [deg] | heading p-p [deg] | attitude [deg] | joints [deg] | rendered \|F_hat_y\| [N] | raw \|F_hat\| [N] |
|---|---|---|---|---|---|---|---|---|
| 6-D A | 40.2 | 5.5 | **8.94** | **13.23** | 1.62 | 9.08 | **0.30** | – |
| 6-D B | 38.8 | 5.2 | **9.52** | **13.05** | 1.63 | 9.58 | **0.30** | – |
| 4-D 0.25 | 33.9 | 5.8 | **0.20** | **0.53** | 1.40 | 3.11 | **0** | 0.32 |
| 4-D 0.25 A | 35.5 | 6.4 | **0.31** | **0.66** | 1.57 | 3.61 | **0** | 0.37 |
| 4-D 0.25 B | 34.4 | 6.1 | **0.35** | **0.79** | 1.43 | 3.09 | **0** | 0.33 |

**The 6-D heading error is a slow OSCILLATION, not a standing offset**: 0 → 13°
peak-to-peak at **0.06–0.11 Hz**, continuous through hover and through the settled
window of every leg. The 4-D holds 0.8° p-p. The joint column is the same error
seen at the actuators — the EE heading is carried by the `(q1,q4)` pair, which is
why the two columns track each other to a few tenths on the 6-D.

**Whole mission, all eleven legs pooled** (peak = worst sample on any leg; rms and
settled pooled across legs; settled = last 2 s of each leg, inside its 16 s hold):

| state | unit | 6-D peak / rms / settled | 4-D peak / rms / settled | settled gain |
|---|---|---|---|---|
| CoM position `\|x_c − x_cd\|` | mm | 247 / 50 / 30 | 241 / 46 / 13 | 2.3× |
| CoM velocity `\|v_c − ẋ_cd\|` | mm/s | 342 / 65 / 22 | 338 / 65 / 17 | 1.3× |
| platform attitude `asin‖e_R‖` | deg | 9.39 / 2.05 / 1.29 | 9.26 / 1.91 / 0.50 | 2.6× |
| EE position `\|e_y[0:3]\|` | mm | 32.3 / 7.8 / 5.5 | 34.5 / 6.9 / 3.0 | 1.8× |
| **EE heading** `asin\|e_y[3]\|` | deg | 26.28 / 9.72 / **9.57** | 26.87 / 6.00 / **1.33** | **7.2×** |
| **joints** `max_j\|q_j − q_dj\|` | deg | 25.38 / 9.85 / **9.68** | 26.42 / 6.07 / **1.88** | **5.1×** |

**THE PEAKS ARE IDENTICAL ON EVERY CHANNEL AND THE SETTLED ERRORS ARE NOT.** Peak
error is the planner's transition, which both designs fly the same way (247 vs
241 mm, 9.4 vs 9.3°, 32 vs 35 mm, 26.3 vs 26.9°). What the attribution changes is
what is left once the vehicle has arrived, and it changes the ORIENTATION channels
by far the most — heading 7.2× and joints 5.1×, against 2.3× for CoM position and
1.8× for EE position. A translation-only metric would have called these two designs
nearly equivalent.

**Per leg, peak / settled** (6-D = mean of 2 runs, 4-D = mean of 3 at ω_x = 0.25):

| leg | CoM 6-D | CoM 4-D | EE pos 6-D | EE pos 4-D | EE head 6-D | EE head 4-D | joints 6-D | joints 4-D |
|---|---|---|---|---|---|---|---|---|
| step x +0.5 m | 233 / 15 | 186 / 20 | 14.4 / 4.8 | 12.0 / 2.9 | 14.3 / 8.6 | 1.1 / 0.7 | 14.3 / 8.5 | 4.5 / 1.7 |
| step x back | 232 / 35 | 240 / 30 | 11.1 / 4.7 | 10.7 / 4.3 | 12.3 / 8.5 | 0.8 / 0.1 | 12.3 / 8.6 | 2.5 / 2.5 |
| step y +0.5 m | 225 / 24 | 222 / 4 | 8.5 / 5.4 | 7.0 / 1.9 | 14.6 / 10.4 | 2.7 / 0.1 | 13.2 / 10.3 | 2.6 / 1.0 |
| step y back | 206 / 29 | 208 / 5 | 9.2 / 5.8 | 7.5 / 2.6 | 15.2 / 7.9 | 2.6 / 0.3 | 14.9 / 7.9 | 1.6 / 1.2 |
| yaw +30° | 44 / 7 | 14 / 2 | 28.4 / 5.7 | 31.8 / 1.4 | 12.3 / 11.2 | 1.3 / 0.3 | 12.5 / 11.3 | 1.4 / 0.4 |
| yaw back | 43 / 34 | 7 / 2 | 31.0 / 5.2 | 32.7 / 1.4 | 15.0 / 8.5 | 1.4 / 0.1 | 15.1 / 8.6 | 1.3 / 0.8 |
| EE traj (arm moves) | 68 / 43 | 27 / 3 | 18.5 / 6.1 | 13.2 / 2.7 | 19.3 / 7.6 | 16.7 / 2.4 | 18.9 / 8.3 | 15.4 / 2.5 |
| EE traj back | 43 / 39 | 40 / 15 | 14.9 / 4.1 | 21.6 / 4.8 | 14.0 / 10.0 | 23.2 / 2.1 | 14.1 / 9.9 | 22.0 / 2.3 |
| base + arm traj | 113 / 35 | 157 / 11 | 13.6 / 6.0 | 12.2 / 3.5 | 14.3 / 5.9 | 25.3 / 0.9 | 14.4 / 6.5 | 24.4 / 2.4 |
| base + arm back | 44 / 20 | 61 / 9 | 8.3 / 5.6 | 10.1 / 3.1 | 13.4 / 11.4 | 11.8 / 1.7 | 13.6 / 11.6 | 11.6 / 1.8 |
| final hold | 43 / 20 | 13 / 4 | 8.2 / 6.7 | 4.2 / 2.0 | 12.1 / 8.7 | 2.1 / 1.4 | 12.1 / 9.1 | 2.4 / 2.2 |

- **The 6-D carries 8–11° of EE heading into the settled window of EVERY leg**,
  step and trajectory alike, with the joint column mirroring it. The 4-D settles to
  0.1–2.5°.
- **The one place the 4-D peaks HIGHER is the three trajectory legs (17–25°), and
  that is tracking lag on a commanded 60° EE heading sweep, not a standing error**
  — it decays to 0.9–2.4° inside the hold while the 6-D's comparable peak never
  decays. Both designs lag that sweep, because the heading impedance is
  deliberately soft (`K_y,ψ = 0.3` N·m/rad).
- **Peak on a single leg of a single run is not reproducible on this rig** and the
  means above hide two cases worth stating: the 6-D `base + arm` peak is 182 mm in
  run A and 44 mm in run B, and one of the three 4-D runs took a 147 mm excursion
  on `base + arm back` where the repeats took 16 and 19 mm. The settled columns and
  the pooled rms are the reproducible quantities.
- **Same entry transient** (peak 499–542 vs 435–439 mm — within the 374–545 mm
  scatter of this handover, §7.15.8; settle 7.2–7.3 s both), same `d_hat_z`
  (−11.18 N both: Layer 1 is the same code), same peak torque (0.85–0.88 vs
  0.83–0.96 N·m).
- **The raw `F_hat` is 0.15–0.24 N late** (the collision test's reading), with
  `w_hat_q` still converging at `ω_q = 0.2` over the 5-minute flight; it is a
  250 Hz momentum difference through `J_yq⁻ᵀ` (5–10 m⁻¹), so its instantaneous
  value is noisy (0.9–1.7 N std) and a collision threshold must sit on its filtered
  value — which is what `C_x` and the hysteresis are for.

**Not covered:** contact (the `χ = 0` branch — trim freeze, `F_hat_y` rendered,
`w_hat_int` split — is parity-locked and self-tested to 2e-15 against the note's
exact-attribution lemma, but no contact exists in Isaac, so it has never been flown);
`ω_q` in flight (never a torque in free flight); hardware.

#### 7.17.5 The 4-D rig on the C++ trajectory planner, and its hardware pair (2026-09-17)

**What moved under it.** The whole-body planner is now `fsc_trajectory_planner`
(§7.15.11), and it gained the END-EFFECTOR TRAJECTORY mode (§7.15.12). On this
desktop the package is cloned beside the flight stack at
`~/ros2_ws/src/fsc_trajectory_planner`. It links `fsc_autopilot_ros2::wb_law`, and
that library compiles `wb_l1_observer.cpp` too, so it is rebuilt after every
law/observer change (§7.17.1 step 1 now builds both). The 4-D rig needed no law
change for it: the planner streams the same `WholeBodyReference` the node already
consumed, and the attribution sits entirely downstream of the reference.

**What was adapted, and how to re-derive it.** The 4-D sim yaml and its isaacsim
stack script were REBUILT as "the merged 6-D twin + the saved 6-D→4-D patch" —
not hand-edited. So they now carry the `/**/whole_body_trajectory_planner` section
(bspline backend, every `ee_traj_*` key) and launch the C++ node in the `planner`
window exactly as the 6-D stack does. The old-vs-new 4-D delta was precisely the
incoming planner change. Diff the pair before any campaign, as before:

```bash
cd ~/ros2_ws/src/fsc_autopilot_ros2/config
diff params_single_aerial_manipulator_whole_body_l1{,_4d}_direct_actuation_t650_sim.yaml
diff params_single_aerial_manipulator_whole_body_l1{,_4d}_direct_actuation_t650.yaml
```

**Flying the EE trajectory mode on the 4-D rig.** From the arm GS's "EE
trajectory" tab (select Circle/Figure-8 → slider → Go to start → Start trajectory),
exactly as §7.15.12. Or unattended through the standard harness. The planner
package's own `ee_trajectory_sim_cycle.sh` hard-codes the 6-D stack, so
`wb_l1_tune_cycle.sh` gained a mission selector that runs that package's installed
driver on any of the three rigs:

```bash
WB_L1_MISSION=ee_circle  WB_L1_EE_SCALE=0.8 application/robotic_arm/utils/wb_l1_tune_cycle.sh l1_4d <tag> shiqi_machine
WB_L1_MISSION=ee_figure8 WB_L1_EE_SCALE=0.8 application/robotic_arm/utils/wb_l1_tune_cycle.sh l1_4d <tag> shiqi_machine
# WB_L1_MISSION unset (= standard) is the 7.15.5 mission via wb_l1_campaign_driver.py, unchanged
```

That driver records odometry, the planner's EE reference and the measured EE —
not `wb_control_debug`, so §7.17.4's ripple and attribution scorers do not apply
to its npz.

**FLOWN 2026-09-17 on this desktop — the 4-D rig through BOTH shapes**
(`docs/docs_aerial_manipulator/l1_4d_planner_20260917/`, harness above, scorer
`ee_run_score.py`). Plant is §7.15's config A: friction ×1.05, arm mass ×1.05,
+17.6 % allocator kf, body mass/inertia ×1.10, CoM shift, rotor lag. Law is
§7.17's: posture term absent, `ω_x` 0.25. Both flights ran the full sequence
with no abort, no refusal and no INFEASIBLE: SAFETY climb → DIRECT (accepted
first try) → select → READY → rescale to 0.8 s_max → Go-to-start → Start gate
passed → run → HOLD → SAFETY → land → disarm. Go-to-start was a real planned
transition (8.7 s for the circle).

| run | s / s_max | T (lap) | EE ref speed | raw EE err mean / max | lag | residual after lag | flown vs commanded |
|---|---|---|---|---|---|---|---|
| circle 0.5 m, 2 laps | 0.905 / 1.131 | 57.0 s (26.5 s) | 0.133 m/s | 124 / 178 mm | 1.05 s | 55 mm | radius 0.450 of 0.500 m, centre (+0.035, +0.015) m |
| figure-8 0.5 × 0.25 m, 2 laps | 0.286 / 0.358 | 171.7 s (83.8 s) | 0.034 m/s | 50 / 106 mm | 1.37 s | 27 mm | extent 0.970 × 0.464 of 1.0 × 0.5 m |

z error 2 mm on both; DIRECT lasted 92 s and 205 s. In DIRECT the 4-D watch line read `chi=free`, raw
`|F_hat|` 0.08–0.24 N, and `w_hat_q` on J2 at −0.07 to −0.11 N·m — the arm
mass/friction mismatch being trimmed on the joint rows, as designed. **This is
the first recorded figure-8 flight on any rig** (§7.15.12 and the planner
package record circles only).

**WHAT LIMITS s_max IS THE DRONE'S YAW RATE, FOR BOTH SHAPES** — measured
2026-09-17, and it corrects the planner package's own note (which attributes the
figure-8's limit to acceleration). The EE heading follows the path tangent and
the arm's q1 is pinned at 0, so **the airframe itself must yaw once per lap**;
`w_max` (0.30 rad/s, the planner section's generic angle-rate bound) is what
binds. At their s_max both shapes sit far inside the translational bounds —
circle peak |v| 0.15 of 0.30 m/s and |a| 0.077 of 0.15; figure-8 0.078 and
0.035. Raising ONLY `w_max` to 1.0 rad/s in a scratch yaml moves s_max
**1.131 -> 1.90** (circle, which then binds on speed) and **0.358 -> 0.815**
(figure-8, which then binds on acceleration at |a| = 0.1498 of 0.15). So the
lever for a faster demo is the yaw-rate bound or a longer `ee_traj_lap_time`,
not `v_max`/`a_max` — but raising it asks the attitude loop for more, on a
plant whose rotor-lag margin is the reason for §7.14.4's tune. Unflown above
0.30; screen it in sim first. The planner's own diagnostics at s_max (fake-rig
capture, no sim): sigma_nd 0.306 both shapes, peak joint torque 0.76 N·m,
peak |q̇| 0.04 / 0.02 rad/s, rotor forces 8–10 N, reference FK round trip
0.1 mm / 0.0004°.

**Do NOT read the circle row as "4-D tracks better than §7.15.12's 6-D circle"**
(215 mm, 1.90 s lag, radius 0.387 m). That flight ran on fsc_lab_machine
(RTF ≈ 0.34, §7.15's 2026-09-11 finding) on a frictionless arm with the 6-D
design. Three things differ, and §7.15.12 does not record how its lag was
fitted. The matched comparison is one command on this desktop:
`WB_L1_MISSION=ee_circle WB_L1_EE_SCALE=0.8 wb_l1_tune_cycle.sh l1 <tag>`. One
flight per shape — repeat before quoting any number as a property (the
2026-08-10 scatter lesson). The error has the same structure §7.15.12
describes, a first-order lag with gain below one that shrinks with speed: the
figure-8 moves at a quarter of the circle's speed and halves every column.

**THE ARM SPLIT OF THE EE RUNS — q2 30 +- 10 deg AT A 60 deg FOLD, 48 s PERIOD
(2026-09-17, user request).** Both 4-D yamls; the 6-D twins still carry the old
80 / 40 +- 8 / one-lap set.

**The fold and the q2 centre are ONE choice.** At rest `q3 = fold - q2` EXACTLY,
so lowering the q2 centre raises q3 by the same amount, and symmetric margins
mean `centre = fold/2`. The old 40 +- 8 at fold 80 put BOTH joints at 48 deg,
2 deg off the +50 stop. The requested 30 +- 10 **at the old fold is refused by
the planner in 0.1 ms** — q3 would run to 60 deg — and its message names the only
admissible centre there, exactly 40. Dropping the fold to 60 puts q2 and q3 both
in [20, 40] deg, 10 deg of margin each, which is what the request was for.

**The period must DIVIDE `laps * lap_time`** (48 s here) or the run does not end
on the pose it started from; 48 s is one q2 cycle per run. Peak joint rate is
`amp * 2pi * s / period`, and **the GS slider sets the worst case**, so it is
scored at s_max, not at s = 1:

| | s = 1 | at s_max | bound |
|---|---|---|---|
| circle (s_max 1.131) | 1.58 deg/s | **1.79 deg/s** | 2 deg/s |
| figure-8 (s_max 0.358) | 0.55 deg/s | **0.62 deg/s** | 2 deg/s |

The old default (period 0 = one lap, 24 s) would be 2.96 deg/s at the circle's
s_max. What the lower fold costs, measured: the EE sits **0.135 m below the base
instead of 0.061** (reach 0.264 vs 0.253 m), sigma_nd **0.306 -> 0.258** (still
2.6x the 0.10 keep-out), peak joint torque 0.76 -> 0.82 N.m of 3.0. s_max is
unchanged in both shapes — it is the drone yaw rate, which does not care about
the arm. The run's start arm pose becomes [0, 30, 30, 0] deg, so Go-to-start now
unfolds the arm from the folded home as part of the transition.

Check any `ee_traj_*` edit against the planner BEFORE flying it — same
feasibility checks as DIRECT, no Isaac, ~1 min:

```bash
cd ~/ros2_ws/src/fsc_trajectory_planner/test && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
PYTHONNOUSERSITE=1 /usr/bin/python3 \
  ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/l1_4d_planner_20260917/ee_plan_probe.py \
  ~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml circle
```

It prints the READY/INFEASIBLE verdict, the diagnostics, and the rates re-planned
at s_max. NOT re-flown at this split — the 2026-09-17 circle and figure-8 flights
above were flown at 40 +- 8 / fold 80.

**SIM AND EXPERIMENT NOW PLAN THE SAME WAY (2026-09-17, user request).** The
`planner` backend was the ONLY difference between every whole-body yaml and its
`_sim` twin — measured across all three pairs, 33 keys each, one difference. All
three hardware files (GMO, 6-D L1, 4-D) now select **`bspline`**, so the planner
sections are byte-identical and a sim rehearsal plans the trajectory the flight
will fly. Verified: the 4-D hardware yaml drives the planner's own rig loopback
end to end (`PLANNED T=5.5s (a_max, 0 dilations)` — the bspline message format —
Send, 546 samples, hold at the goal, SAFETY revert).

The precondition the old comment set is met: bspline has the 2026-09-14 full
§7.15.1 sim mission (10/10 legs, no refusal) and it planned every Go-to-start of
the 2026-09-17 EE flights. It also enforces MORE than straight_line — rotor force
and joint torque are checked in the bspline path and not in straight_line — so
its refusals are a superset.

**`straight_line` WAS THEN REMOVED (same day, user request).** With every
shipped config on bspline it was a selectable name nothing selected, and its
refusals were a SUBSET of bspline's — rotor force and joint torque are checked
in the flat path and were not checked at all in the Picard one. Gone from
`fsc_trajectory_planner`: the backend (~420 lines), its registry entry, the
`straight_line`-only `PlanOptions` knobs, four `StraightLine.*` gtests and their
156 kB fixture, and the transition half of `dump_python_fixtures.py`; gone from
this repo: `utils_planner/transition_planner.py`'s `plan_transition` (221 lines,
no callers — `flat_bspline_planner.plan_transition` takes the same rest specs
and returns the same dict). **That file keeps the model, FK, IK and rest
algebra**, which eleven tools here import. `PlannerRegistry.NamesAndUnknown`
moved beside the surviving backend and asserts the registry holds exactly
`bspline`.

What it was, for recovery: a straight EE line with the CoM on a degree-16
polynomial solved by a Picard fixed point — the backend of the 2026-09-12
hardware flight, and the only one that guaranteed a straight end-effector path.
Recover it from git if a transit ever has to clear something.

Verified after the removal: the planner package builds Release clean and its
10 gtests pass (kinematics 4, flat parity + registry 2, EE trajectory 4); all
four rig loopbacks pass against the built node; **`WbParityTest` 4/4,
`WbReferenceBuilderTest` 2/2 and `WbL1ParityTest` 4/4 still pass against the
COMMITTED fixtures**; `transition_planner.py`'s trimmed self-test passes
(IK round-trip 3.2e-13 rad, rest_ref-vs-FK 8.3e-17 m).

**A fixture trap found while checking this:** regenerating
`wb_l1_truth_t650.json` and `python_kinematics_t650.txt` produces last-bit
drift — max |Δ| **8.6e-13** against a 1e-8 parity tolerance, 13894 of 34950
numbers — on code that did not change. The generators are deterministic within
a run pair, so this is BLAS/threading noise in the iterative IK and rollouts,
not a behaviour change. **Do not commit regenerated fixtures unless a fixture
value actually needed to move**; restore them and re-run the parity tests.

**The HARDWARE pair (new, never flown):**
`config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml`
is the 6-D hardware yaml plus the identical parameter delta the sim pair carries
(checked key by key), with identity `AM-T650-WB-L1-4D-HW`. It deliberately carries
two values over from the 6-D hardware file. The first is the **2026-09-12
experiment kf 4.260431e-05** (bench 4.540431e-05); restore the bench value in both
files together. The second is the J4 stiction cap `wb_u3_estimate_max_j4 0.06`,
which bounds the 4-D feedforward too, because `est_task` includes `d_int`. Launch
with:

```bash
~/ros2_ws/src/fsc_autopilot_ros2/scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh uav_0
```

It is the 6-D hardware launcher (same shared `fsc_indoor_autopilot_stack`
session, same C++ planner window with the measured `base_com` and the
`[-1,1,1,-1]` sign map), plus a **FATAL design gate**. The gate requires
`wb_l1_four_d: true` and observer `l1`, and refuses the 6-D hardware identity.
That file is hardware-clean and would sail through the sim-value gate, and the
node is the same executable, so only the yaml can say which design flies.
`ALLOW_SIM_CONFIG` does not override it. Dry-run of the gates: the 4-D hardware
yaml passes; the 6-D hardware yaml and the 4-D sim yaml are refused, each with
its reason. Loaded into the real node and planner under `/uav_test`: 4-D banner,
posture-OFF law check, mass cross-check, planner `straight_line` with the
overrides and the `ee_traj_*` keys. **Both L1 hardware launchers' arm-side check
now expects `current_loop_bandwidth_hz_joints [0.0, 1.5, 1.5, 0.2]`** — the arm
repo's TEMPORARY 2026-09-14 j4 trim. It had printed a red MISMATCH on every
start. Move the two together when the trim is reverted.

#### 7.17.6 FIRST HARDWARE FLIGHT of the 4-D rig — 83 s of DIRECT, stable, arm friction and a mocap dropout (2026-09-18)

Bag `docs/experimental_data_ros2_bag/0918 - T650-AM whole-body-L1-4D-.../flight_wb_l1_4d_20260918_144828`
(the second directory, `..._152112`, has a `metadata.yaml` describing a 242 s recording and
NO `.db3` — not analysed). Report + tools + printed numbers:
`docs/docs_aerial_manipulator/wb_l1_4d_flight_20260918/` (`report.html`; `tools/extract_bag.py`
→ `prepare.py` → `analyze{,2,3}.py` + `figures.py`; `analysis1-3.txt`). Config
`AM-T650-WB-L1-4D-HW` as committed 2026-09-17 (kf 4.260431e-05, K_y 20/D_y 12, ω_x 0.25).

**Mission.** DIRECT 9.66–92.65 s (20 748 ticks, median 4.00 ms, p99 5.3, max 16.9), five
planner legs all completed: EE +0.02 x/−0.07 z (q → [0.3, 33.1, 26.2, −4.7]°), go home, base
+0.61 m along world x (T 5.6 s), the same EE leg again, go home. SAFETY revert was the
OPERATOR (`activate 'Baseline (Safety)'`), not a watchdog; land to z 0.35, disarm 104.8 s.

| question | answer | numbers |
|---|---|---|
| stability | STABLE, bounded, nothing grows | |e_R| rms 0.03–0.09 in holds, tilt p-p 1.1–3.6°, peak 5.3°; 0 rotor sat, 0 bound hits, 0 unallocated, arm ref/state fresh 100 %, stream 99.99 % |
| tracking | bounded, not tight | CoM xy rms 24 mm hover, 30–78 mm holds (0.08–0.16 Hz wander ±15–35 mm std); leg peaks 60–95 mm; EE 4 mm hover, 8–16 mm holds, 29 mm peak; heading 0.7° → 2.3° standing; z ≤ 8 mm |
| arm torque | PRECISE IN HOLDS, not during moves | j2 applied − written mean 0.00 / rms 0.04–0.05 N·m of 0.75; j3 0.03–0.05; moves rms 0.17/0.22 with 0.9 N·m spikes on j3; j1/j4 residual 0.02–0.03 = current-sensor floor; law peak 1.10 of 3.0 N·m; clamp only 19 ticks (j1, the mocap jump) |
| observer | COMPENSATES | u1 + d̂z = 36.79 vs mg 36.74 N; d̂z +0.7…+1.0 N first 3 s → 0 by 12 s → −1.0 N by 60 s (pack 24.06 → 23.64 V; effective kf 4.21e-05 → 4.14e-05); standing yaw −0.10 N·m (motors CCW−CW = −0.10), lateral +0.57/−0.43 N; bounds 16/20 N (jump), 0.44/2, 0.20/1.5 N·m |
| phantom | ZERO, χ = free 100 % | F̂_y consumed 0.000 on all ticks; raw F̂ std 3–6 N (p95 12–15), LP 0.25 rad/s mean 0.1–0.4 max 0.49 N; ŵ_q j2 swings +0.18 → −0.18 N·m with pose; 0912 (6-D) had +0.51 N standing / 3.6 N peaks |

**Findings.**
- **MOCAP DROPOUT 44.65–46.87 s (2.2 s) became a 151 mm ESTIMATOR JUMP.** The estimator
  republished the frozen pose (134 of 210 odom samples byte-identical) so `system_fb_timeout_s`
  1.0 never fired; at re-acquisition odom stepped 151 mm with a 2.6 m/s velocity sample →
  k_v·e_v drove the law sideways for 2 ticks: u1 9 N, motors 0.13–0.41, |e_R| 1.0, j1 on its
  0.34 N·m duty clamp 76 ms. Recovered in ~30 ms, tilt ≤ 4.7°, no watchdog; the 200 mm error
  re-converged over ~10 s and leg 3 started with ~90 mm open. Also an 80 ms loss at 27.95 s.
  Nothing in the law; the estimator must not publish a frozen pose as fresh, and the node
  should treat a single-sample step/velocity spike as a feedback fault.
- **THE ARM MOVES NOW (0912: j2 rotated 0.35° of 3.8°) BUT PARKS WHERE FRICTION CATCHES IT.**
  j2 40 → 35.2 → 35.2° against 40 → 33 → 40 (never returned, −4.8° standing); j3 overshoots
  2.5–4° per move and holds it; j4 3° short. The pass-through friction feedforward (§7.15.13)
  works while q̇_d ≠ 0 and is zero at the hold; K_y 20 × 9 mm = 0.2 N ≪ breakaway. Sets the
  standing EE (8–16 mm) and heading (2.3°) errors. Fix candidates: FF tail after the reference
  stops, K_y 20 → 50 / K_ψ 0.3 → 1.0 (sim-flown), or a hold-phase breakaway pulse.
- The 0.08–0.16 Hz base wander (hover 18/15 mm std, before any arm motion) is §7.18.1's mode,
  unchanged — a model change, not a gain.
- The DEADBEAT estimate is 25× noisier than in Isaac (std 24/29/15 N vs ~1 N): the 2 rad/s
  filter brings it to 0.3–0.5 N, but any collision latch must be defined on a FILTERED reading.
- Standing yaw disturbance −0.10 N·m (10-point CCW/CW motor split) and lateral 0.5 N bias are
  real vehicle asymmetries, found and cancelled; `alloc_thrust_coeff` still the 0912
  experiment value — the observer's −1.0 N is the cost.

**Traps.** Vehicle-side stamped topics run 0.348 s AHEAD of the recorder (odom is recorder-
stamped); `/rosout` from the flight node arrives 3.0 s late between 9 and 13 s — use topic
transitions for timing. The extractor's npz (numpy 2.x object arrays) does not load under the
apt numpy 1.x that matplotlib needs — `prepare.py` writes a numeric-only npz for the figures.
`joint_state_broadcaster` order is [j2, j3, j1, j4]. The written duty (`law_debug[13..16]`)
can exceed `max_effort` because the back-EMF feedforward is added AFTER the clamp by design.


#### 7.17.7 CIRCLE ATTEMPT on hardware — the go-to-start transition tracked, two mocap rigid-body flips, and the 120 Hz feed (2026-09-21)

Bag `docs/experimental_data_ros2_bag/0921 - T650-AM whole-body-L1-4D-.../flight_wb_l1_4d_circle_20260921_123637`
(flights `_112328` and `_115207` the same morning never got past the DIRECT hold: their
planner stalled on stale odometry, "current_ee (world) withheld", twice each). Report, tools,
figures: `docs/docs_aerial_manipulator/wb_l1_4d_flight_20260921/report.html`.

- **Timeline.** DIRECT 4.13–25.10 s. Circle READY at 8.2 s. Go To Start at 13.51 s: T = 14.85 s
  (w_max-limited), base 0.49 m, **heading −88.5° → −236° (148° spin with the EE pinned)**, arm
  40° → 30.7°. Revert at 25.09 s was the OPERATOR (`activate 'Baseline (Safety)'`; no
  `DIRECT WATCHDOG TRIPPED` line in /rosout).
- **The transition was tracked**: 13.5–20.2 s CoM rms 13/9/4 mm, EE 3.5/4.5/7.3 mm, EE heading
  0.64°, e_R,z 0.0006 ± 0.008 through 17°/s of yaw, tilt ≤ 2.7°, 0 saturation, 0 clamp.
- **Two mocap RIGID-BODY FLIPS in DIRECT (20.28, 24.27 s): one-sample jumps of 0.18 m + ~110°**,
  identical offset every time, `mocap_status` normal, no timing gap — an alternative marker
  solution, not a dropout. 18 flips in 49 s, ALL at headings ≤ −70° (0918: 1; flights 1–2 at a
  fixed heading: 0). Orientation jitter > 3°/sample: 1% of samples at 0° heading, 33–40% at
  −120..−180°. The law consumes odometry VERBATIM (no innovation gate): the flip became a
  5–7 m/s velocity sample → u1 −60/+152 N for one tick, motors on the 0/1 rails ~0.1 s, j2
  2.5 N·m, d̂ on its bound 14 ticks, "Wrench allocator saturated … −42 N". PX4's EKF gated the
  same samples (≤ 3 mm) but NOTHING in the loop reads it: SAFETY's position loop is on the same
  raw odometry, and at each post-revert flip its tilt setpoint spiked to 23–35° for 2–3 samples;
  the vehicle moved ≤ 2° only because that path runs through PX4's softened attitude loop and
  the rotor lag. Post-revert handover: 27.8° tilt and a 36 cm drop within 0.5 s.
- **Feedback sources on the hardware stack (verified in all four bags: odom is bit-identical
  to mocap)**: position + linear velocity = RAW mocap via `indoor_mocap_feedback_launch.py`'s
  `MocapOdomBridge` passthrough (velocity finite-differenced upstream by the mocap broadcaster),
  read by the DIRECT law, the SAFETY position loop and the planner; attitude = EKF2
  `vehicle_attitude`; body rate = `sensor_combined` gyro. EKF2's fused `vehicle_odometry` is read
  only by the feedback-loss failsafe landing. The fused alternative is `indoor_estimator_launch.py`.
- **120 Hz mocap → the law's velocity feedback is a short finite difference of position and got
  2.7x noisier**: odom v std 2.6 (0918) / 4.0 (#1) / 10.8 cm/s (#3); deadbeat d̂_x std 13.7 /
  23.6 / 91.5 N; |e_R| mean 0.033 / 0.033 / 0.074 with 95% of its power above 5 Hz at 19–21 Hz;
  stamps jitter 1.43 ms (0.06 at 60 Hz). BUT flight 2 at 60 Hz was as noisy (3.9 mm/sample,
  12 cm/s) — tracking quality varied between flights independent of the rate. Position hold was
  not hurt (10/14/2 mm, best of the four). PX4's EKF velocity: 1.2 cm/s std, 9x cleaner.
- **200 Hz sensor_combined / vehicle_attitude: no adverse sign.** PX4 attitude 0% power > 5 Hz,
  gyro high-frequency fraction lower than 0918, law tick p99 5.5 ms.
- **Before the next attempt**: fix the uav_0 rigid body (hand-track a full yaw circle at flight
  height; keep obj_0's markers out of its asset); gate or fuse the DIRECT feedback (EKF2-fused
  estimator, or a one-sample step/velocity rejection in the client); take velocity from PX4's
  EKF rather than a 120 Hz finite difference; start the circle at the tangent matching the
  current heading so there is no spin.
- Open: after flip #1 the base held a −63 mm y offset for 4 s that d̂_t / ŵ_q do not explain.

#### 7.17.8 HARDWARE STACK WITH EKF2-FUSED FEEDBACK — the 2026-09-23 configuration (checked 2026-09-23)

The 7.17.7 findings (mocap rigid-body flips consumed verbatim, a 120 Hz finite-difference
velocity 2.7x noisier) are both properties of the RAW-MOCAP estimator. Today's experiment
flies the fused twin, which already exists and was sim-proven on 2026-09-21
(`docs/docs_aerial_manipulator/wb_l1_4d_fused_sim_20260921/`, 2 fused missions, 0 aborts):

```bash
# Orin
~/dev_ws/src/fsc_autopilot_ros2/scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator_fused.sh uav_0
```

**What it changes, verified in code (2026-09-23), nothing else differs from the raw script:**
the estimator pane runs `indoor_estimator_launch.py` -> `indoor_state_estimator_node`
(`IndoorStateEstimatorNode` + `Px4MsgBridge` + `Px4FusedOdomBridge`) instead of
`indoor_mocap_feedback_launch.py`. That is the SAME estimator executable as the X650
reference `start_autopilot_stack_x650_fused.sh`. Resulting feedback into the whole-body node
(`pos_feedback_sub_`, `vehicle_attitude_sub_`, `sensor_combined_sub_` in the client):

| quantity | raw script (7.17.6/7.17.7 flights) | FUSED script (today) | read by |
|---|---|---|---|
| position | `/mocap` copied verbatim | EKF2 `fmu/out/vehicle_odometry` -> ENU by `Px4FusedOdomBridge` | law (DIRECT + SAFETY), planner, drift watchdog |
| linear velocity | mocap twist = optitrack processor finite difference | EKF2 velocity (IMU-fused), NED->ENU | law (DIRECT + SAFETY) |
| attitude | EKF2 `vehicle_attitude` | EKF2 `vehicle_attitude` (unchanged) | law, planner |
| body rate | `sensor_combined` gyro | unchanged | law, rate watchdog |
| header stamp | mocap capture time | EKF2 `timestamp_sample` via `timesync_status` offset (arrival time until the first timesync message) | nothing in the law (it uses arrival time for freshness) |

Mocap still reaches EKF2 identically in both variants (`fmu/in/vehicle_visual_odometry`,
position + velocity + yaw, fixed message variances 1e-4 m^2 / 1e-4 (m/s)^2 / yaw 0.0076 rad^2).
The bridge publishes world-frame velocity with an empty `child_frame_id`, so the client takes
its "inertia-frame velocity" branch — the same branch the raw path used. The planner
(`whole_body_trajectory_planner`) reads the same odom topic + `vehicle_attitude`, so law and
planner see one estimate.

**Why this addresses 7.17.7 and what it does NOT change.** A mocap flip now meets EKF2's
innovation gate first (on 0921 EKF2 moved <= 3 mm at every flip). The law's velocity is
EKF2's, not a finite difference, so the 120 Hz feed no longer sets the velocity noise the
law sees; on the X650 fused flights EKF2 vs mocap was 11-15 mm RMS with ~10 ms lag and the
fused topic ran at ~100 Hz (PX4's `vehicle_odometry` cadence, independent of the mocap rate).
NOT changed: EKF2 still fuses the 120 Hz mocap velocity, so the finite-difference noise is
still an EKF2 INPUT; how much of it survives depends on the Pixhawk's `EKF2_EV_NOISE_MD` /
`EKF2_EVV_NOISE` / `EKF2_EVP_NOISE` floors and the `EKF2_EVP_GATE` / `EKF2_EVV_GATE` gates,
which are Pixhawk parameters this repo does not set. Costs to expect (sim measured at RTF 0.5,
hardware numbers unknown): a coarser, lagged feedback (sim: 4.4 mm / 7.7 mm/s rms, ~30 ms),
EKF2 resets arriving as position jumps (the bridge logs `reset_counter` changes), and NO
staleness on mocap loss — EKF2 dead-reckons at full rate, so only `cs_ev_pos` on
`estimator_status_flags` reveals it (the node's `feedbackLost()` watches it, 1.0 s).

**Pre-flight checks specific to the fused path (on the Orin, stack up, vehicle seated):**

```bash
# 1. the RIGHT estimator, and only one publisher on the feedback topic
tmux list-panes -t fsc_indoor_autopilot_stack -F '#{pane_title}'      # expect estimator[EKF2-fused]
ros2 topic info /uav_0/state_estimator/local_position/odom             # "Publisher count: 1"
pgrep -fa indoor_mocap_feedback_node                                   # must print NOTHING
# 2. timesync is running (the fused bridge and the EV stamps both need it; sim never exercised it)
ros2 topic hz /uav_0/fmu/out/timesync_status
# 3. EKF2 is fusing the mocap BEFORE arming
ros2 topic echo /uav_0/fmu/out/estimator_status_flags --once | grep -E "cs_ev_(pos|hgt|vel|yaw)|cs_yaw_align"   # all true
# 4. fused feedback sits on the mocap, at EKF2's rate
ros2 topic hz /uav_0/state_estimator/local_position/odom                # ~100 Hz expected, NOT 120
ros2 topic echo /uav_0/state_estimator/local_position/odom --once --field pose.pose.position
ros2 topic echo /uav_0/mocap --once --field pose.pose.position          # agree to ~1-2 cm seated
# 5. the EKF2 EV settings that decide how much of the 120 Hz velocity noise gets through
#    (read on the Pixhawk console / QGC): EKF2_EV_CTRL 15, EKF2_EV_DELAY, EKF2_EV_NOISE_MD,
#    EKF2_EVP_NOISE, EKF2_EVV_NOISE, EKF2_EVP_GATE, EKF2_EVV_GATE. Record them beside the bag.
```

**First-flight profile with this feedback**: hover in SAFETY first and compare
`state_estimator/local_position/odom` against `/uav_0/mocap` in the bag (the 7.17.7 tools
already extract both) before the first DIRECT entry; rotate through the -70..-150 deg headings
where 0921 flipped and confirm the fused position does not jump. The law, yaml, planner and
arm stack are the 7.17.6/7.17.7 configuration — this is a feedback change only.

### 7.18 Flight-test preparation: x-axis settling tune, cruise speeds, and the WHOLE-BODY vs DECOUPLED comparison — 2026-09-18

Four asks the night before the §7.17 flight test, in order. Data directories:
`l1_4d_xtune_20260918/` (tune), `wb_vs_decoupled_20260918/` (speeds + comparison).

#### 7.18.1 The along-arm (x) settling — measured, swept, NOT fixable by a gain

`l1_4d_xtune_20260918/README.md` carries the full table (`step_score.py` over the
step legs: peak during the move, t20/t50 = last time the along-step CoM error exceeds
20/50 mm after the move, the envelope ratio between hold seconds 8-12 and 0-4, the
settled rms). Baseline: after a 0.5 m step ALONG the arm the CoM error carries a
~0.15 Hz, lightly damped oscillation (envelope 0.4-0.6) and is still >20 mm at the end
of a 16 s hold; the LATERAL step settles under 20 mm in ~5 s (envelope 0.25). Twelve
single-change flights (`run_xtune.sh`, a new `--legs` option of the campaign driver
flies only the four translation steps, ~5 min each):

- `omega_x` 0.1, `omega_c_r` 0.25, `k_x` 24, `M_r_d_x` 0.16: no change.
- `k_R` 3/`k_w` 1.8: crash; `k_v` 28: diverging oscillation; `k_w` 1.1: worse on both
  axes; `M_r_d_x` 0.19: reverted to SAFETY 3.8 s after entry. The loop sits at its
  delay margin, as §7.14.4 found.
- `omega_c_t` 1.0 is the ONLY lever that moves the arm-axis mode (t20 10 s, settled
  7-8 mm) — and it moves the lateral axis the other way (16 s, 23-25 mm). 1.5 is a wash.

**Shipped config unchanged.** The mechanism is the ~1.1-1.4 s effective closed-loop lag
every trajectory run below measures; the x/y asymmetry is that lag through the arm-axis
coupling. A fix is a model change (observer predicting the rotor-lagged applied wrench,
or a reference lead in the planner), not a gain — see 7.18.4.

Tooling added: `application/robotic_arm/utils/wb_step_sim.py` (offline planned step
on `wb_entry_sim.py`'s plant, 4-D observer + arm-mass scale added to that tool). It
does NOT reproduce the flown step (17 mm vs ~200 mm peak; aborts at 40 ms delay) —
ordering only, like its parent.

#### 7.18.2 The decoupled rig now flies the SAME plant, planner and task

For a fair comparison the geometric+L1 rig (`start_t650_aerial_manipulator_geometric_L1_adaptive_sitl.sh`)
was moved onto the whole-body rig's plant and task path. Everything additive:

- **Plant**: the launcher runs `06_px4_direct_t650_aerial_manipulator_ros2_arm_torque.py`
  with the new `PEGASUS_ARM_COMMAND_MODE=position` (06 emulates the position servo —
  PD + gravity + an integral term, the servo's own integrator — tracking
  `isaacsim_manipulator/position_commands` THROUGH the same servo model: current
  residual, gearbox friction x1.05, arm mass x1.05, body mass/inertia x1.10 + CoM
  shift, rotor lag). Knobs come from the geometric L1 `_sim.yaml`, whose new section 1
  is a byte copy of the WB 4-D yaml's, via `scripts/indoor_sim/lib/am_plant_from_yaml.sh`
  (a verbatim copy of the WB launcher's block; the WB launchers were deliberately not
  touched before the flight test — fold them onto the lib afterwards). The base
  launcher bakes the new variable into the Isaac pane like every other knob.
- **Allocator error matched**: `alloc_thrust_coeff` 4.8495396e-05 (+20 %) → 4.7544506e-05
  (+17.6 %, = WB); `armff_mismatch_x` 0.010 → 0.0 (the CoM error is plant-side now).
  SAFETY block aligned too (`ude_disturbance_lbz/ubz` ±2 → ±10, `ude_height_threshold`
  0.4 → 0.35, `system_wd_max_tilt_deg` 40 → 20): with ±2 N the +10 % plant STALLED at
  z = 0.94 m in SAFETY and never reached the hover (measured, first attempt).
- **Arm**: `PositionController` gained `external_reference_topic`/`_timeout` (pure
  tracking of a streamed JointTrajectory, freeze on stale, no homing, the GS interface
  not advertised — TorqueControllerBase's 2026-09-05 contract). The Isaac aerial
  position yaml sets it and carries an `arm_planner` section (mode topic = the
  geometric+L1 fork's); `position_control_isaac.launch.py` starts `arm_planner`
  (`arm_planner:=false` for the old bring-up).
- **Planner + bridge** (stack script, own tmux window `planner`): `fsc_trajectory_planner`
  launched by `ros2 run` with the **WB 4-D sim yaml** as params (ONE planner config for
  both rigs) and `mode_topic`/`arm_reference_topic` remapped, plus
  `fsc_autopilot_ros2/scripts/tools/decoupled_reference_bridge.py`: the planner's
  WholeBodyReference → base position (`x_b = x_cd − R0 r_0c(q_d)`), velocity/acceleration
  (smoothed differences of the arm term), yaw (`atan2(b1_d) + π/2`) →
  `position_controller/reference`, published ONLY in DIRECT. Loopback:
  `application/robotic_arm/utils/test_decoupled_wiring_loopback.py`.
- **Ground stations**: the arm GS "EE Trajectory" tab has a Controller lamp (from the
  flight node's latched `fsc_autopilot_ros2/controller_type`) saying WHOLE-BODY /
  DECOUPLED / Baseline; every button is the same press in both DIRECT stacks. The drone
  GS shows Baseline / Decoupled / Whole-Body display names (registry names unchanged).
- **Arm-GS tab tidied (2026-09-18, user request)**: group title "End-Effector
  Trajectory" (the "Periodic Whole-Body Run" half is gone), the view's
  "<shape> · Time Scale x Of Max y" overlay line deleted (every one of those numbers is
  already in the table above it), the three status rows COMPACTED to ONE row of two
  blocks carrying a STATE only — Controller (WHOLE-BODY / DECOUPLED / BASELINE) and
  Planner (Idle / Calculating / Ready / Planned / Executing / Paused / Infeasible) —
  with the full status strings, including an INFEASIBLE reason, in the tooltips; the
  world triad 0.25 -> 0.10 of the view extent; the reference frame's label "EE Ref" ->
  "EE".
- **THE VIEW SHOWS TWO TRAJECTORIES, AND ONE OF THEM IS DRAWN TWICE — that is real
  data, measured** (`wb_wb_c24.npz`, the shipped 2-lap circle): the EE path is an EXACT
  circle (least-squares fit r = 0.7500 m, radius span over the whole run **0.0 mm**),
  but the DRONE path is the EE path minus the arm offset, and the arm's assigned q2
  sinusoid sweeps ONCE PER RUN (`ee_traj_q2_period_s` 48 s against 2 x 23.6 s laps), so
  lap 2 flies a ring **11.5 mm smaller in radius and 29 mm higher** than lap 1. Three
  rings on screen = EE (r 0.750) + drone lap 1 (0.799) + drone lap 2 (0.7875), which is
  exactly the 37 mm / 11 mm spacing the screenshot shows. Both paths were also coloured
  by their own SPEED map and a constant-speed run puts both at the top of their ramp
  (two near-identical yellows), so nothing said which line belonged to which body. Fix:
  **the drone path is now DASHED** (drawn by skipping segment groups — a dashed PEN
  restarts its phase on every few-pixel segment). To make every lap identical instead,
  set `ee_traj_q2_period_s` to ONE LAP; that doubles the arm's peak joint rate
  (~1.5 -> ~3 deg/s at s_max, well inside the planner's 0.5 rad/s bound) and has not
  been flown.
- Comparison tooling: `am_ee_compare_driver.py` (stability gate before the shape is
  selected; sets the planner's `ee_traj_*` live; records odom, joints, the stream, the
  debug array, the arm reference), `am_compare_cycle.sh`, `am_ee_compare_score.py`
  (EE position + FK heading, base position/velocity/yaw against the converted
  reference, joints; 3-D and error plots), `wb_vs_decoupled_20260918/run_compare.sh`.
  Two scorer traps: the estimator's odometry TWIST reads ~2x the true speed on this
  rig (base velocity is differenced from position instead), and yaw must be
  interpolated UNWRAPPED (a 135 deg phantom at the ±180 crossing otherwise).

#### 7.18.3 Cruise speeds for the 0.75 m circle and the 1.5 x 0.75 m figure-8

`wb_vs_decoupled_20260918/run_compare.sh`, whole-body rig, `am_ee_compare_driver.py`
(`--laps 2 --time-scale 1.0 --hover-z 1.2`, circle centred on the world origin at the EE
height, figure-8 through the EE start point with its long axis along the nose):

| run | EE speed | EE err mean / p95 [mm] | lag fit | residual after lag | EE heading mean | base yaw mean |
|---|---|---|---|---|---|---|
| circle lap 24 s | 0.220 m/s | 202 / 256 | 1.08 s | 69 mm | 9.4 deg | 9.8 deg |
| circle lap 48 s | 0.111 m/s | 121 / 143 | 1.26 s | 35 mm | 5.0 deg | 5.2 deg |
| figure-8 lap 67 s, heading 0 (long axis at −45 deg) | 0.064 m/s | 91 / 142 | 1.38 s | 35 mm | 4.9 deg | 6.2 deg |
| figure-8 lap 67 s, heading 90 (long axis at +45 deg) | 0.064 m/s | 88 / 142 | 1.32 s | 34 mm | 4.8 deg | 6.1 deg |
| figure-8 lap 67 s, heading 45 (long axis on world x) | 0.064 m/s | **ABORTED at 113 s of 138**: tilt grew 5 → 9 → 17 → 20 deg over the last third, tilt watchdog → SAFETY | | | | |
| figure-8 lap 67 s, heading 135 (long axis on world y) | 0.064 m/s | 92 / 142 | 1.38 s | 35 mm | 5.0 deg | 6.3 deg |

**THE FIGURE-8 IS ANCHORED BY ITS START TANGENT ALONG THE NOSE, 45 deg OFF ITS LONG
AXIS** (the Gerono lemniscate's tangent at theta = 0 is (A, 2B) = (0.75, 0.75)): heading
0 put the long axis at −45 deg and heading 90 at +45 deg (measured by SVD of the flown
reference), so "long axis on world x/y" needs `--yaw-deg 45` / `135`. The first two rows
above were flown before this was noticed and are the diagonal placements.

(the lap-32 circle was refused once — `run does not start/end at rest` — because the arm's
q2 sinusoid period must divide `laps x lap_time`; the driver now sets it to one cycle per
run.) **The error is a LAG: EE error ~ speed x 1.1-1.4 s** on every run, with a 34-69 mm
residual once the lag is removed, so the cruise speed is a budget, not an optimum. The
figure-8 is bounded by the planner's 0.3 rad/s yaw-rate limit (s_max 0.358 at the 24 s
default lap = 67 s per lap); its ORIENTATION barely matters (88 vs 91 mm on the two diagonals; the
on-axis rows DECIDE it: long axis on world x = the arm axis TRIPPED the 20 deg tilt watchdog at
113 s, long axis on world y completed at 92 mm) — **recommendation: take off heading 135 deg so the
long axis lies on world y** (the better-damped lateral axis of 7.18.1). For the flight test: circle at 0.10 m/s
(48 s lap) and figure-8 at 0.064 m/s (67 s lap) hold the error near 100 mm; 0.20 m/s
doubles it. The lever that would move this curve is the lag itself (7.18.1).

#### 7.18.4 Whole-body (4-D L1) vs decoupled (geometric + L1) on the same task — flown 2026-09-18

Same plant, same planner config, same EE task, same driver (`am_ee_compare_driver.py`:
SAFETY takeoff to 1.2 m → DIRECT → at least 20 s AND a stability gate (|v| < 0.03 m/s,
drift < 0.03 m for 4 s) → planner parameters → select → time scale 1.0 → Go-to-start →
Start gate → run → hold → SAFETY → land). Scored by `am_ee_compare_score.py` over the
EXECUTING window; EE heading by FK from the measured joints and attitude against the
streamed `b1_de`; base position against the bridge's converted airframe reference (same
conversion on both rigs); the estimator's odometry twist is NOT used (reads ~2x).

| run | rig | EE pos mean / p95 [mm] | lag | resid. | EE heading mean [deg] | base pos mean [mm] | base yaw mean [deg] | base z [mm] | joints rms [deg] |
|---|---|---|---|---|---|---|---|---|---|
| circle lap 48 s (0.11 m/s) | whole-body | **121 / 143** | 1.26 s | 35 | **5.0** | 129 | 5.2 | 0.4 | 1.20 |
| | decoupled | 138 / 158 | 1.46 s | 31 | 10.3 | 156 | 10.3 | 11.2 | 0.31 |
| circle lap 24 s (0.22 m/s) | whole-body | **202 / 256** | 1.08 s | 69 | **9.4** | 214 | 9.8 | 1.6 | 3.92 |
| | decoupled | 248 / 302 | 1.38 s | 52 | 20.3 | 280 | 20.3 | 11.5 | 0.87 |
| figure-8 lap 67 s, heading 90 (0.064 m/s) | whole-body | **88 / 142** | 1.32 s | 34 | **4.8** | 100 | 6.1 | 0.4 | 1.99 |
| | decoupled | 98 / 162 | 1.46 s | 29 | 11.5 | 120 | 11.5 | 11.2 | 0.22 |
| figure-8 lap 67 s, heading 135 (long axis on y, 0.064 m/s) | whole-body | **92 / 142** | 1.38 s | 35 | **5.0** | 103 | 6.3 | 0.4 | 2.03 |
| | decoupled | 97 / 155 | 1.44 s | 26 | 11.4 | 119 | 11.4 | 11.2 | 0.22 |
| figure-8 lap 67 s, heading 45 (long axis on x) | whole-body | ABORTED at 113 s: tilt 5→20 deg, watchdog → SAFETY (7.18.3) | | | | | | | |

- **EE position: whole-body better by 10–23 % on every run**; both are lag-dominated
  (1.1–1.3 s vs 1.4–1.5 s) and the lag-fitted residuals are alike.
- **EE heading: whole-body halves it**, and the decoupled rig's heading error IS its base
  yaw error to the decimal (10.3/10.3, 11.5/11.5, 20.3/20.3): the paper's constant-yaw
  specialization has no yaw-rate feed-forward, the airframe lags the turning reference, and
  the position-mode arm (0.2–0.9 deg rms to its joint reference) passes the lag straight to
  the gripper. The whole-body law's arm moves 1.2–3.9 deg rms against its joint reference to
  keep the gripper on task.
- The decoupled rig holds altitude ~11 mm LOW on every run (its L1 settling against the
  +17.6 % allocator error) vs 0.4–1.6 mm for the whole-body observer.
- Both took off through the same SAFETY cascade, which on this +10 % plant swings ~1.5 m
  sideways at t ≈ 8–12 s before settling (pre-existing, both rigs, before the gate).
- ONE flight per configuration. Report: artifact "AM-T650 Flight-Test Prep"
  (`wb_vs_decoupled_20260918/report.html`, built by `make_report.py`).

#### 7.18.5 The tuning question settled ON THE DEMO TRAJECTORY, and the safety guard during it (2026-09-18)

**`omega_c_t` 2 -> 1.0 is a STEP-RESPONSE result and does NOT carry to the demo.** 7.18.1's
table is four 0.5 m base steps; the demo is a tracked trajectory, and the two rank
differently. Same circle (r 0.75, lap 48 s, 2 laps), same plant, one flight each:

| | shipped `omega_c_t` 2.0 | candidate 1.0 |
|---|---|---|
| EE position mean / p95 / max [mm] | **121 / 143 / 149** | 130 / 163 / 179 |
| EE lag fit | **1.26 s** | 1.34 s |
| EE heading mean / max [deg] | **5.0 / 6.4** | 5.4 / 8.9 |
| base position mean / max [mm] | **129 / 157** | 139 / 186 |
| tilt max [deg] | **1.48** | 5.22 |
| joint rms q4 / peak [deg] | **1.61 / 2.08** | 2.23 / 6.24 |

The candidate is worse on every column and costs 3.5x the tilt, which is consistent with
7.18.1's own reading: a slower translational observer bought the arm axis and paid for it
on the lateral one, and a circle drives both axes every lap. **The shipped configuration
IS the best tuned result for the demo; nothing was changed.**

**THE SAFETY GUARD, CHECKED DURING THE TRACKED TRAJECTORY (2026-09-18, user request).**
What it does on a trip, first, because it is the question people get wrong: **it HOLDS
STILL, it does not land.** `switchMode(kSafety)` re-seeds `outer_ref_` to the pose of that
instant, PX4 regains attitude+rate+mixing, the trajectory planner goes silent on the mode
edge, and `arm_planner` folds the arm home 1.0 s later (the delay is so the arm's reaction
torque arrives after PX4 has the attitude). **Landing is a DIFFERENT failsafe**: only
`feedbackLost()` (odometry stale, or EKF2 stopped fusing external vision, gate
`system_fb_timeout_s` 1.0 s) descends, at `system_fb_land_speed` 0.2 m/s, holding x/y at
the pose where it tripped — and if PX4's own fused odometry is ALSO stale it does not
descend at all, it holds and says TAKE MANUAL CONTROL.

**It is armed throughout the trajectory, and that is structural, not incidental**: the
tilt/rate check runs at the top of every DIRECT inner-loop tick, and the drift check runs
in the same tick AFTER the reference is selected, so it measures the law's CoM error
against the STREAMED plan — a planned 0.5 m move is not drift, only losing the vehicle is.
One trip latches; only a deliberate `set_direct_mode(true)` re-arms it.

Three flights, and the node's own log quoted:

| flight | what tripped it | what followed |
|---|---|---|
| `wb_c48_guardtrip` (drift limit tightened to 0.12 m) | `excess drift (horizontal CoM error 0.12/0.12 m)` at 1.6 s into DIRECT, 7 deg tilt | coasted out its 0.46 m/s (511 mm, +440 mm of altitude), came back and **settled 3 mm from the trip point, altitude 1.050 m = the trip altitude, arm home to 0.7 deg, planner STOPPED** |
| `wb_c48_diverge` (`wb_k_r` 3.0 / `wb_k_w` 1.8, the gain that crashed in 7.18.1) | `excess tilt (tilt 20.1/20.0 deg, rate 96 deg/s)` | **flipped anyway**: 47 deg at +0.5 s, 124 deg at +1 s, on the ground at +2 s |
| `wb_c48_midrun_abort` (SAFETY commanded 30 s INTO the tracked circle) | operator abort, the same `switchMode` path | see MIDRUN below |

**Read the second row carefully: the guard is a FALLBACK, NOT A RESCUE.** It fired exactly
as designed, at the 20 deg limit — but the vehicle was already 95 s into a 12-20 deg
attitude limit cycle and was rotating at **96 deg/s** when it fired, and nothing recovers
that in a metre of altitude. The rate limit (360 deg/s) is far too permissive to have
caught it earlier; tilt at 20 deg was the binding one. The argument this makes is for
keeping the tilt limit LOW (early) rather than raising it, and for never starting a run on
a vehicle that has not settled — which the campaign driver enforces with its own stability
gate (|v| < 0.03 m/s and < 0.03 m of drift for 4 s), and which is why that flight never
started its trajectory at all.

**A trip cannot be forced mid-trajectory by lowering a threshold**, and that is worth
knowing before someone tries: the DIRECT-ENTRY transient (0.4-0.9 m, 6-7 deg) is larger
than anything the tracked trajectory produces (0.13-0.19 m, 1.5 deg), so every threshold
that survives entry is above the whole run. That is the correct ordering — it means no
false trip during the demo — but it makes the entry, not the run, the guard's sizing case.

**The DECOUPLED rig had no drift guard, and now does (2026-09-18).** Its fork carried tilt
(20 deg), rate (360 deg/s) and the feedback-loss failsafe, but not the sideways-drift
watchdog the whole-body fork gained on 2026-09-11 — the one failure the other two miss is
a vehicle walking away LEVEL. Added as the exact twin: `system_wd_max_drift_m`, **default
0.0 = off so every other config is byte-identical**, measured against `outer_ref_` (on the
comparison rig, the airframe reference converted from the planner's plan, so a planned
motion is not drift), sharing the same latch and the same `switchMode(kSafety)`. Set to
**0.75 m** in `..._geometric_l1_direct_actuation_t650_sim.yaml` — 4x the 0.19 m worst base
error measured on the flown circle.

### 7.19 PS4 REMOTE — guiding the end-effector with a gamepad (2026-09-20, user request; AIM → CONFIRM → MOVE flow 2026-09-24)

A fourth tab on the ARM ground station, plus a `PS4 REMOTE` toggle beside the
gripper buttons and a `Gamepad: ON/OFF` box in the Status column. The sticks
move a VIRTUAL end-effector target; nothing is sent while aiming, the vehicle
hovers. `Confirm Target` hands the target to the whole-body trajectory planner
through the SAME interface the "EE Whole-Body" tab uses (publish
`whole_body_planner/ee_target`, read `PLANNED` / `INFEASIBLE`); `Move`
executes the planned compatible transition (`whole_body_planner/send`); `Go
Home` asks the planner for the transition to the folded home pose (then Move);
`Reset Target` puts the target back on the measured end-effector. **The
planner, the law and the reference path are UNCHANGED** — every joystick move
is one ordinary rest-to-rest compatible transition.

**Why plan once, not continuously (settled 2026-09-24).** The planner's
transitions are rest-to-rest and floored at `t_min` (3.0 s), it ignores a
target that arrives while EXECUTING, and a running transition cannot be
interrupted short of a SAFETY revert. Re-planning from a moving state would put
steps into the CoM reference at every join (the §7.15 compatible-trajectory
guarantee rests on jerk/snap-continuous joins) and would turn every stick
wiggle into a commitment beside the object. Planning takes ~3–4 ms — the MOTION
is what takes time — so overlapping plans buys nothing. Aim freely, confirm
once, watch it go.

**The tab, top to bottom.** `Joystick Input` (the pad, sticks labelled with
the world direction they command; `Stick Gain` under it — Translation
**2 cm/s**, Rotation **5 °/s**: hold a stick fully for one second and the
TARGET moves 2 cm / turns 5°, the vehicle covers that on Move) beside
`Virtual Aerial Manipulator`, THE key picture (the EE Trajectory tab's 3-D
view with a 0.1 m grid on the floor and the two far walls, world (x, y, z)
beside every triad: the MEASURED Drone and EE (FK) triads from `current_base`
/ `current_ee`, the dashed **EE Target** triad the sticks move — it carries
the current EE's roll and pitch, only its position and yaw follow the sticks
— a dashed line for the move it asks for, and — ONLY once a plan exists — the
planned EE and CoM paths from the planner's latched `viz_path`, cleared when
the plan is spent, so aiming costs the view three triads and nothing else;
and a SIMPLIFIED GRIPPER on the EE Target — back frame rectangle, two finger
wedges with their yellow pad faces, a crosshair on the grasp point — because
the EE Target IS the grasp point: the model's `r_e`, 108 mm out along the claw
axis between the finger pads (pads ~68–124 mm, tips 16 mm beyond it),
dimensions measured from AM_xfwd.usda's meshes 2026-09-25; the fingers follow
the station's Gripper state, 9.7 mm inward per finger when closed; framing
0.35 m so the gripper is large enough to aim with) · `End-effector Workspace` (tab 2's side view, bearing dial and
heading dial, display only) · `End-effector Space` (`Current EE Pose` read-only
over `Target EE Pose`, which the sticks write and which can be typed) · the
four buttons · `Status`.

**Status lamp.** `AIMING — sticks move the target` (green, engaged) ·
`PLANNING…` (amber) · `PLANNED (T=…) — press Move` (green) · `Target changed
since the plan — Confirm Target again` (amber: a planned target was moved; Move
is disabled until it is re-confirmed) · `MOVING — sticks frozen until arrival`
(blue) · `INFEASIBLE — adjust the target and Confirm again` (red, reason
under it; the target stays where it is) · `Not in whole-body DIRECT` (grey).
On arrival (EXECUTING → HOLD) the target re-seats on the measured end-effector,
position AND yaw (`current_ee`'s frame has x = the claw axis, so its yaw is the
heading `ee_target` carries), and follows it until the sticks or the Target row
touch it again. **The sticks re-arm on NEUTRAL**: after any freeze (and on
engage) they take effect only once every axis has read zero, so a stick still
held when the vehicle arrives does not carry the re-seated target off — release
it, then push again (the loopback test caught the opposite behaviour, 2026-09-25).
The 3-D view opens on an oblique orbit with a 0.1 m grid (floor + the two far
walls), framed 0.6 m wide on the midpoint of airframe and gripper, and prints
world (x, y, z) beside every triad plus **Δ (dx, dy, dz) m / Δyaw** under EE
Target — at 2 cm/s a nudge reads there long before it reads as pixels.
`joy_node` (ros-humble-joy 3.3.0) republishes the pad state at 20 Hz
(`autorepeat_rate`), verified with a Python subscriber: 74 msgs / 6 s on both
`/joy` and `/uav_0/rc/input`. Do NOT judge that feed with `ros2 topic hz|echo`
piped through `timeout` on this box — both printed only "Terminated" and read as
a dead feed (2026-09-25, cost a needless joy restart).

```bash
# The 4-D launcher now starts joy_node + gamepad_input itself, in a tmux window
# named `joy`, if a pad is present. Manually, or on any other rig:
ros2 launch px4_offboard_control gamepad_input.launch.py device_id:=0 ns:=/uav_0

# Every button is also a service on the SAME code path (scriptable, testable
# without a display). Engage = the sticks are live; REFUSED unless the vehicle
# is in whole-body DIRECT:
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/set_engaged   std_srvs/srv/SetBool "{data: true}"
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/confirm_target std_srvs/srv/Trigger {}
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/move           std_srvs/srv/Trigger {}
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/go_home        std_srvs/srv/Trigger {}
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/reset_target   std_srvs/srv/Trigger {}

# Loopback test: fake planner + synthetic sticks + the real ground station,
# no Isaac, no PX4, no gamepad. 21 checks of the aim/confirm/move flow.
# ROS_DOMAIN_ID=77 runs it beside a LIVE stack without touching it (the
# stale-process guard is skipped on a non-default domain).
cd $FSC_OM_ARM_WS/src/fsc_open_manipulator/utils_custom_ground_station/test
ROS_DOMAIN_ID=77 /usr/bin/python3 test_ps4_remote_loopback.py

# Synthetic sticks alone, to drive a running station by hand:
/usr/bin/python3 joy_sim.py --ly 1.0                 # hold +X
/usr/bin/python3 joy_sim.py --pattern circle         # sweep the left stick

# VERIFY THE MAPPING ON A REAL PAD BEFORE FLYING IT. Push each stick and read
# the direction it names; nothing here commands the arm:
/usr/bin/python3 check_ps4_mapping.py
```

**Mapping VERIFIED on the real pad, 2026-09-20, over USB** — indices and signs
both, by pushing each stick against `check_ps4_mapping.py` and reading back the
direction it named. The table above is therefore measured on this hardware, not
carried over from the Bluetooth notes.

Worth keeping for the next pad, because it cost three attempts: a TIMED capture
("hold left at t=5-10 s") does not work, since the operator cannot see the
recorder's clock and the windows never line up — two runs produced extremes
that could not be attributed to a direction at all. `check_ps4_mapping.py`
removes the clock from the problem by naming the direction live, which settled
it in seconds. A wrong index or sign on some other connection is a PARAMETER
(`ps4_axis_*` / `ps4_invert_*`), never a rebuild.

**Stick mapping** (`sensor_msgs/Joy` on `<vehicle_ns>/rc/input`; on this pad
LEFT and UP read **+1.0**, confirmed over USB 2026-09-20 — 8 axes, 13 buttons,
triggers resting at 1.0 on axes 2/5; the table below is the reference, the pad
node itself is `gamepad_input` from fsc_virtual_remote_controller's MAIN branch):

| stick | axis | push | moves the target |
|---|---|---|---|
| left  | `axes[1]` | up / down    | +X / -X world |
| left  | `axes[0]` | left / right | +Y / -Y world |
| right | `axes[4]` | up / down    | +Z / -Z world |
| right | `axes[3]` | left / right | +Yaw / -Yaw |
| L1 | `buttons[4]` | press | gripper OPEN |
| R1 | `buttons[5]` | press | gripper CLOSE |
| PS / Home | `buttons[10]` | press | Go Home (plans the home transition — press Move) |

The three buttons act on the PRESS (holding does nothing more), only while PS4
REMOTE is engaged, through the same requests as the station's OPEN / CLOSE and
Go Home buttons; L1 + R1 together are ignored. Indices are parameters too
(`ps4_button_open` / `_close` / `_home`, defaults 4 / 5 / 10).

**The gripper in Isaac (2026-09-25).** The Isaac arm stack has no ros2_control
gripper interface, so until now every OPEN/CLOSE ended in "Gripper … ignored: no
action server", and 06 pinned the asset's `gripper_joint` at 0° with a 1e6 drive.
Now `open_manipulator_x_isaac_bridge`'s `isaac_gripper_action_server.py` (started
by both Isaac arm launch files, node `isaac_gripper`) serves
`gripper_controller/gripper_cmd` in the arm namespace — the action the station
and L1/R1 already call — and drives 06 over a gripper servo bus beside the arm's:
`/uav_0/isaacsim_manipulator/gripper_command` (Float64, hub angle rad) and
`…/gripper_state` (JointState, 50 Hz). 06 slews the drive at 0.5 rad/s.
Mapping: the station's 0.019 m (open) → **0°**, −0.010 m (closed) → **−50°**,
01's flown pick-and-place convention on the same joint. The asset's linkage
(hub + two slider-cranks, crank 15 mm, rod 40 mm), solved from its authored
joint frames: pad separation 74.2 mm at 0°, 54.7 mm at −50°, 100.6 mm at +50°
(negative closes). Results follow `GripperActionController`: SUCCEEDED on
arrival, ABORTED + `stalled` when the fingers stop short on an object (the
station keeps CLOSE lit — a grasp), ABORTED if Isaac publishes no gripper state.
Needs the sim restarted (06 and the arm launch changed).

If a stick moves the wrong pair of numbers, the axis order differs on that
connection: the pad picture shows the knob where the pad reports it, and
`ps4_axis_lx/ly/rx/ry` + `ps4_invert_lx/ly/rx/ry` fix it as ground-station
parameters, without a rebuild.

**Workspace saturation** is tested against the planner's OWN published envelope
(`whole_body_planner/workspace_rz`), per axis, so a target pressed into the
boundary SLIDES along it; the blocked axis turns red in the Target row and the
side view shows the target against the reachable region. That envelope is a
NECESSARY bound only — it is swept at q1 = 0, so it says nothing about arm-yaw
or wrist-roll range, and the planner can still answer `INFEASIBLE` for a point
inside it. It does, the reason is shown, and the target stays put to be
adjusted (at the folded home, DOWN is the direction with room — §7.15).

Two implementation notes worth keeping. The (r, z) test uses
`r = hypot(dx, dy)`, which is invariant under rotation about z, so the base yaw
— and the `R0_model = R0_actual*Rz(-90)` frame offset every other conversion on
this rig has to get right — cannot corrupt it; only the base POSITION enters.
And a `PLANNED` counts as the answer to Confirm/Go Home only if a status
message arrived AFTER the request (the panel counts them): the previous plan's
latched `PLANNED` is still on the wire when the next request goes out, and
without that check it passed as the new one and enabled Move against a plan
the planner no longer held (caught by the loopback test, 2026-09-24).

Files (all additive; `ps4_panel.{hpp,cpp}` are new, the rest are hooks):
`utils_custom_ground_station/src/ps4_panel.*`, `state_panel.*` (the toggle
column + the Gamepad box), `control_panel.*` (the tab), `joint_plot_window.cpp`
(the signals), `ee_trajectory_panel.*` (`Traj3DView` gained the `EeTarget`
frame, `setFocus` — a locked framing — and `setEmptyHint`), and
`test/{test_ps4_remote_loopback,fake_whole_body_planner,joy_sim}.py`.
NOT yet flown against Isaac or the real planner.

#### 7.19.1 Run sequence — shiqi_machine (`shiqi-desktop`), Isaac simulation only

Each step in its own terminal; never chain step 0 with a launcher. Plug the pad
in before step 1. The Pegasus launcher sees the running gamepad node and does
not open a second one.

**Step 0 kills the pad too (since 2026-09-25), so step 1 comes AFTER it — every
time.** Step 0 wipes `/dev/shm/fastrtps_*`, which breaks the shared-memory
links of any ROS 2 process that survives it; a pad started before the clean
slate then publishes into the void (`rc/input` reads 0 Hz on the ground station
while `/joy` looks alive). Never restart the pad, or any other node, while the
vehicle is in whole-body DIRECT: the 2026-09-25 flip followed a mid-flight pad
restart by 32 s (the planner's reference stream to the law dropped to ~3 Hz from
that instant; cause not isolated). Land, or abort to SAFETY (step 6), first.

```bash
# 0. clean slate            (any terminal — BOTH lines, this order, as TWO separate calls)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh
~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y

# 1. PS4 joystick           (terminal 1 — leave running)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch px4_offboard_control gamepad_input.launch.py device_id:=0 ns:=/uav_0

# 2. ROS 2 stack            (terminal 2)
~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0

# 3. Pegasus / PX4 SITL     (terminal 3 — also the arm stack + arm ground station)
~/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh shiqi_machine

# 4. OFFBOARD, then arm     (terminal 4 — order is mandatory)
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 service call /uav_0/rc/offboard std_srvs/srv/Trigger {}
sleep 2
ros2 service call /uav_0/rc/arm     std_srvs/srv/Trigger {}

# 5. PS4 remote on / off    (terminal 4 — after take-off and whole-body DIRECT from the drone ground station)
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/set_engaged std_srvs/srv/SetBool "{data: true}"
ros2 service call /uav_0/fsc_open_manipulator/ps4_remote/set_engaged std_srvs/srv/SetBool "{data: false}"

# 6. abort back to SAFETY   (terminal 4 — have it ready)
ros2 service call /uav_0/fsc_autopilot_ros2/whole_body_direct_actuation/set_direct_mode std_srvs/srv/SetBool "{data: false}"
```
