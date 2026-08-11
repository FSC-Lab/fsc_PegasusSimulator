# Aerial-Manipulator Simulation — Command Reference

Running record of the commands used to launch, batch-run, and post-process the
**aerial-manipulator** (X650 quadrotor + 4-DOF arm, `AM_realign.usda`) scenarios.
Scope is deliberately limited to this scenario — the bare-X650 / slung-load
launchers under `scripts/indoor_sim/` and `scripts/outdoor_sim/` are not covered here.

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

## 1. The five aerial-manipulator rigs

| Rig | Launcher | Isaac entrypoint | Controller | Rotor path |
|---|---|---|---|---|
| **Free-flying in-process** *(current default)* | `start_aerial_manipulator_free.sh` | `02_aerial_manipulator_free.py` | `controller.py` + `config/free.yaml` (no posture anchor, GMO observer) | direct `input_ref` |
| **Pick-and-place in-process** | `start_aerial_manipulator_pick.sh` | `02_aerial_manipulator_pick.py` | `controller.py` + `config/pick.yaml` (same law, `pickplace*` plans in utils_planner) | direct `input_ref` |
| Track in-process | `start_aerial_manipulator_track.sh` | `01_aerial_manipulator_track.py` | `controller_track.py` (posture anchor, no observer) | direct `input_ref` |
| Hover, ROS 2 two-process | `start_aerial_manipulator_hover.sh` | `01_aerial_manipulator_hover.py` | `controller_hover.py` (separate node) | ROS 2 topics |
| Free flight + PX4 direct-actuator | `start_px4_direct_aerial_manipulator_free.sh` | `03_px4_direct_aerial_manipulator_free.py` | `controller.py` + `config/px4_direct_free.yaml` in-process, rotors gated by PX4, lagged motors | PX4 `ActuatorMotors` via bridge |

Each launcher opens a **new terminal window** and takes exactly one argument.

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

### 1.3 Track in-process demo (posture-anchor baseline)

```bash
./scripts/start_aerial_manipulator_track.sh shiqi_machine
```

Log: `/tmp/aerial_manip_inproc.log`; npz: `…/robotic_arm/results/log/track_log.npz`.

```bash
$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/track_log.npz --tag track
```

### 1.4 Hover demo (pure ROS 2, two processes)

```bash
./scripts/start_aerial_manipulator_hover.sh shiqi_machine
```

Two tmux panes: Isaac Sim + the `controller_hover.py` node under the `/uav_0`
namespace. Log: `/tmp/aerial_manip_isaac.log`. This demo does **not** write an
npz — it predates the history logging, so there is nothing to plot.

### 1.5 GMO with the PX4 direct-actuator rotor path

```bash
./scripts/start_px4_direct_aerial_manipulator_free.sh shiqi_machine
```

Three tmux panes in session `am_gmo_px4` (the script attaches automatically):
PX4 SITL `none_iris` + `MicroXRCEAgent udp4 -p 8888`, Isaac Sim, and the rotor
bridge. PX4 runs **no** control loops — it is only the arming/offboard gate.
`apply_aerial_manipulator_px4_offboard_params.sh` fires 15 s in.
Logs: `/tmp/am_gmo_px4_{isaac,px4,bridge,xrce}.log`; npz: `…/robotic_arm/results/log/gmo_px4_log.npz`.

To reattach after detaching (`Ctrl+b d`):

```bash
tmux attach -t am_gmo_px4
```

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

To change it without editing the file, override per run through `AM_SWEEP`:

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
| GMO in-process (§1.1) | `$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/gmo_log.npz` | `gmo` |
| Pick-and-place in-process (§1.2) | `$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/pick_log.npz` | `pick` |
| Track in-process (§1.3) | `$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/track_log.npz` | `track` |
| GMO + PX4 direct-actuator (§1.5) | `$ISAAC_PY $AM/utils_plot/plot_results.py $AM/results/log/gmo_px4_log.npz` | `gmo_px4` |
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

## Notes

- `CLAUDE.md` refers to `scripts/indoor_sim/start_aerial_manipulator.sh` with
  `direct` / `px4-offboard` modes. **That file does not exist in this repo** —
  the five launchers in §1 are the real entrypoints.
- `02_aerial_manipulator_free.py`'s header mentions `utils/gain_sweep.py`; the
  only sweep harness actually present is `utils/px4_gmo_gain_sweep.py` (§4),
  which targets the PX4 rig.
- Isaac's `python.sh` resets `PYTHONPATH`, so a bare-module import plus
  `PYTHONPATH` does **not** survive into the process. The demos use
  fully-qualified `fsc_aerial_manipulation.*` imports for this reason — keep any
  new module inside the editable-installed package rather than adding a path hack.
