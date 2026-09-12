# ★★★ BRANCH `slosh` — FREE-SURFACE TANK, PHASE 1 DONE (2026-09-12)
# Plan: https://claude.ai/code/artifact/7d8de8f4-dde5-4efa-b16e-fc97fb217d78
# This section exists only on the `slosh` branch. `pre-slosh` is the baseline.

**THE BASELINE IS A TAG IN FOUR REPOSITORIES, NOT ONE.** `pre-slosh` is cut in
fsc_PegasusSimulator (ea74406), fsc_autopilot_ros2-master (88127e2),
fsc_autopilot_ros2_msgs-master (ebeded8) and ros2_ground_station_gui (971fba6);
each tag message pins the other three. Checking out one of them alone does NOT
restore the flying stack. Bags live in `~/flight_bags` (db3 only, no
metadata.yaml -- read them directly, `ros2 bag` will not open them).

**THE MODEL COPY IS A SUBCLASS, DELIBERATELY.** `logic/vehicles/vtol_slosh.py`
(`VTOLSlosh` / `VTOLSloshConfig`) adds the tank to `vtol.py` and nothing else;
the baseline plant keeps ONE definition. A fork would inherit every plant bug
the day it is fixed in vtol.py and not here -- which is this project's most
expensive recurring failure (the FLU/FRD frame, the body-link inertia, CQ at
half value). Freeze a plant by checking out the tag, not by copying the file.

**THE SLOSH MATHS IS ISAAC-FREE ON PURPOSE:** `logic/dynamics/slosh.py`
(`TankConfig`, `slosh_parameters`). `vtol.py` cannot be imported outside Isaac,
so every headless test of it so far RE-DERIVED the maths (see
`test_actuator_models.py`) -- the same drift trap again. The slosh tests import
the shipped module by file path and check the real code. Do not add
`from __future__ import annotations` to it: with a by-path load, @dataclass then
resolves annotations through an unregistered `sys.modules` entry and the import
dies with a bare AttributeError.

**A CONFIGURED TANK REFUSES TO CONSTRUCT** (phase 1 has no slosh physics). It
raises at construction, not on the first physics step, so it can never be
airborne when it is discovered. `tests/test_slosh_scaffold.py` (AST + the
plan's numbers) and `tests/test_slosh_import.py` (real import, Isaac stubbed)
both PASS; the scaffold test is MEANT to fail at phase 3 and says so.

**SELECT IT WITH `VTOL_SLOSH=1`** on scene 03 -- same airframe, `tank=None`, so
it is step-for-step the baseline. Fly the mission that way and compare against
`pre-slosh` BEFORE any tank physics exists to blame. The spawners resolve the
vehicle class from the CONFIG TYPE (`_vehicle_class_for`), one selector, so a
slosh config cannot be flown on the baseline class or the reverse.

**★★ FLOWN, 2026-09-12. THE COPY FLIES THE FULL MISSION.** `VTOLSlosh` spawned in
Isaac and flew climb -> transition -> 90 deg wing-borne cruise -> back-transition ->
**landing 0.00 m from the commanded point at 800 m**, scored from the bag by
`night_metrics.py` (bags + `results.tsv` in `~/flight_bags/slosh`):

| trial | model | verdict | wb_s | peakV | tilt | maxroll | maxyaw | fails |
|---|---|---|---|---|---|---|---|---|
| slosh_p1b_165344 | **VTOLSlosh** | **LANDED** | 47.3 | 16.7 | 90 | 13 | 15 | 0/0 |
| base_p1_170214 | baseline VTOL | **CRASHED** | 47.1 | 16.7 | 90 | 180 | 180 | 0/0 |

Same driver, same 30 m / 800 m / 70 s mission, ONE FLAG APART. Wing-borne seconds,
peak speed and tilt match to within noise (47.3 vs 47.1 s; 16.66 vs 16.64 m/s); the
BASELINE is the one that departed. That is the documented P-only bank-to-turn
scatter, not a difference between the models -- the force path is provably
identical -- but it is worth stating plainly: **one flight each proves the copy is
not worse, and nothing more.** A lateral claim from this stack needs repeats.

**AND THE FIRST ATTEMPT WAS A NULL FLIGHT THAT LOOKED LIKE A CONTROLLER FAILURE.**
A full auto mission ran during stack BOOT (climb, engage cascade, 90 deg cruise,
then a roll departure at -124 deg and the ground at x=351 m). The driver then flew
the WRECK for three minutes, reporting 0.00 m/s and "climb did not settle" -- and
Isaac resurrects a crashed vehicle, so nothing downstream noticed. `night_campaign`
already knew this ("the vehicle must be fit, not just the stack"); the new driver
now refuses unless `vehicle_state.py` says the aircraft is at the origin, upright,
grounded and stopped. **The bag is what told the truth, for the fifth time.**

# ★★★ PHASE 2 FLOWN (2026-09-12): TANK AS DEAD MASS. CRUISES; THE BACK-TRANSITION DEPARTS.
**THE MASS, FROM THE CONFIG'S OWN LINKS** (`vtol_slosh.assembled_mass_properties`): the
baseline assembles to M=3.450 CG=(0.1764,0,-0.0353) I=(0.265,0.303,0.490) -- reproducing the
controllers' hardcoded constants EXACTLY, which is what makes it credible for the tank:
**M=3.9516 kg, CG unmoved (0.00 mm), I +0.2%**. Mounted at the assembled CG the tank changes
ONE thing. `TankConfig.position=None` now means the assembled CG, NOT the body-link origin
(176 mm apart -- the `aero_center` bug in a new coat). Stall 14.13 -> **15.12 m/s**.

**THE VEHICLE MASS HAS FOUR COPIES IN THE CONTROLLER PACKAGE** -- `so3.M`, `FSC["m"]` (the
trim generator), `geometric_hover_node.mass` (the SAFE controller), `vtol_nmpc.M` + its BAKED
TRIM table -- plus `tests/plant_double.py` on this side. Moving only `so3.M` left a **1.245
m/s^2 vertical residual in every baked reference: 0.502 kg x g, the water exactly**.
`tests/test_tank_model_pairing.py` reads all of them and fails on drift; the trim table was
REGENERATED from `nmpc6.Schedule` (hover row 33.84 -> 38.76 N). All at 3.9516 on the
controller's `slosh` branch (paired with this one; baseline = `pre-slosh` in BOTH repos).

**`i_max` 8 -> 12 N ON THE SAFE CONTROLLER -- SAFETY-RELEVANT, REVIEW BEFORE HARDWARE.** Measured
on the plant double: a 6 m/s crosswind needs **9.38 N** of integral force at 3.95 kg; at the 8 N
clamp it PINS and the aircraft parks 0.46 m downwind forever (0.004 m at 12). The 8 N value was
this same finding one payload ago. Anti-windup is clamp + back-calc + freeze, not bare.

**GATES AT 3.9516 kg:** reference audit all pass; cascade audit |qdot| 0.051, az 0.000; 26/26
geometric-hover; offline cascade gate 89.7 deg / 16.00 m/s / **0/1801 fails**, cruise AoA
**+7.1 (was +5.7)**, rotors 10.0 N (was 8.1). **Corridor peak AoA 12.4 deg vs the CL peak at
10.0** (baseline 10.7): ~10 s past the peak at t=20-32 s. Held altitude offline; that is the
back side of the curve where flights 12/13/15 were lost. A quicker `ttr` lowers it (CLAUDE.md:
ttr 6 -> 6.7 deg) at the cost of the lateral clean window. Not retuned -- a mission-design call.

**HIL (bag slosh_20260912_173754, results.tsv row tank_p2_173901): CRUISE EXCELLENT, BACK-TRANSITION
CRASHED.** ~50 s of 90 deg cruise at V 16.3-16.6, z 30.0-30.1 FLAT, AoA +5.6..+7.2, wing
105-119%, roll +-1, **fails 0 throughout**. Then, from the cascade's own log:
```
V=16.41 vcmd=16.00 tilt=90.0 th= +6.0   fails=0      cruise
V=15.55 vcmd=15.13 tilt=64.6 th=+11.7   fails=0      back-transition begins
V=14.60 vcmd=13.63 tilt=35.6 th=+19.2   fails=0      BELOW the 15.12 m/s stall, nose +19
V= 7.75 vcmd=12.13 tilt= 8.3 th=+56.0   fails=18     lost 7 m/s in ONE second; solver failing
V= 3.49                     th=+36.3   fails=36  -> th -57, +83, roll +177: tumble, ground
```
The SAME schedule on the no-tank copy 40 min earlier (row slosh_p1b): V 16.3 -> 3.95 over 8 s
= the commanded 1.5 m/s^2, AoA never above +18, tilt walked to -6.8 aft, **landed 0.00 m**.
**MECHANISM: the flare pitches up at V~15 either way; the no-tank aircraft has 0.9 m/s of stall
margin there and the tank aircraft has -0.5.** A stalled wing drops its lift, the schedule holds
altitude by pitching MORE, and it runs to +56 in a second. The plan's +0.88 m/s cruise margin
is exactly what the back-transition's first second consumes. This is the "decel_rate 1.5 vs
implied -8.68 m/s^2" observation already recorded above, made unavoidable by 0.5 kg.
**PHASE-2 GATE IS THEREFORE NOT MET AS SPECIFIED.** Options, none taken yet: less water
(0.25-0.3 kg -> stall 14.6, +1.4 m/s); raise `vcruise` (check the momentum thrust model
leaves headroom at 17-18); or fix the back-transition to keep the wing above ITS stall until
the aft tilt is braking -- the underlying defect, and the durable fix.

# ★★★★ THE TANK AIRCRAFT FLIES THE FULL MISSION -- WITH A RE-SEQUENCED BACK-TRANSITION
# (2026-09-12, bag slosh_20260912_182643, `nmpc_cascade_tank_node.py`). LANDED 0.00 m.
**THE FIX IS SEQUENCING, NOT GAINS.** From the crash bag at full rate: at decel onset the
aircraft pitched +7 -> +18.5 in 0.4 s against a reference rate-limited to ~3 deg, then a
~2.5 Hz pitch oscillation that GREW. Offline at 5 Hz with 0.1 deg noise the outer solver's
own planned pitch HUNTS +7..+22 (finding C9's flat-optimum hunting, on pitch), seeded by a
flare that asks trim+15 = 22 deg of a wing that peaks at 10. **Capping the flare alone made it
WORSE and reproduced the crash exactly**: the base schedule sends the front pair AFT from the
first tick, while the wing is what carries the weight; the tank aircraft's wing quits at
15.12 m/s with the tilt still at 60-70 deg, the outer has no feasible way to hold altitude,
its solves fail, the fallback freezes the tilt, the nose ratchets to +88.
**Schedule now: rotors UPRIGHT first (60 deg/s); aft tilt only below 60% wing share (the
trim split then gives the rotors only the wing's deficit -- no balloon); pitch capped at
12 deg + (1 - wing share) x flare.** Offline with noise, seeds 1-3, noise 0.3/2, latency
0.05/comp 0.049: peak pitch +13..+18, altitude >= 29.3, stops every time, <= 10 fails/1701.
**BOUNDARY: decel_rate 2.5 crashes it** (1.5 flown). Flown: 17.4 -> 3.3 m/s over 9 s,
altitude 28.8-30.1, pitch <= +13, rotors upright at V=13, aft tilt at V<7, 0 solve fails
after the onset, hover, 1 m/s descent, **touchdown (800.00, 0.00) level**.

| flight | aircraft | cascade | verdict | wb_s | peakV | maxroll | maxyaw |
|---|---|---|---|---|---|---|---|
| slosh_p1b | copy, no tank | base | LANDED 0.00 m | 47.3 | 16.7 | 13 | 15 |
| base_p1 | baseline | base | CRASHED (roll, in cruise) | 47.1 | 16.7 | 180 | 180 |
| tank_p2 | tank 3.95 kg | base | CRASHED (back-transition stall) | 48.2 | 16.7 | 180 | 180 |
| **tank_seq** | **tank 3.95 kg** | **tank** | **LANDED 0.00 m** | 48.4 | 19.3 | 35 | 99 |

**CAVEATS, stated plainly.** (1) ONE flight. (2) The cruise carried the documented P-only
bank-to-turn limit cycle at +-29 deg roll / +-23 m cross-track / V 16.6-19.3 (banking raises
speed; 19.3 is where the 12x6 prop saturates). Run-to-run scatter of the marginal lateral
mode -- the previous tank flight cruised at +-1 deg on the same config -- NOT the tank or the
schedule, but it is the open defect and the tank plan's phase-7 note stands. (3) The 99 deg
yaw was the low-speed cross-track chase; `night_metrics.py` scored it CRASHED via
"maxyaw>90 and reached the ground" until that rule learned to look at HOW the ground was
reached (slow, upright = landing). Every earlier verdict re-scored unchanged.
**L1 ADAPTIVE IS NOT NEEDED FOR THE DEAD-MASS CASE**: mass paired into the model + the
schedule re-sequenced covers it. Where it may matter is phase 3, when the slosh moves the
effective CG dynamically. Deliberately not started.

**THE STACK DIED MID-FLIGHT TWICE TODAY (16:48, 18:20), every pane in the same second = the
tmux server, cause unknown; and a GS selection made during a scripted climb fought the
driver at 1 Hz over active_controller** (the GS re-publishes its last selection forever,
`trajectory_player.py:1894`). Ops answer: `run_slosh_flight.sh` = teardown -> launch ->
verify -> close the GS -> fly SUPERVISED (PX4 + Isaac alive, mocap changing; dead or
frozen = exit 2) -> score -> teardown, retrying on exit 2. One command; it does not need
anyone to diagnose why the stack died to get a result.

# ★★★ LATERAL LIMIT CYCLE: FIXED IN FLIGHT BY HEADING-RATE DAMPING ON THE BANK COMMAND
# (2026-09-12, bag slosh_20260912_194108). k_bank_d, declared since the noise campaign, was
# INERT -- "a yaw-rate term does not fix it" was measured on a knob the law never read.
**MEASURED, from the ±29 deg flight (bag ...182643) at 120 Hz:** the guidance loop oscillates at
**0.9 rad/s (7.1 s period), growing x1.8 per cycle (+0.08/s)** until the 25 deg bank limit bounds
it; yaw a quarter period behind roll, cross-track a quarter behind yaw, ailerons +-0.05. That is
exactly the ideal loop's sqrt(g*k_bank*k_ct) = 0.99 rad/s; the DAMPING is not (ideal zeta 0.6,
flight unstable) -- the real roll->heading path carries ~60 deg more lag at 0.9 rad/s than the
model, and a P-only command cannot supply phase. **The offline gate does not reproduce the mode
at all** (worst roll 0.5 deg over 200 s with noise, 1.8 with a 15 m seed): it was designed from
the bag and validated in flight. The open-loop lateral modes at the trim (CasADi Jacobian of
`so3.build_ode`): dutch roll 0.78 Hz zeta 0.21, roll subsidence 0.2 s, spiral -0.08/s -- the
7 s mode is the GUIDANCE loop, not the airframe.
**THE FIX:** `roll_ref = wing * (k_bank*e_yaw + k_bank_d*psi_dot_est)`, `psi_dot_est =
r/(cos(phi) cos(theta))` -- NO pitch-rate term, which is what turned the two earlier turn-rate
rewrites into pitch-into-roll coupling. k_bank_d=1.0 s = 24 deg of lead at 0.9 rad/s.
**CRUISE-ONLY: off the moment decel starts.** Left on through the back-transition it cost 1-2
of 6 seeds on the gate; and that gate showed the back-transition's stall crossing (V 15 -> 12)
on a KNIFE-EDGE: runs identical to 0.1 deg at t=61 diverge to +84 or hold by t=62, ~1 in 6-8
under noise + a 10 m cross-track history, whichever lateral gains. That margin is the open item.
k_ct 0.03 tried (lower loop frequency) and cost a seed: left at 0.05.

| flight | maxroll | cruise \|roll\| | cross-track | V |
|---|---|---|---|---|
| tank_seq (P-only) | 35 | ±29 growing | ±23 m | 16.6-19.3 |
| **tank_lat (+k_bank_d)** | **7** | **±5.5 flat (5.1/5.5/4.3 per 20 s)** | **6.3 m** | 15.5-16.7 |
Both LANDED 0.00 m. One flight each.

# ★★★ PHASE 3 — THE PENDULUM IS IN THE PLANT (2026-09-12). Tests pass; NOT YET FLOWN.
`dynamics/slosh.py::PendulumSlosh`: 2-DOF pendulum in the body frame driven by the vehicle's
SPECIFIC FORCE `f = R^T(a - g_vec)`; restoring "gravity" is the AXIAL specific force (`f_z/L1` =
omega1^2 at 1 g; free fall has no mode, 2 g rings sqrt(2) faster). **The force applied is
`-m1 * x1_ddot` ONLY** -- the whole liquid stays folded as dead mass (phase 2), and the bob's
motion adds just what a moving bob does beyond a frozen one, which is identically zero in every
steady state (hover, banked turn, constant acceleration) -- no double count, tested. Semi-
implicit Euler at the physics step. `tests/test_slosh_pendulum.py`: 3.325 Hz vs closed form
3.323; decay 10 -> 1.31 mm in 20 s as the 0.49% zeta predicts; undamped energy bounded 1.7%
over 200 cycles; steady 2 m/s^2 lateral -> 0 N slosh force, bob at -f/w^2. `_apply_slosh`
applies it at the tank (the assembled CG); `PEGASUS_SLOSH_DEBUG=1` prints [SLOSH] lines.
Select with `VTOL_SLOSH=1 VTOL_TANK=slosh`. Phase 4 (pendulum in the offline gate) next.

**OPS — `scripts/start_slosh_hil.sh` / `scripts/kill_slosh_hil.sh`.** The launcher
delegates to `restart_hil.sh` (one real launcher) and adds what a bare
`VTOL_SLOSH=1` cannot: it refuses on a branch with no `vtol_slosh.py`, runs both
gates first, and then **VERIFIES that Isaac logged the model banner** -- the env
var travels INLINE on a tmux send-keys line, a pane does NOT inherit the calling
shell, so a missing forward would fly the BASELINE while the operator believes
the tank is in. `--baseline` flies the baseline down the same path (the A/B is
one flag), `--dry-run` checks everything that does not need a stack.
Bags go to `~/flight_bags/slosh`, NOT `/tmp/bags` -- which is still the default
everywhere else and is how the post-extraction verification bag was lost.
The kill script wraps `kill_hil.sh` (the pattern list has ONE home) and adds two
things: it **SIGINTs the recorder first** -- `kill_hil.sh` -9s it, and a
SIGKILLed bag has no metadata.yaml, which is why every bag in `~/flight_bags` is
db3-only -- and it **exits non-zero if anything survived**, since kill_hil.sh
prints survivors and exits 0 either way. It refuses to run from inside tmux
(`kill_hil.sh` ends in `tmux kill-server`, which would kill the checking shell).

**THE NUMBERS THAT DECIDE THE DESIGN** (recomputed by the test, not assumed):
water is capped near **0.5 kg** by cruise margin (+1.0 kg puts stall at 16.05
against a 16.0 m/s cruise -- one litre does not fly); the default 41 mm tank
rings at **3.32 Hz, 2.34x the 1.42 Hz short period**; and it is **0.49% damped**
-- 23 cycles to halve, below the elevator servo corner, i.e. inside the loop's
reach. `TankConfig.baffled()` exists from day one for that reason.

# PegasusSimulator — Developer Checklist

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

---

# ★★★★ ACTUATOR MODELS: CONFIGURABLE THRUST + PROP INFLOW + SURFACE SERVO (2026-09-10)
# `vtol.py`. ALL DEFAULTS REPRODUCE THE LEGACY BEHAVIOUR EXACTLY -- nothing is re-tuned unless
# you opt in. `tests/test_actuator_models.py` (headless, no Isaac) guards the maths.

**WHAT THE SIM USED TO MODEL** (and the FSC airframe does not override `rotor_dynamics`, so this
was what every flight flew):

| | modelled | present but DISABLED |
|---|---|---|
| thrust | `throttle * max_thrust`, instantaneous, LINEAR | `rotor_tau_spin`=0.12 s, T ~ omega^2 |
| tilt | pure rate limit 80 deg/s | `tilt_tau`=0.06 s lag |
| surfaces | **nothing at all** | -- there was no surface model |

Worse, `_apply_aerodynamics` read the RAW COMMAND, so the surface moment appeared the instant the
command did and the hinge joint's own response was decorative. A controller could slew a surface
at any rate it liked -- and one did: the HIL elevator sign-flipped near full deflection every
~0.25 s, which needs several hundred deg/s to be physical.

**`thrust_model`: "instant" (default) | "lag" | "momentum".** `rotor_dynamics=True` is kept as an
alias for "lag". `CT0` is DERIVED from each rotor's own `(max_thrust, rotor_omega_max,
prop_diameter_m)`, so STATIC thrust is preserved EXACTLY by every model and only the airspeed
behaviour is new. "momentum" adds the inflow term, linear in advance ratio `J = V_ax/(nD)`:
`T = rho*D^2*( CT0*(nD)^2 - (CT0/J0)*(nD)*V_ax )`, clipped at zero, with `V_ax` the airspeed along
**this rotor's TILTED thrust axis** -- at 90 deg tilt that IS the cruise airspeed.

**★★ CONSEQUENCE, AND IT IS LARGE. THE CRUISE TRIM DEPENDS ON THE FLAT THRUST MODEL.** For a
representative 12 in prop (`prop_diameter_m`=0.3048, `prop_zero_thrust_J`=0.6, sized so 18 N static
needs ~7150 rpm, `CT0`=0.12 -- squarely in the normal 0.08-0.14 band):

| V (m/s) | 0 | 4 | 8 | 12 | 16 |
|---|---|---|---|---|---|
| max thrust/rotor at FULL throttle | 18.00 | 14.70 | 11.39 | 8.09 | **4.79 N** |

and because thrust vanishes at `J = J0`, **at 16 m/s there is NO positive thrust below ~74%
throttle** -- against a controller that currently cruises at **11%**. Switching "momentum" on WILL
invalidate the cruise trim and the allocation headroom story (the inner loop's differential relies
on spare thrust) and requires re-tuning. That is the point of it: the present 8.1 N of cruise
thrust at 11% throttle is an artefact of a propeller that does not care how fast the air is
already moving.

**`surface_rate_deg_s` / `surface_tau` (both default 0 = legacy instant).** First-order lag then a
slew clamp, the order a real servo behaves in. Representative small digital servo 300-500 deg/s,
tau 0.02-0.05 s; a small stepper is slower, 100-200 deg/s. `self._surface_state` tracks the ACTUAL
deflection and **the aero now reads that, not the command** (`_surface_command_equiv`).

**ALSO FIXED HERE: `DR_MAX` WAS 30 deg WHILE THE SIM APPLIES 25 deg TO EVERY SURFACE.**
`vtol_aerodynamics` scales the normalised [-1,1] command by `max_surface_deflection_rad =
radians(25)` for aileron, elevator AND rudder, and every airframe sets `max_deflection_deg=25`.
The controllers computed `dr = M/(k*DR_MAX)` against a limit 20% too large, so **every yaw-moment
demand was delivered at 25/30 = 83% of what was asked**. Corrected to 25 in
`nmpc_so3_transition_node`. It survived because `test_controller_aero_match.py` feeds RADIANS
directly to both models -- it checks the aero FUNCTIONS agree but never exercises the
[-1,1] -> radians NORMALISATION. `tests/test_surface_normalisation.py` now does.
**NOTE this changes nmpc_full's and nmpc6's rudder gain by 1.2x -- a correction, but they were
flight-validated with the wrong value, so re-check their yaw behaviour.**

# ★★★★★ CASCADE — HIL-FLOWN, FULL MISSION, 0.1 m BACK-TRANSITION EXCURSION (2026-09-10)
# `nmpc_cascade_node.py`, controller_id `cascade`. Wired into the GS "Run Full Auto Mission".
#
# **HIL RESULT (bag casc3, third flight): TRUE FIXED-WING, 79.0 s UNBROKEN controlled+level.**
# ```
# climb 30 m -> transition -> 900 m of 90 deg cruise -> back-transition -> stop, ALL at 30 m
# cruise        V=16.00  tilt=90.0  AoA=+5.5  wing=96%  rotors=8.1N of 33.8N  roll/yaw=0
# back-trans    V 16.00 -> 0.08 m/s, tilt -8.7 deg (front pair AFT), altitude 29.9-30.1
# EXCURSION     0.1 m       (nmpc_full's best was 2.2 m; with no back-transition, 65.7 m)
# solve         2 failures in 1861 across the WHOLE mission; 0 node exceptions
# inner loop    held a full 120.0 Hz, p95 8.7 ms -- the background-thread decouple works
# ```
# HIL and offline agree to 0.2 deg of AoA and 0.02 m/s of speed. Offline the same mission is
# 0/2401 solve failures; the numbers below are the offline bench that predicted it.
The 19-state `nmpc_full` was COMPUTE-SATURATED, not mistuned: four independent stiffenings each
improved the physics and each saturated IPOPT at MAX_ITER=25, and MAX_ITER=40 put p95 at 93 ms
against a 100 ms tick. Splitting the problem buys the budget back.

    OUTER  6-state longitudinal NMPC, background thread ~20 Hz -> Tf, Tr, beta, de (+ ref pitch)
    INNER  geometric, every mocap sample                       -> roll/yaw + allocation

| | 19-state (nmpc_full) | **cascade** |
|---|---|---|
| solve | 27 ms median / 58-93 p95 **in flight** | **9-12 ms bench, whole corridor** |
| MAX_ITER | 25, saturated | not binding |
| steady state | phugoid, ~50% repeatability | **z=30.0, V=16.02, tilt=90.0, AoA=+5.7 CONSTANT 120 s** |
| lateral | mid-corridor departure at 6-10 m/s | **worst \|roll\| 0.0, \|yaw\| 0.0 over 200 s** |
| solve failures | ~20% | **0/4001** |
Recovers roll10+yaw20, roll15+yaw-25, yaw30, roll20 -- the combined upsets that crash nmpc6.

**FIVE THINGS HAD TO BE RIGHT. Each was measured, and four were bugs I introduced by omission:**
1. **DELAY COMPENSATION IS NOT OPTIONAL.** On the outer's OWN model -- perfect match, no lateral, no
   inner loop, nothing else to blame: lat 0 -> 0/1201 fails and smooth 0->90 deg; lat 0.05 ->
   87/1201; lat 0.10 -> **ground at t=35 s**. Omitting it made the cascade look like an architecture
   problem when it was a missing `predict6`.
2. **AUDIT FINDING C9 WAS NEVER BACK-PORTED TO nmpc6** (`w_beta`=12, a SOFT tilt guide). C9 says a
   soft guide leaves a flat tilt-vs-pitch optimum and the solver HUNTS -- and it does: driving the
   true plant the tilt command bang-banged **10->40->11->0->39->43->15->2->46->45->0 deg at ~1 Hz
   with ok=1 throughout** (hunting, not failing). nmpc6's OWN test shows it too. `OuterNMPC`
   subclasses with `w_beta`=60 so the stock solver and its JIT cache are untouched.
3. **★ BANK-TO-TURN. Holding HEADING with the RUDDER does not survive wing-borne flight.** A banked
   aeroplane TURNS; that is not a disturbance to cancel. Inner loop alone at the 90 deg trim, a 5 deg
   roll upset drove yaw to **-163 deg with the rudder SATURATED at -1.000** while roll recovered
   fine. Heading error now commands ROLL, blended on `(V/14.13)^2` = the wing's share of
   weight-carrying (0 in hover -> multicopter yaw, 1 wing-borne -> pure bank-to-turn). Physical, not
   a tuned crossover speed. This was a multicopter reflex applied to an aeroplane.
4. **THE PITCH SIGN INVERTED FOR THE SIXTH TIME.** `theta = asin(R[2,0])` is nose-up POSITIVE but the
   rotation about body +y that produces nose-up is NEGATIVE, so the attitude term is **+kR, not -kR**
   (the RATE term keeps its minus). Backwards it is positive feedback: pitch diverged in HOVER,
   0 -> -13 -> +17 -> +57 deg in 2.2 s. **Verify empirically every time** -- front-heavy split gives
   `omega_dot_y = -12.9` and drives theta to +58, which settles it in one command.
5. **THE PITCH AXIS SPLITS BY BANDWIDTH.** Three variants measured: both loops doing pitch ATTITUDE
   -> they fight (the bang-bang above); inner doing NO pitch -> hover unstable, ground at t=4.6 s;
   inner doing RATE DAMPING only -> parked nose-up at +12 deg CLIMBING at 1 m/s with ok=1, and
   `v_cmd` then pinned itself via the `V + v_lead` clamp. The outer owns pitch ATTITUDE, the inner
   owns roll/yaw and supplies pitch rate damping.

**THE ALLOCATION, AND WHY nmpc6's `lateral_control` GAVE AWAY AUTHORITY.** It faded differential
thrust out on `lowspd = clip((6-V)/6,0,1)` -- gone by 6 m/s -- leaving a NOTCH exactly where the
campaign kept departing: roll authority 38.6 (V=0) -> **8.7 rad/s^2 (V=6)** -> 15.5 (V=8) -> 62 (V=16).
At V=6 the airframe still HAS 8.50 N.m of differential (cos 33.8 = 0.83 of hover) and the fade threw
all of it away; keeping it gives 40.8 instead of 8.7. Its docstring claims no roll authority at
90 deg tilt -- false. Measured `cond(B_rot)` never exceeds 33.8 and roll stays **0.304 N.m/N at
90 deg (54% of hover)**, because the REAR pair is still vertical (pure roll) while the front is
horizontal (pure yaw): the axes DECOUPLE at 90 deg. The old code lost it by applying one scalar
`droll*cos(beta)` to BOTH pairs. `B_rot(beta)^-1` is solved against **dFz = 0** so the inner never
disturbs the outer's trim, and each moment is split rotor-vs-surface by ACTUAL authority (headroom
vs qbar) -- no crossover speed anywhere. `rear_floor`=4 N keeps the rear pair loaded for roll.

**LATENCY IS THE CEILING, AND IT IS THE ATTITUDE LOOP'S, NOT THE SOLVER'S.** Over-compensating does
NOT help (lat .15/comp .20 still crashes). Softening the inner gains does, and softer wins EVERYWHERE
-- it is not a trade: `kR`=3.0/`kW`=1.8 gives **0/2401 fails at lat 0.05 and 0.10** (vs 5/4001 at
kR=9) and settles on the EXACT +5.7 trim AoA instead of wandering to +10.8; at lat 0.15 kR=9 is on
the ground at 7 s while kR=3 flies the full 120 s. Limit ~150 ms at kR=3, ~125 at kR=9 -- the same
number CLAUDE.md already recorded for the safe controller ("the ATTITUDE loop goes first"). **If HIL
measures more than ~150 ms, soften kR further; do NOT raise `latency_s`.**

**nmpc6's OWN OFFLINE TEST NOW CRASHES (t=8.0 s, most solves ok=0).** It was validated 2026-08-12,
BEFORE the 2026-09-06 aero frame fix and pitch-convention flip. `build_ode6` was converted; its
TUNING and closed-loop behaviour never were. Do not quote its 2026-08-12 numbers.

**PROBE TRAP, AGAIN:** an isolation probe that sets `v = R @ [V,0,0]` puts velocity along the BODY
x-axis, so **AoA starts at 0 instead of the trim's +5.7** -- not a trim at all. The aircraft pitches
up hunting for lift and the Euler angles flip through vertical, which READS AS A LATERAL DEPARTURE
(roll/yaw to +-180) when the real event is longitudinal. Level flight means velocity HORIZONTAL IN
THE WORLD. I diagnosed a lateral instability that did not exist before catching this.

**★ THE TRANSITION CAN BE MUCH FASTER THAN nmpc_full's, AND FASTER IS SAFER.** Swept offline
against the true plant at the measured 30 ms latency, `ttr` from 40 s down to 4 s:

| ttr | 40 | 30 | 25 | 20 | 16 | 12 | 10 | 8 | 6 | 4 |
|---|---|---|---|---|---|---|---|---|---|---|
| t to 90 deg | 42 | 32 | 27 | 22 | 18 | 14 | 12 | 10 | 8 | 6 s |
| peak AoA | 11.3 | 11.1 | 11.0 | 10.7 | 10.4 | 9.6 | 8.9 | 7.9 | **6.7** | 8.6 |

**Altitude is 30.0 m in EVERY case and every run is 0/2201 solve failures.** Peak AoA IMPROVES as
the ramp quickens, because a quicker crossing spends less time below the 14.13 m/s stall speed
where the wing cannot carry the weight and the solver has to buy lift with angle of attack. This
INVERTS the nmpc_full lore ("ttr 18 rang the phugoid; do not go there") -- that was a different
controller on the pre-2026-09-06 plant, and it does not transfer.
**Default is now `ttr`=20** (half the flown transition, same peak AoA as the flown config).
Going FASTER than that is limited NOT by the transition itself but by the lateral defect below:
at ttr=8 the post-disturbance clean window (~60-70 s) is SHORTER THAN A TYPICAL CRUISE, so 20 is
chosen to clear a ~90 s cruise, not because the transition cannot go quicker. `WB_PARAMS["ttr"]` STAYS 40 -- it is shared with
nmpc_full, where 40 is flight-validated -- so the cascade's value lives in the GS's
`CASC_OVERRIDES`, applied only on the cascade path, and `tests/test_gs_cascade_params.py` fails if
it ever leaks into nmpc_full.

**★★★ THE WHOLE SIMULATION CHAIN HAS NO SENSOR NOISE — SO NO CONTROLLER HERE HAS HAD ITS NOISE
REJECTION TESTED. (2026-09-10, measured.)** Prompted by "the controller seems almost too perfect".
It was. `worst |roll| 0.0, worst |yaw err| 0.0` is not a result -- the offline plant is perfectly
symmetric with EXACT state feedback, so roll sits at 0 because nothing ever displaces it.
**AND THE HIL RIG IS THE SAME:** measured on a flown bag, the mocap's frame-to-frame attitude
jitter is **0.00042 deg sd** (max 0.002). A real OptiTrack at 120 Hz shows **0.02-0.1 deg**, i.e.
the rig is 50-200x cleaner, because `isaacsim_optitrack_ros2_emulator` republishes the EXACT sim
state. Offline and HIL share one blind spot, so agreement between them proves nothing about noise.

`nmpc_cascade_node.py --test` now takes `--noise_att` / `--noise_rate` / `--noise_vel` (corrupting
only the CONTROLLER's view, never the plant's integration) and `--gust`. What that exposes, at
ttr=20, 150 s, lat 0.03:

| disturbance | worst roll | worst yaw | altitude | solves |
|---|---|---|---|---|
| none (what was reported) | **0.0** | **0.0** | 30.0 | 0/3001 |
| attitude noise 0.1 deg | **19.5** | **33.2** | 30.1 | 0/3001 |
| attitude noise 0.3 deg | 20.2 | 34.0 | 30.1 | 0/3001 |
| body-rate noise 1 deg/s | 19.4 | 33.2 | 30.2 | 0/3001 |
| velocity noise 0.05 m/s | 0.0 | 0.0 | 30.0 | 0/3001 |
| lateral gust 3 m/s^2 | 24.8 | 49.2 | 30.2 | 0/3001 |

**0.1 deg of attitude noise -- trivial and entirely realistic -- takes roll 0.0 -> 19.5 and yaw
0.0 -> 33.2.** It barely worsens from 0.1 to 0.3 deg, so it is NOT noise amplification: any
attitude/rate perturbation excites the marginally-unstable heading mode, which grows to a limit
cycle bounded by the 25 deg `bank_max`. VELOCITY noise alone does NOT excite it (roll stays 0.0),
which fits the P-only bank law being driven by attitude. Onset is t~60-75 s and is the SAME for
ttr=20 and ttr=40 -- so `ttr` does not govern it, and the earlier "clean window" figures (from one
large upset) do not apply to the noise-driven case.

**WHAT SURVIVES THE SCRUTINY, AND WHAT DOES NOT:**
  * LONGITUDINAL IS REAL. Altitude 30.0-30.2 m in EVERY case including 3 m/s^2 gusts, airspeed
    15.2-16.0, tilt 88.8-90.0, and **0/3001 solve failures throughout**. Nothing crashed.
  * LATERAL IS UNVALIDATED AND PROBABLY POOR ON HARDWARE. Expect a ~20 deg roll / ~33 deg yaw
    limit cycle within ~75 s of cruise on a real mocap. Neither the offline gate nor HIL predicts
    it. This is the same defect as the P-only bank-to-turn law (above) -- noise just makes it
    unavoidable rather than disturbance-triggered.
**RULE: a clean lateral result from this stack is evidence of nothing until it is re-run with
`--noise_att 0.1 --noise_rate 1.0`. Add that to any lateral claim before believing it.**

**DYNAMICS AUDIT — `scripts/audit_cascade.py` (ttr=20 shipped config).** `audit_reference.py`
audits nmpc_full's reference; the cascade has its own, PLUS an allocation matrix and surface
formulas derived BY HAND, which is where this project has been bitten before. All four checked
NUMERICALLY against `so3.build_ode()`:

| check | result |
|---|---|
| `B_rot(beta)` vs plant dF/dT, dM/dT | **2.5e-13** -- exact at every tilt |
| `surface_authority(V)` vs plant | 0.0-0.1% at 8/12/16 m/s |
| reference is a TRIM, forward | worst \|qdot\| **0.038** rad/s^2, az 0.000 |
| reference is a TRIM, back-transition | worst \|qdot\| **0.018** rad/s^2, az 0.000 |
| `build_ode6` (outer's model) vs plant | vsdot/vzdot EXACT; qdot off by 0.038 rad/s^2 |

The 90 deg allocation column is worth recording, because the whole inner loop rests on it:
`dFz +0.0000, dMx -0.1200, dMy -0.0330, dMz -0.2840` -- the front pair makes NO vertical force and
almost pure YAW there, which is why the REAR pair (still vertical) has to carry roll.
NOTE: nmpc6's `--validate` claims build_ode6 is byte-identical to the plant; it is not quite --
0.038 rad/s^2 of pitch, small but non-zero.

**OBSERVATION, not yet a bug: the back-transition reference implies -8.68 m/s^2 of deceleration at
V=16 while `decel_rate` commands 1.5.** With only (Tf,Tr) you can satisfy vertical and pitch
balance and the along-track acceleration is whatever falls out, so this is expected -- but the
flare/aft-tilt schedule and `decel_rate` are describing DIFFERENT manoeuvres (~5.8x apart). The
solver resolves it by tracking the speed reference, and the flown back-transition braked at
~1.6 m/s^2 matching `decel_rate`, with a 0.1 m excursion. Left alone because it flies; but this is
the same SHAPE as the bugs that cost bags full_tr7 and full_tr25/27 (shipping one half of a trim),
so if the back-transition ever misbehaves, look here first.

**★★ OPEN DEFECT FOUND WHILE SWEEPING ttr: A MARGINALLY-UNSTABLE LATERAL MODE IN CRUISE.**
A DISTURBANCE seeds a slow roll/yaw oscillation that grows over MINUTES. It is present at EVERY
transition rate -- `ttr` only changes how long until it is visible:

| ttr | clean window after a roll15+yaw-25 upset | then |
|---|---|---|
| 8 | ~60-70 s | yaw +26 -> -32, roll to +19 |
| 20 | ~120-150 s | yaw -9.9 -> +8.3 -> -18.5, roll +19.2 |
| **40 (the FLOWN config)** | **~250-300 s** | yaw -32.3 at t=300, roll -18.5 at t=420 |

**UNDISTURBED flight is clean at every ttr for 240 s+** -- which is exactly why it was missed: the
200 s validation soak had NO disturbance, and the upset tests only ran 90 s, just short of onset.
**A long soak with no upset does not exercise this mode at all.** The flown HIL mission cruised
~90 s, inside the clean window for its ttr=40, so it never appeared in flight either.

CAUSE (structural, not a tuning miss): the bank-to-turn command was written P-ONLY --
`roll_ref = wing * k_bank * e_yaw` -- and heading responds to roll through a DOUBLE integration
(roll -> turn rate -> heading). P-only on a double integrator is marginally stable at best. The
open-loop spiral is already unstable at +0.042/s (t2 = 16.5 s at V=16), so the loop has to supply
the damping and it does not.
**A yaw-RATE term does NOT fix it as a gain tweak:** `k_bank_d` swept 0.0/0.5/0.8/1.5 against the
upset over 300 s, none converged (1.5 parks the yaw at +34 deg with roll drifting instead). The
parameter is kept, DEFAULT 0.0 = OFF, because the structure is right but the loop needs proper
design -- a washout on the heading term, or heading tracked through a commanded turn RATE rather
than a commanded bank. **Do not enable `k_bank_d` expecting a fix.**
**CONSEQUENCE: sustained cruise beyond ~2 min after any disturbance is NOT yet trustworthy.**
Keep `ttr` well above the intended cruise duration until this is fixed.

**`tilt_margin_deg` IS NOT THE LEVER.** 15 / 25 / 35 / 50 / 90 deg all give BYTE-IDENTICAL results
(22 s to 90 deg, peak AoA 10.7): the airspeed tilt clamp never binds, because the outer's own
commanded tilt already sits below trim(V) + 15 the whole way. The tilt follows the SPEED ramp
through the trim schedule, so `ttr` is the only knob that moves it. Do not go looking in the clamp.

**★ FOUR BUGS THE FIRST TWO HIL FLIGHTS FOUND, NONE VISIBLE OFFLINE:**
1. **`latency_s` WAS 4x THE TRUE DELAY.** MEASURED at **30 ms** by cross-correlating the elevator
   command against the resulting pitch ACCELERATION over 20 s of steady cruise (the elevator is a
   direct moment actuator, so that lag IS the command->effect delay of the whole chain):
   `scripts/casc_timing.py`. It was 0.12, inherited from the 19-state node. Over-compensating is
   NOT benign here -- the outer plans for a state the aircraft never reaches, and the elevator
   entered a ~4 Hz limit cycle slamming -0.97..+0.86 with pitch rate swinging +-100 deg/s while
   pitch itself moved only +-5. At 0.04 the same corridor is monotonic and smooth (AoA
   +8.0,+7.7,+7.4,+7.2,+6.9 vs +11.4,+10.5,+14.3,+9.0 before). **Re-measure after any change to
   the stack's rates; do not inherit this number from another node.**
2. **THE DIVERGED FALLBACK PUBLISHED RADIANS ON A NORMALISED CHANNEL** -- `self._beta` (1.405 rad)
   went out where 0.894 was meant, and it also HELD the cruise tilt instead of bringing the rotors
   up. The bag's impossible "tilt 126.4 deg" (= 1.405 x 90, on a servo clamped to 90) was the
   giveaway. safe_hover means ROTORS UP; it now slews the tilt to zero.
3. **`slow()` WAS NOT EXCEPTION-WRAPPED.** `inner()` was, and that felt like enough. It is not: an
   UnboundLocalError on the FIRST `decel_now` transition (a log line used the airspeed one line
   before the state was unpacked) killed the rclpy executor and the whole node mid-cruise. PX4 held
   the last frame, the tilt froze at 90 deg and the aircraft mushed from 30 m into the ground with
   the status lines simply stopping. **Wrap BOTH loops** -- `tests/test_cascade_structure.py`
   now fails if either is bare.
4. **THE PITCH REFERENCE NEEDS THE SAME RATE LIMIT AS THE TILT.** The flare is
   `flare * min(1, V/Vc)`, FULL at the instant decel begins: the reference pitch stepped
   +5.7 -> +20.7 deg in one tick and the aircraft was on the ground 5 s later. Rate-limiting the
   tilt alone is not enough -- the flare and the aft tilt are two halves of one manoeuvre.

**GS WIRING (auto mission).** "🚀 Run Full Auto Mission" now targets whichever wing-borne
controller the new `ctrl` dropdown selects. The two do NOT share a parameter surface, and
`_set_node_params` counts an unknown parameter as a failure that ABORTS the build, so the GS
filters `WB_PARAMS` through `CASC_KEYS` (11 of 22 keys). **`tilt_slew_deg_s` is deliberately NOT
sent: both nodes declare that name and mean different things** -- on nmpc_full an external limiter
on the commanded tilt (15 deg/s), on the cascade the node's MODEL OF THE SIM SERVO (80 deg/s, a
plant property). Sending 15 would allocate for a tilt the aircraft is not at and stretch the
0->90 deg sweep from 1.1 s to 6 s. A NAME MATCH IS NOT A MEANING MATCH.
`tests/test_gs_cascade_params.py` compares the two sources and knows about that collision.

**OPS:** launcher windows `6h-cascade` (both `restart_hil.sh` and `start_vtol_smc_hil.sh`, gated on
`want cascade`; the OLD SO(3) cascade's gate was renamed `so3_cascade` -- they collided and started
a second standby NMPC). GS button "⇄ Cascade (NMPC + geometric)" plus the mission dropdown.
`auto_arm` defaults FALSE; arm explicitly with `scripts/arm_vehicle.py`. Gate offline with
`nmpc_cascade_node.py --test [--lat S] [--comp S] [--kR K] [--decel_at S] [--roll0 D] [--yaw0 D]`.
Measure the loop with `scripts/casc_timing.py <bag>`.

# ★★★★ FULL MISSION FROM 30 m — THE TRANSITION NEEDS NO ALTITUDE (2026-09-08, bag full_tr37_30m)
Takeoff -> transition -> 90 deg cruise -> back-transition -> hover, **all from 30 m**:
```
transition  z 29.0 -> 30.6 m   tilt 12.6 -> 90.0 in ~33 s   AoA pinned +8..+9 (just under the CL peak)
cruise      z 31.2 - 31.5 m    tilt 90.0   V 16.3-16.5   wing 70-99%   rotors 6.5-10.2 N   700 m
verdict     TRUE FIXED-WING (sustained, level, controlled), 275 controlled level samples
back-trans  slowed < 3 m/s, excursion 10.6 m, aft tilt reached its full -20 deg
```
**THE 25 m DESCENT WAS NEVER PHYSICS.** The drag-free floor for reaching the 14.13 m/s stall speed
is 10.2 m of height -- but the aircraft does not have to buy speed with height at all: at 45 deg
tilt the FRONT pair gives 25.5 N forward (7.4 m/s^2) while total vertical thrust is 61.5 N against
a 33.8 N weight. Flights 19/20 descended because the REFERENCE UNDER-DEMANDS THRUST (the trim
treats forward force as a residual). Capping `descent_budget` at **8 m** forces the acceleration to
come from THRUST instead, and the corridor is then crossed with altitude essentially unchanged.

**AND IT FIXES THE ALTITUDE DRIFT.** full_tr36 (110 m, `descent_budget`=45, `cruise_alt_band`=60)
wandered **110 -> 183 m**; this held +-0.3 m for the whole cruise. The drift was the reference being
ALLOWED to wander, not commanding a climb -- a tight `cruise_alt_band` (15 m) contains it.

**30 m CONFIG:** `alt`=30, `descent_budget`=8, `cruise_alt_band`=15, `trans_sink`=0.5, `ttr`=40,
`v_lead`=2.5, `vcruise`=16, `wingborne_v`=14.5, `tilt_margin_deg`=8, `flare_deg`=15,
`beta_aft_deg`=20. This is now the GS "Wing-borne 90 deg (validated)" default; the button's
altitude guard follows `descent_budget + 20 m` (28 m) instead of a hardcoded 60.

**OPEN — the back-transition has a violent pitch transient at low altitude:** +54 deg then -48 deg
within 2 s, AoA swinging +25.6 -> -26.8. It recovers and stops the aircraft, but at 30 m there is
far less room to absorb it than at 110 m. Smooth this before trusting the low-altitude config on
hardware -- likely a slew limit on the commanded flare/aft-tilt rather than the step it applies now.

**TOOLING:** `scripts/restart_hil.sh` now POLLS for Isaac/PX4 readiness instead of fixed 100 s/32 s
sleeps, and takes `--if-needed`, which skips the restart when the stack verifiably passes its
health checks (link up, no poll timeouts, both controllers, no traceback). A repeat data point goes
from ~10 min to ~5. `scripts/watch_flight.sh` prints a live one-line status (procs, controllers,
link, position, tilt, solver ok) so a run is not 3.5 min of silent restart followed by a filtered log.

# ★★★★★ SOLVED — 228 s OF STEADY 90 deg FIXED-WING CRUISE (2026-09-07, bag full_tr30)
**THE TRANSITION WAS TOO AGGRESSIVE.** The phugoid was never a limit cycle and never needed
damping: measured on full_tr20 it was already **DECAYING** (peak-peak 19.5 -> 14.4 m, trend
-0.084 m/s) with a **10.3 s period**. It was a lightly-damped mode RINGING from a large entry
kick -- and with a 1.6 s horizon the solver sees only 16% of a cycle, which is why every cost-term
fix failed. The fix is to NOT EXCITE IT.
**GENTLE TRANSITION: `ttr` 18 -> 40 s, `trans_sink` 2.0 -> 0.8 m/s, `v_lead` 5 -> 2.5,
`tilt_margin_deg` 12 -> 8.** Nothing else changed; all live params, no rebuild.

| | full_tr20 (aggressive) | **full_tr30 (gentle)** |
|---|---|---|
| episode | 41.7 s | **228.3 s** |
| level samples | 13% | **99%** |
| \|vz\| | +-7.5 m/s | **0.07 avg** |
| rotors | 8.7 N | **4.7 N (14% of weight)** |
| altitude swing | 27 m | **11.5 m (mostly the entry)** |
| AoA | swinging to 46 deg | **+6.2 steady** |

**The steady state is an EXACT TRIM:** from t=159 to t=344 s the bag shows
`z=110.88 m, V=15.87 m/s, vz=0.00, tilt=90.0, AoA=+6.2, wing=101.5%, rotors=4.7 N` -- constant to
0.01 m and 0.01 m/s for 184 seconds. Wing carries 101%, rotors supply drag only.
Verdict from the hardened harness (which requires CONSECUTIVE controlled+level samples):
**TRUE FIXED-WING (sustained, level, controlled)**, 417-sample unbroken run = 208.5 s, no
altitude-loss flag.

**LESSON: when a manoeuvre rings, measure whether the oscillation DECAYS before trying to damp it.**
A decaying oscillation is an EXCITATION problem (shape the entry); only a sustained one is a
damping problem (change the cost). This campaign spent four flights -- TECS, a longer horizon, a
force clamp, and a thrust override -- damping a mode that was already stable. One measurement
(peak-peak amplitude vs time) would have redirected all of it.

**FLIGHT-TEST HYGIENE IS NOW ENFORCED: `scripts/restart_hil.sh`.** Restart the FULL stack before
EVERY test. Stale-stack faults cost four flights in this campaign (battery Land failsafe, a
silently dead HIL link, a 34 m/s runaway on a retained reference, and the vehicle 3.8 km from
origin where the safe controller's envelope refused to recover it). The script kills nodes -> PX4
-> Isaac, starts Isaac FIRST (it waits for the heartbeat) then PX4, re-applies the PX4 params that
do not survive a restart, brings both controllers back, and REFUSES to report ready unless there
are zero poll timeouts, Isaac is receiving HIL frames, and the NMPC node has no traceback.
(Check Isaac for the `VTOL_HIL vid=` tag, NOT a motor name -- while disarmed the controls are all
zero, the channel map yields `-> {}`, and a motor name never appears on a perfectly healthy link.)

### ★★★ AFT ROTOR TILT — THE BACK-TRANSITION BALLOON SOLVED (2026-09-07, bag full_tr32)
**User's insight:** with `beta >= 0` the ONLY decelerating force is nose-up pitch, and that
converts speed into HEIGHT -- it cannot be tuned away. At 15.87 m/s the aircraft carries 434 J,
which is EXACTLY 12.8 m of climb. Letting the FRONT pair lean AFT puts the braking force along the
flight path instead, so the body stays closer to level and the energy goes into drag, not altitude.

**ENABLED: `tilt_min_deg` 0 -> -20 on the front rotors**, `BETA_MIN`=20 deg in the NMPC bounds,
and the aft tilt is COMMANDED IN THE REFERENCE (`beta_aft_deg`) with `(Tf,Tr)` RE-SOLVED by
`so3.trim_split` for the negative beta, so the reference stays a balanced package (w_beta=60 holds
applied tilt to the schedule, so leaving it to the solver would not have worked).

| back-transition | none (full_tr30) | flare only (full_tr31) | **flare + AFT TILT (full_tr32)** |
|---|---|---|---|
| altitude excursion | 65.7 m | 18.9 m | **2.2 m** |
| slowed to hover | never (>5 m/s) | yes | **yes** |
| peak \|AoA\| | +78 deg (stalled) | 28 deg | (102 at 2.4 m/s -- no dynamic pressure) |

**2.2 m against a 12.8 m floor for pure energy conversion** proves the aft tilt is DISSIPATING the
energy rather than storing it as height. Flown: tilt walks 77 -> +9 -> **-15 deg** and HOLDS aft
while speed collapses 13.3 -> 2.35 m/s.

**★ THE SIM'S TILT MAPPING HAD TO BE RE-ANCHORED FIRST.** It was `angle = lo + (hi-lo)*cmd`, so
setting `tilt_min_deg=-20` would have made hover's `cmd=0` mean **-20 deg** -- and the SAFE
controller writes literal ZEROS on the tilt channels. That is the same class as the old
`(4,0.5,0.5)` channel map which made hover's 0 into 45 deg and tilted the aircraft on arm.
`_tilt_target_rad` now scales against `tilt_max` and CLIPS to `[tilt_min, tilt_max]`, so **cmd=0 is
always 0 deg** and the change is bit-identical for every airframe with `tilt_min_deg=0`. The
nacelle spawn state also had to move from `tilt_min` to 0, or the rotors spawn leaning back.

**★ TUNED, AND A CORRECTION: THE FLARE AND THE AFT TILT ARE COMPLEMENTARY, NOT SUBSTITUTES.**
I first credited the 18.9 -> 2.2 m improvement to the aft tilt alone -- but full_tr32 changed TWO
things at once (flare 25->15 AND aft 0->20), so it never isolated anything. Cutting the flare to 5
to "let the aft tilt carry the braking" **DEPARTS the aircraft** (full_tr35): the aft tilt still
brakes hard (15.7 -> 9.7 m/s in 2 s) and then AoA runs to **172 deg** -- flying backwards -- and it
falls 40 m. The nose-up flare is not merely extra braking; it is what keeps the deceleration
CONTROLLED.

| flare | aft tilt | result |
|---|---|---|
| 25 deg | 0 | stops, 18.9 m balloon |
| **15 deg** | **20 deg** | **stops, 2.2 m** <- validated |
| 5 deg | 20 deg | **DEPARTS** (AoA 172, -40 m) |

**VALIDATED BACK-TRANSITION: `flare_deg`=15, `beta_aft_deg`=20.** Never lower the flare below ~15
while aft tilt is enabled.

**TWO HARNESS BUGS found here, both mine, both worth keeping:**
  * **A MOVING CARROT never lets the vehicle stop.** The back-transition sent a target 40 m ahead
    of the CURRENT heading every tick, so there was always somewhere to go: full_tr34 chased it for
    70 s in a slow turn and only bled 16.3 -> 12.1 m/s. The stop point must be LATCHED ONCE.
  * **A FROZEN SIM reads as a successful manoeuvre.** full_tr33 reported "slowed below 3 m/s: True"
    while airspeed sat at 16.60 for the entire window -- PX4 had been killed and mocap was
    repeating its last sample. `fly_transition.py` now hashes (position, V) each sample and prints
    `!! SIMULATION FROZEN -- THIS RUN IS INVALID` after 4 identical reads.

### ★★ BACK-TRANSITION (cruise -> hover) — IMPLEMENTED (2026-09-07, bag full_tr31)
**There was no back-transition at all.** `fly_transition.py` handed a 16 m/s WING-BORNE aircraft
straight to the multicopter controller, which writes tilt to literal ZERO. Bag full_tr30 shows the
result: lift doubled, it ballooned 10 m, stalled at **AoA +78 deg** and departed into a dive --
**65.7 m excursion, never slowed below 5 m/s**. That failure hid behind the "recover" label for the
whole campaign because the harness only ever judged the CRUISE.

**FIX (two parts, both in the reference):**
1. **Run the trim schedule BACKWARDS** (`decel_now`, `decel_rate`): the commanded speed bleeds down
   and the schedule hands back tilt/pitch/thrust per speed, so the rotors come upright IN STEP with
   the wing unloading instead of snapping.
2. **NOSE-UP FLARE** (`flare_deg`=25). A tiltrotor cannot tilt backwards (beta>=0), so with the body
   LEVEL there is NO decelerating force. Pitching nose-up vectors the near-vertical rotor thrust
   BACKWARD (`T*sin(theta)`) and adds high-AoA drag. Faded with speed so it is strongest while there
   is speed to kill. Flown: pitch reaches **+53 deg** while tilt walks 90 -> 46, killing
   **15.9 -> 6.9 m/s in 5 s**, and **AoA stays at +3.3** because the flight path climbs with the
   nose -- it never stalls.

| | full_tr30 (none) | **full_tr31 (flare)** |
|---|---|---|
| slowed to hover | never (>5 m/s) | **yes, < 3 m/s** |
| altitude excursion | 65.7 m | **18.9 m** |
| peak \|AoA\| | +78 deg (stalled) | **28 deg** |
| peak \|vz\| | -13.6 m/s | **6.3 m/s** |

**THE REMAINING BALLOON IS MOSTLY PHYSICS.** At 15.87 m/s the aircraft carries 434 J; converted
entirely to height that is **12.8 m of climb**, so 68% of the observed 18.9 m is unavoidable
kinetic->potential conversion. Only ~6 m is excess. To remove it the energy must be DISSIPATED as
drag (longer, higher-AoA decel) or absorbed by commanding a descent through the flare.

**CRUISE REFERENCE TRACKING IS EXCELLENT** (measured on full_tr30 over 184 s):
airspeed cmd 16.00 -> **15.867 +- 0.001** (-0.8%), altitude **110.875 +- 0.0022 m** (12 mm of drift),
climb rate **0.0001 +- 0.0017 m/s**, tilt 90.00 exact. The only structural error is the 0.8% speed
offset, which is the trim treating forward force as a RESIDUAL rather than tracking it -- the same
root cause as the coasting problem, and what `trim_split(..., Fx_req=)` would fix.

# ★★★ 90 deg WING-BORNE FLIGHT SUSTAINED -- BUT IT IS A PHUGOID, NOT STEADY CRUISE
# (2026-09-07, bag full_tr20)
**READ THE CORRECTION AT THE END OF THIS SECTION BEFORE QUOTING ANY NUMBER HERE.**
`nmpc_full_smc_node` on the corrected plant: **tilt 90.0 deg, rotors 8.6-8.9 N of a 33.8 N weight
(drag thrust only -- at 90 deg they make NO lift), wing carrying 175-242%, V 14-15 m/s avg (peaks
20-23), altitude START 110 m -> END 111 m, NO altitude-loss flag.**
Episodes: **41.7 s, 38.7 s, 29.4 s, 25.6 s, 23.4 s -- ~159 s of cumulative 90 deg flight.**
Verdict from the (hardened) harness: TRUE FIXED-WING (sustained, level, controlled).

**THE THREE CHANGES THAT DID IT, in order of size:**
1. **THE SETTLE GATE MUST LATCH (bag full_tr15/16).** `settling` was re-evaluated EVERY TICK as
   `(...) or (z < z0 - 1.0)`, so a **1 m** altitude dip in cruise threw the vehicle back to the
   hover reference -- tilt to 0 at 12 m/s -- while it was holding altitude to within 1 m. Latch it
   once satisfied; in cruise only revert on a real `cruise_alt_band` loss. 4.2 -> 6.2 s, tilt 83.
   FOURTH instance of a reference surrendering to its own tracking error.
2. **THE REFERENCE MUST COMMAND THE DESCENT, NOT MERELY PERMIT IT (bag full_tr18/19).** The
   altitude reference was FLAT through the corridor. Below the 14.13 m/s stall speed the wing
   cannot carry the weight, so obeying it forced the solver to buy lift with ANGLE OF ATTACK:
   full_tr18 mushed at **AoA +28..+35 deg at 8-9 m/s**, deep on the BACK SIDE of the drag curve
   where it can never accelerate out. Widening `cruise_alt_band` did NOTHING because that only
   relaxes the settling GATE -- the COST still demanded the old altitude. `build_refs` now commands
   a bounded SINK (`trans_sink`) while below `wingborne_v`, then levels off. **AoA +28..35 -> +12.7**
   and tilt hit a true 90 deg. PERMISSION IS NOT DEMAND.
3. **GIVE THE MANOEUVRE ENOUGH ALTITUDE (bag full_tr19/20).** The dive-out needs ~25 m. Starting at
   50 m with a 25 m band put the settling threshold at 25 m, and full_tr19 was dumped at z=24.8 m
   **at the exact moment it succeeded** (18.78 m/s, 90 deg, wing 141%). At 110 m with
   `descent_budget`=45 / `cruise_alt_band`=60 it completes the trade and stays wing-borne.

**CONFIG:** `alt 110, dist 2500`; `trans_sink`=2.0, `wingborne_v`=14.5, `descent_budget`=45,
`cruise_alt_band`=60, `v_lead`=5, `tilt_margin_deg`=12, `latency_s`=0.12, `rate_hz`=15,
`w_aoa`=1.5 with the **^4** exponent, N=8/DT=0.20 (1.6 s horizon). Safe controller `z_max` raised
to 130 to allow the 110 m start (a SAFETY-RELEVANT widening -- review before hardware).
**★★ CORRECTION -- IT IS NOT STEADY CRUISE.** Inside the 41.7 s episode the vehicle is
PORPOISING: altitude cycles **63-90 m**, speed swings **7-20 m/s**, vz reaches **+-7.5 m/s**, and
only **642 of 4992 samples (13%) are level**. AoA averages +11 deg but peaks at 46. What is TRUE:
it stays airborne ~159 s, returns to its start altitude (110 -> 111 m), and repeatedly reaches a
genuine 90 deg tilt with the rotors at ~10 N while the wing carries the aircraft. What is NOT
true: steady 1-2 minute cruise.
**AND THE HARNESS FLATTERED IT AGAIN (the 4th time).** At the BOTTOM of each phugoid cycle `vz`
passes through zero, so those instants pass an INSTANTANEOUS `|vz| < 1.5` test while the aircraft
is oscillating violently -- the headline sample (22.80 m/s, 90 deg, wing 238.9%, z=63.9) is exactly
such a point. `fly_transition.py` now requires the longest UNBROKEN RUN of consecutive
controlled+level samples, not the total count. Instantaneous level is not steady flight.
**REMAINING: DAMP THE PHUGOID.** That is now the whole problem -- the corridor is crossed, 90 deg
is reached and held, and the vehicle is stable in the large. The oscillation is the ~4 s phugoid
the 1.6 s horizon still cannot see a full period of; candidates are a longer horizon (DT 0.20 ->
0.30 was never tried), an explicit vz/flight-path damping term, or an energy (total-energy)
formulation rather than separate altitude and speed tracking.

# ★★★ AoA GUARD BREAKS THE PLATEAU (2026-09-07, bag full_tr11) — 52% wing share, 72 deg tilt.

**The 8.4-8.7 m/s plateau (identical across FOUR tunings) was ANGLE OF ATTACK.** The AoA swung to
+-46 deg against a 13 deg stall break; every excursion collapsed CL, dumped the wing's share back
onto the rear rotors, broke the pitch trim and bled the speed just built. Nothing in the cost knew
what AoA was. Added `_aoa_excess` + `w_aoa` to `nmpc_full_smc_node`: **max |AoA| 46 -> 32 deg, and
in LEVEL flight it reached 10.5 m/s at 72 deg TILT with the wing carrying 52% of weight and the
rotors down to 16.8 N from 33.8** (vs 36% at 50 deg before). Biggest single gain of the campaign.
  * **SMOOTH form is mandatory.** My first version, `fmax(0, |alpha| - lim)`, has a corner exactly
    at the limit -- where corridor trajectories ride -- and `fabs` adds a second at alpha=0, which
    the vehicle crosses constantly. This project already paid for that once (the piecewise
    post-stall break -> IPOPT convergence failures at the transition tilt angles). Use the even
    power `(alpha/lim)^6`: 0.04 at the +5.8 deg cruise trim, 365 at 30 deg, no corner anywhere.

**★★ DO NOT TRUST A PEAK. full_tr11 reported "peak 15.83 m/s, 120% wing share, TRUE FIXED-WING"
and it was a DIVE** -- it traded 17 m of altitude for speed (20 m -> 3.7 m -> 0.33 m) and the safe
controller caught it at the ground. Everything measured while descending is gravity, not lift.
`scripts/fly_transition.py` now judges ONLY on SUSTAINED LEVEL samples (|vz| < 1.5 m/s, >= 10 of
them) and appends a warning if the flight lost more than 5 m of altitude.

**TWO SILENT-FAILURE MODES cost a flight each; both looked completely healthy:**
  * the SAFE controller's envelope was measured from the ORIGIN only, so after 1430 m of
    cumulative flight it silently refused every command -- the one controller whose job is to
    recover the vehicle could not even hold station. **A safety fallback must always be able to
    operate where the vehicle IS**: the check is now `within radius_max of home OR radius_local
    (100 m) of the vehicle`.
  * **PX4 latched "Landing detected" at 21 m** because the vehicle sat motionless for minutes
    during debugging; actuator output was inhibited from then on. Symptom in both cases:
    everything nominal, nothing moving, no error anywhere. Watch for it.

**REMAINING:** it reaches ~10.5 m/s level (stall 14.1). The next lever is the HORIZON: 0.8 s
(N=8, DT=0.10) against a ~4 s phugoid at 9 m/s -- the solver cannot see the mode it is exciting.
Try DT 0.10 -> 0.25 (2 s for the same solve cost; solve is 17-34 ms of a 100 ms budget).

# ★★★ FULL-STATE NMPC — HIL CAMPAIGN (2026-09-07). Stable, altitude-perfect, SPEED-LIMITED.

Five HIL flights of `nmpc_full_smc_node` on the corrected plant (bags `full_tr1..full_tr5`).
Went from "departs at 4.2 m/s and falls" to "flies 317 m holding altitude to +-0.2 m" -- but it
CANNOT build speed past ~5 m/s, so it never reaches the corridor (stall 14.1, wing-borne 15.6).

**THREE bugs found by FLYING, all invisible to the offline test:**
1. **`R_ref` built `Ry(+theta)` = NOSE-DOWN while its docstring said nose-up** -- the FIFTH copy
   of the 2026-09-06 inversion; four were fixed and this one was missed. The node's own log
   printed `pitch=-77` where the truth was `+87`: it MEASURED pitch in the old convention while
   the trim schedule fed it the NEW one, so past a few degrees the attitude error changed sign
   and the loop DROVE the divergence. (Also fixed: both `asin(R[2,0])` readouts.)
2. **`w_beta` 12 -> 60.** A soft tilt guide leaves a nearly flat tilt-vs-pitch optimum and the
   solver hunts along it (applied tilt 17->5->24->13 deg against a steady 21-24 reference).
   `vtol_nmpc` raised it for exactly this and it was never back-ported -- audit finding C9.
3. **The tilt servo was modelled as a first-order lag with NO rate limit** while the sim
   implements a pure 80 deg/s RATE LIMIT (`rotor_dynamics` off). The solver believed it could
   swing the tilt arbitrarily fast. Fixed in `so3.build_ode` with a tanh saturation
   (`betadot = TILT_RATE*tanh((betacmd-beta)/(TAU_BETA*TILT_RATE))`) and the node's own tilt
   estimate now integrates the same law. **This is audit finding C10**, documented and unfixed
   since the audit. It measurably improved tilt tracking (hunting +-7 deg -> within 2-3 deg)
   and raised the speed ceiling 4.2 -> 5.6 m/s.

| flight | outcome | distance | altitude | peak V | peak wing share |
|---|---|---|---|---|---|
| full_tr1 | departed 4.2 m/s | 13 m | lost | 4.1 | 9.8% |
| full_tr2 | departed 4.2 m/s | 13 m | lost | 4.2 | 7.5% |
| full_tr3 | **survived** | 152 m | 20.1-20.3 | 4.3 | 8.5% |
| full_tr4 | survived (departed in RECOVER) | 373 m | 20.1-20.2 | 4.7 | 13.3% |
| full_tr5 | **survived** | 317 m | 20.0-20.2 | 5.6 | ~13% |
| full_tr6 | **survived, PARTIAL WING-BORNE** | -- | ~20 | **8.45** | **36.1%** |

**★ BREAKTHROUGH (flight full_tr6): THE REFERENCE WAS SURRENDERING TO ITS OWN TRACKING ERROR.**
`build_refs` computed `vtar = min(V_now + k*DT*(Vc/ttr), Vc)` from the **MEASURED** speed, so over
the 0.8 s horizon it could only ever ask for `V_now + 0.7 m/s`. When the pitch/tilt coupling bled
speed off, the reference re-anchored to whatever was left and followed the vehicle DOWN -- there was
no persistent demand to accelerate. **This is the THIRD instance of the same pathology in this
project** (the SMC re-seeding on tracking lag = C7, the GS mission clock freeze, and now this).
Treat it as a design rule: a reference must advance on its OWN clock with a BOUNDED lead, never
re-anchor to the achieved state.
FIX: the node carries `self._v_cmd` on a clock (ramps at `Vc/ttr`, clamped to `V_now + v_lead`).

| flight | change | peak V | tilt | wing share | verdict |
|---|---|---|---|---|---|
| full_tr5 | rate-limited tilt model (C10) | 5.6 | 33.5 | 13.3% | multicopter |
| full_tr6 | **+ speed on its own clock** | **8.45** | **50.4** | **36.1%** | **PARTIAL WING-BORNE** |
| full_tr7 | + decouple demand/feed-forward | 6.91 | 46.7 | 25.9% | WORSE -- reverted |
| full_tr8 | lead 3 -> 5 | **8.71** | **60.1** | 32.0% | PARTIAL WING-BORNE (fastest, highest tilt) |

**★ DO NOT RETRY (full_tr7):** splitting the reference into a leading "demand" speed for the STATE
and a measured-anchored "feed-forward" speed for the CONTROL. The reasoning is tempting (the trim
split is only valid at the speed it was solved for) but the TILT appears in BOTH `x[12]` and `u[4]`,
so the split told the solver two tilts ~18 deg apart and the reference stopped being a
self-consistent trim. **A trim is a package -- do not take it apart.** If the feed-forward is too far
ahead, shorten `v_lead` (which moves the whole package back together). Recorded in the source.
**`v_lead` 3 vs 5 -- COMPARABLE, not a clear win either way** (I first called 5 "worse" from a
mid-flight log sample instead of the summary; that was wrong):
  * lead 3 (full_tr6): V 8.45, tilt 50.4, wing 36.1%
  * lead 5 (full_tr8): V **8.71**, tilt **60.1**, wing 32.0% -- FASTEST and HIGHEST TILT of any
    flight, but rougher: AoA swinging +-46 deg and more `ok=0` solve failures at 46-58 ms.
Default left at 3.0 for the smoother ride; 5.0 is the one to revisit if the oscillation is damped.
**CONFIG:** `v_lead=3.0`, `latency_s=0.12`, `w_beta=60`.
**LESSON:** judge a flight from its SUMMARY, never from a mid-flight log sample -- the vehicle is
oscillating, so any single sample says whatever the phase of the cycle says.

**THE REMAINING PROBLEM IS A DESIGN ONE, NOT A BUG.** A pitch/tilt/speed LIMIT CYCLE: tilt rises
toward its schedule -> the solver commands a big nose-up (mot front-heavy 0.62/0.25 AND elevator
saturated nose-up TOGETHER) -> pitch +19..+41, AoA +18..+30 -> speed bleeds off -> tilt backs off
-> repeat at ~0.5-1 Hz. Altitude is held perfectly THROUGHOUT (that is the point: it is buying
altitude with pitch because at 5 m/s the wing supplies ~5% of weight while the schedule's 26-31 deg
tilt removes ~15% of vertical thrust). **The cost has pos/vel/att/tilt/rate terms but NOTHING that
rewards BUILDING AIRSPEED through the corridor**, so the solver is content to sit at 3-5 m/s
slowly satisfying a position reference. Candidate fixes (NOT attempted -- needs a design call):
an explicit airspeed-tracking term, a corridor-crossing time constraint, or committing the tilt
open-loop through the corridor while the wing loads.

**Ops notes:** solve is 17-34 ms in HIL (NOT the 43-96 ms this node was tuned around), so
`latency_s` 0.25 was ~2x over-compensating; 0.12 is closer (live `ros2 param set`, no rebuild).
`w_beta`/`build_ode` changes DO force a ~4 min JIT rebuild (the widened cache key working as
intended -- the old August caches have different hashes and were correctly NOT reused).
**The safety architecture worked every time:** `scripts/fly_transition.py` aborts on altitude loss
or attitude departure and the safe controller recovered the vehicle to a stable hover from 20 m on
all four departures. Every earlier attempt in this project ended inverted on the ground.

**★ THE OFFLINE TEST PASSED WITH BUG 1 PRESENT** and reported textbook wing-borne numbers
(tilt=88, AoA=+5.2, rotors 11.3 N). It has no disturbances and no HIL timing, so a near-level
inverted attitude reference tracks fine -- the sign only matters once something perturbs you.
Model-matching the plant fixed finding C5's worst form; it did NOT make a benign test
discriminating. Treat `--test` as a numerical-stability check, never as a flight gate.

# ★★★ HIL VALIDATED (2026-09-06) — the safe controller flies full missions, first attempt.

`geometric_hover_node.py` on the corrected plant, `CONTROLLERS="geometric"` (bag `/tmp/bags/geo1`,
421 s, 50k mocap / 84k actuator samples). **Publisher count on actuator_motors: 1** — the
CONTROLLERS selector removes the ghost-publisher risk by construction, no kill race.

- **THREE complete missions, no failures, no retries.** Ground-to-ground: takeoff -> 47 m diagonal
  at 12 m -> descend -> land, `scripts/fly_mission.py` (NEW: completion-gated waypoint driver with
  telemetry, same advance rule as the GS Fly-to-point button).
- **Precise landing: hold error 6 mm mean / 15 mm max**, drift ~1 cm over 20 s. Arrival errors are
  the commanded tolerance (0.08-0.10 m), not a limit.
- **Level-cruise altitude hold 0.09-0.27 m at ~5 m/s** (offline plant double predicted 0.23 -- the
  double is faithful).
- **Hover steady state: 2 mm position spread, 0.04 deg attitude**, motors `[0.474, 0.475, 0.465,
  0.466]` -- the front>rear split the CG geometry predicts (balanced hover 8.55 N front / 8.37 N
  rear) and exactly what `tests/plant_double.py` predicted (0.470 mean).
- **Tilt servos identically 0.000000 across 84k samples** -- "cannot tilt, by construction" holds
  in flight, not just in review.
- **No actuator freeze, no gaps > 0.5 s, no LOOP EXCEPTION, no PX4 failsafe.**
- Envelope guard verified: a target at 400 m / 90 m (limits 300/60) is ignored and the vehicle
  does not move.
- Peak |roll| 13.4, |pitch| 17.4 deg -- well inside the 35 deg lean cap. Peak speed 5.1 m/s vs
  `v_max`=4.0 (the ramp limits the SETPOINT; the vehicle overshoots ~25% on a diagonal where the
  horizontal and vertical limits add -- harmless, worth knowing).

NOT yet flown: the wing-borne controllers (nmpc6 / nmpc_full / vtol_nmpc). They need their JIT
caches rebuilt (the keys now hash the model source, so the first launch rebuilds ~2 min) and have
never flown against the corrected plant.

# ★★★ READ FIRST — AERO COEFFICIENTS + CONTROLLER PROPAGATION + REFERENCE TRIMS (2026-09-06)

**Coefficients re-derived from the airframe's OWN surfaces** (`tests/test_aero_consistency.py`
checks this and will fail if they drift again):
- `cl_aileron` 0.08 -> **0.49**. 0.08 was the AVTOL's (small outboard ailerons on a 1325 kg
  aircraft). THIS aileron is 0.35 x 0.12 m on a 1.4 m span / 0.25 m chord wing = 50% span,
  48% chord -- an ELEVON; the strip estimate is 0.49. **At 0.08 the ailerons NEVER overtook
  rotor differential-thrust roll, so as the rotors tilt to 90 deg (where they stop making
  roll) there was no hand-over and roll authority genuinely vanished.** At 0.49 the cross-over
  is 8.9 m/s, below the 11.5 m/s stall. This is very likely the lateral fragility that
  defeated every 90-deg attempt.
- **ALL-MOVING surfaces:** vtol.py builds each surface as a whole hinged box (no fixed
  stabiliser/fin), so tau=1 and the derivative is the full volume x slope. `cm_elevator`
  -1.1 -> **-1.75**, `cl_elevator` 0.4 -> **0.70**, `cy_rudder` 0.17 -> **0.24**,
  `cn_rudder` -0.12 -> **-0.109**. The fin is now EXACTLY self-consistent
  (|Cn_dr|/|CY_dr| = l_v/b = 0.447). Pitch hand-over moves 14.1 -> 11.2 m/s.
- `cn_beta` 0.25 -> **0.11** (= fin-alone V_V*a_v; a fuselage only REDUCES weathercock, and
  the pair (0.25,-0.98) claimed Cn/CY = 0.26 where the fin arm allows 0.45). `cy_beta` LEFT at
  -0.98 (defensible once fuselage side area is counted).
- **SIDE FORCE now saturates** (`cy_max`=0.6, `sideslip_break_rad`=20 deg): the linear fin term
  extrapolated to beta=90 claimed 16 N in a 6 m/s crosswind (half the weight) and double-counted
  with `cd_beta2`. Blends to a flat plate, same as lift does in alpha -> 5 N. Symmetric fix.
- Stall blend rewritten as the **tanh form** of the Beard sigmoid (identical to 3e-3, saturates
  instead of overflowing) so the CONTROLLERS can evaluate the same expression in CasADi.

**Velocity scaling (the answer to "which axis"):** the sim scales on the FULL 3-axis |v_body|;
static terms ~V^2, rate-damping ~V (phat/qhat/rhat divide one power back out). Aero surface power
is V^2 while ROTOR authority is flat (no inflow model, finding F6) -> hand-over speeds roll 8.9 /
pitch 11.2 / yaw 12.3 m/s. The CONTROLLERS used to gate on `u^2/(u^2+4)` -- FORWARD speed only --
so they modelled ZERO aero in descent and lateral translation where the plant makes real force.

**CONTROLLER PROPAGATION (finding C1 + C2/C3/C4 + the velocity fixes):**
- CONVERTED + VERIFIED against the plant by `tests/test_controller_aero_match.py` (0.01 N):
  `nmpc_so3_transition_node.aero_body` (the hub -- nmpc6, nmpc_full, cascade share it),
  `vtol_nmpc._aero_body`, `nmpc_so3_cascade.aero_world`, `nmpc6.build_ode6`.
  Recipe: convert FLU->FRD in and back out, publish-standard coefficients, no sign patches.
  The `u^2/(u^2+4)` gate is GONE (V is floored at MIN_V instead).
- **PITCH CONVENTION FLIPPED:** `_R_pitch` / `R_ref` / nmpc6's `theta` are now **+theta = NOSE-UP**,
  matching the trim schedules. They used to build Ry(+theta) = nose-DOWN, which cancelled against
  the inverted aero: a +8 deg cruise trim was flown 8 deg nose-DOWN. `theta_act = +asin(R[2,0])`.
  All diagnostic `aoa` prints are now the STANDARD angle (old bags have the opposite sign).
- Constants: M 3.5->3.45, **I = diag(0.265, 0.303, 0.490)** (ASSEMBLED, was the body link's
  0.20/0.28/0.35), **CQ 0.06 -> 0.12** (= rotor_moment_ratio), **RPOS z front -0.033 / rear +0.016**
  (was +0.02 for all four -- wrong SIGN on the front pair, and it IS the pitch moment at 90 deg tilt).
- **JIT cache keys now hash the MODEL SOURCE** (`inspect.getsource`) + geometry/inertia/CQ, not just
  the weights -- otherwise a model change silently reloads a stale compiled solver (C6).
- nmpc6's surfaces are **q-bar NORMALISED** (`delta = M / (qbar*S*span*C_delta)`) instead of the
  hand-tuned `k_ail/k_rud` behind an airspeed ramp: the delivered moment matches the demand at
  every speed and the SIGN comes from the coefficient (the +k_rud sign trap ran yaw to -100 deg).
- **NOT converted, and they now REFUSE TO START:** `nmpc_cruise_node`, `nmpc_quad_node`,
  `aero_transition_node` (`FSC_ALLOW_STALE_MODEL=1` overrides, offline only).

**★★ REFERENCE PROFILE AUDIT — the schedules were NOT trims.** Plugging (xref,uref) into
`build_ode` left up to **5.3 rad/s^2 of residual pitch** and 1.5 m/s^2 vertical. Two causes:
1. the aero pitch moment entered the front/rear split WITHOUT being negated into the FLU
   (nose-down positive) convention the thrust moment is written in -- correct for the old
   flipped Cm, wrong for the corrected one;
2. **the AVTOL generator returns ONE Ttot on the premise that the WHOLE rotor set tilts.** This
   airframe tilts only the FRONT pair; mid-corridor that under-counts the vertical thrust by
   7.8 N = 23% of weight.
FIX: keep the generator's ATTITUDE schedule (tilt, alpha, theta, de) and SOLVE the two thrusts
from this airframe's geometry for both balances at once --
`Tf*cos(beta-theta) + Tr*cos(theta) + Fz_aero = W` and `Tf*(zf sinb - xf cosb) - xr*Tr + My_aero_FLU = 0`
-- with the aero taken from the plant's own model. Residual now **az = 0.000, |qdot| <= 0.04**, and
`ax` is the intended +2 m/s^2 transition acceleration tapering to +0.2 at cruise.
`vtol_nmpc.TRIM` REGENERATED from it. `tests/test_reference_profiles.py` guards all of this.

**★★ ALL FOUR REFERENCE PATHS now audited — the two I had only sign-fixed were still wrong:**
- **`TransitionRef.get` (what the SO(3) node flies):** pitch was trimmed but VERTICALLY NOT
  (az up to 1.45 m/s^2) -- it still used the generator's `Ttot`. Now 0.000.
- **`fsc_mission_ref` (the full mission) was commanding a FREE FALL.** Its `get()` built
  `Rp = Ry(+theta)` = NOSE-DOWN while `theta` is constructed as `alpha + gamma` (nose-up) --
  the SAME inversion as `_R_pitch`/`R_ref`, missed in the first pass. In cruise the wing flew
  at NEGATIVE AoA: **az = -10.10 m/s^2, qdot = -9.02 rad/s^2**.
- **ONE shared solver now: `nmpc_so3_transition_node.trim_split(beta, theta, V, de, Fz_req,
  gamma, Fx_req, beta_max)`.** Used by nmpc6.Schedule, TransitionRef.get and fsc_mission_ref.
  With only (Tf,Tr) you can satisfy TWO demands; vertical+pitch use up all the freedom and the
  FORWARD acceleration is then whatever falls out (reference asked +2.42 m/s^2, plant gave
  +1.13). Passing `Fx_req` makes it solve the TILT as a third unknown (Fx is monotone in beta,
  so bisection). `Fz_req = M*(G + zdd)` so a climb/descent leg is trimmed for what it is
  ACTUALLY doing, not for level flight.
- **BRAKING PITCH (mission):** a tiltrotor cannot tilt backwards (beta>=0), so with the body
  LEVEL there is NO decelerating force -- the profile commanded -3.5 m/s^2 and the plant
  produced -0.02. The reference now commands NOSE-UP during a decel leg (`theta = asin(-a/g)`),
  bounded by `alpha_brake_max_deg`=12 so the total AoA stays inside the linear range. THAT
  BOUND is what limits this airframe to ~1-2 m/s^2 of braking, and it is the real reason a
  mission overshoots a finite target -- it is a REFERENCE limit, not a controller failure.
  `decel_cap` 3.5 -> 2.0; the speed profile is re-integrated from the CLIPPED acceleration so
  the profile and the trim describe the same manoeuvre (they disagreed: -4.5 vs a 3.5 cap).
- FSC `m` 3.5 -> **3.45** (assembled) -- 3.5 left a steady 0.14 m/s^2 bias in every reference.
- **Mission residual: 4.76 -> 0.162 m/s^2, pitch -9.02 -> 0.029 rad/s^2.**
- PROCESS NOTE: a `try/except Exception` around the new solver silently swallowed an
  AttributeError and the fix appeared to do nothing for a whole iteration. Don't wrap a
  numerical path you are trying to verify.
- Probe trap hit AGAIN: `xref` stores R **row-major**, and numpy's reshape is row-major, so
  `np.array(x[3:12]).reshape(3,3)` needs **NO transpose** -- the `.T` is only for `ca.reshape`
  (column-major). A stray `.T` made a correct nose-up reference read as nose-down.
Trajectory files: `aircraft_transition/quad_forward/quad_transition.csv` were UNSCALED AVTOL
(30-40 m/s, up to 603 m -- 2-3.5x this airframe) -> moved to `trajectories/unscaled_avtol/` with a
README; `full_mission_transition.csv` had velocity columns 1.9 m/s off its own position track ->
rewritten; GS playback fence 300 -> 500 m (the FSC missions reach 378 m).

# ★★★ READ FIRST — CONTROLLER SPLIT + STREAMLINE (2026-09-06). Two jobs, two controllers.

**SAFE HOVER = `ros2/.../geometric_hover_node.py` (id `geometric`).** It flies to the x,y,z the
operator types and HOLDS it. A PURE MULTICOPTER -- the tilt servos are written as literal zeros
and there is no tilt parameter to set. No aero model, no trajectory clock, no outer loop: it
generates its OWN velocity-limited motion profile (`SetpointRamp`, sqrt stopping profile, matching
v/a feed-forward), so a step target from the GS is smooth by construction. This is where STOP
always sends the vehicle, so its behaviour is deliberately boring. It OWNS: the GS "Set Position"
panel, "Fly to point + LAND", and STOP.
**TRAJECTORIES / PATHS / TRANSITION = the NMPC nodes.** Unchanged. Playing a trajectory on
`geometric` is now REFUSED by the GS (it chases points, not paths).
(`traj_track_node.py` is DELETED -- superseded. `start_vtol_minimal.sh` is now just a CONTROLLERS list.)

Streamline (details in the closed-loop audit):
- **ONE reference topic on the safe path.** `geometric` reads the RAW reference; it is NOT SMC-fed.
  Removed: the GS's second publisher on `reference_smc` (it INTERLEAVED with smc_outer's own 50 Hz
  output -- two writers on the safety path), and the differentiated feed-forward with its hardcoded
  `dt_ref=1/30`. smc_outer still feeds the NMPC nodes that were tuned around it.
- **`CONTROLLERS` env selects which nodes start** (default `geometric` alone; `all` = old behaviour).
  Every standby node still SOLVED at full rate and competed with Isaac/PX4/GS for CPU -- the 19-state
  solve is 22 ms idle vs 48-96 ms under HIL load, and an overrunning loop is a documented tumble cause.
  `scripts/start_vtol_minimal.sh <cfg> [controller]` = that list (was: launch all, then kill windows).
- **GS mission = 3 COMPLETION-GATED WAYPOINTS** (climb / traverse / descend), replacing the open-loop
  min-jerk clock + lead-clamp + clock-freeze (all of which existed because the old controller couldn't
  keep up with its own reference). GO/HOLD, Fly-to-point and STOP now FORCE `geometric` (they used to
  use whichever button was last pressed). Playback geofence RE-ENABLED (300 m / 60 m; it had been set
  to 1e7 after a dead-call bug).
- **Fixed in the safe controller** (audit C3/C12): `cg_x` 0.195 -> **0.176** (the ASSEMBLED CG) and
  `cQ` 0.06 -> **0.12** (= the sim's `rotor_moment_ratio`); anti-windup is now clamp+back-calculation
  + freeze-while-saturated with `i_max` 3 -> **8 N** (3 N could not trim a 5 N wind side-force, and the
  1 m integral BAND refused to act at all once the offset exceeded it); the integral FADES with
  setpoint speed (what it charges in transit is drag, which discharges as overshoot on arrival);
  attitude-priority desaturation; the loop is exception-wrapped (bag vt1).

**★ TEST HARNESS THAT CAN SEE MODEL ERROR — `tests/plant_double.py`.** 6-DOF plant built from the
SIM's own `vtol_aerodynamics.py` + the assembled mass properties of `quad_tiltrotor_config`
(M=3.45, CG_x=0.176, I=diag(0.265,0.303,0.490)). Reproduces the sim's measured hover split
(8.55/8.37 N vs measured 8.6/8.4). Use it instead of a controller's own `build_ode` -- that is the
C5 finding (the old selftests validate each controller against itself, so model error cancels).
Tests: `tests/test_geometric_hover.py` (26 closed-loop assertions; `--latency 0.1`),
`tests/test_vtol_aerodynamics.py`, `tests/test_gs_trajectory_tab.py` (real GS tab, offscreen,
run with `/usr/bin/python3`). All pass. NONE of it is HIL-flown yet.
**MEASURED: the safe controller's loop-latency margin is 125 ms** (stable to 125, diverges at 150 --
the ATTITUDE loop goes first, kR=9 at 200 Hz). Budget for mocap->DDS->ctrl->DDS->PX4->Isaac.

# ★★★ READ FIRST — AERO FRAME FIX (2026-09-06). Everything below it predates this.

The sim's aerodynamic model had computed `alpha`/`beta` from the **FLU** body velocity and fed
them into **FRD**-published stability derivatives. FLU->FRD is a 180 deg rotation about x, so
`v,w,q,r,alpha,beta,Fy,Fz,My,Mz` flip while `u,p,Mx` do not; six coefficients were consequently
sign-wrong. **The wing produced lift only at NEGATIVE true angle of attack, and the pitch axis was
statically DIVERGENT** (-61.9 N.m/rad at 16 m/s, time-to-double 0.04 s). That -- not the airframe,
not latency, not the controllers -- is what made "genuine wing-borne cruise" impossible all session.
The 2026-08-11 "FLU pitch-moment sign fix" (`cm_alpha` -1.2 -> +1.2) was a REGRESSION: it corrected
for the My flip but not the alpha flip, which cancels it.

FIXED (`extensions/.../dynamics/vtol_aerodynamics.py`): the model now converts **FLU->FRD on the way
in and back on the way out**, so every coefficient is the STANDARD published value and no term ever
needs a frame sign-patch again. Also: stall now blends into a Newtonian flat plate (the old model
clipped alpha for the COEFFICIENTS but used the raw alpha for the lift DIRECTION -> 15 N of
downforce while backing up, 10 N backward force in a vertical descent); and the aero force is applied
at `VTOLConfig.aero_center` (default = the CG) instead of the body prim ORIGIN, which was an
invisible 0.176 m lever arm = an implicit aerodynamic centre at 71% MAC contributing 3x more pitch
moment than the entire Cm model, and silently re-tuned every time `body_com_offset` changed.
Also fixed: HIL watchdog in `vtol_mavlink_backend` (0.25 s of silence -> safe command, instead of
holding the last one forever) and its mutable default args.

**The corrected plant is a normal aeroplane:** pitch stiffness +16.5 N.m/rad (25% static margin),
short period 1.3 Hz at zeta=0.33, level trim +5.6 deg nose-UP at 16 m/s, stall ~11.5 m/s, wing-borne
14-18 m/s at +4..+8 deg AoA. Regression tests: `tests/test_vtol_aerodynamics.py` (run with the
isaac_env python; 26 physical-property assertions, no Isaac needed).

**CONSEQUENCE — every claim below about AoA sign, pitch direction, wing downforce, "nose-down for
lift", climb-away and the corridor was measured on the INVERTED plant.** `[AERO] aoa=` now prints
the STANDARD angle (positive = nose-up), so old bags/logs have the opposite sign.

**THE CONTROLLERS HAVE NOT BEEN CHANGED and now model the WRONG plant** (deliberate: half-fixing
them would be worse). All 8 model-based nodes use `alpha=atan2(vb[2],|u|)` with CLA=+4.8/CMA=+1.2,
AND their reference pitch is nose-DOWN-positive (`_R_pitch`/`R_ref` build `Ry(+theta)` while the trim
schedules produce standard nose-UP theta) -- the two inversions cancelled against the old sim.
Verified: the node's `theta=+8 deg` reference is 8 deg NOSE-DOWN; its own aero says +37.9 N, the
corrected sim says -22.9 N. Four more shared model errors found in the controller audit:
  * inertia = the BODY LINK's `(0.20,0.28,0.35)` (and even that is the VTOLConfig DEFAULT, not the
    CAD airframe's 0.20/0.23/0.36); the ASSEMBLED vehicle is `(0.265,0.303,0.490)` -> Ixx -24%,
    Izz -29%, i.e. the two axes that stayed fragile. Same class of bug as the CG error.
  * `CQ=0.06` vs the sim's `rotor_moment_ratio=0.12` -> HALF the real yaw torque (and yaw is the
    only hover actuator for it). With the Izz error the true yaw response is ~2.8x the modelled one.
  * `RPOS[:,2]=0.02` for all four rotors; the real front HUB is z=-0.033 rel CG (WRONG SIGN). Zero
    effect in hover, dominant at 90 deg tilt where `My = z_arm*Fx` IS the pitch moment.
  * **the offline selftests integrate the controller's OWN `build_ode` as "the true plant"**, so
    none of the above can ever show up offline -- exactly why "NONE reproduce in the offline test".
Ordered repair plan (cache key -> plant double -> constants -> conventions -> re-tune -> fly) and
the full finding list are in the audit; also C7 (the SMC re-seed fires on TRACKING LAG, which is the
sawtooth that `SMC_LEAD=500` papers over), C8 (`vtol_nmpc` lost the z-ramp), C9 (`latency_s` 0.10 and
the loop-exception/safe-hover guards were never back-ported out of `vtol_nmpc`), C10 (three different
tilt-servo models in one loop), C11 (`dt_ref=1/30` hardcoded in the feed-forward).

# VTOL (dual independent tiltrotor) — bring-up assets

Config-driven articulated VTOL for Isaac Sim / Pegasus, driven by PX4 SITL over
`direct_actuator` offboard HIL. Full operational guide (staged validation, start
order, verification, troubleshooting): **`docs/vtol_bringup_runbook.md`**. Deep
DDS/offboard rationale: `docs/direct_actuator_offboard.md`.

End-to-end data flow:
```
ROS2 direct_actuator node ─DDS→ PX4 SITL (airframe 4100) ─HIL(4560)→ Isaac VTOL
Isaac VTOL ─(opt. ROS2Backend)→ /uav_0/state/* ─(mocap emulator)→ /uav_0/mocap ─→ [controller: TODO]
```

## Isaac side
- `extensions/pegasus.simulator/.../logic/vehicles/vtol.py` — generic `VTOL` +
  `VTOLConfig` + `RotorConfig`/`SurfaceConfig`; airframe = data (lists of rotors/
  surfaces). `dual_tiltrotor_config()` is the target airframe. `apply_force` uses
  the link-**local** frame, so thrust on a tilted prop vectors automatically.
  `_validate_channels()` warns once if a command's channels don't match the airframe.
- **Rotor + servo dynamics** (opt-in `VTOLConfig.rotor_dynamics=True`, default off):
  each rotor has a speed state omega that chases `throttle*rotor_omega_max` with a
  first-order lag (`rotor_tau_spin`); thrust = `max_thrust*(omega/omega_max)^2` (lags
  throttle, ~RPM^2); yaw reaction torque = `rotor_moment_ratio*thrust*(-spin_dir)`;
  tilt servos become a first-order lag (`tilt_tau`) capped at the slew rate. Off =
  legacy instantaneous `throttle*max_thrust` + visual-only spin. Demo:
  `application/vtol/06_vtol_rotor_physics.py` (step inputs, logs omega/thrust/tilt).
- `.../logic/dynamics/vtol_aerodynamics.py` — lumped lift/drag/moment model.
- `.../logic/backends/vtol_mavlink_backend.py` — `PX4VTOLMavlinkBackend` maps PX4
  `HIL_ACTUATOR_CONTROLS.controls[]` → VTOL channel dict via a channel_map.
- `.../rotorcraft/vtol_utils.py` — `spawn_vtol` / `spawn_vtol_with_mavlink`.
  The mavlink spawner auto-selects a channel_map matching the config (raises on no
  match) and takes `publish_state_ros2=` (opt-in state publisher, see Mocap).
- Scenes `application/vtol/`: `01_basic_vtol` (joints only), `02_vtol_hover`
  (forces + stand-in controller), `03_vtol_px4_hil` (full HIL; reads `VTOL_ROS2_STATE`).

## PX4 side — airframe `4100_fsc_dual_tiltrotor`
Source of truth: `px4/airframes/` (+ README). Installed into
`$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/`, registered in that dir's
`CMakeLists.txt`; rebuild with `make px4_sitl` (or use the pre-dropped copies in
`build/px4_sitl_default/{etc,rootfs}/etc/...`). Boots via `PX4_SYS_AUTOSTART=4100`.
**Actuator output mapping is designed to match `default_dual_tiltrotor_channel_map()`
AND the ROS2 node's MOTOR/SERVO order** — verify with `PEGASUS_HIL_DEBUG=1`:

| `PWM_MAIN_FUNC` | function | HIL `controls[]` | channel |
|---|---|---|---|
| 1=101,2=102 | Motor1,2 | 0,1 | motor_left, motor_right |
| 3=201,4=202 | Servo1,2 | 2,3 | tilt_left, tilt_right |
| 5=203,6=204,7=205 | Servo3,4,5 | 4,5,6 | aileron, elevator, rudder |

CA_ (control allocation) geometry is best-effort and **bypassed** by direct_actuator;
a 2-rotor tiltrotor is under-actuated for PX4 hover allocation, so arming may need
preflight relaxation. Airframe bakes `UXRCE_DDS_SYNCT=0`, `COM_DISARM_LAND/PRFLT=0`.

## ROS 2 side (`~/ros2_ws`, Humble, px4_msgs 2.0.1)
- `ros2/px4_vtol_direct_actuator/` — streams `ActuatorMotors`+`ActuatorServos`
  (MOTOR_ORDER=[motor_left,motor_right], SERVO_ORDER=[tilt_left,tilt_right,aileron,
  elevator,rudder]) and drives OFFBOARD+ARM. Symlink into `~/ros2_ws/src`, build.
- `isaacsim_optitrack_ros2_emulator` — mocap tool; subscribes `/uav_0/state/*`,
  publishes `/uav_0/mocap` (`fsc_autopilot_ros2_msgs/Mocap` = header+pose+twist).

## Mocap pipeline (feeds the controller)
`VTOL_ROS2_STATE=1` (env-gated, **off by default**) makes `spawn_vtol_with_mavlink`
attach a state-only `ROS2Backend` (namespace `uav_`) → `/uav_0/state/{pose,twist,
twist_inertial}`. Then run the emulator → `/uav_0/mocap`. The controller consuming
`/uav_0/mocap` → direct_actuator setpoints is **not written yet (TODO)**.

## Scripts (`scripts/`)
- `start_vtol_hil.sh <cfg>` — one command, 4-pane tmux HIL: DDS agent + PX4 + Isaac
  + ROS2 node. Env overrides: `AUTO_ARM`, `MOTOR_*`, `TILT_*`, `VTOL_ROS2_STATE`,
  `PEGASUS_HIL_DEBUG`, `ROS_DISTRO`, `ROS2_WS`, `PX4_SIM_MODEL`. Opens a **new
  terminal** by default (`--here` to run in the current one).
- `stop_vtol_hil.sh` — teardown (VTOL-anchored targets; `--dry-run`/`--keep-agent`/
  `--tmux-only`).
- `start_vtol_mocap.sh` — runs the mocap emulator (needs `VTOL_ROS2_STATE=1` in Isaac).
- `start_vtol_sitl.sh` — PX4+Isaac only (subset of the HIL launcher).

## Importing a CAD airframe (mesh -> USD -> config-driven links)
`vtol.py` links can use imported CAD meshes instead of procedural boxes. Each
`RotorConfig`/`SurfaceConfig` has `mesh` + `mesh_scale`/`mesh_offset`/`mesh_rpy_deg`,
and `VTOLConfig` has `body_mesh*`. A rotor's `mesh` attaches to its **prop link**
(tilts via the now-invisible nacelle for tiltable rotors, spins, and is the thrust
point); `None` -> procedural box (mix freely for incremental bring-up). Mesh links
are visual-only (no PhysX collision — a dynamic triangle mesh is invalid; add
convex collision in-GUI if you need ground contact). **Each articulated link needs
its own separate mesh** (a link can't share geometry with the body).

Workflow: (1) export each CAD component (body + one file per rotor) as a mesh
(STL/OBJ/FBX/glTF); (2) `application/vtol/convert_meshes_to_usd.py --input <dir|file>
--output <dir|file>` (run with Isaac python, headless) -> USD; (3) use
`quad_tiltrotor_config(body_mesh=, front_left_mesh=, ..., mesh_scale=)` (2 front tilt
+ 2 rear static — matches a "2 tilting + 2 static rotor" airframe) and view/verify
with `application/vtol/04_vtol_cad.py`. `mesh_scale=(0.001,)*3` if the CAD is in mm.
Part origins should sit at each joint pivot; otherwise fix in the mesh tool or use
`mesh_offset`/`mesh_rpy_deg`. Rotor hub positions in `quad_tiltrotor_config()` are
placeholders — measure them off the CAD.

## FSC CAD airframe (wired into HIL)
The real airframe is the CAD tilt-quadplane (2 front tilting + 2 rear static rotors),
built from the imported STL. `fsc_cad_vtol_config()` (fsc_aerial_manipulation.rotorcraft)
= `quad_tiltrotor_config()` wired to the converted part USDs (`$VTOL_CAD_USD_DIR`,
default `~/vtol_cad_usd/parts2_usd`) with tuned hinge (front `position≈(0.459,±0.284,
-0.067)`, `spin_offset≈(0,0,0.09)`) + `mesh_scale=0.001`, `mesh_rpy_deg=(90,0,90)`.
- **STL→parts**: `application/vtol/split_cad_stl.py` (needs trimesh; connected-component
  split + region/servo/plate rules + `HINGE_OFFSET_MM`) → OBJ → `convert_meshes_to_usd.py` → USD.
- **Nacelle vs prop**: `RotorConfig.nacelle_mesh` (arm+motor, tilts, no spin) on the
  nacelle link; `mesh` (prop) on the prop link; `spin_offset` puts the prop link on
  its shaft so it spins in place.
- **Surfaces**: 2 ailerons (opposite `deflect_scale` ±1 -> roll), 1 elevator, 1 rudder
  — procedural boxes in `quad_tiltrotor_config()`, `SurfaceConfig.size`/`deflect_scale`.
- **HIL**: `03_vtol_px4_hil.py` spawns `fsc_cad_vtol_config()`; channel_map auto-selected
  to `default_quad_tiltrotor_channel_map()` (controls[0..3]=motors FL/FR/RL/RR,
  [4..5]=tilt L/R, [6..8]=aileron/elevator/rudder). PX4 airframe `4101_fsc_quad_tiltrotor`
  (px4/airframes/, registered, rebuild PX4). `start_vtol_hil.sh` defaults `PX4_SIM_MODEL=fsc_quad_tiltrotor`; node
  pane params `motor_front_L/R`, `motor_rear_L/R`, `tilt_left/right` (env `MOTOR_FRONT`,
  `MOTOR_REAR`, `TILT_*`). ROS2 node MOTOR/SERVO order updated to match.
- **Viewer**: `06_vtol_rotor_physics.py` (forces off) shows the airframe + a red pivot
  marker at each tilt joint + surfaces wiggling.
- **Physics model**: `07_vtol_cad_physics.py` — forces ON (thrust∝RPM², yaw torque,
  aero, gravity) held up by the shared `hover_controller.py` `HoverControllerBackend`
  (allocation from rotor positions; inverts the RPM² thrust when `rotor_dynamics`).
  Same controller now used by `02_vtol_hover.py` (deduped). Mesh bodies get an
  invisible box collider (`VTOLConfig.body_collision_size`) so they don't fall through.
- **External ROS2 control loop** (no PX4): `08_vtol_hover_control.py` runs the CAD
  VTOL with `VTOLCommandROS2Backend` (subscribes `/uav_0/vtol/actuator_cmd`) +
  state-publishing `ROS2Backend`. The controller is a ROS2 package
  `~/ros2_ws/src/fsc_vtol_controller` (`vtol_hover_controller` node): `/uav_0/mocap`
  -> hover/attitude PID -> `/uav_0/vtol/actuator_cmd` (9-ch Float64MultiArray, tilt
  held fixed, all gains/mass/max_thrust live `ros2 param`s). Launcher
  `scripts/start_vtol_hover_control.sh <cfg>`: tmux Isaac + mocap emulator +
  controller + monitor (Isaac bundled rclpy ↔ system ROS2 over FastDDS, same domain).

## Control-loop options + controllers (position/trajectory tracking)
Three ways to fly the CAD quadplane; all consume the SAME reference topic
`/uav_0/fsc_autopilot_ros2/position_controller/reference` (pos+vel+yaw), so the
ground-station Trajectory tab drives any of them.

- **KEY FINDING (CORRECTED) — direct_actuator DOES track trajectories in HIL; the earlier
  tumble/fly-away was a CONTROL-LAW bug, NOT a latency wall.** Two root causes, both fixed:
  (1) a CG bug (`body_com_offset` 0.174→0.195, see below) caused the −38° liftoff pitch;
  (2) the outer position loops IGNORED the trajectory's velocity/accel FEED-FORWARD, so
  tracking relied entirely on delayed feedback → oscillation/fly-away on the laggy HIL plant
  (this is what got mislabeled a "latency wall"). The fix is **`traj_track_node.py`** — an
  SE(3) geometric attitude law + velocity/accel FEED-FORWARD on the reference. VALIDATED
  live (2026-08): re-arms from the ground, hovers dead-on (errx=0.00), and tracks a 40 m
  out-and-back at **mean|err_x|=0.86 m (max 1.92 m), altitude held 0.18 m mean, stable, no
  fly-away** (log: `/tmp/track_out.txt`). So BOTH paths are viable: the PX4 inner loop is
  tightest for fast *attitude*, but direct_actuator + feed-forward tracks *trajectories* fine.
  - **`traj_track_node.py`** (`ros2/px4_vtol_direct_actuator/...`): `method=se3`,
    `controller_id=geometric`, `auto_arm`, 200 Hz. Position outer loop with vel+accel FF →
    `F_des`; lean-capped; SE(3) attitude (`kR/kW`); allocation `Ainv@[f,Mx,My,Mz]` to 4
    rotors from CG-relative geometry. Robustness: DEFAULTS to a hover ref at the current
    x,y (z=1.5) until a trajectory arrives (publishing NaN actuators with no ref stalls the
    PX4 arm handshake — this was the "flaky arm"); FF clamped against reference jumps.
  - **PID+FF (band-limited integral) — VALIDATED, the best variant.** A naive position
    integral WINDS UP on this laggy plant and diverges (ran away, flipped the vehicle
    inverted). Band-limiting fixes it: `i_band`=1 m (integrate only near the set-point,
    FREEZE during fast tracking so no windup) + `i_max` anti-windup clamp + reset on ref
    jump + gated on armed. Result (same 40 m out-and-back): **mean|errx|=0.32 m (vs 0.86 m
    for PD+FF — 63% better), max 1.40 m, altitude 0.19 m mean; hover dead-on (errx=0.00),
    settles to ±0.02 m at each hold.** `ki_xy=0.4, ki_z=0.8` (log `/tmp/pid_final.txt`).
  - **Tilt-for-forward (rotors tilt during cruise) — VALIDATED.** During a forward traverse
    the FRONT rotors tilt forward instead of the airframe pitching over. It is COORDINATED:
    the tilt is sized to the forward-force demand (`tilt=atan2(F_fwd, ½·F_up)`, capped by
    `tilt_max_deg`=35°), that force is then REMOVED from the SE(3) pitch target (no pitch/tilt
    double-counting), and the front pair's collective is boosted by 1/cos(tilt) so their
    VERTICAL thrust (altitude + pitch balance) is preserved. Gated on forward speed
    (`tilt_v_gate`=0.4 m/s) so hover/hold stays a clean multicopter `tilt=0`. Live: tilt ~18°
    at 1 m/s cruise (scales to the 35° cap for faster), hover `tilt=0`, tracking
    mean|errx|=**0.31 m** (same as no-tilt) + altitude 0.15 m. NOTE this is *tiltrotor forward
    flight* (rotors vector thrust), NOT a wing-borne fixed-wing cruise transition (that still
    needs the tilt+aero NMPC). Set `tilt_max_deg=0` to disable (pure multicopter).
  - **Precise-landing mission — VALIDATED.** Flew takeoff→diagonal traverse (0,0)→(12,6)→
    descend→land: **landed at (11.98, 6.00), 0.02 m from the commanded target** (log
    `/tmp/mission_out.txt`). Point-to-point tracking stayed within 0.15 m; the descent held
    the target line to ±0.02 m. This is the "land at a precise point" goal.
  - **Launcher `scripts/start_vtol_traj_hil.sh <cfg>`** — DDS+PX4+Isaac+mocap+estimator +
    traj_track (ACTIVE, `controller_id=geometric`) + GS. It is GS-drop-in: the Trajectory tab
    reference + the "Safe Geometric" switch button drive it (Play/HOVER/reference-dropdown).
  - **Test without rebooting:** the flaky-arm/tumble grinding traced largely to rebooting the
    whole HIL stack every run. Keep DDS+PX4+Isaac+mocap UP; only relaunch the controller
    against the live stack (arm was reliable every time that way). **To reset a crashed/inverted
    vehicle you MUST restart PX4 (fresh EKF) BEFORE Isaac** — restarting Isaac ALONE leaves
    PX4's EKF diverged ("Preflight Fail: Attitude failure (roll)" → cannot arm). Procedure:
    `pkill -9 -x px4` (kills only the binary, not the tmux bash wrapper → the pane survives),
    then `tmux send-keys -t <sess>:1-px4 "<px4 launch>" Enter`; then relaunch Isaac
    (`03_vtol_px4_hil.py`, it connects to PX4:4560); the mocap emulator reconnects on its own.

- **fsc autopilot (`autopilot_sv_baseline_node`)** — the rock-solid hover (sub-mm at
  (0,0,1.5)). PX4 inner-loop, publishes `VehicleAttitudeSetpoint`+`OffboardControlMode
  (attitude=true, thrust_and_torque=true)`. Launcher `scripts/start_vtol_fsc_hil.sh`.
  Arm via the helper window: `fsc_takeoff 1.5` then `fsc_stopref` to hand the reference
  to the tab. It ALSO tracks a moving reference, so it can fly trajectories too.

- **attitude-setpoint NMPC** (`.../px4_vtol_direct_actuator/.../nmpc_attitude_node.py`) —
  outer-loop CasADi NMPC: state=[pos,vel] (ENU), control = thrust-accel vector (tilt≤30°
  + thrust-mag constrained); converts the optimal thrust vector → desired attitude quat
  (NED, via fsc's `getNEDqFromENUq` = q(Rie)·q_enu·q(Rbv), Rie=[[0,1,0],[1,0,0],[0,0,-1]],
  Rbv=180°-about-x) + normalized collective thrust; publishes the SAME PX4 attitude
  interface. Stable in HIL (hovers ~0.25 m). Launcher `scripts/start_vtol_nmpc_hil.sh`
  (`NMPC_MODE=attitude` default; `direct` = raw actuators, sim-only). Gentle-hover gains:
  w_pos5/w_vel20/max_tilt18 + governor sp_max_vel 0.6. `hover_throttle=0.70` (HIL-measured;
  was 0.46 -> under-thrust), `a_min=7` (0.7g floor; 3.0 let it near-free-fall -> vertical
  limit cycle). Hovers cleanly at 1.5 (x,y ±7 cm) when the GS drives the reference.

- **TWO-WAY inner-loop switch — `scripts/start_vtol_inner_hil.sh` (the "ship" path).** Runs
  BOTH inner-loop controllers (fsc = safe hover, attitude-NMPC = tracking) behind the GS
  switch, with `attitude_setpoint_mux.py` (NEW): each controller's VehicleAttitudeSetpoint is
  launch-time REMAPPED to /<ns>/mux/{fsc,nmpc}/... and the mux forwards ONLY the active one to
  /<ns>/fmu/in/vehicle_attitude_setpoint (both keep sending OffboardControlMode+arm to the real
  topics -> idempotent -> NO controller code change). GS "Safe Geometric"/"NMPC" buttons publish
  "geometric"/"nmpc" (mux maps "geometric"->fsc), so it drops in with no GS change. Mux switch
  VERIFIED (toggles nmpc<->fsc; NMPC hovers at 1.5). **fsc side needs engagement**: run
  `fsc_takeoff 1.5` in the helper window (win 9) once -- the fsc has its own arm/takeoff state
  machine, so a reference alone doesn't make it emit setpoints (else switch->fsc = `forwarded=0`
  -> drift). After that the switch is seamless.

- **SMC outer loop + A/B inner switch (`scripts/start_vtol_smc_hil.sh`).** An outer-loop
  sliding-mode controller (`smc_outer.py`: `v=-λ·e−K·sat(e/Φ)` advancing a CONVERGING stationary
  set-point; LOOSE cruise `vmax=20 m/s` vs STRICT landing, by target z) drives a GS-switchable
  inner loop ("A: Geometric+SMC" / "B: Attitude-NMPC+SMC" -> `active_controller`=geometric|attnmpc):
  - **A = `smc_outer_node.py` → `traj_track_node.py`** (reference remapped to `.../reference_smc`).
    RELIABLE, DEFAULT. Reads state from MOCAP + streams direct_actuator → NEVER uses the px4 EKF →
    immune to HIL EKF flakiness. Flew takeoff→fly-to-spot→hover@spot 0.04 m→land 0.00 m; live it
    holds a steady hover 20 m out.
  - **B = `nmpc_attitude_node.py`** (SMC governor + attitude setpoints, px4 inner loop). Tracks
    against the px4 EKF. Two bugs found: `hover_throttle` 0.70→**0.50** (0.70 over-scaled → could
    never descend → climb-away); and **EKF HEIGHT divergence** (live: px4 `z_ned=+4.86` vs true
    −1.41, ~6 m too low → B over-thrusts → fly-away). NOTE: the EKF *yaw* is FINE (sim mag; live
    mocap-vs-EKF yaw DIFF=0.0° — the earlier "yaw off" diagnosis was wrong). Diag: `/tmp/diag_ekf.py`.
  - **EV bridge `mocap_to_px4_ev.py` (makes B viable).** mocap (ENU/FLU) → `VehicleOdometry`
    (NED/FRD) on `/<ns>/fmu/in/vehicle_visual_odometry`; stamps in PX4 time via a ROS↔PX4 clock
    offset learned from `/fmu/out/vehicle_attitude` (works with `UXRCE_DDS_SYNCT` 0 or 1). Verified:
    emits `z_ned=−1.41` (correct) vs the EKF's diverged +4.86. Engage: `EKF2_EV_CTRL=11`
    (hpos+vpos+yaw) `EKF2_HGT_REF=3` `EKF2_GPS_CTRL=0` `EKF2_EV_DELAY=50` then `ekf2 stop; ekf2 start`
    — EKF2_* need an ekf2 RESTART, so do it PRE-ARM at boot (launcher 8-relax window), NOT mid-flight
    (offboard failsafes on the estimator gap). Launcher wires it in window 4b-ev + the relax window;
    env knobs `EV_BRIDGE=1`, `DISABLE_GEOFENCE=1` (GF_*=0), `SMC_VMAX=20`.
  - **VALIDATED (2026-08-08):** with EV the EKF `z_ned` went +4.86 (6 m wrong) → −1.50 (exact), and
    **B then HELD HOVER 22 s (maxZ 1.94, maxR 0.28 m)** — vs the 160 m climb-away before. Residual: a
    mild ~+0.3 m altitude bias on B (tune `hover_throttle`/`w_pos`,`w_vel`). A↔B switch is clean; A
    returns to a dead-on (0,0,1.5) hover after B. `/tmp/diag_ekf.py` = EKF-vs-mocap diff tool.
  - **CRUISE / TILT-TRANSITION LIMIT (2026-08-08) — A cannot do wing-borne cruise.** The GS "Cruise
    speed" spinbox was capped at 10 m/s (default 1) → the vehicle never reached wing-borne speed →
    the tilt (force-sized `atan2(F_fwd,half_up)`) stayed ~0 → "tilt never changes". Cap raised to 20
    (`trajectory_player.py`), default kept LOW (2). The AERO MODEL IS REAL (full linear fixed-wing,
    `vtol_aerodynamics.py`, applied every step): a 3.4 kg airframe needs **8–14 m/s** for the wing to
    carry its weight (lift∝V²). BUT the geometric controller (A) does NOT model wing lift, so at
    cruise speed it is UNSTABLE: an airspeed-scheduled tilt→70° made the WING OVERPOWER it →
    **climb-away to 110 m**; reverting to force-based tilt (cap 45°) at a 6 m/s traverse made it
    **descend and crash** (lean loses vertical thrust). So the PLAIN geometric controller (A) is a
    hover / ≲4–5 m/s-traverse controller only.
  - **AERO-AWARE TILT TRANSITION — SOLVED (2026-08-09), `aero_transition_node.py`.** The fix is aero
    FEED-FORWARD: subtract the MODELED wing lift/drag (same coeffs as `vtol_aerodynamics.py`) from
    F_des so the rotors back off as the wing takes over → holds altitude through the transition.
    Offline-validated (`/tmp/nmpc_transition_offline.py` = the receding-horizon CasADi NMPC;
    `/tmp/aeroff_offline.py` = the real-time aero-FF feedback law: aero-FF OFF 3.76 m alt loss vs ON
    0.42 m). The online node is the aero-FF law (robust at 200 Hz; it mirrors the NMPC's steady-state).
    Tilt is COORDINATED: airspeed-gated cap (tilt far only once the wing carries weight), servo slew
    limit, and a YAW-ALIGNMENT gate (yaw to face travel first — the body-frame tilt else pushes the
    wrong WORLD way). Mocap-based (no EKF). Launcher window `6b-transition` (controller_id=transition,
    SMC-fed); activate with active_controller=transition. **HIL VALIDATED — FULL MISSION (fly +y at
    vmax=16): takeoff → transition (tilt 12°→52°, accel to 16.4 m/s) → cruise → hover@spot (0,79,3) →
    clean vertical landing at (0.00,78.9,0.15). Altitude held ±0.15 m through the whole transition,
    dead-straight.** Higher vmax → higher tilt (14→~29°, 16→52°, offline 20→80°).
  - **OUT-AND-BACK (return trips) — VALIDATED.** Two additions handle a 180° turn-around: HOLD-AND-YAW
    (when the nose isn't aligned with travel, hover + yaw first instead of chasing) + YAW-CMD SLEW
    (`yaw_rate_deg_s`) so the attitude heading turns gradually — avoiding the SE(3) 180° singularity
    that had dropped it. **HIL: takeoff → transition out to (0,70,3) → 180° yaw-around (altitude held
    3.02–3.07 m) → transition back → land near origin, altitude held ~3.0 m throughout.** Guidance must
    point the reference yaw at the target (see `/tmp/mission_outback.py`). GS button "C: Aero Transition"
    added. Each HIL crash inverts the vehicle → full stack restart to recover.

- **direct_actuator controllers** (`nmpc_quad_node.py` = CasADi NMPC → actuators;
  `tiltrotor_control_node.py` = geometric PID → actuators, tilt-for-forward). These two were
  the ones that "confirmed the latency wall" — but that was the missing-feed-forward bug (see
  KEY FINDING); `traj_track_node.py` supersedes them for tracking. Both take `controller_id`
  + `active_at_start` and
  gate PX4 output on a std_msgs/String `/<ns>/ground_station/active_controller` — only the
  ACTIVE one publishes (the other is standby but keeps computing). This is the **two-way
  controller SWITCH**: `scripts/start_vtol_dual_hil.sh` runs both (geometric active, nmpc
  standby) + estimator + GS; the Trajectory tab has Safe-Geometric/NMPC buttons
  (HOVER→geometric, Play→nmpc, geofence-bail→geometric). Switch infra works. (These two
  legacy direct nodes lacked feed-forward → fly-away; use `traj_track_node.py` — see KEY FINDING.)
  - **ROOT CAUSE FOUND + FIXED — it was a CG bug, not (mostly) latency.** The ~−38° liftoff
    pitch disturbance was `body_com_offset` set to 0.174 (rotor centroid, IGNORING the
    articulated-link masses). Front rotors carry 2 links each (tiltable nacelle + prop) vs 1
    (prop) for the rear, plus 4 aft surfaces — all pull the TOTAL CG ~2 cm AFT of the rotor
    centroid where thrust acts → symmetric thrust = ~0.6 N·m nose-up = the disturbance. FIX:
    `fsc_cad_vtol_config` body_com_offset x → **0.195** (total CG on the rotor centroid; 0.195
    was the correct value wrongly reverted to 0.174). With it, the direct geometric controller
    **hovers at z=1.5 (pitch ±7°) for ~13 s** — vs the immediate tumble before. Open-loop probe
    (`/tmp/probe2.py`) had confirmed all axis mappings are correct (not a sign bug); yaw
    divergence was ground-contact (fixed by decisive liftoff + `kp_yaw`). The ~13 s residual
    fly-away was NOT a latency limit — it was the position loop lacking velocity/accel
    feed-forward (adding it in `traj_track_node.py` tracks a 40 m round trip; see KEY FINDING).
    Geometric node also has structural fixes (velocity-profiled altitude w/ sqrt stopping profile,
    takeoff punch, altitude-gated position hold, attitude-priority mixer desaturation, pitch_ff off).
  - **fsc_autopilot_ros2-packaged variant** (only-adds-files): the NMPC direct_actuator
    node is also placed in the fsc package as `fsc_autopilot_ros2/fsc_autopilot_lib/
    single_vehicle_baseline/modules/position_control/python/nmpc_vtol_control.py` (copy of
    gated `nmpc_quad_node.py`, controller_id=nmpc; run via `python3`, no CMake change).
    Launcher `scripts/start_vtol_nmpc_fsc_hil.sh` = a version of `start_vtol_fsc_hil.sh`
    that runs the geometric direct_actuator as the ACTIVE hover-safety + this NMPC node
    STANDBY (instead of the fsc attitude autopilot) + estimator + GS + helper. GS
    Safe-Geometric/NMPC buttons activate the NMPC to track the reference. Same
    direct_actuator stability caveat as above.

- **★ IN-LOOP TRANSITION NMPC — WORKS, the cruise problem is SOLVED (2026-08-09).**
  `ros2/px4_vtol_direct_actuator/.../nmpc_cruise_node.py` (controller_id=`nmpc_cruise`, launcher
  window `6c-nmpc-cruise`, GS button "D: NMPC Cruise"). Longitudinal NMPC (CasADi/IPOPT, state
  `[s,z,vs,vz]` along the TRUE body heading, control `[Tf,Tr,beta]`, wing aero in the model,
  N=15/DT=0.08, ~8 ms/solve) co-optimizes tilt+thrust, PAIRED with a gentle rate/saturation-limited
  attitude+lateral+pitch loop. **FULL MISSION HIL-VALIDATED: hover → climb → cruise to a 40 m target
  (sustained ~5 m/s via GENUINE TILT 18–21°, altitude held ±0.2 m) → precise arrival at (40.0,0,4.0),
  no overshoot → precise LANDING at (40.1,0,0.3) dead-on.**
  - **★★ CORRECTION (2026-08-10, sim-force-verified): the "wing-borne" claim below was OVERSTATED.**
    Re-ran with the sim's own `[AERO]`/`[PROP]` force prints. What nmpc_cruise actually does at "cruise":
    the WING does carry ~49% (sim `[AERO] Fbody_z=+16.9 N` lift at AoA +5°, V=11.5) and rotors unload to
    ~16.8 N — BUT the **rotors stay VERTICAL (tilt≈0)**, so it is a wing-ASSISTED multicopter drift, NOT
    airplane-mode flight (no forward-thrust-by-tilt), and it is NOT robust: on a long cruise it degrades
    to high AoA and **stalls/crashes** (watched it hit the ground at ~1200 m). So: partial (~50%) wing
    OFFLOAD, stable only ~400-500 m, not airplane-mode. **REAL airplane-mode (rotors tilt forward, wing
    carries ~all the weight, +AoA) is NOT achieved** — `aero_transition_node` reactive mode gives a stable
    tilted cruise but NOSE-DOWN with wing DOWNFORCE (tilting props forward makes a nose-down moment → −AoA);
    its snap-to-80° cruise mode CRASHES at the onset. See the airplane-mode discussion. Original (overstated)
    notes kept below for the mechanics.
  - **★★ WING-BORNE UPGRADE — 5-STATE PITCH NMPC (2026-08-09). GENUINE POSITIVE-AoA WING-BORNE CRUISE
    ACHIEVED + rosbag-verified.** The state is now `[s,z,vs,vz,theta]` (theta = BODY PITCH, nose-up +)
    and control `[Tf,Tr,beta,q]` (q=pitch-rate): the solver CO-OPTIMIZES pitch WITH tilt+thrust (`_f`
    rotates the rotor thrust by theta; aero AoA = theta − flight-path). **HIL result (bag wb10/wb11):
    hover@8 m → accelerate on TILT (16→23°, body level) → at ~11 m/s TRANSITION (tilt→0, pitch nose-UP
    +5°) → sustained WING-BORNE cruise: AoA=+5°, altitude 8.0 m DEAD-FLAT, rotors unloaded to the floor
    (16 N — the WING carries ~53% of weight), 11.5 m/s, ~45 s / 490 m steady, recovers from
    disturbances.** The path to it (all HIL-diagnosed via rosbag, `/tmp/anabag.py`):
    (1) **monotonic SMC** — `SMC_LEAD=500` (reset_dist huge → never re-seeds) killed the 4 s setpoint
    SAWTOOTH that had capped speed ~5 m/s and driven a pitch limit-cycle; smooth velocity FF → smooth
    tilt → speed builds. (2) **speed-gate via theta-weight** (`wp[2]=1.0`) + **slow pitch trim**
    (`Q_MAX=7°/s`, tight bounds `TH_UP=9/TH_DN=3`, `theta_lp` low-pass) — pitch is a gentle trim, not a
    fast altitude actuator (that limit-cycled through the HIL latency). (3) **rotor FLOOR** (`rotor_min`=8 N
    NMPC bound) — rotors never fully unload so they RETAIN differential-thrust pitch/roll authority (full
    unload → pitch ran away to stall). (4) **speed-scheduled, altitude-regulated nose-up TRIM**
    (`cruise_trim_deg`=3, `k_trim_alt`; `k_trim_vz`=0 — the climb-rate feedback set up a limit cycle) →
    the body targets +AoA for wing lift but backs off if it climbs. (5) **DAMPED aero surfaces** — route
    the FULL damped moment `M=−kR·eR−kW·eW` to the surfaces: `elevator=k_elev·M[1]`, `aileron=k_ail·M[0]`,
    `rudder=k_rud·M[2]` (k_elev/ail=−0.15, k_rud=−0.10, airspeed-gated). Position-only (`eR`) surfaces ran
    the pitch AWAY (no damping); the moment-matched DAMPED surfaces give authority WITHOUT runaway, and the
    ailerons fix a slow ROLL divergence that tumbled the elevator-only version at ~50 s. Nose-DOWN is
    tightly capped (`TH_DN=3`) so forward thrust comes from the TILT (tiltrotor), keeping the body nose-up.
    - **Ops:** SMC-fed (`start_vtol_smc_hil.sh`, env `SMC_VMAX=11 SMC_LEAD=500`; 6c passes vmax_fwd/a_fwd/
      pitch_rate + reads reference_smc). Drive by publishing the RAW reference (pubraw → smc_outer → nmpc).
      Cruise altitude must be > `cruise_alt_min`=4. Take off on `geometric` then switch
      `active_controller`=nmpc_cruise (the NMPC does NOT take off cleanly from the ground). NEVER
      mid-flight node restarts — one test = one fresh stack (`bash /tmp/teardown.sh` + relaunch), or a
      corrupted state carries into the next run.
    - **WING-BORNE (`cruise_trim_deg`≈3)**: tilt→0, body nose-up +5°, wing carries ~53%, rotors
      unloaded. Lightly-damped roll limit-cycle (±18°, recovers) can appear on >~500 m legs.
    - **DECELERATION / fly-to-point+land.** Root weakness: a tiltrotor can't reverse-tilt (beta≥0) and
      at tilt=0 the wing-borne pitch trim holds the body LEVEL, so it has little forward-braking authority
      and coasts far past a finite target (commanded 120 m → landed 200-270 m before the fix). Two fixes:
      (1) **`vff` fix** — use the reference velocity directly, don't hold the last non-zero value (holding
      it kept the vehicle cruising when a mission's FF decayed to 0, ~doubling the overshoot). (2)
      **DECELERATION BRAKE** (`k_brake`,`brake_max_deg`) — in HOLD, pitch nose-up ∝ forward speed so the
      thrust tilts BACKWARD and brakes. Fires only in hold (`vs>0.3 or e_fwd>0.5`), so a far-target
      wing-borne CRUISE (hold=0 throughout) is untouched. **Result: overshoot 200-300 m → ~20-30 m**
      (fly-to-(120,0)+land landed 94-116 m across runs, dead-on in y). The brake suppresses full
      wing-borne DURING a point-to-point mission (it slows the traverse to ~3 m/s), so a mission is
      tilt-cruise+precise-ish, while a far constant target is full wing-borne. **STILL OPEN:** the landing
      is not yet cm-precise/consistent (brake oscillates ±20 m around the target); combining fast
      wing-borne cruise WITH a precise stop needs a proper distance-scheduled decel (sqrt profile to the
      true target) or a small reverse tilt. The nmpc doesn't see the true target through the SMC (only
      the SMC setpoint), which is part of why anticipating the stop is hard.
    - **`vff` fix:** on_ref uses the reference velocity DIRECTLY (don't hold the last non-zero value) —
      holding it kept the vehicle cruising when a mission's FF decayed to 0, doubling the overshoot.
  - **Key fixes (all found live, this is why "hover-only" became "full mission"):** (1) lateral
    **roll-sign** was inverted → fly-away; `roll=+(kp·e_lat+kv·v_lat)/g`. (2) **pitch drift-arrest**
    (symmetric to roll) — with `beta≥0` and level pitch there was NO backward force to stop forward
    drift; nose-up pitch cancels it → rock-solid hover. (3) **★ attitude AUTHORITY** — soft `kR=5` +
    moment caps 3/3/1.5 starved the loop; hover was fine but ANY maneuver (even a climb) crashed.
    **`kR=9` + caps `8/8/4` N·m = the breakthrough.** (4) reads the **RAW reference** (NOT
    `reference_smc`) — it self-limits, the SMC's converging setpoint caused tilt chatter; +
    hysteresis (`engage/disengage_dist`=4/1.5) on tilt-cruise vs pitch-hold. (5) **cruise stall** (CG
    pitch-trim cancels the tilt's forward force) → keep the pitch loop active in cruise tracking
    forward SPEED (`vs_cmd`), with a **sqrt decel** on approach to avoid arrival overshoot. (6)
    yaw-to-target disabled by default (`yaw_face_dist=10000`; it flips 180° near a target) — command
    targets along the heading. (7) `z_bail` safety = level-climb if it drops near ground.
  - **Ops:** mocap-based (no EV/EKF needed). Drive by publishing the RAW reference
    (`python3 /tmp/pubraw.py X Y Z`) + `active_controller`=`nmpc_cruise`. Params are LIVE
    (`ros2 param set /nmpc_cruise_control ...`, no restart). NEVER mid-flight-handover to geometric
    (stale SMC-ref velocity crashes it) — relaunch to reset (each crash inverts the vehicle).
    Limits/TODO: >~8 m/s cruise + return-trips-with-yaw not yet validated on this node.

- **★ FULL CLOSED-LOOP AUDIT of nmpc_cruise (2026-08-09), sim-truth-verified.** Traced every stage
  `controller → PX4 HIL → Isaac (channel-map → propulsion → aero → PhysX) → mocap → controller` with
  live instrumentation (`PEGASUS_PROP_DEBUG=1` → `[PROP]`/`[AERO]` in vtol.py) cross-checked against the
  controller log, over a full hover→climb→cruise(0→5 m/s)→arrive(40 m) mission.
  - **(A) Actuator→HIL→channel-map: 1:1 CORRECT, verified live.** mavlink backend log:
    `controls[0..3]→motor_FL/FR/RL/RR, [4,5]→tilt_L/R, [6,7,8]→aileron/elevator/rudder`
    (`default_quad_tiltrotor_channel_map`, all scale=1/offset=0), exactly matching the node's
    `pub(mot=[FL,FR,RL,RR], servo=[tservo,tservo,ail,elev,rud])`. Rotor order + spin dirs
    (FL+1,FR−1,RL−1,RR+1) match config `quad_tiltrotor_config`.
  - **(B) Propulsion: ACCURATE.** `rotor_dynamics=False` → thrust = throttle·max_thrust (18 N),
    INSTANTANEOUS (controller assumes this). Live hover: `mot=[0.47,0.48,0.46,0.47]` → `[PROP]
    front 8.6/rear 8.4 N` (0.47·18=8.5 ✓), all four ≈ weight/4, total 33.9 N ≈ 34.3 N weight. Thrust
    dir = body-frame `[T·sinθ,0,T·cosθ]` at the rotor position (matches the node's Frx=Tf·sinβ). Yaw
    reaction `−spin_dir·moment_ratio·T` about the tilted axis (matches the node's yaw alloc).
    **Tilt slews at 80°/s** (rate-limit, rotor_dynamics OFF → no tilt_tau lag) — keeps up with the
    node's gradual tilt: live CTL tilt 17→18→20→0 vs `[PROP]` 16→18→20→0 (≤1° lag on ramps, exact at
    steady). So tilt/thrust the sim applies == what the controller commands.
  - **(C) Aero: coeffs MATCH, but a MAJOR AoA-definition mismatch is the real cruise limiter.** The
    node's NMPC aero coeffs (RHO 1.225, S 0.55, CL0 0.2, CLα 5.5, CLmax 1.5, CD0 0.04, k 0.04, stall
    22°) EXACTLY mirror the sim `VTOLAerodynamicsConfig`. BUT the node's NMPC computes AoA from the
    **flight-path** (`atan2(−vz, √(vs²+…))` ≈ 0 in level cruise) while the sim uses the **body-frame**
    AoA — and the vehicle cruises **nose-down ~20°** (the pitch-for-forward + tilt-moment), so the real
    AoA is −5→−22°. Live `[AERO]` at V≈5: `alpha=−19.5°, Fbody_z=−11.6 N` (wing DOWNFORCE, CL hits the
    −1.5 stall floor). The node's model predicts `alpha≈0 → +1.5 N up`. **≈14 N of unmodeled downforce**
    (grows as V²) the feedback loops silently absorb (rotor thrust rises 8.6→10–14 N, altitude still
    held ±0.2 m). This is the root of the ~6 m/s sustained-cruise ceiling seen all session: the NMPC
    (point-mass, no body-pitch state) and the attitude loop (pitches the body nose-down) are DECOUPLED,
    so neither predicts the resulting wing downforce. Minor extras: sim aero hard-gates `<0.25 m/s`
    while the node ramps `vs²/(vs²+4)` (node under-models aero at 2–5 m/s); sim also has cl_q / elevator
    lift / side-force / aero-moments the node's longitudinal model ignores. Fix for >6 m/s (future):
    put the body pitch into the NMPC AoA (couple attitude↔NMPC), or command a small +AoA / use the
    elevator to flip the wing to lift — but that reopens the climb-away (needs the coupled controller).
  - **(D) State feedback: CORRECT frames.** `/uav_0/mocap` = `frame_id:map` WORLD pose (matches the
    node's p), `twist.linear`=WORLD velocity (≈0 at hover, +x during +x cruise — the node uses it as
    world v for e_lat/vs ✓), `twist.angular`=BODY rates (the node uses it as eW ✓), orientation quat
    level at hover (w≈1). No frame bug.
  - **(E) Closed-loop consistency: all check out.** hover thrust=weight (34 N), throttle→thrust exact,
    tilt→genuine forward cruise (tilt 18–21° @ ~5 m/s), altitude held ±0.2 m through the transition,
    precise arrival (40.0 m, no overshoot). The loop is ACCURATE; the only model gap is the aero AoA
    (C), handled by feedback within the validated ~5 m/s envelope.
  - **(F) Roll/lateral path verified + a YAW-HOLD limitation found.** A +5 m lateral (y) move: CTL
    `roll=−14°` → `[PROP] front_L 9.6 > front_R 8.1` (left rotors higher → rolls to translate +y),
    reached (40,5.1,4) with altitude held 4.0 — roll allocation is applied correctly in the sim, mocap
    `twist.linear.y` tracks the world +y motion. **BUT the yaw drifted 0°→61° and STAYED** (only
    damped, never restored): with yaw-to-target off (`yaw_face_dist`=1e4) the node sets `psi_tgt=cur_yaw`
    each loop, so `yaw_cmd` tracks the CURRENT yaw → the yaw error is ~0 by construction → M[2] is
    rate-damping only, no position hold → the roll/reaction coupling of a lateral maneuver yaws the
    body and it isn't corrected. Straight-line forward flight is unaffected (no yaw disturbance), which
    is why the validated mission held yaw≈0. **Recommended fix (small, low-risk): latch `yaw_cmd` to the
    heading captured at engage (a fixed reference) instead of tracking `cur_yaw`, so yaw disturbances
    are corrected** — forward cruise (yaw≈0) is unchanged, lateral/combined moves get a real yaw hold.
  - **Net:** the closed loop is faithful end-to-end (commands → sim forces → state → feedback all
    match) — no plumbing/frame/sign bugs. The two real model gaps are (C) the NMPC AoA (flight-path vs
    body → unmodeled wing downforce, the >6 m/s limiter) and (F) the missing yaw position-hold (lateral
    maneuvers only). Both have concrete fixes above; neither affects the validated hover/climb/
    forward-cruise/precise-arrival/landing envelope.
  - **★ BOTH GAPS FIXED + HIL-VALIDATED (2026-08-09).**
    - **(C) fix — aero AoA coupling.** The NMPC now takes the measured body pitch as a parameter
      (`theta`) and uses `alpha = theta − flight_path` in its aero, instead of assuming a level body.
      `theta` is derived from the sim's EXACT AoA: `v_body = Rᵀ·v_world`, `alpha_meas = atan2(w,u)`
      (identical to `vtol_aerodynamics.update`), `theta = alpha_meas + gamma_meas`. **Sign gotcha:**
      the first attempt used `asin(R[2,0])` (body-x elevation) which is the OPPOSITE sign to the aero
      AoA → the NMPC modeled lift where the sim has downforce; fixed by using the body-frame velocity.
      Live cross-check: controller `aoa=` now tracks `[AERO] alpha` (−8° vs −8° at steady cruise).
      **A/B (same node, `use_body_aoa` 1 vs 0, identical leg): ON → body ~8° nose-up, downforce −5 N,
      altitude held 4.0; OFF → ~17° nose-up, downforce −14 N, altitude dips to 3.8.** So modeling the
      aero keeps the attitude less extreme (3× less downforce, tighter altitude). Param `use_body_aoa`
      (default 1) toggles it live.
    - **(F) fix — yaw position-hold.** When not yawing-to-target the node now holds `yaw_cmd` as a
      FIXED latched reference (re-latched to `cur_yaw` only while standby) instead of tracking the
      current yaw, so a yaw disturbance is actively restored. **Validated:** a lateral move that used
      to drift yaw 0→61° permanently now disturbs it to 54° and it RECOVERS to the held 80°.
    - Both fixed in `nmpc_cruise_node.py`; hover/climb/cruise/precise-arrival unchanged (re-flew a
      40 m cruise: arrived (46.0,82.1,4.0), altitude 4.0, yaw held). Instrumentation: controller log
      now prints `aoa=` (measured body AoA) to compare against `[AERO]`.

- **CHASE CAMERA + Fly+Land wiring + rosbag workflow + the +AoA cruise limit (2026-08-09).**
  - **Chase camera:** `application/vtol/03_vtol_px4_hil.py` render loop now follows the VTOL body
    (reads `vehicle._state.position` each frame, low-passed viewport eye). Env: `PEGASUS_CHASE_CAM=0`
    (static), `PEGASUS_CHASE_OFFSET="dx,dy,dz"` (default `-8,-8,4`).
  - **Fly+Land wired to nmpc_cruise:** GS `_on_mission` now `_set_controller("nmpc_cruise")` and sets the
    traverse yaw toward the target; nmpc_cruise now tracks the **reference yaw** (`self.yaw_ref`, from
    on_ref) instead of a latched heading, so the tiltrotor FACES the target and flies FORWARD (cruise).
    Trajectory file `trajectories/cruise_fly_and_land.csv` (takeoff→cruise→land, loadable in the tab).
  - **★ rosbag diagnosis workflow (use this to verify runs):** `/tmp/recbag.sh <name>` records
    `/uav_0/mocap` + `fmu/in/actuator_{motors,servos}` + the reference(+_smc) + active_controller to
    `/tmp/bags/<name>`; `/tmp/anabag.py <name> [stride]` prints a time series with the sim-EXACT AoA
    (`v_body=Rᵀv`, `alpha=atan2(w,u)`), tilt (servo), elevator, motors, ref + altitude/AoA/tilt summary.
    (anabag imports the msg classes directly — the ROS python here has no `yaml` for get_message.)
  - **★ Fly+Land VALIDATED from a bag (mission1):** takeoff → cruise to (60,0) at 4.0 m with **rotors
    tilted forward ~20°** (tilt servo 0.17–0.24) and **altitude held 3.9–4.1 m** → **landed precisely
    at (60.1, 0, 0.2)**. The reference (`refz=4.0`) is tracked throughout.
  - **★ +5–6° AoA wing-borne cruise — FRAGILE / envelope limit.** Added an elevator-driven AoA-hold
    (`cruise_aoa_deg`, `k_aoa_elev`, `aoa_v_gate`, `k_aoa_alt/vz`, + pitch-gate + hysteresis latch;
    default 0=off). Bag/sign truth: in cruise the body flies **nose-up ~5° (`asin(R[2,0])≈+5`) ↔ AoA≈−5°**
    (small wing downforce) — so a POSITIVE AoA needs the body **nose-down**, and that makes the wing LIFT,
    which the rotors can't offset at ~6 m/s (wing carries weight only ~12 m/s) → **climb-away/fly-away**
    (repeatedly flew to >800 m / 66 m). So the reliable cruise trims at ≈−5° AoA (holds altitude, rotors
    tilt ~20°); a true +5–6° AoA wing-borne cruise needs ~12 m/s (punch the transition corridor) and is
    beyond the current stable envelope. `cruise_aoa_deg` is left as a tunable experimental knob.
  - **Ops caution:** mid-flight NODE RESTARTS disturb the vehicle into unrecoverable stall-glides — test
    controller changes from a CLEAN boot (vehicle on geometric) or via LIVE params, not mid-flight
    restarts. `tmux kill-server` if a teardown leaves a stale session (a relaunch can attach to it).

- **★ AERO COEFFICIENTS SCALED to the real airframe + SMC-routing fix (2026-08-09) — CLIMB-AWAY FIXED.**
  The aero geometry was the AVTOL's (a 1325 kg aircraft): `wing_area=0.55 m²`, `wing_span=2.0 m` on a
  ~0.6 m fuselage — unphysical, ~5× too much lift, so the wing carried weight already at ~8 m/s and
  drove the climb-away. Scaled `vtol_aerodynamics.py` to a sensible small quadplane: **S=0.35 m², b=1.4 m,
  c=0.25 m (AR 5.6), CL_max 1.5→1.2, CL_α 5.5→4.8, CL0 0.2→0.15** (wing loading ~98 N/m², stall ~11.5 m/s,
  wing-borne cruise ~18 m/s). **The nmpc_cruise controller aero constants were updated to MATCH** (must
  agree per the loop audit). **RESULT (bag aero1, SMC-fed cruise): altitude HELD dead-on at 8.0-8.1 m the
  whole traverse (x 5→110 m), AoA small (−4..+6°), no climb-away, no fly-away.** The over-lift is gone.
  - **SMC-routing fix (answers "why does nmpc+smc not track fly_cruise"):** nmpc_cruise had been reading
    the RAW reference (bypassing the SMC), so in "nmpc+smc" mode the SMC's smooth converging setpoint on
    `_smc` never fed it — it got the raw (step-like) trajectory, which both isn't the smc-smoothed
    reference and is what triggered the fly-aways. Re-added the launcher remap `reference:=reference_smc`
    so nmpc is SMC-fed → **smooth reference → stable tracking** (aero1). TRADE-OFF: the SMC's converging
    setpoint caps the speed (~4 m/s in aero1) — to reach wing-borne speed the SMC convergence rate must
    be raised, or feed a smooth fast reference from a KNOWN start (the raw fast tests flew away only
    because the mission started from origin while the vehicle was left at x≈110 by a prior run — a
    test-state mismatch, not an aero/control failure).
- **WING-BORNE FLIGHT ATTEMPT (2026-08-09, rosbag-diagnosed) — transition ENGAGES but altitude climbs
  away; PARTLY FIXED by the aero scaling above (SMC-fed cruise now holds altitude).** Goal: genuine wing-borne cruise (rotors unload, wing carries weight) with altitude
  held + precise landing. Added to nmpc_cruise: a z-REFERENCE RAMP (a big z-step commanded a max-thrust
  climb that corrupted the AoA and tumbled the vehicle on takeoff — `_z_ramp` at vzmax fixed that,
  rosbag-confirmed), altitude-gated + level-gated AoA-hold (`aoa_z_min`, `|vz|<1.5` — only engage in
  steady level cruise, not takeoff/climb). **RESULT (bag wb2, mission fly 300 m @ peak ~15 m/s):** clean
  ramped takeoff to 8 m, then **GENUINE WING-BORNE — rear rotors fully OFF (mot=0.00), front near-idle
  (0.25), tilt engaged, ~7-8 m/s.** BUT the **altitude climbs away 8→35→80 m**: once the rotors unload,
  the body AoA runs to the ~22° STALL (bag shows AoA 38-52°) → CL_max lift the (now-idle) rotors can't
  offset, and the **elevator saturates (-1.00) unable to pull the AoA back down** to the target. Live
  sign-flip of `k_aoa_elev` came too late (already a stalled glide, flew to 700 m/80 m). CONCLUSION: the
  elevator-only AoA-hold is INSUFFICIENT authority for wing-borne (the AoA is dominated by the flight
  dynamics once the rotors idle; in BOTH the rotor-on and rotor-off cases the elevator couldn't move the
  AoA to its target). Stable wing-borne needs the AoA/pitch CO-OPTIMIZED with tilt+thrust (the offline
  NMPC holds altitude to 20 m/s because it does this) OR a partial-wing-borne law that keeps a rotor-
  thrust floor for pitch authority. **Reliable deliverable stays the ROTOR-BORNE cruise** (Fly+Land,
  tilt ~20°, altitude ±0.2 m, precise landing); `cruise_aoa_deg=0` default (wing AoA-hold OFF).
  Ops: the launcher BOOT-node has bad state → RESTART the node once (on geometric) before switching to
  nmpc, else the switch flies away. Bags in `/tmp/bags/{mission1,wb2,...}`.

- **CRUISE SMC + wing-borne status (2026-08-09, rosbag-verified) — STABLE tilt-cruise achieved,
  genuine wing-borne NOT reached (speed-limited).** New `smc_cruise_node.py` (separate outer SMC for
  cruise): same SMC law but leads the set-point FAR ahead (`reset_dist=30`) at high `vmax`, with a NEW
  `amax` accel-limit on the velocity cmd (added to `smc_outer.py`; gentle ramp = no lurch). nmpc reads
  `reference_cruise` (remap) + `cruise_alt_min` (only tilt-cruise ABOVE this alt → takes off VERTICALLY
  first; a far set-point else tilted it fwd at z=2 m and crashed). **RESULT (bag cruise3): a STABLE
  tilt-cruise — yaw held 0°, ALTITUDE held 8.0–8.2 m over 300 m, rotors tilted ~34°, no crash/fly-away,
  arrives.** Gentle SMC `amax=2.5` + stronger yaw (`kR_yaw=4`,`m_yaw_max=6`) fixed the yaw-flip. **BUT
  rosbag proves it is NOT wing-borne:** the body flies **NOSE-DOWN, AoA −13..−27° (wing DOWNFORCE)** —
  the rotors carry it and FIGHT the wing; and the speed caps at **~3 m/s** (SMC set-point convergence +
  nmpc's decel-toward-set-point + the cruise engage/disengage hysteresis toggling as the vehicle catches
  the set-point). At 3 m/s the wing makes ~0 lift and the elevator has no authority, so the AoA-hold
  (even with the corrected `k_aoa_elev` sign) can't pull the AoA positive. **★ PITCH-SIGN CORRECTION
  (user-caught):** `asin(R[2,0])` is nose-DOWN-positive (it read +18° while the aero AoA was −17°) — the
  analyzer now NEGATES it (`/tmp/anabag.py`) so + pitch == nose-up == +AoA == lift; the AoA (`atan2(w,u)`,
  matches the sim) is the reliable quantity. **The barrier to genuine wing-borne is SPEED:** need ~15-18
  m/s (scaled wing) so the wing lifts + rotors unload + a +AoA holds; the SMC/nmpc pipeline caps ~3-4 m/s
  and pushing the accel re-triggers the yaw-flip. Next: rework nmpc to TRACK the SMC velocity-FF as the
  cruise speed (not decel toward the moving set-point), + a speed-scheduled yaw authority.

- **★★★ SO(3) PATH-FOLLOWING NMPC PORT + DECOUPLED CASCADE — wing-borne transition AND fly-to-point+land
  WORK in HIL (2026-08-10).** The user's AVTOL SO(3) path-following transition NMPC
  (`~/Documents/PhDStudies/ControlDesign_VTOL/NMPC_Design/AVTOL_Work-main/Aircraft_Model_SO3_pathfollowing/
  nmpc_vtol_reduced_pos_att_so3.py`) was PORTED onto the FSC quad, then run in a cascade to make it real-time.
  - **`nmpc_so3_transition_node.py`** (controller_id `so3_transition`) — the faithful in-loop port: full
    6-DOF RK4 predictor, cost = pos + SO(3) attitude `0.5·trace(I−RᵣᵀR)` + vel + tilt vs a hover→cruise
    `TransitionRef` (horizon anchored on `s_from_beta`). State x(19)=[pos,R(row-major),beta,vel,omega];
    control u(8)=[T_FL,T_FR,T_RL,T_RR,betacmd,de,da,dr]. Isaac tilt convention (beta=0 up, 90 fwd).
    Offline `--selftest` does the perfect transition (alt held, tilt 0→70°, wing takes over).
    - **HOVER-HOLD fixes (all REQUIRED, HIL-diagnosed):** (1) **`_uprev` must init to the HOVER TRIM, not 0**
      — the `w_du` control-rate penalty is centred on uprev, so uprev=0 front-loaded a ~0-thrust first
      control (16 N vs 34 N weight) → droop/"hover drift". (2) **never apply OR keep a FAILED IPOPT
      iterate** (`return_status` not Solve_Succeeded/Acceptable): it can be ~0 thrust → free-fall, and
      saving it as the warm-start (`zsol`) poisons every later solve → fall back to last-good + reseed.
      (3) **`CG_X` 0.195 → 0.177** (MEASURED: the sim's balanced hover is front=8.6>rear=8.4 →
      Ffront·(0.459−cg)=Frear·(cg+0.112) → cg=0.177 ≈ the rotor centroid). With these the SO(3) hover is
      rock-solid; `IXX,IYY,IZZ=0.20,0.28,0.35` match the VTOLConfig default.
    - **WHY in-loop fails the TRANSITION (not a gains problem — a SPEED problem):** the IPOPT solve is
      **0.24 s at hover, up to 0.94 s during the transition** (measured; N=25). In-loop that runs the
      control at ~1–4 Hz — fine for quasi-static hover but far too slow for the FAST tilt-transition
      attitude → pitch tumble. IPOPT can't hit the 33 ms budget even at N=12/max_iter=3 (~40 ms floor);
      no acados on this machine; CasADi `qrqp` SQP-RTI gets ~26 ms but is borderline.
  - **`nmpc_so3_cascade_node.py`** (controller_id `so3_cascade`) — **the DECOUPLE (the fix).** OUTER = the
    same SO(3) NMPC in a **background `threading.Thread`** (CasADi releases the GIL during the solve —
    verified the fast loop holds ~99 Hz while solves run) producing the optimal tilt feed-forward `beta_ff`;
    it NEVER blocks. INNER = a fast geometric tiltrotor POSITION controller on every mocap update
    (conventions reused verbatim from the HIL-validated `traj_track_node.py`: forward force via front-rotor
    TILT sized to the fwd-force demand then removed from F_des so the body stays LEVEL; decel via SE(3)
    backward LEAN; altitude by collective; 4-rotor allocation `Ainv@[f,Mx,My,Mz]`, front pair boosted 1/cos).
    - **Wing-borne transition (HIL):** hover → tilt ramps 0→20° → accel 0→10 m/s, ALTITUDE HELD →
      `[AERO]` CL=1.2, +25 N lift (~73% of weight), rotors unload (rear 8.3→0.5 N) → wing-borne cruise.
    - **Fly-to-point + LAND mission (HIL, the deliverable):** a `Mission` reference (trapezoidal-speed
      straight line to `target_x,target_y` at `cruise_alt`/`cruise_speed`, then a controlled vertical
      descent — FSC-scaled, in the spirit of their `TransitionRefInterp`, which is unusable directly
      because it's scaled to the 1325 kg AVTOL). **Result (target (60,0), Vc=12, alt 15): hover → accel
      to ~10 m/s (straight, alt held) → cruise → decel → LANDED at (59.9, 0.0), 0.1 m from the target.**
      Speed control + precise point-landing both work (that run: `beta_ff_gain=0`, NMPC tilt-FF OFF —
      the geometric inner loop's own tilt-for-forward carried it; set `beta_ff_gain>0` to blend the NMPC's
      optimal tilt in). Original rough spot: a ~5 m altitude BULGE (15→20→15) + a +32° pitch transient
      during the ~10 m/s cruise as the wing suddenly lifts (the position loop lagged the uncommanded lift).
    - **WING-LIFT FEED-FORWARD added (to kill the bulge) — code done, HIL re-test PENDING (interrupted).**
      `aero_world(R,v)` computes the modeled wing force and the inner loop uses `F_des -= wing_ff_gain*aero`
      so the altitude loop anticipates the lift. **Sign gotcha (fixed):** the ported `aero_body` in
      `nmpc_so3_transition_node.py` has the LIFT sign FLIPPED vs the sim (fine offline, wrong for a real-sim
      FF); `aero_world` instead mirrors the SIM's decomposition exactly (`lift_dir=[-sin a,0,cos a]`, drag
      along −velocity) — verified it reproduces the sim `[AERO]`: at α=+18.3/V=10.3 → Fbody=(−10.7,0,+25.2)
      vs the sim's (−10.6,0,+25.1). `wing_ff_gain` default 0.8. NOT yet HIL-confirmed to remove the bulge.
  - **GS-WIRED (code done, end-to-end HIL re-test PENDING):** (1) GS button **"E: SO(3) Cascade"** added
    to the controller-switch row (`trajectory_player.py`, publishes `so3_cascade`); (2) `_on_mission` now
    runs on the CURRENTLY-SELECTED controller (`_active_ctrl()`) instead of hard-coded `nmpc_cruise`, so the
    "🛬 Fly to point + LAND" button (Target X/Y, Cruise alt/speed) drives whichever is selected; (3) the
    cascade node now SUBSCRIBES to the RAW GS reference `…/position_controller/reference`
    (`PositionControllerReference`) and a fresh stream OVERRIDES the internal `Mission` — so the GS
    Trajectory tab / Fly-to-point drive it like any controller; (4) launcher window **`6d-cascade`** added
    to `start_vtol_smc_hil.sh` (STANDBY, reads the RAW reference NOT `_smc`; env `CASC_VC/CASC_BFF/CASC_WFF`).
    GS is launched from SOURCE (`/usr/bin/python3 src/single_drone_ground_control.py`) → no colcon rebuild.
    Usage: take off (A), click **E: SO(3) Cascade**, set Target X/Y + Cruise alt/speed, click **Fly to
    point + LAND**. See [[so3-cascade-transition]].
  - **Run (standalone, no GS):** `/tmp/run_cascade.sh` env `TX/TY/ZC/VC/ACC/VD/BFF/WFF` (pass DOUBLES —
    `TX=60.0`; an int override crashes rclpy param typing). Take off on `geometric` (`pubraw.py 0 0 15 0`),
    then publish `active_controller=so3_cascade`. Offline solver probes + selftest in the transition node.

## Ground-station Trajectory tab (`~/ros2_ws/src/ros2_ground_station_gui`)
Extra "Trajectory" tab (added by `src/single_drone_ground_control.py`, logic in
`src/ROS_Node/trajectory_player.py`, numpy-only — GS python has no working scipy).
Loads `.csv/.npz` reference files (`t,x,y,z,yaw_deg[,vx,vy,vz]`, ENU), streams them as
`PositionControllerReference` with **Play/Pause/Stop, spatial+time scale, auto-fit to
geofence, "start at current position", loop**, a **big HOVER (hold) safety button**
(hold current pos, always available, geofence-guarded), and the **controller switch**.
Exporter `tools/export_reference_trajectory.py` turns the AVTOL `transition_reference.py`
generators into trajectory files (run with the isaac_env venv — needs scipy; the
generators model a FIXED-WING transition so even "quad" presets are ~166 m/30 m/s → the
tab's auto-fit shrinks the shape to the geofence). Samples in `trajectories/`.

## Environment gotchas — controllers / GS / HIL (this session)
- **casadi/scipy live in the isaac_env venv** (`~/envs/isaac_env/bin/python3` = the
  ROS-node python; pip-bootstrapped via ensurepip). NMPC/exporter must use it. **System
  `/usr/bin/python3` (the GS python) has BROKEN scipy** (numpy 2.x vs apt scipy) → GS-side
  code must be numpy-only. `pyqtgraph` installed --user for the GS (plots + tab preview).
- **A leftover `autopilot_sv_baseline_node` from a prior session** publishes the SAME PX4
  setpoint topics → two controllers fight → tumble. The plain px4/isaac/agent process
  check misses it. `pkill -9 -f autopilot_sv_baseline` before running any NMPC/direct node
  (the launchers do this in the BODY, not the node pane — the pane's own cmdline contains
  the pattern and would self-kill).
- **tmux: use separate WINDOWS, not tiled split-panes** — 5 tiled panes overflow the min
  pane height and the last split silently drops (the controller pane never launches).
- **PX4 4101 airframe**: set `param set-default SYS_HAS_NUM_ASPD 0` (both ROMFS + the
  `build/px4_sitl_default/etc` copy) or arming fails "Airspeed selector module down". VTOL
  spawns at z=0.3 (on ground). Isaac ROS2-bridge env for VTOL_ROS2_STATE=1: export
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` + `LD_LIBRARY_PATH+=.../exts/isaacsim.ros2.bridge/humble/lib`.
- **Self-kill**: never put a `pkill -f <pat>` (or a heredoc containing `<pat>`) in a Bash
  command whose own command line matches `<pat>` — it kills the invoking shell. Put kills
  in a Write-created /tmp script, run as `bash /tmp/x.sh`.
- **Teardown from an automated Bash sandbox silently no-ops** — `pkill -f`, `kill -9 <pid>`
  and `tmux kill-server` return exit 1 and the px4/isaac/DDS/tmux stack SURVIVES (this is why a
  crashed/inverted vehicle "carried into the next run"). Fix: run teardown with the sandbox
  DISABLED, and even then `pkill -f`/`tmux kill-server` are unreliable → gather pids with
  `pgrep -f` and `kill -9` each EXPLICITLY. Use `bash /tmp/kill_hil.sh` (does exactly this),
  sandbox-off, before every relaunch. Launcher: `start_vtol_smc_hil.sh --here pieper_machine`
  (needs the config NAME arg; run standalone, not chained after the teardown in one command).

## Environment gotchas (this machine)
- **GUI terminal launch**: `~/.bashrc` sources ROS + a venv (`isaac_env`) into every
  shell; that polluted env crashes gnome-terminal ("undefined symbol:
  `__libc_pthread_init`") when a script spawns it. `terminal_utils.sh
  open_new_terminal` launches the terminal with a **pristine env** (`env -i` + only
  DISPLAY/DBUS/XDG/HOME/PATH), like the desktop shell; `bash -lc` re-sources the
  profile for the work command.
- **rclpy inside Isaac**: run scenes from a **clean** shell (`PYTHONPATH`/conda unset,
  as the scripts do) so Isaac's bundled rclpy is used, not system ROS py3.10 (ABI clash).
  Isaac's ROS2 bridge only provides `rclpy` if its env is set (`ROS_DISTRO`,
  `RMW_IMPLEMENTATION`, `AMENT_PREFIX_PATH`, bridge `LD_LIBRARY_PATH`); otherwise
  `import rclpy` fails. Therefore **do not hard-import rclpy at module top in code an
  Isaac scene imports** — guard it (`fsc_aerial_manipulation/utils/__init__.py` now
  wraps `ROS2RigidBodyBackend` in try/except; a bare `import rclpy` there previously
  crashed scene 03 at import via `add_dome_lighting`).
- **Mocap (`VTOL_ROS2_STATE=1`) needs Isaac's ROS2 bridge working** (see above) —
  the plain HIL path (`VTOL_ROS2_STATE=0`) does not use rclpy and is unaffected.
- Status: scene 01 runs; scene 03 imports fixed. Follow the runbook's staged 01→02→03.

## ★★ FIXED-WING CRUISE via the SCALED AVTOL REFERENCES (2026-08-10) — D validated, full analysis in docs/fixed_wing_control_analysis.md
The AVTOL reference generators (`transition_reference[_consistent].py`, profile builders) are
WIRED IN + SCALED to this airframe: `ros2/px4_vtol_direct_actuator/.../fsc_transition_ref.py`
(m=3.5, S=0.35, T_max=60 → force-balanced trim schedule V→(tilt, α, θ, Ttot, de); self-tested:
hover 34.9 N → cruise@15: tilt 90°, α 6.9°, T=3.4 N, wing ~100%; env `AVTOL_REF_DIR`,
`FSC_TILT_CAP_DEG` default 65).
- **D (`nmpc_cruise`, GS button D) = the VALIDATED FIXED-WING fly-to-point** (bags fwD8/9/10:
  3/3 250 m arrivals; clean hover→fixed-wing transition — tilt 0→65°+, rear pair unloads →0,
  altitude dead-flat through the hand-off; residual ±10 m cruise phugoid = open issue). Profile
  drives the horizon θ/β refs + a THRUST-hand-off term (`w_T_ref` — without it the solver sits in
  a rotor-borne KKT trap) + a PITCH-BALANCE term `w_pm·(Tf·cosβ·xf − Tr·xr)²` (point-mass model is
  blind to the split's pitch moment → rear-loaded → nose-over, bag fwD2; cg_x=0.177 MEASURED) +
  schedule-bounded β (`bref+6°`) + applied-β slew 35°/s (β bang-banged 80↔0 at the wing-borne
  boundary, fwD4) + AoA gates at 2 m/s (0.2 let hover-drift noise fake θ=9° → doomed runs, fwD7).
  **KEY BUG (explains the old "rotors stay vertical in cruise" mystery): the NMPC's du penalty was
  anchored to a HARDCODED hover Uprev → the first-step tilt was dragged to 0 every cycle =
  receding-horizon procrastination; uprev now = last applied control via P.**
- **E (`so3_cascade`, GS button E) = validated geometric fly-to-point** (fwE8: landed 0.12 m from
  the 60 m target) in its MC mode, improved by Mission LEAD-CLAMP (ref ≤10 m ahead), z-RAMP, and a
  CLOCK-FREEZE (trapezoid pauses while lagging). Its profile fixed-wing branch is EXPERIMENTAL
  (`CASC_FW=1`): traverse-engage transient unresolved (bags fwE2–7: MC↔FW tilt flapping → dive;
  backward slide from the one-sided alt-corr; FW self-trap at v≤0 — each fixed, engage still not robust).
- **GHOST PUBLISHER (fixed):** standby `aero_transition_node` (window 6b) co-published actuators at
  200 Hz against the active controller (constant 45°-tilt + zero-elevator stream, bags fwD1/fwE2) —
  6b now DISABLED by default (`RUN_TRANSITION_C=1` to re-enable); `nmpc_so3_cascade` added to
  `/tmp/kill_hil.sh` (it survived every teardown before).
- **GS STOP+SAFE-HOVER**: now actually forces "geometric" (it silently kept the active controller
  before) with a STAGED handover (active controller decelerates on the hover ref; switch <4 m/s /
  12 s). **LIMIT: an instant STOP from full fixed-wing cruise (>10 m/s tilted) is NOT survivable**
  (stop1–3: tilt-up at speed → wing+rotors=2× weight → zoom-climb → stall-drop). Abort from cruise =
  let the mission decelerate, or a far-ahead hover point. Multicopter-envelope STOP unchanged/safe.
- **Why fixed-wing kept failing (the user's question)** — six stacked causes, five fixed
  (procrastinating du anchor; no force-consistent reference; pitch-blind point-mass split; bistable
  boundary replanning; garbage low-speed AoA + the ghost publisher), one characterised+open (the
  LAST 15% of the lift hand-off: one-sided rotor pitch authority + phugoid with no fast
  lift-shedding actuator → needs a 6-state longitudinal NMPC with pitch dynamics + elevator in the
  loop). Full write-up: `docs/fixed_wing_control_analysis.md`.
- GS Reference dropdown now has FSC-SCALED AVTOL trajectories (`fsc_fixed_wing_transition.csv`,
  `fsc_transition_climb_turn.csv`); exporter `--consistent` velocity-index bug fixed (30-state
  layout carries v at [17:20], not [16:19]).
- Test drivers: `/tmp/run_{d,e,stop}_test.sh` (verified param-set + bag + monitor); launcher knobs
  `FW_VCRUISE=15 SMC_VMAX=20 FSC_TILT_CAP_DEG=65 CASC_FW=false RUN_TRANSITION_C=0`.

## ★★★ THE USER'S SO(3) NMPC FLYING 90° PURE-PULLER IN-LOOP (2026-08-10 late) — see docs/fixed_wing_control_analysis.md (UPDATE section)
Directive: fly the AVTOL NMPC derivations directly. `nmpc_vtol_reduced_pos_att_so3.py` (via its
port `nmpc_so3_transition_node.py`) now runs IN-LOOP at 25 Hz: **JIT-compiled IPOPT (N12/DT0.1/RK1)
= 20 ms mean / 36 p95 per solve** (was 240–940 ms), cached via jit_serialize=embed
(`~/.cache/fsc_so3_nmpc_*`; cache key MUST include weights+max_iter — serialized solvers bake
options; a stale cache silently no-ops changes). **MAX_ITER=60 REQUIRED** (50 = deterministic
corridor failure cascade). fatrop present but slow here; qrqp-RTI diverges.
- **Fixes that made it fly:** (1) ★ LIFT-SIGN in the ported aero_body was FLIPPED vs the sim
  (port trimmed at α=−9 offline = downforce in HIL → dive; bag so3a) — now mirrors
  vtol_aerodynamics verbatim; (2) reference schedules from the FSC-scaled TransitionRefInterp
  incl. TRIM-PITCH attitude ref (level ref fights the wing at high tilt); (3) w_om=3 planned rate
  damping (roll blew up where front differential = yaw authority) + w_att 120 + w_du 6; (4)
  TIME-RATCHET path anchor with 0.15 leash (tilt-only anchoring parks in a rotor-borne local
  optimum at ~30°); (5) rear floor relaxes with s; tilt integrator freezes on failed solves.
- **OFFLINE: full 90° transition + INDEFINITE steady pure-puller cruise** (β=90.0, V=13.8, z flat,
  AoA +7.2 = true trim, rotors 6 N tot). Latency: needs true ≲130 ms, comp ≥ true (under-comp −30 ms
  fatal; over-comp = safe partial-tilt stall).
- **HIL (bag so3c, SO3_TF=20): transition completed + ~22 s / 200 m STABLE 90° cruise** (tilt servo
  1.00, rear ~1 N, front ~4 N ≈ drag, alt flat 8.1–8.4, V 11.9, straight) — ended by a slow
  roll/yaw departure (~6 s) = the lightly-damped lateral mode at full unload. tf=30 lingers in the
  corridor and crashes (so3d) — FASTER through the corridor is better. Corridor in HIL sags
  nose-down (elevator saturates) with run-to-run variance — latency calibration is the lever.
- Ops: window `6e-so3`, knobs SO3_VC/BC/TF/LAT (launcher FORCES DOUBLES — rclpy int-for-double
  rejection killed one run). Activation: verify the ENGAGED log (one-shot pub misses discovery).
  EVERY sim launch records a bag; read it after each test (user directive). Bags so3a–so3d.
- NEXT: per-axis w_om (roll), da/dr gains 1.0, latency calibration from commanded-vs-realized tilt
  slew in a bag; then long-cruise soak.

## ★★★ 90° WING-BORNE CRUISE SOLVED — 6-STATE PITCH-MOMENT NMPC (2026-08-12)
The rotor→wing handoff corridor that defeated EVERY controller all session (SO(3) mission-tracking,
point-mass nmpc_cruise, cascade) is CRACKED by a new node that co-optimizes the PITCH MOMENT.

- **ROOT CAUSE of the whole-session failure (found by auditing the REFERENCE):** plugging each
  reference (xref,uref) into `build_ode()` showed `omega_dot(pitch) = -7..-8 rad/s²` through the whole
  corridor — the reference was NOT a trim. The old front/rear thrust split was a heuristic `rear_frac`
  fade that never PITCH-BALANCED the FSC 4-rotor geometry. **Fix: solve the split so `My_thrust +
  My_aero = 0`:** `Tf = (xr*Ttot - My_aero)/(2*(zf*sin b - xf*cos b + xr))` (xf=0.282, zf=0.02,
  xr=-0.289 rel CG_X=0.177). Drove the pitch residual -7 → ~0 everywhere. Wired into
  `nmpc_so3_transition_node.TransitionRef.get()` AND `fsc_mission_ref.py`.

- **`ros2/px4_vtol_direct_actuator/.../nmpc6_cruise_node.py` — the SOLUTION.** A 6-state LONGITUDINAL
  NMPC: state `X=[s,z,vs,vz,theta,q]`, control `U=[Tf,Tr,beta,de]`, `qdot=My/Iyy` = thrust-split
  moment + aero (`cm0+cma*alpha+cmq*qhat+cmde*de`), ELEVATOR IN THE LOOP. Every other controller plans
  body pitch as if directly commandable; this one predicts+counters the pitch moment. `build_ode6()`
  is VALIDATED byte-identical to the sim plant (`--validate`, err 0.0000; aero/geom imported from
  nmpc_so3_transition_node so they can't drift). **Offline vs the TRUE plant WITH 100 ms latency:
  hover → transition → sustained tilt=90°, V=16, AoA=+6.4°, altitude DEAD-FLAT, rear rotors OFF, front
  3.8 N (drag), wing carries the weight.** The 5 keys (all necessary): (1) 6-state pitch dynamics;
  (2) STATE bounds on the stiff pitch (theta∈[-20,30]°, q∈[-3,3] — unbounded, IPOPT explores qdot~-200
  → ill-conditioned → solve fails); (3) reference-feedforward FALLBACK on solve failure (a frozen
  control diverges); (4) SOFT tilt guide (`w_beta=12` — the NMPC self-limits the tilt through the
  corridor, then commits ~V=10; forcing the schedule tilt drove the nose-down trap); (5) delay
  compensation. N=14/DT=0.06/RK4×2, JIT-compiled (`jit_serialize:embed`) → **9 ms solve**, cached to
  `~/.cache/nmpc6_cruise_*.casadi` (first build ~2 min, then Function.load 0.1 s). Reference =
  `Schedule.at(V)` (FSCTransitionProfile trim + pitch-balanced split, SPEED-gated pitch=`theta_cmd` so
  the engage is a LEVEL hover). Lateral (roll/yaw) = a TILT-AWARE stabilizer (differential-thrust roll
  scaled by cos(beta) → fades to 0 by 90° where the front rotors are horizontal + the rear is
  unloaded; AERO surfaces take over at cruise).
  - **Wiring:** launcher window `6f-nmpc6` in `start_vtol_smc_hil.sh` (STANDBY; env `NMPC6_VC/CAP/TTR/
    LAT/ALT`); GS button **"✈ 6-state Wing-borne (90°)"** in `trajectory_player.py` (publishes
    `nmpc6_cruise`). Take off on geometric, then engage. Runbook `docs/wing_borne_90_runbook.md`.
  - **HIL hardening (each root-caused offline; NONE reproduce in the non-real-time offline test):**
    (a) solve 53 ms > 40 ms 25 Hz budget → loop falls behind → latency balloons → tumble: fixed by
    JIT (9 ms). (b) +8° hover-pitch engage → nose-up with no airspeed → tumble: fixed by speed-gating
    the pitch. (c) `self.beta` was UNDEFINED (the 6-state model has tilt as a DIRECT control, no
    tilt-state) → loop threw AttributeError before pub() → 'does nothing / just lands': use the local
    `beta`. (d) ★ RUDDER SIGN backwards (`CNDR=-0.12` NEGATIVE → +dr gives -yaw): `+k_rud*Mz` AMPLIFIED
    the yaw → ran to -100°, huge arc (HIL bag myrun); fixed to `-k_rud*Mz`. (e) engaged at 1.5 m → sank
    to 0.2 m repeatedly: param `cruise_alt` (default 15) climbs to a safe altitude BEFORE transitioning.
  - **MOCAP pitch VERIFIED consistent** (user asked): mocap = FLU-body rel ENU-world, angular_velocity
    = FLU BODY frame (State def). `theta=-asin(R[2,0])` EXACTLY equals the sim AoA (`v_body=Rᵀv`), and
    driving to the reference theta=+8 makes the plant CLIMB (+2.07 m/s²) — pitch measurement + reference
    are correct. But the pitch is RAZOR-sensitive at 90° (theta=0 → -8 m/s² dive), so any lateral
    divergence that drops it → nose-dive. Empirical check: `/tmp/mocap_pitch_check.py`.
  - **OPEN:** combined roll+yaw disturbance still crashes offline (`--roll0 10 --yaw0 20`) = residual
    lateral robustness; pure roll or pure yaw recover. NOT yet cleanly HIL-flown (3 attempts, each fixed
    a distinct bug).

- **FULL MISSION (≤14 m/s tilt-assisted) on the SO(3) node — `mission_mode` (Track A).** `fsc_mission_ref.py`
  (FSCMissionProfile) builds a TIME-parameterised climb→transition→cruise→back-transition→settle→land
  trajectory through the AVTOL `_solve_coupled_thrust_and_beta` trim (force-EXACT tilt(V), wing-load
  gate, pitch-balanced split). `nmpc_so3_transition_node._mission_step` (param `mission_mode:=1`, a
  SELF-CONTAINED branch — the HIL-validated legacy path is untouched) flies it: state-gated clock
  (speed-gated trans_out) + `solve_mission(anchor_xy=)` + strong attitude weights (`mission_w_att=220`)
  + elevator pitch-hold. Offline: landed 0.4 m from a 200 m target. Launcher env `SO3_MISSION MISSION_TX/
  TY/VC/CAP/ZC`; runbook `docs/mission_hil_runbook.md`. (>14 m/s reintroduces the corridor → that's what
  the nmpc6 node is for.)

- **Bag tooling for nmpc6:** `scripts/recbag_nmpc6.sh <name>` (mocap + actuator_motors/servos +
  active_controller + px4 out estimate); analyze with `/tmp/ana6.py <name>` (NOT anabag.py — it chokes
  on the px4 out-topics). Healthy cruise = pitch≈AoA≈+6°, roll/yaw≈0, tilt→90°, rear motors→0. Full
  topic table + failure signatures: `docs/nmpc6_rosbag.md`.

## ★★★ FULL-STATE (19) NMPC + OUTER POSITIONAL SMC (2026-08-12 late) — supersedes nmpc6 for robustness
`ros2/px4_vtol_direct_actuator/.../nmpc_full_smc_node.py` (controller_id `nmpc_full`, launcher window
`6g-full`, GS button "★ Full-state NMPC + SMC (90°)").

**WHY:** nmpc6 cracked the corridor by co-optimizing the PITCH MOMENT, but its lateral axes were a
hand-rolled PD OUTSIDE the solver — fragile exactly at 90° (front rotors horizontal → differential
thrust makes YAW not roll; rear unloaded → no roll authority), and a **combined roll+yaw disturbance
still crashed it**. This node keeps the pitch-moment insight and puts the FULL attitude + tilt + body
rates INSIDE one NMPC over the sim-exact 6-DOF plant, so the solver allocates 4 rotors + 3 surfaces
against ALL coupled moments at once.

- **Model:** `so3.build_ode()` (the 19-state sim-exact plant) — x=[pos, R(row-major), beta, vel,
  omega], u=[T_FL,T_FR,T_RL,T_RR, betacmd, de, da, dr]. No new dynamics written = cannot drift.
- **Cost:** pos + vel + **SO(3) attitude `0.5·tr(I − Rᵣᵀ R)`** against `Rᵣ = Rz(heading)·Ry(θ_trim)`
  (roll=0, yaw=heading → **lateral is TRACKED, not patched**) + soft tilt + rate damping (all 3 axes)
  + control-ref/du. N=12, DT=0.08, RK1, JIT (`jit_serialize:embed`) cached to `~/.cache/nmpc_full_*`.
- **Outer SMC:** subscribes `reference_smc` (launcher remap) and CLAMPS the along-path reference at the
  SMC destination, so the NMPC flies the wing-borne schedule TOWARD the commanded point.
- **Reference:** the same validated `nmpc6.Schedule.at(V)` (FSCTransitionProfile trim + pitch-balanced
  split + SPEED-GATED pitch → level engage) wrapped into a full 19-state xref/uref.
- **The 5 nmpc6 keys retained:** full moment dynamics; STATE bounds on the stiff rates (now all 3 axes,
  |ω|≤3, |vz|≤12); reference-FF fallback on solve failure; SOFT tilt guide (`w_beta=12`); delay comp.
- **OFFLINE vs the TRUE plant @100 ms latency:** sustained **tilt=90°, V=16, AoA=+6.4°, altitude
  DEAD-FLAT 20.0 m, rotors 3.8 N (rear≈0), roll=0, yaw=0** — AND it **RECOVERS from roll10+yaw20,
  roll15+yaw−25, yaw30**, every one of which the 6-state node could not survive.
- **Timing:** solve **22 ms mean / 31 ms max** → default `rate_hz=20` (50 ms budget). 25 Hz leaves only
  ~9 ms and an overrunning solve is what killed an earlier HIL run.
- **Ops:** `FULL_VC/CAP/TTR/ALT/RATE/LAT` env; take off on geometric, then activate `nmpc_full`.
  `cruise_alt` (default 15) climbs to a safe altitude before transitioning. Bag with
  `scripts/recbag_nmpc6.sh` and read with **`scripts/ana_bag6.py`**. NOT yet HIL-flown.

**★ /tmp IS WIPED ON REBOOT** — it took out every rosbag AND all the /tmp helper scripts
(anabag.py, ana6.py, recbag.sh, kill_hil.sh, pubraw.py, the offline probes). Keep reusable tooling in
the REPO: `scripts/recbag_nmpc6.sh` + `scripts/ana_bag6.py` (persistent). Check `ls -dt /tmp/bags/*/`
before claiming to analyze a run.

### FIRST HIL FLIGHT of the full-state NMPC (2026-08-13, bag `full1`) — FAILED, 4 causes fixed
Engage/hover was ROCK SOLID (z held, roll=0, yaw=0, ok=1) — the SO(3) attitude cost does hold the
lateral at hover. The TRANSITION failed: repeated attempts, each pitching **nose-down −20..−26°**
(AoA −21..−25 = wing DOWNFORCE), speed never past ~3.6 m/s, then tumble to inverted (roll ±178°),
fall, auto-recover via `cruise_alt`, retry — until it crashed at x=71.6. Fixes applied (offline
regression re-passed after: 90°/AoA 6.4/roll,yaw 0, and roll10+yaw20 still recovers):
1. **Loop overrun** — solve **43 ms mean / 69 ms max** vs the 50 ms budget @20 Hz (offline it is
   22/31 ms; HIL CPU load from Isaac+PX4+GS ~doubles it). → `rate_hz` 20 → **15** (67 ms budget).
2. **Latency UNDER-compensation** — `latency_s` 0.10 was below the true loop latency (43 ms solve +
   DDS + PX4 + Isaac). Prior finding: under-comp is FATAL, over-comp is safe. → **0.18**.
3. **da/dr were UNPENALIZED in the cost** → IPOPT parked them anywhere; the bag shows the **rudder
   pinned at +1.00** all run = a standing yaw disturbance at speed. → added `w_surf=3.0`.
4. **Tilt-state estimate vs the sim's RATE-LIMITED servo** — commanded tilt thrashed 32→60→45→90°;
   the sim servo rate-limits (~80°/s) while our first-order estimate LEADS it → the NMPC plans with
   the wrong thrust direction. → applied-tilt **slew limit** (`tilt_slew_deg_s`=60) + the estimate
   now integrates the SLEW-LIMITED command with the same 80°/s cap.
**Ops gotchas learned:** (a) `scripts/pubref.py` (raw-reference streamer) SURVIVES teardown like
pubraw — now in `kill_hil.sh`'s pattern list; a stale one had the geometric controller chasing a
400 m target at 20 m/s on the next launch. (b) Don't run the OFFLINE test while tearing down —
`kill_hil.sh` matches `nmpc_full_smc_node` and kills it. (c) Helper scripts now persist in
`scripts/` (`kill_hil.sh`, `switch_ref.sh`, `pubref.py`, `recbag_nmpc6.sh`, `ana_bag6.py`).
**STATUS: the 4 fixes are in + offline-validated but NOT yet re-flown in HIL.** Next run: launch,
climb on geometric, bag, engage `nmpc_full`, then release the far target; read the bag and check
whether the nose-down persists (if it does, the tilt estimate is still the prime suspect).

### 2nd/3rd HIL runs of nmpc_full (bags full1/full2) — TWO ROOT CAUSES FOUND
**(A) MY SMC-CLAMP BUG (bag full1, "never accelerates past 3.6 m/s").** The clamp did
`xr[13:16]=0` — ZEROING the velocity reference for every horizon stage past the SMC set-point. The
SMC set-point is a CONVERGING one leading by only `reset_dist` (8-30 m) that re-seeds, so the NMPC
got a CONTRADICTORY reference: the schedule said "tilt to 16 m/s cruise" while pos/vel said "stop
here, v=0" → could never accelerate, attitude fought the tilt → nose-down → tumble. **FIX:** brake
only when genuinely approaching the destination, via a sqrt stopping profile
(`v_cap=sqrt(2·a_br·rem)`, param `brake_accel`); outside braking distance the schedule runs free.
Confirmed in flight: after the fix it accelerated 0.5→4.6 m/s (vs the old 3.6 cap).

**(B) OVER-TILT AT ZERO AIRSPEED → NO PITCH AUTHORITY (bag full2, the actual crash).** Timeline
from engage: stable hover → tilt ramps 0.04→0.24→0.34 → **pitch −2→−5→−8→−17→−80° in ~6 s** with the
**elevator saturated at +1.00** fighting it and the rotors front-heavy → inverted (roll −171°) →
ground. Mechanism: the solver tilted to **82-90° while V was still ~1 m/s**; at high tilt the front
rotors are horizontal (almost no pitch moment) AND the elevator has no dynamic pressure → **zero
pitch authority** → the nose simply falls. `w_beta=12` is too soft and the 0.8 s horizon can't see
the authority loss. **FIX: the AIRSPEED TILT CLAMP** — applied tilt ≤ trim_tilt(V_now) + 12°
(`tilt_margin_deg`), self-relaxing to 90° as speed builds. This is the SAME clamp the SO(3) node
already needed (bag so3i) and which I failed to carry over into this node.

**(C) REAL-TIME: the 19-state NMPC is too big for this machine under HIL load.** N=12/iter60 =
22 ms on an IDLE machine but **48-96 ms** with Isaac+PX4+GS competing (budget 67 ms @15 Hz), with
`ok=0` solve failures. **FIX: N 12→8, DT .08→.10 (same ~0.8 s horizon), MAX_ITER 60→25.**
Real-time margin matters more than horizon length here — measure solve time IN HIL, never offline.

### nmpc_full HIL campaign — the plateau is a STALL DEPARTURE, not a lift wall (2026-09-07, bag full_tr12)
Level-flight speed across the campaign: 5.6 -> 8.45 -> 8.71 -> 10.5 -> **12.42 m/s** (tilt 71.2 deg,
wing 54.4%). The 0.8 -> **1.6 s horizon** (`N=8, DT=0.20, NRK4=1` -- same symbolic size, ~5 min JIT
build; DT=0.25/NRK4=2 does NOT build inside 15 min) broke the 10.5 m/s plateau, as predicted.
**MEASURE LEVEL FLIGHT, NEVER PEAK SPEED.** `fly_transition.py` now reports only samples with
|vz|<1.5 and flags >5 m of altitude loss -- flight 11's "TRUE FIXED-WING 15.83 m/s" was a DIVE, and
flight 12's 14.80 m/s peak likewise. The honest numbers are the LEVEL ones.
**Root cause of the departure (quantitative, bag-confirmed):** the effective CL peaks at **AoA
exactly 10.0 deg (CL 0.790)** and is down to **0.334 at 25 deg** (`stall_alpha_rad`=13 deg, the
sigmoid blend bites well before it). Flight 12 ran to AoA +25.3 deg -> wing share collapsed
52.4% -> 23.8% (42% predicted / 45% measured -- the model and the bag AGREE) -> sank -> and the sink
RAISED AoA further (descending at speed tilts the flight path down) -> unrecoverable.
**It was NOT short of force:** at 12.4 m/s the wing can carry 77% of weight and the rotors have
23.4 N of vertical capacity at 71 deg tilt vs the 7.8 N needed. Level flight there was feasible; it
was lost to attitude, not capacity. Stall speed is **14.13 m/s** (wing carries 100% at the CL peak),
so 90 deg tilt needs ~14 m/s and the whole problem reduces to holding AoA on the peak while speed builds.
**Fix under test:** `w_aoa` 0.5 -> **2.0** (`aoa_lim` stays 10 deg -- it sits ON the CL peak by
design). The corridor requires riding a knife edge with no margin either side, so the guard has to be
a wall, not a suggestion.

**★ DO NOT RETRY (bag full_tr13): `w_aoa` 0.5 -> 2.0.** It is a REGRESSION and the reason is
numerical, not aerodynamic. `(alpha/aoa_lim)^6` reaches 244 at alpha=25 deg, so at w=2.0 the cost
term is ~488 with gradients to match; IPOPT cannot converge that inside `MAX_ITER=25`.
**Every solve returned ok=0 from V=6 m/s / tilt 46 deg onward** and the solve time went 21-34 ->
**50-80 ms against the 67 ms budget** at 15 Hz. The node fell back to `safe_hover()` (visible as
`mot=[0.47,0.47,0.47,0.47]`, rotors pinned at 33.8 N), the vehicle coasted, pitched to +38 deg,
stalled, slid backwards (AoA -> 179 deg) and tumbled to the ground.
**READ THE SUMMARY'S PROVENANCE, NOT JUST ITS NUMBERS:** that run reported "50 level samples, wing
80.3%" -- both were recorded AFTER the controller had dropped out, during an uncontrolled glide at
tilt=0. A level-flight filter does not distinguish flying from gliding. Check WHO WAS FLYING (ok=1
and a non-hover motor pattern) before crediting any number.
**CONSEQUENCE: the AoA hold is limited by the SOLVER BUDGET, not by tuning.** 0.5 is the largest
weight that still solves. Making the guard stiffer requires making the solve cheaper first (lower
exponent for better conditioning, fewer states, or a faster QP) -- not a larger weight.

### ★★ EXPONENT 4 FIXES THE SOLVER — 79.5 deg TILT, ROTORS TO 9.5 N (2026-09-07, bag full_tr15)
`_aoa_excess` is now `(alpha/aoa_lim)^4` with **`w_aoa`=1.5** (was ^6 at 0.5). The ^6 form blew up
in the FAR TAIL (244 at alpha=25 deg), so raising the weight to make it hold at the limit made the
gradient unsolvable. ^4 reaches only 39 there: the guard is **3x STRONGER at the limit** (1.5 vs
0.5, where it must hold the CL peak) with a **12x SMALLER gradient at 25 deg** than the ^6/2.0 form
that broke -- and ~3x smaller than even the ^6/0.5 that already solved. **`ok=1` held right through
the corridor** (V=6.6 tilt 48, V=8.3 tilt 60, V=10.3 tilt 73, all 26-37 ms) where full_tr13 collapsed
at V=6 / tilt 46.
**BEST CONTROLLED STATE OF THE CAMPAIGN** (rotors unloaded, altitude held, ~4 s each):
```
t  76.0- 79.9 (3.9s)  V  9.9 avg/11.8 max  tilt 79.1  wing 64.1%  rotors 10.0N  alt 18.9-22.7
t 121.1-125.3 (4.2s)  V 10.9 avg/13.5 max  tilt 79.5  wing 91.2%  rotors  9.5N  alt 19.0-21.1
```
vs flight 12's 71 deg / 54% / 16.8 N: **+8 deg of tilt and the rotor load nearly halved** at
comparable speed. Still NOT sustained -- it drops back to the fallback after ~4 s and eventually
departs. Cruise AoA offline is unchanged at 5.7 deg, so the stiffer guide does not bias the trim.

**★★★ "LEVEL" IS NOT ENOUGH -- THE SAMPLE MUST ALSO BE FLOWN.** When a solve fails the node falls
back to `safe_hover()`: equal thrust ~= weight (33.8 N), tilt 0. The vehicle then GLIDES, and a
glide at speed is level, fast AND wing-loaded. full_tr15 reported **"15.31 m/s, 79 deg, 120.7% wing,
TRUE FIXED-WING"** -- but **6697 of its 8411 level samples were at fallback thrust**, and both the
15.31 m/s and the 120.7% were recorded while gliding. The genuine controlled maximum was 12.0 m/s.
`fly_transition.py` now requires `thrust < 32 N` (not the hover fallback) for a sample to count,
prints the CONTROLLED count next to the level count, and prints the single best SIMULTANEOUS sample
-- the old line printed per-column maxima that were never simultaneous. Three runs in a row
(11 dive, 13 post-dropout, 15 glide) produced spectacular false headlines; check WHO WAS FLYING.

**TWO SILENT STACK FAULTS cost a flight each, both looking perfectly healthy:**
  * **PX4's SIM BATTERY drained to its 50% floor and `COM_LOW_BAT_ACT`=3 (Land) fired.** The
    controller commanded correct hover thrust throughout while PX4 walked the vehicle down at
    0.5 m/s. Fix: `SIM_BAT_MIN_PCT`=100, `COM_LOW_BAT_ACT`=0 (saved in the airframe params). Every
    ~5 min JIT rebuild leaves the vehicle idling airborne, so this WILL recur without the fix.
  * **STALE DDS DISCOVERY after a node restart:** the node listed the right subscriptions but
    received nothing, so `self.state` stayed None and its loop returned silently every tick -- no
    logs, no engagement, no error. The whole run was flown by the geometric controller (tilt 0).
    **Always verify the node LOGS and `ok=1` before committing to a flight**, not just that it
    appears in `ros2 node list`.

### ★★★ THE 1 m ALTITUDE BAND WAS ENDING EVERY CRUISE (2026-09-07, bags full_tr16/17)
`settling` was re-evaluated EVERY TICK as
`(self._eng < settle_s*rate) or (z < self.z0 - 1.0)`. That is meant as a PRE-transition gate
("do not tilt over until settled and at altitude"), but in cruise a **1 m altitude dip threw the
vehicle straight back to the hover reference**: tilt commanded to 0 at 12 m/s (the documented
unsurvivable manoeuvre) and `_v_cmd` re-anchored to the achieved speed. full_tr15's two wing-borne
episodes each died this way after ~4 s while the flight was holding altitude to **within 1 m**
(19.0-21.1 with z0=20) -- the REFERENCE punished it for touching the edge of its own band.
**This is the FOURTH instance of a reference surrendering to its own tracking error** (SMC re-seed
C7, GS mission clock, `build_refs` vtar, now the settle gate). Look for this pattern FIRST.
**FIX: the gate LATCHES OFF once satisfied** (`self._settled`); in cruise it only reverts to hover
on a real `cruise_alt_band` (6 m) loss. **RESULT (full_tr16): episode 4.2 -> 6.2 s, tilt 79.5 ->
83.0 deg** -- the highest CONTROLLED tilt of the campaign.

**★ DO NOT RE-ENABLE THE VERTICAL-FORCE TILT CLAMP AS WRITTEN (`lift_margin`, default 0 = OFF).**
The idea -- allow tilt only while `T_max*cos(beta) + L_wing(V) >= k*W` -- is sound and needs no
tuning (it self-relaxes to 90 deg at exactly 14.1 m/s). As implemented it made things WORSE
(full_tr17: level wing share 74.5% -> 24.7%, verdict back to MULTICOPTER):
  1. it credits the wing with its PEAK CL, which only holds at alpha=10 deg -- during acceleration
     the real lift is far lower, so the clamp was most permissive exactly when the wing was
     under-delivering;
  2. **restricting the TILT does not remove the demand for forward force, it MOVES it.** Denied
     tilt, the solver accelerated by pitching DOWN: it reached 89.4 deg tilt at 15.0 m/s with the
     wing at 95% -- while SINKING 6.8 m. Another gravity-not-lift reading.
To make it work it must use the ACTUAL modelled lift at the current state AND be paired with
something that stops the solver trading altitude for speed.

**WHERE THE CAMPAIGN STANDS:** best CONTROLLED wing-borne state is full_tr16 --
**tilt 83.0 deg, wing 86.8%, rotors 9.4 N (28% of hover), 6.2 s**. The corridor is genuinely
ENERGY-LIMITED below the 14.13 m/s stall speed: forward force must come from tilt (costs vertical
thrust) or from pitching down (costs altitude), and neither is free. Sustained 90 deg needs the
vehicle to arrive at ~14 m/s WITH altitude intact; that is the open problem.

### ★★ TWO STACK HAZARDS FOUND WHILE RE-FLYING (2026-09-07)
**(a) THE HIL LINK DIES SILENTLY AFTER A LONG SESSION.** bag full_tr21 recorded a vehicle whose
position was CONSTANT TO FOUR DECIMALS while the controller happily varied tilt 1.6->13.3 deg and
thrust 31-38 N. PX4 was printing `ERROR [simulator_mavlink] poll timeout` and Isaac was holding
the last stale controls -- after ~6.7 h of sim time and 2.1 km of travel. Nothing in the controller
or the harness noticed. **A frozen position with LIVE, VARYING commands means the LINK is gone**;
check the PX4 pane for poll timeouts before believing any flight result.

**(b) ★ THE SAFE CONTROLLER REFUSED TO RECOVER A RUNAWAY -- MY OWN ENVELOPE GUARD.** After the
restart the geometric controller still had flight 21's LATCHED target (2500 m ahead) and flew
toward it at 34 m/s, descending. Commanding a hover at the vehicle's position did NOTHING because
the guard accepts a target only if it is within `radius_max` (300 m) of HOME **or** `radius_local`
(100 m) of the VEHICLE -- and at 34 m/s the vehicle outran the 100 m local ball between reading
its position and the command taking effect. It reached 11 m altitude before `radius_max`/
`radius_local` were raised, after which it recovered instantly.
**`radius_local` MUST scale with speed** (100 m is 3 s of flight at cruise) -- or the recovery
target must be placed AHEAD of the vehicle, not where it was. This is the SECOND time this guard
has blocked the one controller whose job is to save the aircraft (the first was the home-relative
check after 1430 m of cumulative flight). A safety fallback that can refuse is not a fallback.
**Also: controllers RETAIN their last reference across a PX4/Isaac restart** -- always command a
local hover BEFORE relaunching, or the vehicle bolts for a stale target the moment the link comes up.

### CAMPAIGN CLOSE-OUT: what damps the phugoid, and four things that DO NOT (2026-09-07)
**STATE:** 90 deg wing-borne flight is REPEATABLE. full_tr28 (w_vz=2.5 build, flight-20 params):
four episodes of 8-11 s at **tilt 90.0, rotors 8.8 N of 33.8 N weight, wing 195-350%, V 13.7-17.5
avg**. full_tr20 (w_vz=0) gave the LONGEST episode (41.7 s) at lower speed (14.1 avg). The phugoid
(20-40 m altitude swing) is the one remaining defect.

**THE ONE THING THAT TOUCHED IT: `w_vz` -- weight on (vz - vz_ref)^2.**
  * w_vz=0  (full_tr20): 41.7 s episodes, 27 m phugoid, V 14.1
  * w_vz=2.5 (full_tr24/28): 8-11 s episodes, V up to 17.5, still 20-40 m
  * w_vz=6  (full_tr23): phugoid 27 -> **7.6 m**, but the episode collapses after 14.7 s -- holding
    altitude that rigidly stops the vehicle trading height for speed, V decays below the 14.13 m/s
    stall and it drops into an 8 m/s mush.
  The trade is real and bracketed; the sweet spot is between 2.5 and 6 and was not found.

**FOUR THINGS THAT DO NOT WORK (each cost a flight; do not retry as written):**
1. **TECS energy/balance** (full_tr22). B = g*dz - 0.5*d(V^2) is the textbook phugoid term, but the
   ROLLING TRANSITION IS ITSELF a potential->kinetic exchange, so it charges for the corridor
   crossing. Never crossed: tilt 73 vs 90, wing 61% vs 238%.
2. **Longer horizon `DT` 0.20 -> 0.30** (offline gate). 2.4 s vs the ~4 s phugoid is the right idea,
   but RK1 at 0.30 s cannot integrate the vertical dynamics: the offline test drifted z 20 -> 50 m
   and the last solve failed. The correct version is DT=0.25/NRK4=2, which does NOT build inside
   the 15-minute timeout. NOT FLOWN.
3. **Vertical-force tilt clamp** (full_tr17). Credits the wing with its PEAK CL (only true at
   alpha=10 deg) so it is most permissive when the wing under-delivers; and restricting TILT does
   not remove the demand for forward force, it MOVES it -- the solver dove instead. Wing 74->25%.
4. **★ COMMANDING THRUST IN THE REFERENCE** (full_tr25/27). The trim asks only 5-19% throttle at
   ANY speed (drag is tiny; 80% would balance at ~80 m/s), so the vehicle coasts at 12-14 m/s with
   72 N unused. But you cannot fix that by asking for more:
     - demanding more SPEED (`v_lead` 15) -> the solver bought speed with GRAVITY: **0.0 N thrust,
       81 m dive**;
     - overriding `uref` thrust to 28.8 N -> 28.8 N front against 2 N rear is a huge pitch moment
       and a vertical over-thrust, so (xref,uref) stopped being a balanced state. The solver sat in
       FALLBACK for the entire flight (625/625 level samples at hover thrust), lost 84 m.
   **SAME ERROR AS full_tr7: A TRIM IS A PACKAGE.** For a given speed in level flight the trim IS
   the trim -- thrust is not independently commandable. The correct mechanism already exists:
   `so3.trim_split(..., Fx_req=M*a_cmd)` solves the TILT as a third unknown while keeping vertical
   and pitch balance. That is a SCHEDULE change (nmpc6.Schedule.at), not a uref override, and it is
   the recommended next step.

**AIRFRAME FACT worth stating: ONLY THE FRONT PAIR TILTS.** At 90 deg the rear rotors still point
straight up, so the maximum FORWARD thrust is the front pair alone -- 2 x 18 = 36 N = **50% of
installed thrust**. "80% throttle in cruise" is not a thing this airframe can do.

**PROCESS: an unasserted string edit cost a flight.** Four substitutions were applied to the node
and only two carried `assert`; the `accel_thr` parameter declaration silently did not match
(`descent_budget` defaults to 20.0, the patch said 45.0), so the node raised
ParameterNotDeclaredException every loop and the vehicle fell 108 m. **The offline `--test` passed
throughout because it uses the class defaults and never touches the ROS parameter path.** ASSERT
EVERY EDIT, and check the node pane for tracebacks + `ok=1` before every flight -- the offline gate
cannot see this class of bug.

## ★ STANDALONE `vtol_nmpc.py` — TRANSITION SOLVED offline (2026-08-13)
`ros2/px4_vtol_direct_actuator/.../vtol_nmpc.py`: ONE file, ZERO cross-imports (plant + trim table +
NMPC + ROS node + selftest all inlined; trim baked as a TABLE so the AVTOL generators aren't needed).
Built to escape the tangle after 4 failed HIL runs. Controller id `vtol_nmpc`.
**SELFTEST RESULT (vs the same plant, lat=0.25): hover -> 90 deg wing-borne, ok=1 EVERY solve.**
The AIRSPEED TILT CLAMP is visibly doing its job: tilt HELD at 11-12 deg while speed builds 0->10 m/s
(refuses to over-tilt while the wing can't carry), then COMMITS 15->41->62->75->90 as the wing loads.
**Altitude DEAD-FLAT 20.0 m through the whole transition, roll=0, yaw=0**; cruise tilt=90, V=16,
AoA +6.3, Tf=3.7 N / Tr=0.2 N (rotors carry only drag).
**OPEN — sustained cruise wanders after ~10 s:** tilt oscillates 83-89-79-73-87-64, AoA swings
0.7-11.9-2.2-12.5, altitude drifts 20.0->20.8, Tr touches 0/negative. At full unload there is almost
no control authority and 10 Hz + 0.25 s delay-comp is marginal for that lightly-damped mode. Fixes to
try: a rear-rotor FLOOR (keep ~1-2 N for authority, as nmpc6 needed), more w_om, or a higher rate if
the HIL solve budget allows. TRANSITION is the part that was broken and it is now fixed.
The 7 hard-won invariants are documented in the file header (tilt clamp, real-time sizing, never zero
the velocity ref, state bounds, ref-FF fallback, penalise surfaces, ghost publishers).
NOT yet HIL-flown -- fly it on the MINIMAL stack (`scripts/start_vtol_minimal.sh`, verify
`Publisher count: 2`); the last 2 HIL crashes were ghost publishers flipping it before engage.

### vtol_nmpc HIL bags vt1/vt2 — two AMPLIFIER bugs in the failure paths (2026-08-14)
**vt1: the NODE DIED.** Hover was rock solid (z=15.0, roll/pitch/yaw 0, tilt clamp working exactly:
held ~14 deg = trim(V)+12). Then pitch drifted -5 -> -20 and at t=7.3 s it inverted in one step. The
tell: **actuators FROZE byte-identical (`mot=[0.60,0.60,0.81,0.81] tilt=0.28`) for 17 s** while it
fell, and every monitor line after engage was blank -> an uncaught exception killed the loop, PX4
HELD the last command. FIX: the loop body is now wrapped (`_loop()`), logging the traceback ONCE and
falling back to hover. A frozen actuator stream is the worst failure mode; it must be impossible.
**vt2: the FALLBACK amplified an upset into a death spiral.** Every solve returned `ok=0`, and the
solve-failure path applied `urefs[0]` = the CRUISE TRIM, which at speed is **tilt=90**. So while the
vehicle was inverted at AoA +75 it kept commanding full forward tilt -> deeper upset -> next solve
also failed -> unrecoverable (roll +-180, altitude 5.8-53.3, AoA 70-97). FIX: the trim feed-forward
is applied ONLY while the state is near the reference; if `diverged()` (|roll|>50, |pitch|>35, or
R[2,2]<0.4) the fallback is `safe_hover()` (rotors UP, equal thrust, neutral surfaces) -- the only
command recoverable from an upset.
Also: `w_beta` 12 -> 60 (the airspeed TILT CLAMP now owns corridor safety, so the soft tilt guide is
no longer needed; soft w_beta left a flat tilt-vs-pitch optimum -> cruise tilt hunting 43<->89).
**STILL OPEN: why does every solve fail (`ok=0`) once it is moving?** MAX_ITER=25 may be too low
after the w_beta change. That is the entry cause; the two fixes above only stop the amplification.
Ops: /tmp was wiped again -- `scripts/rosenv.sh` + `scripts/stop_bag.sh` now live in the repo. A bag
killed with -9 is UNREADABLE (sqlite I/O error, no metadata.yaml): always SIGINT the recorder.
