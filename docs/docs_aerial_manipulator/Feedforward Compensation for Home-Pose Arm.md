# Feedforward Compensation for the Home-Pose Arm

**Scope.** The AM-T650 fallback rig (§7.7 of [Command.md](Command.md)): the aerial
manipulator flown by `fsc_autopilot_ros2`'s quadrotor-only DIRECT-actuation
controller with the arm **PD-held at its home pose** `q_home = [0, 40°, 40°, 0]`.
Because the arm never moves in this configuration, its effect on the drone is a
**constant, fully computable disturbance** — exactly the case that belongs in
feedforward rather than being re-learned by an integrator on every engagement.

**Result up front: no control-law change was needed.** `fsc_autopilot_ros2`
already ships the right mechanism (`ratectl_trim_*`), and the correct value is
derivable in closed form. The fix is one number in one config file:

```yaml
ratectl_trim_y: -0.040        # params_single_drone_direct_actuation_am_t650.yaml
```

Measured effect on the reported symptom — X peak excursion at DIRECT engagement
**97.7 cm → 2.1 cm**, settling **23.1 s → 4.1 s** (§4.1).

---

## 1. Symptom

In a DIRECT hover the **X axis** is slow and underdamped — a single large
undershoot taking ~8 s to settle — while **Y and Z are clean**. Nothing in the
gains is x/y-asymmetric (`posctl_k_pos_x == k_pos_y`, `ratectl_kp_x == kp_y`),
so an axis-asymmetric symptom means an axis-asymmetric *plant*.

## 2. Cause

The arm folded at home hangs **forward along body +x** (after the x-forward
re-authoring, §7.7). That moves the system centre of mass forward of the
geometric centre where the rotors are symmetric.

| Quantity | Value |
|---|---|
| CoM offset `dx` (forward) | **+19.5 mm** |
| CoM offset `dy` (lateral) | −0.1 mm (zero — arm is symmetric about x–z at `q1` = 0) |
| Hover thrust `T` | 36.75 N |
| Standing pitch moment `dx·T` | **+0.72 N·m**, nose-down |

With the rotors commanded equally, weight acts 19.5 mm ahead of where thrust
acts, so the vehicle pitches nose-down. **Pitch is what drives X**, and `dy` ≈ 0
is why roll/Y is untouched — the asymmetry of the symptom is the asymmetry of
the arm.

`alloc_rotor*_px/py` in the config are the **geometric** rotor centres, so the
allocator does not know about the offset. Per `rate_controller.hpp`, that leaves
exactly one thing able to cancel it:

> *"a centre-of-mass offset produces a roughly constant torque bias that ONLY the
> integrator can absorb (the attitude loop is pure P, and the UDE compensates
> force, not torque). Starting from zero means every engagement re-learns it from
> scratch, and the vehicle tilts while it does — measured on the X650 at 8.92 deg
> for a 2 cm offset."*

Our offset is 1.95 cm. Integral action is slow and adds phase lag, so the
position loop above it inherits both the lag and the transient. That is the
sluggish, poorly damped X.

### Two independent measurements of `dx`

1. **Forward, from the model.** `dynamics()` at `q_home` with the T650 body-mass
   override gives `r_0c` = 19.50 mm forward (after mapping the model's old
   +y-front frame through `Rz(−90°)`).
2. **Backward, from a real flight.** The ground station showed hover rotor
   commands **front 59.7 / 59.6 %, rear 54.1 / 54.0 %**. Converting through
   `ω = 665.99u + 64.06`, `F = kω²` gives front 9.96 N, rear 8.42 N, total
   36.75 N (= `m·g`, a good consistency check), and
   `dx = Σxᵢ Fᵢ / T` = **19.3 mm**.

Agreement to 0.2 mm. The cause is not in doubt.

## 3. Why the obvious fix is wrong

The natural move — reference `alloc_rotor*_px` to the **true** CoM
(`px' = px − dx`), which is `design_decisions.md`'s own action item — **does not
work here, and makes things worse in the opposite direction.**

The allocator's effectiveness matrix assumes **thrust ∝ normalized command**. The
real curve is `F = k(665.99·u + 64.06)²`, which linearised about hover has a
large **negative intercept** (≈ −6.5 N at `u` = 0). So a motor set satisfying
`Σ px'ᵢ·uᵢ = 0` does *not* satisfy `Σ px'ᵢ·Fᵢ = 0`:

| Allocation | Residual pitch moment about the true CoM |
|---|---|
| Geometric (shipped) | **+0.72 N·m** |
| CoM-referenced `px' = px − dx` | **−0.50 N·m** (over-corrected) |

Simulated through this repo's own `NormalizedMix()`. Worth knowing before anyone
"fixes" the allocation table again.

## 4. The fix

Seed the rate integrator with the known bias — `ratectl_trim_*`, which exists for
this exact purpose and is applied in `RateController::reset()` at engagement:

```cpp
integrator_ = trim().cwiseMax(-i_max()).cwiseMin(i_max());
```

**Value, computed not tuned.** Solve for the normalized pitch torque whose
allocated motor commands produce zero *physical* moment about the true CoM at
hover collective (0.5691), keeping the real quadratic thrust curve in the loop:

```
tau_y_FRD  = +0.03997        ->  ratectl_trim_y (FLU) = -0.040
```

Sign chain, verified twice: CoM forward ⇒ nose-down bias ⇒ need nose-**up** ⇒
positive in FRD (`+y` right, rotation `z→x` lifts the nose) ⇒ **negative in FLU**
(`+y` left), and `InterconvertFluFrd` negates `y`. The rate controller's output —
and therefore `ratectl_trim_*` — is **FLU**.

**Cross-check.** The trim predicts hover commands **59.7 / 54.1 / 59.7 / 54.1 %**
against the **59.7 / 54.1 / 59.6 / 54.0 %** actually flown. Scale reference:
normalized pitch `1.0` = **11.12 N·m**; the trim uses **44%** of `i_max` (0.09).

`trim_x` and `trim_z` stay `0.0`: `dy` ≈ 0 (no standing roll bias) and rotor-drag
yaw is balanced by construction.

### 4.1 Measured A/B (2026-08-10, headless full stack)

Matched pair: identical vehicle, identical reference, only `ratectl_trim_y`
differs. Each trial arms from the ground (the arming edge is what seeds the
integrator — see §4.2), climbs to z = 1.2 m in SAFETY, then engages DIRECT while
recording `rate_control_debug[4]` and X.

| | `trim_y = 0` | `trim_y = -0.040` |
|---|---|---|
| `I_y` at DIRECT engage | 0.00000 | **−0.04000** (exactly the seed) |
| `I_y` settled | −0.03935 | −0.03938 |
| **X peak excursion** | **97.7 cm** | **2.1 cm** |
| **X settling (\|err\| < 2 cm)** | **23.1 s** | **4.1 s** |

**47× smaller excursion, 5.6× faster settling.** The X trace without the trim
tells the story directly — it is the integrator winding up, seen from the
position loop:

```
 t[s]   I_y        x[m]          <- trim_y = 0
    0  -0.00000  +0.0819
    1  -0.04184  +0.1455
    2  -0.02911  +0.5182
    3  -0.04302  +0.9002         <- ~1 m off, still hunting
    5  -0.03732  +1.1378
    8  -0.03883  -0.0301
   12  -0.03993  -0.2199
   20  -0.03959  -0.1733
```

Both trials settle to the *same* `I_y` ≈ −0.0394. That is the point: the bias is
real either way, and the only question is whether the vehicle flies a metre
sideways discovering it.

### 4.2 Two implementation facts that shape how you test this

Both were learned the hard way while producing the table above.

1. **`ratectl_trim_*` is read once, at node startup.** There is no
   `add_on_set_parameters_callback` anywhere in `fsc_autopilot_ros2_node`, so
   `ros2 param set ratectl_trim_y …` updates the parameter server and changes
   what `ros2 param get` reports **while the controller keeps using the value it
   loaded from the YAML**. An A/B done that way silently compares a config
   against itself. Relaunch the node with a different `params_file:=` instead.
2. **`reset()` — which applies the trim — fires on the ARMING edge, not on the
   mode switch.** `single_drone_direct_actuation_client.cpp:859` calls it on the
   disengaged→engaged transition; the SAFETY↔DIRECT handover deliberately calls
   `resetDerivative()` instead, which *preserves* the integrator (line 719: "This
   used to call `rate_ctrl_.reset()`, which threw away the torque trim"). So
   toggling DIRECT off and on does **not** re-test the seeding — you must land,
   disarm, and re-arm between trials.

## 5. What this does and does not do

- **Removes the wind-up transient in DIRECT.** The integrator starts where it
  would have ended, so X no longer waits for it. Steady-state behaviour was
  already correct — the integrator got there eventually; this removes the
  *getting there*.
- **Self-limiting, not self-scaling.** The required moment `dx·T` scales with
  thrust; a fixed trim is exact at hover and slightly off during aggressive
  climbs. Irrelevant for a hovering fallback rig; the integrator mops up the
  remainder as before.
- **DIRECT only.** In SAFETY, PX4 owns the rate loop and its own integrator
  re-learns the bias every time — the same transient, unreachable from this
  config. Takeoff happens in SAFETY, so expect it there until someone sets PX4's
  own trim/geometry. Not fixed here.
- **Re-measure if the home pose, the arm, or the mass changes.** `dx` is a
  property of `q_home`. Procedure: fly DIRECT, let it settle, read elements
  `[3..5]` of `fsc_autopilot_ros2/direct_actuation/rate_control_debug`.

## 6. Both paths: what is compensated where (2026-08-11)

The rig now has **two** configs, one per control path, and the arm's disturbance
splits into two parts that are **not** equally reachable.

| Arm effect | DIRECT (`params_single_drone_direct_actuation_am_t650.yaml`) | Baseline / SAFETY (`params_single_vehicle_baseline_am_t650.yaml`) |
|---|---|---|
| **Force** — weight 0.71 kg → 7.0 N | `vehicle_mass: 3.746170` | **same — the deliverable here** |
| Hover point | `thrust_scaling` / `idle_thrust` re-derived | same |
| **Torque** — CoM 19.5 mm fwd → +0.72 N·m | `ratectl_trim_y: -0.040` ✓ | **not reachable** |
| Added inertia | PX4-side gains, out of scope | same |

**The force term is the arm feedforward on both paths**, and it is delivered
entirely by `vehicle_mass`, which is read twice independently:
`robust_controller.cpp`'s gravity feedforward and `velocity_based_ude.cpp`'s
weight and damping terms.

> **How to check it in one number.** Watch
> `position_controller/ude` → `disturbance_estimate.z`. Measured on both paths
> after this change: **+0.05 to +0.06 N**, i.e. zero. Before the AM baseline
> config existed, the baseline path would have loaded the bare-T650 file's
> `vehicle_mass: 2.9` against a 3.746 kg plant — a 29% error the UDE would have
> carried as roughly **−8.3 N**, most of its ±10 N budget spent on a known
> modelling error.

**The torque term cannot be fed forward on the baseline path — this is a hard
fact, not a to-do.** Only an integrator can absorb a standing torque, and there
the integrator belongs to PX4: the baseline node builds no rate controller (it
publishes `VehicleAttitudeSetpoint`; PX4 runs attitude+rate+mixer), so
`ratectl_trim_*` does not exist for it. PX4 v1.16 has no multicopter equivalent
either — `TRIM_PITCH` is defined only in `fw_rate_control` and
`commander/rc_calibration`, with **zero** references under `control_allocator/`
or `mc_rate_control/`. CoM-referencing `CA_ROTOR*_PX` fails for the same reason
it failed on this repo's own allocator (§3). PX4's own rate integrator absorbs
it, measured benign in both modes. **Do not add an inert `ratectl_trim_y` key to
the baseline yaml.**

## 7. Why feedforward does not fix slow step convergence

A separate symptom, investigated 2026-08-11: step responses converged slowly in
x, y **and** z, and the natural guess was a missing feedforward term. It is not.

**The position law already implements feedforward** — velocity, acceleration and
thrust (`robust_controller.cpp:64-82`), and `PositionControllerReference` already
carries all three fields. But a **step** reference has zero velocity and zero
acceleration by definition, so every feedforward term evaluates to zero. *No*
feedforward, existing or added, can speed up a step. Feedforward pays off only
against a **smooth** reference — which is worth doing when the whole-body
controller starts issuing trajectories, and costs nothing extra because the path
already exists.

What actually sets the convergence is the closed-loop poles. `posctl_pid_form:
"nested"` computes `pd_term = k_vel*(vel_err + k_pos*pos_err)` **in newtons with
no mass normalisation**, so `wn = sqrt(k_vel*k_pos/m)` and
`zeta = sqrt(k_vel/(4*k_pos*m))` — and the arm's +23% mass alone cost 10% of both
versus the T650 the gains were validated on. The fix is therefore gains, per path:

- **Baseline → `k_vel` 3.70** = 3.0 × 3.746170/3.033921, a pure **mass restore**
  that reproduces the T650's hardware-validated response exactly (`wn` 0.994,
  `zeta` 0.497). Deliberately not raised further:
  `docs/sim_to_real_fidelity.md` measured sim **overstating the settling benefit
  of raising `k_vel` by 5.8×**, which is how a sim-tuned 8.0 was walked back to
  3.0 on hardware.
- **DIRECT → `k_vel` 7.0**, the X650 DIRECT value, hardware-validated at a
  near-identical 3.5 kg, where higher `k_vel` is *stabilising* because DIRECT adds
  ~120 ms of attitude lag.

**Measured A/B on the DIRECT path** (0.6 m steps in x/y, 0.4 m in z; headless full
stack). 7.0 wins on every axis and every metric:

| | rise | overshoot | settle (2%) | final err |
|---|---|---|---|---|
| `k_vel` 3.0 — x/y | 2.0–2.5 s | 42–53% | 19–20 s | ±52–100 mm |
| `k_vel` 3.0 — z | 2.3–2.4 s | 14–19% | 11.0 s | ±5 mm |
| **`k_vel` 7.0 — x/y** | **1.6–1.7 s** | **25–30%** | **15.7–17.8 s** | **±2–14 mm** |
| **`k_vel` 7.0 — z** | **2.0–2.2 s** | **0–0.5%** | **5.8 s** | **±2 mm** |

> **The second-order formula describes the Z AXIS ONLY.** It predicted 4.3 s /
> 5.3% at `k_vel` 7.0; z measured 5.8 s / 0%, but x/y measured 15.7–17.8 s /
> 25–30%. The reason is structural: z drives collective thrust directly, while
> x and y close through the **attitude loop**, which the formula ignores.

### 7.1 Removing the x/y ringing — `k_pos`, after two failed hypotheses

The x/y oscillation above (visible as several decaying cycles on a 1 m step, with
z perfectly clean) was chased to ground on 2026-08-11. **Two plausible causes were
measured and rejected first**; both are recorded so they are not retried.

**Rejected — more torque feedforward.** The natural guess, but wrong twice over:
the arm's standing torque is *already* compensated on this path (`ratectl_trim_y`;
the integrator settles at −0.0394), and more decisively, **a pitch-only bias
cannot ring the roll axis**. The CoM offset is purely in x (`dy` = −0.1 mm), yet
y oscillated *as much as* x (measured y overshoot 29.5% vs x 25.5%). A constant
bias shifts an equilibrium; it does not move poles.

**Rejected — inertia-scaled rate gains.** The arm genuinely does raise rotational
inertia by **×1.53 roll / ×1.86 pitch / ×2.02 yaw** (coupled mass matrix at
`q_home` vs the bare T650 the gains were tuned on), so the rate loop really is
running at about half its tuned gain. Scaling `ratectl_kp/ki/kd` by that ratio
(kp_y 0.045 → 0.0838 etc.) measured **no improvement**: overshoot 23.5–29.5%
against 25–30%, settling slightly worse. The rate loop is not the limiter — the
**99.7 ms rotor lag** is, and no gain in this file can shorten it.

**The fix: lower `k_pos`, not raise anything.** Since
`zeta = sqrt(k_vel/(4*k_pos*m))`, lowering `k_pos` raises damping *without* asking
the inner loop for more bandwidth — the standard cascade remedy when the inner
loop's lag is fixed. `k_pos_x/y` 1.0 → **0.6**, `k_vel` 7.0 unchanged:

| | k_pos 1.0 | **k_pos 0.6** |
|---|---|---|
| x/y overshoot | 25–30% | **0.0–0.4%** |
| x/y settling (2%) | 15.7–18.9 s | **9.6–10.1 s** |
| x/y final error | ±2–14 mm | **±0.1–3.0 mm** |
| x/y rise | 1.6–1.7 s | 3.0–3.8 s |

The ringing is gone. Rise time roughly doubles — that is the trade — but settling
still nearly **halves**, because there is no longer any oscillation to decay.
`k_pos_z` stays 1.0: z never rang.

Note this is a *reduction* in gain, i.e. the conservative direction, which is why
it is safe to adopt from simulation evidence where raising `k_vel` would not have
been (§7's 5.8× warning applies to raising, not lowering).

**Still open, deliberately:** the AM **baseline** path keeps `k_pos` 1.0 /
`k_vel` 3.70 and still shows 42–50% x/y overshoot. The same `k_pos` reduction
would very likely help there too — the physics is identical and PX4's attitude
loop is also lagged — but it has not been measured on that path, and the baseline
config's remit here was the mass restore. Measure before changing it.

## 8. The true feedforward (2026-08-11) — supersedes the trim on the AM

The user asked for the real thing: the arm is a *known* wrench, so compensate it
exactly instead of seeding an integrator. This is the **first control-law change**
of the whole effort — everything before this section was config-only.

**Form.** The mixer zeroes torque about the geometric rotor centre, so the true
CoM sees `τ = −r_com × (T·ê_z)`, proportional to thrust. In the allocator's
normalized FLU units, with the affine motor map `ω = a·u + b`, the cancelling
torque is **linear in the normalized collective c**:

```
τ_ff_y(c) = −dx/(√2·L) · (c + b/a) = −0.060·c − 0.0058
            dx = 0.0195 m, L = 0.22990663 m, a = 665.9904, b = 64.0603
```

**Validated before any code was written:** at hover c = 0.5691 this evaluates to
**−0.0399** against the independently flight-calibrated trim of **−0.040** —
the closed form reproduces the flown calibration to 0.1%.

**Implementation** (fsc_autopilot_ros2, `dev_CCM`): four **optional, default-0.0**
client parameters — `system_ff_tau_{x,y}_per_coll`, `system_ff_tau_{x,y}_const` —
plus a guarded ~10-line block in `DirectActuationClient::innerLoop()` that adds
`per_coll·collective + const` to the rate controller's output torque before the
FLU→FRD conversion and allocation. Configs that do not set the keys are
**bit-identical** (kOptional loader semantics; the block is skipped) — the same
isolation pattern as `ratectl_trim_*` and `ratectl_kff_*`. The AM yaml sets the
y pair and **returns `ratectl_trim_y` to 0.0** (keeping both would
double-compensate; the flown −0.040 stays in the comment as fallback).

**Flight-validated same day** (headless full stack, 0.6 m x/y and 0.4 m z steps):

| | trim config | **feedforward config** |
|---|---|---|
| settled `I_y` (rate integrator) | −0.0394 (44% of `i_max`) | **+0.0005 ≈ 0** |
| x/y overshoot | 0.0–0.4% | 0.0–0.1% |
| x/y final error | ±0.1–3.0 mm | ±0.1–1.9 mm |
| z settling | 5.7–5.9 s | 5.1–6.4 s |

The integrator row is the point: the arm's moment is now carried **openly, every
tick, at every thrust level** — not learned, not seeded, and the integrator's
full ±0.09 authority is back in reserve for genuine disturbances. By
construction the compensation also tracks the collective through climbs and
z-steps (the trim could not — its residual changed by 0.060·Δc), and there is no
arming-edge dependence left.

What remains beyond it: this is still the **fixed-pose** (q = home) evaluation of
the arm. The q-dependent generalisation — recompute `dx(q)` from live joint
angles — is the whole-body controller's opening move, and the linear-in-collective
structure derived here carries over unchanged.

## 8.1 Gain sweep at 1.0 m steps (2026-08-11): the shipped gains are the ceiling

After the feedforward landed, the remaining "slightly underdamped" x/y feel (a
soft final approach on 1.0 m GUI steps) was swept. **Every candidate lost to the
shipped 0.6/7.0**, measured on the same 1.0 m x/y step suite:

| candidate | rise (10–90%) | settle (2%) | outcome |
|---|---|---|---|
| **kp 0.6, kv 7.0 (shipped)** | **3.5–4.5 s** | **10.1–10.8 s** | 0.0% overshoot — best |
| kp 0.5, kv 7.0 | 6.8–8.0 s | 12.3–14.3 s | overdamped-slow |
| kp 0.6, kv 9.0 | 5.6–5.9 s | 9.4–11.1 s | no settle gain, slower rise |
| `attctl_kp_angle` 3.25 → 4.5 | — | — | **flipped the vehicle on takeoff** |

The last row is the important negative result: the attitude-P softening (3.25,
inherited from the lagged-rotor tuning) is **load-bearing** — the MN4010's
99.7 ms rotor lag leaves no attitude-bandwidth headroom, and raising it to buy
x/y damping destabilizes the cascade outright. Do not retry.

**Conclusion: further x/y improvement is not a gain — it is the reference.** A
raw position step carries zero velocity/acceleration, so the law's feedforward
terms (§7) sit idle and the loop does all the work against its own lag ceiling.
Streaming a ramped or min-jerk reference (populating the message's `velocity`/
`acceleration` fields) engages those terms and sidesteps the ceiling — which is
exactly what the whole-body planner will do anyway.

## 9. If a q-dependent feedforward is ever wanted

§8's feedforward is exact for a **fixed** pose: its coefficients bake in
`dx(q_home)`. Once the arm *moves*, the extension is to recompute
`per_coll(q) = −dx(q)/(√2·L)` from live joint angles and feed it in each tick —
the linear-in-collective structure survives unchanged; only the coefficient
becomes time-varying. That needs a channel carrying the arm state into
`fsc_autopilot_ros2`, which is the whole-body controller's opening move.
Deliberately **not** built on this rig: at the fixed home pose it would compute
the same coefficients every step, and this rig exists to be the simple,
predictable fallback.

---

## Reproduce

```bash
# CoM offset from the model (pure numpy, no Isaac)
cd /home/shiqi/fsc_PegasusSimulator
python3 -c "
import sys; sys.path.insert(0,'extensions/fsc_aerial_manipulation')
import numpy as np
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C
p=C.make_params(); p['m_i'][0]+=2.95-2.4760795
X=np.concatenate([np.zeros(3),np.eye(3).flatten(order='F'),
                  np.radians([0,40,40,0]),np.zeros(3),np.zeros(3),np.zeros(4)])
r=C.dynamics(X,p)['r_0c_0']
R=np.array([[0.,1,0],[-1,0,0],[0,0,1]])          # model frame -> x-forward frame
print('dx,dy [mm] =', np.round((R@r)[:2]*1000,2), ' m_total =', round(sum(p['m_i']),5))"
```

Verify in flight: the settled value of `rate_control_debug[4]` should sit near
**−0.040**, and X should settle without the long undershoot. To A/B it, mind
both facts in §4.2 — relaunch the node with a different `params_file:=`, and
land/disarm/re-arm between trials.

---

*Written 2026-08-10 alongside the AM-T650 integration. Config lives in
`fsc_autopilot_ros2` (branch `dev_CCM`),
`config/params_single_drone_direct_actuation_am_t650.yaml`.*
