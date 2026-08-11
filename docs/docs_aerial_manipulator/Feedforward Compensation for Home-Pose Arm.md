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

## 6. If a true feedforward is ever wanted

The trim is a *constant* seed. A genuine feedforward — recomputing `dx(q)` from
the live joint angles and injecting `dx(q)·T` every step — is the natural
extension once the arm **moves**, i.e. for the whole-body controller. It would
need a new term in the rate controller (`ratectl_kff_*` exist but are rate
feedforwards, not torque) plus a channel carrying the arm state into
`fsc_autopilot_ros2`. Deliberately **not** built now: at a fixed home pose it
would compute the same −0.040 every step, and this rig exists to be the simple,
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
