# L1 adaptive augmented disturbance observer — implementation and simulation campaign

**Date:** 2026-09-06
**Design note:** `Aerial-Manipulator-Cartesian-Impedance-Control-.../disturbance observer design/disturbance_observer_draft.tex`
("Decompose the lumped disturbances into end-effector and orthogonal components", 2026-08-27)

This is the record of implementing that note on the AM-T650 whole-body rig and
flying it in simulation against the GMO it replaces. It documents what was
built, the two places the note leaves a choice open and what measuring them
settled, and the flight results.

---

## 1. What the note asks for, and what it replaces

The whole-body law consumes a disturbance estimate in exactly three places:

```
f_d = -k_x e_x - k_v e_vx + m xdd_cd + m g e3   - d_hat_t
u_2 = M_r(...) + C_r w_0 + C_rp rho             - d_hat_r
u_3 = -J_3y^-1( ... - (Lambda_y M_y^-1 - I) F_hat_y ) - ...
```

Today those three come from a generalized-momentum observer:

```
p = M~ xi ,   p_hat_dot = h + u + d_hat ,   d_hat = K_o (p - p_hat)
F_hat_y = (J_y^#)^T d_hat                  ("lumped")
```

The note replaces the update law and adds an attribution step. Two things are
wrong with the GMO, and the note addresses one each.

**(a) One gain for two jobs.** `d_hat = K_o (p - p_hat)` closed on its own
predictor is exactly a first-order low-pass of the true residual at bandwidth
`K_o`. So `K_o` sets estimate accuracy AND loop robustness at once. On this
rig it is pinned at `0.5 / 0.1 / 0.1`, and raising the six body channels to
1.0 crashed it — the observer books the 99.7 ms rotor lag as a phantom
disturbance (Command.md §7.14). There is no setting that is both accurate and
safe.

The L1 law splits them. Layer 1 is a *deadbeat* piecewise-constant inversion of
the predictor error whose accuracy is `2 L_Sigma T_s`, uniform in time and
independent of any gain; Layer 2 is an explicit filter `C(s) = omega_c/(s +
omega_c)` on the way into the control loops, which alone decides robustness.
`omega_c` is the direct analogue of `K_o` — the same transfer function reaches
`f_d` and `u_2` either way — but now the *estimate itself*, which Steps 2–4
consume unfiltered, stays accurate however low `omega_c` is set.

**(b) No attribution.** A momentum residual measures only the SUM `d + d_e`, so

```
F_hat_y^lumped -> F_y + (J_y^#)^T d
```

The internal disturbance arrives at the impedance law as a **phantom contact
force**: the controller renders compliance against a force the environment
never applied. The note's repair is the four directions of the residual that
no contact wrench can reach — the arm self-motions `N(J_e)`, along which
`Z_0^T J_e^T = 0` exactly, so `Z_0^T w_Sigma = Z_0^T w` **whatever the
environment does**. No contact flag anywhere.

---

## 2. What was built

Everything is new files; the GMO path is byte-identical and its parity test
still passes.

| Piece | Path |
|---|---|
| Python reference + self-test | `extensions/.../robotic_arm/utils_controller/l1_observer.py` |
| C++ observer | `fsc_autopilot_ros2/.../wb_l1_observer.{hpp,cpp}` |
| Hook into the law (additive) | `wb_controller.{hpp,cpp}` — `WbGains::observer`, default `kGmo` |
| Config schema (all optional) | `WholeBodyL1Config`, keys `wb_l1_*` |
| Parity fixture | `application/robotic_arm/utils/generate_wb_l1_truth.py` |
| Parity test | `test_wb_l1_parity.cpp` — 5 gain sets x 40 steps, 1e-8 |
| Second executable | `autopilot_whole_body_l1_direct_actuation_node` |
| Simulation yaml | `params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml` |
| ROS2 stack | `scripts/isaacsim/start_whole_body_l1_direct_actuation_t650_aerial_manipulator_stack.sh` |
| Pegasus launcher | `scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_direct_actuation_sitl.sh` |
| Campaign harness | `application/robotic_arm/utils/wb_l1_{tune_cycle.sh,campaign.sh,campaign_driver.py,metrics.py,set_gains.py}` |

**One client, two executables.** The older forks in this repo are full copies
because they run genuinely different *laws*. Here the law is identical and only
the estimator's update rule changes, which is a yaml key — so `wb_l1_observer_type`
selects it and a second `main()` over the same client gives the distinct
executable name every stale-node `pgrep` guard keys on. Neither name is a
substring of the other. The ROS namespace is shared on purpose: the ground
station, the whole-body planner and the stop scripts need no changes, and the
two rigs are mutually exclusive anyway (one PX4, one vehicle). `vehicle_name`
tells them apart on the GS.

**The two yamls differ by exactly one name and one block** — verified by diff
before every campaign. If anything else differs, the comparison is not one.

---

## 3. Two places the note leaves a choice, and what measuring them settled

### 3.1 `Phi` is not the deadbeat gain of the predictor that actually runs

The note derives `Phi = A_s^-1(e^{A_s Ts} - I)` from the CONTINUOUS error
equation, and its unit-DC-gain property ("for every Ts and every A_s") holds
only if the predictor is integrated with the same exponential kernel. It
cannot be: the `(h + u)` term must be plain Euler, because the PLANT
integrates `h + u` exactly and any other weight there leaves a
`(weight - dt)(h + u)` bias — about **0.15 N on the z channel at hover**.

Over the `N = Ts/dt` sub-steps of one adaptation interval the implemented
recursion is

```
p~_N = e^{A_s Ts} p~_0 + Phi_d (d_hat - dbar_Sigma) ,
Phi_d = dt (I - e^{A_s Ts}) (I - e^{A_s dt})^-1        (diagonal)
```

so `Phi_d`, not `Phi`, is the exact deadbeat gain. `Phi_d = dt` exactly when
the adaptation fires every tick, and `Phi_d -> Phi` as `dt -> 0` at fixed `Ts`,
so this is the note's own `O(Ts^2)` statement made exact rather than
approximated.

**Measured** (self-test 2, frozen plant, constant residual):

| | note's `Phi` | `Phi_d` |
|---|---|---|
| `A_s = 2`, N = 1 | 0.79 % DC error | **2.5e-13** |
| `A_s = 20`, N = 1 | 6.9 % | **2.5e-13** |
| `A_s = 20`, N = 5 | — | **1.6e-13** |

**Consequence worth knowing:** at N = 1 the deadbeat cancellation is exact and
`A_s` **drops out of the estimate entirely** — it only weights the average
within one interval, which is the note's own Remark ("the bound is independent
of A_s, which only shapes the weights"). `A_s` becomes a live knob only when
the estimate is HELD longer than a tick, which is also the only way to buy
down the `1/Ts` noise gain of the momentum difference. So `wb_l1_adapt_period_s`
and `wb_l1_a_*` are **one trade, not two**. In noise-free simulation N = 1 is
right and `A_s` is inert; on hardware expect to hold.

### 3.2 The `L_c` metric — minimum-norm makes the phantom force WORSE here

The note's `L_c = B_a G^+` books the innovation to the actuation channel by
MINIMUM NORM, and minimum norm on this plant compares **newtons of collective
against newton-metres of joint torque**. The collective's coupling into the
wrench-free rows is

```
a_3 = (hat(r_0e) J_we + A_e)^T e_3 = [0, -0.002, -0.103, -0.001]
```

— metres of end-effector lift per joint radian — against the joint rows' exact
`I_4`. A 5.5 N thrust deficit and a ~10x smaller joint torque therefore explain
the SAME innovation, and minimum norm picks the joint torque.

**Measured** on the T650 model at the home pose, exact residual fed in,
`theta = [-5.5 N collective, 0.20/-0.15/0.05 N.m moments, ~0.03 N.m joints]`:

| `L_c` metric | `theta_hat_f` (true −5.50 N) | phantom `\|F_hat_y\|` |
|---|---|---|
| none (no attribution) | — | 0.890 |
| minimum-norm (the note) | **−0.041** | **4.000** (4.5x WORSE) |
| prior-variance (100, 0.25, 0.0025) | **−4.903** | **0.440** |

So `L_c` is generalized to `L_c = B_a W G^T (G W G^T)^-1` with a diagonal
prior-variance `W`. **Every property the note's proof uses survives for any
`W > 0`**: `Z_0^T L_c = G W G^T (G W G^T)^-1 = I_4`, the range stays inside
`R(B_a)` so `B_a^perp^T L_c = 0` and the lateral rows stay frozen, and the
error dynamics stay stable with the Lyapunov function taken in the `W^-1`
metric instead of the Euclidean one. The note itself calls this open ("Four
numbers do not determine a ten-vector, so where the correction is placed is a
design choice"). `W = 1,1,1` recovers its own choice exactly and is the code
default; the shipped yaml sets `(100, 0.25, 0.0025)` — "a 10 N thrust error is
as plausible as a 0.5 N.m body moment or a 0.05 N.m joint torque".

With a real 2.6 N contact wrench also present, recovery goes from useless to
usable:

```
w_e true      [ 1.50, -0.80,  2.00,  0.050, -0.030,  0.020]
minimum-norm  [ 1.50, -6.18,  1.05,  1.313, -0.011,  0.097]
prior-variance[ 1.50, -1.39,  1.90,  0.181,  0.002,  0.012]
```

### 3.3 The note's persistency-of-excitation remedy does not work on this plant

The note offers arm motion as the way to recover the other four matched
directions, and it is correct in principle. **Measured**: a vigorous 60 s sweep
(q1 ±30°, fold ±25°, wrist ±60°) gives an averaged projector with eigenvalues

```
[4e-4, 0.017, 0.051, 0.097, 0.89, 0.95, 1.00, 1.00]
```

so the four weak directions converge 10–2500x slower than `omega_i`; **400 s of
sweeping moved the collective estimate from −0.04 to −0.30 N** against a true
−5.5. The PE condition is satisfied and practically useless here. The metric of
§3.2 is what actually fixes it, and `omega_i` changes only how fast the fixed
point is reached, not where it is (identical from 0.2 to 10 rad/s).

---

## 4. Verification before flight

* **Geometry.** `J_e Z_0 = 0` to 4e-17, `rank J_e = 6` at every configuration,
  `J_y = S_e Jbar_e` to 3e-16, `Lambda_e` identical computed in either
  coordinate set to 1e-15, `P^2 = P` with `rank P = 4`. All six random
  configurations of the model.
* **Contact-flag-free.** `w_hat` is identical to 2.7e-15 with no contact
  wrench, with a 2.6 N one, and with a 26 N one — the property that makes the
  scheme need no contact detection.
* **C++/Python parity.** 5 gain sets x 40 steps, covering N = 1 and N = 5
  adaptation, unequal `A_s` across channel groups, both `L_c` metrics, the
  `decompose` on/off switch and a set whose bounds actually clamp. `1e-8`
  relative, the same gate as the law's own.
* **The GMO is untouched.** `WbParityTest` 4/4 and `WbReferenceBuilderTest`
  2/2 before and after, and `WbGains{}.observer == kGmo`.

---

## 5. The simulation campaign

### 5.1 What the plant carries

The `_sim` configuration is the one that carries every deliberate deviation, so
this is the configuration in which "does the augmentation work?" is a
meaningful question:

| factor | value | set where |
|---|---|---|
| thrust loss | allocator believes `kf = 4.7544506e-05`, plant truth `4.041283e-05` — **+15 %** | `alloc_thrust_coeff` |
| model uncertainty | plant mass **x1.10**, inertia **x1.10** | `sim_plant_mass_scale/_inertia_scale` |
| model uncertainty | CoM shift **10 / 10 / 5 mm** | `sim_plant_com_shift_*` |
| motor delay | first-order rotor lag, **lambda = 10.0265 1/s** (tau 99.7 ms) | Isaac plant, `LaggedQuadraticThrustCurve` |

Together these are ~10.8 N the observer must find before the vehicle holds
station — confirmed in flight: `u1 = 47.56 N` against a 36.75 N nominal hover,
with `d_hat_z = -10.81 N`.

### 5.2 Missions

**The sweep mission (§5.3) is a HOVER SOAK**: `offboard -> arm -> SAFETY climb
to 1.2 m -> settle -> DIRECT -> 20 s settle allowance -> 90 s soak -> SAFETY
abort -> descend`. In free flight the true interaction wrench is EXACTLY zero,
so whatever the controller reports as an end-effector force is entirely
phantom — the note's own proposed experiment ("settled by comparing the
free-flight reading of `F_hat_y` with the forces of the task"), and a hold is
the cleanest place to read it.

**The rig's STANDARD test (§5.4) is more**: hover at 1 m, step x, y and yaw,
then a compatible-trajectory leg. The driver flies it by default;
`--no-steps` reproduces the soak so the sweep stays comparable.

A step is NOT a reference publish here: in DIRECT the whole-body planner
captures a drone-GS target as PENDING and only an explicit Send executes it,
so every leg runs `target -> PLANNED -> Send -> EXECUTING -> HOLD`.

### 5.3 Results — four flights, 110 s of DIRECT each, none aborted

Data: `l1_observer_20260906/{gmo_baseline,l1_matched,l1_wc2,l1_wc6}.npz`,
figure `campaign.png`, scored by `wb_l1_metrics.py`.

| | GMO<br>`K_o` 0.5/0.1/0.1 | L1<br>`ω_c` = `K_o` | **L1<br>`ω_c` 2/0.5/0.5** | L1<br>`ω_c` 6/2/1 |
|---|---|---|---|---|
| estimate rise to 90 % [s] | 2.86 | 3.28 | 1.38 | **0.73** |
| entry sag [mm] | 463 | 428 | 187 | **69** |
| entry lateral excursion [mm] | 1327 | 1260 | 810 | **322** |
| position recovery [s] | 49.3 | 49.2 | **21.0** | never |
| soak CoM rms [mm] | 73.6 | 78.2 | **4.58** | 153.0 |
| steady CoM, last 30 s [mm] | 4.59 | 3.17 | **1.65** | 52.0 |
| EE error, mean [mm] | 44.1 | 47.2 | **17.9** | 135.4 |
| \|e_R\| max | 0.0885 | 0.0934 | **0.0059** | 0.2615 |
| tilt p-p [deg] | **0.108** | 0.337 | 0.564 | 22.2 |
| joint clamp, % of samples | 0 | 0 | **0** | 10.4 |
| **phantom \|F̂_y\| [N]** | **1.884** | 0.065 | **0.055** | 2.045 |
| `u1` mean [N] | 47.559 | 47.559 | 47.559 | 49.565 |
| `d̂_z` [N] | −10.809 | −10.809 | −10.809 | −12.271 |
| `ŵ_thrust` [N] | — | −10.753 | −10.733 | −12.376 |

`u1`, `d̂_z` and `ŵ_thrust` agreeing to three decimals across the first three
runs is the check that the plant and the injections really were identical in
every flight.

**(a) At matched bandwidth the two estimators are the same, and they should
be.** The GMO's `d̂ = K_o(p − p̂)` closed on its own predictor is exactly
`K_o/(s+K_o)` applied to the true residual, and the L1's is
`ω_c/(s+ω_c)` applied to a deadbeat estimate of the same thing. At `ω_c = K_o`
every control metric matches (rise 2.86 vs 3.28 s, sag 463 vs 428 mm, recovery
49.3 vs 49.2 s). That is the sanity check that the swap changed nothing it
should not have — not a result.

**(b) The phantom force is a separate axis, and it collapses. 1.88 N → 0.065 N,
a 29x reduction, at matched bandwidth and therefore at no cost in flight
quality at all.** In free flight the true interaction wrench is exactly zero,
so every newton of the GMO's 1.88 N is fictitious — a force the impedance law
renders compliance against for no reason. Three numbers show it is the
mechanism and not a coincidence:

* `ŵ_thrust = −10.753 N` of the true −10.81 N: the null-channel observer
  booked **99.5 %** of the residual to the COLLECTIVE channel, which is
  exactly what the prior-variance metric of §3.2 was designed to do and what
  minimum-norm failed at offline (0.04 of 5.5 N).
* `ŵ_e` = 0.074 N / 0.016 N·m — the estimated *interaction* wrench is
  essentially zero, which is correct: nothing is touching the arm.
* `σ(d̂^c) = 0.657 N` against `σ(d̂_f) = 0.0021 N` — the deadbeat estimate
  really is a noisy 250 Hz momentum difference and the filter really does
  remove it. That is the two-layer split working, and it is why Steps 2–4 can
  consume the unfiltered signal while the control loops cannot.

**(c) Raising `ω_c` past where the GMO can go improves everything at once.**
`K_o = 1.0` on the body channels crashed this rig, which is what pinned it at
0.5/0.1/0.1. At `ω_c` = 2/0.5/0.5 — 4–5x that — the estimate rise halves, the
entry sag falls 2.5x, the position recovery is 2.3x faster, steady CoM error
improves 2.8x, EE error 2.5x, attitude error 15x, and nothing saturates. This
is the setting shipped in the yaml.

**(d) `ω_c` = 6/2/1 is past the limit, and it fails the way the theory says it
should.** The ENTRY is the best of the four (0.73 s rise, 69 mm sag, 322 mm
excursion — the estimate takes up the mismatch almost immediately), but the
vehicle then sits in a **22° peak-to-peak limit cycle**: `d̂_z` swings between
−5 and −20 N and rails on the 20 N bound, 10.4 % of samples sit on the joint
clamp, and the phantom force becomes meaningless (peaks of 14 N). It did not
abort — the tilt stayed under the 35° envelope and the bounds held it — but it
is not a flying configuration. 6 rad/s is on top of the 10.03 rad/s rotor-lag
pole with DDS transport on top of that, which is the same mechanism that
capped the GMO.

**Not resolved:** the run raised all three channel groups at once, so which
one binds is unknown. The limit is somewhere between (2, 0.5, 0.5) and
(6, 2, 1), and separating the translational, rotational and arm channels is
the obvious next sweep — the translational channel is the one carrying the
10.8 N and may well tolerate more than the rotational one.

**Also not covered by this campaign**, and worth stating rather than implying:

* No contact. Every flight is free flight, which is what makes the phantom
  force measurable but means the *true* wrench estimate was never exercised
  against a real one. `ŵ_e` reading ~0.07 N when the answer is 0 is necessary,
  not sufficient.
* No arm motion. The base was held and the arm parked at home, so the
  persistency-of-excitation question was not flown — though §3.3 measured
  offline that it would not have helped.
* `sim_arm_backemf_enable` is false, so no joint-space disturbance was
  present. The shipped `lc_var_q = 0.0025` deliberately tells the observer a
  joint-torque error is implausible; with the back-EMF droop on, that prior is
  wrong and `lc_var_q` should be raised. That is the one shipped number this
  campaign did not test.
* One flight per configuration. The 2026-08-10 lesson on this rig is that a
  single completed run proves nothing near a stability boundary; the
  `ω_c` = 2 point should be repeated before it is trusted on hardware.

---

### 5.4 The standard test — 1 m, x/y/yaw steps, compatible trajectory

Five attempts, four completed; run E flew every leg with no refusal:

| leg | peak CoM err | settled | max tilt | duration |
|---|---|---|---|---|
| step x ±0.5 m | 248 / 268 mm | 55.2 / 62.5 mm | 2.4 / 3.0° | 11.7 / 11.5 s |
| step y ±0.5 m | 203 / 222 mm | 49.8 / 52.8 mm | 2.2 / 2.3° | 11.7 / 11.6 s |
| step yaw ±30° | 30 / 9.4 mm | 6.7 / 0.9 mm | 0.80 / 0.13° | 9.4 / 9.5 s |
| **compatible trajectory (EE −6 cm)** | **9.8 mm** | **0.74 mm** | **0.17°** | 9.5 s |

`u1` 47.57 N, `d_hat_z` −10.814, `w_hat_thrust` −10.738, zero saturation,
phantom 0.107 N. Runs C/D/E agree on those three to 3 decimals.

**The compatible-trajectory leg tracks best of anything in the mission** —
0.74 mm settled while the arm moves 6 cm and the base counter-moves. Expected,
not luck: it is the only leg whose reference is dynamically consistent by
construction. Translation costs ~250 mm peak / ~55 mm settled (the structural
`T·e_R/K_p` offset again); yaw is nearly free.

**The EE workspace at the folded home is small and lopsided.** Two flights
found this, each refused in ~0.1 s with a readable reason:

| EE step from home | \|target\| | verdict |
|---|---|---|
| out + down 0.15 m | 0.480 m | `IK did not converge` — past the arm's reach |
| in + up 0.05 m | 0.215 m | `joint limits: q3 = 63.0 vs 50` — retracting folds it further |
| **down 0.06 m** | 0.280 m | **q = [0, 27.1, 38.8, 0]° — flies** |

Down is the direction with room because it UNFOLDS. Map it with
`transition_planner.ik_position_azimuth` before guessing; the planner also
publishes the reachable set on `whole_body_planner/workspace_rz`.

**One attempt in five aborted 8.4 s after DIRECT entry** with a growing
lateral oscillation and the arm on its 3.0 N·m clamp for 21.4 % of samples.
The altitude recovered; the estimator was live throughout and the streamed
reference fresh 96.6 %. Runs B–E flew the same 1 m, so it is the run-to-run
scatter this rig is documented to have, not the observer or the altitude.

## 6. The follow-up campaign, 2026-09-06 — audit, posture, back-EMF

Three questions asked after the first campaign: is the shipped law the
manuscript's law, does it fly without the one extra term, and what happens when
the arm stops being an ideal torque source. Full tables and the raw numbers:
**Command.md §7.15.7**; data `docs/docs_aerial_manipulator/l1_final_20260906/`.

### 6.1 The law carries exactly one term the manuscript does not

Audited line by line against the note and the MATLAB original. `u1`, `u2`, the
geometric attitude chain, the impedance inner and `u3`'s DLS solve are the
published law; the DLS regularisation (`λ = 0.3`) and the saturation-consistent
rebuild are numerics and actuator bookkeeping, not force terms. The one
addition is a **clamped joint-space PID on `u3`**, and it is gain-gated, so the
node now states which way each run is configured rather than leaving it to be
inferred — grep the controller pane for `LAW CHECK`.

### 6.2 Removing it: two flights, two aborts — and the reason is not the observer

| run | outcome | peak CoM | peak tilt | joints on the clamp |
|---|---|---|---|---|
| L1, `wb_posture_* = 0` | abort 7.7 s | 1313 mm | 13.2° | 32.4% |
| L1, `wb_posture_* = 0` (repeat) | abort 8.0 s | 1408 mm | 36.1° | 46.9% |
| L1, term restored | 121 s, 8 legs | — | 8.7° | 0.0% |

**The two observers fail for different reasons, and only one of them is an
observer's to fix.** The GMO's failure mode is attribution — the lumped
residual arrives in `u3` as a phantom contact force (1.35–1.89 N measured
today), the impedance law yields to it, and the arm leaves home. The L1 removes
that: its phantom force is 0.044–0.132 N, **13–40x** smaller. What survives is
**kinematic**: the 4-DOF EE task has isolated but non-unique solutions
(`[0,40,40,0]` and `[0,128.7,−108.4,0]` deg agree on the EE pose to 4.7e-16),
so no task-space law selects a branch at all. Under the entry transient q3
reaches its +50° stop at t = 1 s and then crosses into **negative** q3 — this
asset's elbow-singular branch — while q1 rails at −35°; `J_3y` degrades, the
DLS solve demands torque the servos cannot give, and the reaction takes the
base.

So the term stays, as a **declared deviation**: 0.004 of 0.77 N·m (0.5%) in
steady flight, no null-space projection (there is none to project onto). It is
a branch guard, not a control gain, and the proper fix is an explicit branch
guard or a planner-side joint limit — stiffening the EE task instead was
already measured to be worse (§7.14.6).

### 6.3 The arm's back-EMF droop defeats both observers, 4 runs of 4

`τ_app = clip(τ_cmd) − b·q̇` with `b = [0, 0.9337, 1.4934, 0]` N·m per rad/s —
the OM-X servos' real PWM-mode behaviour, identified with no fitted parameter.
The controller does not model it; that mismatch is the test.

| observer | droop | runs | DIRECT | peak tilt | peak τ | clamp % |
|---|---|---|---|---|---|---|
| GMO | off | 2 | 122.6 / 138.0 s, completed | 8.4° | 0.73 | 0.0% |
| L1 | off | 2 | 121.3 / 136.4 s, completed | 8.7° | 0.83 | 0.0% |
| GMO | **on** | 2 | **abort 9.7 / 10.8 s** | 20.6 / 24.5° | 3.00 | 4.9 / 9.2% |
| L1 | **on** | 2 | **abort 5.8 / 5.8 s** | 32.3 / 31.0° | 2.45 | 0.0% |

Repeats agree to a few percent, so this is not scatter. **Neither observer
compensates it, and neither should be expected to**, for three reasons of
increasing depth:

1. **It is a gain error, not an additive disturbance.** `−b q̇` is a feedback
   path; an additive estimate would have to track it at the arm's own
   bandwidth. `b/I ≈ 75 1/s` against the observers' arm channel at `ω_c = 0.5`
   (L1) or `K_o = 0.1` (GMO) — 150–750x slower.
2. **The damage lands on the base.** `τ = T^T u` makes the rotors
   pre-compensate the arm's reaction through `N_1^T u_3`; if the arm delivers
   less than `u_3`, the base is compensated for a reaction that never arrives.
3. **It closes a positive loop around the entry transient.** DIRECT entry has a
   large, normal excursion; that moves the arm, the moving arm loses torque,
   the lost torque disturbs the base, the base moves the arm more. Measured on
   the GMO run: droop 0.09 N·m at t = 1 s with tilt already 5.5°, then
   **1.70 N·m** by t = 7 s — over half the joint clamp.

The two fail *differently*, which is diagnostic: the GMO takes 10 s and rails
the clamp before drifting 2.1–2.4 m; the L1 takes 5.8 s, reaches a higher tilt
and **never touches the clamp**. Its faster arm channel responds to the growing
residual sooner and drives the base harder. Faster estimation does not help
when the residual is not a disturbance the estimator can cancel.

**Not concluded:** that the law cannot fly a PWM-mode arm. Untested and worth
trying first — a gentler DIRECT entry (the transient is the trigger); an
explicit `+b q̇` feedforward in the arm channel, which is one line given `b` is
identified to ±5%; and on hardware the real fix, **current-control mode
(Mode 0)**, where `R` and `K_e` drop out, `b → 0`, and this model should be
deleted rather than compensated.

### 6.4 The comparison table

Same plant, same mission, ideal arm, **16 s hold per leg**. Figure
`l1_final_20260906/compare_gmo_l1.png`.

| leg | GMO peak / rms / settled | **L1 peak / rms / settled** |
|---|---|---|
| step x +0.5 m | 296 / 96 / 6.8 | **193 / 61 / 2.6** |
| step x −0.5 m | 251 / 80 / 5.0 | **209 / 66 / 3.0** |
| step y +0.5 m | 267 / 86 / 2.5 | **238 / 76 / 5.1** |
| step y −0.5 m | 268 / 85 / 4.9 | **222 / 70 / 4.9** |
| step yaw ±30° | 8–9 / 4.5 / 1.4–2.8 | 8 / 4.0 / 1.8–3.2 |
| **compatible traj, out** | 64 / 39 / 14.7 | **47 / 18 / 2.6** |
| **compatible traj, back** | 57 / 33 / 18.9 | **35 / 13 / 1.4** |

CoM tracking error, mm. Plus the two that are not close: the phantom force
(1.35–1.89 N vs **0.044–0.132 N**) and the DIRECT-entry transient — peak
1273/1273/1305 mm recovering in 37.9/38.6/39.3 s against **796/800/854 mm
recovering in 7.1/7.6/7.7 s**, three runs each.

**The compatible trajectory is where the L1 wins clearly** (2.4–2.9x rms,
6–13x settled), which is the expected ordering: it is the leg where arm and
base move together, so a lumped estimate is most wrong there. **The entry
transient is the most reproducible difference** and is the `ω_c`-vs-`K_o`
bandwidth split doing what it exists for. On x/y steps the L1 is 6–30% better
on peak and rms with settled errors a wash; on yaw the two are
indistinguishable.

**A hold-length trap that inverts the verdict.** An earlier pair at the
driver's default 6 s hold made the L1 look worse — 65–72 mm "settled" against
6–21 mm. This rig has a slow, lightly damped position mode, so at 6 s neither
observer has settled and the 2 s average lands at an arbitrary phase. Never
compare settled errors across runs with different `--hold-between`.

## 7. Operating notes

* Params are read at controller STARTUP; there is no on-set-parameters
  callback. Every gain change needs a yaml edit and a full relaunch —
  `wb_l1_set_gains.py` does the edit, `wb_l1_tune_cycle.sh` the relaunch.
* PX4 never disarms this rig, so the vehicle stays armed and the next run finds
  PX4 latched in flight AND the SAFETY UDE integrating the ground reaction; the
  takeoff then produces zero lift with no error message. Full clean between
  runs; the driver refuses to start against an armed vehicle.
* `vehicle_status` does not publish on this PX4 v1.16 / px4_msgs release/1.16
  pairing — it is `vehicle_status_v1`. Subscribing to the un-suffixed name
  strands a driver in its WAIT phase with no error at all.
* Gate arming on `estimator_status_flags`, never on
  `vehicle_status.pre_flight_checks_pass`: that field is false on this rig even
  while it is armed and flying.
* Land to `z >= 0.35`; the resting height is 0.305 m.
* Read `wb_control_debug` **by index, never by length** — SAFETY publishes a
  shorter prefix. `[57]` says which observer was live; `[58..61]` is the task
  force `u3` consumed on BOTH paths (lumped under the GMO, attributed under
  L1), which is what makes the phantom-force comparison like for like.
