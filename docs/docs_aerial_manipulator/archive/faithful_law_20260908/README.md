> **SUPERSEDED, 2026-09-09.** The conclusion below — "keep the joint-posture
> term, at the measured minimum `kp = 0.5`" — is no longer the best answer.
> `Removing the Joint-Posture Term.md` (commit `451a903`) identifies the two
> structural mechanisms behind the failure and removes the term outright with
> two changes that leave `u3` structurally as published: anchoring the
> free-flight EE reference to the CoM, and letting `u3`'s coupling feedforward
> carry the observer estimate (`wb_u3_internal_ff`). Flown 3/3 at the full
> injection with the posture term removed.
>
> What survives from this campaign is the NEGATIVE evidence, which that note
> reaches independently: the failure is not the mismatch level (it reproduces
> on a matched plant), and **no gain can fix it** — 17 flights across three
> mismatch levels, four uniform impedance cells, two per-channel cells, two
> `M_y` cells, a DLS setting and a servo-layer joint guard. Parts 3 and 4 also
> stand on their own: `Lambda_y` cancels in the joint-stiffness map, the DLS is
> innocent, and `M_y` is overloaded (it sets the observer feed-forward
> coefficient as well as the impedance, so it cannot be used to hold the
> impedance shape).

# The published law, without the joint-posture PID — and it does not fly

2026-09-08, on request: run the whole-body **L1** rig with `u3` exactly as the
manuscript writes it (the four `wb_posture_*` zeroed), and lower the plant's
model uncertainty and arm torque-coefficient mismatch from 10 % to 5 %.

Six flights with the term removed, plus a control on the same plant with it
restored. Harness `wb_l1_tune_cycle.sh l1 <tag> fsc_lab_machine`, the §7.14
mission (takeoff → DIRECT → 20 s soak → 6 base steps → EE leg → 2 whole-system
legs → abort → land), driver defaults, scored by `wb_l1_metrics.py`. Every run
here is the L1 observer; no GMO flights.

The posture PID being off was verified on the LIVE node each run
(`ros2 param get /uav_0/fsc_autopilot_ros2 wb_posture_kp` → 0.0), not just in
the file.

## Result: 6 aborts of 6, and the mismatch level is not the variable

| run | posture PID | mismatch | k_f | outcome |
|---|---|---|---|---|
| `l1_gain10b` (arm_counts_20260908) | **on** | 10 % | +15 % | full mission |
| `pid_clean` | **on** | 0 %, matched | matched | full mission |
| `nopid10a` / `nopid10b` | off | 10 % | +15 % | **abort** 31.5 s / 12.4 s |
| `nopid05a` / `nopid05b` | off | 5 % | +15 % | **abort** 31.0 s / 8.2 s |
| `nopid_clean_a` / `_b` | off | 0 %, matched | matched | **abort** 34.7 s / 119.0 s |

Abort times are measured from the DIRECT edge; the envelope is 35° tilt.

"Mismatch" is `sim_plant_mass_scale` = `sim_plant_inertia_scale`, the CoM shift
scaled with it (10/10/5 mm at 10 %), and `sim_arm_counts_true_j*` as a fraction
over nominal. The register quantization stays on in every run — it is a real
property of an int16 Goal PWM command and cannot be "matched away".

## The failure is kinematic, not a disturbance-rejection failure

| run | q3 range | q1 min | arm clamped | max tilt | phantom \|F̂_y\| | peak CoM err |
|---|---|---|---|---|---|---|
| `gain10b` (PID on) | 8.9 … 41.4° | −11.6° | 0.0 % | 9.4° | 2.17 N | 680 mm |
| `pid_clean` (PID on) | 28.3 … 43.0° | −3.5° | 0.0 % | 4.0° | 0.70 N | 337 mm |
| `nopid10a` | −90 … +50° | −35.0° | 28.8 % | 34.1° | 30.5 N | 1255 mm |
| `nopid10b` | −90 … +50° | −35.0° | 30.3 % | 33.9° | 29.9 N | 1622 mm |
| `nopid05a` | −90 … +39.8° | −35.0° | 18.7 % | 35.3° | 31.1 N | 1166 mm |
| `nopid05b` | −90 … +50° | −35.0° | 29.4 % | 32.3° | 26.9 N | 1338 mm |
| `nopid_clean_a` | −90 … +50° | −35.0° | 11.5 % | 35.0° | 27.4 N | 1141 mm |
| `nopid_clean_b` | −90 … +50° | −35.0° | 6.9 % | 34.1° | 32.6 N | 1762 mm |

Identical in all six: `q3` reaches its **+50° stop**, crosses zero into this
asset's **elbow-singular negative branch** and runs to the −90° limit while
`q1` rails at its −35° stop. `J_3y` degrades there, the DLS solve asks for
torque the servos cannot deliver, the arm sits on the 3.0 N·m clamp for
7–30 % of samples, and the reaction takes the base.

`nopid_clean_b` is the run worth reading twice. On a matched plant it held the
20 s soak and **all six base-only step legs**, then diverged on **leg 7,
`traj_ee` — the first leg that moves the arm.** The branch escape is triggered
by arm motion, not by a disturbance the observer failed to find.

## Why lowering the mismatch cannot help

Nothing in a task-space law selects an IK branch: the 4-DOF EE task has
isolated but non-unique solutions ([0, 40, 40, 0] and [0, 128.7, −108.4, 0] deg
agree on the EE pose to 4.7e-16). The observer's job is to find the residual
wrench, and on the matched plant it does that perfectly — `pid_clean` measures
`d_hat_z` = **−0.000 N** and `u1` = **36.760 N**, exactly the 36.75 N weight,
with `dhat_rise90` 0.26 s and a 1 mm entry sag. There is nothing left for a
better estimate to fix. The 30 N phantom `|F̂_y|` in the aborted runs is a
*consequence* of the arm being in a degenerate configuration, not its cause.

## What the matched plant is worth knowing for its own sake

`pid_clean` is the first matched-plant flight of this rig, and it is the
cleanest one on record: entry peak CoM error **47.8 mm** against 680 mm at
10 % mismatch and 1245 mm without the term, entry sag 1 mm, tilt 0.97° mean /
4.0° p-p, joint tracking 0.65–0.77° mean absolute, `tau_max` 0.80 of 3.0 N·m,
phantom force 0.144 N. Read it as the floor the injections are measured
against, not as a flight result — nothing is being tested there.

## Not covered

* **One condition was not flown: the term removed with a gentler DIRECT
  entry.** Every abort begins in or just after the entry transient, and the
  entry is the largest excursion this rig ever sees. A softer handover is the
  obvious next thing to try and is untested.
* **An explicit branch guard was not implemented.** That is the actual fix
  named by the 2026-09-06 audit, and it remains unwritten. A joint-limit-aware
  IK seed or a null-space projector would let the law stay faithful; the
  posture PID is a blunt stand-in for it.
* **Two runs per condition.** Enough to rule out the run-to-run scatter this
  rig has (`nopid_clean_b` outlived `nopid_clean_a` by 3.4×, so the scatter is
  large in *time to abort* — but not in outcome, which was the same 6/6).
* **The back-EMF droop is off** in all of these (`sim_arm_backemf_enable:
  false`), as it is in the shipped config.

## Tree state

`params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml`
is left in the REQUESTED state: posture gains 0, mismatch 5 %, `+15 %`
`alloc_thrust_coeff` unchanged. It carries a header note at the posture block
saying it does not fly and how to restore it.

---

# Part 2: the gain tuning — why no gain can do it

Ten more flights (2026-09-08, same harness, matched plant unless stated). The
question was whether the published law's own knobs — the EE impedance
`wb_ky_*` / `wb_dy_*`, the DLS regularization, and a joint guard placed
OUTSIDE the law — can replace the posture term. They cannot, and the reason is
measurable before any of them is flown.

## The measurement that settles it

With the base doing its job (CoM and attitude held at their references),
`xi[0:6] = 0` and `rho = qdot`, so the EE task rate from pure joint motion is
`ydot = J_y[:, 6:10] qdot`. The singular values of that 4x4 block at the folded
home pose are

    1.016   0.282   0.230   0.0657      [m per rad]

and the weakest right-singular vector is **[-0.002, +0.58, -0.82, 0]** — the
COUNTER-FOLD `q2` up / `q3` down. That is exactly the escape every failed
flight executes.

The joint-space stiffness the task term supplies along a joint direction `u` is
`K_y |J_y[:,6:10] u|^2`. Along the weak direction:

| arm pose | sigma_min | task stiffness at K_y = 2 | K_y needed to match kp = 2 |
|---|---|---|---|
| home [0, 40, 40, 0] | 0.0657 | 0.0086 N·m/rad | **463** |
| drifted [0, 50, 16, 0] | 0.0470 | 0.0044 N·m/rad | **907** |
| drifted [0, 50, 5, 0] | 0.0370 | 0.0027 N·m/rad | **1461** |

Two things follow. The task term is **230x weaker** than the posture PID in the
one direction that matters; and **the direction becomes less observable as the
arm escapes along it**, so the required gain grows without bound. This is a
positive feedback, not an operating point that a fixed gain can cover.
Reproduce with `authority.py` (in the campaign notes) or any FK of the model.

## The sweep, and it agrees

Faithful law, matched plant, `K_y`/`D_y` on the x/y/z rows only (the psi row
stays at 0.3 — raising it drove `q4` across its full clamp on the L1 path).

| run | K_y | D_y | extra | time to abort | q3 range | arm clamped | max tilt |
|---|---|---|---|---|---|---|---|
| `nopid_clean_a/b` | 2 | 4 | — | 34.7 / 119.1 s | −90 … +50 | 11.5 / 6.9 % | 35 / 34° |
| `ky2_dy12` | 2 | **12** | — | **81.9 s** | −90 … +50 | 11.7 % | 25.6° |
| `ky8_dy12` | 8 | 12 | — | 7.6 s | −82 … +40 | 27.0 % | 30.4° |
| `ky8_dy24` | 8 | 24 | — | 6.5 s | −90 … +40 | 46.4 % | 32.6° |
| `ky20_dy24` | 20 | 24 | — | 7.7 s | −90 … +46 | 39.8 % | 33.9° |
| `guard_a/b` | 2 | 12 | servo guard | 49.7 / 65.0 s | **−15.1 / −16.8** | 9.0 / 6.9 % | 34 / 36° |
| `dy24_guard` | 2 | 24 | servo guard | 5.2 s | −5.7 … +40 | 40.2 % | 23.2° |
| `dls1_guard` | 2 | 12 | guard, lambda 1.0 | 5.3 s | −14.0 … +40 | 33.6 % | 21.2° |

**Damping is the only knob that ever helped** (34.7 → 81.9 s at `D_y` 12), and
only by slowing the drift — it never stops it. **Stiffness hurts
monotonically**: `K_y` 8 and 20 abort in 6–8 s, an order of magnitude sooner
than `K_y` 2, because a stiffer task on this laggy plant couples the (large,
observable) task errors into the arm faster than it resists the (invisible)
internal one. The earlier GMO-path result at `K_y` 20 was not a GMO artifact
after all — it reproduces on the L1 path at zero mismatch.

## The joint guard: works mechanically, does not save the vehicle

`ExternalTorqueController`'s position-limit pull-back moved to q2/q3 >= +15°
with its PD stiffened 3.0/0.25 -> 12.0/1.0 — a guard in the SERVO layer, where
hardware puts it, leaving `u3` byte-faithful.

It does what it says: `q3` bottoms at **−15.1° / −16.8°** instead of −90.0°,
`q1` never leaves 0°, clamping falls to 7–9%. And both runs still diverge, with
the **largest CoM excursions of the whole campaign** (2.70 / 2.74 m). Blocking
the escape does not restore stability: the arm pins against the guard, the task
becomes unsatisfiable, and the base runs away instead. Adding more damping or
more DLS damping on top made it much worse (5.2 / 5.3 s).

## Conclusion

Eleven faithful-law flights, zero completions, across three plant mismatch
levels, four impedance-gain cells, a DLS setting and a servo-layer joint guard.
The obstruction is not a tune: the published task set has a joint direction it
can barely observe, and the arm leaves along it. The remaining options are all
structural:

1. **Keep the joint-space term and declare it** (what ships). 0.5% of `u3` in
   steady flight.
2. **Project it onto the weak direction** — `u3 += -k (v v^T)(q - q_d)` with `v`
   the smallest right-singular vector of `J_y[:,6:10]`. Nearly task-neutral by
   construction (it perturbs the task 13x less than a flat PID at the home
   pose), and it is a standard, defensible regularization of a rank-deficient
   task map rather than an ad-hoc PID.
3. **Augment the task** with a ninth regulated output that observes the fold
   (e.g. beta = q2 + q3). Same content as 2, stated as task design, and the
   cleanest thing to publish.

Not tried: a gentler DIRECT entry (every abort still begins in that transient),
and per-channel `K_y` — the sweep moved x/y/z together. Neither addresses the
sigma_min = 0.066 obstruction, so neither is expected to change the verdict.

---

# Part 3: the corrected analysis, and M_y

Part 2's "sigma_min weak direction / K_y = 463" explanation is **WRONG** and is
superseded here. It computed the joint stiffness as `J^T K J`. That is not what
the law does: it maps task force to joint torque through the damped
least-squares **inverse** of `J_3y`, not a transpose.

## What the feedback term is actually worth

`Mtilde` is block diagonal (verified: off-diagonal blocks exactly 0), so the
code's `J_3y = Lam_y J3q M_rho^-1` — verified numerically to **1.8e-15**. The
`Lam_y` in the gain therefore **cancels** the one inside the inverse:

    tau_fb = -J_3y^# [Lam_y M_y^-1 K_y e_y]
    K_q    =  M_rho J3q^-1 (M_y^-1 K_y) J3q          [N.m/rad]

a SIMILARITY transform of `M_y^-1 K_y` scaled by the arm's transformed inertia.
So:

* **`Lam_y` does not shrink the feedback.** The user's hypothesis about
  `Lam_y M_y^-1` is right about the `M_y^-1` half and wrong about `Lam_y`.
* **The task conditioning largely drops out.** `K_q` eigenvalues are
  0.021–0.133 N·m/rad, a spread of 6, not the 240 Part 2 claimed. There is no
  single catastrophically weak direction; the whole loop is **15–95x softer**
  than the posture PID's flat 2.0 N·m/rad.
* **The DLS is innocent.** `sigma(J_3y)` = 8.7, 3.8, 2.9, 0.98 against
  `lambda` = 0.3 → worst attenuation 0.915, i.e. at most 8.5%. (Consistent with
  `lambda` 1.0 flying no differently.)
* Closing the gap needs `K_y/M_y` ≈ 30–200, i.e. a task loop at
  **5.5–14 rad/s**, straddling the 10.03 rad/s rotor-lag pole — marginal, not
  the 2x-impossible Part 2 asserted. It fails at `K_y` = 8 anyway.

## Per-channel gains: no help

The drift lives in two of the three translation channels (measured in flight:
`|e_x|` 109 mm, `|e_z|` 117 mm, `|e_y|` **2.2 mm**), so raising only those two
targets it without paying gain elsewhere. Flown: `perchan8` [8,2,8]/[12,4,12]
aborts in 6.5 s and `perchan20` in 5.5 s — indistinguishable from the uniform
cells. The binding constraint is the loop frequency, not which channels carry it.

## M_y: the prediction, the flight, and the finding

Standard impedance practice says scale `M_y` with `K_y` to hold `wn` and
`zeta`. The algebra above agrees — authority is `∝ K_y/M_y` — so
`K_y` 20 / `D_y` 40 / `M_y` 10 is IDENTICAL to the shipped 2/4/1 in shape and
in static stiffness, and should reproduce the shipped cell's slow drift.

**It did not. `my10` aborted in 3.9 s, the fastest failure of the campaign,
with a 7.42 rad/s arm-torque mode** against the shipped cell's 0.23 rad/s drift
over 35–119 s. `my4` (M_y = 4): 4.7 s, 8.40 rad/s.

The reason is that **`M_y` appears twice**. Besides the impedance it sets the
observer feed-forward coefficient `(Lam_y M_y^-1 - I)`:

| M_y (xyz) | eigenvalues of `(Lam_y M_y^-1 - I)` | meaning |
|---|---|---|
| 1 | −0.60, −0.52, −0.34, **+0.44** | mixed sign, partial compensation |
| 4 | −0.88 … −0.60 | mostly full |
| 10 | −0.95 … −0.60 | ≈ full feed-forward of `F_hat_y` |

`F_hat_y` is not an external force — it is the observer's own estimate, built
from the same loop. Feeding ~100% of it forward closes a second loop, and it
rings at 7.4–8.4 rad/s, just under the rotor pole. **In this formulation the
desired task inertia and the disturbance-compensation gain are the same
number**, so `M_y` cannot be used as a pure impedance-shaping knob. That is a
structural remark about the law, not a tuning one.

## Failure-mode table (dominant joint-3 torque frequency in DIRECT)

| cell | K_y / D_y / M_y | wn, zeta | arm mode | abort |
|---|---|---|---|---|
| shipped | 2 / 4 / 1 | 1.41, 1.41 | — | 34.7 / 119.1 s |
| damping only | 2 / 12 / 1 | 1.41, 4.24 | **0.23 rad/s** (drift) | **81.9 s** |
| stiffer | 8 / 12 / 1 | 2.83, 2.12 | 5.67 rad/s | 7.6 s |
| stiffer | 8 / 24 / 1 | 2.83, 4.24 | 5.74 rad/s | 6.5 s |
| stiffer | 20 / 24 / 1 | 4.47, 2.68 | 3.87 rad/s | 7.7 s |
| per-channel | 8/2/8 | — | 5.41 rad/s | 6.5 s |
| per-channel | 20/2/20 | — | 5.81 rad/s | 5.5 s |
| **shape held** | 20 / 40 / **10** | **1.41, 1.41** | **7.42 rad/s** | **3.9 s** |
| middle | 20 / 20 / 4 | 2.24, 1.12 | 8.40 rad/s | 4.7 s |

Base attitude loop `sqrt(k_R/M_r_d)` = 4.14 rad/s; rotor-lag pole 10.03 rad/s.
`zeta` was never the problem — every cell was overdamped and the more damped
ones failed sooner.

## Revised conclusion

**17 faithful-law flights, zero completions.** Every path that raises the loop
frequency fails sooner than the soft shipped gains. Options, revised:

1. Keep the joint-space term and declare it (what ships).
2. **Separate `M_y`'s two roles** — give the disturbance feed-forward its own
   coefficient — then re-run this sweep. Until that is done, "raise K_y and M_y
   together" is untestable on this law, which is the single most useful thing
   Part 3 found.
3. Augment the task with a ninth output observing the fold (`beta = q2 + q3`),
   regulating the internal motion directly rather than hoping the EE task
   reaches it.

Part 2's option "project the posture term onto the weakest singular direction"
is WITHDRAWN: with the correct mapping there is no dominant weak direction to
project onto.

---

# Part 4: the minimum posture gain that flies

If the term has to stay, the useful question is how small it can be. Only
`kp = 2.0` had ever been flown. Matched plant, shipped impedance (2/4/1), only
`wb_posture_kp` varied — `kd` scaled as `sqrt(kp)` to hold the damping ratio,
`ki` and `i_max` scaled with `kp`.

| kp | kd | outcome | q3 range | arm clamped | max tilt | peak CoM err |
|---|---|---|---|---|---|---|
| 2.0 (shipped) | 0.25 | full mission | 28.3 … 43.0° | 0.0% | 4.0° | 337 mm |
| **0.5** | 0.125 | **full mission (2/2)** | 23.5 … 50.0° | 0.0% | 4.0° | 319 / 335 mm |
| 0.2 | 0.079 | abort 123 s | −85.2 … +50° | 4.9% | 33.8° | 891 mm |
| 0.05 | 0.040 | abort 77 s | −90 … +50° | 8.6% | 31.7° | 1965 mm |
| 0 (faithful) | 0 | abort 6/6 | −90 … +50° | 7–30% | 32–35° | 1141–1762 mm |

**The threshold is between 0.2 and 0.5, so the shipped 2.0 is 4x more than this
rig needs.** At 0.5 the flight quality is indistinguishable from 2.0 — zero
clamping, 4.0° peak tilt, the same peak CoM error — and it repeated 2/2.

Two honest caveats:

* **Margin, not just outcome.** At `kp` = 0.5 `q3` touches its +50° upper stop
  (max 50.0 vs 43.0 at `kp` = 2.0) and `q1` reaches −10° vs −3.5°. It flies,
  but with visibly less room. `kp` = 0.2 reaching leg 8 before diverging says
  the boundary is close.
* **The term does not shrink proportionally.** Measured over the mission
  (DIRECT + 20 s to the end), mean `|u3_posture|` is 0.0235 N·m at `kp` = 2.0
  and 0.0148 N·m at 0.5 — a 1.6x reduction for a 4x gain cut, because the mean
  arm tracking error grows 0.67° → 1.70°. As a share of the arm torque it
  actually commands, the deviation goes **9.6% → 6.1%**, not 4x smaller.
  (The 0.5% figure quoted elsewhere is a steady-hover number at 0.11° error;
  this is the mission-wide figure and it is the one to declare.)

Both failing gains fail the same way as `kp` = 0 — `q3` across zero into the
elbow-singular branch, `q1` railed — and `kp` = 0.2 diverged on **leg 8,
`traj_ee_back`**, an arm-moving leg, the same signature as `nopid_clean_b`.

## Tree state

The sim yaml is left at the measured minimum: `wb_posture_kp` **0.5**,
`kd` 0.125, `ki` 0.0125, `i_max` 0.2, with the requested 5% plant mismatch and
the shipped impedance 2/4/1. To go back:
`wb_posture_kp: 2.0 / kd 0.25 / ki 0.05 / i_max 0.8` for the shipped tune, or
all four to 0 for the faithful law (which does not fly).
