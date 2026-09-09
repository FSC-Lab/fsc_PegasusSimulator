# Removing the joint-posture term from the whole-body law — 2026-09-09

The whole-body impedance law as flown carries one term the manuscript
(`Aerial-Manipulator-Cartesian-Impedance-Control-…-v2/main.tex`) does not
have: a clamped joint-space PID on `u3` (Command.md 7.14.6, 7.15.7). Without it
the rig aborted every time at the full disturbance injection, on the GMO and on
the L1 observer alike. This note answers the three questions asked about that
term, with flights and an offline reproduction of the failure:

1. **Which disturbance makes the published law fail, and is it sensitivity or a
   design fault?** Two distinct mechanisms, both structural, neither a gain
   problem. Both are consequences of the manuscript modelling `d_e` as the
   *contact* wrench only.
2. **Can whole-body gain tuning fix it?** No. Every gain group of the law was
   swept offline (position, attitude, `M_r_d`, `K_y/D_y`, `M_y`, DLS,
   observer bandwidth, handover seeds); none survives the full injection
   without the term.
3. **What is the minimum modification that keeps a clean proof?** Two changes,
   neither a joint-space term: (a) anchor the free-flight end-effector
   reference to the CoM, and (b) let `u3`'s coupling feedforward carry the
   observer's estimate. Both leave `u3` structurally as published; the
   stability proof changes by a change of error variable and one exponentially
   decaying input. **Flown 3/3 at the full injection with the posture term
   removed** — the configuration that aborted 2/2 on 2026-09-06.

Data: `posture_ablation_20260909/` (runs, driver logs, `score.py`,
`run_ablation.sh`, `run_fix.sh`). Offline tool:
`application/robotic_arm/utils/wb_entry_sim.py`.

---

## 1. What the flown traces show

Every posture-off abort on file (`l1_final_20260906/l1_nopid_A`, yesterday's
`l1_seed_20260909/l1_seedA`, the 2026-09-05 GMO series) has the same first
five seconds. From the DIRECT edge, at the full injection (allocator kf +17.6%,
plant mass/inertia ×1.10, CoM shift 10/10/5 mm):

| t [s] | base error `e_x` | EE error `e_y` | arm | note |
|---|---|---|---|---|
| 0.5 | 25 mm | 16 mm | at home | observer restarts from zero |
| 1.0 | 115 mm (−114 z) | 100 mm | q3 on its **+50° stop** | EE reference already out of reach |
| 2.0 | 363 mm | 448 mm | q1 → −35° | |
| 3–5 | 0.74 → 1.31 m | 0.57 → 1.22 m | q3 crosses to **−40…−78°** | elbow-singular branch, all four joints on the 3 N·m clamp |
| 7.7 | abort | | | |

**The EE error tracks the base error one-to-one.** The world-fixed EE
reference sits where the arm cannot reach once the base has sagged 10 cm: the
gripper is 0.26 m from the body origin at the folded home and the arm's whole
workspace there is ~5 cm. The impedance law then drives the arm into its joint
stops, and from a stop the 4-DOF task (isolated, non-unique IK solutions)
crosses into the elbow-singular branch where the DLS solve rails the servos and
the reaction takes the base.

With the posture term ON the same 0.8 m transient happens (`l1_v2`,
`l1_seed_posture`: peak `e_x` 0.80 m, `e_y` 0.81 m) — but the arm never moves
(q stays at [0, 40, 40, 0] to ±3°). The 2 N·m/rad PID simply overrides a 2 N/m
task: the whole-body EE task was never satisfied through the entry, it was
being ignored. That is why `K_y` had to be softened to 2 in the same campaign
that added the PID (7.14.6): the two are a compensating pair for one
underlying problem.

## 2. Offline reproduction (`wb_entry_sim.py`)

Exact law (a copy of `controller.py`'s `MatlabController` with the hooks the
study needs), exact model, the Python L1 observer, the launcher's plant
injections (mass/inertia scale on the body, CoM shift in the actual frame,
plant-side or allocator-side kf), joint stops at the OM-X range, MN4010 rotor
lag, 16 ms transport delay, the 3 N·m clamp. The handover is modelled as it is
flown: the plant hovers level with its true weight and CoM moment already
compensated, and the DIRECT law starts with zero estimates.

It reproduces the flights it was built from and the ones flown today:

| case | flight | offline |
|---|---|---|
| full injection, PID on | 0.80–0.99 m entry transient, settles, `e_y` 6–33 mm | 0.99 m, settles, 33 mm |
| full injection, PID off | q3 +50° at 1 s, negative branch, abort 7.7 s | q3 +50° at 0.5 s, negative branch, abort 2.2 s |
| +5% model, PID off (today) | completed, arm parked at q = [−35, 11, 24, 30]°, `e_y` 214 mm | completed, q = [−35, 9, 26, 31]°, `e_y` 214 mm |
| fix (a)+(b), PID off (today) | peak `e_y` 31 mm, settles 1.2 mm, q = [0, 40, 41, −1]° | peak 38 mm, 1.3 mm, [0, 40, 41, −1]° |

It is a screening tool (the plant is the model with injections, not PhysX):
trust the ordering and the mechanism, fly the winner.

## 3. Q1 — which disturbance, and is it a design fault

### 3.1 Flights, posture term OFF, L1 observer, hover-only mission

`run_ablation.sh`: matched allocator (`alloc_thrust_coeff` = the plant's
4.041283e-05), disturbances added one at a time, thrust loss injected
**plant-side** (`PEGASUS_PLANT_KF_SCALE`, so SAFETY and DIRECT fly the same
plant). Motor delay (rotor lag λ = 10.03 1/s) is in every run.

| level | runs | vehicle | arm at the end | steady `e_y` | steady `d̂_z` |
|---|---|---|---|---|---|
| motor delay only | 3 | 3/3 completed | ~home (q2 40→44–50, q3 24–40) | 3–14 mm | 0.00 N |
| + 5% mass/inertia, CoM 5/5/2.5 mm | 2 | 2/2 completed | **q1 railed at −35°**, q = [−35, 11, 24, 30]° | **214–218 mm** | −1.84 N |
| + 5% thrust loss | 2 | **1 abort (19 s)**, 1 completed | negative branch, q down to −90° | 227 mm | −1.93 N |
| + 5% model + 5% thrust | 1 | completed | **elbow branch**, q = [−35, 50, −62, 33]°, clamped | 264 mm | −3.87 N |
| + 10% thrust, + full (15%) plant-side | 2 | **never took off** | — | — | — |

The last row is a SAFETY limitation, not a result: with the plant 10–15% short
on thrust the SAFETY gravity feedforward is short by the same amount and its
UDE is gated below 0.35 m — the bare-T650 deadlock of 7.13.3 run C. That is
why the fix flights below inject the thrust loss allocator-side (the 2026-09-06
configuration).

### 3.2 Two mechanisms, read off the flights

**Mechanism 1 — the entry transient makes the world-anchored EE reference
infeasible.** Needs a base excursion larger than the arm workspace (~5 cm).
At the full injection the unlearned 10.8 N deficit and 0.4 N·m CoM moment
produce 0.8–1.3 m; at 5% they produce 0.18–0.25 m and the arm is already at a
stop at t = 2 s (`model5_A`: q3 = 50° at 2 s).

**Mechanism 2 — the internal disturbance is rendered as a phantom contact
force on the EE, permanently.** The manuscript's `F_y = (J_y^#)^T d_e` is
"the generalized external force in the task coordinate" because `d_e` is
defined as `T^{-T} J_e^T w_e`, the contact wrench. On this plant `d_e` also
contains the thrust deficit and the CoM moment, and the closed loop
`M_y ë_y + D_y ė_y + K_y e_y = F_y` deflects the EE against them. At the
folded home `(J_y^#)^T` maps **0.22 N of task force per newton of vertical
deficit** (computed on the model; `J_1y e_3 = [0, 0.074, 0.208, 0]`). So:

| internal disturbance | phantom `F_y` | deflection at `K_y` = 2 N/m | measured |
|---|---|---|---|
| 5% model, `d̂_z` = −1.84 N | 0.41 N | 203 mm | **214 mm** (`model5_A`), 218 (`model5_B`) |
| 5% thrust, `d̂_z` = −1.93 N | 0.43 N | 213 mm | 227 mm (`kf5_B`) |
| full, `d̂_z` = −10.8 N | 2.4 N | 1.2 m — impossible, hence the stops | |

The L1 attribution does not remove this: it sets `F̂_y ≈ 0` (correct — no
contact), but the closed loop is `M_y ë + D ė + K e = F̂_y + M_y Λ_y^{-1}(F_y −
F̂_y)`, so the internal part still enters, scaled by `M_y Λ_y^{-1}`. With the
GMO's lumped estimate it enters unscaled. Either way the arm walks off home at
a rate set by `K_y`, reaches a stop, and the branch crossing follows — which is
exactly the slow 20–60 s drift the 5% flights show (`kf5_A` reached q2 = −33°
at 15 s and aborted at 19 s with the base still within 5 cm).

**Verdict.** Not sensitivity: the law is exponentially stable exactly as
proven, for the system it models. Both failures are the manuscript's
free-flight assumptions being violated by the rig — `d_e ≡ 0` in free flight
(3.1's C1) and initial errors inside the non-singular workspace (C3, C5) — and
neither can be restored by a gain. The thrust mismatch (15%) and model
mismatch (10%) only set the size of the transient and of the phantom; at 5%
the vehicle survives but the arm is already off its branch. The motor delay is
not the trigger.

## 4. Q2 — whole-body gain tuning does not fix it

`wb_entry_sim.py --sweep`, posture off, full injection, one gain group at a
time from the shipped values (k_x 16, k_v 12, k_R 2, k_w 1.5, K_y 2, D_y 4,
M_y 1, DLS 0.3, ω_c 2/0.5/0.5):

| group | values tried | outcome |
|---|---|---|
| position k_x/k_v | 8/8, 24/15, 32/20, 48/28 | all abort (0.8–8 s); higher gains shrink the transient to 0.48 m and the arm still rails |
| attitude k_R/k_w | 1/1, 3/2, 4/2.5, 6/3.5 | all abort |
| EE K_y/D_y | 0.5/2 … 200/24 | all abort; stiffer is faster (0.8 s at 200) — the stiff task fights the base |
| task inertia M_y | 0.25 … 4 | all abort |
| DLS λ | 0, 0.6, 1.0, 1.5 | 0.6 "survives" with the arm on the negative branch (`e_y` 440 mm, 10% clamped) — not a flight |
| observer ω_c | 1/0.25/0.25 … 6/2/1 | all abort |
| handover seeds d_t, d_t + d_r | | all abort (mechanism 2 remains) |

Yesterday's in-law flight sweep (`l1_inlaw_20260909`: M_y, DLS, a_*, D_y,
K_y/D_y pairs, "natural" impedance — 17 flights) reached the same conclusion
on the rig: 3 completions out of 17, none repeatable.

The reason is structural: the position loop cannot be made fast enough to keep
the entry transient inside a 5 cm workspace (k_x = 48 still leaves 0.48 m,
against a 10 rad/s rotor pole), and no gain removes a steady phantom force —
only `K_y → ∞` would, and stiffening the world-anchored task is the worst case
(7.14.6 run E, 3.8 s).

## 5. Q3 — the minimal, provable modification

### 5.1 (a) Anchor the free-flight EE reference to the CoM

The manuscript defines `e_{x_E} = r_e − r_{e,d}` with `r_{e,d}` an arbitrary
desired trajectory. In free flight, choose it relative to the **actual** CoM:

    r̃_{e,d} = x_c + δ_d,   δ_d := r_{e,d} − x_{c,d}   (the planned EE-minus-CoM offset)
            = r_{e,d} + e_x,
    ṙ̃_{e,d} = ṙ_{e,d} + e_{v_x},   r̈̃_{e,d} = r̈_{e,d} + e_{a_x}

`e_{a_x} = ẍ_c − ẍ_{c,d}` is already available from the translational dynamics
(the manuscript's own remark computing `x_c^{(3)}` uses it); nothing is
differentiated. The task error becomes `ẽ_y = e_y − [e_x; 0]` and the control
law `u_3` (eq. u_3) is **unchanged** — only the reference it is given.

*Proof delta.* The matching condition gives `M_y ë̃_y + D_y ė̃_y + K_y ẽ_y = F_y`
with `ẽ_y` in place of `e_y`, the same ODE, so `V_3` in Theorem 1 is written on
`(ẽ_y, ẽ_{v_y})` verbatim and `(e_x, e_{v_x}, e_R, e_ω, ẽ_y, ẽ_{v_y}) → 0`
exponentially exactly as before. Since `e_y = ẽ_y + [e_x; 0]`, `e_y → 0`
exponentially as well. What changes is the region of attraction (C5): the bound
on `e_y(0)` no longer has to be smaller than the arm workspace, because the
task the arm is given is reachable for every base error — the workspace
condition C3 becomes a condition on `ẽ_y` alone. For the interaction phase
(Theorem 2) keep the world anchor: with the CoM anchor the passivity port
becomes `(F_y, s̃)` and the extra term `[e_{v_x} + c_3 e_x; 0]^T F_y` is of the
same class as the estimation-error perturbation `c_e ē_d ‖z‖` — provable, but
weaker than the published OSP. The anchor is a reference-generation choice
the planner already knows the phase of.

### 5.2 (b) Let `u_3` pre-compensate the base wrench the observer knows

Split `d_e = d_int + d_ext` (internal: thrust deficit, CoM moment, model
error; external: the contact wrench). The EE Cartesian dynamics are

    Λ_y e_{a_y} = (J_y^†)^T (u + d_int − C̃ξ − g̃) + F_y^ext + Λ_y (J̇_y ξ − ÿ_d)

so the base wrench `u_3` must pre-compensate is `u + d_int`, not `u`. With the
observer estimate `d̂_int` (the lumped filtered estimate in free flight; the
L1's internal share `ŵ` when contact is possible), `u_3` becomes

    u_3 = −J̄_{3,y}^{-1} ( [J̄_{1,y} J̄_{2,y}] [u_1 − g̃_t + d̂_t ; u_2 − C̃_r ω_0 − C̃_{rρ} ρ + d̂_r]
                          + Λ_y (J̇_y ξ − ÿ_d) + Λ_y M_y^{-1} (D_y e_{v_y} + K_y e_y)
                          − (Λ_y M_y^{-1} − I) F̂_y^ext ) − C̃_{rρ}^T ω_0 + C̃_ρ ρ − d̂_ρ

i.e. the published `u_3` plus `−J̄_{3,y}^{-1} (J_y^#)^T d̂_int`. Nothing else
moves. The closed loop is then

    M_y ë_y + D_y ė_y + K_y e_y = F̂_y^ext + M_y Λ_y^{-1} [ (F_y^int − F̂_y^int) + (F_y^ext − F̂_y^ext) ]

**Why the term has three blocks, and why the arm block is the least of them.** The
addition is the single term `−J̄_3y⁻¹ (J_y^†)ᵀ d̂`; it appears as three because
`(J_y^†)ᵀ = [J̄_1y J̄_2y J̄_3y]`. The two base blocks are not a second copy of the
`−d̂_t`, `−d̂_r` already in `f_d` and `u_2`: those sit in the command and `d` sits in
the plant, and `u_3` must pre-compensate their sum, the arm being bolted to a base
that a thrust deficit accelerates. Splitting the flown settled estimate at the home
pose gives 2.380 N from `J̄_1y d̂_t`, 0.498 N from `J̄_2y d̂_r` and 0.653 N from
`J̄_3y d̂_ρ`, total 2.127 N — the arm block partly opposes the base blocks — so:

| blocks the feedforward carries | settled EE error, offline |
|---|---|
| none (anchor only) | 102.7 mm (flown: 102.7) |
| `d̂_ρ` only | **148.3 mm — worse than none** |
| `d̂_t`, `d̂_r` only | 36.3 mm |
| all three | **1.3 mm** (flown: 1.2) |

*Proof delta.* In free flight `F_y^ext ≡ 0` and the only input is the observer
error `F_y^int − F̂_y^int = (J_y^#)^T d̃_e`, which the manuscript already bounds
(eq. gmo_iss_bound: exponentially decaying for a constant `d_int`, ultimately
bounded by `d̄/λ_min(K_o)` otherwise). Theorem 1's cascade argument goes
through with that exponentially decaying input; Theorem 2's perturbation
constant `c_e` gains the same `L_Λ L_J` factor it already has. This is the
same treatment the translational and attitude channels already receive from
`−d̂_t` in `f_d` and `−d̂_r` in `u_2` — the arm channel was the only one left
out.

### 5.3 (c) A joint-space potential, if a branch selector is still wanted

If a joint-space term is kept, the provable form is a potential rendered
through the task, not a torque added to `u_3`:

    K_y e_y  →  K_y e_y + J_{3,y}^{-T} K_q (q − q_d),   V_3 += ½ (q − q_d)^T K_q (q − q_d)

On the non-singular workspace the map `q ↦ y` is a local diffeomorphism, so
`U_q` is a potential in `y` and its time derivative cancels the new term in
`V̇_3` up to cross terms `−e_q^T K_q [J_{3,y}^{-1} J_{1,y} e_{v_x} + (J_{3,y}^{-1}
J_{2,y} + N_1) e_ω]`, the same kind the theorem already absorbs with the `c_i`
and `W` blocks. The flown PID is this term with a different metric (it adds
`K_p e_q` to `u_3` directly, i.e. `J̄_{3,y} K_p` in the task); it works for the
same reason, but its equilibrium is biased whenever the planner's `q_d` and the
world-anchored `y_d` disagree — which they do through every transient.
Offline, (c) alone at `K_q` = 2–8 N·m/rad reproduces the PID's behaviour (arm
rides the base, `e_y` 20–90 mm steady); it is not needed once (a)+(b) are in.

### 5.4 Offline: which combination, at which stiffness

`wb_entry_sim.py --fixes` and follow-ups, posture off, full injection:

| configuration | result |
|---|---|
| published law | abort 2.2 s |
| (a) only, K_y 2 | arm still rails (relative `e_y` 538 mm peak): with K_y = 2 the acceleration feedforward error alone moves the arm |
| (a) only, K_y 20/12 | flies, **steady `e_y` 103 mm** = the 2.4 N phantom / 20 N/m; K_y 200 → 10.5 mm |
| (b) only, K_y 2 | survives, 20% clamped, arm at the stops |
| **(a)+(b), K_y 8/8** | flies, peak `e_y` 61 mm, **settled 3.1 mm**, q → [0, 39, 42, −1]° |
| **(a)+(b), K_y 20/12** | flies, peak 38 mm, **settled 1.3 mm**, q → [0, 40, 41, −1]°, 0% clamp |
| (a)+(b), K_y 50/20 | peak 19 mm, settled 0.5 mm |
| (a)+(b), K_y 20/12, GMO observer | flies, settled 34 mm (the GMO's slower, lumped estimate) |
| (a)+(b), K_y 20/12 at every ablation level | 0.0–0.5 mm settled, arm at home, 0% clamp |
| (a)+(b), kf 0.85 + mass 1.15 + CoM 15 mm | abort at 1.8 s — the **base** (e_x 1.5 m, arm fine at 44 mm) |

The last row is the honest limit: beyond ~15% thrust and ~10% mass the entry
transient itself exceeds the driver's 1.5 m envelope, an SAFETY→DIRECT
handover problem (7.15.8: seed the L1 from what SAFETY knows), not an arm
problem.

### 5.5 Flown

Two feedforward sources were flown. `wb_u3_internal_ff` alone feeds the **filtered
lumped** estimate `d̂_Σ,f`; `wb_u3_internal_ff_use_w_hat` switches it to the design
note's **internal** estimate `d̂ = T⁻ᵀ ŵ` (its Step 2), which is the form eq. (u3_int)
of the note states. In free flight `d_e = 0`, so the lumped estimate *is* `d` and is
the better source there; the internal one is the only one admissible under contact.
Measured cost of the internal source: settled EE error 1.2 → 3.1–3.4 mm and peak arm
torque 0.8 → 1.3 N·m, with the arm dipping to q3 = 19° mid-transient against 38° for
the lumped source. That is `ŵ` being partial — with a stationary arm the note's
persistency condition holds with α = 0 and only four directions are identified — and
it works at all here because `L_c` books **99.3% of the residual to the collective**
(`ŵ_z` = −10.73 N of −10.81 N, measured in flight). Offline predicted 3.3 mm; flown
3.1 and 3.4.

`run_fix.sh`: full injection (mass/inertia ×1.10, CoM 10/10/5 mm plant-side,
kf +17.6% allocator-side — the 2026-09-06 configuration that aborted 2/2),
posture gains all 0, L1 observer, `wb_ee_anchor_com` + `wb_u3_internal_ff`,
`K_y` 20 / `D_y` 12 on x, y, z (heading row untouched at 0.3):

| run | vehicle | peak `e_x` | peak `e_y` | arm through the entry | settled `e_x` / `e_y` | clamp | peak τ |
|---|---|---|---|---|---|---|---|
| fix_ab_A | completed | 919 mm | **31 mm** | q within [30, 50]° | 1.9 / **1.2 mm** | 0.0% | 0.76 |
| fix_ab_B | completed | 901 mm | 29 mm | q within [31, 50]° | 2.5 / 1.2 mm | 0.0% | 0.85 |
| fix_ab_C | completed | 848 mm | 29 mm | q within [31, 50]° | 1.6 / 1.2 mm | 0.0% | 0.88 |
| fix_ab50_A (K_y 50/20) | completed | 858 mm | **15 mm** | q within [31, 50]° | 2.1 / **0.5 mm** | 0.0% | 0.80 |
| fix_a_A (anchor only, K_y 20) | completed | 798 mm | 114 mm | q1 to −13°, q3 to 10° | 2.7 / **102.7 mm** (offline said 102.7) | 0.0% | 0.84 |
| fix_b_A (feedforward only, K_y 2) | completed, arm lost | 903 mm | 880 mm | q1 railed −35°, q2/q3 on the +50° stops | 12.2 / 221.5 mm | **67.6%** | 3.00 |
| 2026-09-06, PID off, same plant | **abort 7.7 s / 8.0 s** | 1313 / 1408 | 1.2 m | negative branch | — | 32 / 47% | 3.00 |
| ffw_A / ffw_B — (a)+(b) with the note's internal estimate `T⁻ᵀ ŵ` | completed | 894 / 855 mm | 31 / 30 mm | q within [28, 50]° | 1.5–2.0 / **3.1–3.4 mm** | 0.0% | 1.34 / 1.28 |

Every row lands on its offline prediction (fix_a: 102.7 mm predicted, 102.7 flown;
fix_b: arm at the stops, clamped; fix_ab: 38/1.3 mm predicted, 29–31/1.2 flown).
The base transient is unchanged (0.8–0.9 m — it is the handover, see 7.15.8);
what changed is that the arm no longer cares: `e_y` stays under 4% of it, the
arm never approaches a stop, and it returns to exactly home with `u1`,
`d̂_z` = −10.81 N identical to the PID-on flights.

## 5.6 What the compensation buys — the isolated A/B

Four flights, same plant, same mission, same gains (`K_y` 20 / `D_y` 12), CoM
anchor on and `wb_posture_* = 0` in all of them. The only difference is whether
`u_3` carries `(J_y^†)ᵀ d̂`. Settled = mean over a 20 s window ending 5 s before
the SAFETY revert; rms over the whole 78 s of DIRECT.

| metric, DIRECT soak | compensation off | on (3 runs) | change |
|---|---|---|---|
| End-effector error, settled | **102.8 mm** | **1.2 mm** | 86x |
| End-effector error, rms | 101.2 mm | 5.2 – 5.7 mm | 18x |
| End-effector error, peak | 113.8 mm | 28.6 – 30.5 mm | 3.8x |
| Arm offset from home, settled | **22.6 deg** | **1.1 – 1.3 deg** | 18x |
| Settled arm pose [q1 q2 q3 q4] | −5.3, 17.4, 37.6, 1.2 | 0.1, 39.6, 40.7, −1.2 | home = 0, 40, 40, 0 |
| Arm torque, settled / peak | 0.65 / 0.84 N·m | 0.70 / 0.76–0.88 N·m | unchanged |
| Samples on the 3 N·m clamp | 0.0% | 0.0% | unchanged |
| CoM error, settled | 2.4 mm | 1.5 – 2.4 mm | unchanged |
| Tilt, peak | 8.35 deg | 8.5 – 10.3 deg | unchanged |

Two readings beyond the headline. The term acts on the **arm**, not the
vehicle: base error and tilt are identical with and without it, because the
platform loops were never the ones missing a cancellation. And it is **free** —
the arm holds home on the same 0.7 N·m of gravity torque, so removing the
phantom force removes work rather than adding it.

### Stiffness, estimate source, and the anchor

| configuration | EE settled | EE rms | EE peak | arm from home | peak tau | clamp |
|---|---|---|---|---|---|---|
| off, `K_y` 20 | 102.8 mm | 101.2 | 113.8 | 22.6 deg | 0.84 | 0.0% |
| on, `K_y` 20, lumped | **1.2 mm** | 5.2–5.7 | 28.6–30.5 | 1.1–1.3 deg | 0.76–0.88 | 0.0% |
| on, `K_y` 50, lumped | **0.5 mm** | 2.3 | 15.4 | 1.3 deg | 0.80 | 0.0% |
| on, `K_y` 20, internal `T⁻ᵀ ŵ` | 3.0 – 3.5 mm | 7.3–7.5 | 29.5–31.5 | 2.0–2.3 deg | 1.28–1.34 | 0.0% |
| on, `K_y` 2, **no CoM anchor** | 224.2 mm | 318.3 | 880.3 | 35.6 deg (all four at their stops) | 3.00 | 66.9% |

`K_y` 20 → 50 halves what is left, which is residual observer error rather than
phantom force. The internal estimate costs ~2.5x in settled error and ~1.6x in
peak torque, the price of `ŵ` being partial. The last row is the reminder that
this term fixes one of the two mechanisms: with the world-anchored reference
still in place, compensating perfectly does not stop the arm reaching its stops.

Metrics regenerated by `posture_ablation_20260909/compensation_metrics.json`.

## 5.7 Mismatch grid — how far the compensation carries

Nine flights, compensation on (`wb_ee_anchor_com` + `wb_u3_internal_ff`, lumped
source), posture PID off, `K_y` 20 / `D_y` 12, L1 observer with attribution.
THRUST mismatch is injected ALLOCATOR-side (plant-side above ~5% cannot take
off: SAFETY's gravity feedforward is short by the same amount and its UDE is
gated below 0.35 m, the 7.13.3 run C deadlock); MODEL mismatch is plant-side,
mass and inertia x(1+y) with a proportional CoM shift.

| thrust | model | runs | `d̂_z` | settled `e_y` | rms `e_y` | peak `e_y` | peak `e_x` | arm from home | peak tau | clamp |
|---|---|---|---|---|---|---|---|---|---|---|
| 10% | 5% | 2/2 | −6.12 N | 0.7, 0.8 mm | 2.8 | 18 mm | 345 mm | 0.5° | 0.75 | 0.0% |
| 15% | 5% | 2/2 | −8.65 N | 0.9, 0.8 mm | 3.8–4.0 | 24 mm | 437 mm | 0.5–0.6° | 0.75 | 0.0% |
| 10% | 10% | 2/2 | −8.17 N | 1.0, 1.0 mm | 4.4–4.5 | 24 mm | 723 mm | 1.0–1.1° | 0.76 | 0.0% |
| 15% | 10% | 3/3 | −10.81 N | 1.2 mm | 5.2–5.7 | 29–31 mm | 848–919 mm | 1.1–1.3° | 0.76–0.88 | 0.0% |

Every cell completed, every repeat agrees to a decimal, zero saturation
anywhere, and the arm ends at home in all nine. The settled EE error is
**linear in the disturbance magnitude at ~0.11 mm/N** (0.7 mm at 6.1 N to
1.2 mm at 10.8 N), which is residual observer error rather than phantom force —
the phantom is what the compensation removed. What grows fastest is the base
transient, 345 → 919 mm, and that is the SAFETY→DIRECT handover, untouched by
any of this (7.15.8).

For the same plant WITHOUT the compensation the arm does not stay on its
branch at all: at 5% model / no thrust error it parks 214 mm off with q1
railed (§3.1), and at 15%/10% the published law aborts in 7.7 s (§1).

Regenerate with `run_grid.sh`, score with `grid_table.py` (writes
`grid_metrics.json`).

## 5.8 Full-mission gate, and the removal

Every compensation flight up to here was a hover soak. The 8-leg standard test
(x/y/yaw steps, then four compatible-trajectory legs where the arm actually
moves) was flown twice with the posture gains at zero and both options on,
`--hold-between 16`:

| run | DIRECT | legs | clamp | peak tau | peak `e_y` | q3 on the singular branch |
|---|---|---|---|---|---|---|
| mission_full_A | 314 s | 9/10 executed | **0.00%** | 0.79 N·m | 31.1 mm | **0.00 s** |
| mission_full_B | 276 s | **10/10** | **0.00%** | 0.77 N·m | 30.3 mm | **0.00 s** |

Joint travel, both runs: q1 −4.9…18.2, q2 30.4…50.0, q3 30.6…48.5, q4 −6.8…64.0 deg.
The arm moved through a real range with nothing holding it and never entered
the negative-q3 branch the term existed to prevent. q2 touched its +50 stop for
0.31 s in both runs, during the entry transient of the first step, and not
again in the remaining ~300 s.

Per-leg (run A): settled EE error 1.5–2.3 mm on the x/y steps and all four
trajectory legs, 9.4–14.0 mm on the yaw steps; peak base error 886 mm on the
first step (the entry transient), ≤233 mm thereafter.

**Run A's missing leg is a HARNESS race, not a control failure.** At t = 263.47
the planner went PENDING → CALCULATING → `PLANNED T=4.8s` inside one 10 ms
sample; the driver saw PLANNED and called `send`, but the second half of the
target had already restarted the solve, so the service answered
`success=False, "nothing to send (state CALCULATING)"` and the driver waited
out its 45 s exec timeout. The vehicle held the intermediate pose for 61 s at
1.5 mm EE error and 0% clamp. Run B, same configuration, executed all ten.
Fix belongs in `wb_l1_campaign_driver.py` (wait for PLANNED *after* both
targets are published), not in the law.

### The removal

On this evidence the five `wb_posture_*` keys were **deleted from
`params_..._whole_body_l1_direct_actuation_t650_sim.yaml`** (2026-09-09). They
are declared `kOptional` with default 0.0, so their absence makes the term
unreachable from that file rather than merely switched off, and the node's
green `LAW CHECK: joint-posture PID is OFF` line proves it every launch. The
block they occupied now carries the reasoning and a DO-NOT-RE-ADD note.

**Scope: that file only.** The C++ is untouched — the three GMO whole-body
yamls still carry `wb_posture_kp: 2.0` with `K_y` 2, none has flown with the
compensation, and the GMO path cannot adopt it as-is because there `F̂_y` is
the same lumped vector as `d̂` and the feedforward would double-count.

## 5.9 Entry-transient tune (2026-09-09) — `k_x` 16→32, `k_v` 12→20

The SAFETY→DIRECT transient was the largest remaining artifact: peak base error
848–919 mm, falling below 50 mm only after 18–19 s. Tuned with **no UDE seed**
(`wb_seed_disturbance_from_ude` stays false).

**Mechanism.** The peak error is mostly HORIZONTAL, not vertical: the vehicle
sags ~220 mm in the first 2 s, then the unlearned CoM moment tilts it and the
tilt converts thrust into sideways drift. So both the position loop and the
ROTATIONAL observer bandwidth are levers.

**Flights** (hover-only, 60 s soak — round 1's divergence only appeared at 8 s,
so a short soak would have passed it):

| configuration | peak `e_x` | <100 mm | <50 mm | peak tilt | outcome |
|---|---|---|---|---|---|
| shipped, `k_x` 16 / `k_v` 12, `omega_c_r` 0.5 (x3) | 848–919 mm | 12.4–13.2 s | 18.2–19.3 s | 8.5–10.3° | ok |
| `omega_c_r` **2.0** | 303 mm | — | — | 29.3° | **ABORT 15 s** |
| `omega_c_r` 1.0 | 545 mm | 11.4 s | 14.9 s | 7.0° | ok |
| `omega_c_r` 1.5 | 388 mm | 7.8 s | 12.1 s | 6.3° | ok |
| **`k_x` 32 / `k_v` 20**, observer untouched (x2) | **400 / 433 mm** | **6.4 / 5.5 s** | **11.5 / 10.7 s** | 6.5° | **ok** |

**`omega_c_r` 2.0 fails, and how it fails is the point.** The ENTRY was the best
of any candidate (303 mm) and then a slow rotational oscillation grew from
t = 8 s to a 36° abort at 15 s: |e_R| 0.04 → 0.19 → 0.31 → 0.38 with `d_hat_r`
swinging ±2 N·m. Command.md 7.15.8's warning about that gain STANDS — the
compensation did not lift the ceiling. **The offline screener's delay-margin
column is NOT a stability certificate for `omega_c_r`**: it gave that case 32 ms
of margin against a 16 ms nominal and the rig failed at nominal.

**Shipped: `k_x` 32 / `k_v` 20, observer left at `omega_c_r` 0.5.** It matches
`omega_c_r` 1.5 on peak and beats it on recovery, and it does not touch a gain
whose measured instability is only a factor 1.33 away. Position-loop natural
frequency goes 2.07 → 2.92 rad/s with damping 0.78 → 0.91, still far under the
10.03 rad/s rotor pole. Combining both levers was rejected: offline it is the
best of all (259 mm) but keeps only 24 ms of delay margin, and the screener has
just been shown unreliable there.

**Full-mission check** (8 legs, `--hold-between 16`), against the same mission
on the old gains:

| | `k_x` 32 / `k_v` 20 | shipped 16 / 12 |
|---|---|---|
| legs | **10/10** | 10/10 |
| peak `e_x` | **411 mm** | 900 mm |
| peak tilt | **6.3°** | 9.8° |
| peak / settled `e_y` | 30.4 / 1.18 mm | 30.3 / 1.14 mm |
| clamp | 0.00% | 0.00% |
| q3 on the singular branch | 0.00 s | 0.00 s |

End-effector behaviour is unchanged; only the base transient improves. Note the
`<50 mm` column is meaningless on a full mission — the base legitimately leaves
that band during the steps — so it is a hover-only metric.

**Not addressed:** the transient is halved, not removed. Seeding the observer
from the SAFETY UDE at the mode switch is the structural fix and is still
untried with the compensation in place.

## 6. Implementation

Both options are OFF by default and byte-identical to the previous law when
off (`WbParityTest` 4/4, `WbL1ParityTest` 3/3, `WbReferenceBuilderTest` 2/2
before and after).

- `wb_controller.{hpp,cpp}` (fsc_autopilot_ros2, whole-body fork): `WbGains::
  ee_anchor_com`, `WbGains::u3_internal_ff`; the anchor shifts `r_ed`,
  `r_ed_dot`, `r_ed_ddot` by `e_x`, `e_vx`, `e_ax` (already formed in the
  translation section); the feedforward adds `J_1y d̂_t + J_2y d̂_r + J_3y d̂_ρ`
  to `coupling_ff`.
- client: `wb_ee_anchor_com`, `wb_u3_internal_ff` (optional, false), each
  printing a `LAW CHECK` line when on.
- `params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml`:
  both keys present, **false** — the shipped configuration is unchanged
  (posture PID on, K_y 2). Adopting the fix means: the two keys true,
  `wb_ky_x/y/z` 20, `wb_dy_x/y/z` 12, `wb_posture_*` 0.
- `application/robotic_arm/utils/wb_entry_sim.py`: the offline study
  (`--validate --ablate --sweep --fixes`, `--case k=v,...`).
- Not done: the Python reference `controller.py` does not carry the two hooks
  (they live in the C++ and in `wb_entry_sim.py`'s copy of the law), so the
  parity fixture only locks the options OFF. Add them to `ControlParams` and
  regenerate `wb_truth_t650.json` before the options are trusted on hardware.

## 7. What this does not cover

- One mission shape (hover soak). The steps and the compatible-trajectory leg
  of the standard test have not been flown with the options on.
- The interaction phase: the CoM anchor is a free-flight choice; the planner
  must switch it off (or blend it) before contact, and that switch is not
  implemented.
- Plant-side thrust loss above ~5% cannot be flown at all until the SAFETY
  takeoff deadlock (UDE height gate) is addressed.
- The GMO variant of the fix is offline only (settled 34 mm vs the L1's
  1.3 mm — the lumped, slower estimate leaves more phantom through the
  transient).
- **No contact, on any run** — which is the entire reason the internal estimate
  `T⁻ᵀ ŵ` is preferred over the lumped one. `06` applies no end-effector wrench
  (that block existed in `02` and was dropped from `03` onward), so `w_e ≡ 0`
  everywhere and the distinction between the two sources is a consequence of the
  note's eq. (channels), not a measurement. Closing it needs an EE-wrench
  injection in the plant, which does not exist on this rig.
- `controller.py` has no counterpart of either option, so the parity fixture locks
  them OFF only; the C++ terms are checked against `wb_entry_sim.py`'s copy of the
  law, not against the parity-locked Python reference.
- Three flights per configuration on a stack with run-to-run scatter (7.14.6).
