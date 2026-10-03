# Arm armature (J_arm) — where it enters the law, what the flights say, and whether fixing it + gains restores joint stiffness (2026-09-26)

## 1. Where J_arm enters

`J_arm = 353.5² × 1.6e-7 = 0.0200 kg·m²` (XM430-W350 ratio × a guessed rotor inertia, carried over
from the MATLAB model) is added as `J_arm·h hᵀ` to each CHILD LINK's body inertia:
`controller.make_params()` (utils_controller/controller.py:235-239) and `wb_model.cpp:135-140`
(parity-locked). It is a model constant, not a gain, and reaches every inertia-dependent quantity
`dynamics()` builds each tick:

| law term | how J_arm enters | effect at q = [0, 25, 30, 0]° (flown → joint-diagonal 0.020) |
|---|---|---|
| **u3** (arm): K_q = M̃_ρ J₃⁻¹(M_y⁻¹K_y)J₃, D_q = M̃_ρ J₃⁻¹(M_y⁻¹D_y)J₃ via J̄₃ᵧ = Λ_y J₃ M̃_ρ⁻¹ | ~75 % of the j2/j3 block of M̃_ρ | j2/j3 stiffness eigenvalues 0.20 / 1.35 → 0.45 / 0.71 N·m/rad at K_y 20 |
| **τ_body = u2 + u1·r̂0c e3 + N1ᵀu3** (the arm reaction the rotors pre-compensate) | N1 = M̃_ρ⁻¹B | N1 base-x row on j2 1.14 → 0.45, j3 −0.20 → +0.06 |
| **u2** (attitude): M_r·(… − M_r_d⁻¹(k_R e_R + k_w e_w)) | M_r = Schur complement (arm free) | M_r diag 0.078/0.093/0.083 → 0.088/0.094/0.091 (the attitude gain M_r·M_r_d⁻¹ moves ≤ 14 %) |
| task inertia Λ_y, J_y^#, feed-forward Λ_y(J̇_y ξ − ÿ_d), coupling_ff | through M̃ | Λ_y diag 0.47/1.57/0.65/0.020 → 0.47/1.93/0.40/0.016 |
| observers (momentum p = M̃ξ) | an inertia error × acceleration is booked as disturbance | arm channel at 0.5 rad/s — too slow for stick-slip bursts |
| planner (fsc_trajectory_planner links wb_law) | joint-torque feasibility + reference torques | compatible CoM solve is mass-only, unaffected |
| NOT affected | gravity, u1/f_d, CoM kinematics | — |

Physics says the armature is JOINT-DIAGONAL (the rotor turns at N·q̇ relative to its own housing;
the cross terms scale with N·J_r = 5.7e-5 kg·m², negligible). The Isaac plant (06) already authors it
that way (PhysX joint armature 0.020), so every sim flight has flown a structure mismatch. For the
parallel j2/j3 axes the modelled `h hᵀ`-on-link becomes `J·[[2,1],[1,1]]` — the source of the 6.7:1
stiffness anisotropy.

If J_model ≠ J_true: the STATIC stiffness is set by the model (τ = −K_q e with K_q ∝ J_model), while
the loop BANDWIDTH scales as √(J_model/J_true) — an over-estimated J makes the arm loop faster than
designed (delay-unstable), an under-estimated one sluggish and soft.

## 2. What the flights say (tools in `../wb_l1_4d_flight_20260924/tools/armature_id.py`, `armature_structure.py`; outputs in `../wb_l1_4d_flight_20260924/analysis/armature_*.txt`)

Regression per joint: `τ_applied (Present Current/κτ) − τ_links(q, q̈, base ω̇, specific force)` =
armature + friction, on moving samples, q̈ from the encoder through a zero-phase low-pass.

- **Only flight 4 (6 s period) excites the armature** (|q̈| p95 3–4 rad/s²): diagonal J = 0.009–0.022
  (j2) and 0.008–0.012 (j3) depending on the filter band; a single J fitted to both rows: **0.017**.
- Flights 1–3: |q̈| p95 ≤ 2 rad/s² → armature torque ≤ 0.04 N·m, buried in friction scatter
  (residual 50–140 mN·m). Estimates from −0.016 to +0.008, uninformative.
- Structure: the flights cannot discriminate (flight 4 fits the `h hᵀ` form marginally better, at
  J 0.011 — within friction-model error). Physics decides it.
- Indirect bound: the sim below says the flown configuration would LIMIT-CYCLE if J_true were 0.012.
  It did not on hardware → consistent with J_true ≳ 0.02 (given the modelled velocity lag and friction).
- **Conclusion: 0.02 is the right order (±50 %); the structure is what is wrong.** Measure it on the
  bench: arm props-off/fixed base, current-mode multi-sine or chirp (0.5–5 Hz) on j1 (vertical axis, no
  gravity) and j2/j3, fit τ = (M_links + J)q̈ + friction; the armature is the ω²-growing part of the
  current→position response. Or ask Robotis for the XM430-W350 rotor + gear-train inertia.

## 3. Preliminary simulation (`arm_stiffness_sim.py`, offline — NOT Isaac)

Exact law (wb_entry_sim.Law), 4-D hardware config (K_y 20/D_y 12, M_y 1/1/1/0.05, 4-D L1 ω_x 0.25,
anchor-CoM, internal FF), rotor lag, 16 ms transport; plant armature joint-diagonal; **stick-slip
friction** (static = the yaml fc + µ|τ_g|, kinetic 0.85×/0.65× on j2/j3 = the flight-identified
ratios) + the flown reference-velocity friction FF + servo caps; the law's q̇ = **Present Velocity,
48 ms late, 0.024 rad/s quantum**; reference = flight 3's arm design (fold 55°, q2 = 25 ± 15°, 12 s)
with the CoM held. The diagonal-model cases rescale M_r_d with M_r so the attitude loop's gain is
unchanged. Model hook: `controller.dynamics()` optional `params["armature_diag"]` (absent = bit-identical,
verified; consistent to 1e-11 with it).

**Validation**: the flown config gives j2/j3 5.7/9.3° rms, j3 overshoot +9.5° (hardware F3: 5.0/8.3°,
7–12°); without friction 0.1°. The hold offsets are PESSIMISTIC (sim 5–11° vs hardware 1.8–3°) —
trust the ordering.

Sine, j2/j3 error rms [°], plant J 0.020 (0.012 / 0.030 where run):

| law | Present Velocity (48 ms) | 12 ms observer | holds, max offset j2/j3 |
|---|---|---|---|
| flown: `h hᵀ`, K_y 20/12 | 5.7/9.3 (0.012: limit cycle, 18° tilt; 0.030: 5.6/9.2) | 5.5/9.1 | 10.6/11.2 |
| flown, K_y 50/20 | **limit cycle** (caps 59 %) | — (no lag: 2.3/3.7) | — |
| flown + joint PD 2/0.25 | 0.8/1.0 (0.012: limit cycle) | — | 0.7/1.2 |
| flown + joint PD 10/0.5 | limit cycle | — | — |
| diag, K_y 20/12 | 2.5/3.9 (0.012: 2.5/3.9) | — | 4.7/5.8 |
| diag, K_y 50/20 | **1.1/1.6** (0.012: 1.2/1.7; 0.030: 1.0/1.7) | — | 1.3/2.2 |
| diag, K_y 100/28 | 0.7/0.9 (0.012: limit cycle, unless model J = 0.012: 0.9/1.3; 0.030: 0.6/0.8) | 0.5/0.8 at 0.012 | 0.2/0.7 |
| diag, K_y 200/40 | limit cycle (caps 92 %) | **0.3/0.4** (0.012: 0.3/0.4) | 0.1/0.4 (observer) |

Base stays quiet in every stable diagonal case: tilt ≤ 0.2°, CoM ≤ 24 mm.

**Reading**: (1) the armature STRUCTURE alone halves the joint error and removes the stick-slip;
(2) with it, the task gain can be raised — K_y 50/20 is robust to J_true 0.012–0.030 and matches a
2 N·m/rad joint PD; (3) the ceiling is the law's **velocity signal**, not the vehicle: with Present
Velocity, K_y 100 needs J known to ~30 % and K_y 200 limit-cycles; with the arm's 12 ms position
observer (`velocity_observer_topic`, already logged in flight) K_y 200/40 tracks to 0.3/0.4° and
holds to ≤ 0.4°, robust to J 0.012; (4) raising gains WITHOUT the model fix is destabilising
(flown + K_y 50 limit-cycles). **Isaac cannot test this faithfully yet**: 06 hands the law exact PhysX
joint velocity and its friction is nearly cancelled by the FF, which is why K_y 50/20 flew there on
2026-09-09 with the flown model — emulate Present Velocity (lag + quantum) and stiction in 06 first.

Commands: `/usr/bin/python3 arm_stiffness_sim.py terms` · `… run [case names]` (~27 s per case).
Outputs: `terms.txt`, `runs.txt`, `holds.txt`, `sim_*.npz`.

## 4. Bench calibration of the law's J_arm, ground bags only (`bench/README.md`)

From the 2026-09-09 / 09-11 PD+ torque-mode calibration bags (no flight data), in the law's own
`J_arm·h hᵀ` structure: **J_arm = 0.0097 kg·m² (0.0077–0.0120 over 72 method variants), about half
the 0.020 in the law.** Per unit, in flight order J1..J4, it is 0.0086 / 0.0136 / 0.0084 / 0.0090. These differences are inside the
bands, so one scalar is what the data support. The data lean toward the law's `h hᵀ` structure (χ² 15/8
vs 27/8 for joint-diagonal), but that rests on the j2 row alone, whose fast-move estimates disagree.
It does not settle §1's physical argument. §2's flight-4 `h hᵀ` fit (J 0.011) agrees. Not applied to the law.

**ADOPTED 2026-09-26** (bench/README.md "ADOPTED"): joint-diagonal bench armature in the law
+ planner (parity-locked), the law's q̇ from the arm's velocity observer, K_y/D_y 80/24,
M_r_d rescaled; Isaac A/B on the same plant j2/j3 2.29/4.85 → 1.07/1.39°, EE 20.8 → 10.3 mm,
base unchanged. NOT on hardware yet.
