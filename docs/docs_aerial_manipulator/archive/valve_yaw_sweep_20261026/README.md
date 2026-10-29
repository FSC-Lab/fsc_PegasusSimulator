# Valve turning: heading-channel sweep (2026-10-26)

The user's plan: make the 4th (heading / torque) channel of the end-effector impedance show clearly in the
valve-turning task. The task becomes a 90° turn in 12 s, with a stiffer heading impedance and more valve
friction. The sweep maps bench-measured friction to (K_ψ, D_ψ) and gives the safe friction band and a
target friction for cutting the rubber pads. Whole-body 4-D L1, Isaac headless at RTF 1, raw mocap.
24 flights. Simulation only.

**Verdict: no cell meets the plan's selection rule.** No (τ_f, K_ψ) is both safe and shows the heading
couple. The measured couple never reached the 0.07 N·m visibility level on the plan's "M̄ ≥ 0.07 N·m and
≥ 2σ" test. Raising K_ψ above 1 makes the turn unsafe even at the lowest friction. The cause is the grasp:
in the simulation the claw twists on the fin, so the couple stays at the grip's level whatever K_ψ is.
There is therefore no lookup table and no target friction for cutting pads. Section 5 says what would
make the heading channel showable.

## 1. Setup

- **Config:** the shipped valve yaml (`valve_turning_20261015/tools/make_valve_turning_yaml.py`, the 10-22
  final config, free-axis anchor 0.75 on). Changes per run:
  - `push_pull_turn_deg` 90 and `push_pull_push_time_s` 12; the 1.5 s settle is kept.
  - `wb_ky_psi` = K_ψ and `wb_dy_psi` = D_ψ = 2.6 √(0.05 K_ψ) (ζ ≈ 1.3); `wb_my_psi` stays 0.05.
  - x / y / z unchanged.
- **Mirror plant:** keys, feedback and guards as flown: force 5 N, tilt 10°, the node's 15° watchdog.
- **Mission:** the driver's timetable slots before the turn. Release 17 s after the push step starts
  (settle + turn + 3.5 s on the fin). The valve's own exit, asked for by name (`--open-first-exit`): open
  with the claw world-held, Exit 3 s later. The run ends 2 s after the exit (`--stop-after-exit`, ported
  into the shared `pl_mission.py`, default off).
- **Valve:** Coulomb friction τ_f in the new stick-slip model (section 2). Load, ratchet and damping off.
- **Harness:** `tools/chain.sh` / `tools/chain2.sh` (a 4th spec field adds overrides); runs in
  `../valve_turning_20261015/runs/yw_*`.
- **Off-plan diagnostic:** a free-axis-anchor-off column, tagged `fa0` (section 4.2). It changes a law
  option, not the plant, the planner geometry or the guards.

## 2. Pre-checks

**Friction model (pre-check 1): the scene's default FAILED, and a stick-slip model was added.**
`tools/friction_check.py` runs the valve alone in a 250 Hz PhysX world: a constant torque of 0, ±0.9 and
±1.1 × τ_f for 6 s. Table: `analysis/friction_check.txt`.

| model | τ_f | 0.9 τ_f (must stay put) | 1.1 τ_f (must turn) | at rest |
|---|---|---|---|---|
| `tanh` (the default, every flight before today) | 0.10 | creeps 8.0 °/s | 3.85 rad/s² (Coulomb 3.89) | still |
| `tanh` | 0.25 | creeps 20.1 °/s | 9.61 (9.73) | still |
| `drive`, D 1000 N·m·s/rad (**used**) | 0.10 | 0.03° in 6 s | 3.84 (3.89) | still, no jitter |
| `drive`, D 1000 | 0.25 | 0.08° in 6 s | 9.57 (9.73) | still, no jitter |

- **Why `tanh` creeps:** it applies −f tanh(ω/0.05) explicitly, momentum-clamped to I|ω|/dt. Below f the
  rotor creeps at τ dt / I.
- **Why it cannot deliver large frictions:** the clamp caps the resistance at ~0.65 N·m per rad/s. At the
  turn's speeds, frictions above ~0.08 N·m never reached the valve.
- **The new model:** scene knob `PEGASUS_VALVE_FRICTION_MODEL=drive`, default `tanh`, so every earlier
  campaign is unchanged. The bearing joint's drive becomes a stiff velocity constraint whose force limit is
  τ_f (`PEGASUS_VALVE_STICTION_DAMPING`, default 1000).
- **Plumbing:** the valve wrapper forwards both knobs. Every run below flew `drive`.
- **In flight:** the axis balance confirms it. M + F_t r sits on τ_f whenever the valve slides (figures).

**Logging (pre-check 2): complete.** Every signal the plan asks for is in each recording:
- the contact truth on the rotor at 62 Hz, which gives the heading moment about the vertical at the claw
  with the force channels' sign rule;
- `wb_control_debug` at 250 Hz: e_y(4), the body torque (yaw = [23]), the four motor commands [41..44] and
  the task-force reading [97..99];
- q1–q4 at 250 Hz and Isaac's true attitude at 248 Hz;
- the gains, from each run's yaml copy.

The PhysX report's sign is fixed at +1, calibrated on `ref_f048` / `k1_f048`, where the mean power on the
rotor is 5–6 mW ≈ τ_f × the mean turn rate. The per-run power rule of the valve builder is reported but
not used: under the 1 Hz swing it flipped `k3_f048`'s sign.

**Reference (pre-check 3):** today's gains (K_ψ 0.2484, D_ψ 0.2903) at τ_f 0.048, flown twice. Run 1
completed at 7.2° peak tilt, failing the plan's 7° limit; run 2 completed and passed every check at 5.9°.
The plan expected 5–6° from the 360°/36 s runs, but the 90°/12 s turn has 2.25× their peak angular
acceleration. The relative checks (q1/q4 std, heading error) use the mean of the two reference runs.

## 3. Checks as scored (`tools/yaw_score.py`)

**Safety, as the plan defines it:**
- The mission completes through Exit, with no guard trip and no SAFETY handover.
- Tilt in the turn step ≤ 7°.
- No motor at its limit, and the yaw torque peak ≤ 70 % of the allocator's free yaw torque at the run's
  collective (4 k_m min(f_max − f_h, f_h − f_min) ≈ 0.69 N·m).
- Platform heading error ≤ 2× the reference's.
- q1 / q4 std ≤ 1.3× the reference's.
- Joint stop ≤ 10 % of the turn, and q3 ≥ 5°.
- Task force (the planner's guard reading, low-passed at 3 rad/s) ≤ 4 N; ε_UAV ≤ 80 mm.
- The beam turns ≥ 85° and rotates back ≤ 5° after the release.

**One deviation, the growth check.** The plan compares the swing's amplitude in the last third of the turn
with the first third. But the 1 Hz q1/q4 swing is forced by the turn and starts from rest, so that ratio
fails by construction: the reference scores 3.4–3.8. The verdict instead uses the last third against the
middle third (≤ 1.5) and the hold against the last third (≤ 1.2). Both ratios are in the table.

**Visibility,** over the middle 60 % of the turn: M̄ ≥ 0.07 N·m and M̄ ≥ 2σ_M.

**Rendering:** the mean moment against the rendered M_ψ ë + D_ψ ė + K_ψ e, the correlation over the turn,
the share M̄ / (M̄ + F̄_t r) against K_ψ / (K r² + K_ψ), and the beam's lag against τ_f / (K r² + K_ψ).

## 4. Results

`figures/map.png` (shipped config), `figures/map_anchor_off.png`, `figures/split.png`,
`figures/cell_*.png`. One row per run: `analysis/run_table.md` (every check) and `analysis/scores.json`.
The couple is reported resisting the turn as positive.

| run | anchor | K_ψ | D_ψ | τ_f | outcome | tilt max | yaw τ peak | q1/q4 std | q3 min | stop | force | ε max | turned | M̄ ± σ | rendered | corr | F̄_t r | M̄+F̄_t r | share meas/pred | failed checks |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| ref_f048 | 0.75 | 0.248 | 0.29 | 0.048 | complete | 7.2° | 63 % | 6.0/5.0° | 21.7° | 0 % | 1.6 N | 45 mm | 87.7° | -0.019 ± 0.078 | -0.000 | +0.04 | +0.040 | +0.022 | -0.88 / 0.07 | tilt>7 |
| ref_f048_2 | 0.75 | 0.248 | 0.29 | 0.048 | complete | 5.9° | 67 % | 5.7/3.5° | 23.8° | 0 % | 2.1 N | 78 mm | 90.8° | -0.117 ± 0.063 | -0.037 | -0.02 | +0.152 | +0.035 | -3.38 / 0.07 |  |
| k1_f048 | 0.75 | 1 | 0.581 | 0.048 | complete | 5.0° | 49 % | 4.4/3.6° | 22.5° | 0 % | 1.4 N | 36 mm | 87.2° | -0.026 ± 0.056 | -0.073 | -0.47 | +0.056 | +0.030 | -0.86 / 0.25 |  |
| k1_f050 | 0.75 | 1 | 0.581 | 0.050 | complete | 5.3° | 54 % | 5.5/2.8° | 18.7° | 0 % | 2.4 N | 58 mm | 89.4° | -0.131 ± 0.077 | -0.125 | +0.06 | +0.167 | +0.036 | -3.64 / 0.25 |  |
| k1_f100 | 0.75 | 1 | 0.581 | 0.100 | complete | 4.8° | 56 % | 5.8/4.6° | 9.7° | 8 % | 3.0 N | 117 mm | 83.4° | -0.130 ± 0.111 | -0.084 | +0.65 | +0.182 | +0.053 | -2.46 / 0.25 | q1 growing, q4 growing, eps>80mm, turned<85 |
| k1_f150 | 0.75 | 1 | 0.581 | 0.150 | complete | 4.0° | 58 % | 4.1/5.5° | 15.1° | 0 % | 2.5 N | 56 mm | 83.6° | -0.054 ± 0.077 | -0.077 | +0.44 | +0.167 | +0.114 | -0.47 / 0.25 | q4 growing, turned<85 |
| k1_f200 | 0.75 | 1 | 0.581 | 0.200 | complete | 5.2° | 58 % | 4.7/4.6° | 3.0° | 18 % | 2.8 N | 108 mm | 80.8° | -0.059 ± 0.141 | -0.045 | +0.67 | +0.213 | +0.155 | -0.38 / 0.25 | q4 growing, on stop>10%, q3<5, eps>80mm, turned<85 |
| k1_f250 | 0.75 | 1 | 0.581 | 0.250 | q3 guard (in the release) | 5.5° | 54 % | 5.0/6.4° | 10.4° | 0 % | 2.1 N | 63 mm | 90.6° | +0.025 ± 0.097 | +0.052 | +0.40 | +0.117 | +0.142 | +0.18 / 0.25 | — |
| k2_f048 | 0.75 | 2 | 0.822 | 0.048 | q3 guard (in the release) | 8.0° | 76 % | 7.6/6.6° | 7.3° | 17 % | 3.1 N | 138 mm | 83.0° | -0.187 ± 0.194 | -0.218 | +0.62 | +0.204 | +0.019 | -10.68 / 0.39 | — |
| k3_f048 | 0.75 | 3 | 1.01 | 0.048 | complete | 8.4° | 89 % | 8.9/9.6° | 4.2° | 0 % | 2.9 N | 75 mm | 97.1° | +0.090 ± 0.119 | +0.203 | +0.25 | -0.077 | +0.014 | +6.85 / 0.49 | tilt>7, yaw torque>70%, q1 std>1.3x ref, q4 std>1.3x ref, q3<5 |
| k3_f048_2 | 0.75 | 3 | 1.01 | 0.048 | q3 guard | 9.8° | 78 % | 7.2/7.3° | 1.4° | 0 % | 3.7 N | 95 mm | 76.8° | -0.040 ± 0.185 | +0.005 | +0.24 | +0.054 | +0.014 | -2.91 / 0.49 | — |
| k6_f048 | 0.75 | 6 | 1.42 | 0.048 | tilt guard | 13.1° | 84 % | 11.4/11.2° | 14.9° | 0 % | 4.1 N | 107 mm | 93.4° | -0.015 ± 0.182 | +0.144 | -0.04 | +0.026 | +0.012 | -1.36 / 0.66 | — |
| k6_f048_2 | 0.75 | 6 | 1.42 | 0.048 | tilt guard, ended INVERTED | 180° | 112 % | 14.7/18.5° | -26.1° | 0 % | 4.7 N | 93 mm | 3.4° | -0.068 ± 0.394 | +0.478 | -0.02 | +0.977 | +0.009 | — | — |
| k10_f048 | 0.75 | 10 | 1.84 | 0.048 | tilt guard + SAFETY handover | 26.2° | 76 % | 14.2/15.2° | 26.1° | 0 % | 3.9 N | 61 mm | -132.6° | +0.005 ± 0.036 | -0.298 | -0.14 | -0.182 | -0.015 | — | — |
| k10_f048_2 | 0.75 | 10 | 1.84 | 0.048 | tilt guard | 10.4° | 87 % | 6.5/7.5° | -4.0° | 0 % | 6.9 N | 106 mm | 0.1° | +0.000 ± 0.003 | -0.003 | +0.26 | +0.005 | +0.005 | — | — |
| fa0_ref_f048 | off | 0.248 | 0.29 | 0.048 | complete | 6.4° | 55 % | 5.3/4.3° | 16.3° | 6 % | 1.7 N | 53 mm | 89.6° | -0.041 ± 0.112 | -0.014 | +0.26 | +0.063 | +0.022 | -1.87 / 0.07 |  |
| fa0_k2_f048 | off | 2 | 0.822 | 0.048 | q3 guard (in the release) | 5.4° | 57 % | 5.2/5.3° | 16.0° | 0 % | 1.4 N | 48 mm | 90.2° | -0.007 ± 0.063 | -0.045 | -0.17 | +0.034 | +0.026 | -0.28 / 0.39 | — |
| fa0_k2_f200 | off | 2 | 0.822 | 0.200 | tilt guard (in the release) | 7.8° | 76 % | 5.5/5.9° | 4.5° | 7 % | 2.4 N | 44 mm | 88.0° | -0.033 ± 0.086 | -0.041 | +0.07 | +0.192 | +0.159 | -0.21 / 0.39 | — |
| fa0_k2_f250 | off | 2 | 0.822 | 0.250 | complete | 3.7° | 35 % | 4.4/5.2° | 19.9° | 7 % | 2.4 N | 52 mm | 87.0° | +0.000 ± 0.059 | -0.041 | +0.33 | +0.226 | +0.226 | +0.00 / 0.39 |  |
| fa0_k3_f048 | off | 3 | 1.01 | 0.048 | complete | 6.3° | 79 % | 4.9/5.0° | 16.0° | 0 % | 1.9 N | 47 mm | 92.3° | +0.011 ± 0.083 | +0.091 | -0.07 | +0.018 | +0.029 | +0.37 / 0.49 | yaw torque>70%, q4 growing |
| fa0_k3_f100 | off | 3 | 1.01 | 0.100 | q3 guard (arm buckled, q3 −19°) | 9.7° | 97 % | 6.0/6.8° | -18.8° | 0 % | 8.2 N | 68 mm | 23.5° | -0.019 ± 0.063 | +0.006 | -0.05 | +0.024 | +0.004 | — | — |
| fa0_k3_f150 | off | 3 | 1.01 | 0.150 | q3 guard, ended INVERTED | 7.1° | 164 % | 7.5/9.0° | -88.7° | 4 % | 225 N | 150 mm | 90.2° | -0.027 ± 0.058 | -0.152 | +0.21 | +0.155 | +0.128 | — | — |
| fa0_k3_f200 | off | 3 | 1.01 | 0.200 | tilt guard (in the release) | 9.3° | 97 % | 7.8/8.2° | 14.8° | 0 % | 2.3 N | 44 mm | 94.5° | +0.016 ± 0.109 | +0.064 | -0.35 | +0.103 | +0.119 | — | — |
| fa0_k3_f250 | off | 3 | 1.01 | 0.250 | complete | 6.3° | 62 % | 6.5/7.4° | 4.2° | 10 % | 2.6 N | 58 mm | 88.0° | +0.015 ± 0.054 | -0.067 | +0.38 | +0.206 | +0.221 | +0.07 / 0.49 | q4 std>1.3x ref, q4 growing, q3<5 |

The statistics of aborted runs cover the whole turn step, so they include the aftermath of the abort
(e.g. the 225 N contact force when `fa0_k3_f150` turned over).

### 4.1 The plan's stages

- **Stage A** (τ_f 0.048; K_ψ 1 / 3 / 6 / 10, failures repeated once):
  - K_ψ 1 passed.
  - K_ψ 3 failed twice: the first run completed but broke tilt, yaw and q3; the repeat hit the q3 guard
    at 1.4°.
  - K_ψ 6 failed twice: two tilt-guard aborts, the second ending inverted.
  - K_ψ 10 failed twice: the first run tripped the tilt guard with a SAFETY handover, the second the tilt
    guard.
  - So **Stage B is empty under the plan's rule**: no K_ψ in {3, 6, 10} goes forward.
- **Stage B, extended** to the one gain that passed, K_ψ 1, shipped config:
  - τ_f 0.05 passes every check.
  - τ_f 0.10–0.20 complete but fail the valve's checks: the beam turns 81–84°, short of 85°. The free-axis
    anchor lets it stop short with the base.
  - Also failed: ε up to 117 mm, the swing still ringing in the hold, and at 0.20 q3 dips to 3° with the
    arm on a stop 18 % of the turn.
  - τ_f 0.25: the q3 guard tripped in the release window.
  - K_ψ 2 at 0.048 tripped the same guard in the release window.
- **Visibility:** never met in any of the 24 runs.
  - The best mean couple resisting the turn is +0.09 N·m (`k3_f048`, σ 0.12), in a run that broke three
    safety checks.
  - In every other run |M̄| ≤ 0.13 N·m, and with the anchor on M̄ is mostly NEGATIVE: the couple helps the
    turn.
- **Stage C** (confirmation of the chosen cells): not flown, since nothing was chosen.

### 4.2 Why: the couple is set by the grasp, not by K_ψ

1. **The valve's load is the SUM about the axis, and the sim gets it right.** M̄ + F̄_t r = 0.01–0.23 N·m
   tracks τ_f while the valve slides (`figures/split.png`). It falls short while the valve sticks or the
   turn stops short.
   - The plan's quasi-static split, share = K_ψ / (K r² + K_ψ), predicts how that sum divides between the
     two channels.
   - The measured share never follows it: −10.7 … +6.9 against a predicted 0.07–0.76.
2. **The claw twists on the fin.** Measured as Isaac's true base yaw + q1 minus the beam's yaw
   (`tools/grip_slip.py`; the claw's FK heading gives the same result):
   - The claw's azimuth relative to the beam moves through 10–45° in every turn, std 3–15°.
   - A rigid grasp would hold it constant.
   - Effective torsional stiffness about the vertical (couple per radian of twist,
     `tools/grip_stiffness.py`): 0.1–0.8 N·m/rad, with weak correlation (stick-slip in character). That is
     below every K_ψ tried and below K r² = 3.07 N·m/rad.
   - The sim claw clamps the fin's 20 mm grip shim between near-point finger contacts, which act almost as
     a hinge about the vertical.
   - So the heading channel mostly drives the claw's twist on the fin, and the couple stays at the grip's
     level, about ±0.1–0.2 N·m.
3. **Raising K_ψ excites that twist and the platform's yaw.** It is the ~1 Hz q1/q4 swing the 10-21 load
   tests saw:
   - frequency 0.92 Hz (K_ψ ≤ 1), 1.0 (3), 1.08 (6), 1.17–1.23 (10);
   - claw twist std grows 3 → 15°;
   - the commanded yaw torque rises to 78–112 % of the free yaw budget.
   - K_ψ 1 is slightly BETTER than today's 0.25 (tilt 5.0 vs 5.9–7.2°, yaw torque 49 vs 63–67 %), on one
     run each.
4. **The free-axis anchor (0.75, shipped 10-22) adds a second offset.**
   - It moves the claw's TRANSLATIONAL target with the base's error along the nose (the fin's tangent),
     but not its HEADING target.
   - With the anchor on, the tangential force over-drives the valve: F̄_t r 0.04–0.21 N·m, often more than
     τ_f. The heading spring then holds the claw back, so M̄ is negative.
   - With the anchor off, the couple averages about zero (+0.000…+0.016 at τ_f 0.25) and F_t r carries
     the load.
   - Turning the anchor off was not safer for K_ψ ≥ 2: 2 of 7 completed, and one ended inverted.

## 5. Deliverables against the plan, and what would work

- **Lookup table / safe band / target friction:** none can be given.
  - The heading couple is not made visible by any safe (τ_f, K_ψ).
  - The shipped config is safe only at K_ψ ≤ 1 and, at K_ψ 1, only at τ_f ≈ 0.05 N·m on every check. It
    completes up to 0.20 with check failures and aborts at 0.25.
  - **Do not cut the pads for heading visibility yet:** more friction only moves load into the
    tangential force.
- **To show the heading channel, the grasp must hold a couple first.**
  - Bench the real gripper on the fin (or the handle that replaces it): torque about the vertical against
    twist angle. It needs to be well above K_ψ, i.e. several N·m/rad.
  - A pin-in-slot, a wider two-point fin, or a keyed handle would do it.
- **Better still, move the grasp onto the axis** (a knob or T-handle at r ≈ 0). Then F_t r → 0 and the
  whole valve load must pass through the couple: share → 1 by the plan's own formula. It still needs a
  torsionally stiff grasp.
- **Keep K_ψ ≤ 1 on this rig.** K_ψ ≥ 2 aborted at the lowest friction.
- **Any future friction study:** use `PEGASUS_VALVE_FRICTION_MODEL=drive`. The `tanh` model cannot hold a
  rotor and caps the resistance during a turn.

## 6. Limits

- Most cells flew once. This rig's run-to-run scatter is wide: the two reference runs differ by 1.3° in
  peak tilt and by 0.10 N·m in M̄.
- No drift model (the 10-22 hardware-like force noise) and no hardware.
- The free-axis-off column is a diagnostic: it changes a shipped law option and the plan did not ask for
  it.
- The grasp's twist stiffness is the simulation's; the real foam pads on the fin are unmeasured.

## Files

- `tools/friction_check.py` — pre-check 1, run with `$ISAAC_PY`.
- `tools/chain.sh`, `tools/chain2.sh` — the flight chains.
- `tools/yaw_score.py` — every check; writes `analysis/scores.json` and `scores.txt`.
- `tools/quicklook.py` — the turn's signals for one or more runs.
- `tools/grip_slip.py`, `tools/grip_stiffness.py` — the claw's twist on the fin.
- `tools/make_figures.py` — the maps, the split, the per-run figures and the run table.
- `tools/export_matlab.py` — the valve run-struct export, phases ending at Exit.
  - `matlab/<run>.mat`: the reference `ref_f048_2`, the two safe K_ψ 1 runs `k1_f050` and (best rendering)
    `k1_f200`, the onset of the instability `k3_f048`, and the anchor-off safe run at τ_f 0.25
    `fa0_k2_f250`. These are representative runs, not chosen cells.
  - Each carries `impedance.F_ext / rendered / e_imp / e_y` (column 4 = heading), `heading` (couple,
    rendered, F_t r, yaw torque, q, tilt) and `meta.sweep` (the knobs and every check).
- `report.html` — the short report page, published as https://claude.ai/artifact/CGsJryjkz1RNWy5LyYaC6q
  (private).
- Changes outside this folder:
  - `application/robotic_arm/07_px4_t650_aerial_manipulator_valve_turning.py`: `PEGASUS_VALVE_FRICTION_MODEL`,
    `PEGASUS_VALVE_STICTION_DAMPING`.
  - The valve wrapper: forwards them.
  - `push_pull_20261003/tools/pl_mission.py`: `--stop-after-exit` / `--stop-hold`.
  - All default off.
