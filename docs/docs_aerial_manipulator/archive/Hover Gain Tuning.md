# Aerial-Manipulator Hover Gain Tuning — PX4 Direct-Actuator Rig (2026-07-31)

> **REVISION (2026-07-31b, supersedes the gains below):** the first shipped
> configuration (arm K_o = 0, TAU_MAX = 1.5) hovered 90 s headless but
> **crashed at t = 8.2 s in the user's rendered run** — rendered stepping is
> bursty (≈4 physics substeps per render frame), which excites a ~2 Hz coupled
> body–arm mode that headless timing never wakes; the never-settling
> oscillation regenerates a ≈ −1 N z-phantom in `d_t_hat` which, in shaped
> mode, leaks a persistent fold bias into the arm through the task map.
> Headless steady state is a perfect model match (`d_t_hat ≈ 0`, thrust
> = m·g exactly), so the phantom is a symptom of the oscillation, not a bias
> in the calibration. Changes (user-directed + re-swept, round R + 4×90 s
> endurance, all with the new clamp):
>
> - **TAU_MAX 1.5 → 3.0 N·m** (user request; interaction headroom). This alone
>   un-crashed the arm-K_o=0.5 config that previously diverged at 33 s — the
>   old clamp was locking the null-space limit cycle in.
> - **K_o = 0.5 · I₁₀ (uniform, arm channels ALIVE)** — required for the
>   physical-interaction goal; 0.5 halves the phantom-loop gain everywhere for
>   rendered-timing margin.
> - **D_y position channels 6 → 12** — task damping aimed at the ~2 Hz mode.
> - K_y stays diag(8,8,8,1); body loops (k_x/k_v/k_R/k_w, M_r_d) stay stock.
>
> Validated headless: 90 s ×2 clean (pos rms ≈ 0.2 mm, zero saturation).
> Faster-observer alternative, also 90 s ×2 clean but thinner margin:
> `K_o = diag([1.0]*6+[0.5]*4)` with `D_y = diag(6,6,6,1)`. Round-R note:
> body K_o=1 + D_y=12 + arm 0.25 crashed once at 27 s — margins near body 1.0
> are thin and run-to-run variance is real; that combination is not shipped.
> **Rendered validation is the remaining gate** — run the launcher and check
> `/tmp/am_gmo_px4_isaac.log` + `log/gmo_px4_log.npz`.

Gain tuning of the whole-body GMO controller for **stable hover** on
`application/robotic_arm/03_px4_direct_aerial_manipulator_free.py` (renamed from
`px4_direct_actuator_aerial_manipulator_gmo.py` on 2026-08-10) — the
PX4 direct-actuator rig described in [Direct Thrust.md](<Direct Thrust.md>).
25 full-stack trials, three root causes found (one environmental, two gain-level),
final configuration validated with 60 s and 90 s endurance runs.

## Configuration under test (all fixed, per the task definition)

| Item | Value |
|---|---|
| Vehicle | `AM_realign.usda`, **body mass overridden to 3.416 kg** (total 4.212 kg / 41.32 N) via the demo's `DRONE_MASS_OVERRIDE` (runtime USD edit; the .usda is never modified). Control model shifted by the same delta. |
| Motor model | `LaggedQuadraticThrustCurve`, bench MN4014+15x5 constants, **λ = 10.51 1/s (τ ≈ 95 ms) — unchanged, by requirement** |
| Rotor path | GMO law in-process → mixer → ω[4] → rotor bridge → PX4 (direct-actuator gate, no PX4 control loops) → `HIL_ACTUATOR_CONTROLS` → lagged thrust curve. Measured transport delay ω_cmd→plant: **12–16 ms** (per-channel correlation ≥ 0.99). |
| Arm policy | `TAKEOFF_ARM_HOLD=True`, `ARM_ALWAYS_PD_HOLD=False`: PD-hold during the 4 s climb, then the **designed impedance+GMO law** for the rest of the flight. |
| Trajectory | `hover` (climb to 1.5 m, hold) |

## Final tuned gains

Applied as `TUNED_GAINS` in the demo (instance-level override at startup).
`controller.py` is **untouched** — `config/free.yaml` remains the tuning
for the in-process `02_*` rigs, which fly a *different plant* (no rotor lag, no
transport delay, 2.476 kg body).

| Gain | 02-rig default | **px4-rig tuned** | Why |
|---|---|---|---|
| `K_y` | diag(200, 200, 200, 40) | **diag(8, 8, 8, 1)** | see Cause 2 |
| `D_y` | diag(6, 6, 6, 1) | **diag(6, 6, 6, 1)** (unchanged) | ζ ≈ 1.06 with K_y=8 |
| `K_o` | 20 × I₁₀ | **diag(1,1,1, 1,1,1, 0,0,0,0)** — body 1 rad/s, arm off | see Cause 3 |
| `k_x, k_v, k_R, k_w` | 16, 8, 8, 2 | **unchanged** | body loops were never the problem; k_w=3 made it *worse* (rate gain through the lag) |
| `M_r_d`, `M_Y`, `DLS_LAMBDA` | — | **unchanged** | probed (M_Y heading 0.2, DLS 0.8, M_r_d ±50 % planned); no benefit |

Per-entry differences inside each diagonal were allowed and explored; the winner
only needs the body/arm split in `K_o` and the heading detune already present in
`K_y`/`D_y` (position 8/6, heading 1/1).

**Validated hover quality** (scoring window t ≥ 8 s, 250 Hz log):
CoM position rms **≈ 0.2 mm**, |e_R| rms **≈ 3×10⁻⁴**, EE error rms **≈ 1.3 mm**,
arm |q̇| rms ≈ 0.01 rad/s, **zero** torque-clamp saturation. Endurance: 60 s
twice + 90 s once, flat (no hidden slow growth). For comparison, GMO fully off
holds only ≈ 3.0 mm — the slow body observer buys ~15× better steady position.

**Validated fallback** (if the arm observer channels must stay alive, e.g. for
d_ρ estimation work): `K_o = diag([0.5]*6 + [0.25]*4)` — 60 s clean. Slower
disturbance convergence, small stability margin; prefer the primary.

## The three root causes, in the order found

### Cause 1 (environmental): a second controller was fighting on `/uav_0`

First headless trials rolled over at t ≈ 1 s with the commanded roll torque and
the realized roll response in **opposite directions**. Per-step npz
instrumentation (now permanent: `omega_cmd` / `omega_plant` / `omega_real`)
showed `omega_plant` **alternating between our command and a foreign smooth
signal**: a leftover `autopilot_direct_actuation_node` + `virtual_remote` from
an earlier bare-X650 test session (tmux `fsc_direct_actuation_x650_stack`) was
still streaming `ActuatorMotors` on the same namespace. PX4's output module
interleaved the two publishers; the foreign node's **iris-convention allocation
is roll/pitch-mirrored on AM_realign's rotor indexing** (AM rotor0 is
front-*left*; iris/x650 rotor0 is front-*right*), so its "corrections" actively
flipped the vehicle. This is the "two rigs at once" / "orphaned processes"
failure already tabled in Direct Thrust.md's troubleshooting.
**Fix**: kill the stale stack; the sweep harness now refuses to start while any
foreign `/uav_0` offboard controller process exists. After the fix the
passthrough is a clean identity (corr ≥ 0.99, 12–16 ms lag) and **takeoff +
PD-held hover is stable at the default body gains** — confirming PX4's
`PWM_MAIN_FUNC1..4 = Motor1..4` channel mapping is identity for this rig.

### Cause 2 (gain): K_y = 200 saturates the ±1.5 N·m arm-torque clamp

At the takeoff→track handover the re-anchored EE reference sits a few mm off
the actual EE. With K_y = 200 that alone is ≈ 1.2 N of task force (+0.24 N·m
joint-3 kick), and any cm-level excursion demands joint torques far beyond
`TAU_MAX = 1.5` N·m — the law's raw commands reached **±20 N·m**. Deep
saturation turns the impedance into a 1.4 Hz bang-bang limit cycle whose
reaction torque drags the base attitude down within ~1 s. K_y softened to the
historically validated **8** (heading 1): EE rms is ~1.3 mm anyway, and torque
commands stay comfortably inside the clamp (zero saturation).
*(Note: `K_y=200` in `config/free.yaml` post-dates the documented `8,8,8,1`
sweep value — the old values survive as a trailing comment on that line. On the
in-process rig 200 may be flyable; on this lagged rig it is not.)*

### Cause 3 (gain, the deep one): GMO bandwidth above the actuator bandwidth

The momentum observer assumes commanded generalized force = applied force. On
this rig the rotors respond through a **first-order lag (10.51 rad/s) plus
12–16 ms transport**, so during transients the observer books the
command-vs-realized gap as a *phantom disturbance*; the law cancels the
phantom, which makes the command more aggressive, which widens the gap —
positive feedback. With `K_o = 20 rad/s` (twice the actuator bandwidth) the
loop diverges ~1 s after the arm law engages, **independent of every other
gain** (the whole K_y/D_y ladder crashed identically; the climb — GMO frozen —
was always clean).

Measured stability boundary (hover, this plant):

| body-channel K_o | arm-channel K_o | result |
|---|---|---|
| 20 (default) | 20 | diverges ~1 s after handover |
| 10 / 5 | any | diverges |
| 2.5 | 2.5 → 0 | body stable; slow arm oscillation → crash 13–16 s |
| 2.0 | 0 | crash (~30 s trial) |
| 1.5 | 0 | 30 s clean (endurance not tested) |
| **1.0** | **0** | **60 s ×2 + 90 s clean — shipped** |
| 1.0 | 0.5–1.0 | clean for 24 s, then a ~0.5 Hz **null-space arm oscillation** (σ ≈ 0.6 1/s, invisible to the task-space D_y) emerges from the noise floor and crashes at ~30 s |
| 0.5 | 0.25 | 60 s clean (the fallback) |

Two practical lessons encoded in that table: (a) the classic DOB rule — keep
the observer well below the actuator bandwidth (here ~10× margin, not 2×,
because the destabilized mode is nearly undamped); (b) **30 s trials are not
proof** — the arm's null-space mode can sit below the noise floor for 20+ s
before emerging, so anything shipped must pass a 60 s+ endurance run.

## Reproducing / re-running the sweep

```bash
# one trial set (JSON list of {name, sweep}); sweep keys = the demo's AM_SWEEP keys
python3 application/robotic_arm/utils/px4_gmo_gain_sweep.py \
    --trials trials.json --t-end 60
```

The orchestrator runs the complete stack per trial (PX4 SITL `none_iris`
headless + MicroXRCEAgent + rotor bridge + headless Isaac), applies the
`UXRCE_DDS_SYNCT=0` / disarm-guard params via the pxh console, waits for
engagement, auto-stops at `t_end` (sim time, clock starts at PX4 engagement),
scores the npz (position/attitude/EE/arm rms, saturation fraction, divergence
detection), and tears everything down. ~2–3 min wall per 30–60 s trial;
`engaged at +~31 s` is normal (Isaac asset load). It aborts if a foreign
`/uav_0` controller process is detected. Trial `t_end` inside `sweep`
overrides the CLI value.

## Caveats & scope

- Tuned and validated **headless** (sim ≈ wall-clock rate). Rendered runs are
  *slower* than wall clock, which shrinks the wall-clock transport delay in
  sim-time terms — i.e. the rendered rig is the *easier* case; gains that
  survive headless carry extra margin rendered.
- Hover only. `circle` / `circle_bent` / pickplace re-introduce arm-moving
  transients — re-validate (and expect K_o headroom to shrink) before claiming
  those.
- The arm-channel observer is **off** in the shipped gains. Structural fixes
  (e.g. a lag-aware observer model) would be a *law* change — out of scope for
  gain tuning, but the right direction if arm-channel d_ρ estimation is needed
  on this rig.
- Run-to-run variance is real (wall-clock DDS/mavlink jitter): one 0.5/0
  configuration crashed early where its neighbours were stable. The shipped
  configuration passed three independent endurance runs; treat single 30 s
  passes as suggestive, not conclusive.
- The EE-disturbance GMO demo (`EE_FORCE_ENABLE`) still works with the shipped
  gains — d_t converges with a ~1 s time constant (vs 50 ms at K_o=20).
