# 2026-10-21 first hardware box pushing (whole-body 4-D L1, raw mocap)

Bags: `docs/experimental_data_ros2_bag/1021 - T650-AM whole-body Box-Pushing [Success]-*/…/`, sorted into
`stale/` (`_161745`, `_162604`, `_165311`), `free_flight/` (`_164338`) and `box_pushing/` (`_170123`).
Setup flown: 240 g box (the real box emptied), Push Distance 0.10 m at the shipped mean speed
21 mm/s (push 4.76 s + 1.5 s settle), push pose [0, 32, 28, 0], hardware box-pushing yaml
(`AM_HW_PROFILE=box_pushing`, raw stack, mocap guard on), mocap 120 Hz.

## 1. Which bag is what

| bag | duration | what happened | class |
|---|---|---|---|
| `_161745` | 73 s | takeoff, DIRECT, **Plan refused** twice (box point (1.002, 0.106) is off the yaml table x 0.600..0.750), SAFETY, land | stale |
| `_162604` | 712 s, **no metadata.yaml** (recorder not closed) | the same refused-Plan flight (0–44 s), then **11 min on the ground** with the recorder running: stack restarted at 529 s, the box put on the table and re-captured | stale |
| `_164338` | 90 s | Plan OK (box point typed / old capture — the real box was on the floor at (0.73, 2.43)), Go To Start, 16 s hover above the "handle", Go To Land Start, Land. No Ready / Push | free-flight test |
| `_165311` | 59 s | box on the table at x 0.576 → **Plan refused** (off the yaml table); flown home from the drone GS | stale |
| `_170123` | 96 s | **the formal push**: all steps, box pushed, the exit was a **q3-guard ABORT**, then Go To Land Start + Land | pushing |

## 2. The raw-mocap guard on hardware (`tools/guard_check.py`, `guard_check.txt`)

- **Never CONFIRMED or LOST in any bag; no false abort; EKF2 fused vision the whole time**
  (`cs_ev_pos` = 1 while armed).
- **8 real VRPN pauses** (75–100 ms, ~1 per 30–40 s of flight; none during contact, one in Go
  To Land Start of `_170123`). The processor republished the last pose **with zero velocity**
  (4–5 samples). Each one: caught as *frozen* after 50 ms, BRIDGING 5–25 ms, back to MOCAP,
  blended back over 8.7–23.8 mm.
- **1 real drone rigid-body flip** (`_164338` 76.36 s, 91 mm / 180°): rejected on its own
  sample, the law's odometry never saw it (odometry tilt ≤ 4.0° in that bag).
- **Two gaps:**
  1. The first ~4 frozen samples (~50 ms, by the 50 ms rule) still reach PX4 and the law,
     at zero velocity.
  2. **The processor's finite-difference velocity spikes after each pause or flip** (it
     differentiates across the gap and the catch-up burst). The guard checks position only,
     so the spikes pass into the odometry's twist: +3.75 m/s for one sample after the flip,
     0.5–2 m/s after pauses. The law's response: 1–2 ticks of u1 up to +61 N, motor command
     ±0.48, body torque ~1 N·m. Rotor lag filters it: tilt unchanged. Not dangerous today; in
     contact it also kicks the arm torques.
     - Fix: during the blend-back (or N samples after a rejected one), use EKF2's velocity or
       bound the velocity step.
     - Or fix the processor: stop publishing on NO DATA (the 09-24 recommendation).
- The CONFIRMED → abort and LOST paths are still exercised only in Isaac (no `test_fault` run).

## 3. The formal push (`_170123`) against Isaac (same task, `box_pushing_speed_20261021/runs/v21_*`)

### Outcome

- **Box:** 107 mm at the push end, pulled back to ~91 mm before the jaws opened, 97 mm at rest
  after landing. Upright.
- **Vehicle:** tilt ≤ 5.7°, no saturation. Arm-cap clipping ≤ 0.03 s per phase.
- **q3-guard abort at 46.62 s** (q3 5.0° < 10°), 0.78 s after the push ended. That was before
  the GS's 1.0 s auto Exit, so the jaws were still closed and the claw still world-held.
- **The abort itself:** jaws opened within 30 ms, climbed 0.30 m, folded home; then Go To Land
  Start and Land were flown. Pilot STAB at 93.6 s → SAFETY in the same tick.

| metric (speed_metrics definitions, `push_metrics.txt`) | hardware | Isaac v21_1..3 |
|---|---|---|
| box travel / plan [mm] | 107 at push end (90 at abort) | 98–101 |
| box stuck in the push [%] | 20 | 0–11 |
| box peak speed [mm/s] (plan 47) | 103 | 58–75 |
| box lag, max [mm] (guard 40) | 24 | 10–15 |
| box tilt, planner's [deg] (guard 2, 0.2 s) | **3.1** (isolated 10 Hz samples > 2) | 0.07–0.37 |
| q2 max, push → exit end [deg] | **45.3** (hardware limit 45) | 39.6–42.4 |
| q3 min, push → exit end [deg] | **−7.3** (guard 10, elbow branch < 0) | 13.2–18.1 |
| vehicle tilt [deg] | 5.7 | 2.6–3.3 |

### The mechanism: the world-held claw swing, 2× the sim's

Source: `phases.txt`, `spectra.txt`, `fig_hw_vs_sim.png`.

- **Before the world hold the arm was locked.** Under the CoM anchor (hover and the Ready
  descent) q2/q3 stayed at 32.2/29.9°.
- **The swing starts the moment the claw is world-held** (EE anchor → WORLD at 36.50 s,
  1 s blend). By 37.3 s, with the jaws still open, the arm absorbs the base's motion along
  the arm: q2/q3 swing in antiphase.
- **The same mode exists in Isaac.** While world-held and in contact:

  | | hardware | Isaac |
  |---|---|---|
  | (q2−q3)/2 rms | 10.2–10.6° | 5.3–5.9° |
  | frequency | 0.54–0.57 Hz | 0.59–0.70 Hz |
  | CoM error along the arm, rms | 14–15 mm | 6–8 mm |

- **The free hover wander is a different motion:** 0.13–0.19 Hz, CoM error 13–17 mm rms on
  hardware vs ~2 mm in Isaac.
- **Ranges on hardware:** q2 13–45° and q3 5–50° in the slide. Both the hardware q2 limit
  (45°) and the q3 stop (50°) were touched; the q3 window 10..50° is narrower than the
  swing's 45° p-p.
- **The swing drove the box:**
  - clamped, before the push started, it was pushed 10.5 mm **backward**;
  - it then stuck for the first ~1.8 s of the slide and surged at ~100 mm/s;
  - after the push the swing pulled it back ~16 mm.
- **The trip was the swing's last half-cycle:** q3 went from its +50° stop (box surge) to 5°.
- **The abort planned from the MEASURED pose:**
  - it held the buckled arm, q2 on its +45° limit and q3 ≈ 2° (−7.3° at worst), for ~10 s
    of climbing before the fold home;
  - the vehicle stayed calm (tilt ≤ 3.5°), but that is the elbow-singular boundary.

### Other hardware-only findings

- **Static claw offset.** Under the world hold the claw sat 8–10 mm **below** its reference
  (Isaac 0–3 mm), so the 120 N/m spring lifted the box ~1 N: F̂_z ≈ −0.8 N on a 2.35 N box.
  **Cause (`tools/claw_offset.py`, `claw_offset.txt`): the arm's u3 feed-forward learned the
  joints' STICTION, not the 4-D joint trim.**
  - 28.9–36.9 s, claw riding the CoM: q2/q3 frozen at 32.2/29.9° (std 0.01°). Static
    friction held them, and the momentum residual's joint rows read it as a disturbance:
    j2 +0.15, j3 +0.11 N·m. That is inside the calibrated friction band at this load
    (j2 ~0.2, j3 ~0.1 N·m). The slow fold before it (Go To Start) had already taught the
    same sign.
  - The feed-forward `d_int_f` (C_x at `wb_l1_omega_x_q` 0.1 → τ 10 s) learned j2 0.12–0.13 /
    j3 0.07–0.09 and subtracted it from u3.
  - The world hold set the arm moving at 37.3 s. Within ~1 s the residual fell to
    j2 0.03 / j3 0.00, but the feed-forward still subtracted ~0.1 N·m per joint → the
    arm sagged.
  - Static prediction, `Λ_y M_y⁻¹ K_y e_y = J̄ (d − d_int_f)`, with d_int_f rebuilt from the
    logged `d_sigma_c` (checked against the logged `u3_est`, 1.5 mN·m rms):
    - world hold, jaws open: −8.3 mm (arm rows alone −9.2) against −9.3 measured;
    - stuck hover: +1.5 / +2.2 against +1.9.
  - **The trim cannot be it.** The offset appears at 37.5 s, 2 s before CONTACT (39.55 s).
    In free flight ŵ_q reaches only the raw force reading: F̂_y = 0 and w_int = w_sigma.
  - After the grasp the box (2.35 N > 1 N of spring) holds the claw where the jaws closed.
    The frozen trim changes only how the force is booked.
  - Isaac v21_1..3: hover joint residual −0.02..+0.05 N·m and the joints never freeze. Its
    friction model is smooth (tanh), with no stiction, so there is nothing to unlearn.
  - Harmless for this grasp: 9 mm lower on a 60 mm fin, and the box's normal force drops
    ~1 N.
  - Untested fixes, law code:
    - clear the arm rows of `d_int_f` at the world-hold switch, or give them a faster filter;
    - `wb_l1_stuck_hold_j2/j3` would NOT help: it freezes the rows at the value the slow
      fold already taught.
    - Isaac cannot test either fix; the offline arm models with stick-slip can.
- **`obj_0` (the box) mis-solves:**
  - 3 samples (25 ms) **in contact** (40.85 s): tilt 96°, a 61 mm jump. The box-lag and
    box-tilt guards saw it; only their 0.2 s persistence prevented a false abort.
  - ~10 more at 15.7–17.9 s, with the vehicle 1.7 m away.
  - Re-define the box body in Motive (asymmetric markers), and gate box samples the way the
    drone guard gates the drone.
- **The 2.4 s wait before the jaws closed.** The GS closes only within 2 cm / 3 mm, and the
  swing kept the claw moving. The swing was already 6° rms then.

## 4. Friction (`force_check.py`)

- **The force estimate is usable when averaged.** Isaac check: the law's raw F̂ averaged over
  the slide reads the push-resisting force within +0.1–0.2 N of the PhysX truth (raw noise
  4 N rms at 250 Hz). Hardware raw noise: 7 N rms.
- **Hardware reading:** push resistance ≈ +0.25 N above the hover baseline, with the box lifted
  ~0.8 N. That implies μ ≈ 0.25 / (2.35 − 0.8) ≈ 0.16 ± ~0.1.
- **The flight cannot refine the bench calibration** (μ 0.20–0.28, 0.25 used), but the
  reading is consistent with it.
- **The box's stick-slip is the arm swing, not friction:** the box moved back and forth while
  clamped, before any push.

## 5. Before 500 g / 0.50 m

- **The swing must come down first.**
  - The 0.50 m push slides 24 s (5× today's 4.8 s) at 2× the force.
  - Isaac's 500 g runs swing 3.6–4.4° std; hardware has been 2× Isaac.
  - The swing already used the whole q3 window today.
- **Prerequisites:**
  1. Measure the real table and set `push_pull_table` (the sim's table is in the yaml today).
  2. Start the box at y ≈ 0.28 for 0.50 m. From today's y 0.10 the planner refuses the push
     (the leading end would reach −0.52 m).
- **A one-change-at-a-time ladder:**
  1. 240 g / 0.10 m with the post-push wait shortened. `push_pull_auto_exit_delay_s`
     1.0 → ~0.2; the trip came at +0.78 s; the swing half-period is ~0.9 s.
  2. 240 g / 0.25–0.30 m (a long contact at the same force).
  3. 500 g / 0.10 m.
  4. 500 g / 0.50 m.
- **Code / sim candidates (not done):**
  - damp the world-held 0.55–0.7 Hz mode: Isaac reproduces it at half amplitude, so a sim
    campaign can rank candidates. Earlier rejections: D_y 40 and k_v 20/25 left DIRECT in free
    flight;
  - an abort that folds the arm off a buckled pose once the jaws are open, when the trigger is
    a joint guard;
  - a mis-solve gate on `obj_0`;
  - velocity sanitising in the mocap guard.

## Tools (all `PYTHONNOUSERSITE` not needed except the figure)

```bash
bash tools/run_extract.sh                         # bags -> npz/ (ROS sourced inside)
cd tools
/usr/bin/python3 timeline.py   > ../timeline.txt  # per-bag event timeline + rosout warnings
/usr/bin/python3 guard_check.py                   # mocap guard vs raw VRPN / processor / PX4
/usr/bin/python3 push_window.py 38.8 47.2 0.1     # dense table of the push
for s in phases push_metrics spectra force_check; do /usr/bin/python3 $s.py; done
/usr/bin/python3 claw_offset.py > ../claw_offset.txt  # why the claw sits low (stiction in u3's ff)
PYTHONNOUSERSITE=1 /usr/bin/python3 fig_compare.py  # -> ../fig_hw_vs_sim.png
```
