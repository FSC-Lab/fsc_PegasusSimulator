# Implementation record — `slosh` branch, hardware-transfer campaign

What was built, why, how to reproduce it, and what it measured. Newest first. `CLAUDE.md` carries the
narrative and the operating rules; this file carries the *implementation*: the edits, the exact
commands, the artefact paths and the commits, so any entry can be re-run without reading the log.

Repos are paired — sim `fsc_PegasusSimulator` (`slosh`), controller `fsc_autopilot_ros2-master`
(`slosh`), GS `ros2_ground_station_gui` (`vtol-cascade-work`), msgs. Baseline for the hardware
campaign is the tag `hw-transfer-1` in all four (sim 05a4a0e / controller f26e09d / GS 96a1fb3 /
msgs ebeded8).

---

## 2026-09-22 — The INDI inner loop as its own controller: wired, tuned, flown, and the loop audit

**controller `401f4a8`…`204c0c4` · sim this commit · GS `f4aac46` + `5bd7177` · flights
`R100_dx3_indi_hover30_a1_122623` and `R100_dx3_indifull_hover30_a1_134802`, both LANDED 0.00 m**

### 1. Its own controller, separate from everything else

`controller_id cascade_indi` → node `/nmpc_cascade_control_indi`, guidance `/uav_0/cascade_indi/*`,
launcher window `6j-indi` (standby, 30 s behind 6h), GS button "INDI NMPC + Geo" + mission ctrl
choice, READY gates, health check, driver, results row. The GS's `CASCADE_IDS` / `ARM_NODES` now
drive every place a cascade id is enumerated, so a fourth one is a single edit.

### 2. The correction that mattered

The 09-21 build put INDI *beside* the geometric law (the L1 seam) and then faded it out at speed.
The ask was to replace the geometric inner loop. `--indi_mode full` does: ν = −ωn²e − 2ζωn·ω and
**M = J ν − k(J ω̇_f − M_c(u₀)) − M_trim** into the same allocation, via a new `MOMENT_LAW` seam in
the base (flown lines byte-identical: `460eb532…` / `b5010243…`).

Three bugs the gate found, in order of importance:
1. the allocation adds the inner moment **on top of the outer's trim**, so the law must return a
   correction — otherwise the trim's own moment is applied twice (zero on roll/yaw, the whole story
   on pitch);
2. `M_c` must come from `model_moment`, not a second hand-written copy — the copy mirrored the
   allocation's inverse formulas including its deliberate `−CNDR` rudder sign, i.e. the yaw control
   moment backwards, and in an incremental loop M_c cancels against J ω̇ *exactly*, so any error is
   fed back with gain one;
3. k must scale the whole increment.

### 3. What the full replacement costs and buys

k = 1 (the textbook increment) loses the aircraft in the corridor at t 14.6 s; k ≤ 0.8 flies. Not
noise (crashes identically with noise off), not the filter (8–40 rad/s), not the ν bandwidth
(3.1→12 only postpones), not the slew limit, not command-vs-achieved. It flies with the elevator
30× faster, or with only pitch inverted, or only roll+yaw — and a *slower* transition is worse. The
pitch effectiveness is not one G: the allocation splits it between rotors (67 ms) and elevator
(30 ms) and the split moves with qbar.

| | flown | INDI full (k 0.7) |
|---|---|---|
| hovers (still / CG 5 cm / quartering / crosswind) | 1.22 / 1.50 / 1.22 / 3.39 m | 0.98 / 1.55 / 1.48 / **2.84** |
| straight + back-transition | 0–1 fails | 21–24 fails, tighter hold |
| turning, 3 seeds | orbit roll −18.2 sd **0.11** | −27 sd 0.74–2.21, bank 29–30 |

### 4. The flights

Both landed 0.00 m with 0 solve failures in the stage. The full form gave the quietest hover yaw
diagonal of the three builds (0.028 vs 0.042 / 0.046) and tracked pitch more tightly than the flown
loop (sd 0.07 vs 0.15) — **and crept 7.2 m along track in 30 s** where the augmented form held 1.3.
The hover hold brakes through the outer's planned pitch, and the flown law's standing pitch error
(+1.3° vs the plan's +0.5°) was part of that braking. So `INDI_MODE` defaults to `aug`; `full` is
one env var away until `hold_k` is retuned. The gate does not reproduce the creep (its hover starts
at rest) — a named blind spot.

### 5. The loop audit (live stack, three cascades + the safe controller)

- **Gating:** 5 publishers on `actuator_motors`; with `active_controller=none` the stream is
  **silent**; with geometric active, 199.9 Hz from one node. Activation is an exact id match.
- **Identity:** 4 distinct node names, each `controller_id` matching; the node name comes from argv
  and the topics from the parameter — the boot test proves they agree.
- **Guidance:** one controller subscriber per `<id>/cmd`, its own.
- **The flight:** only `/uav_0/cascade_indi/status` carried traffic (563 lines vs 0 and 0).
- **Seams:** L1Indi → L1Imu → L1Steso → L1Aug; `STATUS_EXTRA` chains the parent's, so the slosh and
  predictor fields stay on the feed.
- **Two real findings:** `kill_hil.sh` was missing `nmpc_cascade_imu_node` — the *flown* node — so it
  survived every teardown; and the GS HUD subscribed to only two cascade status feeds, so the
  Guidance row would have shown a stale line while the INDI one flew. Both fixed.

## 2026-09-21 (night) — An INDI inner loop as a variant of the flown node (gated, not flown)

**controller `401f4a8` (`nmpc_cascade_indi_node.py`) · sim `21831f2` (boot test `--node indi`, `scripts/indi_ab.sh`)**

The user's question: would an incremental nonlinear dynamic inversion inner loop, built on the model in use,
replace the geometric inner loop so the model need not be as accurate? Built as a copy in the variant pattern
(`L1Indi(imu.L1Imu)` re-pointing `base.L1Aug`; the flown node untouched, `--indi_off` reproduces its hash
`460eb532b42d5443e66edd6218233957`).

### The derivation (the docstring has the long form)

1. `J w_dot = M_c(x,u) + M_a(x) + M_d − w × J w`
2. first order about the current state and the control **in force** (u0, through the delay and the lags)
3. time-scale separation: `w_dot ≈ w_dot_0 + G (u − u0)`, `G = J⁻¹ ∂M_c/∂u`
4. `u = u0 + G⁺ (ν − w_dot_0)` — the increment; `w_dot_0` measured, so the aero, the unmodelled moment and the
   gyroscopics are inside it; only the control effectiveness is modelled (an error in (0, 2) converges)
5. at the moment level: `M_des = M_c(u0) + J (ν − w_dot_0)`
6. with `ν = M_geo / J`: **`M_des = M_geo + [M_c(u0) − J w_dot_0]`** — the flown law plus the measured
   non-control moment cancelled at the rate filter's bandwidth (40 rad/s; yaw 10). It is the L1's job
   (`u_ad = −C(s) σ̂`) with the residual measured instead of adapted, at 40 rad/s instead of 2, **and with the
   aero stiffness and damping cancelled too** — which is the whole story of the gates.

Synchronisation: the rate derivative and the applied control moment (the L1's delay FIFO + actuator-lag states)
go through the same two first-order lags.

### What was measured (the flown IMU node as the A; `scripts/indi_ab.sh`)

| | flown | INDI (roll + faded yaw) |
|---|---|---|
| hovers still / CG 5 cm / lat 50 / noise 3 (worst displacement) | 1.22 / 1.50 / 1.25 / 1.25 m | 1.22 / 1.51 / 1.26 / 1.25 |
| 5 cm CG standing pitch | −1.14° | −1.14 (−1.15 with pitch INDI: the outer's crutch) |
| still-hover yaw diagonal sd | 0.063 | 0.071 |
| **quartering-wind hover, leaning** | 1.22 m, roll 8.4° | **1.12 m, roll 5.3°** (the one win) |
| straight 90 s incl. cQ 0.5×/2×, inertia 0.6, lat aero 1.5, spin 0.18, lat 50/80 ms | 0/1801 | 0/1801, indistinguishable |
| straight back-transition | +12.0 / z 29.9 / stop 71.0 s | identical to the tenth |
| turning mission 3 seeds + cross5 | flies | flies; fly-by peak bank **28.1 vs 24.4** (ref 25), orbit sd 0.14 vs 0.11 |
| yaw INDI unfaded | — | stalls out of the first turn (seeds 2, 3, cross5, diag5) |
| pitch INDI | — | crashes the corridor at t 9.7 |

### The reading

- Yaw at speed: the flown damping-only law is safe **because it is weak** — the fin flies the coordinated turn;
  INDI made it exact and the aircraft stalled out of the turn → the yaw increment fades with the wing share.
- Pitch: the outer plans with the airframe's pitch stiffness (`build_ode6`'s cma); an inner loop that inverts
  it hands the outer a plant it did not model — the 2026-09-10 "two loops doing pitch" bang-bang, measured again.
- Roll: harmless to invert, but INDI removes the roll subsidence (Lp 1.68 N·m·s at 16 m/s > the loop's kW 1.2),
  so the flown gains are under-damped on the inverted plant (ζ 1.0 vs 1.7) and the bank overshoots the 25°
  reference by 3–4°; `indi_kw` (default Lp at cruise) puts the damping back, the rest is the 55 ms of delay + filter.
- **INDI needs the actuator model more, not less** (G, the delay, the lags must be synchronous); what it lets
  you drop is the aero model — and this loop's outer NMPC and yaw law are built on it. The premise holds for a
  multicopter whose inner loop is the controller; here the model lives in the outer, which INDI never touches.

### Standing (only if pursued)

The roll retune for the inverted plant; a hover-stage flight of the variant; yaw INDI in hover under the
measured cQ against the fin (the +13 % hover diagonal is the noise gain).

## 2026-09-21 (evening) — The measured build's first HIL flight: a 30 s hover stage

**sim `f4a9863` · controller `1990e91` · row `R100_dx3_mn4014_hover30_a1_155915`, bag
`~/flight_bags/slosh/slosh_20260921_155735` · LANDED 0.00 m**

`STAGE=hover STAGE_HOLD=30 CASC_SPIN=poll` on the 1b configuration (`CTRL=cascade_adaptive CASC_EST=mdob
ADAPTIVE_NODE_PY=…/nmpc_cascade_imu_node.py CASC_STATE=px4 EKF_SOURCE=gps VTOL_GPS_HZ=10 IMU_DR=1 CASC_ASPD=px4`,
`VTOL_TANK=slosh VTOL_TANK_KG=0.502 VTOL_TANK_R=0.10 VTOL_TANK_DX=0.3`) through `run_slosh_flight.sh`, `af=21fad0371f`.

| stage | result |
|---|---|
| climb (safe controller, yaw-last mixer + weathervane in the loop, still air) | 30 m, no release |
| engage hover (along-track hold only) | `HOVER HOLD at (-0.6,-0.1)`, settled at z 30.1 |
| 30 s hover stage | **go**: 0.6–1.4 m ahead of the point, V 0.15–0.4 m/s, z 29.7–31.1, pitch −1…−2°, roll ±1°, yaw 0, tilt −1…−3.5°, 0/693 solve failures, outer 11–12 ms |
| back-transition from the hover, recover, land | +14/−4° pitch, 3.6 m, V 2.45 (the 09-16 shape); landed at (0.81, 0.08), 0.00 m |

Scored: `bag_axis_fit.py` (now on the airframe file) — yaw diagonal sd 0.046 at 0.9 Hz (quiet < 0.02, thrash
0.4–0.8), yaw I_eff 0.32 vs 0.49 (the plant makes 1.5× the modelled yaw per throttle step: the spin-up
reaction is no longer small beside cQ 0.019 → measure the rotor + prop inertia from the stand's torque
transients); `ekf_retune.py` — floors 0.074 N / (0.009, 0.012, 0.022) N·m, bias bF +0.07 N, the 09-16 rehearsal's
numbers.

### Reproduce

```bash
VTOL_TANK=slosh VTOL_TANK_KG=0.502 VTOL_TANK_R=0.10 VTOL_TANK_DX=0.3 CTRL=cascade_adaptive CASC_EST=mdob \
  ADAPTIVE_NODE_PY=$HOME/envs/fsc_autopilot_ros2-master/px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_imu_node.py \
  CASC_STATE=px4 EKF_SOURCE=gps VTOL_GPS_HZ=10 IMU_DR=1 CASC_ASPD=px4 CASC_SPIN=poll \
  STAGE=hover STAGE_HOLD=30 NAME=R100_dx3_mn4014_hover30 bash scripts/run_slosh_flight.sh
VTOL_TANK_DX=0.3 ~/envs/isaac_env/bin/python3 scripts/bag_axis_fit.py ~/flight_bags/slosh/slosh_20260921_155735
~/envs/isaac_env/bin/python3 scripts/ekf_retune.py ~/flight_bags/slosh/slosh_20260921_155735
```

---

## 2026-09-21 (evening) — The wind table on the measured build, the diag5 regression, and B2 (the tilt servo as a mismatch)

**controller `2ab28bf` (hold: lean + weathervane in the stages only, `hold_wv_s` 1.5, gate `--settle`) +
`1990e91` (B2) · sim this commit · identity references (md5sum of the `--log` CSV): after 2ab28bf steso
`22a5a15381bdcb77d3fcca7d68c38fd4` / IMU `0ebd9753611143df22a618f449377520` (the engage hover's roll_hover
gone, first differing row t 0.115); after 1990e91 **steso `b501024381f70fc1b3d546479031bcb1` / IMU
`460eb532b42d5443e66edd6218233957`** (the gate's node slew 80 → the file's 106; `--tilt_slew 80` reproduces
the previous pair exactly).**

### The wind table (`wind_sweep.sh`, 16 seeds, imu-mdob + pitot + L1; `~/flight_bags/wind/wind3`, `wind4`)

| cell | wind1/2 (12x8, pre-hold) | wind3 (measured build, 660dd83) | wind4 (after 2ab28bf) |
|---|---|---|---|
| still | 16/16, pk 11.6, stop 6.3 | 16/16, 11.5, 6.1 | 16/16, 11.5, 6.0 |
| head3 / head5 | 16/16, 11.7, 3.3 / 16/16, 11.7, 22–24 | 16/16, 11.5, 9.0 / 16/16, 11.3, 23.4 | — / 16/16, 11.3, 23.4 |
| cross3 / cross5 / crossm5 | 16/16 ×3, pk 11.9–12.5, stops 4.5 / 9.9 / 20 | 16/16 ×3, 12.3–12.7, 4.7 / 4.9 / 22.3 | — / 16/16, 12.1, 10.0 / 16/16, 12.0, 21.1 |
| **diag5** | 16/16, 12.3, 4.2 | **0/16 (corridor, t 8–9 s)** | **16/16, 12.0, 9.8** |
| turb 0.5 / 1.0 / head5+turb0.5 | 15/16 / 2/16 / 11/16 | 15/16 / **5/16** / **15/16** | — |

diag5 A/B on seed 1 (all with the pitot): weathervane off → lost; the 12×8 airframe file → lost; `--hold_k 0`
→ flies; `--hold_roll_max 0` or `4` → flies; L1 off → flies; `--settle 4/6/10` → lost 6–7 s after the start;
pure crosswind or pure headwind 3.5 → flies. The chain is the hold's 8° lean into a wind the pitot estimator
puts on the nose (a 45° sideslip the model then lacks), the L1 learning that (u_ad 0.5–0.6 N·m every axis),
and the transition's tilt swing outrunning C(s). Fix: `lean_ok` / `wv_ok` — the hold leans and weathervanes
only in a `v_cap 0` stage or at the RTH stop; the engage hover is along-track only (the pre-hold behaviour).

### B2 — the tilt servo (`~/flight_bags/robust/tilt1`, `scripts/cells_tilt_servo.txt`)

`so3.build_ode(tilt_tau=, tilt_rate=)`, gate `--plant_tilt_tau` / `--plant_tilt_rate` / `--tilt_slew`, the
tilt knob in `test_plant_knobs`. Plant τ 0.10–0.40 s and slew 80–25 °/s (told 106) all 16/16 at pk 13.8–15.4;
25 °/s told 17.0 (FAIL on the peak); τ 0.15 + 60 °/s 13.8. The servo is not a sensitivity; the spin lag is.

### Reproduce

```bash
JOBS=16 bash scripts/wind_sweep.sh wind4 1 16 still cross5 diag5 head5 crossm5   # + wind_metrics.py
# the diag5 A/B (controller repo root; VTOL_TANK_KG=0.502 VTOL_TANK_R=0.10 VTOL_TANK_DX=0.3)
... --airspeed --dur 30 --seed 1 --guide "20:wp 500 0 500 400 100 400;150:rth" --wind -3.5,3.5,0 [--hold_k 0 | --hold_roll_max 4 | --settle 10]
ROBUST_NODE=nmpc_cascade_imu_node.py ROBUST_LINE="--est mdob --l1 --ttr 10 --lat 0.03 --comp 0.049 --slosh --noise_acc 0.02" \
  bash scripts/robust_sweep.sh tilt1 scripts/cells_tilt_servo.txt
```

### Standing

- The pitot wind estimator in a hover that is not into the wind is wrong by the sideslip angle (worse than
  blind): freeze its direction below a ground speed, or hold the L1 through the first seconds of a transition
  in wind. The card's "transition into the wind" is the operational cover.
- The post-stop hover's solve failures are ~7 per seed on the measured build with the pitot (1 on the 12×8);
  all at t_stop + 0.5–3 s, V 0.1. Named three times now.

---

## 2026-09-21 (afternoon) — Yaw served last, and a weathervane, in both controllers

**controller `e2432f1` (safe controller) + `660dd83` (cascade hold) · sim `824d207` (test) · identity: both
flown gate lines byte-identical pre/post (`cmp` on the `--log` CSV); references from here = `md5sum` of the CSV:
steso `3cf1bdfc13a9a8070b7defd72b0fe2de`, IMU `ba1021676f02b6c6853a3339a85b182e`**

### What changed

| where | what | knobs |
|---|---|---|
| `geometric_hover_node.geometric_command` | collective/roll/pitch allocated first; the yaw moment clipped to `yaw_headroom` of the remaining rotor headroom; `yaw_clamped`, `mz_req`, `mz_avail` in the diag | `yaw_headroom` 0.7 |
| `geometric_hover_node.Weathervane` | after `wv_yield_s` on the clip the yaw reference is released to the current heading (damping only) until a new operator yaw re-arms it; the node logs the release / re-arm | `wv_yield_s` 0.5 (0 = never) |
| `nmpc_cascade_node.INNER_DIAG` | `yaw_ratio` = rotor yaw demand / rotor yaw authority (from the headroom), `sat` — written every inner tick | — |
| `nmpc_cascade_node.HoverHold.update(…, yaw_ratio, dt)` | releases the latched yaw after `hold_wv_s` of ratio > `hold_wv_ratio`; the hold's frame then follows the nose; `wv_released` / `wv_released_now`; `wv=1` on the status feed | `hold_wv_s` 0.5, `hold_wv_ratio` 1.0; gate `--hold_wv_s/--hold_wv_ratio` |
| `nmpc_cascade_node.inner_step` | `l1.yaw_mask`: the L1's yaw residual channel zeroed and not learned while the hold is released (the fin's weathercock is exactly that residual); the model-based slosh term stays | set by the node / the gate from the hold |

### Results

Safe controller (plant double, 40 s hold): 3 m/s heading held (−11.8°), 0.00 m; 4.5 / 6 / 8 m/s released at
0.6–0.8 s, nose −90°, 0.00 m final (worst 1.3 / 1.8 / 2.6 m during the swing), mixer 0.0 % saturated, integral
0.6 / 1.2 / 2.6 N (was 13, pinned, and a 29–77 m flyaway). Clip alone (wv 0): 0.00 m, nose −66°.
Cascade hold (gate, imu-mdob + L1, 30 s): crosswind 3 / 4 / 6 m/s → 1.5 / 2.8 / 6.6 m worst then held, nose
−79 / −84 / −89 (was 2.6 m / 0.9 m/s / 2.8 m/s of drift); headwind and still air unchanged (1.13 / 1.22 m, no
release). Turning mission 3 seeds: the 09-17 numbers (3/3, bob 0.049, hits 0, roll sd 0.154, u_ad 0.006,
stop 5.9), 0 releases.

### Reproduce

```bash
~/envs/isaac_env/bin/python3 tests/test_geometric_hover.py                          # §7: the three rows
# the cascade's hover hold in wind (controller repo root; W = 3, 4, 6; -6,0,0 = the headwind)
~/envs/isaac_env/bin/python3 px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_imu_node.py \
  --test --est mdob --ttr 10 --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1 --noise_rate 1.0 \
  --noise_acc 0.02 --l1 --hover_only --dur 30 --every 2.0 --seed 3 --wind 0,6,0
# identity (md5sum of the CSV) and the turning regression
... --dur 8 --every 1.0 --seed 3 --log /tmp/id.csv && md5sum /tmp/id.csv
KINDS="imu-mdob" PREDS="on" bash scripts/est_sweep.sh wv_turns turns 1 3 --noise_acc 0.02
```

### Found on the way

- The gate's model of the node's own tilt slew is a hard-coded 80 °/s (`slew = math.radians(80.0) * DT`) while
  the node reads `tilt_slew_deg_s` = the file's 106 since 2026-09-17: the gate's controller has been 25 % slower
  than the flown one on the tilt estimate. Fixed in the B2 piece (below), with new references.
- The 2026-09-17 identity values do not reproduce on the unchanged tree; the procedure is now written down.

---

## 2026-09-21 — The full suite on the measured build: the cQ casualty is heading-hold in wind, not the yaw gains

**sim `85bce90` + `2ec6729` · controller `334893f` (the 2026-09-17 `airframe_id.cfg` edit, committed today) ·
GS `e7866ac` · identity references unchanged (no controller code moved)**

### What was run

The whole headless suite (`tests/test_*.py`, isaac_env python with the `/usr/bin/python3` fallback, live-stack
tests skipped): 34 pass, 3 fail. The 2026-09-17 build had only had its three pairing tests run.

| failure | what it was | fixed here |
|---|---|---|
| `test_geometric_hover` §7 (safe controller, 6 m/s crosswind) | **real**: hover yaw authority 0.74 N·m < the fin's weathercock 1.87 N·m; the yaw demand saturates the mixer and the aircraft flies away (76 m) | **no** — left failing; the safe controller's yaw policy is its own piece |
| `test_hover_hold` (cascade hover yaw) | the bar was 0.05 of diagonal split in **throttle** units, set at cQ 0.12; ×4.4 on the new build while the yaw held to 0.04° and the torque chatter was 0.046 N·m (old 0.049) | bar in N·m (0.16 = the old bar exactly); the 2026-09-17 "OSCILLATES" is withdrawn |
| `test_plant_knobs` | neutral `--plant_cq 0.12` as a literal | read from the airframe file |
| *(silent)* `tests/plant_double.py` | got the 36.3 N prop on 09-17, kept `ROTOR_MOMENT_RATIO 0.12` — unpinned | paired (0.0190, tilt slew 106) and **pinned** in `test_prop_pairing` (+ cQ / tilt-rate airframe↔controller), mutation-checked |

### The finding

Hover yaw authority at the measured cQ: 4 × 9.7 N × 0.0190 = **0.74 N·m** (was 4.65). The plant's aero at 90° of
sideslip: **1.87 N·m at 6 m/s**, 0.47 at 3. Consequences, measured:

| controller, hover in a pure crosswind | 3 m/s | 4 | 4.5 | 5 | 6 |
|---|---|---|---|---|---|
| safe controller, heading hold (shipped) | 0.00 m, yaw err 12° | 0.00 m, 19°, mixer sat. 1% | **29 m flyaway**, sat. 96% | 71 m | 77 m |
| safe controller, yaw damping only (weathervane) | 0.00 m, nose −90° | 0.00 | — | — | **0.00 m**, integral 1.3 N (was 13) |
| cascade `HoverHold` (`hold_roll_max_deg` 8) | within 2.6 m, heading held | drifts 0.9 m/s | — | 1.9 m/s | 2.8 m/s (headwind 6: 1.1 m) |

### The robustness table, re-sized to the measured build (`scripts/cells_mn4014.txt`)

`~/flight_bags/robust/mn1`, 23 cells × 16 seeds, the flown law (`nmpc_cascade_imu_node.py`, `--est mdob --l1
--ttr 10 --lat 0.03 --comp 0.049 --slosh --noise_acc 0.02`), ~1 min per cell on this machine.

| parameter | passes | fails |
|---|---|---|
| J0 (model 0.52) | 0.42, 0.47, 0.57 | **0.62 (pk 18.5), 0.71 (19.9)** — one-sided: −0.10 free, +0.05 costs 2.6°, +0.10 fails |
| thrust scale | 0.80 … 1.20 (all) | — |
| cQ | 0.5x … 2x (all; corridor peak 13.9 at every level) | — |
| spin lag (model 0.067) | 0.10, 0.13, 0.16 pinned, 0.16 told (pk ≤ 14.5) | — |
| CG cold start | +1.5, +3 cm | **−1.5 cm: 0/16** (piece 2's row, unchanged) |
| inertia | 0.8, 1.25 | — |

**Solve-failure column repaired.** Every `ok=0` of every cell but one sat at t 71–76 s, V 0.3–0.5 m/s: the
outer's post-stop hover solve, a single 40–50 ms tick, swinging 1–17 between identical flights. `ke_metrics`
now also prints `flight fails` (before the stop) and `robust_metrics`' budget reads it (older summaries fall
back to the total). Re-scored `act1`: the 2026-09-16 rows "spin 0.14 / 0.16 pinned: over the budget (5–6)"
were **0 in flight** — below 0.18 s there is no in-flight cost, told or not; the cliff at 0.18 and the spec
stand; "if not measured the tolerance narrows to ~0.14" is withdrawn. The 2026-09-17 "fails 2 → 6" is 0 → 0
in flight.

### Reproduce

```bash
# the suite (from the sim root; a test that fails under isaac_env is retried under /usr/bin/python3)
for f in tests/test_*.py; do ~/envs/isaac_env/bin/python3 $f > /tmp/$(basename $f .py).log 2>&1 || echo "FAIL $f"; done

# the weathercock numbers and the safe-controller A/B (plant double, 40 s, wind 0,W,0)
~/envs/isaac_env/bin/python3 tests/test_geometric_hover.py            # §7 fails: the flyaway
#   yaw-policy A/B: tests/test_geometric_hover.py's test_wind with gains.kR_yaw = 0 (damping only) holds 0.00 m

# the cascade hover hold in a crosswind (controller repo root)
~/envs/isaac_env/bin/python3 px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_imu_node.py \
  --test --est mdob --ttr 10 --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1 --noise_rate 1.0 \
  --noise_acc 0.02 --l1 --hover_only --dur 30 --every 2.0 --seed 3 --wind 0,6,0

# the yaw-gain grid (a test, not a change) and the re-armed hover-hold test
~/envs/isaac_env/bin/python3 tests/test_hover_hold.py [--kR_yaw K --kW_yaw K --dur 20 --yaw0 30]

# the robustness table on the measured build, and its re-scoring
ROBUST_NODE=nmpc_cascade_imu_node.py \
  ROBUST_LINE="--est mdob --l1 --ttr 10 --lat 0.03 --comp 0.049 --slosh --noise_acc 0.02" \
  bash scripts/robust_sweep.sh mn1 scripts/cells_mn4014.txt
~/envs/isaac_env/bin/python3 scripts/robust_metrics.py ~/flight_bags/robust/mn1
```

### Standing (replaces the 2026-09-17 list's item 1 and 4)

1. **Safe controller yaw policy on the measured cQ** — mixer priority (collective/roll/pitch first, yaw with
   the remaining headroom) and a weathervane in hover; gate = `test_geometric_hover` §7 re-armed as "holds
   position with the nose free" plus a heading-hold check within authority at 3 m/s. Safety-relevant.
2. **Cascade hover hold in wind** — the 8° roll clamp limits it to ~3.5 m/s of crosswind; weathervane or a
   drift-sized lean; gate `--hover_only --wind`.
3. Re-run the piece-4 wind table on the measured build (stops and hover ends were at 6× the yaw authority).
4. B2: the gate's plant needs its own tilt-servo τ / rate knobs before a servo mismatch can be gated.
5. Measure J0 (a fan on the stand at 10–16 m/s, or the first flight bag's cruise throttle); until then carry
   the controller's J0 on the high side of the estimate.
6. `vtol_nmpc.py`'s baked trim table (stale, not flown) and the AS5048A encoder read — unchanged from 09-17.

---

## 2026-09-23 (evening) — NMPC + INDI flies the full HIL mission; the first attempt was the harness

### The name
The GS called the INDI controller "INDI NMPC + Geo". In `full` mode there is no geometric law, so it is
now **"NMPC + INDI"** (outer + inner, like the other cascades). `controller_id cascade_indi` is unchanged.
`INDI_MODE` (`aug` keeps the geometric law, `full` replaces it and installs the flare-8 reference) is a
launch choice, printed as `mode=` on the banner and checked by READY.

### Why the first flight "did not work"
INDI never flew it. The stack READY certified was torn down during the mission script's EKF settle; the
script carried on with a second, unverified stack, and two drivers (one orphaned from the killed run)
flipped `active_controller` 87 times in 47 s. Five fixes in the harness (sim 73c993b): fly only the
stack whose tmux `session_created` predates READY, re-checked throughout; refuse while any other
publisher is on `active_controller` or another driver is alive; kill the driver with its children and
on the script's own signals; `kill_hil.sh` knows the drivers; the wrapper keeps its own log.

### The re-flight (row indi_full_mission2_a1_124609)
Bag switch history: geometric → cascade_indi (85 s) → geometric, three changes; the INDI status topic
1064 lines, the other cascades none. Transition 0 → 16.4 m/s in 10 s at z 30.4–31.0; 56 s of cruise at
16.3 m/s, pitch +6.7, roll ≤ 0.6, 0 solve failures; back-transition pitch peak +11.6 (gate 12.25) at
z 30.7–30.9, one solve failure at the stall crossing; landed 0.00 m. The cruise tilt wandered
83.5–89.6° (gate: INDI 89.5 vs geometric 90.0 — the outer backs the tilt off against a small overspeed),
which is why `night_metrics` reports only 12.7 s wing-borne (its cut is tilt ≥ 88). One flight; the
turning mission, wind and the hover-hold creep are still unflown on INDI full.

---

## 2026-09-23 — The reference comes with the loop, and which terms move

### What was wired

`--indi_mode full` now installs **two** seams, not one: `base.MOMENT_LAW` (the law) and
`base.REFS_FN` (the reference, `nmpc_cascade_indi_ref`, flare 15 → 8). `aug`/`off` install
neither, so their identity hashes still hold (`4a557a30…`, `03d9a349…`, verified).

```bash
# per-mode check
INDI_MODE=full  → law=INDI      REFS_FN=build_refs_indi
INDI_MODE=aug   → law=geometric REFS_FN=build_refs_tank
INDI_MODE=off   → law=geometric REFS_FN=build_refs_tank
```

The banner carries `mode=` plus a reference line, and `start_slosh_hil.sh` **refuses READY**
if a full-mode node never announced its reference, or if an aug/off node announced one — a bag
that cannot say which reference flew cannot be reproduced. `--indi_flare -1` is the A/B.

**The body is delegated, not copied** — the tank node's own argument. The file owns INDI's
reference *parameters*; fork the body only for a structurally different schedule.

### Does any other term have a per-loop optimum?

```bash
scripts/ref_loop_cells.sh      # 40 runs, 2 seeds, each loop on its own shipped reference
scripts/ref_loop_metrics.py    # asks whether the OPTIMUM moves, not whether each flies
```

**`ttr` — the strongest form of the finding.**

| ttr | 6 | 8 | 10 | 14 | |
|---|---|---|---|---|---|
| geometric corridor peak | 7.78 | 8.90 | 9.89 | **10.79** | 3.0° of sensitivity |
| INDI corridor peak | 7.37 | 7.51 | 7.72 | **7.82** | **0.45° — flat** |
| **commanded peak, both loops** | **8.00** | **8.00** | **8.00** | **8.00** | the outer's plan does not move |
| geometric delivered − commanded | −0.22 | +0.90 | +1.89 | **+2.79** | |
| INDI delivered − commanded | −0.63 | −0.49 | −0.28 | −0.18 | always under |

The commanded peak is 8.00° at every `ttr` for both loops, so **the whole corridor-peak
sensitivity to `ttr` is the inner loop's own tracking error**. The 2026-09-13 retune of
ttr 20 → 10 "to get the corridor peak under the 10° CL peak" was buying back the geometric
loop's overshoot. Not shipped: `ttr`'s other justifications (turbulence envelope, throttle
headroom at the heavy mass) were measured on the flown loop and in wind; z, V max and solve
failures do not move with `ttr` for either loop.

**`decel_rate` — the sign reverses.**

| decel_rate | 1.5 | 2.0 | 2.5 |
|---|---|---|---|
| geometric BT peak | 12.25 | 13.42 | **15.62** (slower is better) |
| INDI BT peak | 12.23 | 11.21 | **11.02** (faster is better) |

Both stop, neither reverses, z ≥ 29.9. Not shipped: it is a ROS parameter on the base node
(the GS owns that surface), a faster back-transition is safety-relevant, 2 seeds, still air.

**`th_slew_bt` — the control, predicted not to move, and it didn't.**

| th_slew_bt | 4 | 8 | 16 |
|---|---|---|---|
| geometric / INDI BT peak | 12.33 / 12.22 | 12.25 / 12.23 | 12.23 / 12.22 |

0.10° for the geometric loop and 0.01° for INDI across **four times** the rate. Its stated
reason is that the *outer solver* overshoots a step — an outer property. This control is what
makes the other two readable; without it the claim would be unfalsifiable.

### The rule the three cells write

> A shaping term is loop-dependent **exactly when the reason it was given is about the
> delivered trajectory.**

Not loop-dependent: terms justified by the outer's numerics (`th_slew_bt`), by which actuator
can act at all (`w_aft`/`w_flare`), or by the airframe (`aoa_cap`, `Schedule.at(V)`). And the
one shaping term needing no per-loop value is the one that **identifies the closed loop in
flight** — κ re-learned 0.799 → 0.837 across the swap on its own.

### The sign is phase-dependent, not a bias

The geometric loop **overshoots** in the corridor (+2.79° where q̄ is low and the airframe's
spring is 1–9 N·m/rad) and **undershoots** the flare (−2.49° where it is ~16.5): at high aero
stiffness the airframe's own spring resists the commanded change. INDI is the other way round
(−0.18 / +1.66) and its corridor error correlates with nothing (0.22 q̄, −0.09 θ_ref) — a
control-effectiveness error, consistent with the allocation's rotor (67 ms) / elevator (30 ms)
split moving with q̄, the same mechanism that made the textbook increment k = 1 lose the
aircraft on 2026-09-21.

### The deepest version, part-measured

The argument applies one level up, to the outer's **model**: `build_ode6` carries `cma`, and
INDI removes that stiffness from the plant the inner presents, so the outer plans against a
plant that no longer exists. *Real*: pitch INDI in the augmented form crashed the corridor at
t 9.7 s (2026-09-21). *Doesn't bite here*: with the full form on the corrected elevator path
the corridor is INDI's best phase (7.66 vs 9.83, tracking 24 % tighter). The reconciliation —
and why the reference is where the fix lands — is that the outer's pitch channel issues a slow
trim **command**, which the inner delivers whatever the plant's stiffness; the stiffness only
decides how that command must be **shaped**.

## 2026-09-22 (later) — The elevator sign, and what the reference owes the inner loop

### The defect the audit found

`inner_step`'s `de_damp` divided by `qbarS * C_WING * CMDE * DE_MAX`, and `CMDE` is **negative**:
`My` flips under FLU→FRD exactly as `Mz` does, so the elevator can no more use `CMDE` directly
than the rudder can use `CNDR`. The 2026-09-11 rudder fix carries twelve lines about this exact
trap and stopped at the line above it. Fixed to `(-CMDE)`.

Found by `scripts/audit_inner_loops.py`, not by a flight: four of its six pitch conditions had a
`+pitch` error producing a `+pitch` **acceleration**, and the live allocation's measured pitch
effectiveness read **−24.01 N·m/unit where the plant and `model_moment` both give +24.02**.

```bash
# the audit fails on the old line and passes on the fix
~/envs/isaac_env/bin/python3 scripts/audit_inner_loops.py
```

| flown line, 16 seeds, straight mission + BT | before | after |
|---|---|---|
| corridor peak | 13.9° | **12.4°** |
| pitch error rms | 2.69° | **2.33°** |
| the outer's corridor pitch kick | 4.4° | **1.3°** |
| back-transition peak / reversals | 12.0° / 8 | **10.9° / 5** |
| 5 cm CG hover, elevator chatter sd | 0.079 | **0.000** |
| landed / stop / in-flight solve failures | 16/16, 8.8 s, 0 | 16/16, 8.8 s, 0 |

**New identity references** (md5sum of the `--log` CSV on the flown gate line): IMU
`4a557a30c2b84f927389ae264b83820e`, steso `03d9a349f5ce4db947cf39010692d212`. Whole headless
suite: 37 pass, 2 skipped (the two needing a live stack).

### Does the reference have to change for the INDI loop?

The reference is **two kinds of object**. `Schedule.at(V)` — (Tf, Tr, β, de_trim, θ_ref) — is a
force and moment *balance of the airframe*: invariant, the outer NMPC owns it, no inner loop
touches it. The **shaping** (flare, aft-tilt gate, speed ramp, `hold_k`) was fitted against a loop
that does not obey it exactly, so it encodes *that loop's tracking error*.

```bash
# scratchpad/refprobe.py wraps base.inner_step (the seam) and records theta_ref vs theta per tick
refprobe.py geo|indi out.csv -- <gate flags>        # the shipped nodes are untouched
bash scratchpad/refseries.sh                        # 30 runs: full reference, cap, hold_k
```

| phase | ref θ | geometric e = θ−θ_ref | INDI full |
|---|---|---|---|
| hover / engage | 0.00° | **−1.21°** (rms 1.33) | **+0.01°** (rms 0.14) |
| corridor (V 2–14) | +5.44° | +0.59° | −0.92° |
| cruise (tilt 90) | +6.80° | +0.24° | +0.00° |
| back-transition | +8.31° | −0.18° (rms 2.56) | +0.68° (rms 1.60) |

That standing error is **force the reference was getting for free** (`T_total·sin e`): hover
**1.74 N = 0.44 m/s²**, corridor 1.71 N, back-transition 0.81 N. The hover row *is* the 6.3 m HIL
hover-hold creep in newtons.

**Why it is loop-dependent, in one number.** At 16 m/s the airframe's own pitch stiffness
`q̄·S·c·|CMA|` is **16.46 N·m/rad** against the inner loop's own 3.0 — the *wing* supplies 85 % of
the stiffness the reference is tracked with (74 % at 4 m/s, 42 % at 8, 24 % at 12). INDI inverts
that away and replaces the total with `J·ωₙ² = 2.91 N·m/rad`: same gains, ~6× less total pitch
stiffness. (16.5 N·m/rad is exactly the plant's pitch stiffness recorded at the 2026-09-06 aero
fix — an independent cross-check of the probe.)

The two loops' errors are also physically different kinds — the correlations separate them:

| corridor error vs | d(θ_ref)/dt | q̄ | θ_ref | V |
|---|---|---|---|---|
| geometric | −0.11 | **0.61** | **0.88** | **0.74** |
| INDI | −0.13 | 0.22 | −0.09 | 0.10 |

i.e. the geometric error **scales with the flight condition** (it shares the job with the
airframe's spring); INDI's is a **constant offset** (a control-effectiveness error in the
inversion — the pitch axis is not one G).

### The series, and the one term that must change

| straight mission + BT, 6 seeds | corridor pk | cor e rms | BT peak | revs >3° | z min | V end | fails |
|---|---|---|---|---|---|---|---|
| geometric (flown law) | 9.83° | 1.26° | **12.26°** | 0 | 30.00 | 0.02 | 7 |
| INDI full | **7.66°** | **0.96°** | **16.46°** | 0 | 29.80 | 0.01 | 10 |

Both fly the full reference 6/6. **The cap is not the lever** — 8 / 7 / 6 move the delivered peak
only 0.6° per cap-degree and leave the ~1.5° overshoot untouched (it is additive, not a scale).
The schedule says why: `th_max = aoa_cap + rel·flare` — the cap sets the floor, the **flare sets
the peak**.

| INDI, `--flare` | 15 (flown) | 12 | 10 | **8** | geometric at 15 |
|---|---|---|---|---|---|
| ref peak / delivered | 14.76 / 16.42 | 13.27 / 14.63 | 12.30 / 13.45 | **11.33 / 12.25** | 14.76 / **12.27** |
| reference error rms | 1.59 | 1.44 | 1.32 | **1.21** | 2.56 |
| z min | 29.80 | 29.87 | 29.90 | **29.90** | 30.00 |

**`flare_deg` 15 → 8** puts the INDI back-transition on the flown loop's own trajectory (12.25 vs
12.27°) with half its reference error, zero reversals and the same stop. *Not applied*: the flare
is the schedule's parameter (the tank node reads it before the INDI module is imported) and 15 is
flight-validated for the flown loop. It is a live ROS parameter — set it on the INDI pane; the
number and its derivation are in the INDI node beside the `MOMENT_LAW` install.

### The term that needed no change is the one that measures itself

The orbit's bank feed-forward `atan(V²/(g·R·κ))` is by construction a model of the **closed loop's**
turn rate per bank, so it must move with the inner loop — and it does, on its own: the turning
mission (3 seeds) learns **κ 0.799 under the geometric law and 0.837 under INDI** (+4.8 %, ~19 000
samples each), RTH stop 6.20 vs 5.67 m, wing-borne pitch tracked 4.6× tighter (e rms 1.046 vs
0.225). The 2026-09-13 adaptive feed-forward was built to close a 3.4 m radius error; what it
actually bought is a reference term that survives a change of inner loop.

**A shaping term fitted to one loop's error is a hidden dependency on that loop; one that
identifies the closed loop in flight is not.**

**CONFIRMED END TO END: INDI at `--flare 8` flies the FULL GUIDANCE reference 3/3** (legs, two
90 deg fly-by turns, RTH): **1-2 solve failures in 4301**, kappa 0.835-0.838 (the flare retune does
not perturb it), the RTH stop **4.2 / 4.2 / 4.5 m from home** against the geometric loop's 6.20 and
un-retuned INDI's 5.67 -- i.e. the reference change costs nothing on the guidance half and the stop
improves. Tooling persisted (the /tmp rule): `scripts/ref_probe.py`, `ref_metrics.py`, `ref_series.sh`.

### What the gate cannot settle, and one trap

The **hover hold**: on the gate INDI is *better* at every gain (`hold_k` 0.5 / 0.8 / 1.2 → 0.99 /
0.67 / 0.50 m against the geometric loop's 1.22 / 0.81 / 0.61) — the opposite of the HIL creep, for
the reason already recorded: `--hover_only` starts at rest with the hold latched while HIL's engage
hover starts with residual speed. The 0.44 m/s² is real; the `hold_k` retune it implies can only be
settled on the HIL engage hover.

**The trap** (it cost the probe's first run): `import nmpc_cascade_node` with the package dir *also*
on `sys.path` gives a **second module object**, so the variant's `base.MOMENT_LAW = …` lands on a
copy the gate never reads — the INDI mission came back byte-identical to the geometric one and
looked like a null result. Import package-qualified, the way the nodes import each other, and
assert the law is installed before trusting a run.

## 2026-09-17 — The first measured propulsion: MN_4014 + 15×5, and a yaw-gain casualty

**sim `<this commit>` · controller `airframe_id.cfg` · NEW identity references**

### What changed, and where each number came from

| constant | was | now | source |
|---|---|---|---|
| `max_thrust` | 18.0 N | 36.3 N | thrust stand, extrapolated to 100 % from ≤ 80 % |
| `omega_max` | 749 rad/s | 885.8 | thrust stand |
| `tau_spin` | 0.12 s | 0.067 s | thrust stand, **at the hover point** |
| `cq` | 0.12 m | 0.0190 m | thrust stand, identical from both motors |
| prop | 12×8, J0 0.72 | 15×5, J0 0.52 | UIUC wind-tunnel database |
| tilt `tau` / `rate_deg_s` / `max_deg` | 0.12 / 80 / 90 | 0.0756 / 106 / 100 | specified by the user |

Four paired copies moved together: `vtol.py` (`quad_tiltrotor_config` + the `PROPS` catalogue),
`tests/plant_double.py`, the controller's `airframe_id.cfg`, and the catalogue's J0 for every entry.
`test_prop_pairing.py`, `test_tilt_limit_pairing.py` and `test_tank_model_pairing.py` all pass.

### Reproduce

```bash
# the rotor fit (tau, omega_max, max_thrust, cq) from the bench logs
~/envs/isaac_env/bin/python3 scripts/ana_spin_lag.py --dir identification/2026-09-17_rotor_MN4010_MN4014/MN_4014
~/envs/isaac_env/bin/python3 scripts/ana_spin_lag.py --selftest      # validates the fitter first

# J0 from published wind-tunnel data (downloads 87 MB, md5 a41e484f1fd0fb6ff80b76e27410808b)
~/envs/isaac_env/bin/python3 scripts/ana_prop_j0.py --db <UIUC-propDB> --min_d 10 --pd 0.3333

# the regressions
KE_DIR=~/flight_bags/ke_sweeps KE_NODE=nmpc_cascade_imu_node.py bash scripts/ke_sweep.sh \
  reg_mn4014 1 16 --est mdob --l1 --ttr 10 --lat 0.03 --comp 0.049 --slosh --noise_acc 0.02
KINDS="imu-mdob" PREDS="on" bash scripts/est_sweep.sh mn4014_turns turns 1 3 --noise_acc 0.02
~/envs/isaac_env/bin/python3 tests/test_hover_hold.py                # the yaw gate -- see below
```

### Result

| | 12×8 baseline | MN_4014 + 15×5 |
|---|---|---|
| straight line, 16 seeds | 16/16, pk 14.2°, fails 2, stop 8.8 s | **16/16, pk 13.9°, fails 6, stop 8.8** |
| turning mission, 3 seeds | 3/3, bob orbit 0.058 N, hits 0, roll sd 0.150°, u_ad 0.006 | **3/3, 0.049, 0, 0.154, 0.006** |
| hover yaw gate | shipped gains **OK** | shipped gains **OSCILLATE** — *withdrawn 2026-09-21: a throttle-unit bar; the loop is clean* |

### The one casualty, and how it was attributed

The measured `cq` cuts the maximum hover yaw moment 3.99 → 0.74 N·m, i.e. **466 → 86 deg/s² of yaw
acceleration**, and `test_hover_hold.py` turns: the shipped kR_yaw 5.0 / kW_yaw 2.0 oscillates, and
the highest gains that hold a clean hover are **3.0 / 1.5**. A/B'd by putting `cq` back to 0.12 and
re-running — the shipped gains pass there — so it is the cQ and nothing else in the build.
`vtol.py`'s own comment had always said `0.12 (raised for MC yaw authority; sim-only, real props
~0.02)`; the bench says 0.0190 and the yaw loop had been tuned against 6× the real authority.
**Not retuned here** — a controller gain change is its own piece with its own gate.

Neither mission exposes it, because above ~10 m/s yaw is the rudder's job and the rotor differential
is not the actuator. The exposure is hover and the low-speed ends.

### Method notes worth keeping

- **Spin-up vs spin-down is an averaging artefact, not an asymmetry.** Whole-range medians read
  0.064 s up / 0.532 s down; spin-down is drag-driven and drag ∝ ω², so its constant scales with 1/ω
  (0.53 s at 20→10 % throttle, 0.057 s at 80→70 %). At the hover point the two agree (0.064 / 0.072).
  Always take τ where the aircraft lives.
- **The dead time (81–88 ms) is unusable.** The stand's `ESC signal (µs)` column is empty, so it
  cannot be separated from clock offset between the PX4 logger and the stand. *Next bench session:
  log the ESC PWM on the stand.*
- **A pairing test's invariant can be wrong in the safe direction.** `test_tilt_limit_pairing.py`
  demanded the servo's travel *equal* the controller's planned 90°, so +100° of headroom read as a
  mismatch. The correct invariant is containment; the dangerous case is the reverse, and it has
  happened (the outer planned −25° of aft tilt on a servo that stopped at −20).

### Standing

1. ~~Retune the hover yaw gains for the measured cQ (the gate already says 3.0 / 1.5) and re-gate.~~ *Closed 2026-09-21 without a retune — the test's bar, not the loop. The real cQ casualty is heading-hold in wind (see that entry).*
2. `vtol_nmpc.py`'s **baked** trim table is stale against the new propulsion. Not in the flown path
   (the cascade's `Schedule.at` is live Python), but regenerate before that node ever flies.
3. Wire the AS5048A tilt encoder into the node — β becomes measured rather than integrated from the
   command, which closes the one gap the sensor audit could not put a tolerance on.
4. The straight-line solve failures went 2 → 6 over 16 flights. Small, named, not chased. *2026-09-21: all post-stop hover ticks; 0 → 0 in flight.*

---

## 2026-09-16 (evening) — The actuator-lag tolerance on the flown control law

**sim `3f53a14` · controller `a5a6a16` untouched (no identity hash moved)**

### Question

Piece 2 left the rotor spin lag as a cliff with nothing measured between the modelled 0.12 s and the
failing 0.20 s, and its surface-lag row was never a mismatch test. The design question behind it: the
L1 augmentation is meant to absorb actuator error — how much of it actually does?

### What changed

| file | change |
|---|---|
| `scripts/robust_sweep.sh` | the control law is a knob: `ROBUST_NODE` / `ROBUST_LINE` (defaults are piece 2's exactly, so `env1` reproduces); writes `<tag>/law`; resolves a relative cells-file path **before** it `cd`s into the controller repo |
| `scripts/cells_actuator_lag.txt` | new — the 16 paired cells |
| `CLAUDE.md` | the result block + a pointer on piece 2's two superseded rows (left as written, per the house rule) |

**The bug that fix closes:** `robust_sweep.sh` `cd`s into the controller repo early, so a relative
cells path resolved against the wrong repo. The sweep ran **zero cells, printed `done`, and exited 0**;
only `robust_metrics.py` noticed, via a missing `cells.txt`. A sweep that runs nothing must not look
like a sweep that passed.

### Experiment design — the cells are paired, and the pair *is* the experiment

```
--tau_spin X --l1_tau_spin 0.12    the PLANT moves, the controller's model is PINNED  -> a real mismatch
--tau_spin X                       the plant moves, l1_tau_spin FOLLOWS               -> the controller is TOLD
```

The pair separates *"the model is wrong"* from *"the actuator is too slow to fly"* — which no single
cell can do. Checked before the run: the actuator lags live **only** in `L1Aug.applied_moment`
(`nmpc_cascade_steso_node.py:269` passes them through); `slosh_predictor.py` carries none, so pinning
is a complete mismatch with nothing else quietly told the truth.

### Reproduce

```bash
# the flown "Adaptive NMPC + Geo" law: imu-mdob + EKF + L1        (16 cells x 16 seeds, ~25 min)
ROBUST_NODE=nmpc_cascade_imu_node.py \
ROBUST_LINE="--est mdob --l1 --ttr 10 --lat 0.03 --comp 0.049 --slosh --noise_acc 0.02" \
  bash scripts/robust_sweep.sh act1 scripts/cells_actuator_lag.txt
~/envs/isaac_env/bin/python3 scripts/robust_metrics.py ~/flight_bags/robust/act1

# the bridge on piece 2's own law (defaults: the steso node, velocity force twin)
bash scripts/robust_sweep.sh act1_steso <cells: nominal, spin_0.20, spin_0.20_told>
```

Mission and pass rule are piece 2's: the straight mission, back-transition at 60 s, y0 10 m, noise
0.1° / 1°/s, CG tank; PASS = 16/16 landed, solve-fails sum ≤ 4, corridor peak ≤ nominal + 3°,
z ≥ 29.0 m, stop ≤ nominal + 3 s. Never while a flight is in the air.

### Result — `~/flight_bags/robust/act1`

| cell | landed | corridor pk | fails / 16 flights | verdict |
|---|---|---|---|---|
| nominal | 16/16 | 14.2 | 2 | PASS — matches piece 2's nominal (16/16, 14.3, 3) |
| spin 0.06, model 0.12 | 16/16 | 13.9 | 1 | PASS |
| spin 0.09, model 0.12 | 16/16 | 14.0 | 2 | PASS |
| spin 0.14, model 0.12 | 16/16 | 14.2 | **5** | flies; over the solve budget |
| spin 0.16, model 0.12 | 16/16 | 14.3 | **6** | flies; over the budget |
| **spin 0.16, TOLD** | **16/16** | 14.3 | **1** | **PASS — measuring removes the whole cost** |
| spin 0.18, model 0.12 | **11/16** | 14.2 | 4 | FAIL (5 crashed before the BT) |
| **spin 0.18, TOLD** | **15/16** | 14.4 | 5 | FAIL, just — the knee |
| spin 0.20, model 0.12 | **0/16** | — | — | FAIL (all 16 before the BT) |
| **spin 0.20, TOLD** | **4/16** | 14.3 | 2 | **FAIL — the truth buys 4 seeds, not the flight** |
| spin 0.30, model 0.12 | 0/16 | 34.5 | 6 | FAIL |
| **spin 0.30, TOLD** | **0/16** | — | — | **FAIL — the truth buys nothing** |
| surf 0.045 / 0.06 / 0.10, model 0.03 | 16/16 | 14.3 / 14.3 / 15.3 | 1 / 1 / 0 | **PASS, all three** |
| surf 0.10, TOLD | 16/16 | 15.5 | 1 | PASS |

Bridge, `~/flight_bags/robust/act1_steso` (piece 2's law): nominal 16/16, spin 0.20 pinned **0/16**,
spin 0.20 told **3/16**.

### Reading

1. **Two regimes, boundary 0.18 s.** Below it the cost is the MISMATCH and identification removes it
   outright (0.16 pinned: 16/16 at 6 solve failures; 0.16 told: 1 = nominal). Above it the cost is the
   ACTUATOR: told the true lag, 0.20 still loses 12 of 16 and 0.30 loses all 16.
   **So the L1's MODEL covers the band up to ~0.16 s once measured — which is why `l1_tau_spin` is a
   parameter and not something the adaptation discovers — and past 0.18 s nothing covers it.**
2. **Why knowing stops helping, in one number.** Pitch authority in the transition corridor *is*
   front/rear differential thrust (the wing carries nothing at 39° of tilt and 5 m/s), and its
   bandwidth is 1/τ: **8.3 rad/s at the modelled 0.12, 6.3 at 0.16, 5.0 at 0.20 — against a 8.9 rad/s
   short period.** At 0.12 the actuator is already only *at* the mode it must control. No augmentation
   stabilises a mode faster than its actuator.
3. **The failure is identical in both cases and it is the corridor** (spin 0.20, seed 1, t = 5–7 s,
   V 4–7 m/s, tilt 39°): pitch +23° (told: +14.7 then +58), elevator saturated at −1.00, outer solve
   13 → 45 ms with `ok=0`, then roll 156° / yaw 180° and the ground. **Never at the back-transition** —
   every loss in this sweep is before it, the same place piece 2's CG and control-derivative cells fail.
4. **The estimator is irrelevant to it.** Bridge 0/16 and 3/16 vs the flown law's 0/16 and 4/16 — one
   seed of scatter. The limit is a property of the airframe and the inner loop, so piece 2's spin row
   transfers to the flown law unchanged (and its "0.20: 1/16" was that same seed).
5. **★ Correction to piece 2's surface row.** Its surf cells never pinned `--l1_tau_surf`, so the
   controller was told the true servo lag — never a mismatch test. Run as one: 0.045 / 0.06 / 0.10 all
   pass 16/16 with the controller believing 0.03. 3.3× the modelled surface lag costs **one degree of
   corridor peak and nothing else**; the old "0.06, 5 fails, marginal" was seed scatter.

### The spec it writes (for the thrust-stand campaign)

- Measure `tau_spin`, set `l1_tau_spin` to it. Then the airframe requirement is **τ_spin ≤ 0.16 s**
  (0.18 marginal, 0.20 unflyable at any model value) — i.e. rotor bandwidth ≥ ~6 rad/s, 0.7× the
  short period. Unmeasured (model left at 0.12) the tolerance narrows to ~0.14 s.
- Fit the lag on **ω**, not on thrust: the plant lags ω and then squares it, so fitting the raw thrust
  trace gives a systematically different constant. Fit √T if ω is not logged.
- Spin-**up** and spin-**down** separately (drag brakes differently than the motor accelerates; the
  model has one constant — if they differ materially that is a finding, not an average).
- At the **hover throttle** — that is where the differential authority the corridor needs is set.
- `PROP_J0` is **not** a static-stand number; it needs airflow.

### Method notes worth keeping

- The pass rule's `fails sum ≤ 4` is summed over **16 flights**, so a FAIL at 5–6 is 0.3–0.4 solve
  failures per flight against nominal's 0.13. Read the *landed* column first and the budget second.
- When an actuator mismatch crashes a gate, run the **told** cell before blaming the estimator: one
  run separates a wrong model from a slow actuator.

---

## 2026-09-16 (afternoon) — The sensor audit

**sim `f6fdbfc` · controller `a5a6a16` untouched · page https://claude.ai/artifact/M836419j6Lg3CGCbBuSUMH**

### What changed

| file | change |
|---|---|
| `scripts/sensor_sweep.sh` | the cells: IMU noise / bias / walk / scale / vibration / lag / gyro bias, the state source (rate, lag, velocity noise), the pitot, and the combinations |
| `scripts/sensor_metrics.py` | the merged table, extended with the **per-axis** bob (recomputed per seed from the gate CSVs) |

The gate knobs themselves (`--vib A,f`, `--acc_walk`, `--acc_scale`, `--imu_lag`, `--bias_gyro`)
shipped earlier with the hover hold, controller `a5a6a16` — so every identity hash still stands.

### Reproduce

```bash
bash scripts/sensor_sweep.sh 1 3                  # all cells, 3 seeds, 3 cells in flight
~/envs/isaac_env/bin/python3 scripts/sensor_metrics.py base hw_imu hw_ekf2 hw_all ...
```

### Result — 30 cells × 3 seeds, `~/flight_bags/est_sweeps/sens_*`

The decisive split is per axis, because the tank's only moment arm is r_x, so M = r_x·F_y:
**y-bob** is the channel the cancellation acts on, **x-bob** has no arm and is the loop's own response.

- **The cancellation is an IMU-only loop and it holds.** y-bob 0.015–0.088 N in **29 of 30 cells**
  (uncancelled 0.77 N) — through 5× the sim's accelerometer noise, 0.5 m/s² turn-on bias, 3% scale
  error, 1 m/s² of vibration, 1 deg/s of gyro bias, a 10 Hz GPS, and the whole hardware set at once.
- **The one hard tolerance is the accelerometer's *declared* delay.** 40 ms undeclared → y-bob 0.364 N
  and ~15 wall impacts; the same 40 ms passed to the delay buffer → 0.084 N, zero hits. Not the
  latency — the mis-declared latency.
- **Everything else is the state source** and lands in the x-bob, never in the cancellation. Decomposed:
  EKF2's 35 ms of fusion lag drives u_ad 0.006 → 0.081; 0.05 m/s of white velocity noise adds the wall
  hits but leaves u_ad and roll at base. A bare 10 Hz GPS 0.1 s late is 10× worse than EKF2 on every
  column (orbit roll sd 1.57 vs 0.20).
- **The realistic hardware aircraft flies the mission**: `hw_ekf2` 3/3, 1 solve failure, stop 5.8 m,
  orbit roll sd 0.57° (HIL's own band is 0.83–1.26°).
- **The velocity twins collapse on the same set** (y-bob 0.58–0.72 of 0.77) where imu-mdob holds 0.039.

### Standing items it produced

- Measure `IMU_DELAY` on the Orin link before the first tank flight (`scripts/ana_imu_latency.py`).
- FFT the first hover bag's `sensor_combined`; treat a peak below ~40 Hz as a mount to fix.
- Keep the loop on EKF2's odometry, never on a raw receiver.

### Correction carried since

`vehicle_attitude` is published on **every IMU sample** (EKF2's output predictor), and
`sensor_combined` at `IMU_INTEG_RATE` — so both run at 250 Hz on an FMU configured that way, while
`vehicle_odometry` publishes inside `if (_ekf.update())`, i.e. at `EKF2_PREDICT_US`. Open: whether the
250 Hz survives the DDS hop (in the v1.16.0 tree every sent topic gets `orb_set_interval(10 ms)` in
`SendTopicsSubs::init`, a 100 Hz ceiling). `ros2 topic hz` on the companion vs `listener` on the FMU
console settles it; if DDS really delivers 250, piece 6's firmware rebuild comes off the list.
