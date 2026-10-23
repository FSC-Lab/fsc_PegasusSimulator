# 0902 T650-AM Whole-Body Flight Tests — Tracking Analysis

Three flights (bags `wb_tracking_20260902_{124652,125222,125951}` + ulogs log_308/309/310),
whole-body DIRECT with the governor streaming compatible transitions, arm on the
`EXTERNAL_WHOLE_BODY` torque pass-through. Analysis tooling:
`analysis/*.npz` (extracted with ROS2-sourced `/usr/bin/python3`), figures + `stats.txt` per
bag under `analysis/<bag>_figs/`.

| flight | what was flown | DIRECT | verdict |
|---|---|---|---|
| 1 (124652) | arm-only bend + return, then a 0.5 m base move | 6.4–103.9 s | **healthy** — converges to 2–4 cm at holds |
| 2 (125222) | hover only | 12.8–14.8 s | **runaway** 0.64 m in 2 s, pilot abort |
| 3 (125951) | combined base+arm move ×2 | 16.0–102.0 s | **parks 0.55–0.65 m off target**, never recovers |

## ROOT CAUSE: the whole Dynamixel link died — in BOTH directions — before flight 2

### The read path

`/uav_0/fsc_open_manipulator/joint_states` is **bit-frozen for the entirety of bags 2 and 3**
— position, velocity AND effort, exactly one unique value per channel over the whole
recording (q = [0, 39.99, 39.99, 0.18]°; eff j2 = 94 ticks in bag 2, 99 in bag 3).

**It published perfectly the whole time.** This is the important part and the reason nothing
caught it:

| | bag 1 (live) | bags 2 / 3 (frozen) |
|---|---|---|
| rate | 250.0 Hz | 250.0 / 249.0 Hz |
| header stamps | monotonic, dt 4.00 ms | monotonic, dt 4.00 / 4.02 ms, **0 duplicate stamps** |
| receive gaps | max 14 ms | max 14 ms / one 338 ms |
| unique position values, j3 | 273 | **1** |
| unique effort values, j2 | 264 | **1** |

A dead topic would have been caught by every guard in the stack. A topic delivering dead
content on a perfect 250 Hz schedule with fresh timestamps is invisible to all of them —
`wb_control_debug[4]` (arm-state fresh) read 1 throughout, and the DIRECT entry gate
(`wb_gate_arm_rad`) passed trivially because frozen-q ≡ home ≡ q_d.

Three facts localize the fault below the flight stack:

1. **All six joints froze simultaneously**, both gripper joints included — that is the shared
   bus/link, not one servo.
2. **The controller loop stayed healthy**: "Loop health" cycle counters advance continuously
   across all three bags (the process never restarted), update() times normal, the smoothed
   joint reference keeps moving (4090 unique values in bag 3, spanning j3 17.5–40°), and the
   arm's own goal-current output keeps varying (21k unique values). Only the *measurement* is
   dead. Its only live signature was the **velocity-observer resync count jumping 12 → 100**.
3. **The frozen snapshot differs between bags 2 and 3** (j1 −0.2637° vs +0.0879°, gripper_left
   −0.4949 vs −0.4573) — so a fresh read *did* succeed just before each recording, then
   stopped. The pattern is a read that works once at activation and then silently keeps
   returning the previous values.

The bag cannot separate a physical link failure (U2D2 / USB / cable / bus power) from a driver
that silently keeps old values when a sync-read packet fails. Both sit below the flight stack.
Note the arm was **already blind on the ground**, before takeoff, in both flights — this was
pre-arm detectable.

### It is not the ROS 2 network, and a process restart does not fix it

Two things the "Loop health" cycle counter settles, since it is monotonic within one process
life:

| transition | cycles | verdict |
|---|---|---|
| flight 1 → flight 2 | 52506 → 105008 (+52502 over 218.7 s ≈ 240 Hz) | **same process, never died** |
| flight 2 → flight 3 | 112509 → **15001**, nominal period 3.8 → 4.0 ms | **process restarted** |

- **The freeze happened inside a running, healthy process.** Nothing crashed, nothing was
  SIGHUP'd, no supervision gap. So it is not a process-lifetime problem.
- **The arm stack was restarted between flights 2 and 3 — and the freeze came straight back**
  (fresh snapshot captured at activation, then frozen again for all 143 s). Restarting the
  software is therefore already known, from this data, not to clear this fault.
- **The ROS 2 / DDS network is exonerated.** A flaky network shows up as gaps, jitter and
  dropped samples. What was recorded is 250 Hz delivery, monotonic publisher-generated header
  stamps with zero duplicates, and 14 ms max receive gaps. A network cannot manufacture fresh
  timestamps around stale payload — the publisher was building new messages on schedule and
  had nothing new to put in them. The failed link is the **Dynamixel serial bus (U2D2/USB)**,
  which is not a network link at all.

### The write path died too — the arm delivered nothing

The joint-channel GMO (`wb_control_debug[37..40]`) converges to ≈ −tau_joint in flight 3
(ratios 0.87–1.08 on j1/j3/j4), but **that proves nothing**: those channels are built from the
same frozen measurement, so they book −tau whether or not the arm applies it. It is not a
discriminator, and should not be read as one.

The **base-side** GMO channels are, because they come from real measured base motion. If the
law pre-compensates the arm reaction (`N1ᵀu3`) and the arm never delivers it, the base carries
an uncancelled moment that must track the commanded joint torque:

| flight | corr( \|tau_joint\|, \|d_r\| ) over DIRECT | slope [N·m base per N·m joint] |
|---|---|---|
| 1 (live feedback) | **−0.23** | −0.13 |
| 3 (frozen) | **+0.991** | **+1.08** |

One-for-one, at correlation 0.99. Every N·m the law commanded to the arm reappeared on the
base as unexplained moment, growing to **\|d_r\| = 2.2 N·m** by the last hold. A servo that is
powered but *stalled* still transmits its reaction to the structure, so this is not stiction —
no torque reached the arm at all. Combined with all six reads failing at once, the link was
dead in both directions and the servos held their last-written goal current (Dynamixel has no
comms watchdog by default), which is what kept the arm near home and the vehicle flyable.

### What each flight then did

- **Flight 2**: DIRECT entered clean (CoM anchor error 3.8 mm) and ran away: |e_com|
  3.8 → 634 mm and |e_R| 0.07 → 0.46 rad in 2 s, re-seed at (−0.81, −1.23) m, yaw +18°.
  This is exactly the η ≈ 0 case of the 0901 stability screening (η ≤ 0.6 diverges in ~5 s);
  it diverged in 2. The local PD hold had also been blind since the freeze.
- **Flight 3**: survived because the latched goal current kept the arm near home, so the
  frozen q ≡ home happened to match the physical configuration and the CoM model stayed
  right. But the law spent the flight commanding an arm that could not answer: with
  q_d → [0.3, 40.2, 17.5, 12.9]° and believed q stuck at home, **q−q_d parked at +22.5° on j3
  and −13.0° on j4**, the joint command wound to 1.4 / 2.1 N·m, and every bit of the matching
  `N1ᵀu3` base pre-compensation was uncancelled — 2.2 N·m of standing body moment, e_R
  0.23–0.35 rad, and the vehicle parked **(−0.39, −0.57) m** off. That is the "huge
  steady-state error when I move them both", and it is quantitatively accounted for.

**This is not a gain-tuning problem.** Flight 1 — same gains, same trajectory class, live
feedback — tracked and converged, and its base-side correlation is flat (−0.23).

Recommended fixes (ordered):
1. **Chase the serial bus, not the software.** Highest-value check: `dmesg` / syslog on the
   arm machine for USB disconnect / reset / `ftdi_sio` errors in the window
   **12:49:15 – 12:52:22** (between flight 1's last live sample and flight 2's first frozen
   one). Then: the U2D2 cable and the Dynamixel daisy-chain connectors (vibration on a flying
   airframe is exactly what backs a bus connector out), and the servo bus supply — note the
   freeze appeared immediately after the first sustained flight, so a rail sag or brown-out on
   a shared pack is a live hypothesis worth instrumenting. Also try reading each servo ID
   individually: one hung servo can stall the whole bus.
2. **Make the driver fail loudly.** On a failed sync-read the hardware interface must stop
   publishing, publish NaN, or return ERROR — never silently re-publish the previous values.
   That single behaviour is what turned a bus fault into two bad flights.
3. Add a **frozen-stream guard**: arm-state freshness must check *change*, not arrival —
   encoder dither means a healthy stream never has 0.5 s of bit-identical positions at 250 Hz.
   Wire it into the DIRECT entry gate and the in-flight watchdog (the entry gate
   `wb_gate_arm_rad` passed trivially in bags 2/3 because frozen-q ≡ q_d at home). Because the
   arm was already frozen *on the ground*, this also belongs in a pre-arm check.
4. Runtime guard from 0901, now proven needed twice: **|u3| large while q is frozen ⇒ abort
   to SAFETY arm-hold**. In bag 3, |q̇| = 0 with 1–2 N·m sustained for 50 s was visible in the
   debug array the whole time.

**Not a fix: relaunching the arm stack differently (tmux, detached session, supervision).**
The process never died, and the one restart that did happen — between flights 2 and 3 — did
not clear the freeze. tmux remains good hygiene per this repo's own convention, but it does
not address this failure.

## Did the arm execute the commanded torque? (flight 1 = the only measurable flight)

The 0901 two-law-instance defect is **fixed and verified in flight**: `active_law` switches to
`EXTERNAL_WHOLE_BODY` on DIRECT entry, and the arm-side `law_debug[9..12]` equals the flight
node's `joint_torque_command` through the sign map [−1,+1,+1,−1] at r = 0.99–1.00, slope 1.00.

Delivered torque (Present Current ÷ calibrated `nm_to_effort` [160, 173.8, 146.7, 160]) vs
flight command over 97 s of DIRECT:

| joint | corr | slope | mean cmd → mean delivered |
|---|---|---|---|
| j2 | 0.985 | **0.93** | 0.577 → 0.576 N·m |
| j3 | 0.78 | **0.56** | 0.249 → 0.251 N·m |
| j1 | 0.15 | 0.05 | −0.001 → −0.001 N·m |
| j4 | 0.22 | 0.06 | +0.005 → 0.000 N·m |

- The **mean is essentially exact** on the loaded joints (the 0901 "under-delivery" is gone —
  it was the second law instance, not the servos).
- The **dynamic** delivery is partial: j2 93%, j3 56% — stiction/servo current dynamics
  absorb the fast component; during the arm-move transients j3's command spikes to 0.6–0.7 N·m
  while delivered lags visibly (loops in the scatter, fig 05). This is the residual η < 1 the
  stability screening cares about; at these amplitudes it flew stably.
- j1/j4 are commanded only ±0.05 N·m (≈ 8 ticks) and their Present Current reads ~0 — below
  the servo's effective current floor. Harmless at this loading, but do not expect microtorque
  authority on the wrist/yaw joints.
- Commanded pitch torque still carries the anti-damping component (τ_model_x vs v_fwd
  ≈ −0.33 N·m/(m/s), same as 0901) — it is now *cancelled* by the faithful arm delivery,
  which is why flight 1 is stable. It resurfaces the moment delivery breaks (flight 2).

## Is the compatible trajectory reasonable? Yes.

Every plan the governor produced was certified and gentle: defect 1.2e-11–8.0e-09 m,
σ_nd 0.235–0.332, peak CoM speed 0.01–0.25 m/s, T = 3.0–6.0 s. The streamed reference is
continuous (max inter-sample step 2.9 mm at ~57 Hz recorded, = the commanded velocity),
FD-consistent (x_cd_dot matches d/dt x_cd to 1.4–2.0 mm/s rms), q_d rate ≤ 19°/s, no jumps at
execute joins, and the hold is captured at the measured state (anchor error 3–5 mm). Flight 3's
failure is downstream of the reference, not in it.

## Is base_com precise? Close — a −5 mm-class residual remains (≈ 0.2 N·m)

Method: at an arm-static hold the steady rotational GMO channel is the unmodeled CoM moment,
Δy_system = d_r_x/u1 (model y = forward). Converged holds:

| flight / hold | arm pose | d_r_x [N·m] | system-CoM forward residual |
|---|---|---|---|
| 1 / 59–75 s (home) | 40/40 | −0.227 | **−6.0 mm** |
| 1 / 86–104 s (home) | 40/40 | −0.239 | **−6.4 mm** |
| 1 / 33–46 s (bent) | 41/19 | −0.226 | **−6.1 mm** |
| 3 / 16–28 s (home) | 40/40 | −0.155 | −4.2 mm |

The residual is pose-independent → it is the **base term**, not the arm model. The
`wb_base_com_y = −0.017854` correction removed most of the 0901-era −13 mm but the flight says
the true bare-airframe CoM sits another ~5–6 mm aft. Attributed to the base link
(÷ m_base/m_total = 0.830): **wb_base_com_y ≈ −0.0245** would zero it (remember the launcher
mismatch guard: governor `base_com` must move with it). Lateral residual ≈ ±1 mm — leave
`x` at 0. This costs ~0.2 N·m of standing pitch on the GMO today — worth updating but not the
source of the big errors.

## Remaining (real) tracking characteristics, from the healthy flight 1

- Steady hold accuracy after convergence: **+19/−18 mm** (hold 3). Post-move/entry transients
  are large and slow: 120–340 mm decaying with a 15–20 s time constant (|e_com| 166→41 mm over
  the 15 s hold). Same class as the sim's documented 119→5 mm/40 s GMO learning transient —
  a K_o_t = 0.5 rad/s observer characteristic, not a stiffness problem. If faster settling is
  wanted, raise `wb_ko_t` cautiously (the 2026-08-23 campaign showed the *rotational* K_o
  channels must stay at 0.5).
- `d_t_x ≈ −2.8 N` steady in every window — the same mostly-artifact internal-inconsistency
  bias 0901 measured (−3.55 N); unchanged by the base_com fix. ~1 N of it is real. It sets
  part of the parking offset after moves.
- `d_t_z` drifts +0.4 → −1.4 N over flight 1 — battery sag against the bench kf, expected.
- Heading holds to ±1° at holds; the yaw leg of the combined move overswung −14° peak.
- Flight 1's last hold shows |e_com| growing again (63→143 mm) with e_R doubling in its final
  third — worth watching on the next flight; battery sag raising the kf deficit is the likely
  driver.

## Traps for whoever re-analyzes

- `law_debug` (18 f32, arm side): [9..12] received torque (arm sign convention), [13..16]
  servo goal current [ticks], [6..8] FK of the arm's own q (constant ⇒ frozen read).
- The e_y sign is `r_e − r_ed` while the table's CoM error is `x_cd − x_c`: with a compatible
  reference and the arm on-pose the two are the *same* parking offset counted twice with
  opposite signs — don't read them as independent errors.
- Bag 2's pending_base at t=0 is a leftover latched value from flight 1 — the governor was
  not restarted between flights; ignore it.
- Effort ticks → N·m must use the calibrated [160, 173.8, 146.7, 160] counts/N·m, not the
  datasheet 205–208.5.
