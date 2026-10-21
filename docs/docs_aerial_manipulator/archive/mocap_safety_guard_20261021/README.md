# Raw-mocap safety guard for box pushing (2026-10-21)

**Status:** implemented in five repos and checked offline (gtest, loopbacks, offscreen GS,
hardware stack dry run) and in Isaac (§6). Nothing committed, NOT flown on hardware.

Box pushing flies on RAW mocap, because EKF2-fused feedback drifted 10–15 mm in contact
and failed every push (`../push_pull_20261003/README.md` §8.3–8.5; hardware decision in
`../box_pushing_hardware_20261019/README.md`). Raw mocap has no protection of its own: a
frozen pose, a flip or a stopped stream reaches the law directly. This is the guard for it.

## 1. The design (minimal scope, the user's final call)

**The purpose, in the user's words:** if raw mocap has problems, EKF2 guides the drone to a
safe HOVER; the operators then command from the arm GS and the pilot takes over and lands.
No automatic descent, no new buttons, no safe-altitude parameter, no land-anywhere.

**Detection, on the Orin, in the estimator.** Every drone mocap sample is checked; the age
of the last GOOD sample drives one ladder:

| state | when | what happens |
|---|---|---|
| MOCAP | healthy | the odometry IS the mocap, as before |
| BRIDGING | age > 50 ms, or the latest sample is bad | the odometry becomes the EKF2 bridge; the mission continues |
| CONFIRMED | age > 0.15 s | with the claw on the box (or Ready / Push flying): **the planner's ABORT** — release, jaws open (arm GS), the release wait, then a STRAIGHT 0.30 m climb (no back-out, §6), arm home — then HOVER; in free flight: keep bridging |
| LOST | age > 1.0 s | any step still flying is aborted; HOVER; "take over with the RC and land" |
| back to MOCAP | good again for 1 s (after CONFIRMED) | blended back over 0.5 s, stretched to at most 0.10 m/s of odometry motion (§6); the aborted push never resumes |

While the guard is not MOCAP, **every step and Plan are refused** ("every step waits for the
mocap — hover; the pilot lands").

A sample is bad when it is **frozen** (bit-identical position for > 50 ms, ≥ 3 samples —
pairs are normal VRPN repeats) or a **jump** (> 20 mm + 1.0 m/s·dt + ½·2 m/s²·dt² from the
last good position, or > 10° + 90°/s·dt in attitude). A stopped stream is caught by the
age. Reference implementation: `tools/guard_reference.py` (self-test included).

**The bridge.** While the mocap is bad, the odometry the law reads is EKF2's position plus
the offset `d0 = p_mocap − p_EKF2` frozen at the last good sample (no step at the switch),
with EKF2's velocity. Samples are paired by **arrival on the Orin** — EKF2's stamps ran
75–263 ms off in two of the six 10-15/16 flights. The estimator **stops forwarding bad
samples to PX4** — and no sample at all until the guard has recovered (§6) — so EKF2 coasts on
the IMU instead of fusing a frozen pose (the 09-24 fly-away). The bridge is temporary: IMU dead-reckoning drifts ~24 mm in 1 s, ~25 cm in 5 s
(p90, §2.3) — it carries the abort and the hover, never a landing. No EKF2 odometry = no
odometry at all, and the flight node's own feedback timeout trips.

**Known limit, accepted:** the jaws are opened by the arm GS on the laptop. With the WiFi
down too, an abort climbs with the jaws still closed (there is no Orin-side open, by the
minimal scope). The 10-19 PX4–agent cut showed the WiFi carrying traffic through a fault of
the PX4 link, so the two are not the same failure.

*(Superseded designs, kept in §2.4 and §4 for the record: an automatic escape plus a
clearance-checked descent; then a climb to a computed safe altitude, Land enabled in every
state, and an Orin-side jaw open. All dropped for the minimal scope above.)*


## 2. Preparation results

### 2.1 Thresholds, from the six 10-15/16 hardware flights (549 s airborne)

| check | worst normal value | threshold |
|---|---|---|
| gap between samples | 31 ms (laptop side; largest WiFi spike to the Orin on 10-20: 22 ms) | bridge > 50 ms, confirm > 0.15 s |
| bit-identical samples | none | > 50 ms |
| one-sample position step | 5.0 mm | 20 mm + growth |
| one-sample rotation | 1.9° | 10° + growth |
| raw mocap vs EKF2 (fused, free flight, with basket) | 29 mm max, 10–21 mm p99 | 60 mm, **warning only** at first (contact unknown) |

EKF2's error vs the raw mocap is not a slow offset: its p99 change is 2.4–6.6 mm in 0.1 s,
6–15 mm in 0.25 s, 11–25 mm in 0.5 s while it is still fusing the mocap (larger with the
basket). Scripts: `tools/mocap_thresholds.py` (gaps, steps, EKF2 offsets aligned on
receive time) and `tools/ekf_diff_dynamics.py` (aligned on capture stamps; its two 10-16
rows with EKF2 stamps 75–263 ms off are invalid there and were left out of the numbers).

### 2.2 The detector replayed on every recorded flight — `tools/replay_flights.py`

29 hardware flights (09-18 … 10-16, 2793 s airborne, the drone's mocap as recorded on the
laptop — processor-side faults, not WiFi drops). Output: `replay_flights.txt`.

- **Zero false confirmations.**
- **Every recorded freeze is confirmed and reaches LOST:** 09-18 (2.2 s), 09-24 flight 1
  (2.84 s, the fly-away) and 09-24 flight 2 (2.66 s during the SAFETY descent at
  z 0.46 m — not in the 0924 report). None in the 16 flights since 09-28.
- **Every flip is rejected on its own sample and never escalates:** 20 on 09-21 #3
  (largest 189 mm / 112°), more on 10-09 PP-2 and 10-10 PP-4 (after their falls);
  bridging ≤ 0.16 s per flight.
- **Two detector designs failed the replay before this one:** a prediction with the
  processor's twist (a one-sample finite difference — it spikes to tens of m/s across a
  flip, and one rejection then lasted > 20 s) and with the guard's own finite-difference
  velocity (a 20 mm one-sample glitch just inside the gate corrupted it and became a false
  confirmation on 09-21). The constant-position gate with a speed bound is robust and
  enough: flips are 110–190 mm per sample, a normal 8 ms step < 6 mm. **The C++ must
  follow this, not invent a velocity predictor.**
- Recording artefacts, excluded: a bag that starts mid-flight opens with the recorder's
  catch-up (345 ms gap then samples 0.1 ms apart on 09-18) — the first 1 s is not scored.

### 2.3 How far IMU dead-reckoning drifts on the real T650 — `tools/coast_drift.py`

The bridge flies on EKF2 coasting on the IMU; no flight ever lost vision with EKF2 logged.
So: from a start point every 1 s of each flight, the recorded accelerometer
(sensor_combined, ~400 Hz) is integrated through EKF2's attitude (vehicle_attitude) after
fitting a body-frame accelerometer bias over the 8 s before (≈ what EKF2 knows), and
compared with OptiTrack. 23 flights, 1711 windows. Output: `coast_drift.txt`.

| horizontal drift | 0.25 s | 0.5 s | 1 s | 2 s | 3 s | 5 s |
|---|---|---|---|---|---|---|
| p50 | 1.9 mm | 4.2 mm | 11 mm | 28 mm | 52 mm | 115 mm |
| p90 | 4.4 mm | 9.5 mm | 24 mm | 64 mm | 115 mm | 252 mm |
| p99 | 12 mm | 27 mm | 69 mm | 200 mm | 334 mm | 768 mm |

Vertical p90: 9 mm at 1 s, 23 mm at 2 s, 87 mm at 5 s. Without the bias fit (pessimistic):
p90 39 mm at 1 s, 145 mm at 2 s, 833 mm at 5 s. Fitted bias: per-axis median
(−0.007, −0.011, 0.008) m/s², p90 of |b| 0.066 m/s². **p90 ≈ 24 mm·t^1.5,
p99 ≈ 69 mm·t^1.5.**

What it means for the design:
- **In contact** the arm absorbs base error at ~6–8° of q3 per cm and q3 has ~18° to its
  10° guard at the push pose: a ~2.5 cm budget. The release (0.15 s confirm + ~0.7 s jaws)
  ends at ~0.85 s: ~2 cm at p90, fits; ~6 cm at p99, does not — opening the jaws from the
  Orin at confirmation, with no WiFi hop, is required, and shortening the release hold for
  a fault abort is worth testing.
- **While it holds**, the vehicle drifts sideways at this rate: ~6 cm (p90) to ~20 cm (p99)
  after 2 s, ~25 cm to ~77 cm after 5 s — harmless above the safe altitude, but the reason
  the pilot must take over within seconds.
- An automatic landing would have needed this as a clearance margin; §2.4 shows why that
  was dropped.
- Caveats: not EKF2 itself (no filter, no lever arm, attitude from a vision-aided EKF2,
  hindsight bias fit), the recorded IMU crossed the WiFi (windows with an IMU or attitude
  gap > 20 ms skipped), clock offsets between the Orin and the laptop removed with a
  rolling median (0.348 s on 09-18; a +0.6 s step on 10-02). 09-18 remains an outlier
  (p90 123 mm at 1 s, 45 windows) — not chased.

### 2.4 Why the automatic descent was dropped (kept for the record)

The uncertainty keeps growing during the escape and the descent themselves, so the
landing distance is a fixed point. Timeline from the fault: confirm 0.15 s + jaws 0.70 s +
climb 0.80 s + side-step (min-snap, peak speed v: T = 2.19·d/v) + a 1.0 m descent at
0.4 m/s (2.5 s); the landing point must sit 0.075 + 0.52 + 0.10 + u(touchdown) from the
table's centre line (the vehicle starts over it).

| drift percentile used for u | side-step d (v = 0.6 m/s) | touchdown after the fault | u at touchdown |
|---|---|---|---|
| p50 | 0.98 m | 7.7 s | 0.29 m |
| **p90** | **1.44 m** | **9.4 s** | **0.74 m** |
| p99 | does not converge (> 5 m) | — | — |

- p99 is not achievable for an automatic landing on IMU coasting; p90 was workable but
  leaves a 1-in-10 tail, and even at p90 the side-step would have had to peak ~1.0 m/s to
  end inside PX4's 5 s dead-reckoning limit (`EKF2_NOAID_TOUT`).
- The drift is a probability because it comes from errors that cannot be observed during
  the loss (the bias EKF2 has not learned, the velocity error at the instant of loss,
  vibration, attitude error); no guaranteed bound on them exists — the worst recorded case
  was 216 mm after 1 s and 2.2 m after 5 s. **The user chose the definite rule instead
  (§1): climb above everything, hold, land with the RC.**

### 2.5 Facts from the code that shape the implementation

- **The estimator forwards every `/mocap` sample to PX4 unconditionally** with fixed
  variances (position 1 cm, roll/pitch 0.57°, yaw 5°, velocity 1 cm/s;
  `indoor_state_estimator.cpp` SetupSubPub). Its `mocap_status` says "not received" only
  after **10 s** (100 ticks of a 100 ms timer) — why it read normal through 09-24's 2.84 s.
- **The raw odometry is a pure pass-through** (`MocapOdomBridge`), a separate node in the
  same process and executor as the forwarder (`indoor_mocap_feedback_node.cpp`): one shared
  guard object can gate both the odometry and the forwarding. `Px4FusedOdomBridge` already
  has the vehicle_odometry → ENU conversion and the stamp handling the bridge needs.
- **The flight node's failsafe** (`feedbackLost()` in the whole-body client) trips on odom
  stale > `system_fb_timeout_s` (1.0) **or EKF2 not fusing vision for > 1.0 s**
  (`estimator_status_flags.cs_ev_pos`) → SAFETY + land in place at 0.2 m/s, dead-reckoned
  on PX4's velocity. With the gate, EKF2 stops fusing vision BY DESIGN — that condition
  must key on the guard's state instead, and land-in-place is wrong over the table.
- **PX4 v1.16 never leaves OFFBOARD for a position loss here**: OFFBOARD requires only
  attitude, angular velocity and the offboard signal, and the signal counts as lost on an
  invalid local position only if position/velocity control is requested
  (`offboardCheck.cpp`); the node always sets position/velocity/acceleration false (DIRECT
  = direct_actuator, SAFETY = attitude + thrust_and_torque). So PX4 will not land the
  vehicle on its own; our software owns the whole response. After `EKF2_NOAID_TOUT`
  (5 s default) EKF2 reports its horizontal solution invalid — and the node's failsafe
  descent checks only that PX4's odometry is still ARRIVING, not that PX4 still trusts it.
- **The planner streams only in DIRECT**, so the fault sequence (abort, escape, descent)
  runs in DIRECT on the bridge; the node must not revert to SAFETY at LOST while the
  planner's sequence is active and PX4's estimate is valid.
- **The planner's box guards already abort after 0.5 s of a silent or frozen box feed**
  (`plFeedGuardCheck`); the box and the drone share the processor, so a Motive pause trips
  that too — a drone-only loss (occluded drone markers) needs the new guard.
- **The gripper** is `control_msgs/action/GripperCommand` at
  `<arm ns>/gripper_controller/gripper_cmd` (open 0.019 m, effort 50; `control_panel.cpp`);
  the planner can send the OPEN itself.
- Today's ABORT (`onPlAbort`): cuts the step, holds from the measured pose when the box was
  touched, retreats `push_pull_abort_back`, waits `push_pull_abort_release_wait_s` (1.5 s on
  hardware), climbs `push_pull_abort_climb`, folds; the GS opens the jaws on the press.
  Land today requires `pl_completed_ == Go To Land Start` and the claw off the handle.

## 3. What was built

| repo (branch) | change |
|---|---|
| `fsc_drone_state_estimator_ros2` (**new branch `dev_robotic_arm`**, from `cpu_isolation_fifo`) | `MocapGuard` (`mocap_guard.{hpp,cpp}`: checks + ladder, test fault) and a thread-safe `MocapGuardHub` shared by `MocapOdomBridge` (bridge, health topic `state_estimator/mocap_guard`, test service `state_estimator/mocap_guard/test_fault`) and `IndoorStateEstimatorNode` (bad samples, and every sample of a fault episode until recovery, not forwarded to PX4), both in `indoor_mocap_feedback_node`. The blend back is speed-limited (`mocap_guard_blend_speed` 0.10 m/s). Parameter `mocap_guard` (default **false** = the old pass-through, byte for byte) + the thresholds; launch arg `mocap_guard:=true`. The **fused** executable is unchanged (no hub) — pick-and-drop flies it as before. gtest `test_mocap_guard` 10/10: synthetic cases + exact parity with `guard_reference.py` on the 09-18 / 09-21 / 09-24 / 09-28 recordings. Loopback `tools/estimator_loopback.py` ALL PASS (bridge follows the true motion to < 2 mm through a 2 s freeze, no frozen sample forwarded, flip never confirmed, blend back without a step). |
| `fsc_trajectory_planner` (`main`) | `push_pull_mocap_guard_topic` ("" = off): CONFIRMED with the claw on the box or Ready / Push flying → the existing ABORT (once per fault), climbing straight up (no `push_pull_abort_back`); LOST with any step flying → the same; steps and Plan refused while not MOCAP or while the guard's status is stale (> 0.5 s); status `ABORTED (SAFETY GUARD: mocap …) -- HOVERING on EKF2: take over with the RC and land`. Loopback `test_push_pull_loopback.py` [9d] / [9e]: ALL PASS. |
| `fsc_autopilot_ros2` (`dev_robotic_arm`) | Whole-body client: `system_mocap_guard_topic` ("" = off) — while the guard is fresh and not MOCAP, EKF2's vision loss does not trip the feedback failsafe (the gate stops it fusing vision by design); DIRECT entry refused unless fresh and MOCAP; each state edge logged once. `system_fb_land_speed` may now be 0 (= hold altitude, the pilot lands; default 0.2 unchanged). Wb tests 15/15. Box-pushing yamls (sim + hardware, regenerated): both topic keys. The raw stacks (Isaac + hardware) launch the estimator with `mocap_guard:=true` for box pushing only; the hardware raw stack REFUSES a planner, node or estimator binary without the guard (`FSC_ESTIMATOR_BIN` overrides the estimator path). Coordinated-admittance / valve yamls do not get the keys. |
| `fsc_open_manipulator` (`omx-torque-control`) | Arm GS Box Pushing tab: a **Mocap** block in the Monitor (right of Push Time … Box Travelled): lamp MOCAP (green) / EKF2 BRIDGE (amber outline) / EKF2 BRIDGE -- CONFIRMED (amber) / LOST -- LAND WITH THE RC (red, flashing, beep every 2 s) / NO LINK / guard off; detail lines: cause + age + rate, odometry source + mocap–EKF2 difference, episodes + rejected samples, last fault. Valve tab unchanged. Offscreen check `tools/gs_mocap_check.sh` PASS; pictures in `gs/`. |
| `ros2_ground_station_gui` (`dev_robotic_arm`, added later the same day, user request) | **Drone GS voice alarm for the pilot** (the laptop's speakers), through its existing siren+speech `AudioAnnouncer`. It subscribes `/uav_0/state_estimator/mocap_guard`. BRIDGING is silent (a rejected flip). CONFIRMED plays a siren and "Mocap fault". LOST says **"Mocap lost. Pilot, take over and land."** about 1 s into the fault: it cuts a siren that is still sounding instead of adding a second one, and jumps ahead of other alarms (`urgent`, which replays the alarm it cut afterwards). The call repeats every 8 s, with the siren, until PX4 reports any mode but OFFBOARD or disarmed. Disarmed, it says just "Mocap lost". Recovery says "Mocap regained". Silent on stacks without the guard. No widget; the arm GS lamp stays the visual. Offscreen check `tools/drone_gs_voice/run.sh` (real GS on domain 77, stand-in players): all cases pass, `run_20261021.txt`. The arm GS's own "beep" is `QApplication::beep()`, the X11 bell, which is usually inaudible. |
| `isaacsim_optitrack_ros2_emulator` (`dev_robotic_arm`) | Fault injection: `fault_body`, `fault_type` (none / stop / freeze / jump), `fault_duration_s`, `fault_offset_m`, `fault_yaw_deg`, set at runtime (atomically from the driver). |
| Pegasus | `push_pull_planner_check.sh` compares the planner's guard topic; `pl_mission.py --mocap-fault TYPE@STEP+DELAY[:DURATION][,BODY]` + the guard health in the npz (`guard`); `make_push_pull_yaml.py` writes the keys, `make_coordinated_yaml.py` drops them. |

**Rebuild before flying:** Orin — `fsc_drone_state_estimator_ros2` (switch it to
`dev_robotic_arm`), `fsc_trajectory_planner`, `fsc_autopilot_ros2`; laptop —
`utils_custom_ground_station`. The hardware raw stack refuses any of the three Orin binaries
without the guard.

**Found on the way, NOT fixed:** the emulator's launch file has never applied `frequency:=`
(the node-specific `frequency: 250.0` in its params.yaml wins), so every mirror-profile sim
that asked for 60 Hz mocap ran at 250 Hz. Fixing it changes every earlier mirror run; the
guard does not depend on the rate.

## 4. Decisions

**Answered by the user (2026-10-21):**
- **No automatic descent**: abort / hover on EKF2, the pilot lands with the RC. Then the
  minimal scope (§1): no safe altitude, no Land-anywhere, no new buttons, no Orin-side jaw
  open.
- **The estimator develops on `dev_robotic_arm`.**
- **The fused stack (pick-and-drop) stays unaffected**: the guard is in the raw-mocap
  executable only, and only box pushing turns it on.
- **The lab field is the same 4.5 × 4.2 m — keep the drone in the CENTRE region** so it
  never leaves the cameras' field of view.
- **The Pixhawk's EKF2 vision-noise parameters are PX4 v1.16 defaults** —
  `EKF2_EVP_NOISE` 0.1 m, `EKF2_EVV_NOISE` 0.1 m/s, `EKF2_EVA_NOISE` 0.1 rad,
  `EKF2_EV_NOISE_MD` 0, `EKF2_NOAID_TOUT` 5 s, `EKF2_BARO_CTRL` 1. Every number in §2
  already reflects them.

**Not decided:** `system_fb_land_speed` stays 0.2 in the box-pushing yamls (it acts only when
the odometry itself stops — no mocap AND no EKF2); 0.0 would make that case a hold too.


## 5. How this preparation was done without disturbing the running campaign

The other session's box-pushing campaign (mean velocity) was live: its stack's tmux session
was up, and it had uncommitted edits in `fsc_trajectory_planner` (push_pull.*, the node,
tests), `fsc_autopilot_ros2` (the box-pushing yamls, both 4-D hardware stack scripts) and
`fsc_open_manipulator` (`push_pull_panel.*`). So: no builds, no ROS processes, no edits to
any file a stack or campaign reads, no process name its guards or clean-slate match
(`whole_body_trajectory_planner`, the estimator executables, the autopilot node names, the
emulator, the XRCE agent, PX4/Isaac, the virtual RC, the pad nodes) in any command line —
files with those names in their path were opened with the file reader — and the analysis
ran at `nice 19` on the E-cores (16–31), away from Isaac's pinned P-cores.

Run the tools with `PYTHONNOUSERSITE=1 /usr/bin/python3 -I tools/<name>.py`.

## 6. Isaac checks (2026-10-21, headless, raw mocap, 240 g box, 0.10 m push at 21 mm/s)

Driver `tools/run_guard_isaac.sh` (flies `../box_pushing_campaign_20261016/tools/fly.sh`), scorer
`tools/guard_flight_score.py`; runs copied to `runs/`. The fault: the emulator freezes the drone's
mocap for 3.0 s (bit-identical pose, fresh stamps — the processor's 09-24 behaviour), 2.0 s into the
push, with the claw clamped on the fin.

| run | code | guard ladder | EKF2 bridge drift (3.9 s) | box | vehicle |
|---|---|---|---|---|---|
| mg_nom1 | first build | MOCAP throughout (the only episode: the spawn re-seat on the ground, 44 s before the mission) | — | pushed 101 mm, upright | complete mission |
| mg_freeze1 | first build | BRIDGING +70 ms, CONFIRMED +0.21 s → abort the same tick, LOST +1.07 s, MOCAP 1 s after the freeze | 191 mm | dragged 160 mm, **tipped** (37°) | DIRECT kept; lurched back 200 mm at recovery |
| mg_freeze2 | + straight climb, + no forwarding until recovery | same | 584 mm | dragged off the table's edge, **fell** | the 0.5 s blend of a 0.4 m correction: 14° tilt, **left DIRECT**, swung ~0.5 m in SAFETY, landed upright |
| mg_freeze3 | + speed-limited blend (final) | same | 223 mm | dragged 64 mm, **upright** (0.13°) | **DIRECT kept**, tilt ≤ 7.1°, climbed 0.23 m, eased back over ~3 s, hovered within mm |
| mg_freeze4 | final (repeat) | same | 546 mm | dragged 182 mm, **upright** (6.2°) | **DIRECT kept**, tilt ≤ 10.2°, abort complete, hover, landed |
| mg120_nom1 | final, **mocap 120 Hz** (the experiment's rate) | MOCAP throughout at 120 Hz, nothing rejected in the mission | — | pushed 97 mm | complete mission |
| mg120_freeze1 | final, **mocap 120 Hz** | same | 326 mm | dragged 77 mm, **upright** (10.2° on the way) | **DIRECT kept**, tilt ≤ 6.9° |

What the runs say:
- **The ladder is exact and repeatable** (4/4): the abort fires in the tick the fault is confirmed.
- **The EKF2 bridge is short-lived in contact in this sim**: 190–590 mm of horizontal drift in
  3.9 s. EKF2's velocity follows the truth for ~0.5 s and then stays near zero while the vehicle
  accelerates at ~0.05–0.1 m/s² — the dead-reckoning error the 10-05 runs showed in contact (PX4's
  attitude estimate 1–3° off while the push shakes the IMU). Hardware hover statistics (§2.3) are
  better, contact on hardware is unmeasured. **Plan on the pilot taking over within ~2 s of a LOST
  alarm.**
- **Three fixes came out of the first two flights**, all in the final code:
  1. a mocap-fault abort climbs STRAIGHT up (no `push_pull_abort_back`) — the 0.03 m back-out took
     3.5 s at fin height on a drifting estimate;
  2. the estimator forwards nothing to PX4 until the guard recovers — resumed earlier, EKF2 snaps to
     the mocap while the bridge still adds the frozen offset, and the odometry jumps by the drift;
  3. the blend back is limited to 0.10 m/s of odometry motion (`mocap_guard_blend_speed`) — a fixed
     0.5 s blend of 0.4 m tilted the vehicle 14° and the node left DIRECT.
- **The box is not protected**: the open jaws sit around the fin through the release wait, and any
  drift drags the box (64–182 mm, upright, in the final runs). The vehicle is.
- Not run: a free-flight fault, a jump or a stop fault, the hardware hover test with the
  `test_fault` service.

### 6.1 The experiment's rates (user, 2026-10-21): mocap 120 Hz, PX4 `sensor_combined` / `vehicle_attitude` 400 Hz

- **The guard needs no change**: its thresholds came from the 10-15/16 flights, whose mocap was
  already 120 Hz (an 8 ms step); the freeze rule (> 50 ms identical) is 6 samples there; the bridge
  reads EKF2's `vehicle_odometry`, not `sensor_combined` / `vehicle_attitude`.
- **The sim's mocap now runs at 120 Hz.** The emulator's launch file never applied `frequency:=`
  (params.yaml's node-specific 250 beat the override), so every sim before ran 250 Hz whatever its
  yaml said. Fixed in `isaacsim_optitrack_ros2_emulator` (one merged parameter dict; no override =
  still 250 Hz), and `sim_feedback_mocap_rate_hz` 60 -> 120 in the mirror and every live sim config
  derived from it (13 in fsc_autopilot_ros2 + the geometric / modular comparison variants; the
  regenerated files differ only by the rate and its comment). The as-flown campaign copies under
  `archive/*/runs` and `*/variants` records keep their 60. Measured in flight: 120.1 Hz.
- **The sim's PX4 does NOT match 400 Hz: it publishes `sensor_combined`, `vehicle_attitude` and
  `vehicle_odometry` at 100 Hz** (measured in flight). This PX4 v1.16 build's uXRCE-DDS client
  sets every published topic's interval to 10 ms (`UXRCE_DEFAULT_POLL_RATE 10`,
  `src/modules/uxrce_dds_client/dds_topics.h.em`), and the sim's IMU itself only runs at the
  250 Hz physics step. Raising it means rebuilding PX4 SITL and changes the law's attitude / gyro
  feedback in every sim rig — not done; the user's call.
- Measuring: `ros2 topic hz` sees nothing on this desktop (the wedged daemon); measure with a
  direct subscriber instead (`tools/rate_probe.py`).
