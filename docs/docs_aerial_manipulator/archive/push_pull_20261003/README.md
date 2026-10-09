# Push-and-pull with the whole-body 4-D L1 law (2026-10-03)

User request: a box on the lab's console table, grasped by a handle on its SIDE
and pushed 50 cm along the table, the arm held at a fixed pose, the law in its
contact (impedance) phase for the interaction -- five buttons on a new arm GS
tab, the same logic as pick-and-place, safety first. Commands: Command.md
section 17. FLOWN IN ISAAC 2026-10-04 (section 7): three changes were needed,
then 3 / 3 complete missions, the box pushed 495-502 mm and left on the table.

## 1. The scene (07_px4_direct_t650_aerial_manipulator_push_and_pull.py)

| item | value |
|---|---|
| table | HOOBRO BB52UXG01G1 (amazon.ca B0G62DWL11): 39.4 x 5.9 x 31.5 in = **1.000 x 0.150 x 0.800 m**; centred on (1.20, 0), its LENGTH along y; 18 mm top, 25 mm legs, a lower shelf |
| box | 200 (y) x 100 (x) x 60 mm, **200 g** with its handle, centre (1.20, 0.25, 0.83) -- mocap **obj_0** (centre + yaw) |
| handle | a vertical FIN on the box's +y face: 20 mm thick along x (the jaws close along it), 90 mm out, 20..110 mm above the table top |
| grasp point | 15 mm under the fin top, 55 mm out from the box face: **EE offset [0, 0.155, 0.065] m** in the box frame |
| friction | box / table static **0.6** = dynamic **0.6** -> **1.18 N** to break the box loose and to keep it sliding (equal on purpose, section 7; first designed 0.7 / 0.6); the fin carries the pick-and-place grip material (1.2 / 1.0, combine max); gripper drive capped 0.3 N.m |

Why these numbers:
- **Force bound.** The 2026-09-27 interaction campaign's along-the-arm
  capacity is ~6 N (8-10 N nears the tilt guard). 1.2 N is a 5x margin, and it
  sits above the hardware |F_hat| noise floor (p99.9 1.3 N at the 2 rad/s
  reading), so the push reads as a contact. The planner aborts above 5 N.
- **Tipping is mass-independent**: the box tips about its leading bottom edge
  when mu_s * h > L / 2 (h = the grasp height above the table, L = the box
  length along the push). 0.6 x 0.095 = 0.057 m vs 0.100 m: 1.75x margin before
  the grip's moment helps. **For the real box: keep mu_s * h < L / 3** -- choose
  the length first, then the payload / surface for the friction you want.
- **The table is only 15 cm deep**: a 100 mm box leaves 25 mm a side. The claw
  holds the fin, so the box follows the claw's lateral error (mm in sim).

## 2. The geometry that decides the height

The push pose is the pick pose [0, -20, 30, 0] deg (sigma_nd 0.298, claw
pitched 10 deg forward): the claw 0.130 m ahead of and **0.316 m under** the
body origin. The landing gear, measured from AM_xfwd.usda: two skids along the
body x, **0.28 m apart inside**, bottoms **0.313 m** under the origin, struts at
x ~ 0 from |y| 0.10 (z -0.10) to 0.16 (z -0.27). With the grasp at 0.895 m
the body origin flies at 1.211 m and **the skids are 9.8 cm above the table
top** -- and they straddle both the 150 mm table and the 100 mm box laterally
anyway. The planner checks every step against an axis-aligned table keep-out
(`push_pull_table`, 5 cm clearance for the gear footprint and every arm point);
a grasp 6 cm lower is refused.

## 3. The mission (planner: fsc_trajectory_planner push_pull.{hpp,cpp} + the node's push-and-pull section)

| button | planner | what flies (dry run on the sim yaml) |
|---|---|---|
| 1 Go To Start | `push_pull/go_to_start` | the claw 0.20 m above the grasp point, yaw -90 (the box's - 90), arm home -> push pose; 9.8 s. On arrival the claw is held in the WORLD (the pick's anchor) |
| 2 Ready To Push | GS opens the gripper; `push_pull/ready` | straight down 0.20 m onto the fin (gated: claw within 5 cm of the point above, descent trim as the pick's); 3.2 s; on arrival CONTACT on |
| 3 Push | GS closes once within 2 cm / 3 mm across the fin; on the stall `push_pull/push` | the claw stays WORLD-held (`push_pull_world_anchor_push`); 1.5 s settle; the base 0.50 m along the nose, arm / heading / height fixed, >= 12 s (peak 0.09 m/s); 13.5 s |
| 4 Exit To Push | GS opens, waits 1 s; `push_pull/exit` | CONTACT off; straight up 0.20 m (3 s, arm fixed: the open jaws leave the fin vertically), then the arm home in place; 9.5 s |
| 5 Go To Land | `push_pull/go_to_land` | to the land hover 0.5 m behind the start, yaw -90 -> -360 = **270 deg clockwise** (flown as typed); 23.9 s |
| ABORT | GS opens + `push_pull/abort` | contact off, CoM anchor; on the handle a 0.5 s hold for the jaws, then up 0.30 m, then arm home. Touched box -> capture + plan dropped |

Safety guards in the planner (both -> ABORT): tilt > 15 deg for 0.1 s; the task
force the law reads (wb_control_debug [97..99], low-passed at 3 rad/s) > 5 N for
0.3 s while in contact.

## 4. The contact phase (fsc_autopilot_ros2, whole-body client)

New service `whole_body_direct_actuation/set_contact` (std_srvs/SetBool): the
4-D attribution's chi from the task. DIRECT and 4-D L1 only; every DIRECT entry
and SAFETY revert restores the yaml's `wb_l1_contact` (false here) -- no call =
the node as it was. The yaml (generated, `tools/make_push_pull_yaml.py`) = the
mirror + vehicle_name AM-T650-WB-L1-4D-PUSH + k_R/k_w 1.6/1.2 (the
pick-and-place pair) + the contact reading split (wb_l1_omega_x 2.0, per-block
0.2072 -- the interaction campaign's fix; free flight identical) + the planner
block. The pick-and-place campaign was evaluating k_R/k_w 1.4/1.5 on
2026-10-03; not adopted here.

## 5. Verified (offline, 2026-10-03)

- `test_push_pull` (gtest, 8): the grasp on the fin, yaw -90, the push holds the
  arm (< 1e-6 rad) and heading and slides 0.50 m, the claw height moves < 1 mm
  (the acceleration tilt), the descent is vertical, the exit climbs before it
  folds, the land turn is 270 deg clockwise, the table check refuses a low
  grasp, the mission dry-runs at rest at both ends of every step, the abort
  climbs then folds. All other planner gtests pass (49 in total).
- `test_push_pull_loopback.py` (the built node, a perfect-plant rig, fake
  contact / anchor / force feeds): ALL PASS -- Adjust, Get, Plan, out-of-order
  refusals, the world anchor on arrival, ready refused 80 mm off, push asks for
  CONTACT + the CoM anchor (2026-10-04: the world anchor, per the yaml) and slides 0.500 m with the arm fixed to 1e-14, exit
  turns contact off and climbs 0.200 m before folding, 270 deg clockwise land,
  the force guard trips at 5.6 N mid-push (release hold 0.55 s, climb, arm
  home, plan dropped), SAFETY silences the stream.
- The whole-body node: set_contact registers and refuses outside DIRECT.
- The arm GS tab rendered offscreen at 2560 x 1440 against the loopback
  planner (holding above the handle, on the handle, mid-push).

Not covered offline: the friction, the grip on the fin, the box's behaviour,
the contact rendering under a real push -- section 7 flies them.

## 6. Tools

- `tools/make_push_pull_yaml.py [--check]` -- the sim yaml from the mirror.
- `tools/run_pl.sh <tag>` -- one headless flight (after a clean slate in a
  separate call); `tools/pl_mission.py` -- the tab's steps, scripted.

## 7. Flown in Isaac (2026-10-04)

Headless, RTF 1, raw mocap, the mirror plant (`run_pl.sh <tag>` -> `runs/`,
scored by `tools/pl_score.py runs/<tag>.npz`). Every run: Adjust, Get, SAFETY
climb to 1 m, DIRECT, Plan, the five steps as the GS would press them
(`tools/pl_mission.py`), landing. The first three steps were clean in every
run (claw 1-10 mm above the handle, the grasp 2-4 mm from the point, 0-3 mm
across the fin); everything below is about the PUSH.

| run | what changed | push | box |
|---|---|---|---|
| pl_1 | as designed: CONTACT at the Push press, CoM anchor | node watchdog (tilt 20 deg) 7.7 s in | off the table |
| pl_2 | + the law's platform observer HELD in contact, CONTACT from the end of Ready | planner tilt guard 4.2 s in | |
| pl_3 | + hold only the TRANSLATIONAL rows | completed, 0.3 Hz limit cycle, heading 90 deg | 468 mm, yaw -35 deg |
| pl_4 | + the claw WORLD-held through the push | node watchdog 6.3 s in (arm at the wrist singularity) | |
| pl_5 | world anchor, translational hold OFF | force guard (5.3 N) 6.0 s in | 126 mm |
| **pl_6** | **+ box friction 0.6 / 0.6 (was 0.7 / 0.6)** | **completed** | **502 mm, 14 mm sideways, yaw +2.6 deg** |
| **pl_7** | repeat | **completed** | **495 mm, 49 mm sideways, yaw +14.9 deg** |
| **pl_8** | repeat | **completed** | **501 mm, 2 mm sideways, yaw -2.9 deg** |

The push step on the shipped configuration (pl_6 / pl_7 / pl_8): peak tilt
5.5 / 9.8 / 5.5 deg, |e_R| <= 0.12 / 0.16 / 0.10, CoM error peak 51 / 49 / 53 mm
(rms 23 / 24 / 22), EE position error peak 11.5 / 20.4 / 11.4 mm (rms 5 / 7 / 5),
EE heading peak 6 / 12 / 13 deg, peak arm torque 1.20 / 1.44 / 1.32 of 3 N.m,
**0 rotor saturation, 0 joint clamp**, claw-to-box slip <= 22 / 61 / 22 mm.

**The mechanism, in the order it was found.** A box held by STATIC friction
reacts with exactly the force the vehicle applies, so anything that
INTEGRATES that reaction pushes through it until break-away:

1. pl_1: the law's CoM observer (the note's Layer 1, the lumped d_t that f_d
   cancels) integrated it -- d_t,y -0.2 -> +1.3 N and 51 mm of CoM drift on a
   FIXED reference during the 1.5 s settle, then a growing stick-slip cycle.
2. pl_2: holding the WHOLE platform estimate removed that, but the box's
   friction acts ~0.3 m below the CoM, and the pure-P attitude loop alone is
   k_R M_r / M_r_d ~ 1.6 N.m/rad: 0.37 N.m tilted the vehicle 0.26 rad.
3. pl_3: holding only the translational rows completed the mission, but with
   the CoM anchor any base offset drags the claw reference against the stuck
   box, and the ROTATIONAL observer then pushed through it -- a 0.3 Hz cycle,
   box yaw to +-90 deg.
4. pl_4 / pl_5: holding the claw in the WORLD through the push decouples the
   base from the box (base drift moves no reference): the first 4 s of pl_5
   were the cleanest of the campaign. pl_4 also held the translational rows and
   the base sat 25 mm off its plan, stretching the arm to the wrist singularity
   (beta = q2 + q3 = 1.5 deg) -- with the world anchor the hold is not needed.
   pl_5 failed at the break-away: 0.7 -> 0.6 is a 14 % force drop while the
   observers still carried the stuck box's reaction and its moment; the base
   surged 7 cm and the world-held claw folded the arm onto q3's +50 deg stop.
5. pl_6: equal static and kinetic friction removes that drop. The rest of
   the configuration is pl_5's.

**What changed, where:**
- planner (`fsc_trajectory_planner`): CONTACT requested at the END OF READY,
  not at the Push press (the jaws close in between, and the box joins the
  chain at the stall); new `push_pull_world_anchor_push` (default **true**)
  keeps the claw world-held through the push. Loopback updated, ALL PASS.
- law (`fsc_autopilot_ros2`, 4-D L1 observer): new
  `wb_l1_contact_hold_translation` (default false = byte-identical, parity
  17/17): holds the CoM observer's rows in contact. NEEDED WITH THE CoM ANCHOR
  ONLY; written out `false` in the push-and-pull yaml (pl_4). Python mirror in
  `l1_observer.py` (self-test passes).
- scene: box / table friction default **0.6 / 0.6**.
- yaml: regenerated by `tools/make_push_pull_yaml.py` (the planner key, the
  law key written out false, comments).
- tooling: `run_pl.sh` read array parameters wrongly ("value is:" vs "values
  are:") and now starts the scene `--in-terminal` so its output is logged;
  `am_plant_from_yaml.sh` did not know `WB_SIM_PROFILE=push_pull` (the scene
  launcher died silently in its own terminal window); `pl_mission.py` now opens
  the jaws on a planner ABORT and lets the abort finish (pl_5 climbed with the
  box still clamped and threw it 3.5 m); new `tools/pl_score.py`.

**For the real experiment:**
- **The box's base: static friction = sliding friction** (PTFE glides, felt
  pads) -- a grippy base sticks and then lets go, which is exactly what
  defeated pl_5. Keep mu_s * h < L / 3 (section 1).
- **The heading is the soft axis.** The box yaw follows the EE heading error
  (pl_7: -9 -> +12 deg over the slide): pushing compresses the box, and its
  friction acts 155 mm ahead of the grasp (~0.18 N.m/rad of destabilising
  stiffness against K_psi 0.25). A shorter box along the push, or a grasp
  closer to its centre, reduces it.
- **The push pose is near the wrist singularity** (beta = 10 deg): with the
  box clamped, q1 and q4 counter-rotate +-25-35 deg while q1 + q4 holds the
  heading. Harmless here (peak torque 1.4 N.m), worth watching on hardware.
- Not covered: hardware (there is no hardware push-and-pull block), pull
  (negative distance -- not flown), the hardware |F_hat| noise against the 5 N
  guard.


## 8. The 400 g box on the measured table, the evaluation metrics, and what limits the push (2026-10-05)

User request: the experiment table's measured friction is **0.29**; try a **400 g** box.
0.4 x 9.81 x 0.29 = **1.14 N** to break it loose and to slide it -- the same force as
the 200 g / 0.6 design (1.18 N), so the force budget, the 5 N guard and the tipping
margin (now 3.6x) are unchanged. The scene defaults are now 0.400 kg and 0.29 / 0.29
(07, `PEGASUS_PUSH_*`; static = kinetic kept, section 7). Then: "the arm swang during
the push" (the user's windowed raw flight) -- tune it, scored by the IMPEDANCE RESIDUAL
and the PLATFORM DEVIATION. 25 headless flights (pl_9..pl_33), Isaac at RTF 1.

### 8.1 The metrics (new, `tools/pl_metrics.py`)

- **Impedance residual** `e_imp = F_ext - (M_d edot_v + D_d e_v + K_d e_y)`, per task
  channel x / y / z [N] and heading [N.m], with M_d / D_d / K_d = the law's
  `wb_my_* / wb_dy_* / wb_ky_*` of the yaml the run flew (`--yaml`). F_ext is the TRUE
  wrench on the claw: the scene now publishes the box's PhysX contact report
  (`/push_pull_truth/contact`, normal + friction-anchor impulses kept apart, summed per
  16 ms) and the scorer calibrates the PhysX sign on the table (its normal force must
  hold the box up, its friction must oppose the slide). e_y = the TRUE claw (claw_0)
  minus the streamed r_ed (heading: the law's e_y[3]); every term through the same 5 Hz
  zero-phase low-pass. Self-check on every run: in the steady slide F_ext along the push
  = the table's friction on the box (pl_15: +1.05 / +1.05 N; table normal 4.32 N = the
  box's 3.92 N + the claw's 0.39 N press).
- **Platform deviation** `eps_UAV = ||r_UAV^ref - r_UAV||`, `rho = eps / L`, L = 0.370 m --
  exactly the pick-and-place benchmark's (planned airframe from x_cd, q_d, b1_d on the
  planner's model), with r_UAV the TRUE airframe (`/uav_0/state/pose`, now recorded).
- Windows: HOLD (end of Ready -> slide start: the grip + the 1.5 s settle), SLIDE (12 s),
  RELEASE (push end -> Exit), plus APPROACH / EXIT for eps.

### 8.2 Results

| run | feedback | change (push yaml only) | outcome | SLIDE e_imp xyz rms / pk [N] | SLIDE rho max | HOLD e_imp rms |
|---|---|---|---|---|---|---|
| pl_9, pl_15, pl_23 | raw | shipped | **3 / 3 complete** | 2.61 / 7.3, 2.46 / 4.7 | 0.215, 0.175 | 0.70, 0.98 |
| pl_10, pl_19 | fused | shipped | abort (2 / 2) + the user's windowed flight | | | |
| pl_11 | fused | omega_c_t 2.93 -> 1.0 | abort | | | |
| pl_12 | fused | omega_x 2.0 -> 0.5 | abort in the settle | | | |
| pl_13, pl_16 | fused | omega_x 5.0 | push done then release abort; abort | | | |
| pl_14 | fused | K_y / D_y 80 / 24 | abort in the settle | | | |
| pl_17 | fused | omega_x 5 + k_R / k_w 2.134 / 1.567 | abort | | | |
| pl_18 | fused | omega_x 5 + omega_c_r 0.4 | abort at 13.1 s (414 mm) | | | |
| pl_20 | raw | omega_c_r 0.4 | abort | | | |
| pl_21, pl_22 | raw | K_psi / D_psi 0.6 / 0.5 (+ omega_c_r 0.4) | complete, box yaw +25 deg, box knocked off the table after | 2.83, 2.35 | 0.172, 0.179 | |
| pl_24 | raw | k_v 12.58 -> 20 | left DIRECT in Go To Start (free flight) | | | |
| pl_25 | raw | D_y 26.8 -> 40 | the descent knocked the box 19.5 mm | | | |
| pl_26 | raw | push 12 -> 20 s | abort, box yaw +63 deg, off the table | | | |
| **pl_27**, pl_28 | raw | `wb_attitude_from_odometry` | **complete**, abort (box stuck 3 s at the slide start) | **0.87 / 2.0** | **0.119** | 1.69 |
| **pl_29**, pl_30, pl_31 | raw | `wb_attitude_odometry_correction_rad_s` 1.0 | complete, complete, abort (same signature) | **1.38** / 2.8, 1.61 / 3.9 | **0.112**, 0.213 | 3.77, 1.96 |
| pl_32, pl_33 | raw | `wb_attitude_odometry_correction_rad_s` 3.0 | complete, complete | 1.68 / 4.8, 2.15 / 6.0 | 0.236, 0.215 | 0.94, 0.61 |

Shipped raw, SLIDE: F_ext 1.4 N rms against an e_imp of 2.5 N -- **the impedance is not
realised**; its biggest part is LATERAL (2.0 N rms of the 2.6) and it is K e with no
force behind it (the claw 7.6 mm sideways, the true lateral force -0.04 N). Figure:
`runs/metrics_baseline_vs_attitude.png` (pl_23 / pl_27 / pl_29: residual, push force,
rho through the contact).

### 8.3 What limits it -- the law flies on the wrong STATE in contact, not on the wrong gains

Measured by taking the law's own picture of the claw apart with the ground truth:

1. **Attitude (raw AND fused).** The whole-body client takes its attitude from PX4's
   `vehicle_attitude` (EKF2). In free flight it agrees with the true (mocap) attitude
   to 0.1-0.5 deg; **in contact it drifts to 1-2.9 deg** (logged live in pl_27..pl_33,
   one 5.6 deg spike), while the push shakes the IMU 6-7x harder (horizontal HF
   acceleration 0.29 / 0.24 against 0.04 m/s^2, pl_19). The claw hangs ~0.33 m from the
   body, so the law's claw is **6-7 mm from the real one** (the model FK on the true
   attitude matches the true claw to 0.1-0.2 mm; base position and joint angles are
   exact). The impedance then acts on that phantom: it is the lateral residual, and the
   wrist pair q1 / q4 counter-rotating 10-20 deg is the visible "arm swing" (the claw is
   clamped to the fin, so it is the box that turns -- the claw's lateral position
   follows the box yaw to 0.5 mm). The heading stiffness, the observer bandwidths, the
   base / EE damping and the push speed (pl_20..pl_26) do not touch this, and most made
   it worse.
2. **Position (fused only).** The EKF2-fused odometry the hardware stack flies on is 2-4
   mm from truth in free flight and **10-15 mm off along the push axis (+ ~10 mm std) in
   contact** (raw mocap: 0.5 mm everywhere). Not a lag (time-shifting does not shrink
   it); EKF2 keeps fusing the vision (cs_ev_pos = 1). With the claw world-held, K_y 212
   turns 15 mm into a ~3 N phantom force on a box held by 1.14 N -- every fused flight
   failed, and no gain change (7 tried) survived it.

**The remedy is the state, and it is a code option, default off** (fsc_autopilot_ros2,
whole-body client, uncommitted; every existing config byte-identical):
`wb_attitude_from_odometry` (the law's attitude = the odometry orientation) and
`wb_attitude_odometry_correction_rad_s` (PX4's 250 Hz attitude kept, its slow error to
the odometry orientation removed through a first-order filter; the controller logs
`|odometry - PX4|` every 2 s). Both cut the slide residual (by 20-65 %; in pl_27 and
pl_29 also the push tilt, 4 deg against ~10), but **robustness is NOT shown**: 5 of 7 attitude-fix flights
completed against 3 of 3 shipped, and both failures share one signature (the box stuck
for ~3 s at the slide start with the tilt already 5-6 deg, then stick-slip and a 0.9 Hz
oscillation), and the grip phase got worse at 1 rad/s (HOLD 2-3.8 N). A likely reason,
not fixed: the PLANNER still computes the grasp target and its descent trim with PX4's
attitude, so the law and the planner disagree by the PX4 error and the claw lands a few
mm off the fin. **Shipped yaml unchanged: both keys written false / 0.**

### 8.4 For the experiment

- **Check the estimator in contact first, on the bench**: hold the gripped box with the
  vehicle hovering (or on a stand) and log PX4 `vehicle_attitude` + the EKF2-fused odometry
  against the raw OptiTrack pose. 1-3 deg / 1 cm there would reproduce today's failure.
  (The real IMU is far noisier than Isaac's -- 1.3-1.7 m/s^2 above 2 Hz, sim 2-6 % of
  that -- so the sim may understate it.)
- **The likely fix is a hybrid feedback** for the push: position + attitude from the
  motion capture (exact; on hardware a 120 Hz OptiTrack is ~0.1-0.5 mm), velocity from
  EKF2 (smooth; the 120 Hz finite-difference velocity is what made raw 0921 noisy).
  Needs an estimator / relay change (the law's odometry topic is hard-coded) and the
  planner on the same attitude -- not done tonight.
- Command.md section 17's stack step now uses the RAW stack: the fused one fails in this
  scene (above).
- Not covered: the hybrid feedback; the planner on the corrected attitude; pull;
  hardware; repeat counts beyond 2-3 per configuration (run-to-run scatter is large in
  contact, section 7's lesson).

Tools: `tools/pl_metrics.py` (the metrics; `PYTHONNOUSERSITE=1 /usr/bin/python3`),
`tools/pl_timeline.py` (0.5 s bins of tilt / |e_R| / force / box through a step).
`pl_mission.py` now records the contact truth, the true airframe and the reference
derivatives; each tuning run's yaml is saved beside it as `runs/<tag>.yaml`.

### 8.5 Why EKF2 drifts in contact -- tested (2026-10-06, user question)

Question: is it lag (the vehicle moves suddenly, EKF2 smooths)? **No.** Time-shifting
EKF2's position against the truth by -50..+400 ms does not reduce the error, and the push
is slow (<= 0.09 m/s: 20 ms of lag = 2 mm, not 15). It is an OFFSET plus ~7 mm of wander,
while EKF2 keeps fusing the vision (cs_ev_pos = 1).

**Cause: EKF2 trusts the mocap ~200x less than it deserves, so in contact its IMU-driven
prediction wanders and the vision only weakly pulls it back.** Read live from the
running PX4: `EKF2_EVP_NOISE 0.1`, `EKF2_EVV_NOISE 0.1`, `EKF2_EVA_NOISE 0.1` (PX4
defaults, no script sets them), `EKF2_EV_NOISE_MD 0`. In mode 0 EKF2 uses
max(parameter^2, the estimator's variance) (EKF2.cpp 2226-2310) -> the mocap counts as
10 cm / 10 cm/s / 5.7 deg noisy (the estimator itself sends 1 cm / 1 cm/s / 0.57 deg
roll-pitch / 5 deg yaw, hard-coded in indoor_state_estimator.cpp:172-177); the simulated
mocap is 0.5 mm. In contact the IMU's horizontal vibration is 6-7x larger. The attitude
error is mostly YAW (the weakest anchor): -1.3..-1.4 deg mean in the push (raw runs:
up to 2.6 deg); tilt 0.4 deg. With the claw ~0.13 m ahead of the body, 2.6 deg of yaw =
~6 mm sideways -- the lateral phantom of section 8.3.

`tools/ekf_check.py` (EKF2 position + PX4 attitude vs Isaac truth, per step; needs the
`<tag>_px4.npz` the diagnostic recorder writes), fused runs, one each:

| run | EKF2 vision trust | push: position est - truth (mean / std, along the push) | push: yaw err | free-flight pos std | mission | SLIDE e_imp rms | SLIDE rho max |
|---|---|---|---|---|---|---|---|
| pl_34 | shipped (0.1 / 0.1 / 0.1, mode 0) | -8.7 / 6.6 mm | -1.40 deg | 2-4 mm | push done, diverged in the release | 3.73 N | 0.242 |
| pl_35 | EVP 0.01 | -4.3 / 7.5 mm | -1.28 deg | 0.5-1.7 mm | push done, diverged in the release | 3.20 N | 0.244 |
| pl_36 | mode 1, EVP 0.01, EVV 0.03, EVA 0.05 | -4.7 / 6.9 mm | **-0.64 deg** | 1.3-2.7 mm | **complete** | 2.85 N | 0.266 |

Trusting the mocap more HALVES EKF2's contact offset and yaw error and gave the first
complete fused mission -- but ~7 mm of wander stays and the impedance residual is still
no better than raw mocap (2.5 N). **It cannot close the gap: PX4 hard-codes a 1 cm floor
on the vision position std** (`ev_pos_control.cpp:145`, `sq(0.01f)`), so EKF2 can never
use the mocap at its real accuracy. Conclusion: tune the Pixhawk's `EKF2_EV*` anyway
(they are at the 10 cm defaults unless someone set them -- read them before the flight),
but for the contact task feed the law the motion capture's position AND attitude
directly (the hybrid of 8.4: mocap pose, EKF2 velocity). All four parameters were
restored to 0 / 0.1 / 0.1 / 0.1 and saved after the test (verified). One run per setting.

## 9. The real box, a post handle, the folded arm, and the gripper as the contact switch (2026-10-06)

User: "adjust the handle of the box as well and also determine the appropriate arm pose ...
the gripper will be the switch signal for physical interaction for the disturbance observer
... the interaction should be the force parallel to the table surface." The real box: 240
(along the push) x 160 x 95 mm (mass assumed 400 g, mu 0.29). 15 headless flights
(pl_37..pl_51), scored with `tools/pl_metrics.py`; summary page
`summary.html` (artifact "Push-and-Pull Simulation Results", built by
`tools/report_data.py` + `tools/build_summary_page.py`).

**What changed (now the scenario default):**
- Scene (07): the real box; a 20 x 30 mm vertical POST on its top centre
  (`PEGASUS_PUSH_HANDLE=post|fin`, `PEGASUS_PUSH_POST_TOP` default 0.298 m), so the push line
  passes through the box's friction centre (no yaw lever; the fin sat 0.155 m behind it).
- Push pose [0, 30, 40, 0] deg (beta 70): the claw 0.087 m below the system CoM (was 0.284), so
  a 1.14 N horizontal push is 0.10 N.m of pitch (was 0.32); claw 20 deg below horizontal;
  70 deg off the wrist singularity (was 10). Grasp 0.273 m above the table so the gear clears it by
  ~6.5 cm; tipping mu h/(L/2) = 0.66 (L/3 rule 0.079 < 0.080).
- Planner (fsc_trajectory_planner, default-off options, gtests 8/8): `push_pull_approach_back`
  (approach from behind along the nose), `push_pull_exit_dz`, `push_pull_contact_at_ready` false
  (CONTACT at the Push press = the jaws' grip) and `push_pull_contact_off_at_push_end` true (free
  before the jaws open). Push yaml: approach 0.12 m behind + 0.10 m above, exit 0.20 m.
- Mission takeoff 1.3 m (pl_mission default; Command.md section 17): from 1.0 m the planner refused
  Go To Start (gear vs table, pl_37).

**Results (raw mocap):** 6 / 6 complete (pl_38, 39, 42, 49 at the 60 deg fold [0, 20, 40, 0];
pl_50, 51 at 70 deg). SLIDE impedance residual 2.5 N (side fin) -> 1.27-1.68 N (60 deg) ->
**0.94-0.98 N (70 deg)**; rho_UAV max 0.18-0.22 -> **0.06-0.09**; measured net pitch moment of the
contact force about the CoM 0.18-0.22 -> **0.00-0.12 N.m**; push tilt 9-10 -> 3-6 deg; claw slip on
the handle ~21 -> 2-4 mm. Gripper switch: release residual 1.07-1.35 N vs 1.70-4.11 N on the
old timing (pl_39 spiked to 11.6 N as the jaws opened in CONTACT); box yaw -1..+2 deg vs 5-6 deg.

**Not solved:**
- **The force is still not parallel to the table**: 28-38 deg above it (the claw presses down
  0.55-1.0 N, MORE than the side fin's 0.4-0.55; table normal 4.5-5.0 N vs 3.9 N). ~0.4 N is the
  vertical impedance error, the rest the grip on the post; the 70 deg fold barely changed it. It
  costs little torque now (the claw is ahead of the CoM, so the upward reaction offsets the
  horizontal push's moment) but it loads the box onto the table.
- **Fused (EKF2) feedback: 0 / 6** -- the state-estimate problem of section 8 is untouched by
  geometry (force guard x3 with real stick-slip spikes of 15-17 N, left DIRECT x1).
- **The landing gear can hit the box**: at the grasp the skids straddle the 160 mm box 6 cm clear
  a side but hang BELOW its 95 mm top; approaching at grasp height a skid clipped it (pl_40/41,
  45-52 mm). Fixed by the higher approach; the planner's keep-out knows only the table -- a box +
  handle keep-out is the proper fix.
- The scene table is 150 mm deep, the real box 160 mm wide (5 mm overhang a side).

**The printable handle (2026-10-06, user: "we can 3D print the handle").** `handle/`:
`push_handle.scad` (parametric source: post, base plate, root ribs, M4 holes, centring notches,
grasp-height groove), `push_handle.stl` (solid body only, overlapping closed solids -- slicers
union them; glue or tape the plate), `make_handle_stl.py` (regenerates the STL + preview,
`PYTHONNOUSERSITE=1 /usr/bin/python3`), `push_handle_preview.png`. Post 20 mm ACROSS the jaws x
30 mm ALONG the push, 203 mm above the box top (top 0.298 m, grasp 0.273 m above the table), on a
70 x 90 x 4 mm plate centred on the box top. ~150 cm^3 of model -> roughly 80-110 g in PLA at
4 perimeters / 20-30 % infill: weigh box + handle, it sets the friction force. Checks before
printing: the 20 mm thickness must sit inside the REAL gripper's closing range with squeeze margin
(the custom claw strokes -9.7..+13.2 mm each, absolute widths not recorded in fsc_open_manipulator);
the post height follows from the model's landing gear (skids 0.313 m under the body origin) --
re-derive it if the real gear differs.
User's answers (2026-10-06): the real gripper opens to 45 mm with foam/rubber pads (held a smooth
200 g weight) -> 20 mm is fine; **the experiment drone is the T650, whose landing gear is ~7 cm
SHORTER** than the X650 the sim models (airframe bottom plate to ground 22-23 cm vs 29-30 cm). With
the printed post (grasp 0.273 m) the T650's gear clears the table by ~13.5 cm at the 70 deg pose
(sim: 6.5 cm), and a more horizontal claw becomes possible on hardware: [0, 40, 40, 0] (claw 10 deg
below horizontal, gear ~9 cm) or [0, 45, 45, 0] (horizontal, ~5.7 cm, q2 5 deg off its stop). The
planner already takes `push_pull_gear_depth` (0.313 = X650; ~0.243 for the T650, assuming the same
body-origin-to-plate offset); the Isaac asset still has the X650 legs, so the sim cannot fly those
folded poses at this grasp height without shortening them.

### 9.1 The T650 landing gear in Isaac (2026-10-06, user request)

`AM_T650.usda` = a COPY of `AM_xfwd.usda` with the landing gear 0.070 m shorter
(`robotic_arm/utils_model/make_t650_gear_asset.py`, run under `~/isaacsim/python_r_fsc.sh`; the
original is verified untouched). The gear is part of the one body mesh
(`/gripper_bat/body/body`, 550 k points, convexDecomposition, no cooked collision data): every
point below z = -0.06 m (body frame) is compressed linearly in z (k = 0.7228), so the skids end at
-0.2425 m (was -0.3125), the struts stay straight, nothing above the frame moves. The push scene
loads it by default (`PEGASUS_PUSH_AM_ASSET`, resting body height `PEGASUS_PUSH_GROUND_BODY_Z`
0.235 m -- measured 0.235 in pl_52); the planner plans with the T650 gear
(`push_pull_gear_depth: 0.2425` in the push yaml); the scripted landing goes to 0.24 m
(`pl_mission --land-z`), the manual one needs `ps4_teleop_bringup.py land --land-z 0.25`.

Flown (raw, gripper contact switch):

| run | pose | outcome | SLIDE e_imp | rho max | contact force: down-press, angle | table normal |
|---|---|---|---|---|---|---|
| pl_52 | 70 deg [0, 30, 40, 0] | complete | 1.11 N | 0.086 | 0.56 N, +24 deg | 4.48 N |
| pl_56 | 70 deg | complete | 2.47 N | 0.109 | 0.51 N | 4.44 N |
| pl_54 | 80 deg [0, 40, 40, 0] | aborted in the settle (force guard; the box moved 18 mm before the slide) | | | | |
| pl_55 | 80 deg | push done, then left DIRECT in the release | 1.20 N | 0.086 | **0.01 N, ~0 deg** | **3.93 N = the box's weight** |

(pl_53 never flew: the stack's ROS feeds did not come up in 90 s.) The 70 deg default works on the
T650 gear (2 / 2, a large run-to-run spread: 1.11 vs 2.47 N). **The 80 deg fold makes the contact
force PARALLEL to the table** -- the user's goal -- but its grasp and release are not stable yet
(0 / 2 complete); the claw is ~10 deg below horizontal there, and the model's claw point lies 39 mm
beyond the pads along the claw axis, so how the jaws meet the post changes. Not adopted.

### 9.2 Inside the real arm's q2 range [-20, 45] deg, pushing and pulling (2026-10-06, user request)

The user measured q2 at about [-20, 45] deg on the real arm (q3 fine). The scene now authors it as
hard stops on `manip_joint2` at spawn (`PEGASUS_PUSH_Q2_LIMIT_DEG`, default `-20,45`; `asset` keeps
-90..50; forwarded by the push launcher). q3 keeps its +50 stop and must stay above 0 (below is the
elbow-singular branch). The planner's own joint box is a compiled shared constant (q2 <= 50) and was
NOT changed. All runs: T650 gear, raw mocap, gripper contact switch, 400 g, mu 0.29. A pull starts the
box at y = -0.25 and uses `push_pull_push_distance: -0.50`.

**The mechanism.** The claw is world-held, so the arm takes up every millimetre the airframe drifts
from its plan: 4-5.5 deg of q2 and 6-8 deg of q3 per cm along the arm, whatever the fold (planner
model). The fold only sets the window before a limit: [0,40,40,0] 1.3 / 1.6 cm, [0,30,40,0] 3.6 /
1.6 cm, [0,26,24,0] 3.3 / 3.5 cm, [0,22,18,0] 2.3 / 4.2 cm (until q2 = 45 or q3 = 0 / until q3 = 50).
In contact the airframe drifted 10-29 mm (std), peaks 30-40 mm. Earlier 70 deg runs (asset stop)
already reached q2 47-50 and rode q3's +50 stop 22-32 % of the push.

| config (q2 stops -20..45 unless noted) | runs | result |
|---|---|---|
| push 70 deg [0,30,40,0] | pl_66 | 0/1: q3 pinned on +50, box shoved 24 mm in the settle, force guard at 1.5 s |
| push 50 deg [0,26,24,0] | pl_62 | 0/1: 313 mm, then q2 -> 45 and q3 < 0, guard |
| push 40 deg [0,22,18,0] | pl_63 | 0/1: same, 327 mm |
| push 50 deg, `wb_l1_contact_hold_translation` true | pl_67 | 0/1: airframe 55 mm off its plan from the grip |
| **push 50 deg, `wb_l1_omega_c_t` 2.927 -> 1.0** | **pl_71, pl_74** | **2/2 slid 0.50 m**: pl_71 complete (SLIDE e_imp 1.04 N, rho 0.085, q2 12-39, q3 8-42); pl_74 slide 0.84 N, rho 0.067, force 0.6 deg from parallel, then failed at the exit (the arm swung after the release while the slow observer still carried the box force) |
| pull 70 deg, asset stop (q2 <= 50) | pl_73 | 1/1 complete: 1.20 N, rho 0.067, force 6 deg from parallel, q2 peaked 42 |
| pull 70 deg | pl_76 | 0/1: q2 -> 45, q3 < 0 at 5.3 s |
| pull 50 / 40 deg | pl_64, pl_65 | 0/2 |
| pull 50 deg, `omega_c_t` 1.0 | pl_72, pl_75 | 0/2: the box ran ahead of its reference at mid-slide (0.3 m/s vs ~0.09 planned), then q3 < 0 |

Drift with the shipped observer at 50/40 deg: 15-29 mm std; with `omega_c_t` 1.0: 7-11 mm. Its
dominant frequency varies 0.15-1.5 Hz between runs, so the observer is a contributor, not the only
source. `omega_c_t` is global: the approach error rose to ~0.08 of the reach. Why the box runs ahead
in the pull is NOT explained.

The EE reference stays parallel to the table: the planned claw height varies 1.3-1.4 mm over the
0.50 m slide (the planned CoM height is exactly constant) -- the arm is held still while the planned
airframe tilts ~0.1 deg to accelerate.

Nothing adopted (the push yaml is unchanged). Next: a contact-only translational observer bandwidth
(code), the 50 deg pose, the planner's q2 limit as a parameter, the pull's run-away, repeats.
Tools: `tools/plot_q2limit.py` (joint window figure), `tools/report_data.py` (now also q2/q3 window,
q3-stop time, airframe drift std/frequency, failure time and box travel at failure), the scene knob
in `07_px4_direct_t650_aerial_manipulator_push_and_pull.py`, `contact_hold_translation` in
`utils/wb_l1_set_gains.py`. Report: artifact Pq9VWkJk2XqmKM7dymZ2Q4, version 3.
