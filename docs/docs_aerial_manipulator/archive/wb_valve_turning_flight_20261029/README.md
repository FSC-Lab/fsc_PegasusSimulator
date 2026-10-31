# 2026-10-29 hardware valve-turning flights (whole-body 4-D L1)

Two bags in `docs/experimental_data_ros2_bag/1029 - T650-AM whole-body Tracking & Valve-Turning-*/`:
- `flight_wb_l1_4d_valve_turning_20261029_145128` (tag `145128`): hovered only.
- `flight_wb_l1_4d_valve_turning_20261029_145526` (tag `145526`): grasped the fin, then failed and crashed.

Flown config: the hardware valve yaml at fsc_autopilot_ros2 `2431025` (90° in 18 s after a 1.5 s settle, pose
[0, 5, 25, 0], K_y 120 / D_y 20.2, K_ψ 0.25, free-axis anchor 0.75 latched at the room hold, ω_c_t 2.0,
rotor-tilt guard 4°, q3 rate guard 0.1 s / 30°/s / q2 wall 45, exit hold 0.06 m, redesigned markers).
Arm: `external_torque_controller_hardware_aerial_pwm.yaml` (back-EMF feed-forward on the observer velocity,
scale 1.0; error damping only on j4, 0.05; j4 friction relay 0.030 N·m; `command_timeout_s` 0.3).

Figure: `fig_flight.png`. Timeline: `timeline.txt`. Tools: `tools/` (run `run_extract.sh` first; the 10-28
tools are copied here and pointed at this folder's npz).

## Flight 1 (145128): a setup error, caught by the planner

Plan was refused: "the beam (obj_0 +x, yaw -0 deg) points 179 deg away from the start". The obj_0 rigid
body had been created with its +x away from the fin. The vehicle was flown back and landed by the pilot.
At 77.5–79.4 s obj_0's yaw flipped 180° with 0 mm of position change (the body re-defined in Motive). The
planner's box gate dropped those samples, then accepted them, and Get at 81.6 s captured yaw 180°. Flight 2
used that capture. The layout check worked as designed.

## Flight 2 (145526): what happened

| t (s) | event |
|---|---|
| 32.29 | Go To Start done; claw room-held above the handle. CoM error sd 2.9 / 3.4 mm (calm) |
| 33.40 | the jaws start opening — **the wrist roll q4 spins to 41° at 45°/s and sticks there** |
| 33.93–36.96 | Ready descent with the claw heading up to **40° off** its reference; the open jaws push the fin (valve −5°) |
| 38.84 | jaws closed on the fin, 18 mm from the grasp point; base **38 mm sideways off**, q1 +10° |
| 40.34 | the turn starts (plan ≤ 3° until 44.5 s) |
| 39–45.5 | **growing coupled sway**: valve ±20–27°, CoM ±90 mm along the nose and sideways |
| 45.63 | rotor allocator saturated (one rotor at 0.00) |
| 45.70 | q3 RATE guard: "q3 6.2° falling at 93°/s" → ABORT (release, claw room-held) |
| 45.97 | arm torques at the 3 N·m clamp |
| 46.30 | DIRECT watchdog (tilt 15.1°, 114°/s) → SAFETY |
| 46.8 | 61 m/s² impact: the hanging arm hit the valve (valve knocked 22°) |
| 46.99 | mocap guard rejects the drone's real 2.8 m/s motion as a "jump"; LOST at 48.0 |
| 47.6 | 69 m/s² impact: the arm hit the floor (body at 0.35 m) |
| 49.42–52.05 | VRPN loses the drone; it is on the floor at (4.54, 1.76, 0.24) when tracking returns |
| 53.03 / 55.87 | pilot STAB / disarm |

## 1. The cause: the clamped contact phase is unstable (3 of 3 hardware contacts)

The same mode as 10-28, and today it ran long enough to reach the arm's limits. `compare_1028.py`:

| flight | clamped | sideways growth | along-nose growth | mode | how it ended |
|---|---|---|---|---|---|
| 10-28 180652 | 9.2 s | ×2 / 5.1 s | — | 0.56 Hz | false tilt trip |
| 10-28 181833 | 3.6 s | — | ×2 / 2.2 s | 0.59 Hz | false tilt trip |
| **10-29 145526** | **6.9 s** | **×2 / 3.4 s** | **×2 / 3.0 s** | **0.45 Hz** | **arm collapse** |

- **The turn is not the trigger.** The sway grew from the grasp while the planned angle was still ≤ 3°.
- **The valve angle = the free-axis anchor carrying the along-nose wander:** corr 1.00, slope 0.28 °/mm,
  predicted 0.75/r = 0.27 °/mm (`coupling.py`). The valve swung ±20–27° with the plan at ≤ 8.6°.
- **Along-nose and sideways motion are one whirl:** the two are phase-locked at 0.46 Hz (corr 0.76 at a
  ~120° phase).
- **The sideways part is absorbed entirely by the q1/q4 antiphase pair** (q1 −0.25 °/mm, q4 +0.33 °/mm,
  corr −0.99). That pair is not a 4-D task row, the back-EMF feed-forward cancels the motors' own damping,
  and j1 has no error damping, so nothing damps it.
- **The grip is not rigid** (`grip_rel.py`, FK claw vs obj_0, since the grasp): the claw rotates against the
  fin 9.9° rms about the shaft (up to 42°) and 4.5° rms about the tangent (up to 14°, corr −0.92 with q4).
  The fin itself rolls with the claw again (`fin_roll.py`: centroid-height corr −0.96). The observer booked
  a growing roll moment (+0.08 → +0.28 N·m) and a j1 torque (+0.11–0.19 N·m) in contact.
- **The collapse:** at 44.7–45.7 s the base moved 0.13 m back and 0.30 m sideways in ~1 s. The clamped arm
  was stretched: q3 went 20° → −32° (the elbow-singular branch) at up to ~100°/s. The rotors saturated
  first (45.63 s), then the q3 rate guard tripped (45.70 s, 0.1–0.2 s before q3 crossed 0).
- Not the cause: the localization (fitted shaft 2.6–3.5 mm from the planner's axis, centroid radius
  96–97 mm vs 93.5 CAD, `axis_fit.py`); the tilt guard (peak 1.8° of 4°); the mocap feed before the crash.

## 2. Why the failure became a crash

1. **The abort held the claw in the room while the base ran away.** `push_pull_abort_release_world_held`
   keeps the claw target fixed for the 2 s release. Here the base was already moving at 0.3 m/s, so the law
   pulled the stretched arm against it: arm torques at ±3 N·m from 45.97 s, rotors saturated, tilt 8° → 15°
   in 0.6 s.
2. **The SAFETY handover kept the law's last arm command for 0.3 s.** On hardware the whole-body node stops
   streaming in SAFETY; the arm controller keeps applying the last command until `command_timeout_s` (0.3 s).
   That command was [+0.01, +3.0, +3.0, −0.06] N·m (j2 duty at the full 885). The arm whipped
   (q2 13° ↔ 44°) while PX4 took over at 15.9° tilt and −145°/s pitch rate. Tilt reached 41° at 46.6 s and
   57° at 46.9 s.
3. **The mocap guard rejected real motion.** The jump gate is a fixed band around the LAST GOOD sample
   (20 mm + 1 m/s·dt + 1 m/s²·dt², `mocap_guard.cpp:81`), with no velocity prediction. At 2.8 m/s
   (22–30 mm per sample) the drone left the band at 46.99 s and could not get back in: CONFIRMED 47.14,
   LOST 48.0. Vision stopped reaching EKF2. By 49 s EKF2 was 0.65 m off in x and 0.34 m in z. SAFETY flew
   on that estimate: the vehicle reached the room edge (4.68, 2.26), dropped out of mocap, and came down
   on the floor.
4. Contributing: **the standing 0.2 N·m yaw trim** (rotor pairs at 0.52 / 0.72 in free hover, both flights;
   the loose-arm imbalance since 10-14). It uses ~30 % of the yaw budget and leaves the high pair ~0.18
   from saturation.

## 3. The wrist kick at the jaw opening (new)

When the jaws open at the start of Ready, q4 spins and sticks (`tools/grip_kick.py` over 16 bags):

| | valve flights | box pushes |
|---|---|---|
| wrist kick at the pre-Ready opening | 2 of 3 (24°, 41°) | 0 of 13 |

Today the kick started within 50 ms of the gripper's motion, while the law's j4 torque stayed < 0.07 N·m.
Why j4 cannot stop it: the heading spring is only ~0.044 N·m/rad at the joint, and the back-EMF
feed-forward removes the motor's damping. Consequences:
- the open jaws reach the fin rotated (heading error up to 18 / 34 / 40° in the three flights);
- they push the free valve before the close;
- the grasp starts off-center (today: 38 mm sideways, q1 +10°).

## 4. The ceiling

- Sway growth σ ≈ 0.14–0.23 1/s, from 15–40 mm at the grasp to ~100 mm (where the arm runs out of range):
  **~5–8 s of clamped contact.**
- The flown turn needs 19.5 s (1.5 s settle + 18 s). In 5–8 s the 18 s plan covers < 10°.
- So **a 90° turn is out of reach** with this grip and law, whatever the reference. A faster or shorter
  turn does not help: the sway grows with the valve standing still.
- It is not thrust or bandwidth: free flight is fine (hold above the handle 3 mm sd, circle 24–26 mm).
  The limit is the closed chain through a compliant grip.
- It cannot be tuned in sim: Isaac does not reproduce the mode (10-28 §5–9), and the 10-29 gain sweep found
  no gain that helps.

## 5. Early-abort replay (`early_guard_replay.py`)

Each criterion persisting 0.2 s, on the three hardware contacts:

| criterion | 10-28 180652 | 10-28 181833 | 10-29 145526 |
|---|---|---|---|
| sideways CoM > 60 mm | 3.8 s before the end | never | **4.0 s before the collapse** |
| along-nose CoM > 60 mm | never | never | 3.8 s before |
| \|Δq4\| > 20° | 3.7 s before | never | 3.1 s before |
| \|e_R\| > 6° | at the end | never | 4.4 s before |

At those instants the vehicle was at 2–4° tilt. These replays are on three runs only; check any threshold
against Isaac's completed turns for false trips before adopting it.

## 6. A joint on the handle (2026-10-30, user question; `tools/handle_joint_mobility.py`)

The user proposed a roll joint along the fin, at its top (sketch: horizontal pin along the fin's 40 mm
width = the turn's tangent = the claw's nose direction).

**Why the grip passes couples at all.** With the jaws clamped, the claw point is fixed. q1-q3 are used
up holding it, and q4 can only spin the claw about its own axis (60° below horizontal at the flown pose).
So every claw rotation square to the claw axis is forced by the vehicle's motion and taken by the pads,
fin and printed shaft. The 4-D law controls position and heading only; it never sees these two tilts.
The forced rotation per vehicle motion, flown pose [0, 5, 25, 0], valve held by its friction:

| vehicle motion | grip rotation (radial, tangent, vertical) | horizontal tangent pin leaves | pin 30° up, square to the claw, leaves | tangent + radial (2-axis) leaves |
|---|---|---|---|---|
| roll 1° | (0.01, 0.23, 0.13)° | 0.13° | 0.01° | 0.13° |
| pitch 1° | (0.13, 0, 0)° | 0.13° | 0.13° | 0 |
| sideways 10 mm | (0.01, 1.16, 0.67)° | 0.67° | 0.01° | 0.67° |
| fore-aft 10 mm | (0.98, 0, 0)° | 0.98° | 0.98° | 0 |
| vertical 10 mm | (−3.51, 0, 0)° | 3.51° | 3.51° | 0 |

- Sideways 1.16° tangent per 10 mm matches the flights' 0.10-0.15 °/mm claw roll, and the measured
  claw-vs-fin rotation: tangent 4.5° rms (corr with q4 −0.92), radial 2.3° rms, vertical 9.9° rms
  (uncorrelated with q1/q4: the valve's swing).
- Roll and sideways sway give a rotation about one axis: in the fin's plane, square to the claw axis
  (30° above horizontal, rising toward the drone's nose). A pin along it frees them completely. The
  horizontal pin frees the tangent part; the vertical part (half the angle) is left to the pads and the
  valve, which already give in about the vertical. The same axis also takes the yaw the valve's own
  turning forces on the claw.
- Pitch, fore-aft and vertical give a rotation about the radial (jaw-closing) axis. Vertical is the
  largest per mm (the upper arm hangs nearly vertical, so q2/q3 cannot lift the claw without pitching
  it). In flight this one was smaller (2.3° rms), and the valve's turning takes most of the fore-aft.
- Same picture over the arm range of the turns ([0, 10, 20, 0], [0, −10, 45, 0], q1/q4 ±10/∓13).
- The 10-28 reduced model (§9 there): a handle free in roll returns the sideways decay to the free-flight
  −1.92 1/s (−0.05 with the stiff fin). A free joint removes stiffness; it does not damp by itself.

**Advice given:** yes, build it, as the user sketched; tilt the pin to square with the claw if that is
free to print; a radial second axis (small universal joint) is the optional next step; leave yaw locked.
Pin axes through the grip point (25 mm below the fin top) or within ~2 cm; pivot above the paddle's CoM
so it hangs plumb before the grasp (gravity stiffness ~5 mN·m/rad vs the 3-7 N·m/rad path it removes);
light friction (~10-20 mN·m, below the ~70 mN·m couple) to hold it before the grasp and to dissipate;
an axial stop (the turning push runs along the tangent pin); no backlash; LOCKABLE (set screw or dowel)
so D1 (10 s clamped hold, no turn) can be flown locked vs free on one battery. Isaac does not reproduce
the hardware mode, so it can check only the mechanics (grasp, turn, release). Not a fix for the wrist
kick (§3), the valve following the along-nose wander, or the yaw imbalance; recommendations 1-5 stay.

## Recommendations

**Status 2026-10-30:** safety items 1-4 are implemented (`archive/valve_safety_fixes_20261030/README.md`: a sway guard on the
CoM-error rate, the CoM-anchored arm-guard abort, the arm RELEASE at the SAFETY handover, the mocap jump gate centred
by EKF2's velocity); not yet built everywhere, loopback-run or flown. Item 8 is designed
(`archive/valve_handle_joint_20261030/README.md`, section 4 of the Valve Turning Design page). The rest: none applied.

**Before the next valve flight (safety):**
1. An abort on the sway, not on the arm: e.g. sideways or along-nose CoM error > 60 mm for 0.2 s in
   contact (planner guard). Today it would have fired 4 s before the collapse.
2. For an arm-guard abort (q3 static or rate), drop the room hold at once (CoM anchor) and open. The
   world-held release suits tilt and handle guards, not a base already running away.
3. SAFETY handover: stream the PD+gravity hold from the node in SAFETY on hardware too
   (`wb_arm_hold_stream`, as in Isaac), or make the arm controller drop the streamed command on the mode
   edge. Never replay the diverging law's last torque for 0.3 s.
4. Mocap guard: centre the jump gate on the last good sample + EKF2 velocity × dt (or on EKF2's predicted
   position), so real fast motion is not rejected.
5. Re-true the airframe's yaw imbalance (0.2 N·m standing trim).

**To raise the ceiling (each needs a hardware A/B; Isaac cannot rank them):**
6. Grasp: open the jaws earlier (e.g. at the start of Go To Start, far from the valve) so the wrist kick
   settles before the descent. Add a heading check to the close gate (it checks position only).
7. Damp the q1/q4 path: `passthrough_error_damping_joints` on j1 and j4 (e.g. 0.2). With the claw pinned,
   a joint damper b on q1 acts on the base's sideways motion as ≈ 19·b N·s/m (q1 moves 4.4 rad per m), so
   0.2 adds ~30 % to k_v. Config-only.
8. Mechanics: a handle the closed jaws can turn on in roll and about the shaft (e.g. the fin on the beam
   through a small spherical joint), so the grip stops passing a roll couple into the arm and the base.
   10-29 §9's reduced model says this restores free-flight damping.
9. Law: the free-axis anchor shifts the claw's position target along the tangent (±50–60 mm here) but not
   its heading target, which twists the grip against the fin by up to ±20°. Rotating the heading target
   by w·e_f/r as well would keep the two consistent.
10. The 10-28 diagnostics are still the cheapest way to identify the mechanism on hardware (with guard 1
    on):
    - D1: a 10 s clamped hold with no turn (`push_pull_push_settle_s` 10);
    - D2: the same with `push_pull_contact false`;
    - D0: twist the clamped claw by hand on the ground.
