# Fixed-wing flight on the FSC tilt-quadplane — what exists, what was missing, and why

*(2026-08-10 session: AVTOL reference generators wired in + scaled, profile-driven
fixed-wing cruise built into two controllers, 14 instrumented HIL flights, every failure
rosbag-diagnosed. Bags in `/tmp/bags/{fwD1..fwD10, fwE..fwE8, stop1}`.)*

## TL;DR

* The AVTOL SO(3) reference generators (`transition_reference_consistent.py` and friends)
  are now **wired in and scaled to this 3.5 kg airframe** (`ros2/px4_vtol_direct_actuator/
  px4_vtol_direct_actuator/fsc_transition_ref.py`). The scaled trim schedule is
  force-balanced and self-tested: hover 34.9 N ≈ weight → cruise @15 m/s: tilt→90°,
  α=+6.9°, thrust 3.4 N ≈ drag, wing carries ~100%.
* **D (`nmpc_cruise`, GS button D) is the validated fixed-wing fly-to-point**: the profile
  drives the NMPC's tilt/pitch/thrust references; 3 consecutive 250 m missions arrived
  (fwD8/9/10) with a clean hover→fixed-wing transition (tilt 0→65°+, rear pair unloading
  to near zero, altitude dead-flat through the hand-off, elevator trimming).
* **E (`so3_cascade`, GS button E) is the validated geometric fly-to-point** (landed
  0.12 m from a 60 m target, fwE8) in its multicopter tilt-for-forward mode; its
  profile-following fixed-wing branch is implemented but still **experimental**
  (`CASC_FW=1`) — its traverse-engage transient is not yet robust (5 bags, each root
  cause below).
* **Why there was no good fixed-wing flight control method** — it is not one bug; it is a
  stack of six, each of which silently sabotaged every earlier attempt. All six are now
  identified, five are fixed, one (the last 15% of the lift hand-off) is characterised
  with a concrete design for the fix.

## The six reasons there was no good fixed-wing controller

### 1. The NMPC never actually tilted: receding-horizon procrastination (FIXED)
`nmpc_cruise`'s control-rate penalty was anchored to a **hardcoded hover `Uprev`**
instead of the previously applied control. Every solve paid `w_du·β₀²` to tilt *at step
0 only* — so the solver planned a beautiful transition and perpetually deferred it past
the first step. Closed loop: rotors stayed near-vertical forever. This is the mechanical
explanation for the long-standing "wing-assisted multicopter drift, tilt≈0" finding in
the ★★ audit. **Fix:** `uprev` passed through the parameter vector, updated to the
applied control each cycle (`nmpc_cruise_node.py`).

### 2. No force-consistent reference to track (FIXED — this is the AVTOL wiring)
Every earlier controller invented its own tilt/pitch schedule (heuristic trim ramps,
force-sized tilt, speed-gated θ-weights). None of them encoded the *trim manifold* —
which tilt+AoA+thrust combination is force-balanced at each airspeed. The AVTOL
`TransitionRefInterp` solves exactly that (thrust-limited coupled trim), and it is
scale-free: re-parameterised with m=3.5 kg, S=0.35 m², CLα=4.8, T_max=60 N it produces
the correct schedule for this airframe (wing-borne from ~13.3 m/s at α≤8°). Both
controllers now track it; the NMPC additionally gets the profile's **thrust hand-off**
(`w_T_ref`) — without it the solver sat in a rotor-borne KKT trap (level body, rear
rotor carrying 23 N) that tilt/pitch tracking alone could not escape.

### 3. The point-mass model is blind to pitch balance (FIXED)
The longitudinal model treats Tf/Tr as pure forces. When the profile tilts the front
pair early, the solver "compensated vertically" by loading the **rear** pair — creating
a real nose-over moment the attitude loop cannot trim (fwD2: mF 0.38 vs mR 1.00 →
backward crash in 5 s; identically fwE4 for the geometric split). **Fix:** a
pitch-balance cost term `w_pm·(Tf·cosβ·x_f − Tr·x_r)²` (measured CG arms, cg_x=0.177)
— which at high tilt *also* drives the physically correct rear unload. The geometric
version uses a cosine-blended split (equal pairs at low tilt → forward-priority at high
tilt).

### 4. Bistable replanning at the wing-borne boundary (FIXED)
Near the hand-off the thrust-vs-wing trade is bistable: the solver bang-banged the tilt
80°↔0° between solves (fwD4), and gambled on full tilt at 11.6 m/s where the wing could
not yet carry (fwD3 stall-drop). **Fixes:** hard-bound β by the schedule
(`tilt_profile(V)+6°` — the profile *is* the certified feasible trim), slew the applied
tilt (35°/s, which also re-anchors the rate penalty), raise the β rate cost.

### 5. Garbage state at the envelope edges (FIXED)
* AoA/flight-path measured below ~2 m/s is noise; the old 0.2 m/s gate let a hover drift
  produce θ_meas = 9° garbage → the attitude loop executed a bogus 8° nose-down at hover
  and doomed the run before the transition (fwD7 — also the origin of run-to-run
  "flakiness"). Gates now at 2 m/s.
* A standby node (`aero_transition_node`, window 6b) was **co-publishing actuators at
  200 Hz** against the active controller (constant 45° tilt + zero elevator stream —
  fwD1/fwE2; the stream stopped the instant that node died). It is now disabled by
  default (`RUN_TRANSITION_C=1` to re-enable) and `nmpc_so3_cascade` was added to the
  teardown patterns (it survived every teardown before).

### 6. The last 15% of the lift hand-off is structurally hard (CHARACTERISED — open)
Every bag shows the same boundary: **rock-solid to ~65° tilt / wing ~80–85%; fragile
beyond.** Three structural causes on this plant:
* **One-sided rotor pitch authority.** Once the rear pair unloads to ~0, the
  differentials can push it up but not below zero — pitch control becomes asymmetric
  exactly when the wing's lift sensitivity (∂L/∂α ≈ 250 N/rad at 15 m/s) is largest.
  fwD8: body pitch ran to +14° against a 9° command → AoA +40 → stall → dive.
* **The phugoid.** With rotors floored and θ deliberately slow (the historical recipe
  that stabilised partial wing-borne), vertical-loop bandwidth collapses and the
  V↔z energy exchange (period ~6–10 s at 13 m/s — the classical phugoid) is only weakly
  damped. The historically stable wing-borne runs (wb10/11) kept **~50% rotor reserve**
  — that reserve *is* the damping bandwidth. Amplitude ±10 m in cruise at reserve ≤3 N;
  still present at 6 N.
* **No fast lift-shedding actuator.** Nose-down is capped at 3° (for good reasons —
  history shows nose-down → downforce dives), so when the wing over-lifts the only
  authority left is slow.

**The design that closes this** (not attempted this session): a 6-state longitudinal
NMPC with pitch *dynamics* (θ, q states; elevator + differential thrust as moment
controls) so pitch/lift is co-optimised with tilt/thrust at the boundary — plus a
distance-scheduled decel reference for aborts. Until then, the default envelope is
capped at 65° tilt (`FSC_TILT_CAP_DEG` env raises it for research).

## What is validated now (HIL, rosbag-verified)

| Path | Status | Evidence |
|---|---|---|
| FSC-scaled AVTOL trim schedule | self-test PASS (force balance, hand-off) | `fsc_transition_ref.py --selftest` |
| D: hover→fixed-wing transition | clean, repeatable: tilt 0→65°+, rear→0, alt flat | fwD8/9/10 (t≈12–20 s each) |
| D: 250 m fly-to-point | 3/3 arrivals, ~35–40 s | fwD8/9/10 |
| D: cruise altitude | ±10 m phugoid (open issue #6) | fwD8/9/10 |
| E: fly-to-point + precise landing | landed (59.88, 0) vs (60,0) target | fwE8 |
| E: fixed-wing branch | experimental (`CASC_FW=1`); engage transient unresolved | fwE2–fwE7 |
| STOP from hover / ≤6 m/s | safe (geometric catch, forced by the GS fix) | this + prior sessions |
| STOP from full fixed-wing cruise | **NOT survivable** — tilt-up at speed → zoom-climb → stall | stop1–stop3 |

## Operational notes
* Launcher: `scripts/start_vtol_smc_hil.sh <cfg>`; knobs `FW_VCRUISE` (15), `FSC_TILT_CAP_DEG`
  (65), `CASC_FW` (false), `RUN_TRANSITION_C` (0), `AVTOL_REF_DIR`.
* Fly D: take off on A, climb (raw ref), switch `active_controller=nmpc_cruise`, publish a
  far raw reference. Fly E: set `target_x/…` params, switch `active_controller=so3_cascade`.
* GS: buttons D/E + "Fly to point + LAND" run on the selected controller; STOP+SAFE-HOVER
  streams hover-at-current immediately and stages the geometric handover on deceleration.
* GS Reference dropdown now includes FSC-scaled AVTOL trajectories
  (`fsc_fixed_wing_transition.csv`, `fsc_transition_climb_turn.csv`); the exporter's
  `--consistent` velocity-index bug is fixed.
* Test drivers: `/tmp/run_d_test.sh`, `/tmp/run_e_test.sh`, `/tmp/run_stop_test.sh`;
  analysis `/tmp/anabag.py`, servo forensics `/tmp/srvhist.py`.

---

# UPDATE (same day, late session): the user's SO(3) NMPC flying 90° IN-LOOP

Directive: use the AVTOL NMPC derivations directly (`nmpc_vtol*.py`). Chosen variant:
`nmpc_vtol_reduced_pos_att_so3.py` (22-state SO(3) pos/att/tilt) via its existing faithful port
`nmpc_so3_transition_node.py` — full 6-DOF predictor, SO(3) attitude cost, `s_from_beta`
path anchoring. The other variants: full Kane models too heavy; `reduced_vel_rate` (19-state,
no position) is the lightest fallback if an inner-loop-only NMPC is ever wanted.

## What made it real-time (the old blocker)
* N 25→12, DT 0.06→0.10, RK4 substeps 2→1, **JIT-compiled IPOPT** (`jit` + `-O2`):
  **20 ms mean / 36 ms p95** per solve (was 240–940 ms) → true 25 Hz in-loop.
  One-time ~5 min compile, cached via `jit_serialize=embed` → later starts load in ~1 s
  (`~/.cache/fsc_so3_nmpc_*.casadi`). **The cache key must include the cost weights AND
  max_iter** — serialized solvers bake their options in; a stale cache silently no-ops changes.
* `MAX_ITER=60` REQUIRED: the tilt 35–45° corridor solves need up to 60 iterations
  (50 → deterministic failure cascade → crash; verified offline both ways).
* fatrop is present in CasADi 3.7.2 but was ~292 ms here (structure detection degraded);
  sqpmethod+qrqp RTI diverged. JIT'd IPOPT won.

## The fixes that made it fly the CORRECTED physics
1. **Lift-sign fix in the ported aero** (`aero_body`): the port's aero-frame decomposition
   pointed positive-α lift DOWN in FLU — self-consistent offline (trimmed at α=−9°!) but
   inverted vs the sim → nose-down dive + solver failure cascade in HIL (bag so3a/anabag).
   Now mirrors `vtol_aerodynamics.py` verbatim (drag along −v, lift along [−sin α,0,cos α],
   + CL_q, CL_de). Model version tag in the solver cache key ("m2").
2. **Reference = the user's own trim generator**: `TransitionRef` schedules (V(s), Ttot(s),
   trim-pitch attitude ref, de(s)) now come from the FSC-scaled `TransitionRefInterp`
   (`fsc_transition_ref`, tilt_cap=β_cruise). A LEVEL attitude ref fights the wing trim at
   high tilt; the trim pitch is faded in by path progress (full by s=0.15).
3. **Rate damping in the plan** (`w_om=3`): roll blew up in the corridor where front-pair
   differential turns into yaw authority. + `w_att` 40→120 (stick to the trim attitude),
   `w_du` 3→6.
4. **Time-ratchet path anchor with a leash**: anchored purely on achieved tilt, the
   corrected-physics system parks in a rotor-borne local optimum at ~30° (beyond-horizon
   myopia). Elapsed time advances the anchor, leashed to ≤0.15 ahead of tilt-progress
   (their design intends the path traversed in tf; the leash prevents reference runaway).
5. Rear-pair thrust floor relaxes with s (4 N → 0.3 N): the static floor contradicted the
   pure-puller trim. Freeze the tilt integrator on failed solves (no stale-command creep).

## Results
* **Offline (latency-emulated 0.12 s, jitted): FULL 90° transition + indefinite steady
  pure-puller cruise** — β=90.0°, V=13.8, z dead-flat, **AoA +7.2° = the true trim**,
  rotors ~6 N total (front pulling ≈ drag). Robustness: needs true latency ≲130 ms and
  compensation ≥ true (under-compensation by 30 ms is fatal; over-compensation degrades
  safely to a partial-tilt cruise).
* **HIL (bag so3c, tf=20): the transition completed and the vehicle flew a ~22 s / 200 m
  STABLE 90° pure-puller cruise** — tilt servo 1.00, rear pair ~1 N, front ~4 N ≈ drag,
  altitude flat 8.1–8.4 m, V 11.9 steady, dead-straight — ended by a slow roll/yaw
  departure (roll 12→91° over ~6 s) = the documented lightly-damped lateral mode at full
  unload. Solve times 23–42 ms in cruise; spikes to 150 ms only after the departure began.
* HIL corridor robustness is run-dependent: tf=20 crossed it; tf=30 lingered in it and
  crashed (bags so3b/so3d show the mechanism: slow nose-down sag through tilt 20–45°,
  elevator saturating, then a violent recovery).

## Remaining gaps (ranked)
1. **Lateral (roll/yaw) damping at full unload beyond ~25 s** — next levers: raise the
   planned rate damping `w_om` for roll specifically (split the weight per-axis), raise
   `da_gain/dr_gain` to 1.0 (was 0.6; params live-tunable), or add a small aileron/rudder
   rate-feedback outside the solver (the "damped M-routed surfaces" recipe).
2. **Transition-corridor run-to-run variance in HIL** — the offline corridor is clean, HIL
   sags nose-down; likely residual latency/actuator-path mismatch. Levers: measured-latency
   calibration (compare commanded vs realized tilt slew in a bag), latency_s sweep 0.10–0.14.
3. Activation reliability: one-shot `ros2 topic pub` misses discovery — the test driver
   retries and verifies the ENGAGED log line.

Ops: window 6e (`6e-so3`), knobs `SO3_VC/SO3_BC/SO3_TF/SO3_LAT` (forced to DOUBLE in the
launcher — rclpy rejects int for double params). Driver `/tmp/run_so3_final.sh`; bags
so3a–so3d in `/tmp/bags/`. EVERY sim launch records a bag and the bag is read after each
test (analysis via `/tmp/anabag.py`).

---

# PHYSICS AUDIT (user-requested): pitch, rotor share, and the true trim speed

Sim-exact coefficients (`vtol_aerodynamics.py`): W=34.34 N, S=0.35, CL=0.15+4.8α, CLmax=1.2.

## Trim table (level flight, L=W)
| V m/s | CL_req | α | note |
|---|---|---|---|
| ≤11.55 | >1.2 | — | IMPOSSIBLE — V_stall=11.55; rotors must assist |
| 11.9 | 1.13 | 11.7° | near stall — **the so3c cruise speed** |
| 13.8 | 0.84 | 8.3° | offline SO(3) cruise point |
| 15.0 | 0.71 | 6.7° | profile design point |
| **18.2** | **0.485** | **4.0°** | **the α≈4° cruise the user asked for; drag 3.5 N** |

## Rotor share during cruise (bag-measured, actuator channels → N)
* **F/so3c (t=20–42): tilt 88.6°, front pair 5.9 N (5.86 N FORWARD), rear pair 3.2 N vertical →
  rotor vertical = 3.3 N = 9.7% of weight → THE WING CARRIED 90.3%.** The rear rotors were NOT
  holding it up — it was genuinely wing-borne.
* D/fwD10 (t=20–33): tilt 33°, rotor vertical 25.1 N = 73% → D's cruise as flown in that bag was
  mostly rotor-borne (wing 27%).

## Why the extreme nose-up pitch while climbing (controller F) — the user is right to distrust it
The so3c cruise ran at **11.9 m/s = 3% above stall speed**, where the level trim alone needs
α≈11.7° (CL 1.13 of 1.2). During phugoid climb phases body pitch = α + γ went far beyond — and
the SIM HAS NO POST-STALL LIFT BREAK: `alpha_eff` is clamped at ±22° and CL stays pinned at
CLmax for ANY deeper angle (at α=40°, V=12 the wing still gives 28 N of vertical lift). A real
wing stalls sharply there. So the "impossibly nose-high yet climbing" flight is the sim mushing
on the CL clamp — and every controller's internal model shares the clamp, so optimizers can
exploit deep-stall flight the real aircraft would never survive. **Conclusion: the so3c cruise
was real wing-borne flight, but at an unrealistically forgiving angle of attack.**

## What honest α≈4° cruise requires
* Cruise speed 18.2 m/s (front-pair thrust ≈ 3.5 N — trivially available).
* The current SO(3) corridor tuning is Vc=15-specific: raising Vc re-solves the whole trim
  schedule and the transition corridor CRASHES offline at Vc=16.5 and 18.5 (solver-failure
  cascade at tilt 35–48°). Getting to 18.2 needs a corridor retuning pass (pace ds, w_beta/w_vel,
  possibly horizon) — documented as the next iteration.
* RECOMMENDED sim fix: add a post-stall CL decay to `vtol_aerodynamics.py` (e.g. linear break
  to ~0.6·CLmax by α=30°) so no optimizer can exploit the clamp. NOTE this invalidates existing
  tunes that ride high α (so3c-style cruise would genuinely stall — as it should).

---

# HONEST PHYSICS UPDATE: post-stall break added, α≈4° cruise achieved offline

Per the physics audit, the stall-clamp exploit is CLOSED:
* **Sim** (`vtol_aerodynamics.py`): CL now breaks past stall (×`post_stall_cl_frac`=0.6 ramped
  from 22°→30°) with flat-plate drag growth (`cd_post_stall`=0.5). Deep-stall mushing no longer
  lifts like cruise.
* **All controller models mirror it** (SO(3) `aero_body`, D's `_f`, E's `aero_world`). CRITICAL
  LESSON: the model version uses a **smooth tanh blend** (mid 26°, width 3°) — the piecewise
  fmax/fabs version put a non-smooth kink at exactly the α the corridor rides and broke IPOPT
  (deterministic solver-failure cascades at tilt 35–45°). Solver cache tag "m3"; a physical
  sanity guard rejects "optimal" solutions with near-zero thrust below stall speed.

## Results under honest physics (offline, latency-emulated, JIT ~35 ms solves)
* **Vc=18.2, tf=24, α_max=6, leash=0.08: FULL 90° transition + steady cruise at V=16.7,
  AoA=+4.4°, z dead-flat, rotors 6.4 N total** — the α≈4° pure-puller cruise the audit called
  for, with no stall crutch available. Two structural additions made it work:
  (1) **speed-anchor**: achieved airspeed also advances the path anchor (bypasses the leash —
  "speed is reality"); without it the fast vehicle under a lagging tilt made excess lift →
  runaway climb. (2) MAX_ITER=40 suffices with the smooth break (35 fails, 60 needed with the
  kink) → worst-case solve ~100 ms.
* Vc=15/tf=20 regression: still completes (V=13.8, α=+7.2°) — the old corridor failures under
  honest physics were the numerical kink, not the physics.

## Remaining HIL gap (bags so3e/so3f): negative-α basin capture
In HIL the early corridor transient tips the solver into a SECOND valid-per-model trim basin:
nose-down α≈−40° (broken negative lift), rotors carrying, elevator saturated nose-down BY THE
SOLVER'S CHOICE — locally optimal from a nose-down state because climbing back up through the
broken-lift zone looks expensive within the 1.2 s horizon. Altitude holds for a while (rotor
muscle), then departs. Offline never sees it (clean start stays in the +α basin).
**The concrete fix (next iteration): a hard path constraint α ≥ −10° during the transition**
(nonlinear inequality on the model AoA — kills the wrong basin outright), plus an entry gate on
pitch. Secondary: solve-time telemetry is in the log (`sv=`); corridor solves are ~35 ms at
MAX_ITER 40 after the smoothing.

Status: honest-physics 90° @ α≈4° is DONE offline and blocked in HIL only by the basin capture,
with a specific, mechanical fix identified. Bags: so3e, so3f.

---

# OVERNIGHT SESSION (2026-08-10→11): honest α≈4° cruise — offline SOLVED, HIL NOT reliably achieved

Goal: reproducible honest-physics α≈4° wing-borne 90° cruise in HIL. Outcome: **works perfectly
OFFLINE; not reliably in HIL** after ~12 instrumented flights (so3i…so3s, cascFW1). Documenting
straight so the next session doesn't repeat the dead ends.

## Real bugs found + fixed (these are solid, keep them)
1. **Boot SIGBUS**: the 32 MB JIT solver cache mmaps during Isaac's load and SIGBUS-crashed under
   memory pressure. Fixed: build the SO(3) node at `D_SO3=105 s` (after Isaac settles) + a 5×
   retry loop in the launcher window.
2. **GS cleanup**: removed "Auto-fit to geofence" (it silently clamped the spatial scale to 1000×
   on every file load — actively harmful), the dead geofence stub, and "Loop" (reference jump).
3. **Honest physics**: post-stall CL break (→0.6·CLmax by 30° + flat-plate drag) added to the sim
   AND all three controller models (smooth-tanh in the models — a piecewise kink broke IPOPT).
   Closes the deep-stall "mush and climb" exploit.
4. **★ TEST-HARNESS BUG that invalidated ~5 flights**: `ros2 topic echo --once` from ephemeral
   shells routinely returns EMPTY (DDS discovery > the --once timeout). An empty read made the
   altitude `awk` exit 0, so "reached 10 m" passed while the drone was still at 1.5 m → the
   transition kept starting at GROUND LEVEL → "controller failures" that were really ground
   contact. **Lesson: read vehicle state from the PERSISTENT node logs (they print p=(x,y,z)),
   never from spawned `ros2 topic echo` calls.**

## SO(3) in-loop transition (nmpc_so3_transition) — HIL findings
- Offline (exact model + emulated latency), honest physics: **FULL 90° transition + steady cruise
  V=16.7, AoA +4.4°, tilt 90, altitude flat, robust to 0.12–0.15 s latency.** Solid.
- HIL: marginal. Best runs (so3j; so3r with the front-cap) reached **full 90° tilt at 18–21 m/s**,
  but altitude control through the transition is poor (sinks 4 m then zoom-climbs) and the cruise
  **phugoids and diverges**; the IPOPT solve also FAILS (ok=0) at tilt-90/high-AoA and the vehicle
  coasts on last-good commands. Output-stage aids that HELPED: airspeed **tilt-clamp** (don't tilt
  past the trim tilt for the current speed → no front-pair saturation), **rear-rotor floor**
  (keep roll/yaw authority), **front-motor headroom cap** (bag so3r — got it to full-tilt cruise
  speed). Aids that did NOT fix it: NLP AoA ceiling (starves the solver), warm-start-from-last-good,
  surface rate-SAS, slower tf. Net: the in-loop NMPC cannot robustly stabilise the fast
  transition/phugoid through the HIL latency — a bandwidth limit, not a tuning miss.

## Cascade (nmpc_so3_cascade) — the pivot
- **Multicopter fly-to-point+LAND is the ONE reliable HIL capability** (this session: flew to a
  target and LANDED at 0.12 m). Default is now this (`fixed_wing=False`).
- The **fixed-wing branch is broken** (bag cascFW1: flew BACKWARD to (−177,−44) and crashed —
  a direction/sign bug in the FW mission/branch, on top of the earlier engage-transient issues).
  Front-cap + SAS added but the branch needs a real debug of its heading/reference handling.

## Where it stands / next session
- Deliverable that WORKS in HIL: cascade **multicopter** fly-to-point + precise landing.
- Honest α≈4° 90° cruise: **offline only.** The concrete open problems are (a) HIL transition
  altitude control (sink→zoom) and (b) the cruise phugoid + cruise-time solve failures. The
  cascade FW branch's backward-flight bug is the most tractable next lead (its inner loop is the
  robust geometric one — fix the heading/reference sign and it may fly the honest cruise without
  the in-loop NMPC's phugoid problem).
- All bags in /tmp/bags/so3*, /tmp/bags/cascFW1. Read them with /tmp/anaflight.py (merges
  mocap+motors+servos+AoA from the raw .db3; needs no metadata.yaml).

## Cascade FW branch: one real bug fixed, one deeper bug open (2026-08-11)
- **FIXED (offline-proven): the tilt was keyed on MEASURED speed** — a chicken-and-egg (no tilt
  → no forward thrust → no speed → tilt stays 0; the vehicle just climbed). Now keyed on the
  COMMANDED (reference) speed, bounded to measured+6. Offline closed-loop probe (/tmp/fw_loop.py):
  the vehicle tilts 0→65°, accelerates to ~21 m/s, **settles at AoA +2–4° (the honest trim),
  wing-borne (front rotors unload to 0.05), altitude held**. This is the correct behaviour.
- **OPEN (HIL): the attitude loop is unstable through the HIL latency.** With the tilt fix, HIL
  bag cascFW2 still flew BACKWARD and climbed away (−391 m, +171 m, constant ±18° roll) — the
  vehicle does the OPPOSITE of the (correct, +x) reference. The point-mass probe can't see this
  because it assumes pitch tracks the command; the real coupled roll/pitch/yaw attitude dynamics
  diverge. Same latency/bandwidth wall as the in-loop SO(3) node.
- **Debugging tools built:** /tmp/fw_probe.py (static FW-branch I/O), /tmp/fw_loop.py (closed-loop
  point-mass) — both find control-logic bugs deterministically with NO sim, in seconds.
- **Net:** honest α≈4° cruise is now correct in the point-mass model AND the full offline sim, but
  the HIL attitude instability is the real unsolved blocker. Reliable HIL capability stays the
  cascade MULTICOPTER fly-to-point + precise landing.
