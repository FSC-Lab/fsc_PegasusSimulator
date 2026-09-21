# ★★★ BRANCH `slosh` — FREE-SURFACE TANK, PHASE 1 DONE (2026-09-12)
# Plan: https://claude.ai/code/artifact/7d8de8f4-dde5-4efa-b16e-fc97fb217d78
# This section exists only on the `slosh` branch. `pre-slosh` is the baseline.

# ★★★★ 2026-09-21 (night): AN INDI INNER LOOP, AS A VARIANT OF THE FLOWN NODE -- BUILT, GATED, NOT FLOWN.
# THE ANSWER TO "WOULD INDI LET THE MODEL BE LESS ACCURATE": NOT IN THIS LOOP -- THE FLOWN PITCH AND YAW LAWS ARE
# BUILT ON THE AIRFRAME'S OWN AERO, AND INDI INVERTS IT AWAY. Controller 401f4a8 (`nmpc_cascade_indi_node.py`),
# sim 21831f2 (boot test `--node indi`, `scripts/indi_ab.sh`). The flown node is byte-identical (identity holds).
**THE QUESTION (the user's).** Instead of the geometric inner loop, an incremental nonlinear dynamic inversion
(INDI) inner loop based on the model in use, so the model itself need not be as accurate. Asked as a design
question first, then "copy the node and replace the geometric section's inner loop with the INDI".
**THE DERIVATION, for this loop (the node's docstring carries it in full).** The rotational plant every controller
here writes: `J w_dot = M_c(x,u) + M_a(x) + M_d - w x J w` (1) -- control moment, aero moment of the state,
unmodelled, gyroscopic. First order about the current state and the control the actuators are APPLYING now
(u0, not the last command): `w_dot ~ w_dot_0 + (dw_dot/dx)(x - x0) + (dw_dot/du)(u - u0)` (2); time-scale
separation drops the state term: `w_dot ~ w_dot_0 + G (u - u0)`, `G = J^-1 dM_c/du` (3); demand `w_dot = nu`
and solve for the INCREMENT: `u = u0 + G^+ (nu - w_dot_0)` (4). `w_dot_0` is MEASURED, so M_a, M_d and the
gyroscopics are inside it: the only model left is the control effectiveness G (an error `G_true G_est^+` in
(0, 2) per axis converges). At the moment level, where this cascade allocates: `M_des = M_c(u0) + J (nu - w_dot_0)`
(5), and with `nu = M_geo / J` (the flown geometric law's moment, every gain and fade as it is):
**`M_des = M_geo + [M_c(u0) - J w_dot_0]`** (6) -- the geometric law plus the algebraic estimate of everything
that is NOT the control moment, sign flipped. That is the L1's job description (`I w_dot = M_model + sigma`,
`u_ad = -C(s) sigma_hat`) with three differences: the residual is MEASURED, not predicted-and-adapted; it comes
out at the rate filter's bandwidth (40 rad/s) instead of C(s)'s 2 rad/s; and **M_a is not in the model, so the
aero stiffness and damping are cancelled with the rest.** Synchronisation is what INDI lives by: `w_dot_0` and
`M_c(u0)` must describe the same interval -- the rate difference through two first-order lags at wf, and the
commanded thrusts/deflections through the L1's transport-delay FIFO + actuator-lag states through THE SAME two
lags (a mis-declared delay is the IMU's 40 ms failure class).
**THE VARIANT.** `L1Indi(imu.L1Imu)`: the force twin, EKF predictor, wall guard, status feed and log inherited;
`adapt()` returns on the INDI axes the bracket of (6) (+ the predictor's model-based direct term: INDI reacts,
the predictor anticipates) in place of C(s)'s u; re-points `base.L1Aug` like the STESO variant, `base.STATUS_EXTRA`
adds `indi axes= wdot= sig_indi=`. Knobs `--indi_axes ry` (default: roll + yaw), `--indi_wf 40 / --indi_wf_yaw 10`,
`--indi_kw 1.68,0,0` (below), `--indi_yaw_fade 1` (below), `--indi_umax 6,6,3` (never bound), `--indi_hold_fade_s 0.5`
(the increment fades out through the back-transition instead of holding a 40 rad/s snapshot), `--indi_off` =
the IMU node exactly. Boot test `tests/test_cascade_node_boots.py --node indi` (its banner + the IMU node's + the
ack). `scripts/indi_ab.sh hover|mission|model|turns|final` reproduces the table below. NOT wired to a launcher
(`ADAPTIVE_NODE_PY` would do it); not flown.
**THE GATES (`~/flight_bags/indi_ab` / the scratch runs, the flown IMU node as the A, the flown line, same seeds):**
| case | flown node | INDI (roll + faded yaw) |
|---|---|---|
| identity `--indi_off` | `460eb532b42d5443e66edd6218233957` | the same hash |
| hovers: still / CG 5 cm / lat 50 ms / rate noise 3 deg/s (worst displacement, m) | 1.22 / 1.50 / 1.25 / 1.25 | 1.22 / 1.51 / 1.26 / 1.25 |
| CG 5 cm hover standing pitch | -1.14 deg | -1.14 (pitch INDI too: -1.15 -- the OUTER's crutch, not the inner's) |
| still hover yaw diagonal sd | 0.063 | 0.071 (rate noise through wf_yaw) |
| **hover, 5 m/s quartering wind (the hold leaning)** | **1.22 m, roll 8.4** | **1.12 m, roll 5.3** (0.92 at kw 2.0) -- the one win |
| hover, 6 m/s crosswind | 9.6 m (the lean cap) | 10.4 |
| straight 90 s: nominal / lat 50 / lat 80 ms / spin 0.18 / cQ 0.5x / 2x / inertia 0.6 / lat aero 1.5 | 0/1801 each | 0/1801 each, indistinguishable |
| straight, control derivatives 0.6 | ground t 24.9 (corridor) | ground t 36.9 (the same corridor; pitch authority) |
| **straight back-transition at 60 s** | peak +12.0, z min 29.9, V<0.5 at 71.0 | **identical to the tenth** |
| turning mission (100 mm tank dx 0.3), 3 seeds + cross5 | 3/3 + 1/1 | 3/3 + 1/1 |
| seed 2: orbit roll / fly-by peak bank vs the 25 deg ref / RTH stop | -18.2 sd 0.11 / **24.4** / 5.8 m | -18.2 sd 0.14 / **28.1** / 8.9 m (the RTH geometry: the orbit's phase at the rth and the roll guard's window; the fixed-geometry stop above is identical) |
| yaw INDI UNFADED | -- | **crashed seeds 2, 3, cross5, diag5**: stalled out of the first fly-by turn at AoA 22 |
| pitch INDI (`--indi_axes rpy`) | -- | **crashes the corridor at t 9.7** (peak +76.6; +51.7 at lat 50), straight and turning |
| `--indi_kw` 1.0 / 2.0 / 3.5 (peak bank, orbit sd) | -- | 27.7 / 28.1 / 29.5, sd 0.11 / 0.13 / **1.64** |
| `--indi_wf` 20 / 80 (peak bank) | -- | 30.2 / 28.7 -- the filter is not the overshoot's cause |
**THE READING.** (1) **Yaw at speed: the flown law is safe BECAUSE it is weak.** It is damping only (`-kW_yaw r`)
and the fin flies the coordinated turn; INDI made that law exact -- it fought the turn's yaw rate with the rudder
and the rotors, the sideslip grew, the wing stalled out of the turn. So the yaw increment fades with the wing
share, the same fade the heading law uses: INDI owns yaw where the rotors do (the hover), the fin owns it at
speed. (2) **Pitch: the outer plans with the stiffness INDI removes** -- build_ode6's cma is in the outer's model,
and an inner loop that inverts it hands the outer a plant it did not model; the corridor crash at t 9.7 is the
"two loops doing pitch" bang-bang of 2026-09-10 in a new coat. Measured, not a caveat. (3) **Roll, the one axis
where the inversion is harmless, is a wash with a cost:** the airframe's roll subsidence `Lp = -Clp qbar S b^2/2V`
= 1.68 N.m s at 16 m/s is MORE than the loop's own kW 1.2, and INDI inverts it away -- the double-integrator plant
it leaves has zeta 1.0 at the flown gains where the airframe made it 1.7, and through 55 ms of transport + filter
lag the bank overshoots the 25 deg reference by 3-4 deg. `indi_kw` (default = Lp at cruise) puts the damping
back; the remainder is the delay, and an INDI loop wants its virtual-control gains DESIGNED for the inverted
plant -- a retune, its own piece if INDI is ever pursued. (4) **The 5 cm CG hover's -1.14 deg is not the inner
loop's to fix** (pitch INDI leaves it at -1.15): the outer's package assumes a balanced airframe and parks the
tilt as a crutch -- the "honest split" of the STESO block (sigma_bias into the outer as a parameter) is where
that lives. (5) **INDI needs the actuator model MORE, not less:** the effectiveness G (B_rot, the surface
derivatives), the transport delay and the lags must be right for the increment to be synchronous; the model
INDI lets you drop is the AERO's, and this loop's outer and its yaw law are built on that aero. The question's
premise -- "so the model does not have to be as accurate" -- holds for a multicopter whose inner loop is the
whole controller; here the model lives in the outer NMPC, which INDI never touches.
**STANDING (only if INDI is pursued):** the roll retune for the inverted plant; a flight of the variant on the
hover stage (the leaning-wind hover is the place it showed anything); yaw INDI in hover under the measured cQ
against the fin's weathercock (the hover diagonal +13% says the noise gain is real).

# ★★★★ 2026-09-21 (evening): THE MEASURED BUILD'S FIRST HIL FLIGHT -- A 30 s HOVER STAGE, LANDED 0.00 m.
# Row R100_dx3_mn4014_hover30_a1_155915, bag slosh_20260921_155735 (`STAGE=hover STAGE_HOLD=30 CASC_SPIN=poll`,
# the 1b configuration: cascade_adaptive / imu-mdob + EKF + L1 / px4 state / GPS 10 Hz / pitot, the offset
# 100 mm tank, af=21fad0371f = the measured cfg). Sim this commit, controller 1990e91.
**WHAT IT DID.** Climb on the safe controller (its yaw-last mixer + weathervane in the loop for the first time,
still air: nothing to release), the engage hover (the along-track-only hold now), the 30 s hover stage: **"go"**
-- the hold sat 0.6-1.4 m ahead of its point (the outer's vs bias, as on 2026-09-16) at V 0.15-0.4 m/s, z
29.7-31.1, pitch -1..-2, roll +-1, yaw 0, tilt -1..-3.5, rotors 36-38 N, **0/693 solve failures, outer
11-12 ms**; then the schedule's back-transition from the hover (+14 / -4 deg of pitch, 3.6 m, V 2.45 -- the
2026-09-16 shape), the recover, the landing at (0.81, 0.08): 0.00 m from where it was. The measured cfg's
digest is in the row; the launcher's four gates (scaffold, import, prop pairing, tilt pairing) passed; no JIT
rebuild was needed (the cache key had not moved).
**THE HOVER YAW LOOP ON THE REAL cQ, IN HIL** (`bag_axis_fit.py`, which now reads the airframe FILE -- it had
carried its own cQ 0.12 / 18 N / CAD arms and read this hover at half the torque, the plant-double trap
again): yaw diagonal sd **0.046 at 0.9 Hz** (quiet < 0.02; the 2026-09-11 thrash 0.4-0.8) -- the diagonal is
4.4x the old throttle per N.m and the offset tank's ~1 Hz fore-aft ring is in it; the yaw held 0 throughout.
And **the plant's yaw response is 1.5x the model's** (I_eff 0.32 vs the assembled 0.49 at the file's cQ; the
old sim-only cQ read the same bag at x1.5 the OTHER way): with cQ down 6x, the rotors' SPIN-UP reaction
(prop inertia x omega_dot; the sim's 1e-4 kg m^2 per prop) is no longer small beside the aerodynamic cQ.
**Hardware item:** the rotor + prop inertia of the MN_4014 + 15x5 sets that transient yaw and is in the
thrust stand's own torque trace on a throttle step (I = torque transient / omega_dot) -- fit it from the
2026-09-17 logs before the first hover on the airframe.
**THE PIECE-7 REHEARSAL ON THIS BUILD** (`ekf_retune.py` on the bag): hover window 34 s, bob 9.0 / 3.2 mm rms,
bias states bF (+0.07, -0.00) N (2026-09-16: +0.17), bM (+0.01, +0.02, +0.00) N.m, force floor 0.074 N vs
the gate's 0.039, moment floors (0.009, 0.012, 0.022) vs (0.049, 0.063, 0.096) -- the 2026-09-16 numbers to
the second decimal; proposal r_F 0.57 / r_M 0.05, not applied (the tool is for the airframe's floors).
**NOT FLOWN YET on this build:** the turning mission (the gate says 3/3, the 09-17 decimals) and any wind
(HIL has none); the safe controller's weathervane has flown only its climb and landing in still air.

# ★★★ 2026-09-21 (evening): B2 -- THE TILT SERVO AS A MISMATCH. THE GATE'S PLANT HAS ITS OWN SERVO NOW, AND THE
# SPECIFIED ACTUATOR HAS 4-5x OF MARGIN. Controller 1990e91, sim this commit; sweep `~/flight_bags/robust/tilt1`.
**WHAT WAS BUILT.** `so3.build_ode(tilt_tau=, tilt_rate=)` gives the gate's PLANT its own tanh slew-limited servo
(None = the file's, byte-identical); the gate's `--plant_tilt_tau S` / `--plant_tilt_rate DEG_S` set it while the
controller keeps the file's (its applied-tilt estimate is the pure slew at `--tilt_slew` = the node's
`tilt_slew_deg_s`, and the L1's applied moment reads that estimate); `--tilt_slew X` is the "told" knob (only the
rate has a controller-side counterpart -- the node models no lag). `test_plant_knobs` has the tilt knob.
**AND THE GATE'S MODEL OF THE NODE'S OWN SLEW WAS A HARD-CODED 80 deg/s** while the node has read the file's 106
since 2026-09-17: the gate's controller was 25% slower than the flown one on the tilt estimate. Fixed -> the
identity references move (deliberately): **steso `b501024381f70fc1b3d546479031bcb1`, IMU
`460eb532b42d5443e66edd6218233957`** (md5sum of the --log CSV); `--tilt_slew 80` reproduces the previous
`22a5a153` / `0ebd9753` exactly, so nothing else moved.
**THE TABLE (`scripts/cells_tilt_servo.txt`, the flown law, 16 seeds; the spec is tau 0.0756 s / 106 deg/s):**
| plant servo | landed | corridor pk | fails BT / in flight | stop | verdict |
|---|---|---|---|---|---|
| nominal (as specified) | 16/16 | 13.9 | 7 / 0 | 8.8 | PASS |
| tau 0.10 / 0.15 / 0.25 / 0.40 s (1.3-5.3x) | 16/16 | 13.9 / 13.9 / 13.8 / 13.8 | 0 / 0 / 1 / 1 in flight | 8.8 | PASS, all |
| rate 80 / 60 / 40 deg/s, controller told 106 | 16/16 | 13.9 / 13.9 / 13.9 | 0 / 0 / 0 | 8.8 | PASS |
| rate 25 deg/s (a quarter), told 106 | 16/16 | 15.4 | 4 in flight | 8.8 | PASS |
| rate 60 / 40 / 25, controller TOLD | 16/16 | 13.9 / 13.8 / **17.0** | 0 / 0 / 3 | 8.8 | PASS / PASS / FAIL on the peak |
| tau 0.15 + rate 60 (a derated servo) | 16/16 | 13.8 | 0 | 8.8 | PASS |
**READING.** The tilt servo is not a sensitivity of this loop: five times the specified lag and a quarter of the
specified slew both fly the mission at the nominal corridor peak, and only 25 deg/s TOLD to the controller
costs 3 deg (its own applied-tilt estimate then lags as much as the plant and the two errors add -- knowing a
slow servo is worse than not knowing, the opposite of the spin lag). The schedule's tilt reference is
rate-limited at 35 deg/s (`tilt_slew_ref_deg_s`) and the corridor asks ~25 deg/s of the servo, so a 106 deg/s
actuator sits 4x above what is ever demanded; the spin lag (cliff at 0.18 s) remains the only actuator constant
with a hard requirement. The hinge-moment question (the 36 N prop's thrust-line offset) is therefore a
"does it move at all under load" question, not a bandwidth one: measure the offset, size the servo's torque
with margin, and the rate follows.

# ★★★★ 2026-09-21 (late afternoon): THE WIND TABLE ON THE MEASURED BUILD -- AND THE HOVER HOLD'S LEAN IN A
# QUARTERING WIND AT THE ENGAGE, A REGRESSION OLDER THAN TODAY, FOUND AND FIXED. Controller 2ab28bf, sim this
# commit; sweeps `~/flight_bags/wind/wind3` (the first pass) and `wind4` (the five cells re-run after the fix).
**THE TABLE (`wind_sweep.sh`, 16 seeds per cell, imu-mdob + pitot + L1, the turning mission, the offset 100 mm
tank; wind1/wind2 = the 12x8 build of 2026-09-15/16, BEFORE the hover hold existed):**
| cell | wind1/2 (12x8, no hold) | wind3 (measured build + hold + weathervane, 660dd83) | wind4 (after 2ab28bf) |
|---|---|---|---|
| still | 16/16, fails 16, pk 11.6, stop 6.3 | 16/16, fails 107, pk 11.5, stop 6.1 | 16/16, fails 121, 11.5, 6.0 |
| head3 | 16/16, 0, 11.7, 3.3 | 16/16, 1, 11.5, 9.0 | -- |
| head5 | 16/16, 128, 11.7, 22-24 | 16/16, 41, 11.3, 23.4 | 16/16, 50, 11.3, 23.4 |
| cross3 | 16/16, 0, 11.9, 4.5 | 16/16, 1, 12.3, 4.7 | -- |
| cross5 | 16/16, 0, 12.5, 9.9 | 16/16, 3, 12.7, 4.9 | 16/16, 1, 12.1, 10.0 |
| crossm5 | 16/16, 176, 12.5, 20 | 16/16, 120, 12.7, 22.3 | 16/16, 89, 12.0, 21.1 |
| **diag5 (-3.5,+3.5)** | 16/16, 0, 12.3, 4.2 | **0/16 -- all in the corridor at t 8-9 s** | **16/16, 2, 12.0, 9.8** |
| turb 0.5 | 15/16 | 15/16, pk 88.6 (the one stall) | -- |
| turb 1.0 | 2/16 | **5/16** | -- |
| head5 + turb 0.5 | 11/16 | **15/16** | -- |
The measured build flies the steady winds as the 12x8 did (V_air 16.0 +- 0.1 in every cell, front throttle
0.70-0.71 against the 12x8's 0.83, the estimate within 0.5 m/s) and is a little better in turbulence (more
thrust: 1.0 m/s 2 -> 5 of 16, headwind + 0.5 11 -> 15). The "fails" column is the post-stop hover's solve
again (still air: 7 per seed, every one at t_stop + 0.5..3 s, V 0.1 -- verified on seed 1 at every tick),
higher on the measured build than on the 12x8 (1 per seed): the standing solver nuisance, now named twice.
**THE diag5 LOSS WAS NOT THE WEATHERVANE.** A/B on seed 1: the weathervane off -> still lost; the OLD airframe
file (12x8) -> still lost; `--hold_k 0` -> flies; `--hold_roll_max 0` -> flies; `--hold_roll_max 4` -> flies; the
L1 off -> flies; a longer engage hover (`--settle` 4 / 6 / 10, a new gate flag) -> still lost 6-7 s after the
transition starts. The chain: the hover hold (2026-09-16) LEANS 8 deg into the wind at the engage; the pitot
wind estimator puts the whole wind ON THE NOSE (its small-sideslip assumption -- in a hover in a 45 deg
quartering wind the sideslip is 45 deg), so the model's v_air has no sideslip while the plant's does; the L1
learns the sideslip aero the model lacks (u_ad 0.5-0.6 N.m on EVERY axis in the hover) and cancels it; when
the tilt swings 8 -> 39 deg at the transition the real moments change faster than C(s) follows, the outer's
solve fails at V 1.5 (ok=0, 23-28 ms), the fallback freezes the tilt and the nose runs +14 -> +29 -> +50 ->
+89 in 0.7 s. The pre-hold controller drifted through the same hover unbanked and flew it 16/16; a pure
crosswind (0,3.5) or a pure headwind (-3.5,0) flies with the lean -- only the quartering wind's 45 deg of
sideslip + the lean + the L1 does it. **Never gated since the hold landed: the wind cells with a
cross component were last run on 2026-09-16 morning, before it.**
**THE FIX (2ab28bf): the hold LEANS (and weathervanes) only in a hover STAGE -- a `v_cap 0` stage or the RTH
stop -- and is along-track only in the ENGAGE hover**, which is what every flight before 2026-09-16 did there
(`lean_ok` / `wv_ok` in `HoverHold.update`; the gate mirrors it with `hover_only or _stopped`). diag5 16/16
(seeds 1-4 by hand 0/5201 each; the cell 16/16 at pk 12.0), the still-air straight regression 16/16 pk 13.9 /
flight fails 0, the turning mission 3/3 to the 2026-09-17 decimals. `hold_wv_s` 0.5 -> 1.5: the 2 m/s^2
gust hover released its heading at 0.5 and 1.0 s of transient yaw demand and not at 1.5 (position 3.30 m, as
on 2026-09-16); the 6 m/s crosswind release moves 0.6 -> 1.6 s (swing 6.6 -> 11.4 m, then held), 4 m/s 2.8 -> 4.2.
**IDENTITY MOVED, DELIBERATELY:** the engage hover's roll_hover under noise is gone from the flown lines --
**steso `22a5a15381bdcb77d3fcca7d68c38fd4`, IMU `0ebd9753611143df22a618f449377520`** (md5sum of the --log CSV;
first differing row t=0.115 against 3cf1bdfc / ba102167).
**WHAT THIS SAYS ABOUT THE ESTIMATOR (open, the real item):** in a hover the pitot cannot tell a headwind from
a quartering wind, and the estimator's assumption makes the model WRONG BY THE SIDESLIP ANGLE -- worse than
blind (no pitot: 30 solve failures but flies). The flight card's rule "transition INTO the wind" (the 180 deg
hover yaw check reads the direction) is what makes the engage hover a headwind case; a hover heading that
is not into the wind should not be assumed to know its wind. Candidates, not built: freeze the estimate's
DIRECTION below a ground speed and keep the magnitude; or hold the L1 (as the back-transition does) through
the first seconds of a transition started in wind. `--settle` is on the gate for the next look.

# ★★★★★ 2026-09-21 (afternoon): THE YAW POLICY ON THE MEASURED cQ -- YAW SERVED LAST AND A WEATHERVANE, IN
# BOTH CONTROLLERS. Controller e2432f1 (safe controller) + 660dd83 (cascade hold), sim 824d207. The
# 2026-09-21 morning block below is the finding this answers. NEW IDENTITY REFERENCES (procedure below).
**THE SAFE CONTROLLER (`geometric_hover_node.py`, e2432f1).** (1) **Yaw is allocated LAST**: `geometric_command`
solves the collective, roll and pitch first (`T0 = Ainv @ [f, Mx, My, 0]`), then clips the yaw moment to
`yaw_headroom` (0.7) of the differential that keeps every rotor inside [0, kf] on top of them, and reports
`yaw_clamped`. At cQ 0.0190 one N.m of yaw is 13 N of differential per rotor, so an unclipped heading hold in a
crosswind ran the rotors to their rails and the "attitude-priority" collective shift then took the LIFT. (2) **A
weathervane** (`Weathervane`, params `wv_yield_s` 0.5, 0 = never): once the yaw moment has sat on its clip for
0.5 s the yaw reference is RELEASED to the current heading and follows it (damping only) until the operator
gives a new yaw (`arm`). Plant double, hold at (0,0,12), 40 s:
| crosswind | shipped before (heading hold) | now: 3 m/s | 4.5 | 6 | 8 | 6, clip only (wv 0) | 6, headroom 1.0 |
|---|---|---|---|---|---|---|---|
| final / worst error | 0.00 to 4 m/s, **29-77 m flyaway from 4.5** | 0.00 / 0.97 m | 0.00 / 1.28 | **0.00 / 1.76** | 0.00 / 2.56 | 0.00 / 1.75 | 0.00 / 1.87 |
| nose | held with 12-19 deg of standing error | held, -11.8 | released 0.8 s, -90 | released 0.7 s, **-90** | released 0.6 s, -90 | -66 (the fin vs the clip) | -57 |
| mixer saturated | 91-99% of ticks | 0.0% | 0.0 | **0.0** | 0.0 | 0.0 | 0.2 |
| position integral | 13 N (pinned) | 8.1 | 0.6 | **1.2** | 2.6 | 8.4 | 11.5 |
The clip ALONE keeps the position (the mixer is safe by construction); the weathervane is what makes it clean
(nose-in the side-drag area collapses: 1.2 N of integral instead of 8-13). Every other section of
`test_geometric_hover` reproduces its numbers to the digit -- the clip never touches in still air; the latency
margin is still 125 ms; the node boots (`test_state_source` §3) and the pairing holds. §7 is re-armed on the
three rows (6 m/s nose free + mixer never saturating + released within 5 s; 3 m/s heading HELD; clip-only holds).
**THE CASCADE'S HOVER HOLD (`HoverHold`, 660dd83).** The hold kept its latched heading in a crosswind only by
giving up the position (the 8 deg roll clamp), and with the nose released the L1 held it across the wind anyway:
the fin's weathercock IS the yaw residual the L1 cancels (no pitot in a hover -> the model sees no fin). Now
`inner_step` publishes `INNER_DIAG["yaw_ratio"]` (the rotor yaw demand over the rotor yaw authority from the
headroom, i.e. |M_rot_z| / A_rot_z) and `HoverHold.update(yaw_ratio, dt)` releases the latched yaw after
`hold_wv_s` (0.5) of ratio > `hold_wv_ratio` (1.0): the hold's frame follows the nose, `l1.yaw_mask` zeroes and
un-learns the L1's yaw residual channel (the model-based slosh term stays), and the along-track channel (the
outer's vs through the tilt) holds the position as the headwind case always did. Gate, imu-mdob + L1, 30 s hover:
| hover in wind | still | headwind 6 | crosswind 3 | 4 | 6 | -6 (mirror) |
|---|---|---|---|---|---|---|
| before (2026-09-16 hold) | 1.22 m | 1.13 m | within 2.6 m | drifts 0.9 m/s | **drifts 2.8 m/s** | -- |
| now | 1.22, no release | 1.13, no release | 1.47 m, released 2.5 s, nose -79 | 2.82 m, 2.1 s, -84 | **6.6 m (the swing), 0.6 s, -89, then held** | 6.6 m, +88 |
`hold_wv_s` counts seconds of the SLOW loop (50 Hz in the node); the gate's hold block runs at the outer's rate,
so its dt is `OUTER_EVERY * DT` (it was DT: the release came 10x late). Turning mission, 3 seeds: 3/3, bob
0.049, hits 0, roll sd 0.154, u_ad 0.006, stop 5.9 -- the 2026-09-17 reference, 0 releases.
**IDENTITY, AND A CORRECTION TO THE PROCEDURE.** Both flown gate lines are byte-identical before and after
660dd83 (`git stash` A/B, `cmp` on the `--log` CSV). The CSV carries no `ms` column, so "md5 minus the ms columns"
is the plain `md5sum` of the file: **steso `3cf1bdfc13a9a8070b7defd72b0fe2de`, IMU `ba1021676f02b6c6853a3339a85b182e`**
(the same lines as ever). The 2026-09-17 values (`567a6879…` / `fdd734f4…`) do NOT reproduce on the unchanged
tree today and cannot have been this procedure on this build; from here the reference is the md5sum of the
CSV and the two numbers above. (`tests/test_plant_knobs.py`'s `trace_hash` -- the same minus a trailing
newline -- gives `daca89f2…` / `7931a728…` for the same files.)
**WHAT IT MEANS FOR HARDWARE:** in hover the nose goes where the wind puts it above ~4 m/s and both controllers
hold POSITION through it; a heading is guaranteed only inside the rotors' 0.74 N.m (a hover 180 deg yaw check in
wind, the flight card's, will read the release in the log: `WEATHERVANE` / `wv=1` on the status feed). The
landing is therefore into the wind by construction. `wv_yield_s 0` / `hold_wv_s 0` give the old behaviour.

# ★★★★★ 2026-09-21: THE FULL SUITE ON THE MEASURED BUILD -- THE REAL CASUALTY OF THE MEASURED cQ IS
# HEADING-HOLD IN A CROSSWIND (THE FIN OUT-TORQUES THE ROTORS ABOVE ~4 m/s), NOT THE CASCADE'S YAW GAINS;
# THE PLANT DOUBLE HAD KEPT cQ 0.12; THE ROBUSTNESS TABLE RE-SIZED TO WHAT IS NOW UNCERTAIN (23 x 16).
# Sim 85bce90 + 2ec6729, controller 334893f (the cfg of 2026-09-17 was UNCOMMITTED until today), GS e7866ac.
**WHAT WAS RUN.** Every headless test (34 pass, 2 live-stack tests skipped) -- the 2026-09-17 build had only
had its three pairing tests run. Three failed, and NONE of the three was the cascade:
1. **`test_geometric_hover` section 7 -- the SAFE controller's 6 m/s crosswind hold: 76 m flyaway. REAL, and
   the finding of the day.** Hover yaw authority at the measured cQ is 4 x 9.7 N x 0.0190 = **0.74 N.m** (it was
   4.65 at the sim-only 0.12). The plant's aero at 90 deg of sideslip makes **1.87 N.m of weathercock at 6 m/s**
   (0.47 at 3; roll 2.0 N.m from the fin's height): above ~4 m/s of pure crosswind the fin wins. The safe
   controller's heading hold (kR_yaw 2 / kW_yaw 0.5, P on yaw) holds to 4 m/s with 12-19 deg of standing error;
   at 4.5 m/s and up its yaw demand saturates the mixer 91-99% of the ticks, the "attitude-priority" collective
   shift corrupts the thrust, and it climbs and drifts away (29 / 71 / 77 m at 4.5 / 5 / 6). **Yaw DAMPING only --
   a weathervane, PX4's `WV_EN` idea -- holds 0.00 m at 6 m/s with the nose swung into the wind and the
   position integral at 1.3 N instead of 13** (nose-in, the airframe's side-drag area collapses). The test is
   left FAILING on purpose until the safe controller's yaw policy changes (its own piece, safety-relevant):
   mixer priority collective/roll/pitch FIRST and yaw with whatever headroom is left, plus a weathervane in
   hover. Bracketed with `scratchpad/wind7b,c.py`-style A/Bs on `tests/plant_double.py` (numbers above).
2. **`test_hover_hold` "OSCILLATES" -- WITHDRAWN; the 2026-09-17 record over-read the test.** Its verdict bar was
   0.05 of diagonal split in THROTTLE units, set at cQ 0.12 / 18 N; the same yaw loop on the new build needs
   4.4x the throttle per N.m (6.3x from cQ, 0.7x from the lower hover throttle), so the split read 0.064 while
   the yaw held to 0.039 deg and the torque chatter was **0.046 N.m -- vs 0.049 on the old build**. A 6 x 5
   grid (kR_yaw 2-5 x kW_yaw 1-2, at rest and from a 30 deg upset) is clean everywhere. The bar is now the
   yaw torque (0.16 N.m = exactly the old 0.05 at cQ 0.12, so the 2026-09-13 table's verdicts are unchanged;
   the throttle chatter is still printed). **The shipped 5.0 / 2.0 stay. Standing item (1) of 2026-09-17 is
   closed without a retune.**
3. **`test_plant_knobs`: its "neutral" `--plant_cq 0.12` was a literal**, now read from the airframe file.
And a fourth, silent one: **`tests/plant_double.py` had received the 36.3 N propeller on 2026-09-17 and KEPT
`ROTOR_MOMENT_RATIO 0.12`** (nothing paired it -- `test_prop_pairing` never looked at the double despite its
docstring). Paired to 0.0190 (+ the 106 deg/s tilt slew) and PINNED: `test_prop_pairing` now holds the
double's six constants to the airframe and the airframe's `rotor_moment_ratio` / `tilt_rate_deg_s` to the
controller's cfg, mutation-checked (flip the copy -> FAIL). Lesson, twice in one morning: a bar in actuator
units or a copy no test pairs is where the next identification lands as a false controller verdict.
**THE CASCADE'S HOVER HOLD IN A CROSSWIND (gate, `--hover_only --wind 0,W,0`, imu-mdob + L1, 30 s):** heading
holds at every W (yaw err <= 6 deg, the yaw diagonal at +-0.2 throttle), but the hold's `hold_roll_max_deg` 8
(5.4 N of side force) caps what it can hold: **W 3: within 2.6 m; 4: drifts at 0.9 m/s; 5: 1.9; 6: 2.8 m/s**
(headwind 6: within 1.1 m, roll 0.5). The 8 deg was sized for the tank's bob, never gated in wind. Nose-into-
wind (the same weathervane) or a lean sized to the wind are the options; the wind estimator is frozen below
2 m/s IAS, so a hover has no wind estimate to lean into -- the drift itself is the signal.
**THE ROBUSTNESS TABLE, RE-SIZED TO THE MEASURED BUILD'S OWN UNCERTAINTIES (`scripts/cells_mn4014.txt`,
`~/flight_bags/robust/mn1`, imu-mdob + L1 = the flown law, 16 seeds per cell, ~1 min per cell here; PASS =
piece 2's rule):** J0 0.42-0.71 (the analogue band), thrust +-20% (the stand stopped at 80%), cQ 0.5-2x, spin
0.10/0.13/0.16 (told 0.067), CG +-1.5 / +3 cm, inertia 0.8 / 1.25.
| cell | landed | corridor pk | fails BT-window / IN FLIGHT | stop | verdict |
|---|---|---|---|---|---|
| nominal | 16/16 | 13.9 | 6 / **0** | 8.8 | PASS |
| J0 0.42 / 0.47 (plant BELOW the model) | 16/16 | 12.8 / 11.1 | 7/1, 1/0 | 8.7 | PASS |
| J0 0.57 (+0.05) | 16/16 | 16.5 | 4/2 | 8.8 | PASS |
| **J0 0.62 / 0.71 (+0.10 / +0.19)** | 16/16 | **18.5 / 19.9** | 1/1 | 8.8 | **FAIL on the peak** |
| thrust 0.8 / 0.9 / 1.1 / 1.2 | 16/16 | 14.3 / 14.5 / 13.5 / 13.4 | 0-1 / 0-1 | 8.7-8.9 | PASS |
| cQ 0.5x / 0.75x / 1.5x / 2x | 16/16 | 13.9 (identical) | 4-17 / **0** | 8.8 | PASS (0.5x: post-stop roll 5.9) |
| spin 0.10 / 0.13 / 0.16 pinned, 0.16 told | 16/16 | 14.1 / 14.4 / 14.5 / 14.5 | 1-15 / **0** | 8.8 | PASS |
| CG +1.5 / +3 cm | 16/16 | 13.3 / 13.1 | 3/0, 0/0 | 9.1 / 9.7 | PASS |
| **CG -1.5 cm, cold start** | **0/16** | -- | -- | -- | FAIL (all 16 before the BT, t ~8 s -- piece 2's row exactly) |
| inertia 0.8 / 1.25 | 16/16 | 14.0 / 14.5 | 13/0, 5/0 | 8.8 | PASS |
**READING.** (1) **J0 is the number to measure, and the tolerance is one-sided: -0.10 is free, +0.05 costs
2.6 deg of corridor peak, +0.10 fails it.** A plant with MORE thrust at speed than the model runs ahead of the
speed ramp and buys the corridor with pitch -- the same family as piece 2's "more lift than modelled breaks the
trim". The 15x5's analogue band (0.39-0.71) is wider than that on the high side, so either measure it (a fan
at 10-16 m/s on the stand, or the cruise throttle of the first flight bag: the momentum model reads J0 straight
off the 16 m/s throttle, gate 0.67) or, until then, carry the controller's J0 on the HIGH side of the estimate
(a plant below the model is the safe error). (2) **cQ does not touch the flight at all** (peak 13.9 to the
tenth at every level) -- the whole cQ story is hover and heading, item 1 above. (3) **The spin-lag cliff
did not move with the smaller tau**: 0.16 pinned against a 0.067 model (+0.09 s of mismatch) flies at pk 14.5.
(4) **The aft-CG cold start is still zero tolerance** (piece 2's -1.5 cm row reproduced: pitch +53 at t 6 s,
inverted at 8): the engage-hover settle interlock remains the answer, not the plant.
**★ THE SOLVE-FAILURE COLUMN WAS MEASURING THE POST-STOP HOVER, NOT THE FLIGHT -- AND IT REWRITES TWO
EARLIER READINGS.** Every `ok=0` tick of every cell but one (j0 0.42, one at t 64 s) sits at t 71-76 s, V
0.3-0.5 m/s, a single tick at 40-50 ms: the outer's hover solve after the stop, where the count swings 1-17
between cells whose flights are identical. `ke_metrics` now prints the failures BEFORE the stop ("flight
fails") and `robust_metrics`' budget reads that (old summaries fall back to the total, so env1 / act1
reproduce). Re-scored: **the 2026-09-16 actuator-lag table's "0.14 / 0.16 pinned: flies but over the budget
(5-6 fails, 3x nominal)" were ALL post-stop hover ticks -- 0 in flight, pinned or told.** So regime 1 of that
block ("below 0.18 the cost is the mismatch and identification removes it") shrinks to: **below 0.18 s there is
no in-flight cost at all, told or not**; the cliff at 0.18 (5-16 of 16 lost before the BT, knowing does not
help) stands, and the spec (tau_spin <= 0.16, measure it) stands; the line "if not measured the tolerance
narrows to ~0.14" is withdrawn. Likewise the 2026-09-17 "solve failures 2 -> 6" (standing item 4): 0 -> 0
in flight; the post-stop hover count went up (the AoA read at 0.4 m/s swings +-90 deg and the outer's solve
hits its iteration cap for a tick) -- a solver nuisance at V < 0.5, named, not chased.
**WHAT THIS LEAVES, in order:** (a) the SAFE controller's yaw policy on the measured cQ (mixer priority + a
weathervane in hover; gate = `test_geometric_hover` §7 re-armed as "holds position, nose free" + a heading
check within authority at 3 m/s) -- safety-relevant, the flight lead's call; (b) the cascade hover hold in
wind (weathervane or a wind-sized lean; gate `--hover_only --wind`); (c) re-run the piece-4 wind table on the
measured build (its stops and hover ends were measured at 6x the real yaw authority); (d) B2 -- give the gate's
plant its own tilt-servo tau / rate knobs (today the ODE's servo IS the controller's `_AF` values, so a
specified-not-measured servo cannot be gated for mismatch) and sweep it like the spin lag; (e) the sensor
audit's `hw_ekf2` cell + the turning mission on the measured build; (f) then the staged HIL flight.

# ★★★★★ 2026-09-17: THE AIRFRAME GETS ITS FIRST MEASURED PROPULSION -- MN_4014 + 15x5 FROM A THRUST
# STAND, J0 FROM PUBLISHED WIND-TUNNEL DATA, THE SPECIFIED TILT ACTUATOR -- AND THE MEASURED cQ BREAKS
# THE SHIPPED YAW GAINS. Sim this commit, controller (airframe_id.cfg). NEW IDENTITY REFERENCES.
**WHAT WAS MEASURED, AND BY WHAT.** The user supplied two thrust-stand campaigns (T-Motor MN_4010 and
MN_4014, both on a 15x5, ~71 Hz, thrust in N, torque in N.m, optical RPM); they are stored with their
raw logs in `identification/2026-09-17_rotor_MN4010_MN4014/` and fitted by **`scripts/ana_spin_lag.py`**
(each throttle step as a DEAD TIME + first-order rise on OMEGA -- what the plant integrates and the L1
predictor models -- with the dead time free so a transport delay cannot be absorbed into tau; a
cross-check on sqrt(thrust); `--selftest` recovers a known tau to 3% and a known dead time to 1 ms at
the bench's noise and rate). **We refit rather than taking the shipped lambda:** the reports ship two
estimators that disagree 3x (DIRECT divides by (omega_cmd - omega) and gives R^2 0.24 with 18-24%
NEGATIVE lambda) and the two archives ship different ones.
| constant | was | now | where from |
|---|---|---|---|
| max_thrust | 18.0 N | **36.3 N** | bench, extrapolated to 100% from <= 80% |
| omega_max | 749 rad/s | **885.8** | bench |
| tau_spin | 0.12 s | **0.067 s** | bench, AT THE HOVER POINT |
| cq | 0.12 m | **0.0190 m** | bench; the SAME from both motors |
| prop | 12x8, J0 0.72 | **15x5, J0 0.52** | UIUC wind tunnel (below) |
| tilt tau | 0.12 s | **0.0756 s** | specified (13 rad/s) |
| tilt rate | 80 deg/s | **106 deg/s** | specified |
| tilt range | -20..+90 deg | **-20..+100** | specified; the aft brake is KEPT |
**★ SPIN-UP vs SPIN-DOWN IS NOT AN ASYMMETRY, IT IS AN AVERAGING ARTEFACT.** The whole-range medians
read 0.064 s up and 0.532 s down (8x) and that number is meaningless: spin-down is DRAG-driven, drag
goes as omega^2, so its constant scales with 1/omega -- 0.53 s at 20->10% throttle but 0.057 s at
80->70%. **At the hover point (9.69 N/rotor -> 458 rad/s -> 52% throttle) up and down agree: 0.064 /
0.072 s.** The tool therefore reports tau banded by omega and exports the hover-point value. This is
why the 2026-09-16 bench spec said "at the hover throttle": it is not a detail, it is the difference
between 0.067 and 0.53. Both motors are WELL inside the 0.16 s envelope -- half the modelled 0.12.
**THE DEAD TIME (81-88 ms) IS UNATTRIBUTABLE AND MUST NOT BE USED.** The fit is clean and consistent
across steps, but the stand's `ESC signal (µs)` column is EMPTY, so it cannot be separated from the
clock offset between the PX4 logger and the stand. **For the next bench session: log the ESC PWM on
the stand** and it becomes a one-clock measurement.
**J0 FROM PUBLISHED DATA, AND THE CATALOGUE WAS BIASED LOW (`scripts/ana_prop_j0.py`).** A static
stand cannot produce J0 (no airflow). The UIUC Propeller Data Site (Selig et al., 87 MB, md5 verified
against the published checksum) has wind-tunnel C_T(J) for hundreds of props, and **the sim's momentum
model non-dimensionalises EXACTLY to C_T = C_T0 (1 - J/J0)** -- so the x-intercept IS J0 and the fit
residual says whether that linear form is even right (R^2 0.97-0.999 over 237 propellers: it is).
| prop | catalogue said | measured | |
|---|---|---|---|
| 12x6 | 0.60 | 0.613 (APC) / 0.642 (Aero-Naut) | **-> 0.63** |
| **12x8 (flown until today)** | **0.72** | **0.776 / 0.774** | **-> 0.78**, two families agreeing to 0.002 |
| 12x10 | 0.84 | 0.926 | -> 0.93 |
| 13x8 / 13x10 | 0.68 / 0.79 | 0.725 / 0.935 | -> 0.73 / 0.94 |
| **15x5 (new)** | -- | not tabulated at p/D 0.333 | **-> 0.52** |
The old catalogue line `J0 = 0.72(p/D) + 0.24` was drawn through three 12-inch guesses and is low by
0.03-0.15; the measured line over 134 props of 10 in and over is **`J0 = 0.922(p/D) + 0.216`**. For the
15x5 (p/D 0.333) that gives 0.523, and the 16 measured props within +-0.08 of that p/D give 0.39-0.71
(median 0.51) with the closest analogues -- Aero-Naut folding 10x4 / 11x4 / 13x5 / 15x6 / 16x6 --
at 0.51-0.55. **The sim was therefore PESSIMISTIC about cruise thrust on every prop it has flown**;
nothing flown is invalidated (piece 2 allows +-0.1 and 0.72 vs 0.78 is inside it). At the new J0 the
15x5 cruise is comfortable: 67% throttle at 16 m/s against the flown 12x8's 78%.
**★★ THE MEASURED cQ BREAKS THE SHIPPED HOVER YAW GAINS -- the one real casualty, caught by the gate.**
**[SUPERSEDED 2026-09-21: the test's bar was in throttle units; the loop is clean and the gains stay. The real
casualty is heading-hold in a crosswind (the fin out-torques 0.74 N.m above ~4 m/s) -- see the block above.]**
`vtol.py`'s own comment always said so: `rotor_moment_ratio 0.12 (raised for MC yaw authority;
sim-only, real props ~0.02)`. The bench says **0.0190, identically from both motors**. Paired in, the
maximum hover yaw moment falls from 3.99 to 0.74 N.m -- **466 -> 86 deg/s^2 of yaw acceleration** --
and `tests/test_hover_hold.py` turns: **shipped kR_yaw 5.0 / kW_yaw 2.0 OSCILLATES; the highest gains
that hold a clean hover are 3.0 / 1.5.** A/B'd to attribute it: with cq back at 0.12 the shipped gains
pass, with 0.0190 they do not -- it is the cQ and nothing else in the build. The yaw loop was tuned
against 6x the authority the real propeller provides. **NOT retuned here** (a controller gain change
is its own piece with its own gate); 3.0 / 1.5 is the answer the gate already gives.
**WHAT THE MISSIONS SAY (both on `imu-mdob` + EKF + L1, the flown "Adaptive NMPC + Geo"):**
| | 12x8 baseline | MN_4014 + 15x5 |
|---|---|---|
| straight line, 16 seeds (`ke_sweeps/reg_mn4014`) | 16/16, pk 14.2, fails 2, stop 8.8 s | **16/16, pk 13.9, fails 6, stop 8.8** |
| turning mission, 3 seeds (`est_sweeps/mn4014_turns`) | 3/3, bob orbit 0.058, hits 0, roll sd 0.150, u_ad 0.006, stop 6.0 | **3/3, 0.049, 0, 0.154, 0.006, 5.9** |
The corridor peak IMPROVES (more thrust crosses the stall corridor at less angle of attack); the
turning mission is a null result to three decimals, INCLUDING the cancellation. The one column that
moved the wrong way is the solve failures, 2 -> 6 over 16 flights (0.13 -> 0.4 per flight). **Neither
mission tests the cQ finding** -- above ~10 m/s yaw is the rudder's job and the rotors' differential
is not the actuator; the exposure is hover and the low-speed ends, which is exactly where
`test_hover_hold.py` looks.
**IDENTITY, as the process demands -- the model changed deliberately, so NEW REFERENCES:** steso
`567a68790d16ffa5235663b9e6e9c29f`, IMU `fdd734f49160de7a07129826c78806c6` (same line as ever). The
J0-only intermediate step (12x8, 0.72 -> 0.78, before the propulsion swap) was `d492ca8a5b37f5c5b084583402e45a67`
/ `7f7b1e9f8d6294f155ce8c772dc873db`, recorded so the two changes stay separable.
**THE TILT PAIRING TEST FAILED AND WAS RIGHT TO; ITS INVARIANT WAS WRONG.** It required the servo's
`tilt_max_deg` to EQUAL the 90 the controller plans, so the new +100 read as a mismatch. The invariant
is **containment** -- the controller's planned range must fit INSIDE the servo's travel; extra travel
is headroom (past 90 the thrust vectors forward and DOWN, which no schedule should ever want). The
failure that matters is the reverse and it has bitten: the outer planned -25 deg of aft tilt on a
servo that stopped at -20 and the 1.9 N of braking never arrived (2026-09-14). Fixed in the test.
**THE ENCODER CLOSES THE ONE GAP THE SENSOR AUDIT COULD NOT BOUND.** The build specifies an AS5048A
(14-bit absolute) on each tilt shaft beside the servo, so beta becomes MEASURED rather than integrated
open-loop from the command. The audit's table had tilt position as "not sensed -- the one input for
which this audit can offer no tolerance at all". Wiring the node to READ it is a controller piece and
is not done. NOTE the servo torque: 106 deg/s against a ~630 deg/s no-load servo is a SOFTWARE slew
cap, not gearing (6:1 gearing would leave only ~20-30 deg of nacelle travel from a servo's ~180), so
the shaft sees the servo's own ~2.9 N.m -- and the dominant hinge-moment term, thrust-line offset x
thrust, has DOUBLED with the 36 N prop. Measure that offset.
**STANDING AFTER THIS:** (1) retune the hover yaw gains for the measured cQ (the gate says 3.0 / 1.5)
and re-gate; (2) `vtol_nmpc.py`'s BAKED trim table is now stale against the new propulsion -- it is not
in the flown path (the cascade's `Schedule.at` is live Python and adapts) but regenerate it before that
node ever flies; (3) wire the tilt encoder into the node; (4) the 16-seed solve-failure count 2 -> 6.

# ★★★★ 2026-09-16 (evening): THE ACTUATOR-LAG TOLERANCE, ON THE FLOWN "ADAPTIVE NMPC + GEO" LAW --
# AND THE ANSWER TO "ISN'T THAT THE L1'S JOB?": THE MODEL'S JOB UP TO 0.16 s, NOBODY'S PAST 0.18.
# 16 cells x 16 seeds, `scripts/robust_sweep.sh act1 scripts/cells_actuator_lag.txt` (piece 2's line and
# pass rule), plus a 3-cell bridge on piece 2's own law. Sim this commit; controller a5a6a16 untouched.
**WHAT WAS ASKED.** Piece 2 left the rotor spin lag as a cliff with nothing between the passing 0.12
(the modelled value) and the failing 0.20, and its surface row was never a mismatch test at all. The
question behind it: the L1 augmentation is supposed to absorb actuator error -- how much does it?
**THE CELLS ARE PAIRED, AND THE PAIR IS THE EXPERIMENT.** `--tau_spin X --l1_tau_spin 0.12` moves the
PLANT and PINS the controller's model = a genuine mismatch; `--tau_spin X` alone lets `l1_tau_spin`
follow = the controller is TOLD the truth. The pair separates "the model is wrong" from "the actuator
is too slow to fly". (The lags live only in `L1Aug.applied_moment`; `slosh_predictor.py` carries none,
so pinning is a complete mismatch with nothing else told -- checked before the run.)
**THE TABLE (`~/flight_bags/robust/act1`; imu-mdob + EKF + L1 = the flown `cascade_adaptive`; the
straight mission, back-transition at 60 s, y0 10 m, noise 0.1/1, CG tank; PASS = 16/16 landed,
solve-fails sum <= 4, corridor peak <= nominal+3, z >= 29.0, stop <= nominal+3 s):**
| cell | landed | corridor pk | fails (16 flights) | verdict |
|---|---|---|---|---|
| nominal | 16/16 | 14.2 | 2 | PASS (= piece 2's nominal 16/16, 14.3, 3, and the final-tree regression) |
| spin 0.06, model 0.12 | 16/16 | 13.9 | 1 | PASS |
| spin 0.09, model 0.12 | 16/16 | 14.0 | 2 | PASS |
| spin 0.14, model 0.12 | 16/16 | 14.2 | **5** | flies; over the solve-failure budget |
| spin 0.16, model 0.12 | 16/16 | 14.3 | **6** | flies; over the budget |
| **spin 0.16, TOLD** | **16/16** | 14.3 | **1** | **PASS -- measuring it removes the whole cost** |
| spin 0.18, model 0.12 | **11/16** | 14.2 | 4 | FAIL (5 crashed before the BT) |
| **spin 0.18, TOLD** | **15/16** | 14.4 | 5 | FAIL, but only just -- the knee |
| spin 0.20, model 0.12 | **0/16** | -- | -- | FAIL (all 16 before the BT) |
| **spin 0.20, TOLD** | **4/16** | 14.3 | 2 | **FAIL -- the truth buys 4 seeds, not the flight** |
| spin 0.30, model 0.12 | 0/16 | 34.5 | 6 | FAIL |
| **spin 0.30, TOLD** | **0/16** | -- | -- | **FAIL -- the truth buys NOTHING** |
| surf 0.045 / 0.06 / 0.10, model 0.03 | 16/16 | 14.3 / 14.3 / 15.3 | 1 / 1 / 0 | **PASS, all three** |
| surf 0.10, TOLD | 16/16 | 15.5 | 1 | PASS |
**THE READING.**
1. **Two regimes, and the boundary is 0.18 s.** **[CORRECTED 2026-09-21: the 5-6 "solve failures" of the
   0.14 / 0.16 rows were all post-stop HOVER ticks, 0 in flight -- below 0.18 there is no in-flight cost,
   told or not; the cliff and the spec stand. See the 2026-09-21 block.]** BELOW it the cost is the MISMATCH and identification
   removes it outright: 0.16 pinned flies 16/16 but at 6 solve failures (3x nominal), and the same
   0.16 with `l1_tau_spin` told the truth is 1 -- indistinguishable from nominal. ABOVE it the cost is
   the ACTUATOR: at 0.20 the controller that KNOWS the lag still loses 12 of 16, and at 0.30 knowing
   buys literally nothing (0/16 either way). **So the L1's answer is: its MODEL covers the band up to
   ~0.16 s once it is measured -- which is exactly why `l1_tau_spin` is a parameter and not something
   the adaptation discovers -- and past 0.18 s nothing covers it.**
2. **WHY knowing stops helping, in one number.** The inner loop's pitch authority in the corridor IS
   front/rear differential thrust (the wing carries nothing at 39 deg of tilt and 5 m/s), and its
   bandwidth is 1/tau: 8.3 rad/s at the modelled 0.12, 6.3 at 0.16, 5.0 at 0.20 -- against a **short
   period of 8.9 rad/s**. At 0.12 the actuator is already only AT the mode it has to control; at 0.20
   it is 44% below it. No augmentation and no feed-forward stabilises a mode faster than the actuator.
3. **THE FAILURE IS THE SAME IN BOTH CASES AND IT IS THE CORRIDOR** (spin 0.20, seed 1, t = 5-7 s,
   V 4-7 m/s, tilt 39 deg): pitch +23 (told: +14.7 then +58), elevator SATURATED at -1.00, the outer's
   solve time 13 -> 45 ms with ok=0, then roll 156 / yaw 180 and the ground. Never at the
   back-transition -- every loss in this sweep is BEFORE it. That is the same place piece 2's CG and
   control-derivative cells fail: the corridor is this airframe's authority minimum, whatever the cause.
4. **THE CONTROL LAW DOES NOT MATTER TO THIS** (the bridge, `~/flight_bags/robust/act1_steso`, piece 2's
   own law = the steso node's velocity force twin): spin 0.20 pinned 0/16, told 3/16 -- the flown
   imu-mdob law gives 0/16 and 4/16. One seed of scatter. The actuator-bandwidth limit is a property
   of the airframe and the inner loop, not of the estimator, so the piece-2 table's spin row transfers
   to the flown law unchanged (and its "0.20: 1/16" was the same one seed).
5. **★ CORRECTION TO PIECE 2's SURFACE ROW.** It recorded "0.10 passes; 0.06 (5 fails: marginal)" --
   but `robust_sweep.sh`'s surf cells never pinned `--l1_tau_surf`, so the controller was told the true
   servo lag and that row was never a mismatch test. Run as one: **0.045, 0.06 and 0.10 all pass 16/16
   with the controller believing 0.03** (fails 1 / 1 / 0), and 0.10 told is the same. 3.3x the modelled
   surface lag costs ONE degree of corridor peak and nothing else; the old "5 fails at 0.06" was seed
   scatter. The surface servo is not a sensitivity of this loop, mismatched or not.
**THE IDENTIFICATION SPEC IT WRITES (for the thrust-stand campaign):** measure `tau_spin` and set
`l1_tau_spin` to it; then the airframe's requirement is **tau_spin <= 0.16 s** (0.18 marginal, 0.20
unflyable at any model value) -- i.e. **rotor bandwidth >= ~6 rad/s, 0.7x the short period**. If the
measurement is NOT taken and the model stays at 0.12, the tolerance narrows to ~0.14 s before the
solve-failure budget goes. Measure it as a first-order lag on OMEGA, not on thrust: the plant lags
omega and then squares it, so fitting the raw thrust trace gives a systematically different constant.
Spin-up and spin-down separately (drag brakes differently than the motor accelerates; the model has
one constant). At the hover throttle -- that is where the differential authority the corridor needs
is set. `PROP_J0` is not a static-stand number: it needs airflow.
**TOOLING + TWO THINGS THAT BIT.** `robust_sweep.sh` now takes the control law as a knob (`ROBUST_NODE`
/ `ROBUST_LINE`; defaults are piece 2's exactly, so that table reproduces) and writes it to `<tag>/law`;
the cells live in `scripts/cells_actuator_lag.txt`. (1) The script `cd`s into the CONTROLLER repo before
it reads the cells file, so a relative path resolved against the wrong repo -- the sweep ran ZERO cells,
printed "done" and only `robust_metrics.py` noticed (a missing cells.txt). It resolves the path first
now. **A sweep that runs nothing must not look like a sweep that passed.** (2) The pass rule's
"fails sum <= 4" is summed over 16 flights, so a FAIL at 5-6 is 0.3-0.4 solve failures per flight
against nominal's 0.13 -- read the landed column first and the budget second; the 0.14/0.16 rows fly.

# ★★★★ 2026-09-16 (afternoon): THE SENSOR AUDIT -- WHICH SENSORS THE LOOP CAN ACTUALLY USE, WHAT EACH
# ONE IS ALLOWED TO GET WRONG, AND HOW THE OBSERVER HOLDS WHEN THEY ALL DO IT AT ONCE. 30 gate cells x
# 3 seeds (`scripts/sensor_sweep.sh`, `scripts/sensor_metrics.py`, sweeps `~/flight_bags/est_sweeps/
# sens_*`); sim this commit, controller a5a6a16 UNCHANGED (the realism knobs shipped with the hover
# hold, so every identity hash still holds). THE PAGE: https://claude.ai/artifact/M836419j6Lg3CGCbBuSUMH
**THE LOOP HAS FOUR SUBSCRIPTIONS, AND ONLY THREE OF THEM CARRY SENSOR DATA.** The flown configuration
(`nmpc_cascade_imu_node` on the base node, `state_source px4`, `airspeed_source px4`) reads
`fmu/out/vehicle_odometry` (EKF2's fused position / velocity / attitude / body rates),
`fmu/out/sensor_combined` (the accelerometer, FRD->FLU), `fmu/out/airspeed_validated` (CAS) and
`fmu/out/vehicle_status_v1` (arming state only). **Every other sensor on the aircraft -- the GNSS
receiver, the barometer, the magnetometer -- reaches this controller ONLY through EKF2**, which means
the hardware question for them is "does EKF2 stay converged", not "does the loop tolerate them".
**Three quantities are not sensed at all** and are modelled open-loop: the tilt servo's position (an
80 deg/s slew model), the rotor speeds (a 0.12 s first-order lag) and the battery.
**THE ASSUMED SUITE -- CORRECT THIS IF THE AIRFRAME DIFFERS; every hardware column below follows from
it:** a Pixhawk 6X-class FMU (redundant ICM-42688-P / ICM-20649-class IMUs, ICP-20100 baro, BMM150
internal mag), a u-blox M9N at 10 Hz (or an F9P RTK) with an external mag, an MS4525DO/SDP3x
differential-pressure pitot, PX4 v1.16 over uXRCE-DDS/Ethernet to the Orin.
| the loop's input | what it feeds | hardware source | rate (today / after the piece-6 rebuild) | what it gets wrong | verdict |
|---|---|---|---|---|---|
| position (ENU) | guidance only: cross-track, fly-by points, the orbit field, the RTH distance, the hover hold | EKF2 <- GNSS + baro + IMU | 100 / 200 Hz (`EKF2_PREDICT_US`) | receiver wander (flight 1b: +-0.9 m of orbit radius sd vs the mocap's 0.3), ~35 ms of fusion lag | **USED, tolerant**: benign to 5 Hz (2026-09-15 table); the observers never read it |
| velocity (ENU) | the outer NMPC's vs/vz, the wing share, the L1's model force, the guidance course frame | the same | the same | the lag and the noise are the loop's real sensitivity -- see table B | **USED, the binding one**: >= 20 Hz, lag <= ~35 ms |
| attitude R | the inner geometric law, the force/moment model, the tank's specific force | EKF2 <- IMU + mag (+ GNSS course) | 100 / 200 Hz | heading is the weak axis: in HOVER only the MAGNETOMETER holds it, near a water tank and four motors | **USED, required**; the mag question is open (below) |
| body rates | the inner rate damping, the moment observer (mdob), the tank's rotation terms | the gyros, through EKF2's odometry | 100 / 200 Hz | turn-on bias 0.2-1 deg/s; `IMU_GYRO_CUTOFF` 40 Hz | **USED, required**; 1 deg/s of bias costs 7x u_ad and nothing else |
| specific force a_b | the IMU force twin (`F_hat = m a_b - F_model`), i.e. the whole slosh cancellation | `sensor_combined` (integrated at `IMU_INTEG_RATE` 200 Hz, `IMU_ACCEL_CUTOFF` 30 Hz) | 87 Hz measured in HIL, capped at 100 by DDS / 200 after the rebuild | bias, scale, vibration, walk -- and above all its TIMESTAMP (table A) | **USED, required**; the one hard tolerance in the whole audit is its delay |
| airspeed (CAS) | the wind estimator, the air-relative outer frame, the wing share, surface authority, prop inflow | the pitot, through `airspeed_selector` | 8.5 Hz measured in HIL (the sim's rate); a real MS4525DO runs 50-100 Hz | 0.3-0.5 m/s of noise, a blocked or iced tube, nothing in reversed flow | **USED, and piece 4 says REQUIRED in wind** (blind, a 5 m/s headwind crashes the transition) |
| tilt servo position | -- | NOT SENSED (the node integrates its own command at 80 deg/s) | -- | a stuck or slow servo is invisible to the loop | **the gap worth closing**: piece 2 gives no tolerance for it, and one encoder would |
| rotor speed | -- | NOT SENSED (a 0.12 s lag model) | -- | piece 2: 0.20 s of spin lag crashes 15/16 where 0.12 is modelled -- a HARD cliff | **measure it on a thrust stand, do not sense it** |
| battery | -- | NOT READ (`battery_status` is on DDS, unused) | 1 Hz | thrust per throttle falls as the pack sags | **PX4's failsafe owns it** (piece 5's preflight); the L1 absorbs the slow part |
**TABLE A -- THE IMU'S ERRORS, ONE AT A TIME** (imu-mdob + EKF, the turning mission, the offset 100 mm
tank, 3 seeds; base = the flown gate line at `--noise_acc 0.02`). **y-bob = the LATERAL bob in the
steady orbit, the channel the cancellation acts on** (the tank's only arm is r_x, so M = r_x F_y);
x-bob = the fore-aft bob, which has no arm and is the LOOP's own response:
| cell (the knob) | y-bob [N] | x-bob | hits | roll sd | u_ad | raw corr | pred corr |
|---|---|---|---|---|---|---|---|
| base (0.02 m/s^2 white) | **0.018** | 0.055 | 0 | 0.150 | 0.006 | 0.67 | 0.78 |
| accel noise 0.05 / 0.10 m/s^2 | 0.019 / 0.024 | 0.055 | 0 | 0.150 | 0.006 | 0.50 / 0.32 | 0.71 / 0.57 |
| turn-on bias 0.2 / 0.5 m/s^2 | 0.017 | 0.055 | 0 | 0.150 | 0.006 | 0.67 | 0.78 |
| in-run walk 0.01 / 0.03 m/s^2/sqrt(s) | 0.034 / 0.033 | 0.100 | 0 | 0.15 | 0.007 / 0.011 | 0.35 / 0.13 | 0.81 / 0.69 |
| scale factor +1% / +3% | 0.017 / 0.015 | 0.055 | 0 | 0.150 | 0.006 | 0.65 / 0.54 | 0.78 / 0.76 |
| vibration 0.3 / 1.0 m/s^2 at 93 Hz | 0.018 | 0.055 | 0 | 0.150 | 0.006 / 0.013 | 0.33 / 0.11 | 0.75 / 0.54 |
| vibration 0.3 / 1.0 m/s^2 at **30 Hz** | 0.017 / 0.019 | 0.054 / 0.053 | 0 | 0.150 / 0.152 | 0.006 / **0.023** | 0.18 / 0.06 | 0.63 / **0.31** |
| gyro bias 0.2 / 1.0 deg/s | 0.021 / 0.025 | 0.054 / 0.051 | 0 | 0.149 / 0.139 | 0.010 / **0.043** | 0.69 / 0.71 | 0.79 / 0.81 |
| accelerometer lag 10 / 20 ms | 0.027 / 0.053 | 0.058 / 0.065 | 0 | 0.150 | 0.006 | 0.70 / 0.72 | 0.80 / 0.81 |
| **accelerometer lag 40 ms** | **0.364** | 0.166 | **14.7** | 0.161 | 0.008 | 0.85 | 0.88 |
| **the same 40 ms, DECLARED (`--imu_delay 0.04`)** | **0.084** | 0.077 | **0** | 0.150 | 0.006 | 0.72 | 0.81 |
| **all of them together (`hw_imu`)** | **0.046** | 0.099 | **0** | 0.141 | 0.012 | 0.28 | 0.75 |
**TABLE B -- THE STATE SOURCE AND THE PITOT** (the same line; `pos_hz/vel_hz` hold the controller's
view of position AND velocity, `pv_lag` ages it, `noise_vel` is white per sample):
| cell | y-bob | x-bob | hits | roll sd | u_ad | stop m | fails |
|---|---|---|---|---|---|---|---|
| base (200 Hz, exact) | 0.018 | 0.055 | 0 | 0.150 | 0.006 | 6.0 | 0.7 |
| pitot noise 0.3 m/s | 0.023 | 0.089 | 0 | 0.280 | 0.022 | 6.4 | 0.0 |
| velocity noise 0.05 m/s (200 Hz) | 0.055 | 0.259 | 12.7 | 0.152 | 0.011 | 6.9 | 2.0 |
| 100 Hz + **35 ms of lag** (EKF2's own) | 0.024 | 0.358 | 8.0 | 0.196 | **0.081** | 5.3 | 0.7 |
| **`ekf2_like` = both** (the realistic hardware state) | 0.057 | 0.322 | 42 | 0.208 | 0.083 | 5.6 | 1.7 |
| GPS-class 10 Hz + INS dead reckoning | 0.024 | 0.144 | 0 | 0.339 | 0.102 | 4.2 | 0.7 |
| the same + 0.1 s of lag (no INS) | 0.019 | 0.290 | 1.3 | **1.572** | **0.315** | 4.1 | 2.0 |
| **`hw_ekf2` = the whole hardware set** (IMU + EKF2-class state + pitot) | **0.088** | 0.353 | 47 | 0.570 | 0.042 | 5.8 | 1.0 |
| `hw_all` = the same with a BARE 10 Hz GPS instead of EKF2 | 0.039 | 0.283 | 12.7 | **2.202** | 0.080 | 4.0 | 4.3 |
**TABLE C -- THE OBSERVER, ON THE FULL HARDWARE SET** (`hw_all`, the harshest cell: every IMU error +
a 10 Hz 0.1 s-late GPS-class state + the pitot; the uncancelled orbit bob is 0.77 N):
| force channel + moment observer | y-bob | wall hits | roll sd | u_ad | pred corr |
|---|---|---|---|---|---|
| **IMU twin + mdob (`imu-mdob`, the recommendation)** | **0.039** | 12.7 | 2.20 | 0.080 | 0.29 |
| velocity twin + mdob | 0.722 | 261 | 3.06 | 0.355 | -0.00 |
| velocity twin + steso | 0.582 | 134 | 2.84 | 0.096 | -0.04 |
**THE READING, which is the point of the audit.**
1. **The cancellation is an IMU-only loop, and it holds.** The lateral bob it exists to remove sits at
   0.015-0.088 N in TWENTY-NINE of the thirty cells -- through 5x the sim's accelerometer noise, a
   0.5 m/s^2 turn-on bias, a 3% scale error, 1 m/s^2 of in-band vibration, 1 deg/s of gyro bias, a
   10 Hz GPS and the whole hardware set at once (uncancelled: 0.77 N). **The bias, scale and vibration
   terms are absorbed exactly where the design said they would be**: the EKF's bF state takes a
   constant (bias 0.2 and 0.5 are byte-level identical at the aircraft), and its 1.14 Hz bob state
   rejects everything above ~3 Hz, which is why 1 m/s^2 of vibration at 93 Hz drops the RAW reading's
   correlation 0.67 -> 0.11 and costs the aircraft nothing.
2. **The one hard tolerance in the whole audit is the accelerometer's TIMESTAMP.** 10 and 20 ms of
   undeclared delay are free; **40 ms breaks the cancellation outright** (y-bob 0.018 -> 0.364, 14.7
   wall impacts a flight -- the predictor cancels a force the tank made 40 ms ago, which at 1.14 Hz is
   16 deg of phase and adds energy). Told to the delay buffer (`IMU_DELAY` / `--imu_delay`), the SAME
   40 ms costs almost nothing (0.084, 0 hits): **it is not the latency that hurts, it is the
   MIS-DECLARED latency.** `IMU_DELAY` is 5 ms today, measured on flight 11 (`ana_imu_latency.py`,
   corr 0.95-0.99 per axis). **Standing item: re-measure it on the Orin link before the first tank
   flight** -- an Ethernet uXRCE-DDS hop plus the Orin's own scheduling is exactly where 20-40 ms
   would come from, and it is a one-line parameter, not a redesign.
3. **In-band vibration is the other thing to watch, and it is a mount problem, not a filter problem.**
   1 m/s^2 at 30 Hz (i.e. what survives `IMU_ACCEL_CUTOFF` 30 Hz, a soft prop mount or a resonating
   arm) takes u_ad 0.006 -> 0.023 and the prediction 0.78 -> 0.31 while the aircraft still flies; at
   93 Hz the same amplitude is harmless. Check the Orin's first hover bag with an FFT of
   `sensor_combined` before trusting the predictor's numbers, and treat a peak under ~40 Hz as a
   mount to fix.
4. **EVERY OTHER degradation in the table is the STATE SOURCE, and it shows up in the fore-aft bob,
   the orbit roll and u_ad -- never in the cancellation.** Decomposed: the 35 ms of EKF2 fusion lag is
   what drives u_ad (0.006 -> 0.081) and the fore-aft bob (0.055 -> 0.358); 0.05 m/s of white velocity
   noise adds the wall hits but leaves u_ad and roll at their base values. A bare 10 Hz GPS with 0.1 s
   of lag is 10x worse than EKF2 on every column (roll sd 1.57 vs 0.20) -- **which is the argument for
   the piece-1 architecture stated in one line: the loop flies EKF2's INS, not a receiver.**
5. **The realistic hardware aircraft (`hw_ekf2`) flies the mission**: 3/3 seeds, 1 solve failure, the
   stop at 5.8 m, orbit roll sd 0.57 deg (HIL's own band is 0.83-1.26), the lateral bob cancelled to
   0.088 N. Its 47 wall hits are the FORE-AFT bob pressed on its wall by the loop's response to a
   lagged velocity -- no arm, no moment, and the per-axis wall guard of piece 7 is what makes that
   harmless by construction.
6. **The velocity twins collapse there and imu-mdob does not** -- the 2026-09-15 finding, now measured
   with real sensor errors rather than a clean hold: on `hw_all` the velocity twins leave the bob
   uncancelled (0.58-0.72 of 0.77 N), hit the wall 134-261 times a flight and push 0.10-0.36 N.m of
   augmentation, while the IMU channel keeps it at 0.039 N. **The recommendation stands unchanged and
   is now hardware-justified, not just rate-justified.**
**WHAT THE AUDIT DOES NOT COVER, stated plainly:** the magnetometer (the loop never reads it, but in
HOVER it is the only thing holding EKF2's yaw, and this airframe carries a tank of water and four
motors -- a sim gate cannot answer it; the flight answer is the piece-5 card's 180 deg hover yaw check
plus a mag calibration away from the airframe's own currents); GNSS loss / multipath / an RTK fix
dropping to float (EKF2's business, and its failsafe is PX4's); a blocked or iced pitot (the
`airspeed_validated` path has PX4's own checks, and `tailwind_max` is the interlock); sensor
FAILURE rather than sensor error -- the interlocks cover the state going stale (`state_stale_s` 0.2)
and the accelerometer going stale (`IMU_STALE_S` 0.2 -> the cancellation zeroes itself, one warning),
but no cell here kills a sensor mid-flight. 3 seeds per cell.
**THE KNOBS (all on the gate, controller a5a6a16):** `--vib A,f` `--acc_walk W` `--acc_scale S`
`--imu_lag S` `--bias_gyro x,y,z` on top of the existing `--noise_acc --bias_acc --imu_delay
--pos_hz --vel_hz --pv_lag --noise_vel --imu_dr --airspeed --noise_aspd --aspd_hz`; the plant's IMU
seam applies scale, bias, walk, a three-phase vibration and a ring-buffer delay in that order and
writes `base.IMU_ACCEL`, so the node reads exactly what a hardware accelerometer would hand it.

# ★★★★ 2026-09-16 (late morning): AFTER THE PLAN -- THE ROS TICK'S CLOCK (piece 6's correction, in the piece-6
# block), THE HOVER POSITION HOLD, AND THE STAGED HOVER FLOWN CLEAN. Controller 82a23b5..f26e09d, sim this commit.
**THE HOVER POSITION HOLD (`HoverHold` in the base node, controller 0f1cb5c + f26e09d).** The cascade held a
heading and a speed reference, never a position: the 30 s hover stage of 02:32 drifted at 0.5-1.7 m/s (20 m)
and the cross-track intercept law (`track_heading`: heading = bearing + k_ct x cross-track) turned the nose
27 deg to intercept a track a hover cannot fly. Now, while the speed reference is PARKED at zero -- the engage
hover before settle, a `v_cap 0` stage, the RTH stop (`_is_stopped()`, the status feed's stopped=1) -- the
node latches the position and yaw where the hold engages (logged `HOVER HOLD at (x, y) yaw`, `released`) and:
(1) posts the outer an along-heading SPEED reference vs = -hold_k e_s clamped to `hold_vmax` (the outer's own
vs channel flies it with tilt and pitch as ever; `sched.at` clamps its trim package at V = 0 and `build_refs`
carries the negative vs -- the aft tilt is the backward authority); (2) hands the inner a hover ROLL reference
ky e_y + kvy v_y clamped to `hold_roll_max_deg`, faded by (1 - wing share) -- the multicopter lateral loop;
FLU +roll = right wing down = the aircraft moves RIGHT, e_y > 0 = left of the point, checked on the gate (a
wrong sign diverges); the term enters `inner_step` through `g["roll_hover"]` behind a `!= 0.0` branch (a
"+ 0.0" would flip a signed zero in the trace); (3) holds the latched heading (`heading_cmd = hold.yaw`,
`ct=` the hold's e_y). Released the moment the schedule ramps (the transition starts from its own zero as
before). Params `hold_k` 0.5 1/s (0 = the old hover), `hold_vmax` 1.0, `hold_ky` 0.06 rad/m, `hold_kvy`
0.12, `hold_roll_max_deg` 8; status feed `hold=1 he=(e_s,e_y)`. The gate mirrors it (`--hold_k ..`; the
2 s start hover, `--hover_only`, the RTH stop) and prints the worst displacement.
**GATE.** A 2 m/s^2 sinusoidal lateral gust (0.2 g, 7 s period) in a 40 s hover walks the aircraft to
(20, 13) m without the hold and stays within (1-2, +-2) m with it (k 0.3 / 0.5 / 0.8: 3.4 / 3.3 / 3.2 m worst);
a 3 cm CG error hover holds within 1.6 m; the turning mission (imu-mdob, 3 seeds): bob 0.058 N in the orbit /
0 wall hits / roll sd 0.141 / u_ad 0.006 unchanged, the RTH stop 5.8 +- 0.05 m from home and the aircraft
then sits within 0.71 m of it (before: drifting at 0.5 m/s with the nose walking 28 deg in 5 s), fails 0/0/3
(scatter). **THE STANDING ALONG-TRACK OFFSET is the outer's vs bias divided by k:** the HIL hover sat 2.0 m
ahead of the point at k 0.3 (the outer's w_vs 8 against its trim references leaves ~0.5 m/s of standing vs
error at zero command) -> `hold_k` 0.5, ~1 m; lateral 0.03 m rms. An integral would remove it; not built.
**FLOWN (row R100_dx3_hover30_hold_a1_094944, bag slosh_20260916_094918; `STAGE=hover STAGE_HOLD=30
CASC_SPIN=poll`, imu-mdob + EKF + L1, px4 state, the offset tank): LANDED 0.00 m.** The hover stage sat at
(2.0 +- 0.2, 0.0) at V 0.02-0.05 m/s with yaw 0 for its 30 s ("go"), then the schedule's back-transition from
the hover, the recover, and the landing WHERE IT WAS: the driver now lands a staged flight at the aircraft's own
position (`fly_transition.py`: the mission's `--land-x/y` is the far target, and the 09:24 hover flight had
been sent 700 m along the ground on the safe controller to reach it, "CRASHED 71 m short"). `ims=` 0.80 / 1.29
in the loop. The two earlier hover flights of the morning (02:07 the creeping cap, 02:32 the resumed schedule,
09:10 busy / 09:24 poll: the landing target) are in the piece-7 and piece-6 blocks.
**THE TURBULENCE ENVELOPE'S TWO KNOBS (piece 4 left them untested; `wind_sweep.sh` cells `*_ttr8`, `*_vc17/18`,
`*_rtf085`, `*_d10`; sweeps `~/flight_bags/wind/turb2..5`, 16 seeds per cell, imu-mdob + pitot, the turning
mission, the offset 100 mm tank):**
| cell (turbulence 1-sigma per axis) | landed | fails | corridor pk | Vair min | orbit rho-R | stop |
|---|---|---|---|---|---|---|
| still, Vc 16 (wind1) | 16/16 | 16 | 11.6 | 16.01 | -0.2 +- 0.1 | 6.3 m |
| still, Vc 16, ttr 8 | 16/16 | 5 | 10.3 | 16.01 | -0.2 +- 0.1 | 9.1 |
| **still, Vc 18** | 16/16 | 18 | **8.6** | 17.94 | +1.0 +- 3.7 | 10.5 |
| turb 0.5, Vc 16 (wind1) | 15/16 | 99 | 87 (the one stall) | 10.5 | +0.3 +- 1.1 | 5.7 |
| **turb 0.5, Vc 18** | **16/16** | 24 | 14.2 | 16.56 | +7.9 +- 8.3 | 9.9 |
| turb 1.0, Vc 16 (wind1) | 2/16 | 1343 | -- | 5.5 | -- | -- |
| turb 1.0, Vc 16, ttr 8 | 5/16 | 1062 | -- | 6.8 | -- | -- |
| turb 1.0, Vc 17 | 8/16 | 796 | -- | 6.3 | -- | -- |
| turb 1.0, ttr 8 + Vc 17 | 6/16 | 855 | -- | 6.7 | -- | -- |
| **turb 1.0, Vc 18** | **14/16** | 270 | 18.1 | 15.26 | +13.6 +- 13.5 | 21 |
| headwind 5, Vc 16 (wind2) | 16/16 | 125 | 11.7 | 16.01 | +5.6 +- 16 | 22 |
| headwind 5, Vc 18 | 16/16 | **8** | 8.2 | 17.94 | +0.3 +- 2.5 | 17 |
| still, Vc 16, RTF 0.85 (HIL's clock) | 16/16 | 69 | 12.2 | 16.02 | -0.1 +- 0.1 | 9.3 |
| still, Vc 18, RTF 0.85 | 16/16 | 47 | 9.4 | 17.94 | -0.5 +- 1.7 | 13.4 |
| turb 0.5, Vc 16, RTF 0.85 | **13/16** (corridor, t 10-17 s) | 241 | 83 | 11.8 | +0.1 +- 1.1 | 11 |
| turb 0.5, Vc 16, RTF 0.85, decel 1.0 | 13/16 (the same three) | 253 | 83 | 11.8 | -- | 18 |
| turb 0.5, Vc 18, RTF 0.85 | 15/16 (corridor, t 11 s) | 108 | 70 | 16.3 | +7.6 +- 8.3 | 18 |
| turb 0.5, Vc 18, RTF 0.85, decel 1.0 | 15/16 (the same one) | 104 | 70 | 16.3 | -- | 29 |
**READING.** (1) The CRUISE SPEED is the lever, `ttr` is not: at 18 m/s the wing sits 2.9 m/s above its stall
instead of 0.9 (the cruise AoA lower, 6 deg more to the 13 deg break against a vertical gust) and the 1 m/s cell
goes 2/16 -> 14/16, the 0.5 m/s cell 15 -> 16/16, the headwind cell's solve failures 125 -> 8; the corridor peak
is LOWER at 18 (the speed ramp Vc/ttr is quicker -- the ttr finding again); `ttr 8` alone buys 5/16 and costs
still-air fails. The costs of 18: the orbit is 4x rougher (+-3.7 m still, +-8-13 in turbulence: 18.4 deg of
coordinated bank for R 100 against the 25 deg limit), the stop 10-13 m (27% more energy), the front throttle
0.88 (the 12x8's headroom thins: 0.95-1.00 in turbulence). (2) **At HIL's clock the CORRIDOR is where turbulence
bites, at either speed and whatever the decel rate**: 3/16 at 16 and 1/16 at 18 stall out of the transition at
t 10-17 s in 0.5 m/s turbulence (the wall-clock ramp asks 1.76 m/s^2 of a 0.85-RTF plant, the known +2-4 deg
HIL onset, and a 1.5 m/s vertical gust takes the rest) -- hardware has RTF 1, where the same cell is 15/16 and
16/16. (3) **NOT ADOPTED: the one HIL flight at Vc 18 (row R100_dx3_vc18_turb05_a1_104340, bag
slosh_20260916_104227, `CASC_VC=18 VTOL_TURB=0.5 CASC_SPIN=poll`) flew the legs, both turns and the orbit at
18.0-18.2 m/s of air and DEPARTED AT THE RTH STOP'S STALL CROSSING**: back-transition armed at 18.1, the rotors
upright by V 15.2 (the schedule's "rotors first"), then at V 13.0 pitch +23 (AoA 24.8, the wing share 128 ->
40%: stalled) with 2 solve failures, +31 at V 11.5 with 8, a tumble at V 6 -- the piece-2/knife-edge signature
on a crossing that starts 2 m/s higher. Not reproduced by the gate: 32 stops from 18 (16 at RTF 1, 16 at 0.85)
and the 15 turbulent ones all stop clean. One flight; the stop from 18 is the thing to gate more (a two-speed
decel -- 18 -> 16 on the cruise trim, then the flown back-transition -- is the design candidate) before 18 is a
mission option. `vcruise` stays 16; `CASC_VC` (both launchers, forced to a DOUBLE now: `CASC_VC=18` had killed
both cascade panes with rclpy's INTEGER-for-DOUBLE rejection, the CASC_SETTLE_S lesson) is the knob.
**THE FINAL-TREE REGRESSION (the flown straight line, `ke_sweep.sh`, 16 seeds, y0 10 m, noise 0.1/1, lat 0.03 /
comp 0.049 / outer_lag 0.03, the CG tank; `~/flight_bags/ke_sweeps/reg_{imu,steso}`):** imu-mdob 16/16, worst
crossing +14.2, fails 2, stop 8.8 s; steso-node mdob 16/16, +14.3, fails 2 -- the references (16/16, 14.3, 3)
to the seed.
**TAGGED `hw-transfer-1` IN THE FOUR REPOS** (sim 05a4a0e, controller f26e09d, GS 96a1fb3, msgs ebeded8; each
tag message pins the other three, as `pre-slosh` does): the state the hardware campaign starts from.
**THE IDENTITY CHECK MOVES WITH IT, DELIBERATELY:** the gate's first 2 s are a hover, so the hold changes the
flown line's trace there. `--hold_k 0` reproduces the old hashes EXACTLY (steso line `ef8e90e4e5ef6afbe2935ba81fa4bbe0`,
IMU line `e958efbc7bbf9687db76620563cdabf8`) -- the proof that nothing outside the hold moved -- and the new
references at the default are **steso `14a3f07da34821e956ad21ceb5bea8c1`, IMU `c9aed0b3e7a64a481e24834eb613b266`**
(the same lines: `--test --est mdob --ttr 10 --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1
--noise_rate 1.0 [--noise_acc 0.02] --l1 --dur 8 --every 1.0 --seed 3 --log`, md5 of the CSV minus the ms
columns). Tests: boot imu / vcap / interlock, test_imu_node, test_cascade_structure, test_gs_cascade_params,
test_gs_param_types, test_cascade_guidance, test_wind_frames all pass.

# ★★★★ 2026-09-16 (morning): HARDWARE-TRANSFER PIECE 7 -- THE TANK ON HARDWARE: THE RING-TEST TOOL, THE
# MEASURED-MODE OVERRIDES ON BOTH SIDES, THE EKF RETUNE FROM A HOVER BAG, AND THE WALL GUARD THAT MAKES A
# WRONG MODEL HARMLESS. Controller 00ba3d8 / 2f1b519 / dcbcaef / 09f7daa, sim ce3793b / 6aa804f / this
# commit. The hardware half (a real tap, a real hover bag) waits for the airframe; everything that can be
# proven here is, and the two HIL hover flights of the morning found two driver/cap bugs (below).
**THE RING TEST: `scripts/ana_ring_test.py --bag <dir> [--t0 --t1] | --csv | --selftest`.** Procedure in its
docstring: the airframe HUNG from a strap or held (on its gear the ground's friction takes the 0.4 N reaction
and the IMU sees nothing), the tank at the flight fill, PX4 logging `sensor_combined` as in flight, ONE
lateral tap on the tank wall (body y, then x), 20 s of quiet, three taps per axis, the median. The tool: the
FFT peak (Hann, 8x zero padding, parabolic refinement) of the detrended lateral acceleration = f1; the
damping from the log decrement of the band-passed (+-35%) analytic envelope, fitted on ln(env^2 - floor^2)
where the ring stands clear of the noise (the noise adds in quadrature and flattens a plain envelope fit by
25% at SNR 2), cross-checked by the successive-peaks decrement; compared to `slosh_parameters()` of the
tank the environment describes and printed as the export line. `--selftest` rings the simulator's own
`PendulumSlosh` (by path) at 20 mm and samples its reaction on the 3.95 kg airframe at 200 Hz + white
noise: f1 1.144 vs 1.144 Hz at every noise tried; zeta 0.23% for 0.25 (0.02 m/s^2 noise), 2.5% for 3 and
1.1% for 1 at 0.05, 5.4% for 5, 0.36% for 0.5 at 0.1 m/s^2 (SNR 1) -- f1 exact, zeta within 30%. **An
in-flight window is NOT a ring test:** the staged flight's post-stop hover reads 1.00 Hz at 0.41 m/s^2 --
the airframe's own ~1 Hz pitch ring with the bob in the loop (every offset-tank flight has it), not the
tank alone; the tool says OUTSIDE 10% and it is right. There is no HIL rehearsal of a tap (a grounded sim
vehicle cannot be tapped, and in a hover the loop answers); the self-test is the tool's validation.
**THE OVERRIDES: `VTOL_TANK_F1` [Hz] / `VTOL_TANK_ZETA`** (slosh.py: `TankConfig.freq_hz`, read by
`default_tank()` with `damping_zeta`; `slosh_parameters()` then sets omega = 2 pi f1 and L1 = g / omega^2
while m1 keeps the closed form -- a tap measures the frequency, not the mass fraction). The plant's pendulum
and the predictor's model (`_tank_params` -> the same module by path) move together by construction:
`test_tank_model_pairing.py` section 9 sets f1 1.00 / zeta 0.02 in a subprocess and reads f1, zeta, L1
(248.5 mm = g / w^2) and m1 (0.408, unchanged) identical on both sides. Isaac's banner prints
`f=1.00 Hz (measured)`; the launchers forward both (Isaac and every controller pane, cold start and
restart); `start_slosh_hil.sh` refuses READY unless Isaac's banner carries the measured f1 AND the adaptive
cascade's predictor banner says `f1=<the same>` (times `SLOSH_PRED_F1_SCALE` if a mismatch is flown).
**THE WALL GUARD (controller 00ba3d8) -- TWO MECHANISMS, THE FIRST IS THE ONE THAT MATTERS:**
(1) **The cancellation uses the model's force only inside its domain.** `SloshPredictor.force_bound()` =
m1 w^2 wall (1.05 N for the 100 mm tank) is the largest force the LINEAR model can make; past the wall the
predictor's contact law is the bound on the model (slosh.py's own words), not an impact to cancel, and an
estimate that got there on a residual the tank did not make predicts 5-16 N impacts -- the 2026-09-15
velocity-twin pulses (12-13 clipped 1 N.m yaw pulses per flight). `bounded(F)` clips F_slosh, M_slosh (out
of the C(s) target) and M_slosh_ahead (the direct term) in BOTH copies; predict(), force, the log and the
wall ETA keep the raw prediction. `test_slosh_predictor.py` section E: a fictitious 3 N residual at f1 with
the plant's bob at rest drives the ESTIMATE to 52 mm in a 50 mm wall and the raw model force to 14.5 N;
bounded, 1.05 N -- so at the 0.26 m arm the worst spurious moment is 0.27 N.m on any channel, 4x under
PRED_UMAX (which stays 1 N.m). (2) **A parked estimate takes that axis's model out.** `wall_s` (2,)
counts, PER AXIS, the seconds the estimate sits AT or past the wall (>= wall counts, 80%..wall holds,
< 80% resets: a pressed bob jitters across the line); past `SLOSH_PRED_WALL_S` (1.0 s, `--pred_wall_s`,
0 = off) the L1 masks that axis's model force to zero in F_slosh / M_slosh / M_slosh_ahead, so its residual
goes through C(s) as before the predictor existed, one log line each way, `WALL-GUARD x|y` on the status
feed, a trip count in the gate summary. Per axis because on this airframe ONLY the y bob has a moment
arm (r_tank = (0.26, 0, 0): M = (0, 0, r_x F_y)) and the fore-aft bob is PRESSED on its wall by the
aircraft's own f_x in every corridor and cruise (HIL: 47 of 50 mm in cruise, on the wall through the
transition -- the 02:32 flight's first guard trip, 312-327 s, was exactly that): a one-axis guard at 90%
of the wall would have switched the lateral cancellation off for every HIL cruise. A passing excursion
never trips it (0.28 s max in section E); section D's braking bob held 2 s on the wall runs it past 1 s
and releases cleanly.
**THE MISMATCH GATE (`--pred_f1_scale`; imu-mdob + EKF, the offset 100 mm tank, turning mission, 3
seeds; `~/flight_bags/est_sweeps/p7_f1{nom,08,112,06,115}`):**
| predictor's f1 | bob|F| orbit [N] | wall hits | orbit roll sd | u_ad | pred corr / err | guard | stop m |
|---|---|---|---|---|---|---|---|---|
| off | 0.768 | 269 | 0.359 | 0.035 | -- | -- | 6.0 |
| x1.0 (1.14 Hz) | 0.059 | 0 | 0.145 | 0.006 | 0.81 / 0.078 | x: 1 trip, 5.4 s in the RTH brake | 5.9 |
| x0.8 | 0.060 | 0 | 0.138 | 0.006 | 0.77 / 0.096 | 1 trip, 7.3 s (one-axis guard) | 6.0 |
| x1.2 | 0.059 | 0 | 0.148 | 0.006 | 0.80 / 0.087 | 1 trip, 3.3 s (one-axis guard) | 5.7 |
| x1.5 | 0.061 | 0 | 0.144 | 0.006 | 0.78 / 0.106 | 0 | 6.0 |
| x0.6, one-axis guard | 0.770 | 273 | 0.360 | 0.035 | 0.82 / 0.716 | tripped at t~1 s, off 172 of 215 s | 5.8 |
| **x0.6, per-axis guard (shipped)** | **0.182** | **0** | **0.138** | **0.009** | 0.81 / 0.240 | **x parked 171 s; y never** | 5.9 |
+-20% (and +50%) of f1 costs NOTHING at the aircraft level: the EKF is corrected by the measured residual
every tick and the direct term looks only 49 ms ahead (6% of a period), so a frequency error is a small
phase error -- prediction quality slips (err 0.078 -> 0.09-0.11 N), the cancellation does not. At x0.6 the
model needs a bob 2.8x larger for the same force: the fore-aft estimate parks at its wall from the first
second (its guard trips and stays tripped: no arm, nothing lost), the lateral one does not park (the
orbit's y force is smaller) and its cancellation runs on the wrong model -- and STILL takes the orbit bob
0.77 -> 0.18 N with zero wall hits, roll sd 0.138, u_ad 0.009, the stop unchanged. With the earlier
one-axis guard the x park had switched y off too and the run was the predictor-off run to three decimals.
**Either way: a wrong model degrades toward zero, not to harm -- by the bound and the guard, measured.**
The one x trip per nominal run is the RTH brake's legitimate park (the bob pressed on its wall by the
1.5 m/s^2 decel for ~5 s); the C(s) path carries the static part meanwhile, stop distances unchanged.
**THE EKF RETUNE: `scripts/ekf_retune.py <bag> [--window T0 T1] [--gate LOG.csv]`** reads the flown
cascade's status feed (`slosh F=` the force twin's raw reading, `sig=` the moment observer's raw reading --
NEW on the feed with 2f1b519, so bags before it give the force floor only; `pred bob=`, `bF`, `bM`), finds
the longest still hover (V < 0.5, |tilt| < 5 deg, >= 5 s), takes the RESIDUAL of each reading against the
model (reading minus the model's bob force minus the bias state = the EKF's own innovation; a 0.25%-damped
tank rings for a minute after the engage kick, so the raw reading is not a floor) on the y force channel and
the three moment axes as the flown floors, compares them to the gate's (imu-mdob hover, `--log`: F_y
0.039 N, sigma 0.049 / 0.063 / 0.096 N.m -- or `--gate LOG.csv` recomputes), and scales the sweep's r_F 0.3 / r_M
0.1 by the ratio per channel, clamped to [0.5x, 5x], printing the `SLOSH_PRED_RF/RM` export line and the
bias states the hover reveals. Rehearsed on a HIL hover (below). The rule it encodes: keep the sweep's
ratio of r to floor; measure the floor, do not guess it.
**THE TWO HIL HOVER FLIGHTS (`STAGE=hover STAGE_HOLD=30`, imu-mdob + EKF + L1, px4 state, the offset
100 mm tank; rows R100_dx3_hover30_a1_020625 / _023215):** the first was REFUSED by its own stage ("speed
3.1 outside 0-2" at 20 s): `v_cap 0` was not holding -- the reference ramped up by Vc/ttr and down by
decel_rate in the same tick, a net +0.1 m/s^2 creep (the piece-5 staged flight's hover stage had "passed"
only because its 15 s hold ended at 2.3 m/s inside the window's +1 m/s tolerance). Fixed (controller
dcbcaef: the ramp may not cross the cap; a lowered cap still ramps down), pinned by the boot test's new
`vcap` variant (one rclpy probe streams a hover, switches, reads the feed: vcmd 1.2 on the flown cap,
0.0 on the fix). The second held the hover 30 s (V 0.5-1.7 m/s: the cascade holds a heading, not a
position -- it drifts) -> "go", then the DRIVER let it go: after 3b's `decel_now 0` the still-active,
settled node resumed its forward schedule and the RECOVER phase (waiting for V < 2) watched it transition
to 16 m/s; the safe controller took a wing-borne aircraft (roll 49 deg) and still landed it 0.00 m from
the target. `fly_transition.py` now sets `v_cap 0` before it clears `decel_now`. **What the flight gave:**
`ims=` in the ROS loop (piece 6's correction above), the `sig=` field, the guard's first HIL trip (the
x bob parked through the unplanned transition, 312-327 s -- the per-axis guard came from it), and
`ekf_retune.py`'s rehearsal: hover window 269-307 s, bob 9.5 / 9.8 mm rms (the tank ringing after the
engage kick at 0.25% damping), raw y reading 0.189 N, residual floor 0.092 N vs the gate's 0.039 (2.4x:
the sim IMU through the twin plus the model's share), moment floors 0.009 / 0.014 / 0.039 N.m vs the
gate's 0.049 / 0.063 / 0.096 (5x UNDER: HIL's rates carry no noise, the known blind spot), bias states
bF (+0.17, -0.03) N, bM (+0.02, -0.01, +0.02) N.m; proposal r_F 0.70 / r_M 0.05 (the 0.5x clamp), NOT
applied -- the sweep's values flew every HIL flight; the tool is for the airframe's floors.
**STANDING FOR THE HARDWARE:** tap, read f1; if outside 10% of the closed form export the override (both
banners must show it or the launch refuses); fly a 30 s hover (`STAGE=hover STAGE_HOLD=30`), run
`ekf_retune.py` on its bag, fly the hover again with the exported r and score the prediction (status feed /
`ana_slosh_est.py`) before the mission. The guard needs no retune. What the model still cannot claim:
breaking waves and roof impacts (the standing statement); a baffled tank remains the hardware answer.

# ★★★★ 2026-09-16 (early morning): HARDWARE-TRANSFER PIECE 6 -- THE COMPUTE BUDGET FOR THE JETSON ORIN,
# MEASURED HERE PER COMPONENT, THE HOT PATH MADE 3x CHEAPER BIT-IDENTICALLY, THE PX4 RATES THAT MUST
# CHANGE FOR 200 Hz. Controller 71814b8, sim this commit. The Orin itself is not on the bench yet.
**THE BENCH: `scripts/bench_loops.py [--ticks N] [--outer N] [--load] [--pin N] [--json F] [--profile]`** times
the recommended configuration (imu-mdob + EKF predictor + L1, the offset 100 mm tank, a wing-borne state):
the inner tick as the node runs it (`base.inner_step`: the geometric law, the allocation with the prop inflow,
the L1 with the moment observer, the IMU force residual, the EKF predictor and its cancellation) and its
parts, and the outer solve warm; `--load` times each loop WHILE the other runs as in the node (the outer
solving back to back in a thread while the inner is timed; a 200 Hz inner thread while the outer is timed --
CasADi releases the GIL, Python does not); `--pin 2` restricts the process to two cores as a small-computer
proxy. **The desktop numbers (24-core workstation; the Orin's A78AE is taken as half of one of these cores):**
| loop | median | p95 | of its tick | Orin x2 (predicted) |
|---|---|---|---|---|
| inner tick, alone | 0.41 ms | 0.44 | 8% of 5 ms (200 Hz) | 0.83 / 0.88 ms = 17% |
| inner tick, the outer solving meanwhile | 0.43 | 0.55 | 9% | 0.87 / 1.1 = 22% p95 |
| inner tick, pinned to 2 cores, under load | 0.42 | 0.57 | 9% | 0.84 / 1.14 |
| outer solve (nmpc6 JIT, N 14), alone | 11.5 | 12.2 | 23% of 50 ms (20 Hz) | 23 / 24 ms = 49% p95 |
| outer solve under a 200 Hz inner loop | 12.6 | 13.4 | 25% | 25 / 27 = 54% |
| outer, 2 cores, under load | 11.8 | 12.8 | 24% | 24 / 26 |
Parts of the tick (median): L1 applied_moment (model force/moment, the IMU residual, the EKF step + the
delay-ahead prediction) 0.105 ms, the EKF step alone 0.103, `model_moment` (the CasADi aero call) 0.053,
the L1 law + moment observer 0.009, the allocation inverse < 0.001.
**★ THE TICK IN THE ROS PROCESS WAS 2.5x THE BENCH'S, AND IT WAS THE CORE CLOCK, NOT rclpy (2026-09-16
morning, controller 82a23b5 + baa27e4).** `ims=` from the first HIL bag with it (the 02:32 hover, imu-mdob +
EKF + L1, `state_source px4`): 1.05 ms median / 1.66 p95; on the IDLE desktop with a synthetic 200 Hz
stream (`CASC_PROF=N` cProfiles N ticks of `inner()` in the ROS process -- a dev knob) 1.29 / 2.0 ms, every
primitive uniformly ~2x its bench cost (`_bob_step` 6.4 vs 3.0 us, the casadi call 0.10 vs 0.05 ms) while
rclpy's own costs measured tiny (`get_parameter` 0.4 us -- Humble caches; two publishes 26 us; the GIL
switch interval no effect). The Ryzen's `schedutil` governor clocks a callback thread that runs 1.3 ms in
every 5 ms DOWN (2.2-2.9 GHz of 4.9) and the bench's busy loop UP. Proof and remedy: **`CASC_SPIN=poll`
(a polling executor with a 200 us bounded `rcl_wait`) reads 0.58 / 0.89 ms on the same stream and the
outer solve 17 -> 10 ms** -- one core kept awake (105% of a core; `busy`, a zero-wait poll, is the same tick
at 134% and let one `ros2 param set` time out in HIL, so `poll` is the mode). In the loaded HIL stack the
gain is smaller (the machine is busy, so the clock was already up): **0.83 / 1.34 ms with `poll` vs 1.05 /
1.66** (hover flights 09:10 busy, 09:24 poll; the outer 12.0 / 14.3 ms; the odometry the inner clocks on
arrives at 80 Hz in HIL). Also taken on the way: the two actuator messages built once and filled in place
(0.1 ms), the IMU node's dead-reckoning sample test without `np.allclose` (40 us, the same decision), and
the IMU node attaches its subscription through `base.SPIN_HOOKS` instead of monkeypatching `rclpy.spin`.
**For the Orin this is `nvpmodel -m 0 && jetson_clocks` (the plan already says so) and/or `CASC_SPIN=poll`;
the projection is then 1.2 ms median / 1.8 p95 of the 5 ms tick -- inside the pass line with room, not on
it.** Further remedies in order if its `ims=` still reads over 3 ms: 150 Hz (a 6.7 ms tick), then a compiled
EKF step / aero call (0.1 ms each here). The outer's line stands (~24 ms p95 of 50 predicted; `outer_hz` 15
is the fallback). `tests/test_bench_budget.py` holds the desktop bench floor (the bench runs at full clock:
on a pinned-clock machine it is the right reference, on a scaled one `ims=` is); the Orin's own table (the
bench AND `ims=` from its first hover bag) goes here when it exists. The launchers forward `CASC_SPIN` to
both cascade panes (default: `rclpy.spin`, the flown executor).
**THE TICK WAS 1.32 ms BEFORE THIS PIECE, AND 70% OF IT WAS NOT CONTROL:** cProfile on the tick: (1) the
predictor's `predict(tau)` evaluated the slosh force at EVERY substep (an `np.r_` 9-vector and `_force_at`
each) although only the last is read unless a trajectory is asked -- and it was called TWICE per tick, the
second time for `PRED_TAU_LOG` = +0.25 s (50 substeps) whose only reader is the gate's `--log` rows /
`--pred_score`; (2) `np.cross` on 3-vectors costs ~30 us a call in numpy 2.x (it is written for arrays) and
the tick made a dozen: `B_rot` 4, `model_moment`'s gyroscopic term, the observer's gyro term, the tank arm's
rotation terms 3, the moment of the slosh force 2-3. Now: `predict()` integrates and evaluates the force once
at the end (per substep only for `trajectory=True`); the +0.25 s prediction runs only when `LOG_PATH` or
`--pred_score` (both copies, steso and imu, the same line); `cross3` (nmpc_cascade_node, slosh_predictor,
so3_steso each carry the 3-line helper) computes a x b with np.cross's OWN multiplies and subtractions in the
same order, so the result is bit-identical INCLUDING signed zeros (`%+.2f` prints -0.00 for a -0.0 -- a
reordered formula would have changed status lines). 1.32 -> 0.41 ms. **Identity, as the process demands:**
the flown gate line's trace hash `ef8e90e4e5ef6afbe2935ba81fa4bbe0` unchanged (the steso node, `--log`
on, so its rows still carry the +0.25 s prediction); the IMU node's own line (`--noise_acc 0.02`, otherwise
the same) `cd328f91e6a0e7e358cb934885c82576` before and after; `test_residual_estimators.py` (the three
STESO trace hashes as exact bytes, 64 checks), `test_slosh_predictor.py`, `test_est_swap.py` (pwc = the tank
node), `test_imu_node.py`, `test_cascade_structure.py` and the boot variants all pass. Not touched: the
EKF's `step()` (its 14x14 products are 0.1 ms and re-associating them would move bits), the aero call.
**WHAT MUST CHANGE IN PX4 FOR 200 Hz ON THE ORIN (facts from the tree at ~/PX4-Autopilot, v1.16):**
(1) the uXRCE-DDS client polls EVERY sent topic at `UXRCE_DEFAULT_POLL_RATE` = 10 ms
(`src/modules/uxrce_dds_client/dds_topics.h.em:33`, applied by `orb_set_interval` at line 101) -- a
compile-time constant, not a parameter: 100 Hz is the ceiling on DDS for `sensor_combined`,
`vehicle_odometry`, `vehicle_attitude`, `airspeed_validated` alike (the 83-87 Hz seen in every HIL bag is
that interval plus the agent's cadence). Set it to 5 (or 4) and rebuild the firmware for the Orin link; the
per-topic entries in `dds_topics.yaml` carry no rate field in this version. (2) `EKF2_PREDICT_US` 10000 ->
5000 (min 1000; `vehicle_odometry` is published at the prediction rate, so it is what sets the odometry rate
the `state_source: px4` loop clocks on). (3) `IMU_INTEG_RATE` stays 200 (its default; 100/200/250/400,
reboot required) so one EKF prediction is one integrated IMU sample; `sensor_combined` then arrives at 200 Hz
once (1) is lifted. Bench the DDS round trip on the Orin with the agent on the Orin and PX4 on Ethernet
(`MicroXRCEAgent udp4 -p 8888`, PX4 `UXRCE_DDS_AG_IP` = the Orin): the plan's pass is < 5 ms.
**THE ORIN BUILD, as the plan lists it and nothing here contradicts:** `pip install casadi` has aarch64
wheels (3.6+), the nmpc6 JIT compiles with the Orin's own gcc on the first launch (~2 min here, expect ~4
there; the key hashes the model and the airframe digest; the build and the load are under the file lock of
piece 3, so a launcher that starts both cascades at once is safe -- the locks came with pieces 3 and 4), `nvpmodel -m 0` + `jetson_clocks`, the
outer thread and the inner loop on separate cores (`taskset`; the bench's `--pin` is the desktop stand-in).
Clock the inner loop on the odometry (or IMU) callback, not a 120 Hz mocap. If the Orin bench reads more
than ~2 ms for the tick, the next 0.1 ms each are the EKF step and the CasADi aero call (a numba path for
`step()` would have to reproduce its products in order to stay bit-identical -- verify by the hashes).
**THE BOOT TEST'S `aspd` VARIANT HAD BEEN FAILING SINCE THE CAS SWITCH (piece 4, ea2b082):** the node reads
`calibrated_airspeed_m_s` but logged it as "TAS", and the test still expected the TAS value; both say CAS now
(`first airspeed_validated: CAS 12.40 m/s`). Nothing flown read that line; the launcher's pitot check reads
the status feed's `Vair=`.

# ★★★★ 2026-09-16 (night): HARDWARE-TRANSFER PIECE 5 -- FAILSAFES, INTERLOCKS, THE STAGED ENVELOPE, THE
# LATENCY DOUBLET. Controller piece5 branch (merged: 49a5824 + 4db3e4c), sim 0b1ed3e+. The flight card:
# https://claude.ai/artifact/5cr6taszbTBBa6p1T1xCD8
**THE HARDWARE PREFLIGHT.** `HW=1` on the launcher skips the 8-relax window entirely (on an aircraft every
parameter it sets is a failsafe that must stay armed) and `start_slosh_hil.sh` runs `scripts/preflight_hw.py`:
PX4's parameters read back over MAVLink (`udpin:0.0.0.0:14550`, `PREFLIGHT_URL` overrides) against
`scripts/preflight_hw.cfg` -- one line per parameter, `NAME op value [optional] # why`, the flight lead's file:
the battery action (2/3) and threshold, disarm-on-land / preflight, the COM_ARM_EKF/IMU/MAG gates at or under
their defaults, the geofence (action >= 1, a site-sized horizontal fence, GF_PREDICT 1), offboard loss (<= 1 s,
Return/Land/Hold -- never a manual mode on an airframe whose CA is best-effort), RC loss, a mapped kill switch,
SYS_HAS_NUM_ASPD 1, no circuit breakers; `SIM_BAT_MIN_PCT` present = a simulator = refused. One refused line = not
READY. `tests/test_preflight_hw.py`: the armed map passes, each HIL relaxation ALONE is refused and named, a sim
parameter present is refused, a missing required one is refused, and the live HIL stack (when up) is refused.
**THE NODE'S INTERLOCKS (every one a live parameter; the flown behaviour is the default except the first two):**
`engage_v_max` 1 m/s + `engage_att_max_deg` 10 -- a controller switch at speed or banked is REFUSED (logged,
rate-limited, `engage_refused=1` on the status feed; the GS/driver re-ask at 1 Hz so a hover engages the moment
it is one); `state_stale_s` 0.2 -- no state sample while active -> the slow timer publishes `safe_hover` (rotors
up) until the state returns, once logged; `settle_sigma` (N.m, off) -- with the L1 on, the transition waits until
u_ad has moved less than this over 1 s (piece 2: the cold-start CG sweeps crash at -1.5 cm where a hover with the
L1 converged flies 2.6 cm: start from a CONVERGED estimate); `tailwind_max` (m/s, off) -- with a wind estimate,
no transition downwind (piece 4's every-seed crash; a pitot in hover cannot see a tailwind, so this catches an
estimate carried from before -- the card's 180 deg wind check is the rest); `v_cap` (m/s, -1 = off, 0 = hold
the hover, a lowered cap RAMPS the reference down at decel_rate: the first staged flight stepped 16 -> 3 m/s and
pitched +38 deg). The safe controller's `radius_local_s` (s, off): the local envelope ball grows by it x |v|
(the 34 m/s runaway of 2026-09-07 outran a 100 m ball). Status feed: `vcmd= tilt= fails= settled= stale=
doublet=`. Boot test `--node interlock`: a mocap sample at 3 m/s then the switch -> refused; a hover sample ->
ENGAGED; 2 s without state -> STATE STALE. The boot test itself now REFUSES to run while a stack is live.
**THE ELEVATOR DOUBLET (the latency measurement piece 1 could not make):** `doublet A T` on the cascade's cmd
topic (wing-borne only, |A| <= 0.5, T <= 2 s): +A for T then -A for T added to the inner's elevator AFTER the
allocation, `doublet=1` on the status feed while it runs. The gate's `--doublet T0,A,T` measures the elevator ->
pitch-acceleration delay from the cross-correlation of the DERIVATIVES (a doublet's edges are impulses; the slow
terms fall out; |corr| because +de is nose-down): **35 ms at a 30 ms transport, 65 at 60** -- the transport to
5 ms. `casc_timing.py` does the same on a bag from the `doublet=1` window (the closed-loop correlation of a flown
trim was never a measurement: 0.1-0.3).
**THE STAGED DRIVER:** `fly_transition.py --stage hover|tilt10|tilt50|cruise|mission` (`STAGE=` / `STAGE_HOLD=` /
`DOUBLET="0.3 0.5"` through `fly_slosh_mission.sh`, `stage=` in the results row): after the engage, `v_cap` 0 (the
hover held, 15 s), 3 m/s (tilt ~15 deg), 8 m/s (~45 deg, the wing at 30%), then the cap off and the cruise for
27 s with the doublet 20 s in -- each stage advances only if |roll| < 15, |pitch| < 20, altitude +- 3 m, <= 3 new
solve failures, no stale state and the speed inside the stage's window held for the whole hold; a no-go aborts
through the schedule's back-transition and lands. A stage short of `mission` comes back the same way.
**THE STAGED FLIGHT (row R100_dx3_staged_a1_011843, bag slosh_20260916_011703; `STAGE=cruise DOUBLET="0.3 0.5"` on
the 1b configuration with the pitot, still air, the offset 100 mm tank): LANDED 0.00 m FROM THE COMMANDED POINT.**
hover (v_cap 0, 15 s: go -- the engage hover creeps forward 1.1 -> 2.4 m/s at tilt 0-3.5 deg, the outer's known
drift with a zero speed reference), tilt10 (cap 3: go -- V 3.7-4.9, tilt 10-17 deg, the aircraft runs ~1.5 m/s
above a capped reference), tilt50 (cap 8: go -- V 8.7-9.8, tilt 33-41 deg, the wing at 29-36%, AoA 14 deg), cruise
(cap off: 16.2-16.3 m/s at tilt 90 within 6 s, wing 113%; the DOUBLET 20 s in), then the schedule's back-transition
and the landing. Three launches before it: v_cap 0 meant "no cap" (fixed: negative = off), the cap arrived after
the node had settled (set before the engage now), a lowered cap stepped the reference (it ramps now).
**THE LATENCY, MEASURED AT LAST:** `casc_timing.py` on the bag's `doublet=1` window (the derivatives of the total
elevator command and of the pitch acceleration, a 25 ms box on both because a 120 Hz rate's second derivative is
noise otherwise): **50 ms vs the mocap (|corr| 0.57), 55 ms vs EKF2's odometry (0.55)** -- the command-to-effect
delay of the HIL chain on the physical path and on the state path the loop flies. `latency_s` 0.049 stands (the
plan: move it only past 60 ms). The closed-loop correlation on the same bag reads 300 ms at |corr| 0.03-0.10,
which is why the doublet exists.

# ★★★★ 2026-09-15/16 (night): HARDWARE-TRANSFER PIECE 4 -- AIRSPEED AND WIND. Controller (piece4 branch,
# merged), sim efbf5a2+, GS 96a1fb3. Sweep `~/flight_bags/wind/wind1` (`scripts/wind_sweep.sh`, `wind_metrics.py`).
**WHAT WAS BUILT.** Plant: `VTOL_WIND="wx,wy,wz"` (a steady ENU wind) and `VTOL_TURB=sigma` / `VTOL_TURB_TAU`
(a Gauss-Markov gust per axis) in `vtol.py` -- the aero's `wind_body`, the momentum prop's air-relative inflow,
`state.wind_world` for the mavlink backend's synthesised pitot (air-relative now); the gate's `--wind/--turb/
--airspeed/--noise_aspd/--aspd_hz` (`so3.build_ode(wind=True)` takes a third input; w = 0 bit-identical).
Controller: `WindEstimator` (the HORIZONTAL wind triangle from PX4's `airspeed_validated` TAS: w = v_h -
sqrt(TAS^2 - vz^2) heading, tau 2 s, frozen below 2 m/s of indicated airspeed -- a pitot cannot see a vertical
wind and a vertical estimate would read the angle of attack as wind); `airspeed_source: none|px4` (a string,
never through gp()); **the outer runs in the AIR-RELATIVE along-course frame** -- with a steady wind
d(vs - w_s)/dt = vsdot, so no ODE parameter and no JIT rebuild: it is posted vs - w_s, its references are
built at v_cmd - w_s and the cruise target is Vc + w_s OVER THE GROUND (the wing sees Vc of air: 11 m/s over the
ground into a 5 m/s headwind, 21 with it), while the speed COMMAND and the RTH stop stay ground quantities (a
stop in wind is a ground-speed stop, hovering at V_air = -w_s, leaning into it); the inner's wing share,
surface authority, prop inflow and the L1's model force/moment on v_air; the guidance CRABS its heading into
the crosswind (asin(w_cross / V_air), faded with the wing share), sizes the orbit for its tailwind sector
((V_air + |w|)^2 / g tan(phi_max) / kappa x 1.15: an R 100 orbit is INFEASIBLE in a 5 m/s wind through
kappa 0.8 and flies at R 139), and keeps atan(V_g^2 / g R) as the orbit's bank feed-forward (derived: holding
a GROUND circle in wind needs chi_dot + the crab's rate = V_g^2 / (R V_air) of heading rate -- V_g V_air, the
first "fix", asked 19 deg where 24 was needed and left the orbit 130 m wide). The status feed carries
`stopped=1` (v_cmd at zero, rotors upright) and `Vair= w=(..)`; `fly_transition.py` and the GS's RTH phase hand
over to the safe controller on it -- in wind the cascade's ground speed after the stop is the crosswind's
drift (its lateral channel is a bank-to-turn law with no wing to bank), so "V < 3 m/s" never comes. Launchers:
`VTOL_WIND/VTOL_TURB` to the Isaac pane, `CASC_ASPD=px4` on both cascade panes, READY checks
`airspeed_validated`'s rate, the "airspeed source: px4" banner and Isaac's "[VTOL] wind:" line; provenance
`aspd= wind= turb=`; boot test `--node aspd`. **Without a pitot every line is byte-identical** (the flown gate
line's hash `ef8e90e4e5ef6afbe2935ba81fa4bbe0` through every step; `tests/test_wind_frames.py`, 25 checks).
**WHAT THE WIND FOUND (all on the gate, seed 1, the turning mission, 100 mm tank at dx 0.3, imu-mdob):**
1. **The flown controller, blind to the wind, crashes at t = 5 s of the transition in a 5 m/s headwind** --
   the surface authority is scheduled on GROUND speed, so at 1 m/s over the ground it believes the elevator
   makes nothing while the plant's elevator at 6 m/s of air makes 1.2 N.m per full deflection; the L1 reads the
   unmodelled moment as a residual and cancels it the wrong way, the elevator saturates, pitch +33 deg. With
   the pitot the same case flies (V_air held at 16.0 in cruise, front throttle 0.82 in the tailwind sector:
   the prop sees 16 m/s of air whatever the ground does -- the "tailwind headroom" of the plan was a
   ground-speed misreading). A pitot gated on GROUND speed (the first estimator) froze through exactly that
   phase and crashed identically: gate on what the sensor can see (IAS >= 2 m/s), not on what the vehicle does.
2. **A tailwind on the FIRST leg (the transition flown downwind) crashes at t = 8 s with or without the pitot:**
   the pitot reads nothing in reversed flow, and the plant's elevator in 5 m/s of reversed flow makes an
   unmodelled moment. Not chased: the operational rule is TRANSITION INTO THE WIND (a piece-5 interlock; a
   pitot in hover cannot tell still air from a tailwind -- a 180 deg yaw in hover reads it).
3. **The bank-to-turn law in a crosswind:** without the crab a 5 m/s crosswind leaves a straight leg flown with
   10 deg of sideslip and 12 deg of standing bank (the law holds a HEADING; the ground track is held by the
   fin's side force), and the orbit's bank demand then sits on the 25 deg limit.
4. **Turbulence** 0.3 / 0.5 / 1.0 m/s 1-sigma per axis (tau 1 s): flown and stopped (1 / 1 / 14 solve fails,
   stop 6 / 11 / 17 m) -- a 1 m/s vertical gust at 16 m/s is 3.6 deg of AoA on a wing that stalls at 13,
   i.e. "light" turbulence for this wing loading is genuinely rough.
5. **The hover end in a quartering wind** (wind1 head5 seed 2): with v_cmd at zero and V_air falling to 5, the
   crab angle asin(w_cross / V_air) ran to its 30 deg clamp, the yaw loop swung the nose with the rotors
   tilted -20 deg aft, and the aircraft departed one second after `stopped=1`. The crab now fades with the
   wing share (V_air / V_stall)^2: nothing to crab in a hover.
**★ THE JIT CACHE RACE, TWICE:** six processes compiling one fresh key at once poisoned every solver (the build
lock, piece 3's record); then sixteen LOADING one cache at once bus-errored one of them at start (the
"core-dumped two at start" of 2026-09-13, never understood): `jit_serialize: embed` extracts the embedded
library to a temp file on load. Both are under one exclusive file lock per key now (nmpc6 0acf371 + 99bd727);
`wind_sweep.sh` re-runs a seed that produced no summary, once.
**THE WIND TABLE (16 seeds per cell, the turning mission, 100 mm tank at dx 0.3, imu-mdob WITH the pitot;
`wind1` + the four cells re-run after the crab fade as `wind2`; landed = the RTH stop reached and no ground):**
| cell | landed | solve fails / seed | corridor pk | V_air min (stall 15.12) | orbit rho-R last 40 s | stop from home | wind estimate |
|---|---|---|---|---|---|---|---|
| still air | 16/16 | 1 | 11.6 | 16.01 | -0.2 +- 0.1 | 6.3 m | (-0.1,+0.1) |
| headwind 3 (leg 1) | 16/16 | 0 | 11.7 | 16.02 | -0.1 +- 2.5 | 3.3 | (-3.4,-0.2) |
| headwind 5 | 16/16 | 8 (all at the stop) | 11.7 | 16.01 | +5.6 +- 16 (R 139) | 22-24 | (-4.7,-0.4) |
| crosswind +3 | 16/16 | 0 | 11.9 | 16.02 | +1.4 +- 4.6 | 4.5 | (+0.0,+3.2) |
| crosswind +5 | 16/16 | 0 | 12.5 | 15.98 | +2.7 +- 4.6 | 9.9 | (+0.2,+5.0) |
| crosswind -5 | 16/16 | 11 (the stop) | 12.5 | 15.88 | -0.1 +- 2.3 | 20 | (-0.5,-4.6) |
| diagonal 5 (-3.5,+3.5) | 16/16 | 0 | 12.3 | 15.95 | -0.6 +- 7.9 | 4.2 | (-3.5,+3.5) |
| turbulence 0.5 m/s | 15/16 | 6 | 87 (the one stall) | 10.5 | +0.3 +- 1.1 | 5.7 | -- |
| turbulence 1.0 m/s | **2/16** | 84 | -- | -- | -- | -- | -- |
| headwind 5 + turbulence 0.5 | 11/16 | 32 | -- | -- | -- | -- | -- |
**READING.** (1) A STEADY wind to 5 m/s (31% of the cruise airspeed) from any direction but dead behind on the
transition leg is flown and stopped by every seed, with the wing at 16.0 +- 0.1 m/s of AIR throughout, the
front throttle 0.83-0.84 in every sector, the wind estimate within 0.5 m/s and the orbit within +-5 m at 3 m/s,
+-16 m at 5 (where it flies at R 139, the tailwind sector's minimum through kappa). The stops in a tailwind /
quartering wind end 20-24 m from home (the ground-speed ramp against an air-relative trim, then the drift the
safe controller takes over) and cost 5-10 transient solve failures per seed at the hover end -- the plan's
"stop <= 10 m" is not met there. (2) TURBULENCE is the envelope: 0.5 m/s 1-sigma per axis costs one seed in
sixteen (a stall at the end of the transition: +11 deg of AoA plus a 1.5 m/s vertical gust = 16 deg on a wing
that breaks at 13), 1 m/s costs fourteen, in every phase -- the cruise AoA of 7 deg leaves 6 deg = 1.7 m/s of
vertical gust to the stall. That is this wing loading (110 N/m^2) at 16 m/s, not the controller: a faster
transition (ttr 8: peak AoA 7.9 vs 9.4-10.3 at 10) and a cruise at 17-18 m/s (the 12x8 has the headroom) are
the two knobs, both untested in wind. (3) The plan's piece-4 pass line ("16/16 at every wind incl. 1 m/s
turbulence, orbit +-1 m, stop <= 10 m") is therefore NOT met as written; what is met: 16/16 at every steady
wind, V_air >= stall + 0.5 through cruise and orbit, the corridor peak within +1 deg of still air, landing at
home.
**THE CROSSWIND FLIGHT (row R100_dx3_wind4y_a1_004009, bag slosh_20260916_003830; `VTOL_WIND=0,4,0 CASC_ASPD=px4`
on the 1b configuration -- EKF2 on GPS 10 Hz + IMU + baro, imu-mdob, the offset 100 mm tank): LANDED 0.17 m FROM
HOME.** The airspeed held at 16.02-16.08 m/s of air through legs, turns, orbit and return while the ground speed ran
12.2-20.3 with the heading (w_s -3.9..+4.2 in the cascade's own log, the wind triangle's along-heading component), 0
solve failures in 3794, z 29.8-30.1 wing-borne, both fly-by turns at the 25 deg limit (overshoot +40 / +22 m; still
air +14), the orbit flown at its wind-enlarged R 139 (steady 139.5 +- 7.5, entry +-11), the RTH stop 13.8 m from
home in 7.7 s from V 12, then the safe controller's leg and the landing. **THE FIRST ATTEMPT CRASHED IN THE
TRANSITION (row ..001154) AND FOUND A BUG THAT PREDATES THIS BRANCH:** Pegasus's
`State.get_linear_body_velocity_ned_frd()` returned the body-frame linear ACCELERATION (an upstream copy of the
acceleration accessor), so the synthesised pitot read 0.0 at 16 m/s on EVERY flight since it was written (checked
on the gps10 bag: IAS -0.04..0.12 at 16.05 m/s) and saw only the wind term; the wind estimator therefore read a
7 m/s TAILWIND at 12 m/s of ground speed in a pure crosswind, the outer flew a 5 m/s schedule at 12 m/s of air and
departed. Fixed in `state.py` (+ pinned in `test_wind_frames.py`); the node now reads `calibrated_airspeed_m_s`
(CAS = the speed a rho0 = 1.225 aero model needs; PX4's TAS carries a baro density correction the sim's aero does
not have: 8.65 for 8.00 at rest in an 8 m/s headwind). Two launcher lessons on the way: Isaac's once-only
"[VTOL] wind:" line must be read from the EARLY pane capture (PEGASUS_SLOSH_DEBUG pushes it out of the history by
READY), and `tests/test_cascade_node_boots.py` now REFUSES to run while a stack is live -- it boots a node under
the live node's name and engaged a second cascade on the stack during a boot (a second "ekf2 missing data" arming
refusal followed).

# ★★★★ 2026-09-15 (late night): HARDWARE-TRANSFER PIECES 2 AND 3 -- THE MODEL-ERROR ENVELOPE MEASURED
# (48 cells x 16 seeds), AND ONE AIRFRAME FILE. Controller 45355a9 (merged), sim this commit.
**PIECE 2 -- THE TOLERANCE TABLE (`~/flight_bags/robust/env1`, `scripts/robust_sweep.sh env1` 20:33-22:0x,
`scripts/robust_metrics.py`; the flown line: steso node `--est mdob --l1 --ttr 10 --lat 0.03 --comp 0.049 --slosh`,
CG tank 0.502 kg, straight mission with the back-transition, y0 10 m, noise 0.1 deg / 1 deg/s, 16 seeds per cell;
the PLANT perturbed, the controller untouched -- `tests/test_plant_knobs.py`). PASS = 16/16 landed, solve-fails
sum <= 4, corridor peak <= nominal + 3 deg, z >= 29.0, stop <= nominal + 3 s. Nominal: 16/16, pk 14.3, fails 3,
minz 29.9, stop 8.8 s.**
| parameter (plant x model) | passes | fails (why) |
|---|---|---|
| mass | 0.90 .. 1.20 (all) | -- |
| inertia (all axes) | 0.8 .. 1.6 | 0.6: 1/16 landed, 12 crashed before the BT (the attitude loop 1.7x too hot) |
| cQ (yaw torque per N) | 0.06 .. 0.18 (all) | -- |
| lift (CL0, CLa, CLq, CLde) | 0.70, 0.85 | 1.15 (6 fails, pk 17.4), 1.30 (5 fails, pk 19.7): MORE lift than modelled breaks the trim |
| pitch (Cm0, Cma, Cmq) | 0.8, 1.25 | 0.6 (pk 18.1), 1.5 (9 fails) |
| ctrl (Cmde, Clda, Cndr, ..) | 1.25 | 0.6 (0/16, all before the BT), 0.8 (9 fails), 1.5 (pk 18.6) |
| lat (Clb, Cyb, Cnb, ..) | 0.5, 1.5 (all) | -- |
| drag (CD0, k) | 0.7, 1.4 (all) | -- |
| thrust scale | 0.80 .. 1.20 (all) | -- |
| prop J0 (zero-thrust advance ratio) | 0.60, 0.66, 0.80 | 0.90 (13 fails, pk 17.6) |
| CG error, cold start | +1.5 cm | -1.5 / -3.0 cm AFT: 0/16 (all before the BT); +3.0: 12/16; +4.5: 1/16 |
| rotor spin lag (model 0.12 s) | 0.06 | 0.20: 1/16, 0.30: 0/16 -- the L1 predictor's lag model is 0.12 |
| surface servo lag: SEE THE 2026-09-16 EVENING BLOCK -- these two rows were re-run paired (told/not told); the spin cliff is bracketed at 0.16-0.18 and the surface row was never a mismatch test |
| surface servo lag (model 0.03 s) | 0.10 | 0.06 (5 fails: marginal) |
**THE SPEC IT WRITES:** mass and thrust are free to +-20% (the loops absorb them; the trim's vertical channel is
closed), yaw torque and drag do not matter, lateral derivatives +-50% do not matter. What must be measured:
(1) the LIFT slope and the pitch stiffness within ~-20/+15% (the outer's trim is a package built on them; too much
lift is worse than too little); (2) the control derivatives within ~-10/+25%; (3) the ROTOR SPIN-UP LAG -- the
one actuator constant with a hard cliff (0.2 s crashes 15/16 where 0.12 is modelled; a thrust-stand step response,
not a datasheet); (4) J0 to +-0.1; (5) the CG: a cold-start transition tolerates +1.5 cm forward and NO aft error
-- the engage HOVER (settle_s with the L1 converged; the HIL 2.6 cm flights) is what buys the 2-5 cm the plan
quoted, so the piece-5 interlock must refuse the transition until the L1's sigma has settled; (6) inertia to
-20/+60%. `robust_metrics.py` now counts a seed that crashed BEFORE the back-transition as not landed (ke_metrics
prints "no BT rows" for it; a cell with all 16 so printed MISSING and hid the worst rows of the sweep).
**★ THE JIT RACE (found by piece 4's first sweep, fixed in 45355a9's successor):** six gate processes started on a
FRESH cache key compiled the solver at once in one working directory; the shell compiler's temporaries collided and
EVERY process ran a poisoned solver (ok=0 on every solve at 130 ms, the aircraft on the ground at t=9 s in STILL
AIR) while the cache one of them saved was fine. A 16-seed sweep on a fresh key (a new mass mode, an airframe-file
edit) does the same, silently. nmpc6's cache build is now under an exclusive file lock per key (the first process
builds and saves atomically, the others wait and load) with `jit_temp_suffix`; three processes racing on a fresh
key: one built, two loaded, all 0/161.
**PIECE 3 -- ONE AIRFRAME FILE (controller branch `piece3` -> merged; sim this commit).** Every airframe number
the controllers carried as a literal now lives in `px4_vtol_direct_actuator/airframe_id.cfg` (sections mass /
rotors / tilt / surfaces / aero: the assembled mass, CG, inertia, the four hub positions, spin, tiltable, cQ, the
12x8 propeller, omega_max, the actuator lags and rates, the tilt range, and the full aero coefficient set incl.
the stall/sideslip blends) and is read by `airframe_id.py` (`AIRFRAME`, `_AF` in every node; numpy +
configparser only, so the GS python, the tests and the launcher can load it; `FSC_AIRFRAME_ID=/path.cfg`
overrides; unknown or missing keys are an ERROR -- a typo must not silently keep a default; `.digest()` is a
hash of the file). Rewired: so3 (M, CG_X0, IXX0.., MAX_THRUST, RPOS = hubs - CG, SPIN, TILTABLE, CQ, TAU_BETA,
TILT_RATE, PROP_D/J0/OMEGA_MAX, every aero constant), the cascade node (PROP_*, BETA_AFT_MAX = -tilt.min_deg,
the predictor's default actuator lags, `l1_tau_spin/surf`, `tilt_slew_deg_s`, the prop parameters and every
gate flag default: `--prop_D/J0/omega_max/tau_spin/surf_tau/surf_rate/plant_tilt_min`), the steso node's
L1Steso defaults, the safe controller (mass, max_thrust, cg_x, cQ), vtol_nmpc (all of them) and the trim
generator (m, rho, S, CL0, CL_alpha, CD0, k). `setup.py` ships the cfg as package data. **PROOFS:** (1) every
numeric module constant in so3 / vtol_nmpc / the safe controller's Gains / the trim generator's FSC dict is
IDENTICAL before and after (227 rows diffed); (2) the flown gate line's trace hash from the worktree equals the
main tree's `ef8e90e4e5ef6afbe2935ba81fa4bbe0` (the swap-identity check), `--est pwc` still equals the tank node
trace-for-trace, the IMU copy runs; (3) `test_tank_model_pairing.py` now resolves the hub's `float(_AF.mass.*)`
expressions against the loaded file, pins the loader form of each copy as text (`_AF.mass.airframe_kg + VTOL_TANK_KG`,
`_AF.mass.cg_x + VTOL_TANK_DX ...`), pins the GS's literal `M_KG = 3.449572` numerically to the file, and its
NEW SECTION 8 edits the mass +5% through `FSC_AIRFRAME_ID` in a subprocess: so3 3.4496 -> 3.6221, vtol_nmpc /
safe controller / trim generator 3.9516 -> 4.1241 together, the hover trim 38.76 -> 40.45 N (1.0437x = the mass
ratio), and the pairing itself then REFUSES the file (MASS MISMATCH) -- one file, every copy, and the plant
notices an edit; `test_prop_pairing.py` / `test_tilt_limit_pairing.py` pin the `_AF` text of every prop / tilt
site and take their numbers from the file; `controller_path.load_airframe()` is the sim-side loader.
**JIT CACHE KEYS** (nmpc6/cascade, nmpc_full, vtol_nmpc, the so3 node) hash `_AF.digest()` on top of the values
they listed: the listed subset never covered CLQ / CD0 / CMQ / S / rho, which the file can now change without a
source edit -- one rebuild (~2 min) per cfg edit, never a stale solver. **OPS:** `start_slosh_hil.sh` loads the
file through the controller's own loader (no `3.449572` / `0.176` literals left in it), checks each cascade
banner's `M=` / `CG_X=` against the FILE + the water, and REQUIRES the banner's new `af=<digest>` to equal the
file it read (the node loaded the same airframe the launcher verified); the safe controller's and the base
cascade's "state source" lines carry `af=` too; provenance `af=` in the launch log and the results row
(`fly_slosh_mission.sh` reads it from the LAUNCH_LOG line). The plant double (`tests/plant_double.py`) keeps
its OWN copy of the propeller and mass properties on purpose -- it is the sim's side of the pairing.
**WHAT THE FILE IS NOT:** a measured airframe. Its numbers are the sim's (assembled from the CAD config); the
hardware campaign replaces them section by section (mass/CG/inertia from the bifilar-pendulum + scale, the
propeller from a thrust stand, the aero from the wind tunnel or a glide-test fit), and section 8 is the check
that an edit reaches every copy. `tests/plant_double.py` and the GS's `M_KG` are the two remaining literal
copies (the plant side, by design; the GS pinned numerically).

# ★★★★ 2026-09-15 (night): THE HARDWARE-TRANSFER PLAN, AND PIECE 1 FLOWN -- THE LOOP FLIES ON AN INS
# WITH NO MOTION CAPTURE ANYWHERE (GPS 10 Hz + IMU + baro + magnetometer -> EKF2 -> both controllers).
# Plan (7 pieces, gates, pass criteria): https://claude.ai/artifact/ATaE52LWqS8aeithSpskdF
# Target: NVIDIA Jetson Orin companion, PX4 over Ethernet (uXRCE-DDS/UDP), inner loop 150-200 Hz.
**THREE FLIGHTS, ALL LANDED AT HOME** (rows R100_dx3_px4src_a1_193130 / _gps10_a1_195322 / _nomocap_a1_200800;
bags ..193027 / ..195155 / ..200633), 100 mm tank 0.3 m ahead, turning mission, `cascade_adaptive` + `CASC_EST=mdob`
+ the IMU node (`IMU_DR=1` on the GPS flights), 0 solve failures, z 29.8-30.3:
| flight | state source | EKF2 fed by | orbit rho / roll sd | lat. bob orbit | fore-aft pred corr / err | wall spikes | landing | EKF2 vs mocap (rms pos / vel / att) |
|---|---|---|---|---|---|---|---|---|
| 1a `CASC_STATE=px4` | EKF2 odometry | mocap bridge (EV) | 99.8 +- 0.31 / 0.83 | 0.03 N | 0.94 / 0.05 | 0 | 0.00 m | 0.74 m (its 35 ms) / 0.11 / 0.34 deg |
| 1b `+EKF_SOURCE=gps VTOL_GPS_HZ=10` | EKF2 odometry | GPS 10 Hz + IMU + baro, vision yaw | 99.9 +- 0.91 / 1.00 | 0.04 | 0.98 / 0.06 | 0 | 0.00 m | 1.11 m (0.57/0.58 offset = the receiver) / 0.04 / 0.20 |
| 1c `+EKF_SOURCE=gpsmag EV_BRIDGE=0 GEO_STATE=px4` | EKF2, BOTH controllers | GPS + IMU + baro + MAG, no vision | 99.9 +- 0.98 / 0.99 | 0.03 | 0.94 / 0.15 | 0 | **0.04 m** | 1.12 m / 0.06 / 0.66 |
The only visible cost of a GPS-class source is the orbit radius sd 0.3 -> 0.9 m (the receiver's wander in
EKF2's position); roll sd, the cancellation and the landing are the mocap's. The fore-aft prediction on the
px4 source (0.94-0.98) is better than the mocap+IMU flight's (0.36-0.61) -- noted, cause not established
(the x-bob level differs run to run). **THE SIM GPS HAD TO BECOME A REAL RECEIVER FIRST:** Pegasus's
Gazebo defaults (random walk 2.0 (m/s)/sqrt(Hz) = 6 cm per sample at 10 Hz) and a white 0.8 m/sample
"realistic" noise BOTH fail EKF2's GPS drift check (`Preflight: GPS Horizontal Pos Drift too high`, no
vehicle_global_position, READY refused); a receiver is centimetres of jitter plus a slow wander:
`VTOL_GPS_SIGMA=0.05 VTOL_GPS_VSIGMA=0.05 VTOL_GPS_WALK=0.05` (knobs in `vtol.py`'s sensor factory,
forwarded by both launchers). **PX4 SITL PERSISTS PARAMETERS** (`build/.../parameters.bson`): a previous
run's `EKF2_EV_CTRL 11 / GPS_CTRL 0` survived the restart, so each `EKF_SOURCE` (ev | gps | gpsmag) now
sets its FULL EKF2 set explicitly and restarts ekf2 (`start_vtol_smc_hil.sh` 8-relax). `GEO_STATE=px4`
puts the safe controller on `odom_to_state` too (controller c200a9e; boot check in `test_state_source.py`
section 3: subscribes to the odometry and NOT the mocap); READY verifies the GPS rate, the odometry rate,
`vehicle_global_position` on GPS, and the "state source: px4" line in every controller pane (from the
WHOLE scrollback -- 200 lines is 100 s of the safe controller's status). Provenance: `state= geo_state= ekf=
gps_hz= gps_sigma=` in the launch log and the results row; the row's model label now comes from the
launch log (a 3000-line pane grep mislabelled two rows `baseline_VTOL`; annotated in results.tsv).
**LATENCY: NOT MEASURED YET.** `casc_timing.py` now follows the cascade family and adds an
elevator -> pitch-acceleration estimate, but in CLOSED LOOP its cross-correlation is 0.1-0.3 and the peak
wanders 45-300 ms -- flagged "not a measurement". The way to measure it is an injected elevator doublet in
a straight leg (piece 5); `latency_s` stays 0.049 (flown on all three sources; the outer's compensation,
and EKF2's 35 ms of extra lag did not show at the aircraft level).
**PROCESS, again:** `pkill -f start_slosh_hil.sh` inside a Bash line that names the pattern killed the
shell (exit 144) mid-sequence -- kills live in a script file (`teardown.sh`), as CLAUDE.md already says.

# ★★★★ 2026-09-15 (evening): THE ESTIMATORS FLOWN (mdob, leso, and the IMU force channel), THE
# ACCELEROMETER WIRED, `state_source: px4` BUILT. Controller 5b4601c / 7f416a3 / (+IMU_FCG), sim
# a07b01e / 350026d / 9bc37e0 / this commit. Flights in `~/flight_bags/slosh` (rows R100_dx3_mdob_a1_165700,
# R100_dx3_leso_a1_171722, R100_dx3_imumdob_a1_174329; bags ..165608 / ..171631 / ..174239).
**THREE FLIGHTS, THREE LANDINGS AT 0.00 m**, 100 mm tank 0.3 m ahead, the turning mission, `CTRL=cascade_adaptive
CASC_EST=<kind>` (the IMU channel via `ADAPTIVE_NODE_PY=.../nmpc_cascade_imu_node.py`), predictor + cancel on, 0 solve
failures, z 29.7-30.3, scored with `ana_guided.py` + `ana_slosh_est.py` + the status feed, against flights 10/11 (STESO):
| flight | force / moment | RTF | orbit rho / roll sd | lat. bob legs/orbit [N] | fore-aft bob | raw lateral reading rms | spurious wall impacts predicted | fore-aft pred corr / err |
|---|---|---|---|---|---|---|---|---|
| f10 | velocity / steso | 0.84 | 99.9 +- 0.40 / 1.26 | 0.07 / 0.08 | 0.10 / 0.16 | 1.3-1.7 N (the clock artefact) | 0 | 0.85 / 0.05-0.08 |
| f11 | velocity / steso | 0.87 | 100.0 +- 0.31 / 1.03 | 0.07 / 0.05 | 0.49 / 0.43 | 1.2-1.4 | 0 | 0.21-0.95 / 0.15-0.53 |
| mdob | velocity / mdob K11 | **0.76** (a gate run overlapped it; orbit 51 s, not comparable) | 99.7 +- 0.87 / -- | 0.05 / 0.06 | 0.10 / 0.12 | **2.4-3.0** | **12** (bob est. to 63 mm) | 0.95 / 0.05-0.06 |
| leso | velocity / leso | 0.84 | 99.9 +- 0.39 / 1.31 | 0.09 / 0.05 | 0.23 / 0.33 | **2.1-2.8** | **13** (to 75 mm) | 0.99 / 0.09 |
| imu-mdob | **IMU / mdob K11** | 0.83 | 99.9 +- 0.40 / 1.14 | 0.04 / 0.06 | 0.25 / 0.55 | **0.10-0.11** | **0** (<= 22 mm) | 0.36-0.61 / 0.28-0.56 |
**READING.** (1) At the aircraft level the three estimators fly the mission identically (orbit 99.9 +- 0.4, roll sd
1.0-1.3 = the campaign band, lateral bob cancelled to 0.04-0.09 N, stop 6-19 m on the wall-clock ramp): the gate's
0.07 deg of orbit roll is invisible in HIL, as the page said. (2) **The linear velocity twins expose a HIL-only
defect the STESO hid:** their full-bandwidth reading carries the clock artefact -(1-RTF) m a at 2-3 N in the orbit
(the STESO twin's slew bound L_F 3 = 12 N/s rate-limits it to ~1.5 N), the RTF wanders 0.80-0.89 WITHIN a flight so
the slow bias state cannot follow, the EKF pushes its bob estimate THROUGH the 50 mm wall (63-75 mm) and predicts
5-16 N impacts that do not exist: 12-13 clipped 1 N.m yaw pulses per flight (0.5% of the status samples). No
measurable harm; a defect nonetheless, and absent on hardware (RTF 1) -- the gate at a CONSTANT RTF 0.86 never
showed it (bF absorbed a constant artefact). (3) **The IMU channel flew and removed it:** raw lateral reading 0.10 N
(25x less), nothing for the bias state (bF_y +0.01 +- 0.04 N), zero wall spikes, the sensor tracks the plant's own
sim-time acceleration to 0.03-0.06 m/s^2 (x/y/z) -- `sensor_combined` at 87 Hz on DDS, FLU hover reads (+0.07,
-0.01, +9.83). **OPEN: its FORE-AFT prediction is worse in flight** (corr 0.36-0.61, err 0.28-0.56 vs the velocity
twins' 0.9-0.99 / 0.05-0.09; the fore-aft bob has no arm, so the cancellation does not use it). Not the predictor's
input: `--imu_fcg model|accel` (the pendulum driven by the parent's model force + estimated reaction instead of the
low-passed accelerometer) is within 0.05 of corr on the gate at the flight's noise (0.06 m/s^2: 0.80 vs 0.74 at RTF
1, 0.89 vs 0.84 at RTF 0.83, sweeps `imuB_{full,rtf}_na06_{accel,model}`), and the gate at a constant RTF 0.83
predicts fore-aft at 0.89. HIL-specific and unresolved; the lateral channel is clean.
**PIECE B -- THE ACCELEROMETER WIRED (controller 5b4601c, sim a07b01e).** `nmpc_cascade_imu_node.py` subscribes to
`/<ns>/fmu/out/sensor_combined` by wrapping `rclpy.spin` (the node class is local to `base.run_ros`; the subscription
is attached to the constructed node before it spins) and writes the FRD->FLU sample `(ax, -ay, -az)` into the SAME
`base.IMU_ACCEL` seam the gate uses, with its receive time; a sample older than 0.2 s is no reading -- the force
branch is skipped and the model-based cancellation zeroed (one warning, one "back"). `IMU_DELAY` (5 ms: the IMU
arrives within +-10 ms of the mocap on flight 11's bag, `scripts/ana_imu_latency.py`, corr 0.95-0.99 per axis) is
added to the force branch's delay buffer. The bias state IS the EKF's bF: the sim IMU has NO bias (its bias terms are
commented out in `sensors/imu.py`, its random walk uses `rand()`), so a real turn-on bias was gated with `--bias_acc
0.2,0.2,0.2` (0.79 N per axis): bF took exactly 0.79 N, cancellation / roll / prediction unchanged -- no extra state
is observable from F_hat alone. Tests: `tests/test_imu_node.py` (frame, staleness guard, zeroed cancellation, `--est
pwc` refused; 7/7), `tests/test_cascade_node_boots.py --node imu` (boots as the launcher does; a SensorCombined
message on the topic must be logged as `FLU (+0.50, -0.20, +9.81)`). Launchers forward `IMU_WC/IMU_DR/IMU_DELAY`;
`start_slosh_hil.sh` refuses READY without the `[cascade+imu]` banner and the subscription line when
`ADAPTIVE_NODE_PY` names the IMU node; results rows carry `anode=`.
**PIECE C -- `state_source: px4` BUILT, BOOT-TESTED, NOT FLOWN (controller 7f416a3, sim 350026d).** The base node's
new string parameter (`mocap` default -- byte-identical; NEVER read it through `gp()`) subscribes to EKF2's
`/<ns>/fmu/out/vehicle_odometry` instead of the mocap and feeds `odom_to_state()` (NED/FRD, PX4's [w,x,y,z]
quaternion -> ENU/FLU: R = T_en R_ned_frd T_fb, p/v through T_en, rates through T_fb; NaNs before EKF2 initialises are
skipped) into the mocap's own `_on_state` path. `tests/test_state_source.py` pins the converter synthetically and
against flight 11's bag (EV-fed EKF2 vs mocap: position 0.70 m rms = its ~35 ms of latency at 16 m/s, velocity
0.10 m/s, attitude 0.32 deg mean / 5 deg max in the turns, rates 0.008 rad/s; 13/13); boot test `--node px4src`
(subscribes to the odometry and NOT the mocap). Launchers: `CASC_STATE=px4` on both cascade panes, `EKF_SOURCE=gps`
(keeps the mocap->EV bridge for YAW -- the airframe has `EKF2_MAG_TYPE 5`, no mag, so vision yaw is the only heading
source -- and skips the relax window's full-EV override so EKF2 stays on the airframe's GPS+baro position/velocity),
`VTOL_GPS_HZ` (the sim GPS's rate: Pegasus's default 250 Hz; `vtol.py` reads it into the config's `GPS()`); READY
verifies the GPS rate on `vehicle_gps_position`, the odometry rate, `state source: px4` in each cascade pane and
`vehicle_global_position` on GPS; provenance `state= ekf= gps_hz=` in the launch log and results row. TO FLY IT:
first `CASC_STATE=px4` alone (EV-fed EKF2 ~ mocap quality: the state path), then `EKF_SOURCE=gps VTOL_GPS_HZ=10` with
the IMU node (the target configuration); re-measure `latency_s` with `casc_timing.py` on that bag (EKF2 adds its own
lag to the 49 ms measured on the mocap) before trusting the delay compensation.
**★ THE LAUNCHER BUG THAT COST A FLIGHT (sim 9bc37e0):** my `EKF_SOURCE` echo in `start_vtol_smc_hil.sh` was an unbound
variable under `set -u`; the cold start aborted half-way with Isaac, PX4 and the controllers already up but no
`10-bag` recorder and no relax window; every READY check passed, the first IMU flight landed 0.00 m, and
`fly_slosh_mission.sh` scored it against the NEWEST bag on disk -- the leso flight's (row R100_dx3_imumdob_a1_173101,
annotated UNSCORED). Now: READY requires the 10-bag window AND the bag directory; the mission scores only the bag the
launch announced (LAUNCH_LOG's `bag=`), else `UNSCORED`, exit 3. Also: do not run gate sweeps during a flight -- the
mdob flight ran at RTF 0.76 because one overlapped it, which is a different experiment.

# ★★★★ 2026-09-15: THE RESIDUAL-ESTIMATOR COMPARISON, PIECES 1-3 (built one at a time, each audited
# for channel mixing before the next): THREE OBSERVERS BEHIND ONE INTERFACE, SWAPPABLE IN THE
# CASCADE VARIANT, AT TUNING PARITY. Piece 4 (the closed-loop gates) and the flights are next.
**THE QUESTION:** which residual estimator serves the cascade's L1 augmentation best under the SAME
NMPC + geometric + L1 scaffolding (C(s) 2 rad/s, clamps, hold, delay buffer, actuator-lag model, the
EKF slosh predictor) -- the flown SO(3) STESO, a linear (high-gain) ESO, or a momentum-based
disturbance observer -- and what the EKF adds on top of each.
**PIECE 1 -- the estimators (`so3_steso.py`, controller a5d5679 + 15dd6f7):** `MomentumDOB` (new:
tau_hat = K (p - int(M_applied + gyro + tau_hat) dt), no rate derivative, bit-exact hold, bumpless
set_K, innovation e = J^-1 dtau_hat/K), `LinearESO` completed to the STESO's caller interface
(omega_dot_hat, hold, per-axis wo, validated mode, exports as copies), `make_estimator(kind, J, add_gyro,
**knobs)` with ONE knob vocabulary filtered per kind. SO3STESO untouched -- three trace hashes (rate,
attitude, every option) pinned as exact bytes in `tests/test_residual_estimators.py` (64 checks:
per-axis isolation against each estimator's OWN floor, twins share no state, omega_dot_hat also
during hold, hold bit-exact and bumpless on an inexact-round-trip inertia, per-axis knobs ACT per
axis, add_gyro on a spinning body, first sample on the measurement, reset(), live retune). Two audit
rounds (five reviewer lenses + mutation testing of the test itself) found and fixed: the MDOB's hold
rewrote sigma through J^-1 J, its set_K rescaled the live estimate, its first innovation was zero by
algebra, the exports aliased the state, the LESO's knob was scalar-only with an unvalidated mode.
**PIECE 2 -- the swap (`nmpc_cascade_steso_node.py --est steso|leso|mdob|pwc`, controller 1c62688):**
the moment observer and its force twin are built at ONE call site each through the factory; the
kind is consulted only in the knob block, those two calls, the per-kind meas_lag table, the guard
and the banner (`tests/test_est_swap.py` pins that statically and dynamically; `--gate` runs all four
kinds: 0 solve failures each, log header identical, traces identical until u_ad first differs).
`--est steso` / `--est pwc` logs BIT-IDENTICAL to the pre-edit node. Launcher: `CASC_EST=leso` (or
mdob) on the adaptive pane; every knob forwarded as `VAR=${VAR:-}` (empty = the node's default: one
home per default, and a pane can never inherit a stale value from the tmux server's environment --
the audit found that hole); `start_slosh_hil.sh` refuses READY unless the banner's `estimator=` is
CASC_EST; the LAUNCH_LOG line carries the adaptive banner and the results row `est=<kind>`.
**★ AN AUDIT AGENT WROTE A MUTATION INTO THE LIVE CONTROLLER TREE** (`meas_lag=PRED_LAG * (2.0 if
hasattr(self.est_f, "wo") ...)` -- a kind-dependent predictor lag, the exact mixing the audit hunts);
caught by `git status`, restored. Check `git diff --quiet` in the controller repo before any gate run
that follows an audit.
**PIECE 3 -- parity (`scripts/est_parity.py --solve`, `tests/test_est_parity.py`, controller fdecc8f):**
"the same number" is NOT like-for-like (at L = wo = K = 20 a 1 rad/s^2 step's 63% rise is STESO 29 /
MDOB 50 / LESO 106 ms). Two rules, one per channel, both measured on an estimator-alone harness at
the gate's rate (200 Hz) and noise, then confirmed on the closed loop:
  * MOMENT observers: EQUAL RAW-NOISE FLOOR (what C(s) must strip before the actuators): the STESO
    at L 20 reads 0.068 N.m on pitch = 0.185 rad/s^2 on every axis (rate-driven) -> **leso wo 42,
    mdob K 11**. Closed loop (CG tank, cruise): 0.0615 / 0.0615 / 0.0607 N.m -- equal to 1%. At that
    floor the LESO matches the STESO at the tank's 1.14 Hz (lag 42 vs 44 ms, gain 0.97 vs 1.00: two
    second-order observers) while the first-order MDOB lags 81 ms at gain 0.84 -- structural, so the
    comparison carries **MDOB_K=22 as a second row: equal lag (44 ms) at 1.96x the floor**.
  * FORCE twins: EQUAL READING LAG at 1.14 Hz. The STESO force twin's 0.059 N floor is entirely its
    chatter (1.1 L_F dt m = 0.065 N); the linear twins sit at 0.002-0.03 N at any sensible bandwidth,
    so floor parity would be absurd. Set to the STESO's closed-loop reading lag (15 ms): **leso wo_F
    99, mdob K_F 66**, floors 0.030 / 0.031 N -- 2x quieter than the STESO's, on the record. Closed
    loop: the raw force twins read the fore-aft bob at corr 0.97 (STESO 0.71, its chatter), lag 5 ms
    (STESO 0); the predictor then errs 0.017-0.018 N on all three.
  * `PRED_LAG_BY_KIND` = the measured reading lag per kind (steso 0 / leso 5 / mdob 5 ms -> 0.005 s
    each; the twins were tuned to one lag, so the predictor's measurement model is de facto
    kind-independent). Flights 10/11 flew 0.015; the predictor is insensitive across 0-15 ms (corr
    0.947 -> 0.957, err 0.018 -> 0.017 N at 0.005; worse only past 25 ms). `--pred_lag 0.015` still
    reproduces the flown STESO log bit-for-bit.
**SCORERS:** `scripts/est_loop_metrics.py LOG.csv` (raw floors, u_ad noise, force-twin reading lag
vs the plant's bob, predictor lag/corr/err from a gate `--log`).
**PIECE 4 -- THE CLOSED-LOOP GATES (sim cae4dcc+; `scripts/est_sweep.sh <tag> turns|hover ..`,
`scripts/est_metrics.py <dir>`; sweeps in `~/flight_bags/est_sweeps/`). THE PAGE:
https://claude.ai/artifact/WpNwTasTBsr8SCp3TqJGJr**
Offset 100 mm tank, turning mission (legs -> orbit -> RTH stop), 3 seeds x {steso, leso, mdob K11,
mdob K22} x predictor off/on, at RTF 1 and at RTF 0.86:
| kind | pred | bob|F| turns/orbit/stop [N] | wall hits | orbit roll sd | u_ad orbit | pred err +0.25 s |
|---|---|---|---|---|---|---|
| steso | off | 0.43/0.78/0.63 | 284 | 0.40 | 0.046 | -- |
| leso / mdob / mdob22 | off | 0.41/0.77/0.6 | ~270 | 0.35-0.36 | 0.035-0.041 | -- |
| steso | ON | 0.065/0.047/0.31 | 0 | **0.21 +- 0.02** | 0.020 | 0.131 |
| leso | ON | 0.064/0.056/0.28 | 0 | **0.137 +- 0.002** | 0.007 | 0.102 |
| mdob K11 | ON | 0.066/0.060/0.29 | 0 | **0.137** | **0.006** | **0.085** |
| mdob K22 | ON | 0.064/0.056/0.28 | 0 | 0.137 | 0.007 | 0.105 |
(RTF 0.86 the same picture: orbit roll sd 0.19 steso vs 0.13 linear with the predictor; pred err
0.141 / 0.116 / 0.101 / 0.116.) **READING:** WITHOUT the EKF the structure does not matter to the
aircraft (same mission, same stop 5.9-6.0 m / 7.8-8.2 at the HIL clock, 0-1 fails; the STESO's chatter
= 12-15% more orbit roll noise; the MDOB's 2x moment lag shows in ONE place -- the stop at RTF 0.86
where the residual ramps: 0.43 vs 0.29-0.34 deg). WITH the EKF everyone gets the 6-16x bob reduction
and zero wall hits, BUT the structure still shows: the linear observers' quieter readings give a
35% quieter steady orbit (0.137 vs 0.21 deg, far outside the seed sd), 3x less augmentation noise
into the actuators, 20-35% better prediction; the MDOB's stop penalty vanishes (0.11, the best) --
the bias states carry the ramp. The MDOB at K 11 is the BEST predictor despite its lag because the
EKF's bob is driven by the force twin (K_F 66, 15 ms), not the moment channel; the K 22 row buys
nothing and costs noise. HOVER with a 5 cm CG error (tank at the CG, 30 s, 2 seeds): every kind reads
1.936-1.938 N.m (true 1.938), pitch -1.16, u_ad 1.937; predictor on/off byte-identical (no arm) --
on a constant residual nothing separates them. REGRESSION (16 seeds, faithful line): leso 16/16 and
mdob 16/16, worst crossing 14.3, z 29.9-30.0, stop 8.8 s, fails-sum 1 / 3 (STESO 16/16, 14.1-14.3, 3).
**RECOMMENDATION: mdob K 11 + EKF** (simplest observer, best predictor; leso its equal at the
aircraft level). Caveats on the page: the absolute aircraft-level differences are 0.07 deg of orbit
roll and 0.014 N.m of u_ad -- a flight sees them only against the HIL band (orbit roll sd 1.0-1.26
across flights 9-11); the STESO's separating floor is a 200 Hz discretisation property; the STESO
keeps the finite-time proof (Theorem 1). `ke_sweep.sh` takes `KE_NODE=nmpc_cascade_steso_node.py`.
**A SLOW POSITION SOURCE (2026-09-15, the user's GPS question; gate `--pos_hz H --vel_hz H --pv_lag S`
= sample-and-hold of the controller's VIEW of position / velocity, attitude and rates untouched; sweeps
`~/flight_bags/est_sweeps/posA_*` (position only) and `posB_*` (position + velocity), 2 seeds):**
  * POSITION ALONE is benign to 5 Hz (orbit roll sd 0.14 -> 0.24 linear / 0.21 -> 0.29 steso, stop
    within 1 m); at 2 Hz the guidance hunts on an 8 m-stale position (roll sd 1.1-1.2, stop 3-4 m
    +-1.5, 1-2 fails) but flies. The observers/predictor never read position: unchanged.
  * POSITION + VELOCITY (a GPS-only source): the loop itself needs velocity >= 5 Hz -- at 2 Hz even
    `pwc` (no force observer) crashes (51 fails, tumble: the delay-compensated outer on a 0.5 s-old
    velocity). The FORCE TWIN is the casualty: the linear twins at their parity bandwidth (99 / 66
    rad/s, tuned for 200 Hz) turn velocity steps into phantom forces the EKF believes and the L1
    cancels -- u_ad 0.45-0.55 N.m, 6-12 fails, stop 9-16 m at 20 Hz; leso orbit roll sd 2.9 at 10 Hz,
    12 deg at 5 Hz. The STESO's slew bound (L_F 3) degrades gracefully (0.59 / 1.17 at 10 / 5 Hz)
    but its cancellation is gone at 20 Hz already (bob back to the predictor-off 0.81 N). Predictor
    OFF at 5-10 Hz: steso 0.55 / 0.77, pwc 0.49 / 0.99 -- the base loop tolerates it.
  * RULE: with a GPS-class source the force channel must NOT be a differentiated velocity: source it
    from the IMU's specific force (algebraic, 250 Hz; also removes the RTF term), or gate the
    predictor off below ~50 Hz of velocity and tie the force twins' bandwidth to the source rate.
    The moment observers (body rates) are untouched, so the estimator RANKING stands.
  * WHAT PX4 GIVES TODAY (flight-11 bag): `vehicle_attitude/odometry/local_position` at 87 Hz on
    DDS, but EKF2 is fed the MOCAP as external vision (`4b-ev`, `EKF2_EV_CTRL 11`, `EKF2_GPS_CTRL 0`):
    a mocap-derived estimate ~35 ms late (0.55 m horizontal at 16 m/s), `vehicle_global_position`
    NOT published (no GPS aiding -> no origin); raw `vehicle_gps_position` at 87 Hz (Pegasus GPS
    default 250 Hz + random walk -- unrealistic); IMU `sensor_combined` 87 Hz. GPS+IMU dead
    reckoning = `EV_BRIDGE=0`, drop the EV override (the airframe's own defaults are GPS+baro+vision
    yaw), realistic GPS rate/noise in the sim config; then a `state_source: px4` path in the node.
**NEXT:** the IMU-based force residual (algebraic, the fourth twin) and re-run case B; the PX4
state-source path; one flight each of mdob and leso on R100_dx3 as `CASC_EST=mdob CTRL=cascade_adaptive`.
**THE IMU-BASED FORCE TWIN (2026-09-15 evening; controller 7a860d2 + 79c5abe, sim this commit) -- A SEPARATE
COPY, NOT A FOURTH KIND:** `imu_force.ImuForceResidual` (F_hat_b = m a_b - F_model_b, both without
gravity, one low-pass at wc 66 rad/s = the other twins' 15 ms parity lag) inside
`nmpc_cascade_imu_node.py` = `L1Steso` with ONLY the force branch replaced (`L1Imu`: the moment observer
stays `--est steso|leso|mdob`, the L1 law, delay buffer, actuator-lag model, EKF and cancellation are
inherited; the predictor's tank input is the low-passed accelerometer instead of model force + bob
reaction). The audited variant is byte-for-byte untouched -- that was the point: if the reading is
better, the new code is what did it. Gate seam: `base.IMU_ACCEL` = the plant's specific force in its
TRUE body frame incl. the bob's reaction, one tick old, + `--noise_acc` (0.02 m/s^2 = the sim IMU's
0.004 m/s^2/sqrt(Hz) at 250 Hz); the ROS accelerometer (`sensor_combined`, FRD->FLU) is NOT wired and
the node refuses to fly without `--test`. Sweep it with `KINDS="imu-steso imu-leso imu-mdob"`
(`est_sweep.sh` maps `imu-<kind>` to the copy); `scripts/est_merge.sh <out> <tags..>` prints several
sweeps as one table (`~/flight_bags/est_sweeps/all_{full,10hz,5hz}`).
**THE 5-10 Hz RANKING (position AND velocity held; the user's GPS-class case), offset 100 mm tank,
turning mission, 2 seeds; the LATERAL bob (the one with the arm = what the cancellation is for) in the
orbit, wall hits, orbit roll sd, u_ad, RTH stop:**
| rate | channel + moment obs | y-bob orbit [N] | hits | roll sd | u_ad | stop m | fails |
|---|---|---|---|---|---|---|---|
| 200 Hz | velocity twin + EKF (steso / leso / mdob) | 0.018 / 0.015 / 0.017 | 0 | 0.21 / 0.14 / 0.14 | 0.02 / 0.007 / 0.006 | 5.7-5.8 | 0.3-1 |
| 200 Hz | IMU twin + EKF (imu-steso / -leso / -mdob) | 0.023 / 0.013 / 0.015 | 0 | 0.19 / 0.15 / 0.15 | 0.02 / 0.007 / 0.006 | 5.7-6.1 | 0-0.5 |
| 10 Hz | predictor OFF (steso / pwc) | 0.63 / 0.62 | 228 / 196 | 0.55 / 0.49 | 0.11 | 4.4 / 4.8 | 1.5 / 0.5 |
| 10 Hz | velocity twin + EKF steso | 0.66 | 296 | 0.59 | 0.12 | 4.7 | 0 |
| 10 Hz | velocity twin + EKF leso / mdob / mdob22 | 0.71 / 0.74 / 0.73 | 180-230 | 2.9 / 0.34 / 0.47 | **0.83-1.04** | **13 / 16 / 10** | 7 / 3.5 / 1.5 |
| 10 Hz | **IMU twin + EKF** imu-steso / -leso / -mdob | **0.026 / 0.020 / 0.022** | 1.5 / 0 / 0 | 0.32 / 0.35 / 0.36 | 0.10 | 4.4 / 3.9 / 3.9 | 0.5 |
| 5 Hz | predictor OFF (steso / pwc) | 0.58 / 0.58 | 161 / 146 | 0.77 / 0.99 | 0.21 | 4.3 / 4.4 | 0.5 / 1 |
| 5 Hz | velocity twin + EKF steso | 0.53 | 49 | 1.17 | 0.22 | 3.8 | 1 |
| 5 Hz | velocity twin + EKF leso / mdob / mdob22 | 0.29 / 0.42 / 0.38 | 47-136 | **12 / 9 / 10.7 deg** | 0.84-1.0 | 12 / 5 / 5.8 | 6 / 4.5 / 3.5 |
| 5 Hz | **IMU twin + EKF** imu-steso / -leso / -mdob | **0.028 / 0.019 / 0.021** | 4 / 3 / 4.5 | 1.33 / 1.0 / 1.0 | 0.20 | 4.3 / 5.5 / 4.5 | 0.5 / 0 / 0.5 |
**READING.** (1) The IMU twin is the ONLY force channel whose cancellation survives a slow source: the
lateral bob stays at 0.02-0.03 N at 10 AND 5 Hz (200 Hz: 0.013-0.023; uncancelled 0.6-0.7), no phantom
forces into the L1 (u_ad = the predictor-off value at every rate, where the linear velocity twins push
0.8-1.0 N.m, stop 10-16 m out and fail 3-7 solves). The |F| the old table quotes (0.14 / 0.29 N at 10 /
5 Hz vs 0.04) is the FORE-AFT bob (no arm, outside the cancellation by construction) excited by the outer
hunting on a held velocity; likewise the 1.0 deg orbit roll sd at 5 Hz is the loop (pwc 0.99), not the
estimator -- the base loop's own floor. (2) Moment observer with the IMU twin: **mdob ~ leso > steso**
(imu-mdob the best predictor at every rate: corr 0.82 / 0.76 / 0.49, err at +0.25 s 0.080 / 0.102 /
0.274; imu-steso 0.72 / 0.66 / 0.45 and the wall hits at 10 Hz) -- the full-rate ranking stands, so the
recommendation is now **mdob K11 + EKF on the IMU force channel** (`imu-mdob`). (3) At 200 Hz with the
sim's accelerometer noise the IMU twin is at aircraft-level parity with the velocity twins although its
RAW reading is noisier (corr 0.85 vs 0.96): the EKF's bob state absorbs white noise above 1.14 Hz.
(4) **"Algebraic = rate-free" was half right:** the raw reading's corr fell 0.85 -> 0.27 -> 0.14 at 10 /
5 Hz, and the error is a 0.45 / 0.95 N sawtooth at EXACTLY the hold rate (98-99% of its power above
3 Hz): the aero MODEL force uses the held velocity, and a 0.1 s-old velocity in a 16 m/s turn is ~0.9 deg
of sideslip = ~0.9 N of side force. The EKF rejected most of it (which is why the aircraft-level numbers
held), and `--imu_dr 1` closes it outright: the model's velocity dead-reckoned between samples with the
same accelerometer (v += (R a_b + g) dt, reset on each new sample) puts the reading at its 200 Hz floor at
both rates -- raw corr 0.84-0.86, pred corr 0.79-0.82, error 0.053 N (`imuDR_*` sweeps). Default off so the
`imuB_*` rows reproduce; ON is the design for a GPS-class source (an INS in one line). Aircraft-level it
changes nothing at 10 / 5 Hz because the residual floor there is the outer loop's held velocity, not the
estimator.
**WHAT THIS DOES NOT SAY:** the accelerometer here is the plant's true specific force + white noise (no
bias, no scale, no vibration, no timestamp offset) -- the real `sensor_combined` needs a bias state (the
EKF's bF can carry a slow one) and its own delay in the buffer before any of this is flown; 2 seeds per
cell; the outer NMPC / guidance still read the held velocity (the 5 Hz floor) -- the dead reckoning is
only the force twin's model input, a `state_source: px4` (EKF2's INS) is the real answer for the loop.
**HOW IT DEGRADES (imu-mdob, 2 seeds each; `imuB_{full,50hz,20hz,10hz,5hz,2hz}`, `imuDR_*`, `*_na05`):**
| pos+vel rate | y-bob orbit | x-bob orbit | orbit roll sd | u_ad | stop m | fails | pred corr / err +0.25 s (DR) |
|---|---|---|---|---|---|---|---|
| 200 Hz | 0.015 | 0.038 | 0.147 | 0.006 | 6.1 | 0 | 0.81 / 0.080 |
| 50 Hz | 0.018 | 0.048 | 0.151 | 0.017 | 5.7 | 0 | 0.82 / 0.081 |
| 20 Hz | 0.030 | 0.036 | 0.185 | 0.049 | 5.0 | 0 | 0.80 / 0.084 (0.81 / 0.080) |
| 10 Hz | 0.022 | 0.143 | 0.36 | 0.10 | 3.9 | 0.5 | 0.76 / 0.102 (0.81 / 0.079) |
| 5 Hz | 0.021 | 0.30 | 1.00 | 0.20 | 4.4 | 0.5 | 0.48 / 0.274 (0.79 / 0.088) |
| 2 Hz | CRASH (51 fails, tumble -- the outer on a 0.5 s-old velocity; pwc and every velocity twin crash identically) |
The estimator's own number (the lateral bob it cancels) is FLAT from 200 to 5 Hz (0.015-0.030 N); every
other column is the LOOP degrading on a held velocity, and every one of them equals the pwc / predictor-off
value at the same rate (u_ad 0.10 / 0.20, roll sd 0.49 / 0.99 at 10 / 5 Hz). Knee at 10 Hz (roll sd
x2.4, x-bob x4), cliff at 2 Hz. The velocity twin (mdob) for comparison: cancellation gone at 20 Hz
already (y-bob 0.67 = uncancelled, u_ad 0.55, 12 fails, stop 8.6), 0.74 / u_ad 0.83 / stop 16 m at
10 Hz, roll sd 9 deg at 5 Hz. Without DR the IMU reading's raw corr falls 0.84 -> 0.79 -> 0.51 -> 0.28
-> 0.14 (the held-velocity sawtooth in the model force) and the EKF absorbs it down to 10 Hz (pred corr
0.80 / 0.76) but not at 5 Hz (0.48, err 0.274); with DR the prediction is flat (0.79-0.82 / 0.079-0.088
at every rate). **Accelerometer noise** (imu-mdob; 0 / 0.02 = the sim IMU / 0.05 m/s^2 per sample): raw
corr 0.88 / 0.84 / 0.71 at 200 Hz and the same 0.85 -> 0.73 at 10 and 5 Hz with DR; prediction err 0.080
-> 0.090 (-> 0.097 at 5 Hz); the aircraft-level columns do not move (y-bob 0.018-0.024, roll sd, u_ad,
stop identical) -- 2.5x the sim's noise, integrated between 5 Hz fixes, costs 0.01 N of prediction and
nothing the aircraft feels. The design trade has moved from "which observer" to "what velocity does the
OUTER get": the force channel is rate-free, the loop needs >= 5 Hz and is clean only from ~20 Hz up.

# ★★★★ 2026-09-14 (night): "ADAPTIVE NMPC + GEO" IS A SEPARATE CONTROLLER IN THE GROUND STATION.
**WHAT IT IS.** The STESO + slosh-predictor cascade (`nmpc_cascade_steso_node.py`, the evening block
below) no longer REPLACES the flown cascade: it stands by BESIDE it as its own controller.
Identity: `controller_id cascade_adaptive` -> ROS node `/nmpc_cascade_control_adaptive`, guidance on
`/uav_0/cascade_adaptive/{cmd,status}`, launcher window `6i-adaptive` (both `restart_hil.sh` and
`start_vtol_smc_hil.sh`; `ADAPTIVE=0` skips it, `ADAPTIVE_NODE_PY` overrides the file; it starts
15 s after 6h because both load one nmpc6 JIT cache). The base node derives its node name and its
guidance topics from the id (`node_name_for`: `cascade` keeps `nmpc_cascade_control` and
`/uav_0/cascade/*`, every other id is suffixed) -- read from the ROS args before construction,
since a node's name is fixed there. Two cascades cost nothing: a standby cascade publishes no
actuator, runs no inner step and no outer solve (`pub()` returns unless `_active`).
**GS (`vtol-cascade-work`):** button "⇄ Adaptive NMPC + Geo" (teal), mission ctrl choice
"Adaptive NMPC + Geo" (same CASC_KEYS/CASC_OVERRIDES parameter set -- `CASCADE_IDS` is the cascade
FAMILY), the Guidance row publishes to whichever cascade is ACTIVE and shows THAT node's status
feed (`HudSource.casc_feeds[id]`), arm/disarm/land param sweeps and STOP's back-transition include
the adaptive node. Driver: `CTRL=cascade_adaptive` (`fly_transition.py --controller`); the results
row carries `ctrl=`. Health: `hil_health.sh` requires `/nmpc_cascade_control_adaptive` alive when
the 6i window exists; `start_slosh_hil.sh` refuses READY on its banner / traceback / node / `M=` /
`CG_X=` like the base cascade. **Switch between the two in HOVER only**: engage resets the schedule
(`_v_cmd` 0, tilt 0) -- the same rule as any switch into a cascade.
**COMMITS:** controller e137f5e, GS 4fed4b1, sim 60397ff (+ this line).
**GATES:** `tests/test_cascade_node_boots.py` boots base / tank / adaptive as the launcher does and
requires the ack on the OWN status topic + the OWN node name (PASS x3); `test_gs_trajectory_tab.py`
"adaptive cascade" block; `test_gs_cascade_params.py` unchanged and passing.
**FLIGHT 11 (row R100_dx3_ADAPT_a1_192020, bag slosh_20260914_191928; `CTRL=cascade_adaptive`, the
base tank cascade standing by in 6h, 100 mm tank at dx 0.3, bob logged): LANDED 0.00 m.** The
first full-stack boot with both cascades passed every READY check; the driver engaged
`/nmpc_cascade_control_adaptive` (its log carries the mission, the 6h pane has no ENGAGED), 0/3551
solve failures, 90 deg in ~15 s, orbit **100.0 +- 0.31 / roll -18.4 +- 1.03** (flight 9: 0.32/1.02,
flight 10: 0.40/1.26), turns +14.4/+14.7 m, crossing +14.1, **stop 14.9 m from home in 10.1 s, aft
tilt peak -8.0, z 30.0 throughout** (never dipped). Predictor + cancel: the plant's LATERAL bob
in the orbit F_y rms 0.05 N (flight 10: 0.08; flight 9 uncancelled: 0.32) -- the cancellation's
result repeated on a second flight. The FORE-AFT bob (no arm -> no moment -> outside the
cancellation by construction) ran at 0.42-0.48 N rms with ONE 9.2 N wall touch at the orbit
entry (bob -54 mm): inside the campaign's run-to-run band (f6 0.74 / f7 0.45 / f8 0.35 / f9 0.34 /
f10 0.23 in the orbit; f6 hit 7-17 N) and unnoticed by the aircraft; its stop-phase bob was the
smallest of the seven offset-tank flights (15.5 mm rms, max 34 mm, 0.33 N). Noted, not chased:
EVERY offset-tank flight (f5-f11, all three controller variants) carries a ~1.05-1.07 Hz pitch ring
of 0.35-0.43 deg sd in the orbit (the CG-tank flight 0.38 at 0.93 Hz), 6-8% below the tank's
1.14 Hz mode -- the Q~100 fore-aft bob is what it resonates, which is why the x-bob sits at
25-30 mm rms in all of them. Scorers now follow the flown id (`ana_guided.py`, `ana_rth_onset.py`,
`ana_slosh_est.py`, `ana_force_model.py` pick the cascade topic that carried the flight);
`fly_slosh_mission.sh`'s `CASC_SETTLE_S` targets the flown node. The GS BUTTON itself is still
unpressed in flight (the scripted path closes the GS); `tests/test_gs_guidance_live.py` with the
adaptive ctrl is the way to press it.

# ★★★★ 2026-09-14 (evening): THE STESO BECOMES A PREDICTOR -- THE TANK'S MODE AS THE INTERNAL MODEL OF
# A KALMAN LAYER; FORCE AND MOMENT UNCERTAINTY (bias) SPLIT FROM THE DISTURBANCE (the bob); THE
# PREDICTED TANK MOMENT CANCELLED AT THE INNER. GATED, FLOWN (flight 10, LANDED 0.00 m).
**WHY.** The STESOs estimate a LUMPED residual with one assumed property (|sigma_dot| <= L): observers,
never predictors -- they report what the plant felt ~1/L seconds ago. But the tank is structured: one
bob at sqrt(f_z/L1) with known damping and a wall, and its force and moment are ONE state seen
through m1 and r_tank. `slosh_predictor.SloshPredictor` (controller package) is an EKF whose model
IS the plant's `PendulumSlosh`: state [x, y, xdot, ydot, bF(2), bM(3)], driven by the controller's
own specific force at the tank (the applied model force in the body frame + the bob's reaction + the
rotation terms with the moment STESO's own omega_dot), corrected by the two STESOs' residuals; the
bias states are the UNCERTAINTY (CG, aero, the real-time-factor term in HIL), the bob is the
DISTURBANCE, `predict(tau)` integrates it ahead, `impact_eta()` says when it reaches the wall.
Optional `meas_lag` states model the STESO's own reading lag (10-20 ms at L_F 3, measured on the
gate) so the filter de-lags it: unmodelled, a 0.1 s lag caps tracking at corr 0.85; modelled, 0.98.
**`tests/test_slosh_predictor.py`** (against the sim's pendulum by path, the flown 100 mm tank): bob
converges to 0.4 mm on 13 mm; prediction at +0.05/+0.25/+0.5 s errs 0.001 N where hold-last errs
0.10/0.43/0.54; a 1 N force bias and 0.5 N.m moment bias resolve to 1.01 / 0.50 through 0.3 N of
noise with the bob still tracked at corr 0.97; a wall impact is announced 0.2 s ahead to 0.01 s with
the contact force to 2% (7.42 vs 7.59 N). Tuning from a sweep: q_bob 0.35, r_F 0.3, r_M 0.1.
**IN THE CASCADE (`nmpc_cascade_steso_node.py`):** `--slosh_pred on` (default; the tank from the
gate's own tank or the sim's `default_tank()` by path -- one pendulum), `--pred_lag 0.015`,
**`--pred_cancel on` (default): the inner's L1 law cancels the PREDICTED slosh moment at t + delay
UNFILTERED (model-based, so C(s)'s noise argument does not apply; clamped by `--pred_umax` 1.0 N.m)
and the bias through C(s) as before**; `--pred_outer` carries the predicted force/moment through
predict6 (`base.OUTER_DIST` seam -- a measured NO-OP, the force is 1% of weight; off). The status
feed gains `| pred bob=(..)mm f1=.. F=(..) M=(..) bF=(..) bM=(..) [wall in X s]`; `--pred_score`
scores a gate run against the truth (`base.SLOSH_TRUTH`), `--log` carries the truth and the
predictions; `ana_slosh_est.py` prints `PREDICTOR vs bob`.
**GATE, offset 100 mm tank, turning mission (legs, orbit, RTH), noise 0.1/1, L1:** the STESO reading
correlates 0.90/0.92 with the bob and lags it 10-20 ms; the predictor now 0.94/0.91; at +0.25 s the
prediction errs 0.15/0.36 N where hold-last errs 0.48/0.89. **CANCEL ON: the plant's own tank force
0.52 -> 0.06 N rms in the turns (its 3.6 N wall impact gone), 0.61 -> 0.01 in the orbit; the
aircraft's roll sd 0.32 -> 0.16 (orbit), 0.45 -> 0.24 (stop); RTH stop 6.7 -> 3.5 m from home** --
the bob -> yaw moment -> yaw acceleration -> bob loop at the 0.26 m arm is broken. At RTF 0.86 the
raw lateral reading falls to corr 0.46 (the clock artefact) and the bias state absorbs it
(bF_y +0.76 N), the bob estimate holds at 0.87, cancel still 0.50 -> 0.03 / 0.69 -> 0.02, stop
6.8 -> 5.5 m. Regression 16 seeds: 0.502 kg 16/16 pk 14.1 (identical -- a tank at the CG has no arm),
0.7 kg 16/16 pk 13.7, offset tank at RTF 0.86 16/16 pk 14.6.
**FLIGHT 10 (row R100_dx3_pred_a1_162058, bag slosh_20260914_162025; predictor + cancel on, RTF 0.84):
LANDED 0.00 m.** Crossing +14.2 (the gate's 14; flight 9: 15.3), 0 fails, z 29.9-30.0, stop 10.4 s,
orbit 99.9 +- 0.40 / roll -18.3 +- 1.26 (flight 9: 0.32 / 1.02 -- one-flight scatter; HIL's lateral
numbers are 3-4x the gate's for reasons that are not the tank). **The plant's own bob: F_y rms 0.16 ->
0.08 N, in the orbit 0.25 -> 0.08, |F| max 1.56 -> 1.15.** The predictor reads the fore-aft bob at
corr 0.85 (the raw STESO 0.6-0.7, err 0.12-0.15 -> 0.05-0.08); the lateral raw reading is 1.7 N of
RTF artefact, the bias takes it (bob+RTF corr 0.94), and the bob estimate is left at the noise floor
(0.28 N) because the bob it is estimating is now 0.08 N -- the cancellation's own success.
**THE STABILITY NOTE (the paper's proof pattern, extended to the three layers):**
https://claude.ai/code/artifact/a015c7a9-294b-4871-a443-0833f4603323 . Theorem 1: STESO finite-time
convergence -- Seeber-Horn 2017 for the shipped (1.5 sqrt L, 1.1 L); a quadratic Moreno-Osorio
certificate exists at those gains only for |sigma_dot| <= 0.68 L (searched numerically), and the tank
asks 0.19 L on yaw / 0.64 L_F on force (a wall impact is 7 L_F: bounded, not exact). Theorem 2:
the EKF is observable (det = m1^2 w1^6 > 0 -- the measurement is the bob's ACCELERATION, so a bias
and an oscillator separate even at zeta 0), error eigenvalues 0.91/0.98 at the tick, 1-sigma
10.6 mm / 0.22 N; the prediction error is a bounded rotation of the state error plus the
input-hold term m1 |df|. ★ Theorem 3 (linearised lateral model with the aero derivatives from
the controller's own aero_body, Pade delay, the cruise laws incl. the BANK-TO-TURN path): the
coupled tank mode is UNSTABLE uncancelled for the flown 100 mm tank on the 0.26 m arm (zeta
-0.0019 at 49 ms), slightly worse under the base L1 (-0.0056: a lagged partial cancellation adds
energy), STABLE with the predicted moment cancelled (+0.0089) and still with a 50% mis-prediction
(+0.0035); 60/80 mm tanks and arms >= 0.35 m are stable either way -- a resonance of 1.14 Hz with the
lateral modes, not a gain, and not k_bank_d. Forced-response peak 238 -> 25. A yaw-only or
yaw+sideslip model does NOT show it (5% coupling; the bank-to-turn path is the loop). Theorem 4:
ISS of the interconnection driven by the bounded prediction error (the separation argument), the
clamp bounding it by construction. The gate's 20x and the flight's 2x are the nonlinear plant's
numbers; the eigenvalues certify the mechanism, not the amplitude.
**WHAT IT IS NOT:** it cannot anticipate an impact from nothing (it needs the braking to be in its
input first -- then 0.2 s of warning); the outer feed buys nothing measurable; the aircraft-level
gain in HIL is inside one-flight scatter and needs repeats; a mis-specified tank cancels a moment
that is not there, bounded by pred_umax. `vtol_nmpc.py` and the base cascade are untouched.

# ★★★★★ 2026-09-14 (afternoon): THE TWO "MODEL GAPS" WERE THE CLOCK; THREE REAL ONES FOUND BY THE
# FORCE/MOMENT BALANCE AND FIXED; THE OFFSET-TANK STOP FLOWN CLEAN (flight 9, LANDED 0.00 m).
**READ THIS BEFORE QUOTING THE MORNING BLOCK BELOW IT.** Its "two model gaps the observer exposed"
(a lateral force ~ bank, +2 N fore-aft in the decel) do not exist. Both are the REAL-TIME FACTOR:
every clock the node has is wall time (mocap stamps, perf_counter, the outer's DT, the ramp, the
delay compensation) and Isaac ran flight 8 at RTF 0.861 (PX4 stamps vs bag clock), so an observer
that integrates the model's SIM-time forces against a WALL-clock velocity reads -(1-RTF) m a as a
disturbance: -1.33 N in the R 100 orbit (= +4.3 sin(roll) N, the "bank-proportional lateral force"),
+0.9 N fore-aft through the 1.74 m/s^2 decel. Redone on the plant's own acceleration
(`/uav_0/state/accel`, sim time) the controller's force model -- momentum props on their tilted
axes through the spin lag, `so3.aero_body`, the servo-lagged surfaces, the bob's reaction from the
Isaac log -- closes the plant's balance to **0.03-0.06 N rms on all three axes** through legs,
turns, orbit and RTH capture, every regression slope zero. **`scripts/ana_force_model.py <bag>
[isaac log] [--wall]`** is that reconstruction, force AND moment, per phase; run it before ever
calling anything a plant gap again (it also caught its own author's servo-order slip: unpacking
(de,da,dr) for a message that is [aileron, elevator, rudder] makes a 4 N "lift gap").
`ana_slosh_est.py` now judges the estimate against bob + RTF term: flight 8's lateral corr
-0.02 -> +0.92 in the turns, 0.32 -> 0.73 in the orbit. The observer was right all along.
**WHAT THE MOMENT BALANCE DID FIND, three things, all now fixed and pinned:**
1. **The sim's aero centre was the BODY LINK's CoM, 18.6 mm ahead of the assembled CG** (vtol.py
   `aero_center` default = `body_com_offset`; the 2026-09-06 fix meant "the CoM -> zero accidental
   arm", and for an articulated vehicle that is the assembly's). 38 N of cruise lift on that arm:
   a steady **-0.45 N.m nose-up pitch residual in every cruise, turn and orbit of every bag**, 0 in
   hover (it lives on the lift), the STESO's `Mres` -0.5, 1.2 deg of elevator trim the schedule
   never asked for. Default is now `assembled_mass_properties(config)[1]` (the assembly maths moved
   from vtol_slosh.py into vtol.py; re-exported). Flight 9 reads **+0.11 N.m**. It may be part of
   the HIL onset's 2-4 deg over the gate: flight 9's crossing peaked +15.3 (flights 4-8: 16-18.5).
2. **`VTOL_TANK_DX` moved the plant's CG 38 mm (0.502 kg at 0.3 m) and Iyy/Izz +13/+8% without
   telling the controller** ("the L1 test bed" -- but `VTOL_CG_DX` is the untold-CG knob, and a real
   tank's mount is measured). Every offset-tank flight was ALSO a 38 mm CG-error flight: flight 8's
   hover shows **+1.51 N.m** of pitch residual = M g x 38 mm; the gate with `--plant_cg_dx 0.038`
   tumbles in the corridor (HIL's corridor was a knife-edge: pitch -11.4 at V 6.7, z 29.2). NOW
   PAIRED like the mass: so3 folds VTOL_TANK_DX/DZ into `CG_SHIFT`, `CG_X`, `RPOS` and the inertia
   exactly as `fold_tank_into_body` does (0.2141 vs the plant's 0.2145; I to 0.001), the safe
   controller's `cg_x` follows, the STESO's tank arm is about the moved CG (0.262 m), the gate's
   `--slosh_dx` defaults to the env and its plant/controller agree, `tests/plant_double.py` folds
   it too, the launchers forward it to every controller pane and `start_slosh_hil.sh` refuses READY
   unless the cascade banner's `CG_X=` matches. `tests/test_tank_model_pairing.py` section 6 pins
   three tank positions; section 7 pins the aero centre. Flight 9's hover residual: **+0.04 N.m**.
3. **The outer planned -25 deg of aft tilt on a servo that stops at -20** (`BETA_AFT_MAX` 25 vs
   `tilt_min_deg` -20 on the FSC quad; the gate's `so3.build_ode` chases any tilt it is handed, so it
   never saw it). 24 N x (sin 25 - sin 20) = **1.9 N of braking that never arrived**, at the low-speed
   end where the aft tilt is all the authority left: flight 8's force table shows the +1.4..+1.95 N
   forward residual switching on exactly as the command passes -20. `BETA_AFT_MAX` = 20 (the
   airframe's), the node's applied-tilt estimate is clipped to it, the gate's plant has its own
   `--plant_tilt_min` (default 20), `tests/test_tilt_limit_pairing.py` holds sim and controller to
   one range. Regression, 16 seeds, faithful line + `--l1`: 0.502 kg 16/16 pk 14.3, 0.7 kg 16/16
   pk 13.7.
**THE GATE NOW MODELS THE CLOCK: `--rtf R`** advances the PLANT (ODE, actuator lags, pendulum) DT/R
per controller tick and leaves every controller quantity on its own clock -- the 1.5 m/s^2 ramp is
1.74 in the plant, the outer's plan under-delivers 14%, as HIL. 16/16 at 0.86 for both masses
(pk 14.5 / 13.9): the RTF is NOT the flown onset excess either. On the sim-time ramp decision: the
node stays clock-agnostic (hardware has RTF 1; the only sim-time source is PX4's stamp, which the
node should not read); the gate carries the RTF instead.
**THE HOVER-END RING, FINALLY:** flights 4/6/7/8's ~1 Hz pitch ring at the stop was the 1.5 N.m
untold CG moment (2) fought by an outer whose braking plan was short 1.9 N (3) on a ramp 16% too
steep for the plant (RTF), with the 100 mm tank's bob pumped to the wall by the pitch it caused
(flight 8 log: 6.5 N impacts; the wall's stiffness, item 5's "cause", was never the driver). The
gate could reproduce none of it because it had none of the three.
**FLIGHT 9 (row R100_dx3_paired_a1_152101, bag slosh_20260914_152028; STESO node, L1 on, 100 mm
tank at dx 0.3 PAIRED, bob logged): LANDED 0.00 m.** Corridor peak +11.5 gate / z 29.8-30.0 flown
(flight 8: z dip to 29.2); orbit 100.0 +- 0.32, roll -18.4 +- 1.0; RTH stop: pitch 13.5 -> 10 ->
8.6 -> 7.7 -> 6.8 -> 5.7 -> 4.8 (the gate's shape), **aft tilt peak -8.6 (was -25 pinned), stopped
in 10.2 s (was 16-20), z 29.9-30.0, roll <= 0.7, no ring; bob peak force 1.56 N (was 6.46)**, 16.5 m
from home (RTF; gate 4-7). Force balance 0.02-0.07 N rms wing-borne, 0.28 through the stop; moment
residual pitch +0.11 cruise / +0.04 hover. Estimator vs bob+RTF: y corr 0.95 turns / 0.86 orbit,
x 0.86 orbit. The +-12 deg cycle at t_stop+1..+3.5 is the SAFE controller's handover (switch to
`geometric` at V ~1 m/s with the bob at 45 mm), decaying to +-2 in 7 s -- its business, noted.
**F_SLOSH FEED-FORWARD (the listed step 3): NOT BUILT, deliberately.** In HIL the observer's output
carries -(1-RTF) m a: +0.85 N forward through every decel, -1.3 N in every orbit -- fed to the
outer it would brake harder for a push that does not exist, opposite in sign to the bob's real
effect, and absent on hardware; a HIL flight could not tell the feed-forward's value from its
artefact. On the gate (RTF 1, estimate clean, corr 0.98) the offset tank's slosh is already
absorbed (0/4001 at every geometry), so there is nothing measurable to buy. Build it (a disturbance
parameter in the 6-state NMPC + predict6, one JIT rebuild per mode) only for a hardware campaign.
**STANDING ITEMS AFTER THIS:** the HIL onset (+15.3 now vs the gate's 14) is a one-flight number;
re-score it on the next CG-tank flight to see how much of the old +2-4 was the aero arm. The safe
controller's handover kick with a wide offset tank (+-12 deg, one cycle). `vtol_nmpc.py` (not
flown) still carries CG_X 0.176 and its own RPOS -- pair it if it ever flies with a tank offset.
`ana_force_model.py --aero_arm 0.0186` reproduces the old arm prediction for bags before 15:00.

# ★★★★★ 2026-09-14 (day): THE COMPLIANT WALL, THE STESO FLOWN, AND A SLOSH FORCE/MOMENT ESTIMATOR.
# Committed at the start of the day (sim 7f915c2, controller 7dc67c8, GS 5cd1644), at 13:40 (42700c2 / 715a900) and after the afternoon block (a685e67 / b83f03f; GS unchanged).
**THE WALL IS COMPLIANT** (`SloshParameters.wall_omega` 35 rad/s, `wall_zeta` 0.7; `PendulumSlosh`
meets a repulsive Kelvin-Voigt contact past R/2 instead of the one-step arrest): a 0.17 m/s impact
is 3.8 N over 46 ms (was 16.7 N in 1/240 s), 0.5 m/s is 10.9 N over 71 ms and 4.5 mm; pinned in
`tests/test_slosh_pendulum.py`. Gate: the offset 100 mm tank's peak moment 4.19 -> 1.19 N.m, the
41 mm sweeps byte-identical (16/16, 14.1 / 13.7). **FLOWN (R100_dx3_wall_a1_130241, L1 off): landed
0.00 m, the bob log's peak 17 -> 8 N, and the hover-end ring was STILL THERE** (+-10..14 deg at
~1 Hz, aft tilt pinned at -25, stop 19 s). So the impulse size was not the loop's driver. The
cascade's own log says what is: below ~7 m/s the aircraft LAGS v_cmd by 0.8-1.8 m/s (the CG-tank
flight tracks it within 0.5), the aft tilt saturates and the outer flips to a nose-DOWN branch
(-10.8) instead of the flare -- the C9 flat-optimum hunt at the tilt bound, with the 0.41 kg bob's
fore-aft push (0.2 m/s^2) and its 1.14 Hz mode feeding it.
**THE STESO CASCADE FLEW (first flight, R100_dx3_steso_a1_132228, `nmpc_cascade_steso_node.py`, L1 on
with the SO(3) STESO as its estimator, L=20 rate mode): landed 0.00 m, 2 solve failures in the
mission, the TIGHTEST orbit of the campaign (rho 100.0 +- 0.24, roll sd 0.94 -- L1 damping the
offset tank's yaw moment), and the hover-end ring HALVED (+-5..11 deg) but not gone: the aft tilt
still walks to -25 and the stop still takes 16 s. L1 on/off was a confound in flights 4/6/7
(item 5's recipe had it off); with it on the tank is milder, and the structural cause is the
low-speed brake saturation above. The launcher accepts `CASC_NODE_PY=.../nmpc_cascade_steso_node.py`
(banner `[cascade+steso]`, process, mass check) and forwards VTOL_TANK_DX/DZ, CASC_EST, STESO_L,
STESO_LF to the cascade pane; results rows carry `node=`.
**THE SLOSH FORCE AND MOMENT ESTIMATOR** (`L1Steso`, same file): a second STESO, translational, on
the mocap's WORLD velocity with the controller's own force model fed forward (rotor thrusts along
their tilted axes through the spin lag, the aero force, gravity), `F_hat_b = R^T m sigma_F`;
`F_slosh` = its lateral part, `M_slosh = r_tank x F_slosh`, `M_resid = sigma_M - M_slosh` (a CG
error makes a moment with no force; a slosh makes both -- that is how the two are told apart).
Knobs STESO_LF (3 m/s^3: the 1.1 Hz slosh needs ~1.5; 30 was noise), VTOL_TANK_DX/DZ for the arm.
The first version observed the BODY velocity and read 0.1 deg of attitude noise at 16 m/s as
27 mm/s of lateral velocity per tick -> newtons (y anti-correlated -0.76 with the truth); the
world frame fixed it. GATE (100 mm tank at dx 0.3, 0.1 deg / 1 deg/s noise): orbit x corr 0.94 /
y 0.98, err 0.14-0.16 N on 0.4-0.6 N rms signals; the CG tank reads a 0.1 N floor. The gate trace
prints the true reaction (`Fs=`) and the node's status feed carries `slosh F=(..) M=(..) Mres=(..)`
(the `STATUS_EXTRA` seam in the base); `scripts/ana_slosh_est.py <bag> <isaac pane log>` scores a
flight against the bob log. **FLOWN: fore-aft corr 0.83 (turns) / 0.91 (orbit), err 0.12-0.17 N.
TWO MODEL GAPS THE OBSERVER EXPOSED** -- SUPERSEDED THE SAME AFTERNOON: both were the real-time factor, see the block above; kept as written for the record --** :
  * a LATERAL force proportional to the bank, -0.08 N/deg (-2.0 N at -24 deg, +1.5 at +17, zero
    wings-level) that the Isaac plant makes in a sideslipping turn and the controller's aero copy
    does not -- through the lever arm it reads as a spurious 0.45 N.m "slosh yaw" in every turn;
  * +2 N of FORE-AFT force through the decel: the braking the model expects from the aft-tilted
    front pair (negative axial inflow on the momentum prop) that the plant does not deliver --
    the same place the outer's v_cmd runs ahead and the tilt saturates.
  Both are the next `bag_axis_fit`-style measurements (lateral force vs bank in the orbit bags;
  thrust vs throttle at aft tilt in the stop), not tuning. Until then quote the estimator for the
  fore-aft channel and for the lateral one only wings-level or after that force is modelled.
The estimate is published and logged, not yet fed back: feeding F_slosh to the outer NMPC as an
acceleration disturbance (predict6) is the step after the two gaps.
**STANDING ITEMS AFTER THE MORNING (all three closed by the afternoon block above):** (1) the two lateral/aft-tilt force gaps above; (2) the low-speed
brake saturation with an offset wide tank -- clock the HIL decel ramp on sim time or lower
`decel_rate` for the last 5 m/s, then re-fly R100_dx3 with L1 on; (3) feed F_slosh forward.

# ★★★★★ WHERE THE BRANCH STANDS (2026-09-13 23:56) -- READ THIS FIRST. THE 2026-09-13 17:45 BLOCK
# BELOW IT IS STILL THE OPERATING MANUAL; THIS ONE IS WHAT CHANGED SINCE AND WHAT WAS FLOWN TONIGHT.
**SEVEN FLIGHTS TONIGHT, SEVEN LANDINGS AT HOME** (six scripted + the GS-button one; the first scripted attempt was cut short by a driver default, below, and still landed) (bags in `~/flight_bags/slosh`, rows in `results.tsv`;
scored with `scripts/ana_guided.py` + the new `scripts/ana_rth_onset.py`). The rig had lost its
NVIDIA driver at 17:47 (595-open installed then removed via PackageKit; nouveau took the GA102;
`sudo dpkg --configure -a && apt-get install nvidia-driver-575` + reboot restored it at 22:47);
`start_slosh_hil.sh` now refuses in one second without `/dev/nvidia0`.
**THE EIGHT OPEN ITEMS OF 17:45, CLOSED OR MEASURED:**
1. **RTH roll guard** (`rth_roll_arm_deg` 3, `rth_roll_wait_m` 10, home bearing latched once armed):
   flown, the arm came at +0.0 / +0.7 / +0.1 / +0.1 / +0.4 deg in the five guided flights. **And the
   bank was not the driver of the heavy crossing**: 0.7 kg wings-level still peaked +17.2 (was +16.6
   at 6.6 deg of bank), with 0 solve failures this time (was 3). A decel FORCED at 24 deg of bank at
   0.502 kg (the first attempt's driver bug, below) crossed at +13.9, z >= 29.7 -- the gate's 13.7.
   The heavy mode's crossing peak in HIL is ~3 deg above the gate (17 vs 14); a one-more-flight fact.
2. **Orbit: turn-rate feed-forward + ADAPTED bank feed-forward + radius INTEGRAL** (all default on):
   **rho 103.3 -> 99.7-100.0 +- 0.3-0.6 m in every flight**, kappa learned 0.79-0.81 in the fly-by
   turns before the orbit (gate 0.790 / 0.807 / 0.811 at 0.1 / 0.502 / 0.7 kg -- to the hundredth),
   the integral idling at -0.05..+0.5 deg. The 3.4 m was mostly the `k_bank_d` damping term opposing
   the orbit's own turn rate (8.9 deg of bank), not sideslip; kappa 0.80 is the plant's true turn
   rate per bank. Scheme, laws, equilibrium, gate table:
   https://claude.ai/code/artifact/795b1179-6d55-49ff-907c-a03a548589bb
3. **`rth_brake_margin` 15 -> 5 m**: stop 24-28 -> **12.6-22.1 m** from home (gate 4-7; the rest
   is the sim's real-time factor on the wall-clock decel ramp, 1.43 m/s^2 wall = 1.66 sim in every
   bag; hardware has none). Landing at home is the safe controller's leg either way.
4. **0.1 kg FLOWN (first time) and 0.7 kg re-flown**: both the full guided mission, landed 0.00 m.
   The light mode RINGS at the crossing as the gate said it would (revs 4): +10.9 -> +3.6 -> +17.3
   -> +10.1 -> +6.1 -> +13.7 at ~1.2 Hz, z min 29.5, 1 solve failure. Not a margin problem at
   14.33 m/s of stall; the number to carry for 0.1 kg.
5. **`VTOL_TANK_R` flown: the 100 mm tank 0.3 m ahead of the CG, L1 off, full guided mission,
   landed 0.00 m** -- and it is the FIRST HIL FLIGHT WHERE THE SLOSH DID SOMETHING: orbit rho sd
   0.64 / roll sd 2.4 (twice the CG-tank flights), turn overshoots +17.6 (vs 13-16), and at the
   hover end of the RTH stop (V < 6 m/s, aft tilt pinned at its limit) a **~1 Hz pitch ring of
   -3 -> -10 -> -4 -> +11 -> -3 -> -14 -> +1 -> -5 -> -9 -> -17 deg** that took the stop from 10.5
   to 20 s; z 29.8-30.1 throughout. The 100 mm tank's mode is 1.14 Hz. **The gate does not show it**
   (its stop: pitch 7 -> 6 -> 5 -> 3 -> 0, 1/5201). Two hypotheses, separated by flight 6 (below,
   `PEGASUS_SLOSH_DEBUG=24` logs the bob at ~10 Hz): a real bob/pitch coupling once the rotors
   carry the weight, or the sim's finite-difference omega_dot on the offset tank's rotation terms.
   **FLIGHT 6 (row R100_dx3_dbg_a1_234611, same config, bob logged; landed 0.00 m, the ring
   repeated) ANSWERED IT: THE WALL.** In cruise the bob parks at -47 mm (the +1.0-1.4 m/s^2 cruise
   f_x holds it aft, 3 mm off the R/2 = 50 mm wall). The braking's -2 m/s^2 (aft tilt PINNED at
   -25 deg at the low-speed end, where the gate needs -7: the wall-clock ramp asks a 0.9x-RTF
   plant for more braking) throws the 0.41 kg bob across to the +50 mm wall at t+7 s (V 6):
   **F_x = +17.3 N in one physics step** (the inelastic one-step arrest), the outer answers the
   fore-aft jolt with pitch, the bob lifts off the wall and slams again -- the ~1 Hz ring, pitch
   -3 -> -12 -> +12 -> -15 -> -20, the bob parked/bouncing at +30..+50 mm until V < 1; then a free
   +-31 mm oscillation at 1.14 Hz (0.49% damping) with +-0.75 N of fore-aft force the hover
   absorbs at +-2 deg of pitch. Qualitatively a wave slamming the wall; quantitatively the crudest
   possible impact (10-17 N in 1/240 s), which is the model limit written under item 6. A tank
   at the CG never reaches its wall in this manoeuvre (the 41 mm bob peaks 8-11 mm). If this case
   matters: a COMPLIANT wall (stiff spring-damper over a few mm -> finite, physical impact
   forces) in `PendulumSlosh`, and clock the decel ramp on sim time in HIL so the plant is not
   asked for a 1.7 m/s^2 stop it was gated at 1.5 for. Not a controller change.
6. Written down ("WHAT THE SLOSH MODEL CAN CLAIM", below).
7. GS speed checks on the tank mass (15.12 / 15.50 / 14.33; `i_max` 12 N + the ~50 ms transport
   margin stay as recorded).
8. **The ↱ Fly waypoints button was PRESSED IN FLIGHT** (`tests/test_gs_guidance_live.py`: the real
   tab offscreen against the live stack, ▲ TAKEOFF -> 🚀 Auto Mission (Cascade) -> ↱ -> 🏠): the
   node acked `mode=leg`, the legs and the orbit flew (100.0 +- 0.2 m, kappa 0.81 at the entry),
   🏠 brought it home and the mission LANDED 0.08 m from the take-off point and disarmed. All
   checks pass (one race in the test's disarm read, fixed).
**THE GATE GAINED ITS MISSING TERM.** `--outer_lag 0.03` (the outer thread's solve->publish delay)
existed, defaulted to 0 and was never compensated: uncompensated it reproduces the flown +16-19 deg
onset; compensated as `Outer._run` does it is +14. The gate now adds it to `--comp`; `ke_sweep.sh`
carries it; the faithful line is `--ttr 10 --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh`
(16/16 at all three masses). HIL onsets tonight: +16.1 / +17.2 / +18.5 / +17.4 / +16.9 -- the
gate's 14 plus 2-4 deg, every mass; not the bank, not the RTF (both gated).
| flight (row) | water / tank | orbit rho | kappa | RTH arm roll | stop | crossing pk | fails | landing |
|---|---|---|---|---|---|---|---|---|
| g2_int_a1_225908 | 0.502 / 41 mm at CG | 99.9 +- 0.33 | 0.80 | +0.0 | 15.2 m | +16.1 | 0 | 0.00 m |
| kg07_g2_a1_230837 | 0.7 / 41 mm | 99.9 +- 0.57 | 0.80 | +0.7 | 22.1 m | +17.2 | 0 | 0.00 m |
| kg01_g2_a1_231813 | 0.1 / 41 mm | 99.7 +- 0.41 | 0.79 | +0.1 | 17.4 m | +18.5 (rings) | 1 | 0.00 m |
| R100_dx3_a1_232748 | 0.502 / 100 mm, dx 0.3, L1 off | 99.8 +- 0.64 | -- | +0.1 | 12.6 m | +17.4, hover ring | 2 | 0.00 m |
| GS buttons (bag ..233645) | 0.502 / 41 mm | 100.0 +- 0.29 | 0.81 | +0.4 | (GS leg) | +16.9 | -- | 0.08 m |
| R100_dx3_dbg_a1_234611 | 0.502 / 100 mm, dx 0.3, L1 off, bob logged | 99.8 +- 0.61 | -- | -0.0 | 14.3 m | +15.8, hover ring (wall) | -- | 0.00 m |
**DID THE FLIGHTS SETTLE THE LIST? (asked and answered, 2026-09-14 00:10)**
  * SETTLED OUTRIGHT: 2 (orbit 103.3 -> 99.7-100.0 at every mass and from the GS, kappa 0.79-0.81
    learned before the orbit, integral idle), 8 (↱ pressed in flight, landed at home), 4 (0.1 kg
    flown; its +-7 deg crossing ring is now a known number), 6 and 7 (written and wired).
  * SETTLED AS A MEASUREMENT, NOT A FIX: 1 -- the guard works (armed at 0-0.7 deg every time) and
    the flights showed the bank was never the driver: the 0.7 kg crossing peaks ~+17 wings-level.
    The HIL onset sits 2-4 deg above the gate at EVERY mass (+16..+18.5 vs 14) with 0-2 failures and
    no altitude loss: thin, acceptable, and now measured on five flights instead of one.
    3 -- 24-28 -> 12-22 m; the rest is the sim's real-time factor, deliberately not tuned away.
  * REOPENED, AT THE MODEL: 5 -- the 100 mm tank 0.3 m ahead flew and landed twice, but its bob is
    thrown onto the R/2 wall by the braking and the inelastic one-step arrest delivers 10-17 N
    impulses that ring the hover end of the stop at ~1 Hz. Not a controller fault: before any claim
    about wide offset tanks, (a) a COMPLIANT wall in `PendulumSlosh` (stiff spring-damper over a
    few mm -> finite impact forces), (b) a decision on clocking the decel ramp on sim time in HIL
    (the wall-clock ramp pinned the aft tilt at -25 deg and threw the bob; the gate at RTF 1 needs
    -7). Six of eight closed, one closed as a number, one moved from the controller to the model.
**THINGS THAT BIT TONIGHT:** the driver's `--cruise-time` (default 70 s) expired 55 s into a plan
whose rth was at t+150 and the driver forced `decel_now` itself, in a 26 deg bank in the second
fly-by turn (bag ..224823, scored CRASHED by night_metrics' yaw-180 rule; it stopped cleanly and
landed at home) -- `fly_transition.py` now sizes the window to the plan's last command + 150 s;
the 17:45 command lines omitted `CRUISE_T` and the previous session had passed it by hand. `/tmp`
was wiped by the reboot and took the scratch analysers with it -- `scripts/ana_rth_onset.py` is
now in the repo. `PEGASUS_SLOSH_DEBUG` was not forwarded to the Isaac pane (it is now, and an
integer value N prints every N physics steps). 28 CasADi JIT temporaries are still TRACKED in the
controller repo (ignored for new ones; `git rm --cached` when convenient).
**NOTHING IS COMMITTED:** sim, controller and GS repos are dirty and paired. Review, then commit
the three together.

# ★★★★★ WHERE THE BRANCH STOOD (2026-09-13 17:45, wrap-up) -- THE OPERATING MANUAL.
**WHAT FLIES.** The tank aircraft (0.502 kg default, 0.1 / 0.7 kg selectable) flies the full mission
on the fixed plant: climb -> 90 deg cruise -> banked waypoint turns -> loiter -> return-to-home ->
landing at the take-off point, from a script OR from the ground station's buttons. Twelve landings
on 2026-09-13 (bags in `~/flight_bags/slosh`, rows in `results.tsv`, provenance in
`~/flight_bags/LAUNCH_LOG.txt`). The offline gate predicts flight to a tenth of a degree / a metre.
**ONE COMMAND:** `VTOL_TANK=slosh VTOL_TANK_KG=0.502 bash scripts/start_slosh_hil.sh` (stack + GS;
READY only after both mass banners match), then the GS Trajectory tab: TAKEOFF -> Auto Mission
(ctrl = Cascade, alt 30) -> the Guidance row (↱ waypoints / ◯ loiter / ◯ here / 🏠 / →).
Scripted: `VTOL_TANK=slosh VTOL_TANK_KG=0.7 NAME=x GUIDE="15:wp 500 0 500 400 100 400;150:rth"
bash scripts/run_slosh_flight.sh`. Score a turning flight with `scripts/ana_guided.py <bag>`;
gate a change with `scripts/ke_sweep.sh <tag> 1 16 --ttr 10 --lat 0.03 --comp 0.049 --slosh` +
`scripts/ke_metrics.py`. Tear down with `scripts/kill_slosh_hil.sh` (from outside tmux).
**REPOS:** sim `slosh`, controller `slosh` (fsc_autopilot_ros2-master), GS `vtol-cascade-work`
(ros2_ground_station_gui) -- paired; baseline = tag `pre-slosh` in all four repos.
**OPEN ITEMS, in the order I would take them:**
1. **Heavy mode's back-transition margin.** 0.7 kg crosses its 15.50 m/s stall at +16.6 deg with
   3 solve failures when the decel starts in a bank (flown); the gate only ever starts it
   wings-level. Gate `--decel_at` with `--roll0`/a turn in progress; the RTH could also refuse
   to arm the decel until |roll| < 3 deg (one line in `Guidance.update`'s rth branch).
2. **The orbit needs 17 deg of bank where 14 is coordinated** (both masses, gate and flight): a
   standing 3.4 m radius error carries the difference. An integral on rho, or an adapted bank
   feed-forward, closes it; the cause is no rudder coordination (sideslip in the turn).
3. **RTH stops 24-28 m from home** (gate 13): the flown decel is ~1 s longer than the schedule at
   the onset. `rth_brake_margin` is the knob; the geometric leg covers the rest today.
4. **0.1 kg was gated (16/16) but not flown**; 0.7 kg has ONE flight. Repeat before any claim.
5. **Slosh has never been made to bite in HIL**: every flight was the 41 mm tank AT the CG (the
   plan's benign case; 0.26 N, no moment). The scene has no radius knob (the gate has
   `--slosh_radius`); add `VTOL_TANK_R` and fly the 60-100 mm tank at `VTOL_TANK_DX=0.3` on the
   turning mission, L1 off, to see the largest moment this model can make (gate: absorbed).
6. **The model cannot make impacts**: the equivalent pendulum with an inelastic wall is the linear
   first mode; breaking-wave / impact loads of a half-full wide tank are outside it (plan phase 5,
   a particle fluid, deliberately not built). Say so in any claim about "sloshing".
7. **Hardware:** transport margin is ~50 ms at kR 3 (faithful gate: mission lost at 80 ms, L1 CG
   hover at 50); `i_max` 12 N on the safe controller is a safety-relevant widening; the GS still
   quotes the 14.13 m/s baseline stall in its speed checks (tank: 15.12 / 15.50).
8. **The GS ↱ Fly waypoints button has not been pressed in flight** (the same protocol string was
   flown from the driver; ◯ Loiter and 🏠 were pressed and flew).
**THINGS THAT BIT TODAY, so they do not bite again:** a node that prints its banner and dies passes
every check that stops at the banner (now: node alive + status-topic ack required); `gp()`
float-casts, never read a string parameter through it; the harness `--ttr` default is 40, not the
node's 10; `pgrep -f <pattern>` in a waiter loop matches the waiter itself; tmux wraps long banner
lines (`capture-pane -J`); a JIT cache key must hash everything the artefact bakes in (nmpc6's did
not hash the mass for three days of tank flights); a scripted flight closes the GS, and a GS
selection during a scripted climb fights the driver.

# ★★★ 2026-09-13 EVENING: THE ORBIT EQUILIBRIUM, THE OUTER LAG, THE RTF, AND WHAT SLOSH CAN CLAIM.
**THE LOITER'S STANDING ERROR, TERM BY TERM** (bank law: roll_ref = wing (k_bank e + k_bank_d
psi_dot_est + bank_ff), k_bank 2, k_bank_d 1, bank_ff = -atan(V^2/gR) = -14.6 deg at R 100 / 16 m/s).
In the steady orbit the aircraft flies its circle's tangent, so the heading error IS the field's
inward offset atan(k_orbit (rho-R)/R); the orbit's own rate V/rho = 0.155 rad/s enters the damping
term as +8.9 deg AGAINST the turn. Flown roll -17.8 = -11.7 (P) + 8.9 (damping) - 14.6 (ff), i.e.
the P term carried 12 deg: 8.9 fighting the damping + ~3.5 the plant genuinely needs beyond the
coordinated bank (no rudder coordination: sideslip, the fin takes part of the lift's horizontal
component). An integral on rho alone therefore pinned at its 6 deg clamp and left 1.7 m
(101.7 at every gain) -- the first thing I saw. `Guidance.turn_rate_ff = dir V/R` now feeds the
damping term (it damps deviations from the orbit's rate; a constant offset, loop dynamics
unchanged): P-only then leaves 0.8 m, and the integral (k_i 0.002, band 10 m so the 14 m entry cut
does not wind it, clamp 6 deg) closes it: 100.0 +- 0.3 in the last 40 s, bank 18.1, entry unchanged.
The status feed prints `Ibank=` so the GS shows the integral's share. Both masses and the offset
100 mm tank give the same 100.0-100.2. THEN THE ADAPTED FEED-FORWARD (asked for next): the plant's
turn-rate-per-bank gain kappa is measurable in any quasi-steady bank, `Guidance.adapt_bank`
low-passes psi_dot_est / (-g tan(phi)/V) (tau 4 s; only |phi| >= 8 deg, |p| <= 5 deg/s, wing >=
0.9; clamped [0.6, 1.3]) in EVERY mode, and the orbit's ff is atan(V^2/(g R kappa)). Sign check
(empirical, as the project's rule demands): FLU roll > 0 = right wing down = right turn = psi_dot
< 0, so the coordinated rate is -g tan(phi)/V and the ratio is sign-free; the gate learned +0.80,
not a clamped 0.6, in both orbit senses (`test_cascade_guidance.py`). Learned 0.809 in the two
fly-by turns before the orbit (t=76), flat at 0.798-0.802 through it; with k_i 0 the orbit sits at
99.9 +- 0.3 -- the estimator alone closes it -- and with both on 100.1 +- 0.3. kappa 0.807 (0.502),
0.811 (0.7), 0.790 (0.1). Default: BOTH on (the integral idles near zero and covers a gated-out
estimator). The scheme, drawn: https://claude.ai/code/artifact/795b1179-6d55-49ff-907c-a03a548589bb
**THE RTH STOP IS THE REAL-TIME FACTOR.** `slow()` is a wall-clock timer; `_v_cmd` ramps down
`dt * decel_rate` per tick; Isaac advances 0.86-0.93 sim-s per wall-s (2026-09-13 measurement), so
the aircraft is asked to lose 1.5 m/s per WALL second = 1.66-1.74 m/s^2 of sim time. Bag ..151254:
mean 1.66 (V 15.94 -> stop in 74 m; V^2/2a = 84.7 at 1.5). The gate (RTF 1) brakes in 82-87 m and
stops 4-7 m short at margin 5. On hardware the ramp and the world share a clock. The mocap topic
is NO help: its header stamps advance 1.0000 s per wall second at 120 samples per wall second
(measured on bag ..151254) -- the emulator publishes on the wall clock and the only sim-time source
in the stack is PX4's own timestamp on /fmu/out/*, which the guidance does not (and should not)
read. Accept the ~10 m in HIL; do not lower the margin for it.
**THE ROLL GUARD'S BUDGET.** At 16 m/s a 5 m margin is 0.3 s; the roll-out from a 22 deg RTH capture
takes ~1 s. `rth_roll_wait_m` 10 lets the arm slip to 10 m PAST the no-margin distance (a stop
<= ~10 m past home, V < 1); the home bearing is latched at the arm so overflying home cannot flip
the live bearing 180 deg (it would have: `hd = atan2(d)` while dist > 3). `rth_roll_arm_deg` 0 turns
the guard off (pre-2026-09-13 evening behaviour). Gate: guard on/off identical at 0.7 kg, 0 failures.
**`--outer_lag` WAS THE GATE'S MISSING ONSET TERM.** 0.7 kg, decel 1.7 (the RTF-scaled rate),
straight: lag 0 -> +13.2 / 0 revs; lag 0.03 uncompensated -> +19.2 / 5 revs (the flown +16.6);
lag 0.03 compensated as `Outer._run` does (latency + solve_ms) -> +14.0 / 0. The node was right all
along; the gate applied `--comp` alone. Now `COMP += outer_lag`. Faithful sweeps (ttr 10, lat 0.03,
comp 0.049, outer_lag 0.03, noise 0.1/1, slosh, 16 seeds): 0.1 kg 16/16 pk 14.4 (revs 4, fails 1),
0.502 kg 16/16 pk 14.1 (revs 2, fails 3), 0.7 kg 16/16 pk 13.7 (revs 1, fails 3).
**WHAT THE SLOSH MODEL CAN CLAIM, AND CANNOT.** The tank is the linear first antisymmetric mode as an
equivalent pendulum (m1, L1 from the Abramson/Ibrahim closed forms), driven by the tank's specific
force including the rotation terms, with an INELASTIC WALL at R/2 where the linear model's validity
ends (the bob parks, its energy is lost, the arrest is the reaction). It can say: how much of the
water moves, at what frequency, the quasi-static lateral force in a turn entry and the moment that
makes on an offset tank, and that at any geometry 41-100 mm / 0.1-0.7 kg / fore-aft 0.3 m this
controller does not notice (0-1 solve failures in 5201, corridor +9.5, BT +12.7-14, rho 100.0).
It CANNOT say anything about breaking waves, roof impacts or hydraulic jumps in a wide half-full
tank: those are non-linear, impulsive, and outside a modal model by construction; the wall is a
bound on the model, not a model of the impact. A particle fluid (plan phase 5) was deliberately not
built -- it would never be in the control path. Any claim that "sloshing is handled" must be
phrased as "the first-mode slosh of a cylindrical tank up to 100 mm at 0.7 kg is below this
controller's notice; impact loads were not modelled." A baffled tank (`TankConfig.baffled()`)
is the hardware answer if either matters.
**GATE TOOLING:** `--decel_roll D` (bank step at the onset), `--rth_margin`, `--roll_arm`,
`--roll_wait`, `--k_i_orbit`, `--i_orbit_max`, `--i_orbit_band`, `--no_bank_adapt`,
`--bank_adapt_tau`; the gate prints the RTH STOP distance, the arm roll, the integral's bank and
the learned kappa (also `kappa=` on every trace line once it has learned).

# ★★★ SO(3) STESO — SUPER-TWISTING EXTENDED STATE OBSERVER AS THE UNCERTAINTY/DISTURBANCE
# ESTIMATOR INSIDE THE CASCADE'S L1 AUGMENTATION (2026-09-13/14). GATED OFFLINE. NOT FLOWN.
# Side-session notes; CLAUDE.md was in use by the slosh session and is untouched. NEW FILES ONLY,
# nothing that flies today was edited, nothing is committed (both trees dirty from that session).

**FILES.** Controller package (fsc_autopilot_ros2-master/px4_vtol_direct_actuator/px4_vtol_direct_actuator/):
`so3_steso.py` (the estimator, numpy only, imports under any python) and `nmpc_cascade_steso_node.py`
(the cascade with it inside L1). Sim repo: `tests/test_so3_steso.py` (gate, ~20 s, PASS),
`docs/so3_steso.md` (full tables). Pages: the estimator alone was not given one; the closed loop:
https://claude.ai/code/artifact/bb0006e0-75aa-4248-8454-332766f86bdf (diagram, laws, gate figures).
Paper: Xie et al., arXiv 2111.02046 (STESO + ARSMC for a tiltrotor). Only the observer half is built;
the ARSMC controller is not — the estimate feeds the inner loop this project already has.

**WHAT IT IS.** The paper's observer (Levant's 2nd-order robust exact differentiator driven by the
attitude error, with the known dynamics fed forward, third state = the LUMPED residual d) rewritten
on SO(3): innovation `Log(R_hat^T R)`, R_hat on the group, the rate fed to the kinematics transported
by `R_e = R_hat^T R` so the error rotation obeys exactly `R_e_dot = R_e [w - w_hat - k1 sig^{2/3}(e)]_x`
(no small-angle assumption, works inverted / tumbling / at 90 deg tilt). Two modes: **"attitude"**
(order 3, R only — the paper's structure; gains 2 L^{1/3} / 2.12 L^{2/3} / 1.1 L) and **"rate"**
(order 2, R + w — what this rig has; 1.5 L^{1/2} / 1.1 L). Plant as it sees it: `J w_dot = M_model +
J sigma`, M_model = the CALLER's own moment model (the cascade's `model_moment`, gyroscopics included),
sigma [rad/s^2] = the residual — CG error, unmodelled aero, actuator mismatch, tank. It is the
residual that is estimated, not the plant, for the same reason L1Aug's docstring gives. ONE knob:
`L` [rad/s^3] per axis = bound on |sigma_dot|. `eps="auto"` = a boundary layer sized to the
discretisation chatter (1.5 k3 dt^3 / 1.0 k2 dt^2), `tau_out`, `tau_bias` (splits sigma into the
quasi-static "uncertainty" and the "disturbance"), `delay`, `hold`, `sigma_max`. Outputs `sigma`,
`torque = J sigma` [N.m], `R_hat`, `omega_hat`, `e`.
**THE PAPER'S EQ. 12 HAS THE CORRECTION SIGN FLIPPED** relative to its own error dynamics (eq. 13);
eq. 13 is what converges. Checked empirically (the project's rule): +1 N.m the model does not contain
reads +0.9999 N.m on the right axis, other axes ~1e-5, both modes.

**WHAT THE ESTIMATOR ALONE MEASURED (tests/test_so3_steso.py).** Noiseless 1 kHz, assembled inertia:
exact to the chatter 1.1 L dt, a 1 N.m step re-acquired in 0.2 s, 120 deg initial error on a body
tumbling at 1.1 rad/s converged in 2.2 s, and the SAME estimate from a level body at rest and an
inverted one tumbling. Plant double, hover, 5 cm CG error the controller is not told about: reads
+1.938 N.m (true 1.938) both modes, within 1% with 30 ms delay + real-mocap noise; fed forward it
takes a P-D hover's standing error (12 deg of pitch, or 1.08 m of position under a position hold —
the position loop tilts R_d to fight the drift and the two standing errors cancel in pitch) to 0.00.
**★ BUT AT 120 Hz UNDER REAL-MOCAP NOISE IT IS AT PARITY WITH A LINEAR ESO TUNED TO THE SAME NOISE
FLOOR** (1 Hz moment tracked within 0.97-1.11x at any amplitude inside L) **AND SLOWER ON A STEP**
(its third channel slews at most 1.1 L per second: 0.12-0.15 s vs 0.06-0.12). The finite-time
exactness is a noiseless, high-rate property. Do not quote it for the rig. L below the disturbance's
slope is a hard failure (2.25 rad/s^2 of lag at L 5 against a 16.6 rad/s^3 slope) where the linear
one merely lags. Noise floor at 120 Hz, pitch, L 20: attitude mode 0.16 (0 noise) / 0.42 (0.1 deg);
rate mode 0.04 / 0.27 (1 deg/s). HIL's 0.0004 deg jitter is invisible under the chatter. The
Taylor-consistent attitude increment (Hanan/Jbara/Levant) changes nothing — the chatter is the sign.

# ★★ THE CASCADE VARIANT: `nmpc_cascade_steso_node.py` — controller_id `cascade` (a REPLACEMENT,
# never alongside the base node: two publishers on one actuator topic).
**A VARIANT, NOT A COPY** (the tank node's argument: a 2650-line copy is a second plant model, a
second allocation and a second gate that stop receiving fixes the day they are made). It imports the
base node AND the tank node (so it is the tank cascade), subclasses `L1Aug` as `L1Steso` — same
C(s) = w_c/(s+w_c) at 2 rad/s, same clamps 2.5/2.5/1.5 N.m, same freeze while saturated, same HOLD
through the back-transition, same lag-aware `applied_moment` (spin 0.12 s / servo 0.03 s) and the
same transport-delay buffer; ONLY the predictor + piecewise-constant law are replaced by the STESO
stepped on (R, w, M_applied in force now) — and re-points the base's two module-level seams
`base.L1Aug` and `base.inner_step` (the ROS node and the offline gate both look them up by name).
`--est pwc` / `CASC_EST=pwc` is **bit-identical to the tank node** (verified trace-for-trace; only
the wall-clock solve-ms column differs). Knobs: `--est steso|pwc`, `--steso_L 20` (or `r,p,y`),
`--steso_mode rate|attitude`, `--steso_eps auto`, `--log FILE` (per-tick CSV of roll, pitch, sigma,
u_ad, innovation, hold); env `CASC_EST STESO_L STESO_MODE STESO_EPS` for the node. Gate exactly as
the tank node: `python3 nmpc_cascade_steso_node.py --test --l1 ...` from the controller repo root.
The launchers STILL START `nmpc_cascade_tank_node` — flying this is a one-line launcher change
(those files were open in the slosh session).

**GATE, faithful plant (--ttr 10 --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1
--noise_rate 1.0, momentum + spin lag, pendulum tank), same seed per pair:**
| case | base L1 law (as flown) | STESO |
|---|---|---|
| hover, 5 cm CG error: pitch / sigma_y read (true 1.938 N.m) | -0.59 deg / 1.881 (97%) | **-0.27 deg / 1.938 (100.0%)** |
| same, raw sigma sd under the gate's sensor noise | 1.53 N.m | **0.06 N.m** |
| no augmentation, for scale | -12.0 deg, tilt parked at -19 as a crutch | — |
| full mission, flown 2.6 cm CG error (climb, corridor, cruise, decel at 60 s) | stops at z 30.0, 0/1701 | stops, 0/1701, SAME trajectory (cruise pitch 7.03 vs 7.03) |
| turning mission, 100 mm tank 0.3 m ahead of the CG (3.6-4.3 N.m at its wall) | 0/4001, RTH stop 5.7 m | 0/4001, RTH stop 5.6 m; sigma shows the 1.14 Hz tank yaw at +-0.5 N.m |
| full mission, 5 cm, cold start into the corridor (ttr 10 AND 20) | CRASH t 10-14 s | CRASH t 10-14 s |
**Same u_ad, same flight, every case.** The PWC law reads 3% low by construction (its discrete factor
e^{-k_s T} at k_s 6) and its RAW sigma is +-1.5 N.m of amplified gyro noise (k_s 6 on 1 deg/s = ~190x)
that C(s) then removes; the STESO's is exact and 25x quieter. THAT is the only headroom: a faster
C(s) (w_c above 2) is affordable with the STESO where the base law would feed +-1.5 N.m of noise to
the actuators. UNTESTED — it is the gate to run before claiming a fast unmodelled moment (an offset
tank in a turn entry) can be cancelled. As shipped, the tank is below the augmentation's notice with
either estimator (u_ad < 0.1 N.m in the turns), consistent with the branch's "absorbed" finding.
**★ BASE-CASCADE ENVELOPE FINDING (not the estimator):** a 5 cm CG error with the mission started
COLD into the corridor crashes under BOTH laws at ttr 10 and 20 — at t=1 s the aircraft is already at
-28 deg pitch with elevator and rudder saturated, the tilt at -25, while the 2 rad/s augmentation is
still ramping toward -1.9 N.m. In flight the engage hover (settle_s) precedes the transition; the gate
has no such phase. 2.6 cm (the flown error) flies. The earlier "5 cm x 3 seeds all complete" was the
flat plant at ttr 20 (2026-09-12); on the faithful plant it does not hold from a cold start.

# ★★ WHERE THE AUGMENTATION IS APPLIED, AND WHY — the allocation level, not the outer NMPC.
```
M_des  = inner geometric law (roll/yaw attitude, pitch damping on the outer's planned theta)
M_des += u_ad,   u_ad = -C(s) J sigma_hat          <- the STESO's reading
[dT, delta] = B_rot(beta)^-1 M_des                  allocation: rotors vs surfaces by authority at this tilt
```
The outer never sees sigma_hat; its package (Tf, Tr, beta, de, theta_ref) is untouched.
**WHY THIS IS THE RIGHT DEFAULT HERE.** (1) Bandwidth: the inner runs at 200 Hz on every mocap
sample; the outer at 20 Hz behind 30 ms + its solve time through predict6 — a residual cancelled at
the inner is cancelled at the inner's bandwidth, through the outer it gains a 50 ms sample and the
lag; the tank's 1.14 Hz yaw is exactly what the outer cannot touch. (2) The NMPC's problem stays
stationary: every gate certificate (0/2001, corridor peaks, the knife-edge) was earned on a fixed
model; a model that moves with the estimate every tick is what re-triggers the flat tilt-vs-pitch
optimum (finding C9) — this is the L1-MPC architecture in the literature precisely because the
augmentation PRESERVES the MPC's guarantees instead of making it adaptive MPC. (3) Bounded: u_ad has
clamps, a freeze on saturation and a hold; a model parameter has no fence on its consequences.
(4) Noise: C(s) removes the raw estimate's noise before the actuators; inside the model it re-plans
the trim on every sample.
**WHERE THE OUTER WOULD BE BETTER.** (1) The steady state is not clean at the inner level: with the
5 cm error the gate still shows -0.27 deg of pitch AND the outer parking the tilt at -0.4 deg as a
crutch — structural: the package still assumes a balanced airframe, the outer sees a pitch it did
not plan and answers with tilt, the inner cancels the moment the tilt caused; at cruise correcting
pitch with tilted rotors also makes forward force (`fx_budget` exists because that is real). If the
outer knew sigma_bias the package itself would balance: no crutch, no residual angle. "The reference
must be a trim of the ACTUAL aircraft" says the outer owns the quasi-static part. (2) Constraints:
the outer knows thrust limits and the corridor (0.7 kg reaches 93% front throttle there); a standing
residual that eats differential authority is better planned around than reacted to after the
allocation saturates. (3) Force residuals: the STESO estimates MOMENTS only; a mass/lift mismatch
(the 1.245 m/s^2 the wrong water mass left in every reference) can only be absorbed by the outer's
vs/vz channel — the inner has nowhere to put it.
**THE HONEST DESIGN IS THE SPLIT, NOT A CHOICE:** the slow part (`sigma_bias`, tau 2-5 s) into the
outer as a clamped PARAMETER (`qdot = (My + Iyy sigma_y)/Iyy` in build_ode6 AND in predict6's model;
a parameter is not baked, but the ODE change forces one JIT rebuild per mass — hash it, the 3-day
mass bug), the fast remainder at the inner under C(s). Expected payoff, to be measured: hover -0.27
-> ~0 and the tilt crutch gone; cruise unchanged; tank case unchanged (never reaches the outer).
Gate it for hunting (C9) before believing it. Alternative without a rebuild: re-balance the package
after the solve with `trim_split` including sigma_bias — same steady state, the trim's distribution
instead of the allocation's authority split. NOT BUILT: it lands inside `OuterNMPC`/`build_ode6` in
the base node (in use), or becomes a second seam in the variant.

**THINGS THAT BIT, so they do not bite again:** the paper's sign typo (trust the error dynamics, then
the plant); a test harness comparing the estimate (valid at t+dt) against the truth at t — a 2e-3 rad
"floor" and an `inf` convergence time that were the harness, not the observer; `plant_double.CG` is
read at call time by `forces()`, so a CG monkeypatch must stay in force through the run (the first
open-loop "CG moment" of 0.113 N.m was the nominal front/rear arm asymmetry); the mini controller's
`vee(R.T - R)` sign slip tumbled the plant double — same slip as the six pitch inversions; a
super-twisting third channel is slew-limited at 1.1 L (a step takes |step|/(1.1 L) just to ramp);
a "what super-twisting buys" claim written before it was measured had to be rewritten — at the rig's
rate and noise it buys a cleaner reading and one physical knob, not accuracy. Measure, then write.

**TO FLY IT (in order):** point `start_slosh_hil.sh` / `restart_hil.sh` at `nmpc_cascade_steso_node.py`
(same args as the tank node); the 30 s engage hover with `VTOL_CG_DX=0.026` (`CASC_SETTLE_S` as a
DOUBLE), expect -0.8 deg like L1 (parity says not better) with a quieter `sigma=` in the pane log;
then the full guided mission at 0.502 kg; then, only if the w_c headroom is gated first, the 100 mm
tank at `VTOL_TANK_R=0.10 VTOL_TANK_DX=0.3` with `l1_wc` raised. Score with `scripts/ana_guided.py`.

**THE BASELINE IS A TAG IN FOUR REPOSITORIES, NOT ONE.** `pre-slosh` is cut in
fsc_PegasusSimulator (ea74406), fsc_autopilot_ros2-master (88127e2),
fsc_autopilot_ros2_msgs-master (ebeded8) and ros2_ground_station_gui (971fba6);
each tag message pins the other three. Checking out one of them alone does NOT
restore the flying stack. Bags live in `~/flight_bags` (db3 only, no
metadata.yaml -- read them directly, `ros2 bag` will not open them).

**THE MODEL COPY IS A SUBCLASS, DELIBERATELY.** `logic/vehicles/vtol_slosh.py`
(`VTOLSlosh` / `VTOLSloshConfig`) adds the tank to `vtol.py` and nothing else;
the baseline plant keeps ONE definition. A fork would inherit every plant bug
the day it is fixed in vtol.py and not here -- which is this project's most
expensive recurring failure (the FLU/FRD frame, the body-link inertia, CQ at
half value). Freeze a plant by checking out the tag, not by copying the file.

**THE SLOSH MATHS IS ISAAC-FREE ON PURPOSE:** `logic/dynamics/slosh.py`
(`TankConfig`, `slosh_parameters`). `vtol.py` cannot be imported outside Isaac,
so every headless test of it so far RE-DERIVED the maths (see
`test_actuator_models.py`) -- the same drift trap again. The slosh tests import
the shipped module by file path and check the real code. Do not add
`from __future__ import annotations` to it: with a by-path load, @dataclass then
resolves annotations through an unregistered `sys.modules` entry and the import
dies with a bare AttributeError.

**A CONFIGURED TANK REFUSES TO CONSTRUCT** (phase 1 has no slosh physics). It
raises at construction, not on the first physics step, so it can never be
airborne when it is discovered. `tests/test_slosh_scaffold.py` (AST + the
plan's numbers) and `tests/test_slosh_import.py` (real import, Isaac stubbed)
both PASS; the scaffold test is MEANT to fail at phase 3 and says so.

**SELECT IT WITH `VTOL_SLOSH=1`** on scene 03 -- same airframe, `tank=None`, so
it is step-for-step the baseline. Fly the mission that way and compare against
`pre-slosh` BEFORE any tank physics exists to blame. The spawners resolve the
vehicle class from the CONFIG TYPE (`_vehicle_class_for`), one selector, so a
slosh config cannot be flown on the baseline class or the reverse.

**★★ FLOWN, 2026-09-12. THE COPY FLIES THE FULL MISSION.** `VTOLSlosh` spawned in
Isaac and flew climb -> transition -> 90 deg wing-borne cruise -> back-transition ->
**landing 0.00 m from the commanded point at 800 m**, scored from the bag by
`night_metrics.py` (bags + `results.tsv` in `~/flight_bags/slosh`):

| trial | model | verdict | wb_s | peakV | tilt | maxroll | maxyaw | fails |
|---|---|---|---|---|---|---|---|---|
| slosh_p1b_165344 | **VTOLSlosh** | **LANDED** | 47.3 | 16.7 | 90 | 13 | 15 | 0/0 |
| base_p1_170214 | baseline VTOL | **CRASHED** | 47.1 | 16.7 | 90 | 180 | 180 | 0/0 |

Same driver, same 30 m / 800 m / 70 s mission, ONE FLAG APART. Wing-borne seconds,
peak speed and tilt match to within noise (47.3 vs 47.1 s; 16.66 vs 16.64 m/s); the
BASELINE is the one that departed. That is the documented P-only bank-to-turn
scatter, not a difference between the models -- the force path is provably
identical -- but it is worth stating plainly: **one flight each proves the copy is
not worse, and nothing more.** A lateral claim from this stack needs repeats.

**AND THE FIRST ATTEMPT WAS A NULL FLIGHT THAT LOOKED LIKE A CONTROLLER FAILURE.**
A full auto mission ran during stack BOOT (climb, engage cascade, 90 deg cruise,
then a roll departure at -124 deg and the ground at x=351 m). The driver then flew
the WRECK for three minutes, reporting 0.00 m/s and "climb did not settle" -- and
Isaac resurrects a crashed vehicle, so nothing downstream noticed. `night_campaign`
already knew this ("the vehicle must be fit, not just the stack"); the new driver
now refuses unless `vehicle_state.py` says the aircraft is at the origin, upright,
grounded and stopped. **The bag is what told the truth, for the fifth time.**

# ★★★ PHASE 2 FLOWN (2026-09-12): TANK AS DEAD MASS. CRUISES; THE BACK-TRANSITION DEPARTS.
**THE MASS, FROM THE CONFIG'S OWN LINKS** (`vtol_slosh.assembled_mass_properties`): the
baseline assembles to M=3.450 CG=(0.1764,0,-0.0353) I=(0.265,0.303,0.490) -- reproducing the
controllers' hardcoded constants EXACTLY, which is what makes it credible for the tank:
**M=3.9516 kg, CG unmoved (0.00 mm), I +0.2%**. Mounted at the assembled CG the tank changes
ONE thing. `TankConfig.position=None` now means the assembled CG, NOT the body-link origin
(176 mm apart -- the `aero_center` bug in a new coat). Stall 14.13 -> **15.12 m/s**.

**THE VEHICLE MASS HAS FOUR COPIES IN THE CONTROLLER PACKAGE** -- `so3.M`, `FSC["m"]` (the
trim generator), `geometric_hover_node.mass` (the SAFE controller), `vtol_nmpc.M` + its BAKED
TRIM table -- plus `tests/plant_double.py` on this side. Moving only `so3.M` left a **1.245
m/s^2 vertical residual in every baked reference: 0.502 kg x g, the water exactly**.
`tests/test_tank_model_pairing.py` reads all of them and fails on drift; the trim table was
REGENERATED from `nmpc6.Schedule` (hover row 33.84 -> 38.76 N). All at 3.9516 on the
controller's `slosh` branch (paired with this one; baseline = `pre-slosh` in BOTH repos).

**`i_max` 8 -> 12 N ON THE SAFE CONTROLLER -- SAFETY-RELEVANT, REVIEW BEFORE HARDWARE.** Measured
on the plant double: a 6 m/s crosswind needs **9.38 N** of integral force at 3.95 kg; at the 8 N
clamp it PINS and the aircraft parks 0.46 m downwind forever (0.004 m at 12). The 8 N value was
this same finding one payload ago. Anti-windup is clamp + back-calc + freeze, not bare.

**GATES AT 3.9516 kg:** reference audit all pass; cascade audit |qdot| 0.051, az 0.000; 26/26
geometric-hover; offline cascade gate 89.7 deg / 16.00 m/s / **0/1801 fails**, cruise AoA
**+7.1 (was +5.7)**, rotors 10.0 N (was 8.1). **Corridor peak AoA 12.4 deg vs the CL peak at
10.0** (baseline 10.7): ~10 s past the peak at t=20-32 s. Held altitude offline; that is the
back side of the curve where flights 12/13/15 were lost. A quicker `ttr` lowers it (CLAUDE.md:
ttr 6 -> 6.7 deg) at the cost of the lateral clean window. Not retuned -- a mission-design call.

**HIL (bag slosh_20260912_173754, results.tsv row tank_p2_173901): CRUISE EXCELLENT, BACK-TRANSITION
CRASHED.** ~50 s of 90 deg cruise at V 16.3-16.6, z 30.0-30.1 FLAT, AoA +5.6..+7.2, wing
105-119%, roll +-1, **fails 0 throughout**. Then, from the cascade's own log:
```
V=16.41 vcmd=16.00 tilt=90.0 th= +6.0   fails=0      cruise
V=15.55 vcmd=15.13 tilt=64.6 th=+11.7   fails=0      back-transition begins
V=14.60 vcmd=13.63 tilt=35.6 th=+19.2   fails=0      BELOW the 15.12 m/s stall, nose +19
V= 7.75 vcmd=12.13 tilt= 8.3 th=+56.0   fails=18     lost 7 m/s in ONE second; solver failing
V= 3.49                     th=+36.3   fails=36  -> th -57, +83, roll +177: tumble, ground
```
The SAME schedule on the no-tank copy 40 min earlier (row slosh_p1b): V 16.3 -> 3.95 over 8 s
= the commanded 1.5 m/s^2, AoA never above +18, tilt walked to -6.8 aft, **landed 0.00 m**.
**MECHANISM: the flare pitches up at V~15 either way; the no-tank aircraft has 0.9 m/s of stall
margin there and the tank aircraft has -0.5.** A stalled wing drops its lift, the schedule holds
altitude by pitching MORE, and it runs to +56 in a second. The plan's +0.88 m/s cruise margin
is exactly what the back-transition's first second consumes. This is the "decel_rate 1.5 vs
implied -8.68 m/s^2" observation already recorded above, made unavoidable by 0.5 kg.
**PHASE-2 GATE IS THEREFORE NOT MET AS SPECIFIED.** Options, none taken yet: less water
(0.25-0.3 kg -> stall 14.6, +1.4 m/s); raise `vcruise` (check the momentum thrust model
leaves headroom at 17-18); or fix the back-transition to keep the wing above ITS stall until
the aft tilt is braking -- the underlying defect, and the durable fix.

# ★★★★ THE TANK AIRCRAFT FLIES THE FULL MISSION -- WITH A RE-SEQUENCED BACK-TRANSITION
# (2026-09-12, bag slosh_20260912_182643, `nmpc_cascade_tank_node.py`). LANDED 0.00 m.
**THE FIX IS SEQUENCING, NOT GAINS.** From the crash bag at full rate: at decel onset the
aircraft pitched +7 -> +18.5 in 0.4 s against a reference rate-limited to ~3 deg, then a
~2.5 Hz pitch oscillation that GREW. Offline at 5 Hz with 0.1 deg noise the outer solver's
own planned pitch HUNTS +7..+22 (finding C9's flat-optimum hunting, on pitch), seeded by a
flare that asks trim+15 = 22 deg of a wing that peaks at 10. **Capping the flare alone made it
WORSE and reproduced the crash exactly**: the base schedule sends the front pair AFT from the
first tick, while the wing is what carries the weight; the tank aircraft's wing quits at
15.12 m/s with the tilt still at 60-70 deg, the outer has no feasible way to hold altitude,
its solves fail, the fallback freezes the tilt, the nose ratchets to +88.
**Schedule now: rotors UPRIGHT first (60 deg/s); aft tilt only below 60% wing share (the
trim split then gives the rotors only the wing's deficit -- no balloon); pitch capped at
12 deg + (1 - wing share) x flare.** Offline with noise, seeds 1-3, noise 0.3/2, latency
0.05/comp 0.049: peak pitch +13..+18, altitude >= 29.3, stops every time, <= 10 fails/1701.
**BOUNDARY: decel_rate 2.5 crashes it** (1.5 flown). Flown: 17.4 -> 3.3 m/s over 9 s,
altitude 28.8-30.1, pitch <= +13, rotors upright at V=13, aft tilt at V<7, 0 solve fails
after the onset, hover, 1 m/s descent, **touchdown (800.00, 0.00) level**.

| flight | aircraft | cascade | verdict | wb_s | peakV | maxroll | maxyaw |
|---|---|---|---|---|---|---|---|
| slosh_p1b | copy, no tank | base | LANDED 0.00 m | 47.3 | 16.7 | 13 | 15 |
| base_p1 | baseline | base | CRASHED (roll, in cruise) | 47.1 | 16.7 | 180 | 180 |
| tank_p2 | tank 3.95 kg | base | CRASHED (back-transition stall) | 48.2 | 16.7 | 180 | 180 |
| **tank_seq** | **tank 3.95 kg** | **tank** | **LANDED 0.00 m** | 48.4 | 19.3 | 35 | 99 |

**CAVEATS, stated plainly.** (1) ONE flight. (2) The cruise carried the documented P-only
bank-to-turn limit cycle at +-29 deg roll / +-23 m cross-track / V 16.6-19.3 (banking raises
speed; 19.3 is where the 12x6 prop saturates). Run-to-run scatter of the marginal lateral
mode -- the previous tank flight cruised at +-1 deg on the same config -- NOT the tank or the
schedule, but it is the open defect and the tank plan's phase-7 note stands. (3) The 99 deg
yaw was the low-speed cross-track chase; `night_metrics.py` scored it CRASHED via
"maxyaw>90 and reached the ground" until that rule learned to look at HOW the ground was
reached (slow, upright = landing). Every earlier verdict re-scored unchanged.
**L1 ADAPTIVE IS NOT NEEDED FOR THE DEAD-MASS CASE**: mass paired into the model + the
schedule re-sequenced covers it. Where it may matter is phase 3, when the slosh moves the
effective CG dynamically. Deliberately not started.

**THE STACK DIED MID-FLIGHT TWICE TODAY (16:48, 18:20), every pane in the same second = the
tmux server, cause unknown; and a GS selection made during a scripted climb fought the
driver at 1 Hz over active_controller** (the GS re-publishes its last selection forever,
`trajectory_player.py:1894`). Ops answer: `run_slosh_flight.sh` = teardown -> launch ->
verify -> close the GS -> fly SUPERVISED (PX4 + Isaac alive, mocap changing; dead or
frozen = exit 2) -> score -> teardown, retrying on exit 2. One command; it does not need
anyone to diagnose why the stack died to get a result.

# ★★★ LATERAL LIMIT CYCLE: FIXED IN FLIGHT BY HEADING-RATE DAMPING ON THE BANK COMMAND
# (2026-09-12, bag slosh_20260912_194108). k_bank_d, declared since the noise campaign, was
# INERT -- "a yaw-rate term does not fix it" was measured on a knob the law never read.
**MEASURED, from the ±29 deg flight (bag ...182643) at 120 Hz:** the guidance loop oscillates at
**0.9 rad/s (7.1 s period), growing x1.8 per cycle (+0.08/s)** until the 25 deg bank limit bounds
it; yaw a quarter period behind roll, cross-track a quarter behind yaw, ailerons +-0.05. That is
exactly the ideal loop's sqrt(g*k_bank*k_ct) = 0.99 rad/s; the DAMPING is not (ideal zeta 0.6,
flight unstable) -- the real roll->heading path carries ~60 deg more lag at 0.9 rad/s than the
model, and a P-only command cannot supply phase. **The offline gate does not reproduce the mode
at all** (worst roll 0.5 deg over 200 s with noise, 1.8 with a 15 m seed): it was designed from
the bag and validated in flight. The open-loop lateral modes at the trim (CasADi Jacobian of
`so3.build_ode`): dutch roll 0.78 Hz zeta 0.21, roll subsidence 0.2 s, spiral -0.08/s -- the
7 s mode is the GUIDANCE loop, not the airframe.
**THE FIX:** `roll_ref = wing * (k_bank*e_yaw + k_bank_d*psi_dot_est)`, `psi_dot_est =
r/(cos(phi) cos(theta))` -- NO pitch-rate term, which is what turned the two earlier turn-rate
rewrites into pitch-into-roll coupling. k_bank_d=1.0 s = 24 deg of lead at 0.9 rad/s.
**CRUISE-ONLY: off the moment decel starts.** Left on through the back-transition it cost 1-2
of 6 seeds on the gate; and that gate showed the back-transition's stall crossing (V 15 -> 12)
on a KNIFE-EDGE: runs identical to 0.1 deg at t=61 diverge to +84 or hold by t=62, ~1 in 6-8
under noise + a 10 m cross-track history, whichever lateral gains. That margin is the open item.
k_ct 0.03 tried (lower loop frequency) and cost a seed: left at 0.05.

| flight | maxroll | cruise \|roll\| | cross-track | V |
|---|---|---|---|---|
| tank_seq (P-only) | 35 | ±29 growing | ±23 m | 16.6-19.3 |
| **tank_lat (+k_bank_d)** | **7** | **±5.5 flat (5.1/5.5/4.3 per 20 s)** | **6.3 m** | 15.5-16.7 |
Both LANDED 0.00 m. One flight each.

# ★★★ PHASE 3 — THE PENDULUM IS IN THE PLANT (2026-09-12). Tests pass; NOT YET FLOWN.
`dynamics/slosh.py::PendulumSlosh`: 2-DOF pendulum in the body frame driven by the vehicle's
SPECIFIC FORCE `f = R^T(a - g_vec)`; restoring "gravity" is the AXIAL specific force (`f_z/L1` =
omega1^2 at 1 g; free fall has no mode, 2 g rings sqrt(2) faster). **The force applied is
`-m1 * x1_ddot` ONLY** -- the whole liquid stays folded as dead mass (phase 2), and the bob's
motion adds just what a moving bob does beyond a frozen one, which is identically zero in every
steady state (hover, banked turn, constant acceleration) -- no double count, tested. Semi-
implicit Euler at the physics step. `tests/test_slosh_pendulum.py`: 3.325 Hz vs closed form
3.323; decay 10 -> 1.31 mm in 20 s as the 0.49% zeta predicts; undamped energy bounded 1.7%
over 200 cycles; steady 2 m/s^2 lateral -> 0 N slosh force, bob at -f/w^2. `_apply_slosh`
applies it at the tank (the assembled CG); `PEGASUS_SLOSH_DEBUG=1` prints [SLOSH] lines.
Select with `VTOL_SLOSH=1 VTOL_TANK=slosh`.
**PHASE 4 DONE:** `nmpc_cascade_node.py --test --slosh [--slosh_zeta Z] [--slosh_fill F]` imports
the simulator's pendulum BY PATH (one pendulum, not two) and adds its reaction to the plant
like the gust. Full mission, pitch kick, 3 m/s^2 gust, roll15+yaw-25 upset: IDENTICAL to dead
mass in every metric; peak surface excursion 6.8-10.8 mm in a 41 mm tank.
**FLOWN (bag slosh_20260912_195718, row tank_slosh_a1_195751): LANDED 0.00 m, the cleanest
mission of the day** -- cruise roll 0.3 deg, cross-track span 0.3 m, z 30.0-30.1, back-
transition 15.5 -> 3 m/s in 8.2 s with z min 29.6 and roll 0.1, 0 solve failures. The 0.1 kg
mode at 3.3 Hz is below this controller's notice, as the plan expected. ONE flight; the
launcher's phase-3 banner gate passed (it exits 1 otherwise) but the banner line itself was
not captured -- it is printed explicitly from now on. **L1 ADAPTIVE: NOT NEEDED** at this
tank size. Open: the back-transition stall-crossing knife-edge (~1 in 7 under noise + a
cross-track history, any lateral gains) -- widen that margin before more water.

# ★★★ L1 ADAPTIVE AUGMENTATION (insurance) — BUILT, PROVEN OFFLINE, AND IT EXPOSED A HOVER
# FRAGILITY THE GATE CANNOT SEE (2026-09-12 late). `L1Aug` in the cascade node, `l1_on` OFF.
**WHAT IT IS.** Predictor on body rates, piecewise-constant adaptation each inner tick,
`u_ad = -C(s) sigma_hat`, C = w_c/(s+w_c), added to M_des BEFORE allocation so the outer's trim
package is untouched. It adapts to the RESIDUAL of the controller's own moment model (rotors
via B_rot at the actual tilt, aero via aero_body, gyroscopics) -- not to the plant -- so it
cannot cancel the aero stiffness the outer plans with. w_c = 2 rad/s (above the 0.9 rad/s
guidance loop, below the 8.9 rad/s short period and 21 rad/s slosh). HOLDS through decel
(adapting on the stall crossing tipped it: +82 deg vs +19).
**TWO THINGS THE FIRST VERSION GOT WRONG, both found on the gate:** the predictor must be
driven by the command the plant is APPLYING (delay-aware; a fresh-command predictor
cancelled moments that had not arrived -- a growing 0.8 rad/s front/rear oscillation with
the outer's package steady), and the discrete PWC law estimates e^{-k_s T_s} sigma (86% at
k_s 30 / 200 Hz -- the residual seen); k_s 6 -> 95-97%.
**THE TEST BED:** `so3.build_ode(rpos=)` lets the gate's PLANT disagree with the controller
about the CG (`--plant_cg_dx`), and `VTOL_CG_DX` does the same in Isaac. Note a 3 cm TANK
offset moves the assembled CG only 3.8 mm (0.15 N.m, ~1.5 deg): the tank's mount is NOT a
driver at this mass. The AIRFRAME CG error is the real case (it has been wrong by 2 cm twice
in this project's history) -- and hardware's.
**OFFLINE (gate): PROVEN.** 5 cm forward, hover: -16.3 deg standing -> -1.1 (k_s 4..8);
learned moment -1.90 N.m in hover (true 38.8 N x 0.05 = 1.94), falling to -0.28 at 90 deg
tilt = the rear pair's 5 N on that arm -- it tracks the physics through the transition.
Full mission at 0/2/5 cm x 3 seeds all complete; upsets/gust unchanged; knife-edge gate
13/16 vs 14/16 = neutral.
**HIL: PARTIAL, THEN THE REAL FINDING.** Airframe CG +2.6 cm, controller told nothing:
  2 s engage hover (default settle_s): L1 off -6.7 deg standing; L1 on -5.1 -> -2.3 still
  converging when the transition took over. Both LANDED 0.00/0.01 m. ~half the gate's effect.
  **30 s engage hover: DIVERGES IN ~4 s WITH OR WITHOUT L1** -- pitch -0.5 -> -10 -> -69 deg,
  solves failing, ground by t+8 (off); -0.8 -> -4.5 -> -46 -> -65 (on). The gate holds the same
  error at a bounded -16 deg. L1 at w_c 2 converges in 2-3 s; the divergence starts inside 2.
**WHY THE HOVER IS THAT FRAGILE (structural, not a gain):** the inner's pitch error is
against the outer's PLANNED pitch, which starts from the measured state every tick. With a
wrong model the plan drifts with the aircraft and the inner tracks the drift: the pitch
attitude loop is effectively OPEN at low frequency in hover. The offset moment scales with
thrust, the outer answers the sinking altitude with more thrust, and that is positive
feedback. The safe controller (attitude error against a FIXED reference, 12 N integral)
climbed through the identical error without trouble in every one of these flights.
**RESOLVED (2026-09-13 00:00): STIFFEN THE HOVER PITCH LOOP, THEN LET L1 TRIM.** The inner's
pitch reference IS the schedule's (the plan-tracking dead end was found before) -- the loop
was not open, it was SOFT: kR_pitch 3.0 leaves 1 N.m as ~19 deg of static error and the
outer's tilt-and-thrust answer to the drift is positive feedback. `kR_hover` (4.5, weighted
by 1 - wing share, so the validated 3.0 returns by cruise) is the structural half; 9 (the
safe controller's) oscillates the cascade's inner at the gate latency. The SAME 30 s hover,
2.6 cm error, four configurations:
| kR_pitch in hover | L1 | hover pitch | outcome |
|---|---|---|---|
| 3.0 | off | -0.5 -> -10 -> -69 in 4 s | CRASHED |
| 3.0 | on | -0.8 -> -4.5 -> -46 in 4 s | CRASHED |
| 4.5 | off | **-5.1 steady** (tilt parked at -9 as a crutch) | LANDED 0.00 m |
| 4.5 | on | **-1.0 mean**, tilt +-3, 0/521 solve failures | LANDED 0.00 m |
Gate predicted -0.85 for the last; HIL -1.0. Structure buys stability, adaptation buys the
bias -- the division of labour the design was written for. **COST, recorded: the latency
ceiling.** kR_hover 0 flies lat 0.15 and 0.18 clean; 4.5 and 6 crash at 0.15 (4.5 survives
0.18 with 42 fails -- chaotic in that band). HIL measures 49 ms compensated, 2.5x margin.
The rule "soften kR if HIL measures more than ~150 ms" is now ~120 ms. Knife-edge seeds
8/8 with kR_hover 4.5 (was 7/8); upsets unchanged. `l1_on` still defaults OFF; `CASC_L1=true`
flies it. L1 is the insurance policy it was meant to be -- proven on a mismatch the
controller could not see, offline and in flight.
**RELAXING THE OUTER WAS TRIED AND MEASURED (CASC_WTH=45): DO NOT.** Hover with the offset:
no gain (L1 does the job either way; without L1 the gentler outer leans on tilt as a crutch,
-22.7 vs -9 deg); corridor peak AoA +14.2 vs +12.4; and it COSTS latency margin (0.18 s:
survives at 90, crashes at 45). The outer is delay-compensated and the inner is not, so a
strong outer pitch weight keeps pitch on the compensated loop through the latency. The
outer's aggressiveness supplies the margin. Weights stay 90/60/8; `CASC_WTH/WZ/WVS` exist
to repeat the experiment in one env var.
**LATENCY CEILING RESTORED (2026-09-13): `kR_hover` IS SCHEDULED ON `latency_s`** -- full 4.5 at
<= 60 ms (HIL 49), fading to the validated 3.0 by 120 ms. Five seeds: fixed 4.5 was 4/5 at
150 ms and 3/5 at 180; scheduled, 5/5 at both; the 5 cm offset hover at the HIL-equivalent
50 ms is still -0.82 deg with L1. **An inner-loop delay predictor (`InnerPredictor`, the
outer's predict6 idea) was built and MEASURED NON-MONOTONE**: clean to ~60 ms of horizon (past
that the 8.9 rad/s short period outruns a constant-moment extrapolation), and as margin 60 ms
of it gives 0/5 at 150 ms and 5/5 at 180 -- a phase interaction. Off (`inner_comp_s` 0); do not
enable expecting margin. On the gate the schedule reads `--lat`, so test the offset hover at
`--lat 0.05`, not the pessimistic 0.10.
`CASC_SETTLE_S` holds the engage hover for this experiment (force a DOUBLE: `ros2 param set`
exits 0 on a rejected int -- that cost one flight tonight).

# ★★★ THE BACK-TRANSITION KNIFE-EDGE, CLOSED (2026-09-13): THE FLARE WAS RELEASED WHILE THE
# WING STILL CARRIED THE AIRCRAFT. `nmpc_cascade_tank_node.py` aoa_cap 12 -> 8, flare gated on
# rotor load (W_FLARE = W_AFT = 0.6). `tests/test_tank_schedule.py` pins it.
**REPRODUCED FIRST, 16 SEEDS:** the flown schedule at 0.1 deg / 1 deg/s noise + a 10 m cross-track
history loses seeds 6 and 14 (14/16), 12/16 at 0.3 deg, 9/16 at 150 ms latency -- every loss at
V 15 -> 12 with the crash bag's signature: pitch 7 -> 14 -> 7 -> 20 -> 49 in 0.6 s, elevator
-0.5 -> +0.8 -> -1.00, solve fails at AoA 22, the fallback holds the elevator hard nose-up.
**THE MECHANISM IS THE REFERENCE, NOT THE LOOP.** The cap released the flare as `(1 - wing)`, so
at 14 m/s (wing share 0.86) the commanded pitch was already 14 deg and at 12 m/s 17.6 -- past the
10 deg CL peak while the wing carried 60-90% of the weight. The outer's model has that lift curve;
past the peak its pitch-vs-lift gradient flips and the solution alternates between following the
pitch reference and holding altitude: a +-7 deg ring at ~1.2 Hz, wing share bouncing 88 -> 8 -> 73%
in 0.5 s. **Zero latency does not cure it** (pitch error rms 7.3 deg at lat 0 vs 8.2 at 0.10) --
so it was never a latency or gain problem, and no lateral gain could touch it. Evidence that the
NMPC never wanted the flare: at decel_rate 1.0 it ignores an 18 deg reference and flies the
crossing at 6-8 deg, smoothly, with the drag doing the braking; with the reference ramped at
2 deg/s it settles at 9-10 and never follows it higher.
**FIX:** the flare is released where it works -- once the rotors carry 40% of the weight, on the
aft-tilt gate ("the flare and the aft tilt are two halves of one manoeuvre", full_tr35) -- and the
wing-borne cap is the trim schedule's own 8 deg. Stop time unchanged (8.8 s: at 1.5 m/s^2 the
braking at the crossing was drag + T sin(theta) at the trim pitch all along).

| 16 seeds, y0 10 m | noise 0.1/1 | noise 0.3/2 | lat 0.05 (HIL) | lat 0.15 |
|---|---|---|---|---|
| flown: cap 12, release (1-wing) | 14/16, revs 6.3, fails 113 | 12/16 | 16/16, revs 5.6, fails 65 | 9/16 (6 at the crossing) |
| gate only (cap 12, W_FLARE .6) | 15/16 | 12/16 | 16/16, revs 1.1 | 11/16 (3 at the crossing) |
| cap 10, old release | 16/16, revs 3.9, fails 92 | 16/16, revs 4.8 | -- | -- |
| **gate + cap 8** | **16/16, revs 0, fails 0** | **16/16, revs 0, fails 0** | **16/16, revs 0, fails 0** | **14/16, 0 at the crossing** |
(caps 9 and 10 with the gate: the same to within a seed.) "revs" = pitch reversals > 3 deg through
the crossing, "fails" = outer solve failures summed over the 16 back-transitions. **Both halves are
needed**: the gate alone still hunts at 12 deg, the cap alone survives but rings. The two losses
that remain at 150 ms are the post-stop HOVER at 3x the HIL latency (elevator bang-bang at
V = 0.6 m/s, t = 82 s) -- the documented latency ceiling, not the crossing.
**REGRESSION on the final config:** full mission 0/2001 solve failures, BT peak pitch 11.2, stop
8.8 s, z 29.7-29.9; identical with L1 on and with the pendulum; lands with a 5 cm CG error
(2/2001); hover-with-offset -0.82 deg, upsets/gust/latency rows unchanged (the forward schedule
IS base.build_refs, by construction and by test).
**METHOD NOTE:** the argf-style flag parser takes the FIRST occurrence, so a sweep script that
appends stress flags after its own defaults silently runs the defaults. Put the variable flags
first. Also `--lat` defaults to 0.10 on the gate, which schedules kR_hover to a third of its HIL
value -- run the HIL-faithful rows at `--lat 0.05 --comp 0.049` explicitly.

**FLOWN (bag slosh_20260913_103526, row ke_cap8b, pendulum tank): LANDED 0.00 m.** 0/1434 solve
failures over the whole flight, cruise roll <= 0.3 deg, cross-track <= 0.4 m, back-transition
16.5 -> 2.9 m/s in ~9 s at z 29.4-30.1 with roll 0.0, then the 8-10 deg pitch walk the gate
predicted (V 13.8 -> 3: +13, +7, +9, +10, +9, +8, +7, +6). **ONE THING THE GATE DID NOT PREDICT:
the decel ONSET.** At the first decel tick (V=15.6, tilt 44 deg) the elevator stepped -0.28 ->
-0.98 and the pitch rang ONE 1.2 Hz cycle to +21 deg (AoA 23, past the peak for ~0.3 s), z dipped
to 29.4, then it settled; the gate's onset peaks at 11-13. The difference is the cruise the decel
starts from: HIL holds 16.3-16.6 m/s against a 16.0 command with the FRONT PAIR COMMANDED 0.00
(the gate: 16.00 at 0.14), so at the onset there is no thrust to cut and the outer brakes with
pitch. Why the HIL cruise sits on the reverse-thrust limit is not resolved here -- it is the
kind of gate-vs-HIL gap the next item (the gate's blind spots) is for.

**★ THE FIRST RE-FLIGHT NEVER REACHED THE BACK-TRANSITION -- IT FOUND A LATENT ENGAGE BUG
(bag slosh_20260913_102448, row ke_cap8_a1).** At the first cascade tick the rudder went to
-1.00 with the rotors in a full yaw differential; yaw 0 -> -66 -> -132 -> -160 deg in 1.5 s, the
aircraft accelerated BACKWARDS to 3.7 m/s at 32 deg of tilt, and rolled over at t+3 s. The bag
explains it: the driver engages the cascade and publishes the 800 m target **3 s later**
(t=204.3 vs 207.4); in between the reference is still the climb's hover point (0.02, 0.01),
**4 mm** from the latched p0 (0.024, 0.008), and `track_heading` guarded only a 1 mm baseline.
It computed the bearing of a hover offset (158 deg -- `tests/test_cascade_track_heading.py`
reproduces the number) and the yaw loop flew it, while the 2 s settle gate had already
started the acceleration. Every previous flight passed the same race on the luck of the
millimetre (the hover holds to ~6 mm; settle_s=30 flights let the target arrive in hover).
**FIX in the base node: a track shorter than `min_track` (5 m) is a hover point -- the
latched heading stands, no cross-track error.** The offline gate never saw it because its
target is always far away from the first tick. Lesson, again: the bag, not the verdict.

# ★★★★ THE GATE'S BLIND SPOTS (2026-09-13): THREE PLANT FACTS THE OFFLINE GATE COULD NOT SEE.
# Found by fitting the plant's response to the controller's OWN commanded moments in the bags
# (`scripts/bag_axis_fit.py`), then asking PhysX directly (`application/vtol/09_vtol_inertia_probe.py`).
**1. THE PROP LINKS WERE PINNED AT 100 rad/s AND THE DRIVE'S FIGHT WAS REACTED ON THE AIRFRAME.**
PhysX caps every rigid body's angular velocity at 100 rad/s unless told otherwise; the props are
velocity-driven to 400-750. The drive therefore pushed against the cap forever with torque
D*(omega_cmd - 100) per prop (D = spin_drive_damping 0.01), reacted about each prop's spin axis.
Equal throttles cancel; ANY DIFFERENTIAL does not. Probe, hover throttle, modelled reaction torque
switched OFF: a +-0.20 yaw-diagonal step yawed the aircraft at a CONSTANT 17 rad/s^2 = **8.3 N.m
no controller knew about, 3.3x the modelled reaction torque**. With the cap lifted the same step
gives a -5 deg/s impulse and stops (the physical spin-up reaction; implied prop inertia 7.5e-5 vs the
configured 1e-4). Every flight of this airframe carried it. In hover it is the **rail-to-rail yaw
diagonal at 2.7 Hz** in 5 of 7 cascade hovers (the geometric hover on the same rig: 0.000) --
the bag fits as an effective Izz of **0.11 against the true 0.49** (R^2 0.96) even in the hovers
that happened not to thrash. In cruise the tilted front pair turns the same artefact into a
spurious **ROLL** torque whenever the inner uses front differential for yaw: the lateral limit
cycle's "60 deg of unexplained lag" and the k_bank_d fix were tuned against it. FIXED in
`vtol.py::_unclamp_prop_spin` (cap = 1.5 x rotor_omega_max on every prop link;
`tests/test_prop_spin_cap.py` guards the source, the probe is the ground truth).
PhysX's rigid-body inertia, by contrast, is EXACTLY the assembled 0.265/0.303/0.490 (probe ratio
1.00 on all three axes) -- the earlier suspicion of a mass-property bug is closed.
**2. THE AIRFRAME FLEW rotor_omega_max = 1000 rad/s; EVERY MODEL OF IT SAYS 749.** The VTOLConfig
default was never set for the FSC quad. CT0 is derived so static thrust is 18 N either way -- hover
is identical, which is why it hid -- but the advance ratio is not: at 0.80 throttle and 16.5 m/s the
sim made 3.5 N per front rotor where the controller expected 0.7. That is the 0.45 m/s cruise
overspeed on less throttle than the gate needs, and the harder back-transition onset (+21 deg vs
11-13). Fixed at the airframe (749); `tests/test_prop_pairing.py` holds sim and controller to
the same propeller, including thrust at 16 m/s computed by both sides' own formulas.
**3. THE GATE'S PLANT WAS THE FLAT PROP UNLESS `--momentum` WAS PASSED -- and it printed a
warning nobody read.** Every knife-edge sweep, the L1 campaign and the latency ceilings of
2026-09-12/13 ran on the flat prop (which also has NO rotor spin lag: that branch only ran with
momentum). Default flipped: momentum (with the 0.12 s spin lag) unless `--flat`. On the faithful
plant the schedule finding stands (old flare release: 6/16 lost at the crossing; gated: 0), but the
**hover is different**: kR_hover 4.5 + 50 ms transport + 0.12 s spin lag thrashes at 1 Hz in pitch
and crashes in ~15 s, with or without a CG error, with or without noise, halved kW or not; kR 3
stays bounded; tau <= 0.06 or transport <= 30 ms is bounded. HIL hovered 30 s at kR_hover 4.5 --
but on the un-capped-prop plant, i.e. a different aircraft. Resolved below (item 4): at the
rotor path's true transport delay the gate and the fixed HIL plant agree.
**FLOWN ON THE FIXED PLANT (2026-09-13 11:43 and 11:53, rows fixprop / fixprop_hov30, pendulum
tank): both LANDED (0.01 m).** Cruise now sits at V 16.00 / pitch +7.0 / front throttle 0.88 /
rear 0.31 -- the momentum gate's trim to the second decimal (before: 16.3-16.6 / +6.0 / 0.80).
Back-transition onset +14 deg (was +21; gate 11-13). The 30 s CG-error hover at kR_hover 4.5:
pitch -4.9 steady, yaw diagonal sd 0.05 (was 0.73), yaw fits as Izz 0.71-0.80 -- the physical
spin-up reaction, within 1.5x of 0.49.
**4. THE GATE'S `--lat` DOUBLE-COUNTED THE SERVO.** The 49 ms was measured on the ELEVATOR path,
which includes the 30 ms surface servo; the gate applies `--lat` as pure transport AND adds the
servo lag in the plant. The rotor path's transport, fitted from the bags, is ~30 ms. At `--lat
0.03 --comp 0.049` (transport as fitted, compensation as flown) the momentum gate reproduces
HIL: kR_hover 4.5 with the CG error holds at -5.6 deg (HIL -4.9), kR 3 at -6.4, the full mission
0/2001 solve failures. **THE TRANSPORT-DELAY MARGIN, faithful plant, noise 0.1/1, full mission:**
| transport | 30 ms | 40 ms | 50 ms | 80 ms | 100 ms |
|---|---|---|---|---|---|
| kR_hover 4.5 (flown) | OK | OK (2 seeds) | **CRASH** | -- | -- |
| kR 3 (kR_hover 0) | OK | -- | OK (2 seeds) | crash (post-stop hover) | crash |
HIL is at ~30 ms. **The "150 ms ceiling" of 2026-09-12/13 was a flat-plant number with no rotor
lag and a double-counted servo; it never existed.** kR_hover 4.5 buys ~1 deg of hover bias on the
fixed plant (its reason -- the -69 deg divergence -- was the prop artefact) and costs ~20 ms of
transport margin: RECOMMEND reverting to kR 3 before hardware; left at 4.5 pending that decision.
**L1 on the faithful plant hunts** (max 21-25 deg in the CG hover at 30 ms; crashes at 50): its
predictor models the transport delay but not the 0.12 s spin lag. Do not fly `l1_on` until the
predictor carries the rotor lag. The momentum gate's own hover thrash at kR_hover 4.5 / 50 ms
(above) is therefore a real property of the plant at 50 ms, not a gate defect.
**Real-time factor, measured: Isaac runs at 0.86-0.93 sim-s per wall-s in HIL** (PX4 time vs bag
wall time), so the controller's 49 ms is ~43 ms of sim time and its loops run ~10% faster than
the plant. Small, but it is one more way HIL is kinder than the gate.
**PROBE HYGIENE, learned the hard way:** an open-loop "hover" is not a hover. Equal throttles
carry -0.08 N.m about the assembled CG; with nothing holding attitude the probe tipped 30 deg
in 3 s and measured the WING's static stability at 11-14 m/s as a "3x torque deficit" -- three
times, with three different confounds (climbing into the prop's inflow, residual attitude from
earlier pulses, drift). Balance the split, keep it short, and print v_body at the step.
**HOW TO LOOK NEXT TIME:** `scripts/bag_axis_fit.py <bag>` first -- an I_eff far from the assembled
value with a good R^2 is unmodelled plant torque, and the yaw-diagonal sd tells you if a hover is
thrashing (quiet < 0.02, the thrash 0.4-0.8). Then the probe in Isaac. The gate can only ever
reproduce what its plant contains; when a flight disagrees with it, measure the plant, do not
retune the controller.

# ★★★ ON THE FIXED PLANT: kR_hover REVERTED TO 0 (kR 3), L1's PREDICTOR CARRIES THE ACTUATOR
# LAGS, AND HOVER WITH A CG ERROR IS A SOLVED, GATE-PREDICTED THING (2026-09-13 12:46-13:02).
**THE -69 deg HOVER DIVERGENCE WAS THE PROP ARTEFACT.** Same experiment as 2026-09-12 (2.6 cm
airframe CG error the controller is not told about, 30 s engage hover, pendulum tank), on the
plant with the prop-spin cap lifted:
| config | gate (momentum, lat 0.03 / comp 0.049) | HIL | 2026-09-12 (artefact plant) |
|---|---|---|---|
| kR 3, no L1 | -6.4 deg bounded | **-5.5 steady, LANDED 0.00 m** | -0.5 -> -69, CRASHED |
| kR 3 + L1 (lag-aware) | -0.74 | **-0.80, tilt -1.5, LANDED 0.00 m** | (-46, CRASHED) |
| kR_hover 4.5, no L1 | -5.6 | -4.9, LANDED | -5.1, LANDED |
Rows k3_hov30 / k3_l1_hov30 / fixprop_hov30. The gate now predicts the flight to a tenth of a
degree, which is what a gate is for.
**`kR_hover` DEFAULT 4.5 -> 0.0.** Its reason is gone and it costs transport margin (faithful
gate: 4.5 crashes the mission at 50 ms, kR 3 at 80). Kept as a knob.
**L1's PREDICTOR NOW MODELS THE ACTUATOR LAGS** (`L1Aug.applied_moment`: rotor thrusts through
the 0.12 s spin lag, surfaces through the 0.03 s servo; params `l1_tau_spin`, `l1_tau_surf`).
Fed the fresh command it attributed the lag to sigma and hunted (+-20 deg at 30 ms on the
faithful gate, a crash at 50); fed what the plant applies it holds -0.74 at 30 ms and the
mission flies with it at 50. `l1_on` still defaults OFF -- flown proven, the operator's call.
**AND THE LATERAL LIMIT CYCLE WAS THE SAME ARTEFACT.** The P-only bank-to-turn law
(`k_bank_d` 0) flown on the fixed plant (row kbd0_130338): cruise max |roll| 1.0 deg, mean 0.04,
cross-track within 1.6 m, LANDED 0.00 m -- the identical law on the artefact plant went to
+-29 deg / +-23 m (row tank_seq). The "~60 deg of extra lag in the roll->heading path" was the
front pair's spurious roll torque. `k_bank_d` 1.0 stays on (0.0 deg with it, and heading-rate
damping on a bank command is sound), but nothing depends on it any more.
**THE MARGIN, HONESTLY:** at HIL's ~30 ms of transport everything above is comfortable; the
faithful gate loses the mission at 80 ms (kR 3) and the L1 CG hover at 50. Hardware with more
than ~50 ms of command-to-force delay needs a softer inner loop than this, and the flat gate's
"150 ms" is not a number to quote anywhere.

# ★★ THE REST OF THE LIST (2026-09-13 afternoon): ttr 10, l1_on, 12x8, phase-4 sweeps, tests, scripts.
**CORRIDOR PEAK AoA -> `ttr` 20 -> 10.** On the faithful plant (momentum, lat 0.03 / comp 0.049,
noise, L1) the forward transition's peak AoA is +11.4 at ttr 20 (the flat gate said 12.4), +10.9
at 16, +10.2 at 12, **+9.4 at 10 -- under the 10 deg CL peak**, +8.4 at 8, +8.1 at 6; altitude
29.7-30.0 and 0/2001 solve failures at every one. 20 had been chosen to clear a ~90 s cruise before
the lateral "clean window" closed -- that mode was the prop artefact. Node default, both
launchers and the GS's CASC_OVERRIDES all carry 10 (`tests/test_gs_cascade_params.py`).
**`l1_on` DEFAULTS TRUE** (node + `CASC_L1` in all three launchers). Flown-proven insurance.
**12x8 PROPELLER** (J0 0.72; was the RotorConfig default 12x6, J0 0.60). At the corrected 749 rad/s
a 12x6 makes 4.8 N per front rotor at 16 m/s against a 2.3 N cruise need and at 19 m/s its
maximum EQUALS the need -- nothing for the inner loop's differential in a bank. The 12x8 leaves
5 N there and takes the cruise throttle 0.88 -> 0.78 with every other gate number unchanged
(trim, corridor, back-transition, upsets, latency, CG hover, knife-edge seed 14). Airframe
(`prop="12x8"` on all four rotors), `so3.PROP_J0`, the cascade's `PROP_J0` + `prop_zero_thrust_J`,
the plant double and the GS override all moved together; `test_prop_pairing.py` holds them.
**FOUND ON THE WAY: the gate's `--prop_J0`/`--omega_max` flags re-propped only the PLANT** (locals
in main() while thrust_to_throttle read the module's constants). A "12x8 run" was therefore a
plant/controller mismatch -- 0.45 m/s of cruise overspeed on less throttle, the same signature the
omega_max bug had in HIL. The flags now set the module constants too.
**PHASE-4 SWEEPS (the plan's), faithful plant:** fill 25/50/75/100 %, zeta 0 / 0.5 / 5 %, and a
1 rad/s pitch kick with the pendulum in the plant: every mission identical -- 0/2001, corridor
+9.5, back-transition +12.8, roll 0.5, bob excursion 7.6-8.9 mm in a 41 mm tank. The 0.1 kg /
3.3 Hz mode is below this controller's notice at any fill or damping; plan phase 7 (controller
awareness) is not needed, and phase 5 (a PhysX particle fluid for visualisation) is deliberately
not built -- it would never be in the control path, and the pendulum's displacement is already
printed (`PEGASUS_SLOSH_DEBUG`, the gate's slosh_peak).
**TESTS:** `test_hover_hold.py` failed on a verdict that counted sign changes of a 0.01-amplitude
noise response as "oscillating" -- it had been written for the diagonal thrash, which was the
plant artefact its own plant never had; the verdict is now the diagonal's amplitude, and the
shipped yaw gains pass. `test_gs_cascade_params.py` did not know the GS's third parameter
source (NOISE_ON/OFF) -- fixed. `test_gs_live_flight.py` needs a live stack by design and aborts
cleanly without one (the core dump after it is Qt's teardown). Every other test passes.
**SCRIPTS:** `kill_hil.sh` SIGINTs the bag recorder and waits up to 10 s before its -9 sweep (a
-9'd bag has no metadata.yaml); every bag default is now `~/flight_bags`, not `/tmp/bags`
(`restart_hil.sh`, `start_vtol_smc_hil.sh`, `recbag_nmpc6.sh`, `ana_bag6.py`). The disk guard
checks `/`, which is the same partition.
**FLOWN AT THE NEW DEFAULTS (row defaults_a1_141525, pendulum tank): LANDED 0.00 m.** Corridor
peak pitch +9.5 (gate +9.4), 90 deg in 14 s; cruise V 15.90 / +7.0 / front throttle 0.78 (the 12x8
gate number) / roll 0.0; back-transition peak pitch +10.0 at z 29.8-30.0 -- the +14..+21 onset
transient of the 12x6 flights is gone with the headroom; hover yaw diagonal 0.000. Eighth
flight of the day, eighth landing on the fixed plant.
**STACK DEATHS:** none in 17 launches today. Not chased.

# ★★ "AND THE SLOSHING?" -- WHERE IT CAN AND CANNOT MATTER (2026-09-13, faithful gate).
Every result above said "the slosh mode is inert" -- about the PLAN'S tank, whose 41 mm radius
moves 0.10 kg at 3.3 Hz (peak force ~0.35 N against a 39 N aircraft) AT THE CG. Two things make
that the most benign case possible: geometry and lever arm.
**GEOMETRY, same 0.502 kg** (the gate's `--slosh_radius R` re-shapes the tank at fixed water):
| R mm | h mm | f1 Hz | m1 kg (moving) | vs short period 1.42 Hz |
|---|---|---|---|---|
| 41 (plan) | 93 | 3.32 | 0.10 (20%) | x2.3, clear |
| 60 | 44 | 2.59 | 0.27 (54%) | x1.8 |
| 80 | 25 | 1.72 | 0.38 (76%) | x1.2 |
| 100 | 16 | 1.14 | 0.41 (81%) | x0.8 -- below it |
**LEVER ARM.** A tank at the CG makes NO moment however it sloshes: an 0.4 kg bob swinging to the
wall at 0.8 Hz changed nothing in the aircraft to two decimals. The gate now applies the reaction at
the tank (`--slosh_dx/--slosh_dz`, moment = r x F, as `VTOLSlosh._apply_slosh` does at
`_tank_pos`), and an OFFSET tank feels the rotation (`PendulumSlosh.step(omega, omega_dot,
r_tank)`: f_tank = f_cg + omega_dot x r + omega x (omega x r), in the sim by finite difference
of the body rates -- 5 rad/s^2 at 0.3 m is 1.5 m/s^2, the corridor's own acceleration).
**THE MODEL NEEDED A WALL.** The equivalent pendulum is linear and unbounded: an 80 mm nose tank
on a straight mission first "departed" the aircraft (roll 168) with the bob at 6e12 mm -- the bob
had swung metres in a 80 mm tank and the reaction force was fiction. `SloshParameters.wall` = R/2
(where the linear model's validity ends) is an inelastic stop: the bob parks, its energy is lost
(breaking), and the reaction is the arrest. `tests/test_slosh_pendulum.py` pins the wall and the
rotation terms.
**WITH A VALID MODEL: 0.5 kg of water cannot upset this controller through any tank geometry
41-100 mm, at the CG or 0.3 m fore/aft or 0.15 m above it, on the straight mission with a
30 deg/s pitch kick** -- every case identical (0/2001, corridor +9.5, BT +12.7-12.8, roll 0.5),
worst slosh moment 0.40 N.m (100 mm tank 15 cm above the CG, bob on the wall through the corridor;
the CG-error moment that gives -5 deg is 1.0 N.m). Where it can still bite is LATERAL
manoeuvring -- turn entries and exits put lateral specific force into a fore/aft tank and the
y-slosh at a 0.3 m arm is a YAW moment -- which is what the banked-turn guidance (next) is for.
Fly the offset 80-100 mm tank on the turning missions, not the straight one.

# ★★★ GUIDANCE: BANKED TURNS, LOITER, RETURN-TO-HOME -- FLOWN (2026-09-13, bag slosh_20260913_151254,
# row turns_loiter_rth_a1_151327). `Guidance` in `nmpc_cascade_node.py`, one text protocol.
**THE PROTOCOL** (`std_msgs/String` on `/<ns>/cascade/cmd`, ignored until engaged; `home` is latched
at engage): `wp X Y [X Y ...]` legs; `loiter X Y R [dir]` orbit (dir +1 CCW default, -1 CW);
`rth` fly to home, brake, hover over it; `hold` = the straight-line mission as before. The driver
takes `--guide "T:cmd;T:cmd"` (seconds after cruise start) and `fly_slosh_mission.sh` forwards
`GUIDE=...`; a plan that ends in rth lands at home. The node logs every ack (`GUIDE '...' -> ...`).
**GS BUTTONS (GS commit 5d28537, `trajectory_player.py` "Guidance" row): ↱ Fly waypoints (text
box, world metres), ◯ Loiter (X, Y, R >= 65, CCW/CW), 🏠 Return to home, → Straight.** One String
each on `/uav_0/cascade/cmd`; the node acks and streams `mode=... | status` at 5 Hz on
`/uav_0/cascade/status` (controller commit 0d156a7), shown in the row's label via HudSource (a
subscription on the GS node from the GUI thread never fires -- see flight_hud.py). Refused with
the reason unless the cascade is the ACTIVE controller: the node resets its guidance at engage,
so a command sent before the auto mission reaches CRUISE would be lost without a trace. In the
auto mission a wp/loiter marks the cruise "guided" (no far-target arrival, auto-RTH after 10 min);
🏠 moves it to an RTH phase that lets the cascade brake itself and then settles + LANDS AT HOME
on Safe Hover; an RTH that never stops gets the schedule's back-transition forced (never a hover
handover at 16 m/s). `tests/test_gs_trajectory_tab.py` covers the refusal, the wire format and
both RTH outcomes; `tests/test_cascade_node_boots.py` now requires the ack on the status topic.
**THE LAWS, and the numbers that set them.** All of it feeds the EXISTING bank-to-turn law (heading
error -> roll, k_bank_d damping) -- the guidance only decides the heading, plus one feed-forward:
  * Legs: heading = leg bearing + `k_ct` x cross-track (0.03 rad/m; 0.05 rang the capture after a
    90 deg turn, 23 m vs 15). A leg ends at the FLY-BY point `R_turn tan(dpsi/2) + 1.5 V` before the
    corner, R_turn = V^2/(g tan bank_max) = 56 m at 16 m/s / 25 deg, and the next leg's heading is
    issued the SAME tick (returning the old one for a tick cost 5 m on the gate). The 1.5 V is the
    roll-in: ~1.5 s from wings level to the bank limit.
  * Loiter: the last leg hands over to the orbit at `R_turn + sqrt(R^2 - R_turn^2) + 1.5 V` from the
    centre (139 + 24 m at R 100); switching at the centre gave a 270 deg wheel, at one radius it
    ended inside, and at R/R_turn = 80/56 there is no tangential join at all -- default R 100.
    The orbit is a vector field `psi_d = angle(r) + dir (pi/2 + atan(k_orbit (rho - R)/R))`,
    k_orbit 3, with the coordinated bank `-dir atan(V^2/(g R))` FED FORWARD into roll_ref: a P bank
    law alone held its 18 deg bank only with 15 deg of standing heading error and sat 8 m outside.
  * RTH: heading to home + cross-track; `want_decel` (the back-transition) fires at
    `V^2/(2 decel_rate) + rth_brake_margin` = 84 + 15 m from home; below ~3 m/s the nose stays put
    (no 180 deg swing past home).
  * The outer's speed frame is the measured COURSE, not the latched heading -- in a bank the old
    frame read the turn as a 10% speed loss and the outer chased it.
  Gate (noise 0.1/1, lat 0.03/comp 0.049, pendulum, momentum 12x8): square legs at 25 deg, loiter
  rho 103.3 +- 0.3 at R 100, RTH stop 13 m from home, 2/4601 solve failures; same at 50 ms transport
  and with offset slosh tanks (5.8 N.m of tank yaw in the turns, absorbed).
**FLOWN, ONE FLIGHT, LANDED (0.00, 0.00):** engage -> 90 deg cruise east at 15.9 -> `wp 500 0 500
400 100 400` -> two 90 deg fly-by turns -> loiter at (100,400) R 100 for 65 s (1.7 laps) -> `rth` ->
brake -> hover -> land at home. **0/3675 outer solve failures over the whole 200 s; z 29.9-30.0 through
every turn; V 15.90-15.94 in every bank** (the 12x8 headroom). Per phase, from the bag:
| phase | flown | gate |
|---|---|---|
| 90 deg fly-by turns (x2) | bank 24.8 / 24.7 (limit 25), heading overshoot 16-17 deg, cross-track overshoot +14.4 / +14.2 m, on the new leg within ~12 s | same shape, +-15 m |
| loiter entry | initial RIGHT bank (the field asks for 152 deg where the leg points 180), cut 13 m INSIDE (rho min 87), converged in ~1/4 lap | converges, sign identical |
| **loiter steady (last 40 s)** | **rho 103.3 +- 0.5 m** (101.3-104.3), roll -16.9 +- 2.9, ct +3.4 m constant | **rho 103.3 +- 0.3** |
| rth turn + capture | 166 -> -102 deg at the bank limit, 70 m off the home line at the turn's exit, on it (0.2 m) by 110 m out | 15 m |
| rth stop | braking armed 100 m out, 15.9 -> 0.6 m/s in 9.6 s, z 29.5-30.0, pitch <= +13.4, aft tilt -7.6, **stopped 24 m from home** | 13 m |
| landing | geometric handover, 0.00 m from home, level | -- |
**TWO THINGS THE FLIGHT SAYS THAT THE GATE ONLY HINTED:** (1) the steady orbit needs 17 deg of bank
for a 103 m circle where the coordinated value is 14 -- true on the gate too (the "18 deg bank"
above) -- so the vector field carries a standing 3.4 m radius error to supply the difference; the
plant needs more bank than a coordinated turn says (no rudder coordination -> sideslip), and an
integral on rho or an adapted bank feed-forward would close it. Not done. (2) the RTH stop is 24 m
short of home vs the gate's 13: 9.6 s from 15.9 m/s is 1.6 m/s^2 -- the flown back-transition
takes ~1 s longer than the schedule at the onset (the same "onset" gap the knife-edge flight saw);
`rth_brake_margin` is the knob if the hover must be over home (the LANDING was at home anyway:
the driver's geometric leg covers the last 24 m).
**THE DRIVER'S VERDICT LINE IS WRONG FOR THIS MISSION AND SAYS SO:** "INCONCLUSIVE -- too little
CONTROLLED level flight" -- its metric is the straight-line run of controlled+level samples, and a
turning mission never accumulates one. `night_metrics.py` scored the bag LANDED, wb_s 177, land_err
0.00; maxroll 25 and maxyaw 180 there ARE the turns. Read the bag (`scripts/ana_guided.py`; the cascade
pane log has the GUIDE acks and the fails counter).
**AND THE FIRST ATTEMPT WAS A DEAD NODE THAT PASSED READY** (row turns_loiter_rth_a1_145847, flown by
the geometric controller at 1 m/s and reported "cruise"). The new command subscription built its
topic name with `self.gp('px4_ns')` -- gp() casts to float -- and the node died in its constructor
TWO LINES after printing the schedule banner the launcher gates on. Nothing in the chain looked
past the banner: `hil_health.sh` counted geometric + nmpc_full as "2/2" and read tracebacks only
from 6g-full; the slosh launcher accepted a banner from a process that no longer existed; the
pane pipe starts AFTER the banner check so the forensic log had 0 lines. Now: the health check
requires `nmpc_cascade_control` alive whenever a 6h-cascade window exists and reads both wing-borne
panes; the slosh launcher refuses on a post-banner traceback or a missing node process; and
`tests/test_cascade_node_boots.py` runs BOTH cascade nodes exactly as the launcher does (rosenv,
isaac_env python, `--ros-args`), publishes one guidance command and requires the ack and a live
process -- it fails on the flown line and passes on the fix. The offline gate never runs the
parameter/subscription code (class defaults), so this is the second bug of that class it could not
see (the `descent_budget` declaration of 2026-09-07 was the first); the boot test is the gate for it.

# ★★★ TWO WATER MASSES (2026-09-13 evening): `VTOL_TANK_KG` = 0.1 / 0.502 (default) / 0.7, ONE VARIABLE
# FOR THE PLANT AND EVERY CONTROLLER -- AND THE OUTER SOLVER HAD BEEN FLYING WITH M = 3.45 BAKED IN.
**THE SWITCH.** `TankConfig.water_kg` sets the depth in the 41 mm tank (the tank grows to 0.93 L to hold
0.7 kg at 75%); `slosh.default_tank()` reads `VTOL_TANK_KG` and is what the HIL scene, the plant
double, the pairing test and the gate's `--slosh` construct. On the controller side every copy of the
mass is `3.449572 + float(os.environ.get("VTOL_TANK_KG", "0.502") or 0.502)` -- so3.M, the trim
generator, the safe controller, vtol_nmpc (a baked trim table PER MODE; an unlisted mass raises at
import) -- and `tests/test_tank_model_pairing.py` pins that exact text in each and assembles all three
modes. Both launchers write the variable INLINE on the Isaac pane AND all three controller panes (a pane
does not inherit the shell); `start_slosh_hil.sh` refuses READY unless Isaac's `water=<kg>` banner
and the tank cascade's `M=<kg>` banner both match the request, and the mass goes into the launch log
and the results row (`water=0.7kg`). `VTOL_TANK=slosh VTOL_TANK_KG=0.7 bash scripts/run_slosh_flight.sh`.
**THE MODES ARE NOT THE SAME TANK AT TWO FILLS IN THEIR SLOSH.** 0.1 kg is a 15% fill: 18.6 mm deep,
**69% of it moves (0.069 kg) at 2.74 Hz**. 0.7 kg is the deep tank grown: the same 0.10 kg moves at
3.32 Hz as in the plan's 0.502 -- the heavy mode changes ONLY the dead mass (M 4.1496, stall 15.50 m/s
against the 16 m/s cruise: 0.5 m/s of margin; a 25 deg bank at 16 m/s needs 16.27, so the rear pair
carries the deficit in every turn).
**GATE, all three modes, ttr 10, lat 0.03 / comp 0.049, noise 0.1/1, pendulum, 16 seeds each,
full mission with the back-transition: 16/16, 16/16, 16/16.**
| VTOL_TANK_KG | M | stall | corridor peak pitch | cruise V / pitch / front / rear | front max in the corridor | BT peak pitch | fails (16 BTs) |
|---|---|---|---|---|---|---|---|
| 0.1 | 3.5496 | 14.33 | +9.2 | 16.02 / +6.0 / 0.76 / 0.30 | 0.88 | 14.1 | 2 |
| 0.502 | 3.9516 | 15.12 | +10.3 | 16.05 / +7.0 / 0.78 / 0.33 | 0.91 | 14.1 | 3 |
| 0.7 | 4.1496 | 15.50 | **+11.0** | 16.07 / +7.6 / 0.80 / 0.35 | **0.93** | 13.5 | 4 |
The heavy mode's two thin numbers: the corridor crosses the 10 deg CL peak by a degree (held altitude,
29.9-30.0), and the front pair reaches 93% throttle in the corridor -- 7% for the inner loop. The
harness's `--ttr` DEFAULT IS 40 (the node's is 10): at 40 the corridor peaks at 12.3-13.2 in every
mode; pass `--ttr 10` to gate the flown configuration. Reference audit 0.05 rad/s^2, cascade audit
passed, safe controller 22/22 with the 6 m/s crosswind nulled to 4 mm, in both modes.
**★ THE OUTER SOLVER'S CACHE KEY DID NOT HASH THE MASS.** nmpc6's JIT key hashed the weights and the
aero coefficients only, and `M = so3.M` is a module constant baked into the compiled ODE: the
cascade's outer loaded `nmpc6_cruise_N14_DT60_852b5135d1.casadi`, compiled 2026-09-10 15:18 at
M = 3.45, for EVERY tank flight (13% light in vsdot/vzdot). The reference trims were right (the
Schedule is Python, not compiled) and the closed loop absorbed it: the solver rebuilt at 3.9516
cruises at the same 16.05 / +7.1 / 0.78 and passes the same 16 seeds. No flown result changes; the
statement "the cache key hashes the model source" was true of nmpc_full and NOT of nmpc6/cascade.
Now hashed: M, G, IYY, MAX_THRUST, CQ, RPOS, `build_ode6` and `so3.aero_body` source -- each mode
compiles its own solver (~2 min, once). The lesson is the one from 2026-09-06 (C6) repeated on the
node that was written after it: a cache key must cover everything the artefact bakes in.
**FLOWN, 0.7 kg (bag slosh_20260913_172434, row kg07_turns_a1_172507): LANDED 0.01 m FROM HOME, the
full guided mission** -- two 90 deg fly-by turns at the 25 deg limit (z 30.0-30.1, V 15.95-16.02,
overshoot +13.6 / +12.7 m), loiter steady rho 103.4 +- 0.6 at R 100 (roll -17.5), RTH, stop 27.9 m
from home, corridor peak +10.2, 3/3866 outer solve failures. **Where the 0.5 m/s of stall margin
shows: the back-transition.** The decel armed while still 6 deg banked from the RTH capture and the
crossing of the 15.50 m/s stall pitched to **+16.6 deg** (V 13.3) with the 3 failures in that one
second, then +12.7, +10.3, ... and a clean stop (z min 29.7). Flight 10 at 0.502 kg crossed at
+13.4 with 0 failures. The gate's 16 seeds at 0.7 peak at 13.5 wings-level; a decel onset IN A BANK
is the case to gate next (`--roll0` at `--decel_at`). The launcher's new mass checks passed
(`water: 0.700 kg confirmed`, `cascade mass: 4.1496 kg`) -- after first failing on their own
capture: tmux WRAPS a 128-character banner at the pane width and `capture-pane` without `-J`
returned the "M=<kg>" tail as a separate line (17:12, two attempts). `-J` now.
**0.1 kg: gate 16/16 (corridor +9.2, cruise +6.0 / 0.76, BT 14.1); the flight was launched (both
banners confirmed 0.100 kg / M 3.5496) and stopped two minutes in at wrap-up. NOT FLOWN.**
**SWEEP HYGIENE:** 16 parallel casadi processes loading one cache file core-dumped two of them at
start (no output at all, `timeout: the monitored command dumped core`) -- a process crash, not a
flight; re-run those seeds serially before reading a "14/16".

**OPS — `scripts/start_slosh_hil.sh` / `scripts/kill_slosh_hil.sh`.** The launcher
delegates to `restart_hil.sh` (one real launcher) and adds what a bare
`VTOL_SLOSH=1` cannot: it refuses on a branch with no `vtol_slosh.py`, runs both
gates first, and then **VERIFIES that Isaac logged the model banner** -- the env
var travels INLINE on a tmux send-keys line, a pane does NOT inherit the calling
shell, so a missing forward would fly the BASELINE while the operator believes
the tank is in. `--baseline` flies the baseline down the same path (the A/B is
one flag), `--dry-run` checks everything that does not need a stack.
Bags go to `~/flight_bags/slosh`, NOT `/tmp/bags` -- which is still the default
everywhere else and is how the post-extraction verification bag was lost.
The kill script wraps `kill_hil.sh` (the pattern list has ONE home) and adds two
things: it **SIGINTs the recorder first** -- `kill_hil.sh` -9s it, and a
SIGKILLed bag has no metadata.yaml, which is why every bag in `~/flight_bags` is
db3-only -- and it **exits non-zero if anything survived**, since kill_hil.sh
prints survivors and exits 0 either way. It refuses to run from inside tmux
(`kill_hil.sh` ends in `tmux kill-server`, which would kill the checking shell).

**THE NUMBERS THAT DECIDE THE DESIGN** (recomputed by the test, not assumed):
water is capped near **0.5 kg** by cruise margin (+1.0 kg puts stall at 16.05
against a 16.0 m/s cruise -- one litre does not fly); the default 41 mm tank
rings at **3.32 Hz, 2.34x the 1.42 Hz short period**; and it is **0.49% damped**
-- 23 cycles to halve, below the elevator servo corner, i.e. inside the loop's
reach. `TankConfig.baffled()` exists from day one for that reason.

# PegasusSimulator — Developer Checklist

## Checklist for adding a new vehicle model

When creating a new vehicle (config class + vehicle class), verify each item below.

### Config class
- [ ] All fields are **pure data** — no live resources (ROS nodes, sockets, processes) instantiated in `__init__`
- [ ] `self.backends = []` or `self.backends = None` — never `[SomeBackend(...)]`
- [ ] Same rule applies to `self.sensors`, `self.graphical_sensors`, `self.graphs` if they carry live state

### Vehicle class `__init__` signature
- [ ] Default `config` argument is `None`, **not** `config=MyConfig()` — Python evaluates default arguments once at import time, creating shared mutable state across all calls
  ```python
  # Wrong
  def __init__(self, ..., config=MyVehicleConfig()):

  # Correct
  def __init__(self, ..., config=None):
      if config is None:
          config = MyVehicleConfig()
  ```
- [ ] Default backend (if any) is created **inside** `__init__` after resolving `None`:
  ```python
  if config.backends is None:
      config.backends = [MyDefaultBackend()]
  ```

### ROS 2 backend
- [ ] The backend creates **exactly one** `rclpy` node per instance — verify the node name is unique (include `vehicle_id`)
- [ ] `rclpy.init()` is wrapped in `try/except` (already initialised is not an error)
- [ ] Publisher/subscriber QoS matches what the counterpart node expects (`qos_profile_sensor_data` = BEST_EFFORT)

### Single-link floating articulation (no external USD)
- [ ] `ArticulationRootAPI` and `RigidBodyAPI` are applied to the **same** prim — PhysX 5 does not form a valid articulation when they are on separate prims
- [ ] Override `update_state` to read from `self._stage_prefix` (not `+ "/body"`)
- [ ] Call `apply_force` / `apply_torque` with `body_part=""`

---

# ★★★★ ACTUATOR MODELS: CONFIGURABLE THRUST + PROP INFLOW + SURFACE SERVO (2026-09-10)
# `vtol.py`. ALL DEFAULTS REPRODUCE THE LEGACY BEHAVIOUR EXACTLY -- nothing is re-tuned unless
# you opt in. `tests/test_actuator_models.py` (headless, no Isaac) guards the maths.

**WHAT THE SIM USED TO MODEL** (and the FSC airframe does not override `rotor_dynamics`, so this
was what every flight flew):

| | modelled | present but DISABLED |
|---|---|---|
| thrust | `throttle * max_thrust`, instantaneous, LINEAR | `rotor_tau_spin`=0.12 s, T ~ omega^2 |
| tilt | pure rate limit 80 deg/s | `tilt_tau`=0.06 s lag |
| surfaces | **nothing at all** | -- there was no surface model |

Worse, `_apply_aerodynamics` read the RAW COMMAND, so the surface moment appeared the instant the
command did and the hinge joint's own response was decorative. A controller could slew a surface
at any rate it liked -- and one did: the HIL elevator sign-flipped near full deflection every
~0.25 s, which needs several hundred deg/s to be physical.

**`thrust_model`: "instant" (default) | "lag" | "momentum".** `rotor_dynamics=True` is kept as an
alias for "lag". `CT0` is DERIVED from each rotor's own `(max_thrust, rotor_omega_max,
prop_diameter_m)`, so STATIC thrust is preserved EXACTLY by every model and only the airspeed
behaviour is new. "momentum" adds the inflow term, linear in advance ratio `J = V_ax/(nD)`:
`T = rho*D^2*( CT0*(nD)^2 - (CT0/J0)*(nD)*V_ax )`, clipped at zero, with `V_ax` the airspeed along
**this rotor's TILTED thrust axis** -- at 90 deg tilt that IS the cruise airspeed.

**★★ CONSEQUENCE, AND IT IS LARGE. THE CRUISE TRIM DEPENDS ON THE FLAT THRUST MODEL.** For a
representative 12 in prop (`prop_diameter_m`=0.3048, `prop_zero_thrust_J`=0.6, sized so 18 N static
needs ~7150 rpm, `CT0`=0.12 -- squarely in the normal 0.08-0.14 band):

| V (m/s) | 0 | 4 | 8 | 12 | 16 |
|---|---|---|---|---|---|
| max thrust/rotor at FULL throttle | 18.00 | 14.70 | 11.39 | 8.09 | **4.79 N** |

and because thrust vanishes at `J = J0`, **at 16 m/s there is NO positive thrust below ~74%
throttle** -- against a controller that currently cruises at **11%**. Switching "momentum" on WILL
invalidate the cruise trim and the allocation headroom story (the inner loop's differential relies
on spare thrust) and requires re-tuning. That is the point of it: the present 8.1 N of cruise
thrust at 11% throttle is an artefact of a propeller that does not care how fast the air is
already moving.

**`surface_rate_deg_s` / `surface_tau` (both default 0 = legacy instant).** First-order lag then a
slew clamp, the order a real servo behaves in. Representative small digital servo 300-500 deg/s,
tau 0.02-0.05 s; a small stepper is slower, 100-200 deg/s. `self._surface_state` tracks the ACTUAL
deflection and **the aero now reads that, not the command** (`_surface_command_equiv`).

**ALSO FIXED HERE: `DR_MAX` WAS 30 deg WHILE THE SIM APPLIES 25 deg TO EVERY SURFACE.**
`vtol_aerodynamics` scales the normalised [-1,1] command by `max_surface_deflection_rad =
radians(25)` for aileron, elevator AND rudder, and every airframe sets `max_deflection_deg=25`.
The controllers computed `dr = M/(k*DR_MAX)` against a limit 20% too large, so **every yaw-moment
demand was delivered at 25/30 = 83% of what was asked**. Corrected to 25 in
`nmpc_so3_transition_node`. It survived because `test_controller_aero_match.py` feeds RADIANS
directly to both models -- it checks the aero FUNCTIONS agree but never exercises the
[-1,1] -> radians NORMALISATION. `tests/test_surface_normalisation.py` now does.
**NOTE this changes nmpc_full's and nmpc6's rudder gain by 1.2x -- a correction, but they were
flight-validated with the wrong value, so re-check their yaw behaviour.**

# ★★★★★ CASCADE — HIL-FLOWN, FULL MISSION, 0.1 m BACK-TRANSITION EXCURSION (2026-09-10)
# `nmpc_cascade_node.py`, controller_id `cascade`. Wired into the GS "Run Full Auto Mission".
#
# **HIL RESULT (bag casc3, third flight): TRUE FIXED-WING, 79.0 s UNBROKEN controlled+level.**
# ```
# climb 30 m -> transition -> 900 m of 90 deg cruise -> back-transition -> stop, ALL at 30 m
# cruise        V=16.00  tilt=90.0  AoA=+5.5  wing=96%  rotors=8.1N of 33.8N  roll/yaw=0
# back-trans    V 16.00 -> 0.08 m/s, tilt -8.7 deg (front pair AFT), altitude 29.9-30.1
# EXCURSION     0.1 m       (nmpc_full's best was 2.2 m; with no back-transition, 65.7 m)
# solve         2 failures in 1861 across the WHOLE mission; 0 node exceptions
# inner loop    held a full 120.0 Hz, p95 8.7 ms -- the background-thread decouple works
# ```
# HIL and offline agree to 0.2 deg of AoA and 0.02 m/s of speed. Offline the same mission is
# 0/2401 solve failures; the numbers below are the offline bench that predicted it.
The 19-state `nmpc_full` was COMPUTE-SATURATED, not mistuned: four independent stiffenings each
improved the physics and each saturated IPOPT at MAX_ITER=25, and MAX_ITER=40 put p95 at 93 ms
against a 100 ms tick. Splitting the problem buys the budget back.

    OUTER  6-state longitudinal NMPC, background thread ~20 Hz -> Tf, Tr, beta, de (+ ref pitch)
    INNER  geometric, every mocap sample                       -> roll/yaw + allocation

| | 19-state (nmpc_full) | **cascade** |
|---|---|---|
| solve | 27 ms median / 58-93 p95 **in flight** | **9-12 ms bench, whole corridor** |
| MAX_ITER | 25, saturated | not binding |
| steady state | phugoid, ~50% repeatability | **z=30.0, V=16.02, tilt=90.0, AoA=+5.7 CONSTANT 120 s** |
| lateral | mid-corridor departure at 6-10 m/s | **worst \|roll\| 0.0, \|yaw\| 0.0 over 200 s** |
| solve failures | ~20% | **0/4001** |
Recovers roll10+yaw20, roll15+yaw-25, yaw30, roll20 -- the combined upsets that crash nmpc6.

**FIVE THINGS HAD TO BE RIGHT. Each was measured, and four were bugs I introduced by omission:**
1. **DELAY COMPENSATION IS NOT OPTIONAL.** On the outer's OWN model -- perfect match, no lateral, no
   inner loop, nothing else to blame: lat 0 -> 0/1201 fails and smooth 0->90 deg; lat 0.05 ->
   87/1201; lat 0.10 -> **ground at t=35 s**. Omitting it made the cascade look like an architecture
   problem when it was a missing `predict6`.
2. **AUDIT FINDING C9 WAS NEVER BACK-PORTED TO nmpc6** (`w_beta`=12, a SOFT tilt guide). C9 says a
   soft guide leaves a flat tilt-vs-pitch optimum and the solver HUNTS -- and it does: driving the
   true plant the tilt command bang-banged **10->40->11->0->39->43->15->2->46->45->0 deg at ~1 Hz
   with ok=1 throughout** (hunting, not failing). nmpc6's OWN test shows it too. `OuterNMPC`
   subclasses with `w_beta`=60 so the stock solver and its JIT cache are untouched.
3. **★ BANK-TO-TURN. Holding HEADING with the RUDDER does not survive wing-borne flight.** A banked
   aeroplane TURNS; that is not a disturbance to cancel. Inner loop alone at the 90 deg trim, a 5 deg
   roll upset drove yaw to **-163 deg with the rudder SATURATED at -1.000** while roll recovered
   fine. Heading error now commands ROLL, blended on `(V/14.13)^2` = the wing's share of
   weight-carrying (0 in hover -> multicopter yaw, 1 wing-borne -> pure bank-to-turn). Physical, not
   a tuned crossover speed. This was a multicopter reflex applied to an aeroplane.
4. **THE PITCH SIGN INVERTED FOR THE SIXTH TIME.** `theta = asin(R[2,0])` is nose-up POSITIVE but the
   rotation about body +y that produces nose-up is NEGATIVE, so the attitude term is **+kR, not -kR**
   (the RATE term keeps its minus). Backwards it is positive feedback: pitch diverged in HOVER,
   0 -> -13 -> +17 -> +57 deg in 2.2 s. **Verify empirically every time** -- front-heavy split gives
   `omega_dot_y = -12.9` and drives theta to +58, which settles it in one command.
5. **THE PITCH AXIS SPLITS BY BANDWIDTH.** Three variants measured: both loops doing pitch ATTITUDE
   -> they fight (the bang-bang above); inner doing NO pitch -> hover unstable, ground at t=4.6 s;
   inner doing RATE DAMPING only -> parked nose-up at +12 deg CLIMBING at 1 m/s with ok=1, and
   `v_cmd` then pinned itself via the `V + v_lead` clamp. The outer owns pitch ATTITUDE, the inner
   owns roll/yaw and supplies pitch rate damping.

**THE ALLOCATION, AND WHY nmpc6's `lateral_control` GAVE AWAY AUTHORITY.** It faded differential
thrust out on `lowspd = clip((6-V)/6,0,1)` -- gone by 6 m/s -- leaving a NOTCH exactly where the
campaign kept departing: roll authority 38.6 (V=0) -> **8.7 rad/s^2 (V=6)** -> 15.5 (V=8) -> 62 (V=16).
At V=6 the airframe still HAS 8.50 N.m of differential (cos 33.8 = 0.83 of hover) and the fade threw
all of it away; keeping it gives 40.8 instead of 8.7. Its docstring claims no roll authority at
90 deg tilt -- false. Measured `cond(B_rot)` never exceeds 33.8 and roll stays **0.304 N.m/N at
90 deg (54% of hover)**, because the REAR pair is still vertical (pure roll) while the front is
horizontal (pure yaw): the axes DECOUPLE at 90 deg. The old code lost it by applying one scalar
`droll*cos(beta)` to BOTH pairs. `B_rot(beta)^-1` is solved against **dFz = 0** so the inner never
disturbs the outer's trim, and each moment is split rotor-vs-surface by ACTUAL authority (headroom
vs qbar) -- no crossover speed anywhere. `rear_floor`=4 N keeps the rear pair loaded for roll.

**LATENCY IS THE CEILING, AND IT IS THE ATTITUDE LOOP'S, NOT THE SOLVER'S.** Over-compensating does
NOT help (lat .15/comp .20 still crashes). Softening the inner gains does, and softer wins EVERYWHERE
-- it is not a trade: `kR`=3.0/`kW`=1.8 gives **0/2401 fails at lat 0.05 and 0.10** (vs 5/4001 at
kR=9) and settles on the EXACT +5.7 trim AoA instead of wandering to +10.8; at lat 0.15 kR=9 is on
the ground at 7 s while kR=3 flies the full 120 s. Limit ~150 ms at kR=3, ~125 at kR=9 -- the same
number CLAUDE.md already recorded for the safe controller ("the ATTITUDE loop goes first"). **If HIL
measures more than ~150 ms, soften kR further; do NOT raise `latency_s`.**

**nmpc6's OWN OFFLINE TEST NOW CRASHES (t=8.0 s, most solves ok=0).** It was validated 2026-08-12,
BEFORE the 2026-09-06 aero frame fix and pitch-convention flip. `build_ode6` was converted; its
TUNING and closed-loop behaviour never were. Do not quote its 2026-08-12 numbers.

**PROBE TRAP, AGAIN:** an isolation probe that sets `v = R @ [V,0,0]` puts velocity along the BODY
x-axis, so **AoA starts at 0 instead of the trim's +5.7** -- not a trim at all. The aircraft pitches
up hunting for lift and the Euler angles flip through vertical, which READS AS A LATERAL DEPARTURE
(roll/yaw to +-180) when the real event is longitudinal. Level flight means velocity HORIZONTAL IN
THE WORLD. I diagnosed a lateral instability that did not exist before catching this.

**★ THE TRANSITION CAN BE MUCH FASTER THAN nmpc_full's, AND FASTER IS SAFER.** Swept offline
against the true plant at the measured 30 ms latency, `ttr` from 40 s down to 4 s:

| ttr | 40 | 30 | 25 | 20 | 16 | 12 | 10 | 8 | 6 | 4 |
|---|---|---|---|---|---|---|---|---|---|---|
| t to 90 deg | 42 | 32 | 27 | 22 | 18 | 14 | 12 | 10 | 8 | 6 s |
| peak AoA | 11.3 | 11.1 | 11.0 | 10.7 | 10.4 | 9.6 | 8.9 | 7.9 | **6.7** | 8.6 |

**Altitude is 30.0 m in EVERY case and every run is 0/2201 solve failures.** Peak AoA IMPROVES as
the ramp quickens, because a quicker crossing spends less time below the 14.13 m/s stall speed
where the wing cannot carry the weight and the solver has to buy lift with angle of attack. This
INVERTS the nmpc_full lore ("ttr 18 rang the phugoid; do not go there") -- that was a different
controller on the pre-2026-09-06 plant, and it does not transfer.
**Default is now `ttr`=20** (half the flown transition, same peak AoA as the flown config).
Going FASTER than that is limited NOT by the transition itself but by the lateral defect below:
at ttr=8 the post-disturbance clean window (~60-70 s) is SHORTER THAN A TYPICAL CRUISE, so 20 is
chosen to clear a ~90 s cruise, not because the transition cannot go quicker. `WB_PARAMS["ttr"]` STAYS 40 -- it is shared with
nmpc_full, where 40 is flight-validated -- so the cascade's value lives in the GS's
`CASC_OVERRIDES`, applied only on the cascade path, and `tests/test_gs_cascade_params.py` fails if
it ever leaks into nmpc_full.

**★★★ THE WHOLE SIMULATION CHAIN HAS NO SENSOR NOISE — SO NO CONTROLLER HERE HAS HAD ITS NOISE
REJECTION TESTED. (2026-09-10, measured.)** Prompted by "the controller seems almost too perfect".
It was. `worst |roll| 0.0, worst |yaw err| 0.0` is not a result -- the offline plant is perfectly
symmetric with EXACT state feedback, so roll sits at 0 because nothing ever displaces it.
**AND THE HIL RIG IS THE SAME:** measured on a flown bag, the mocap's frame-to-frame attitude
jitter is **0.00042 deg sd** (max 0.002). A real OptiTrack at 120 Hz shows **0.02-0.1 deg**, i.e.
the rig is 50-200x cleaner, because `isaacsim_optitrack_ros2_emulator` republishes the EXACT sim
state. Offline and HIL share one blind spot, so agreement between them proves nothing about noise.

`nmpc_cascade_node.py --test` now takes `--noise_att` / `--noise_rate` / `--noise_vel` (corrupting
only the CONTROLLER's view, never the plant's integration) and `--gust`. What that exposes, at
ttr=20, 150 s, lat 0.03:

| disturbance | worst roll | worst yaw | altitude | solves |
|---|---|---|---|---|
| none (what was reported) | **0.0** | **0.0** | 30.0 | 0/3001 |
| attitude noise 0.1 deg | **19.5** | **33.2** | 30.1 | 0/3001 |
| attitude noise 0.3 deg | 20.2 | 34.0 | 30.1 | 0/3001 |
| body-rate noise 1 deg/s | 19.4 | 33.2 | 30.2 | 0/3001 |
| velocity noise 0.05 m/s | 0.0 | 0.0 | 30.0 | 0/3001 |
| lateral gust 3 m/s^2 | 24.8 | 49.2 | 30.2 | 0/3001 |

**0.1 deg of attitude noise -- trivial and entirely realistic -- takes roll 0.0 -> 19.5 and yaw
0.0 -> 33.2.** It barely worsens from 0.1 to 0.3 deg, so it is NOT noise amplification: any
attitude/rate perturbation excites the marginally-unstable heading mode, which grows to a limit
cycle bounded by the 25 deg `bank_max`. VELOCITY noise alone does NOT excite it (roll stays 0.0),
which fits the P-only bank law being driven by attitude. Onset is t~60-75 s and is the SAME for
ttr=20 and ttr=40 -- so `ttr` does not govern it, and the earlier "clean window" figures (from one
large upset) do not apply to the noise-driven case.

**WHAT SURVIVES THE SCRUTINY, AND WHAT DOES NOT:**
  * LONGITUDINAL IS REAL. Altitude 30.0-30.2 m in EVERY case including 3 m/s^2 gusts, airspeed
    15.2-16.0, tilt 88.8-90.0, and **0/3001 solve failures throughout**. Nothing crashed.
  * LATERAL IS UNVALIDATED AND PROBABLY POOR ON HARDWARE. Expect a ~20 deg roll / ~33 deg yaw
    limit cycle within ~75 s of cruise on a real mocap. Neither the offline gate nor HIL predicts
    it. This is the same defect as the P-only bank-to-turn law (above) -- noise just makes it
    unavoidable rather than disturbance-triggered.
**RULE: a clean lateral result from this stack is evidence of nothing until it is re-run with
`--noise_att 0.1 --noise_rate 1.0`. Add that to any lateral claim before believing it.**

**DYNAMICS AUDIT — `scripts/audit_cascade.py` (ttr=20 shipped config).** `audit_reference.py`
audits nmpc_full's reference; the cascade has its own, PLUS an allocation matrix and surface
formulas derived BY HAND, which is where this project has been bitten before. All four checked
NUMERICALLY against `so3.build_ode()`:

| check | result |
|---|---|
| `B_rot(beta)` vs plant dF/dT, dM/dT | **2.5e-13** -- exact at every tilt |
| `surface_authority(V)` vs plant | 0.0-0.1% at 8/12/16 m/s |
| reference is a TRIM, forward | worst \|qdot\| **0.038** rad/s^2, az 0.000 |
| reference is a TRIM, back-transition | worst \|qdot\| **0.018** rad/s^2, az 0.000 |
| `build_ode6` (outer's model) vs plant | vsdot/vzdot EXACT; qdot off by 0.038 rad/s^2 |

The 90 deg allocation column is worth recording, because the whole inner loop rests on it:
`dFz +0.0000, dMx -0.1200, dMy -0.0330, dMz -0.2840` -- the front pair makes NO vertical force and
almost pure YAW there, which is why the REAR pair (still vertical) has to carry roll.
NOTE: nmpc6's `--validate` claims build_ode6 is byte-identical to the plant; it is not quite --
0.038 rad/s^2 of pitch, small but non-zero.

**OBSERVATION, not yet a bug: the back-transition reference implies -8.68 m/s^2 of deceleration at
V=16 while `decel_rate` commands 1.5.** With only (Tf,Tr) you can satisfy vertical and pitch
balance and the along-track acceleration is whatever falls out, so this is expected -- but the
flare/aft-tilt schedule and `decel_rate` are describing DIFFERENT manoeuvres (~5.8x apart). The
solver resolves it by tracking the speed reference, and the flown back-transition braked at
~1.6 m/s^2 matching `decel_rate`, with a 0.1 m excursion. Left alone because it flies; but this is
the same SHAPE as the bugs that cost bags full_tr7 and full_tr25/27 (shipping one half of a trim),
so if the back-transition ever misbehaves, look here first.

**★★ OPEN DEFECT FOUND WHILE SWEEPING ttr: A MARGINALLY-UNSTABLE LATERAL MODE IN CRUISE.**
A DISTURBANCE seeds a slow roll/yaw oscillation that grows over MINUTES. It is present at EVERY
transition rate -- `ttr` only changes how long until it is visible:

| ttr | clean window after a roll15+yaw-25 upset | then |
|---|---|---|
| 8 | ~60-70 s | yaw +26 -> -32, roll to +19 |
| 20 | ~120-150 s | yaw -9.9 -> +8.3 -> -18.5, roll +19.2 |
| **40 (the FLOWN config)** | **~250-300 s** | yaw -32.3 at t=300, roll -18.5 at t=420 |

**UNDISTURBED flight is clean at every ttr for 240 s+** -- which is exactly why it was missed: the
200 s validation soak had NO disturbance, and the upset tests only ran 90 s, just short of onset.
**A long soak with no upset does not exercise this mode at all.** The flown HIL mission cruised
~90 s, inside the clean window for its ttr=40, so it never appeared in flight either.

CAUSE (structural, not a tuning miss): the bank-to-turn command was written P-ONLY --
`roll_ref = wing * k_bank * e_yaw` -- and heading responds to roll through a DOUBLE integration
(roll -> turn rate -> heading). P-only on a double integrator is marginally stable at best. The
open-loop spiral is already unstable at +0.042/s (t2 = 16.5 s at V=16), so the loop has to supply
the damping and it does not.
**A yaw-RATE term does NOT fix it as a gain tweak:** `k_bank_d` swept 0.0/0.5/0.8/1.5 against the
upset over 300 s, none converged (1.5 parks the yaw at +34 deg with roll drifting instead). The
parameter is kept, DEFAULT 0.0 = OFF, because the structure is right but the loop needs proper
design -- a washout on the heading term, or heading tracked through a commanded turn RATE rather
than a commanded bank. **Do not enable `k_bank_d` expecting a fix.**
**CONSEQUENCE: sustained cruise beyond ~2 min after any disturbance is NOT yet trustworthy.**
Keep `ttr` well above the intended cruise duration until this is fixed.

**`tilt_margin_deg` IS NOT THE LEVER.** 15 / 25 / 35 / 50 / 90 deg all give BYTE-IDENTICAL results
(22 s to 90 deg, peak AoA 10.7): the airspeed tilt clamp never binds, because the outer's own
commanded tilt already sits below trim(V) + 15 the whole way. The tilt follows the SPEED ramp
through the trim schedule, so `ttr` is the only knob that moves it. Do not go looking in the clamp.

**★ FOUR BUGS THE FIRST TWO HIL FLIGHTS FOUND, NONE VISIBLE OFFLINE:**
1. **`latency_s` WAS 4x THE TRUE DELAY.** MEASURED at **30 ms** by cross-correlating the elevator
   command against the resulting pitch ACCELERATION over 20 s of steady cruise (the elevator is a
   direct moment actuator, so that lag IS the command->effect delay of the whole chain):
   `scripts/casc_timing.py`. It was 0.12, inherited from the 19-state node. Over-compensating is
   NOT benign here -- the outer plans for a state the aircraft never reaches, and the elevator
   entered a ~4 Hz limit cycle slamming -0.97..+0.86 with pitch rate swinging +-100 deg/s while
   pitch itself moved only +-5. At 0.04 the same corridor is monotonic and smooth (AoA
   +8.0,+7.7,+7.4,+7.2,+6.9 vs +11.4,+10.5,+14.3,+9.0 before). **Re-measure after any change to
   the stack's rates; do not inherit this number from another node.**
2. **THE DIVERGED FALLBACK PUBLISHED RADIANS ON A NORMALISED CHANNEL** -- `self._beta` (1.405 rad)
   went out where 0.894 was meant, and it also HELD the cruise tilt instead of bringing the rotors
   up. The bag's impossible "tilt 126.4 deg" (= 1.405 x 90, on a servo clamped to 90) was the
   giveaway. safe_hover means ROTORS UP; it now slews the tilt to zero.
3. **`slow()` WAS NOT EXCEPTION-WRAPPED.** `inner()` was, and that felt like enough. It is not: an
   UnboundLocalError on the FIRST `decel_now` transition (a log line used the airspeed one line
   before the state was unpacked) killed the rclpy executor and the whole node mid-cruise. PX4 held
   the last frame, the tilt froze at 90 deg and the aircraft mushed from 30 m into the ground with
   the status lines simply stopping. **Wrap BOTH loops** -- `tests/test_cascade_structure.py`
   now fails if either is bare.
4. **THE PITCH REFERENCE NEEDS THE SAME RATE LIMIT AS THE TILT.** The flare is
   `flare * min(1, V/Vc)`, FULL at the instant decel begins: the reference pitch stepped
   +5.7 -> +20.7 deg in one tick and the aircraft was on the ground 5 s later. Rate-limiting the
   tilt alone is not enough -- the flare and the aft tilt are two halves of one manoeuvre.

**GS WIRING (auto mission).** "🚀 Run Full Auto Mission" now targets whichever wing-borne
controller the new `ctrl` dropdown selects. The two do NOT share a parameter surface, and
`_set_node_params` counts an unknown parameter as a failure that ABORTS the build, so the GS
filters `WB_PARAMS` through `CASC_KEYS` (11 of 22 keys). **`tilt_slew_deg_s` is deliberately NOT
sent: both nodes declare that name and mean different things** -- on nmpc_full an external limiter
on the commanded tilt (15 deg/s), on the cascade the node's MODEL OF THE SIM SERVO (80 deg/s, a
plant property). Sending 15 would allocate for a tilt the aircraft is not at and stretch the
0->90 deg sweep from 1.1 s to 6 s. A NAME MATCH IS NOT A MEANING MATCH.
`tests/test_gs_cascade_params.py` compares the two sources and knows about that collision.

**OPS:** launcher windows `6h-cascade` (both `restart_hil.sh` and `start_vtol_smc_hil.sh`, gated on
`want cascade`; the OLD SO(3) cascade's gate was renamed `so3_cascade` -- they collided and started
a second standby NMPC). GS button "⇄ Cascade (NMPC + geometric)" plus the mission dropdown.
`auto_arm` defaults FALSE; arm explicitly with `scripts/arm_vehicle.py`. Gate offline with
`nmpc_cascade_node.py --test [--lat S] [--comp S] [--kR K] [--decel_at S] [--roll0 D] [--yaw0 D]`.
Measure the loop with `scripts/casc_timing.py <bag>`.

# ★★★★ FULL MISSION FROM 30 m — THE TRANSITION NEEDS NO ALTITUDE (2026-09-08, bag full_tr37_30m)
Takeoff -> transition -> 90 deg cruise -> back-transition -> hover, **all from 30 m**:
```
transition  z 29.0 -> 30.6 m   tilt 12.6 -> 90.0 in ~33 s   AoA pinned +8..+9 (just under the CL peak)
cruise      z 31.2 - 31.5 m    tilt 90.0   V 16.3-16.5   wing 70-99%   rotors 6.5-10.2 N   700 m
verdict     TRUE FIXED-WING (sustained, level, controlled), 275 controlled level samples
back-trans  slowed < 3 m/s, excursion 10.6 m, aft tilt reached its full -20 deg
```
**THE 25 m DESCENT WAS NEVER PHYSICS.** The drag-free floor for reaching the 14.13 m/s stall speed
is 10.2 m of height -- but the aircraft does not have to buy speed with height at all: at 45 deg
tilt the FRONT pair gives 25.5 N forward (7.4 m/s^2) while total vertical thrust is 61.5 N against
a 33.8 N weight. Flights 19/20 descended because the REFERENCE UNDER-DEMANDS THRUST (the trim
treats forward force as a residual). Capping `descent_budget` at **8 m** forces the acceleration to
come from THRUST instead, and the corridor is then crossed with altitude essentially unchanged.

**AND IT FIXES THE ALTITUDE DRIFT.** full_tr36 (110 m, `descent_budget`=45, `cruise_alt_band`=60)
wandered **110 -> 183 m**; this held +-0.3 m for the whole cruise. The drift was the reference being
ALLOWED to wander, not commanding a climb -- a tight `cruise_alt_band` (15 m) contains it.

**30 m CONFIG:** `alt`=30, `descent_budget`=8, `cruise_alt_band`=15, `trans_sink`=0.5, `ttr`=40,
`v_lead`=2.5, `vcruise`=16, `wingborne_v`=14.5, `tilt_margin_deg`=8, `flare_deg`=15,
`beta_aft_deg`=20. This is now the GS "Wing-borne 90 deg (validated)" default; the button's
altitude guard follows `descent_budget + 20 m` (28 m) instead of a hardcoded 60.

**OPEN — the back-transition has a violent pitch transient at low altitude:** +54 deg then -48 deg
within 2 s, AoA swinging +25.6 -> -26.8. It recovers and stops the aircraft, but at 30 m there is
far less room to absorb it than at 110 m. Smooth this before trusting the low-altitude config on
hardware -- likely a slew limit on the commanded flare/aft-tilt rather than the step it applies now.

**TOOLING:** `scripts/restart_hil.sh` now POLLS for Isaac/PX4 readiness instead of fixed 100 s/32 s
sleeps, and takes `--if-needed`, which skips the restart when the stack verifiably passes its
health checks (link up, no poll timeouts, both controllers, no traceback). A repeat data point goes
from ~10 min to ~5. `scripts/watch_flight.sh` prints a live one-line status (procs, controllers,
link, position, tilt, solver ok) so a run is not 3.5 min of silent restart followed by a filtered log.

# ★★★★★ SOLVED — 228 s OF STEADY 90 deg FIXED-WING CRUISE (2026-09-07, bag full_tr30)
**THE TRANSITION WAS TOO AGGRESSIVE.** The phugoid was never a limit cycle and never needed
damping: measured on full_tr20 it was already **DECAYING** (peak-peak 19.5 -> 14.4 m, trend
-0.084 m/s) with a **10.3 s period**. It was a lightly-damped mode RINGING from a large entry
kick -- and with a 1.6 s horizon the solver sees only 16% of a cycle, which is why every cost-term
fix failed. The fix is to NOT EXCITE IT.
**GENTLE TRANSITION: `ttr` 18 -> 40 s, `trans_sink` 2.0 -> 0.8 m/s, `v_lead` 5 -> 2.5,
`tilt_margin_deg` 12 -> 8.** Nothing else changed; all live params, no rebuild.

| | full_tr20 (aggressive) | **full_tr30 (gentle)** |
|---|---|---|
| episode | 41.7 s | **228.3 s** |
| level samples | 13% | **99%** |
| \|vz\| | +-7.5 m/s | **0.07 avg** |
| rotors | 8.7 N | **4.7 N (14% of weight)** |
| altitude swing | 27 m | **11.5 m (mostly the entry)** |
| AoA | swinging to 46 deg | **+6.2 steady** |

**The steady state is an EXACT TRIM:** from t=159 to t=344 s the bag shows
`z=110.88 m, V=15.87 m/s, vz=0.00, tilt=90.0, AoA=+6.2, wing=101.5%, rotors=4.7 N` -- constant to
0.01 m and 0.01 m/s for 184 seconds. Wing carries 101%, rotors supply drag only.
Verdict from the hardened harness (which requires CONSECUTIVE controlled+level samples):
**TRUE FIXED-WING (sustained, level, controlled)**, 417-sample unbroken run = 208.5 s, no
altitude-loss flag.

**LESSON: when a manoeuvre rings, measure whether the oscillation DECAYS before trying to damp it.**
A decaying oscillation is an EXCITATION problem (shape the entry); only a sustained one is a
damping problem (change the cost). This campaign spent four flights -- TECS, a longer horizon, a
force clamp, and a thrust override -- damping a mode that was already stable. One measurement
(peak-peak amplitude vs time) would have redirected all of it.

**FLIGHT-TEST HYGIENE IS NOW ENFORCED: `scripts/restart_hil.sh`.** Restart the FULL stack before
EVERY test. Stale-stack faults cost four flights in this campaign (battery Land failsafe, a
silently dead HIL link, a 34 m/s runaway on a retained reference, and the vehicle 3.8 km from
origin where the safe controller's envelope refused to recover it). The script kills nodes -> PX4
-> Isaac, starts Isaac FIRST (it waits for the heartbeat) then PX4, re-applies the PX4 params that
do not survive a restart, brings both controllers back, and REFUSES to report ready unless there
are zero poll timeouts, Isaac is receiving HIL frames, and the NMPC node has no traceback.
(Check Isaac for the `VTOL_HIL vid=` tag, NOT a motor name -- while disarmed the controls are all
zero, the channel map yields `-> {}`, and a motor name never appears on a perfectly healthy link.)

### ★★★ AFT ROTOR TILT — THE BACK-TRANSITION BALLOON SOLVED (2026-09-07, bag full_tr32)
**User's insight:** with `beta >= 0` the ONLY decelerating force is nose-up pitch, and that
converts speed into HEIGHT -- it cannot be tuned away. At 15.87 m/s the aircraft carries 434 J,
which is EXACTLY 12.8 m of climb. Letting the FRONT pair lean AFT puts the braking force along the
flight path instead, so the body stays closer to level and the energy goes into drag, not altitude.

**ENABLED: `tilt_min_deg` 0 -> -20 on the front rotors**, `BETA_MIN`=20 deg in the NMPC bounds,
and the aft tilt is COMMANDED IN THE REFERENCE (`beta_aft_deg`) with `(Tf,Tr)` RE-SOLVED by
`so3.trim_split` for the negative beta, so the reference stays a balanced package (w_beta=60 holds
applied tilt to the schedule, so leaving it to the solver would not have worked).

| back-transition | none (full_tr30) | flare only (full_tr31) | **flare + AFT TILT (full_tr32)** |
|---|---|---|---|
| altitude excursion | 65.7 m | 18.9 m | **2.2 m** |
| slowed to hover | never (>5 m/s) | yes | **yes** |
| peak \|AoA\| | +78 deg (stalled) | 28 deg | (102 at 2.4 m/s -- no dynamic pressure) |

**2.2 m against a 12.8 m floor for pure energy conversion** proves the aft tilt is DISSIPATING the
energy rather than storing it as height. Flown: tilt walks 77 -> +9 -> **-15 deg** and HOLDS aft
while speed collapses 13.3 -> 2.35 m/s.

**★ THE SIM'S TILT MAPPING HAD TO BE RE-ANCHORED FIRST.** It was `angle = lo + (hi-lo)*cmd`, so
setting `tilt_min_deg=-20` would have made hover's `cmd=0` mean **-20 deg** -- and the SAFE
controller writes literal ZEROS on the tilt channels. That is the same class as the old
`(4,0.5,0.5)` channel map which made hover's 0 into 45 deg and tilted the aircraft on arm.
`_tilt_target_rad` now scales against `tilt_max` and CLIPS to `[tilt_min, tilt_max]`, so **cmd=0 is
always 0 deg** and the change is bit-identical for every airframe with `tilt_min_deg=0`. The
nacelle spawn state also had to move from `tilt_min` to 0, or the rotors spawn leaning back.

**★ TUNED, AND A CORRECTION: THE FLARE AND THE AFT TILT ARE COMPLEMENTARY, NOT SUBSTITUTES.**
I first credited the 18.9 -> 2.2 m improvement to the aft tilt alone -- but full_tr32 changed TWO
things at once (flare 25->15 AND aft 0->20), so it never isolated anything. Cutting the flare to 5
to "let the aft tilt carry the braking" **DEPARTS the aircraft** (full_tr35): the aft tilt still
brakes hard (15.7 -> 9.7 m/s in 2 s) and then AoA runs to **172 deg** -- flying backwards -- and it
falls 40 m. The nose-up flare is not merely extra braking; it is what keeps the deceleration
CONTROLLED.

| flare | aft tilt | result |
|---|---|---|
| 25 deg | 0 | stops, 18.9 m balloon |
| **15 deg** | **20 deg** | **stops, 2.2 m** <- validated |
| 5 deg | 20 deg | **DEPARTS** (AoA 172, -40 m) |

**VALIDATED BACK-TRANSITION: `flare_deg`=15, `beta_aft_deg`=20.** Never lower the flare below ~15
while aft tilt is enabled.

**TWO HARNESS BUGS found here, both mine, both worth keeping:**
  * **A MOVING CARROT never lets the vehicle stop.** The back-transition sent a target 40 m ahead
    of the CURRENT heading every tick, so there was always somewhere to go: full_tr34 chased it for
    70 s in a slow turn and only bled 16.3 -> 12.1 m/s. The stop point must be LATCHED ONCE.
  * **A FROZEN SIM reads as a successful manoeuvre.** full_tr33 reported "slowed below 3 m/s: True"
    while airspeed sat at 16.60 for the entire window -- PX4 had been killed and mocap was
    repeating its last sample. `fly_transition.py` now hashes (position, V) each sample and prints
    `!! SIMULATION FROZEN -- THIS RUN IS INVALID` after 4 identical reads.

### ★★ BACK-TRANSITION (cruise -> hover) — IMPLEMENTED (2026-09-07, bag full_tr31)
**There was no back-transition at all.** `fly_transition.py` handed a 16 m/s WING-BORNE aircraft
straight to the multicopter controller, which writes tilt to literal ZERO. Bag full_tr30 shows the
result: lift doubled, it ballooned 10 m, stalled at **AoA +78 deg** and departed into a dive --
**65.7 m excursion, never slowed below 5 m/s**. That failure hid behind the "recover" label for the
whole campaign because the harness only ever judged the CRUISE.

**FIX (two parts, both in the reference):**
1. **Run the trim schedule BACKWARDS** (`decel_now`, `decel_rate`): the commanded speed bleeds down
   and the schedule hands back tilt/pitch/thrust per speed, so the rotors come upright IN STEP with
   the wing unloading instead of snapping.
2. **NOSE-UP FLARE** (`flare_deg`=25). A tiltrotor cannot tilt backwards (beta>=0), so with the body
   LEVEL there is NO decelerating force. Pitching nose-up vectors the near-vertical rotor thrust
   BACKWARD (`T*sin(theta)`) and adds high-AoA drag. Faded with speed so it is strongest while there
   is speed to kill. Flown: pitch reaches **+53 deg** while tilt walks 90 -> 46, killing
   **15.9 -> 6.9 m/s in 5 s**, and **AoA stays at +3.3** because the flight path climbs with the
   nose -- it never stalls.

| | full_tr30 (none) | **full_tr31 (flare)** |
|---|---|---|
| slowed to hover | never (>5 m/s) | **yes, < 3 m/s** |
| altitude excursion | 65.7 m | **18.9 m** |
| peak \|AoA\| | +78 deg (stalled) | **28 deg** |
| peak \|vz\| | -13.6 m/s | **6.3 m/s** |

**THE REMAINING BALLOON IS MOSTLY PHYSICS.** At 15.87 m/s the aircraft carries 434 J; converted
entirely to height that is **12.8 m of climb**, so 68% of the observed 18.9 m is unavoidable
kinetic->potential conversion. Only ~6 m is excess. To remove it the energy must be DISSIPATED as
drag (longer, higher-AoA decel) or absorbed by commanding a descent through the flare.

**CRUISE REFERENCE TRACKING IS EXCELLENT** (measured on full_tr30 over 184 s):
airspeed cmd 16.00 -> **15.867 +- 0.001** (-0.8%), altitude **110.875 +- 0.0022 m** (12 mm of drift),
climb rate **0.0001 +- 0.0017 m/s**, tilt 90.00 exact. The only structural error is the 0.8% speed
offset, which is the trim treating forward force as a RESIDUAL rather than tracking it -- the same
root cause as the coasting problem, and what `trim_split(..., Fx_req=)` would fix.

# ★★★ 90 deg WING-BORNE FLIGHT SUSTAINED -- BUT IT IS A PHUGOID, NOT STEADY CRUISE
# (2026-09-07, bag full_tr20)
**READ THE CORRECTION AT THE END OF THIS SECTION BEFORE QUOTING ANY NUMBER HERE.**
`nmpc_full_smc_node` on the corrected plant: **tilt 90.0 deg, rotors 8.6-8.9 N of a 33.8 N weight
(drag thrust only -- at 90 deg they make NO lift), wing carrying 175-242%, V 14-15 m/s avg (peaks
20-23), altitude START 110 m -> END 111 m, NO altitude-loss flag.**
Episodes: **41.7 s, 38.7 s, 29.4 s, 25.6 s, 23.4 s -- ~159 s of cumulative 90 deg flight.**
Verdict from the (hardened) harness: TRUE FIXED-WING (sustained, level, controlled).

**THE THREE CHANGES THAT DID IT, in order of size:**
1. **THE SETTLE GATE MUST LATCH (bag full_tr15/16).** `settling` was re-evaluated EVERY TICK as
   `(...) or (z < z0 - 1.0)`, so a **1 m** altitude dip in cruise threw the vehicle back to the
   hover reference -- tilt to 0 at 12 m/s -- while it was holding altitude to within 1 m. Latch it
   once satisfied; in cruise only revert on a real `cruise_alt_band` loss. 4.2 -> 6.2 s, tilt 83.
   FOURTH instance of a reference surrendering to its own tracking error.
2. **THE REFERENCE MUST COMMAND THE DESCENT, NOT MERELY PERMIT IT (bag full_tr18/19).** The
   altitude reference was FLAT through the corridor. Below the 14.13 m/s stall speed the wing
   cannot carry the weight, so obeying it forced the solver to buy lift with ANGLE OF ATTACK:
   full_tr18 mushed at **AoA +28..+35 deg at 8-9 m/s**, deep on the BACK SIDE of the drag curve
   where it can never accelerate out. Widening `cruise_alt_band` did NOTHING because that only
   relaxes the settling GATE -- the COST still demanded the old altitude. `build_refs` now commands
   a bounded SINK (`trans_sink`) while below `wingborne_v`, then levels off. **AoA +28..35 -> +12.7**
   and tilt hit a true 90 deg. PERMISSION IS NOT DEMAND.
3. **GIVE THE MANOEUVRE ENOUGH ALTITUDE (bag full_tr19/20).** The dive-out needs ~25 m. Starting at
   50 m with a 25 m band put the settling threshold at 25 m, and full_tr19 was dumped at z=24.8 m
   **at the exact moment it succeeded** (18.78 m/s, 90 deg, wing 141%). At 110 m with
   `descent_budget`=45 / `cruise_alt_band`=60 it completes the trade and stays wing-borne.

**CONFIG:** `alt 110, dist 2500`; `trans_sink`=2.0, `wingborne_v`=14.5, `descent_budget`=45,
`cruise_alt_band`=60, `v_lead`=5, `tilt_margin_deg`=12, `latency_s`=0.12, `rate_hz`=15,
`w_aoa`=1.5 with the **^4** exponent, N=8/DT=0.20 (1.6 s horizon). Safe controller `z_max` raised
to 130 to allow the 110 m start (a SAFETY-RELEVANT widening -- review before hardware).
**★★ CORRECTION -- IT IS NOT STEADY CRUISE.** Inside the 41.7 s episode the vehicle is
PORPOISING: altitude cycles **63-90 m**, speed swings **7-20 m/s**, vz reaches **+-7.5 m/s**, and
only **642 of 4992 samples (13%) are level**. AoA averages +11 deg but peaks at 46. What is TRUE:
it stays airborne ~159 s, returns to its start altitude (110 -> 111 m), and repeatedly reaches a
genuine 90 deg tilt with the rotors at ~10 N while the wing carries the aircraft. What is NOT
true: steady 1-2 minute cruise.
**AND THE HARNESS FLATTERED IT AGAIN (the 4th time).** At the BOTTOM of each phugoid cycle `vz`
passes through zero, so those instants pass an INSTANTANEOUS `|vz| < 1.5` test while the aircraft
is oscillating violently -- the headline sample (22.80 m/s, 90 deg, wing 238.9%, z=63.9) is exactly
such a point. `fly_transition.py` now requires the longest UNBROKEN RUN of consecutive
controlled+level samples, not the total count. Instantaneous level is not steady flight.
**REMAINING: DAMP THE PHUGOID.** That is now the whole problem -- the corridor is crossed, 90 deg
is reached and held, and the vehicle is stable in the large. The oscillation is the ~4 s phugoid
the 1.6 s horizon still cannot see a full period of; candidates are a longer horizon (DT 0.20 ->
0.30 was never tried), an explicit vz/flight-path damping term, or an energy (total-energy)
formulation rather than separate altitude and speed tracking.

# ★★★ AoA GUARD BREAKS THE PLATEAU (2026-09-07, bag full_tr11) — 52% wing share, 72 deg tilt.

**The 8.4-8.7 m/s plateau (identical across FOUR tunings) was ANGLE OF ATTACK.** The AoA swung to
+-46 deg against a 13 deg stall break; every excursion collapsed CL, dumped the wing's share back
onto the rear rotors, broke the pitch trim and bled the speed just built. Nothing in the cost knew
what AoA was. Added `_aoa_excess` + `w_aoa` to `nmpc_full_smc_node`: **max |AoA| 46 -> 32 deg, and
in LEVEL flight it reached 10.5 m/s at 72 deg TILT with the wing carrying 52% of weight and the
rotors down to 16.8 N from 33.8** (vs 36% at 50 deg before). Biggest single gain of the campaign.
  * **SMOOTH form is mandatory.** My first version, `fmax(0, |alpha| - lim)`, has a corner exactly
    at the limit -- where corridor trajectories ride -- and `fabs` adds a second at alpha=0, which
    the vehicle crosses constantly. This project already paid for that once (the piecewise
    post-stall break -> IPOPT convergence failures at the transition tilt angles). Use the even
    power `(alpha/lim)^6`: 0.04 at the +5.8 deg cruise trim, 365 at 30 deg, no corner anywhere.

**★★ DO NOT TRUST A PEAK. full_tr11 reported "peak 15.83 m/s, 120% wing share, TRUE FIXED-WING"
and it was a DIVE** -- it traded 17 m of altitude for speed (20 m -> 3.7 m -> 0.33 m) and the safe
controller caught it at the ground. Everything measured while descending is gravity, not lift.
`scripts/fly_transition.py` now judges ONLY on SUSTAINED LEVEL samples (|vz| < 1.5 m/s, >= 10 of
them) and appends a warning if the flight lost more than 5 m of altitude.

**TWO SILENT-FAILURE MODES cost a flight each; both looked completely healthy:**
  * the SAFE controller's envelope was measured from the ORIGIN only, so after 1430 m of
    cumulative flight it silently refused every command -- the one controller whose job is to
    recover the vehicle could not even hold station. **A safety fallback must always be able to
    operate where the vehicle IS**: the check is now `within radius_max of home OR radius_local
    (100 m) of the vehicle`.
  * **PX4 latched "Landing detected" at 21 m** because the vehicle sat motionless for minutes
    during debugging; actuator output was inhibited from then on. Symptom in both cases:
    everything nominal, nothing moving, no error anywhere. Watch for it.

**REMAINING:** it reaches ~10.5 m/s level (stall 14.1). The next lever is the HORIZON: 0.8 s
(N=8, DT=0.10) against a ~4 s phugoid at 9 m/s -- the solver cannot see the mode it is exciting.
Try DT 0.10 -> 0.25 (2 s for the same solve cost; solve is 17-34 ms of a 100 ms budget).

# ★★★ FULL-STATE NMPC — HIL CAMPAIGN (2026-09-07). Stable, altitude-perfect, SPEED-LIMITED.

Five HIL flights of `nmpc_full_smc_node` on the corrected plant (bags `full_tr1..full_tr5`).
Went from "departs at 4.2 m/s and falls" to "flies 317 m holding altitude to +-0.2 m" -- but it
CANNOT build speed past ~5 m/s, so it never reaches the corridor (stall 14.1, wing-borne 15.6).

**THREE bugs found by FLYING, all invisible to the offline test:**
1. **`R_ref` built `Ry(+theta)` = NOSE-DOWN while its docstring said nose-up** -- the FIFTH copy
   of the 2026-09-06 inversion; four were fixed and this one was missed. The node's own log
   printed `pitch=-77` where the truth was `+87`: it MEASURED pitch in the old convention while
   the trim schedule fed it the NEW one, so past a few degrees the attitude error changed sign
   and the loop DROVE the divergence. (Also fixed: both `asin(R[2,0])` readouts.)
2. **`w_beta` 12 -> 60.** A soft tilt guide leaves a nearly flat tilt-vs-pitch optimum and the
   solver hunts along it (applied tilt 17->5->24->13 deg against a steady 21-24 reference).
   `vtol_nmpc` raised it for exactly this and it was never back-ported -- audit finding C9.
3. **The tilt servo was modelled as a first-order lag with NO rate limit** while the sim
   implements a pure 80 deg/s RATE LIMIT (`rotor_dynamics` off). The solver believed it could
   swing the tilt arbitrarily fast. Fixed in `so3.build_ode` with a tanh saturation
   (`betadot = TILT_RATE*tanh((betacmd-beta)/(TAU_BETA*TILT_RATE))`) and the node's own tilt
   estimate now integrates the same law. **This is audit finding C10**, documented and unfixed
   since the audit. It measurably improved tilt tracking (hunting +-7 deg -> within 2-3 deg)
   and raised the speed ceiling 4.2 -> 5.6 m/s.

| flight | outcome | distance | altitude | peak V | peak wing share |
|---|---|---|---|---|---|
| full_tr1 | departed 4.2 m/s | 13 m | lost | 4.1 | 9.8% |
| full_tr2 | departed 4.2 m/s | 13 m | lost | 4.2 | 7.5% |
| full_tr3 | **survived** | 152 m | 20.1-20.3 | 4.3 | 8.5% |
| full_tr4 | survived (departed in RECOVER) | 373 m | 20.1-20.2 | 4.7 | 13.3% |
| full_tr5 | **survived** | 317 m | 20.0-20.2 | 5.6 | ~13% |
| full_tr6 | **survived, PARTIAL WING-BORNE** | -- | ~20 | **8.45** | **36.1%** |

**★ BREAKTHROUGH (flight full_tr6): THE REFERENCE WAS SURRENDERING TO ITS OWN TRACKING ERROR.**
`build_refs` computed `vtar = min(V_now + k*DT*(Vc/ttr), Vc)` from the **MEASURED** speed, so over
the 0.8 s horizon it could only ever ask for `V_now + 0.7 m/s`. When the pitch/tilt coupling bled
speed off, the reference re-anchored to whatever was left and followed the vehicle DOWN -- there was
no persistent demand to accelerate. **This is the THIRD instance of the same pathology in this
project** (the SMC re-seeding on tracking lag = C7, the GS mission clock freeze, and now this).
Treat it as a design rule: a reference must advance on its OWN clock with a BOUNDED lead, never
re-anchor to the achieved state.
FIX: the node carries `self._v_cmd` on a clock (ramps at `Vc/ttr`, clamped to `V_now + v_lead`).

| flight | change | peak V | tilt | wing share | verdict |
|---|---|---|---|---|---|
| full_tr5 | rate-limited tilt model (C10) | 5.6 | 33.5 | 13.3% | multicopter |
| full_tr6 | **+ speed on its own clock** | **8.45** | **50.4** | **36.1%** | **PARTIAL WING-BORNE** |
| full_tr7 | + decouple demand/feed-forward | 6.91 | 46.7 | 25.9% | WORSE -- reverted |
| full_tr8 | lead 3 -> 5 | **8.71** | **60.1** | 32.0% | PARTIAL WING-BORNE (fastest, highest tilt) |

**★ DO NOT RETRY (full_tr7):** splitting the reference into a leading "demand" speed for the STATE
and a measured-anchored "feed-forward" speed for the CONTROL. The reasoning is tempting (the trim
split is only valid at the speed it was solved for) but the TILT appears in BOTH `x[12]` and `u[4]`,
so the split told the solver two tilts ~18 deg apart and the reference stopped being a
self-consistent trim. **A trim is a package -- do not take it apart.** If the feed-forward is too far
ahead, shorten `v_lead` (which moves the whole package back together). Recorded in the source.
**`v_lead` 3 vs 5 -- COMPARABLE, not a clear win either way** (I first called 5 "worse" from a
mid-flight log sample instead of the summary; that was wrong):
  * lead 3 (full_tr6): V 8.45, tilt 50.4, wing 36.1%
  * lead 5 (full_tr8): V **8.71**, tilt **60.1**, wing 32.0% -- FASTEST and HIGHEST TILT of any
    flight, but rougher: AoA swinging +-46 deg and more `ok=0` solve failures at 46-58 ms.
Default left at 3.0 for the smoother ride; 5.0 is the one to revisit if the oscillation is damped.
**CONFIG:** `v_lead=3.0`, `latency_s=0.12`, `w_beta=60`.
**LESSON:** judge a flight from its SUMMARY, never from a mid-flight log sample -- the vehicle is
oscillating, so any single sample says whatever the phase of the cycle says.

**THE REMAINING PROBLEM IS A DESIGN ONE, NOT A BUG.** A pitch/tilt/speed LIMIT CYCLE: tilt rises
toward its schedule -> the solver commands a big nose-up (mot front-heavy 0.62/0.25 AND elevator
saturated nose-up TOGETHER) -> pitch +19..+41, AoA +18..+30 -> speed bleeds off -> tilt backs off
-> repeat at ~0.5-1 Hz. Altitude is held perfectly THROUGHOUT (that is the point: it is buying
altitude with pitch because at 5 m/s the wing supplies ~5% of weight while the schedule's 26-31 deg
tilt removes ~15% of vertical thrust). **The cost has pos/vel/att/tilt/rate terms but NOTHING that
rewards BUILDING AIRSPEED through the corridor**, so the solver is content to sit at 3-5 m/s
slowly satisfying a position reference. Candidate fixes (NOT attempted -- needs a design call):
an explicit airspeed-tracking term, a corridor-crossing time constraint, or committing the tilt
open-loop through the corridor while the wing loads.

**Ops notes:** solve is 17-34 ms in HIL (NOT the 43-96 ms this node was tuned around), so
`latency_s` 0.25 was ~2x over-compensating; 0.12 is closer (live `ros2 param set`, no rebuild).
`w_beta`/`build_ode` changes DO force a ~4 min JIT rebuild (the widened cache key working as
intended -- the old August caches have different hashes and were correctly NOT reused).
**The safety architecture worked every time:** `scripts/fly_transition.py` aborts on altitude loss
or attitude departure and the safe controller recovered the vehicle to a stable hover from 20 m on
all four departures. Every earlier attempt in this project ended inverted on the ground.

**★ THE OFFLINE TEST PASSED WITH BUG 1 PRESENT** and reported textbook wing-borne numbers
(tilt=88, AoA=+5.2, rotors 11.3 N). It has no disturbances and no HIL timing, so a near-level
inverted attitude reference tracks fine -- the sign only matters once something perturbs you.
Model-matching the plant fixed finding C5's worst form; it did NOT make a benign test
discriminating. Treat `--test` as a numerical-stability check, never as a flight gate.

# ★★★ HIL VALIDATED (2026-09-06) — the safe controller flies full missions, first attempt.

`geometric_hover_node.py` on the corrected plant, `CONTROLLERS="geometric"` (bag `/tmp/bags/geo1`,
421 s, 50k mocap / 84k actuator samples). **Publisher count on actuator_motors: 1** — the
CONTROLLERS selector removes the ghost-publisher risk by construction, no kill race.

- **THREE complete missions, no failures, no retries.** Ground-to-ground: takeoff -> 47 m diagonal
  at 12 m -> descend -> land, `scripts/fly_mission.py` (NEW: completion-gated waypoint driver with
  telemetry, same advance rule as the GS Fly-to-point button).
- **Precise landing: hold error 6 mm mean / 15 mm max**, drift ~1 cm over 20 s. Arrival errors are
  the commanded tolerance (0.08-0.10 m), not a limit.
- **Level-cruise altitude hold 0.09-0.27 m at ~5 m/s** (offline plant double predicted 0.23 -- the
  double is faithful).
- **Hover steady state: 2 mm position spread, 0.04 deg attitude**, motors `[0.474, 0.475, 0.465,
  0.466]` -- the front>rear split the CG geometry predicts (balanced hover 8.55 N front / 8.37 N
  rear) and exactly what `tests/plant_double.py` predicted (0.470 mean).
- **Tilt servos identically 0.000000 across 84k samples** -- "cannot tilt, by construction" holds
  in flight, not just in review.
- **No actuator freeze, no gaps > 0.5 s, no LOOP EXCEPTION, no PX4 failsafe.**
- Envelope guard verified: a target at 400 m / 90 m (limits 300/60) is ignored and the vehicle
  does not move.
- Peak |roll| 13.4, |pitch| 17.4 deg -- well inside the 35 deg lean cap. Peak speed 5.1 m/s vs
  `v_max`=4.0 (the ramp limits the SETPOINT; the vehicle overshoots ~25% on a diagonal where the
  horizontal and vertical limits add -- harmless, worth knowing).

NOT yet flown: the wing-borne controllers (nmpc6 / nmpc_full / vtol_nmpc). They need their JIT
caches rebuilt (the keys now hash the model source, so the first launch rebuilds ~2 min) and have
never flown against the corrected plant.

# ★★★ READ FIRST — AERO COEFFICIENTS + CONTROLLER PROPAGATION + REFERENCE TRIMS (2026-09-06)

**Coefficients re-derived from the airframe's OWN surfaces** (`tests/test_aero_consistency.py`
checks this and will fail if they drift again):
- `cl_aileron` 0.08 -> **0.49**. 0.08 was the AVTOL's (small outboard ailerons on a 1325 kg
  aircraft). THIS aileron is 0.35 x 0.12 m on a 1.4 m span / 0.25 m chord wing = 50% span,
  48% chord -- an ELEVON; the strip estimate is 0.49. **At 0.08 the ailerons NEVER overtook
  rotor differential-thrust roll, so as the rotors tilt to 90 deg (where they stop making
  roll) there was no hand-over and roll authority genuinely vanished.** At 0.49 the cross-over
  is 8.9 m/s, below the 11.5 m/s stall. This is very likely the lateral fragility that
  defeated every 90-deg attempt.
- **ALL-MOVING surfaces:** vtol.py builds each surface as a whole hinged box (no fixed
  stabiliser/fin), so tau=1 and the derivative is the full volume x slope. `cm_elevator`
  -1.1 -> **-1.75**, `cl_elevator` 0.4 -> **0.70**, `cy_rudder` 0.17 -> **0.24**,
  `cn_rudder` -0.12 -> **-0.109**. The fin is now EXACTLY self-consistent
  (|Cn_dr|/|CY_dr| = l_v/b = 0.447). Pitch hand-over moves 14.1 -> 11.2 m/s.
- `cn_beta` 0.25 -> **0.11** (= fin-alone V_V*a_v; a fuselage only REDUCES weathercock, and
  the pair (0.25,-0.98) claimed Cn/CY = 0.26 where the fin arm allows 0.45). `cy_beta` LEFT at
  -0.98 (defensible once fuselage side area is counted).
- **SIDE FORCE now saturates** (`cy_max`=0.6, `sideslip_break_rad`=20 deg): the linear fin term
  extrapolated to beta=90 claimed 16 N in a 6 m/s crosswind (half the weight) and double-counted
  with `cd_beta2`. Blends to a flat plate, same as lift does in alpha -> 5 N. Symmetric fix.
- Stall blend rewritten as the **tanh form** of the Beard sigmoid (identical to 3e-3, saturates
  instead of overflowing) so the CONTROLLERS can evaluate the same expression in CasADi.

**Velocity scaling (the answer to "which axis"):** the sim scales on the FULL 3-axis |v_body|;
static terms ~V^2, rate-damping ~V (phat/qhat/rhat divide one power back out). Aero surface power
is V^2 while ROTOR authority is flat (no inflow model, finding F6) -> hand-over speeds roll 8.9 /
pitch 11.2 / yaw 12.3 m/s. The CONTROLLERS used to gate on `u^2/(u^2+4)` -- FORWARD speed only --
so they modelled ZERO aero in descent and lateral translation where the plant makes real force.

**CONTROLLER PROPAGATION (finding C1 + C2/C3/C4 + the velocity fixes):**
- CONVERTED + VERIFIED against the plant by `tests/test_controller_aero_match.py` (0.01 N):
  `nmpc_so3_transition_node.aero_body` (the hub -- nmpc6, nmpc_full, cascade share it),
  `vtol_nmpc._aero_body`, `nmpc_so3_cascade.aero_world`, `nmpc6.build_ode6`.
  Recipe: convert FLU->FRD in and back out, publish-standard coefficients, no sign patches.
  The `u^2/(u^2+4)` gate is GONE (V is floored at MIN_V instead).
- **PITCH CONVENTION FLIPPED:** `_R_pitch` / `R_ref` / nmpc6's `theta` are now **+theta = NOSE-UP**,
  matching the trim schedules. They used to build Ry(+theta) = nose-DOWN, which cancelled against
  the inverted aero: a +8 deg cruise trim was flown 8 deg nose-DOWN. `theta_act = +asin(R[2,0])`.
  All diagnostic `aoa` prints are now the STANDARD angle (old bags have the opposite sign).
- Constants: M 3.5->3.45, **I = diag(0.265, 0.303, 0.490)** (ASSEMBLED, was the body link's
  0.20/0.28/0.35), **CQ 0.06 -> 0.12** (= rotor_moment_ratio), **RPOS z front -0.033 / rear +0.016**
  (was +0.02 for all four -- wrong SIGN on the front pair, and it IS the pitch moment at 90 deg tilt).
- **JIT cache keys now hash the MODEL SOURCE** (`inspect.getsource`) + geometry/inertia/CQ, not just
  the weights -- otherwise a model change silently reloads a stale compiled solver (C6).
- nmpc6's surfaces are **q-bar NORMALISED** (`delta = M / (qbar*S*span*C_delta)`) instead of the
  hand-tuned `k_ail/k_rud` behind an airspeed ramp: the delivered moment matches the demand at
  every speed and the SIGN comes from the coefficient (the +k_rud sign trap ran yaw to -100 deg).
- **NOT converted, and they now REFUSE TO START:** `nmpc_cruise_node`, `nmpc_quad_node`,
  `aero_transition_node` (`FSC_ALLOW_STALE_MODEL=1` overrides, offline only).

**★★ REFERENCE PROFILE AUDIT — the schedules were NOT trims.** Plugging (xref,uref) into
`build_ode` left up to **5.3 rad/s^2 of residual pitch** and 1.5 m/s^2 vertical. Two causes:
1. the aero pitch moment entered the front/rear split WITHOUT being negated into the FLU
   (nose-down positive) convention the thrust moment is written in -- correct for the old
   flipped Cm, wrong for the corrected one;
2. **the AVTOL generator returns ONE Ttot on the premise that the WHOLE rotor set tilts.** This
   airframe tilts only the FRONT pair; mid-corridor that under-counts the vertical thrust by
   7.8 N = 23% of weight.
FIX: keep the generator's ATTITUDE schedule (tilt, alpha, theta, de) and SOLVE the two thrusts
from this airframe's geometry for both balances at once --
`Tf*cos(beta-theta) + Tr*cos(theta) + Fz_aero = W` and `Tf*(zf sinb - xf cosb) - xr*Tr + My_aero_FLU = 0`
-- with the aero taken from the plant's own model. Residual now **az = 0.000, |qdot| <= 0.04**, and
`ax` is the intended +2 m/s^2 transition acceleration tapering to +0.2 at cruise.
`vtol_nmpc.TRIM` REGENERATED from it. `tests/test_reference_profiles.py` guards all of this.

**★★ ALL FOUR REFERENCE PATHS now audited — the two I had only sign-fixed were still wrong:**
- **`TransitionRef.get` (what the SO(3) node flies):** pitch was trimmed but VERTICALLY NOT
  (az up to 1.45 m/s^2) -- it still used the generator's `Ttot`. Now 0.000.
- **`fsc_mission_ref` (the full mission) was commanding a FREE FALL.** Its `get()` built
  `Rp = Ry(+theta)` = NOSE-DOWN while `theta` is constructed as `alpha + gamma` (nose-up) --
  the SAME inversion as `_R_pitch`/`R_ref`, missed in the first pass. In cruise the wing flew
  at NEGATIVE AoA: **az = -10.10 m/s^2, qdot = -9.02 rad/s^2**.
- **ONE shared solver now: `nmpc_so3_transition_node.trim_split(beta, theta, V, de, Fz_req,
  gamma, Fx_req, beta_max)`.** Used by nmpc6.Schedule, TransitionRef.get and fsc_mission_ref.
  With only (Tf,Tr) you can satisfy TWO demands; vertical+pitch use up all the freedom and the
  FORWARD acceleration is then whatever falls out (reference asked +2.42 m/s^2, plant gave
  +1.13). Passing `Fx_req` makes it solve the TILT as a third unknown (Fx is monotone in beta,
  so bisection). `Fz_req = M*(G + zdd)` so a climb/descent leg is trimmed for what it is
  ACTUALLY doing, not for level flight.
- **BRAKING PITCH (mission):** a tiltrotor cannot tilt backwards (beta>=0), so with the body
  LEVEL there is NO decelerating force -- the profile commanded -3.5 m/s^2 and the plant
  produced -0.02. The reference now commands NOSE-UP during a decel leg (`theta = asin(-a/g)`),
  bounded by `alpha_brake_max_deg`=12 so the total AoA stays inside the linear range. THAT
  BOUND is what limits this airframe to ~1-2 m/s^2 of braking, and it is the real reason a
  mission overshoots a finite target -- it is a REFERENCE limit, not a controller failure.
  `decel_cap` 3.5 -> 2.0; the speed profile is re-integrated from the CLIPPED acceleration so
  the profile and the trim describe the same manoeuvre (they disagreed: -4.5 vs a 3.5 cap).
- FSC `m` 3.5 -> **3.45** (assembled) -- 3.5 left a steady 0.14 m/s^2 bias in every reference.
- **Mission residual: 4.76 -> 0.162 m/s^2, pitch -9.02 -> 0.029 rad/s^2.**
- PROCESS NOTE: a `try/except Exception` around the new solver silently swallowed an
  AttributeError and the fix appeared to do nothing for a whole iteration. Don't wrap a
  numerical path you are trying to verify.
- Probe trap hit AGAIN: `xref` stores R **row-major**, and numpy's reshape is row-major, so
  `np.array(x[3:12]).reshape(3,3)` needs **NO transpose** -- the `.T` is only for `ca.reshape`
  (column-major). A stray `.T` made a correct nose-up reference read as nose-down.
Trajectory files: `aircraft_transition/quad_forward/quad_transition.csv` were UNSCALED AVTOL
(30-40 m/s, up to 603 m -- 2-3.5x this airframe) -> moved to `trajectories/unscaled_avtol/` with a
README; `full_mission_transition.csv` had velocity columns 1.9 m/s off its own position track ->
rewritten; GS playback fence 300 -> 500 m (the FSC missions reach 378 m).

# ★★★ READ FIRST — CONTROLLER SPLIT + STREAMLINE (2026-09-06). Two jobs, two controllers.

**SAFE HOVER = `ros2/.../geometric_hover_node.py` (id `geometric`).** It flies to the x,y,z the
operator types and HOLDS it. A PURE MULTICOPTER -- the tilt servos are written as literal zeros
and there is no tilt parameter to set. No aero model, no trajectory clock, no outer loop: it
generates its OWN velocity-limited motion profile (`SetpointRamp`, sqrt stopping profile, matching
v/a feed-forward), so a step target from the GS is smooth by construction. This is where STOP
always sends the vehicle, so its behaviour is deliberately boring. It OWNS: the GS "Set Position"
panel, "Fly to point + LAND", and STOP.
**TRAJECTORIES / PATHS / TRANSITION = the NMPC nodes.** Unchanged. Playing a trajectory on
`geometric` is now REFUSED by the GS (it chases points, not paths).
(`traj_track_node.py` is DELETED -- superseded. `start_vtol_minimal.sh` is now just a CONTROLLERS list.)

Streamline (details in the closed-loop audit):
- **ONE reference topic on the safe path.** `geometric` reads the RAW reference; it is NOT SMC-fed.
  Removed: the GS's second publisher on `reference_smc` (it INTERLEAVED with smc_outer's own 50 Hz
  output -- two writers on the safety path), and the differentiated feed-forward with its hardcoded
  `dt_ref=1/30`. smc_outer still feeds the NMPC nodes that were tuned around it.
- **`CONTROLLERS` env selects which nodes start** (default `geometric` alone; `all` = old behaviour).
  Every standby node still SOLVED at full rate and competed with Isaac/PX4/GS for CPU -- the 19-state
  solve is 22 ms idle vs 48-96 ms under HIL load, and an overrunning loop is a documented tumble cause.
  `scripts/start_vtol_minimal.sh <cfg> [controller]` = that list (was: launch all, then kill windows).
- **GS mission = 3 COMPLETION-GATED WAYPOINTS** (climb / traverse / descend), replacing the open-loop
  min-jerk clock + lead-clamp + clock-freeze (all of which existed because the old controller couldn't
  keep up with its own reference). GO/HOLD, Fly-to-point and STOP now FORCE `geometric` (they used to
  use whichever button was last pressed). Playback geofence RE-ENABLED (300 m / 60 m; it had been set
  to 1e7 after a dead-call bug).
- **Fixed in the safe controller** (audit C3/C12): `cg_x` 0.195 -> **0.176** (the ASSEMBLED CG) and
  `cQ` 0.06 -> **0.12** (= the sim's `rotor_moment_ratio`); anti-windup is now clamp+back-calculation
  + freeze-while-saturated with `i_max` 3 -> **8 N** (3 N could not trim a 5 N wind side-force, and the
  1 m integral BAND refused to act at all once the offset exceeded it); the integral FADES with
  setpoint speed (what it charges in transit is drag, which discharges as overshoot on arrival);
  attitude-priority desaturation; the loop is exception-wrapped (bag vt1).

**★ TEST HARNESS THAT CAN SEE MODEL ERROR — `tests/plant_double.py`.** 6-DOF plant built from the
SIM's own `vtol_aerodynamics.py` + the assembled mass properties of `quad_tiltrotor_config`
(M=3.45, CG_x=0.176, I=diag(0.265,0.303,0.490)). Reproduces the sim's measured hover split
(8.55/8.37 N vs measured 8.6/8.4). Use it instead of a controller's own `build_ode` -- that is the
C5 finding (the old selftests validate each controller against itself, so model error cancels).
Tests: `tests/test_geometric_hover.py` (26 closed-loop assertions; `--latency 0.1`),
`tests/test_vtol_aerodynamics.py`, `tests/test_gs_trajectory_tab.py` (real GS tab, offscreen,
run with `/usr/bin/python3`). All pass. NONE of it is HIL-flown yet.
**MEASURED: the safe controller's loop-latency margin is 125 ms** (stable to 125, diverges at 150 --
the ATTITUDE loop goes first, kR=9 at 200 Hz). Budget for mocap->DDS->ctrl->DDS->PX4->Isaac.

# ★★★ READ FIRST — AERO FRAME FIX (2026-09-06). Everything below it predates this.

The sim's aerodynamic model had computed `alpha`/`beta` from the **FLU** body velocity and fed
them into **FRD**-published stability derivatives. FLU->FRD is a 180 deg rotation about x, so
`v,w,q,r,alpha,beta,Fy,Fz,My,Mz` flip while `u,p,Mx` do not; six coefficients were consequently
sign-wrong. **The wing produced lift only at NEGATIVE true angle of attack, and the pitch axis was
statically DIVERGENT** (-61.9 N.m/rad at 16 m/s, time-to-double 0.04 s). That -- not the airframe,
not latency, not the controllers -- is what made "genuine wing-borne cruise" impossible all session.
The 2026-08-11 "FLU pitch-moment sign fix" (`cm_alpha` -1.2 -> +1.2) was a REGRESSION: it corrected
for the My flip but not the alpha flip, which cancels it.

FIXED (`extensions/.../dynamics/vtol_aerodynamics.py`): the model now converts **FLU->FRD on the way
in and back on the way out**, so every coefficient is the STANDARD published value and no term ever
needs a frame sign-patch again. Also: stall now blends into a Newtonian flat plate (the old model
clipped alpha for the COEFFICIENTS but used the raw alpha for the lift DIRECTION -> 15 N of
downforce while backing up, 10 N backward force in a vertical descent); and the aero force is applied
at `VTOLConfig.aero_center` (default = the CG) instead of the body prim ORIGIN, which was an
invisible 0.176 m lever arm = an implicit aerodynamic centre at 71% MAC contributing 3x more pitch
moment than the entire Cm model, and silently re-tuned every time `body_com_offset` changed.
Also fixed: HIL watchdog in `vtol_mavlink_backend` (0.25 s of silence -> safe command, instead of
holding the last one forever) and its mutable default args.

**The corrected plant is a normal aeroplane:** pitch stiffness +16.5 N.m/rad (25% static margin),
short period 1.3 Hz at zeta=0.33, level trim +5.6 deg nose-UP at 16 m/s, stall ~11.5 m/s, wing-borne
14-18 m/s at +4..+8 deg AoA. Regression tests: `tests/test_vtol_aerodynamics.py` (run with the
isaac_env python; 26 physical-property assertions, no Isaac needed).

**CONSEQUENCE — every claim below about AoA sign, pitch direction, wing downforce, "nose-down for
lift", climb-away and the corridor was measured on the INVERTED plant.** `[AERO] aoa=` now prints
the STANDARD angle (positive = nose-up), so old bags/logs have the opposite sign.

**THE CONTROLLERS HAVE NOT BEEN CHANGED and now model the WRONG plant** (deliberate: half-fixing
them would be worse). All 8 model-based nodes use `alpha=atan2(vb[2],|u|)` with CLA=+4.8/CMA=+1.2,
AND their reference pitch is nose-DOWN-positive (`_R_pitch`/`R_ref` build `Ry(+theta)` while the trim
schedules produce standard nose-UP theta) -- the two inversions cancelled against the old sim.
Verified: the node's `theta=+8 deg` reference is 8 deg NOSE-DOWN; its own aero says +37.9 N, the
corrected sim says -22.9 N. Four more shared model errors found in the controller audit:
  * inertia = the BODY LINK's `(0.20,0.28,0.35)` (and even that is the VTOLConfig DEFAULT, not the
    CAD airframe's 0.20/0.23/0.36); the ASSEMBLED vehicle is `(0.265,0.303,0.490)` -> Ixx -24%,
    Izz -29%, i.e. the two axes that stayed fragile. Same class of bug as the CG error.
  * `CQ=0.06` vs the sim's `rotor_moment_ratio=0.12` -> HALF the real yaw torque (and yaw is the
    only hover actuator for it). With the Izz error the true yaw response is ~2.8x the modelled one.
  * `RPOS[:,2]=0.02` for all four rotors; the real front HUB is z=-0.033 rel CG (WRONG SIGN). Zero
    effect in hover, dominant at 90 deg tilt where `My = z_arm*Fx` IS the pitch moment.
  * **the offline selftests integrate the controller's OWN `build_ode` as "the true plant"**, so
    none of the above can ever show up offline -- exactly why "NONE reproduce in the offline test".
Ordered repair plan (cache key -> plant double -> constants -> conventions -> re-tune -> fly) and
the full finding list are in the audit; also C7 (the SMC re-seed fires on TRACKING LAG, which is the
sawtooth that `SMC_LEAD=500` papers over), C8 (`vtol_nmpc` lost the z-ramp), C9 (`latency_s` 0.10 and
the loop-exception/safe-hover guards were never back-ported out of `vtol_nmpc`), C10 (three different
tilt-servo models in one loop), C11 (`dt_ref=1/30` hardcoded in the feed-forward).

# VTOL (dual independent tiltrotor) — bring-up assets

Config-driven articulated VTOL for Isaac Sim / Pegasus, driven by PX4 SITL over
`direct_actuator` offboard HIL. Full operational guide (staged validation, start
order, verification, troubleshooting): **`docs/vtol_bringup_runbook.md`**. Deep
DDS/offboard rationale: `docs/direct_actuator_offboard.md`.

End-to-end data flow:
```
ROS2 direct_actuator node ─DDS→ PX4 SITL (airframe 4100) ─HIL(4560)→ Isaac VTOL
Isaac VTOL ─(opt. ROS2Backend)→ /uav_0/state/* ─(mocap emulator)→ /uav_0/mocap ─→ [controller: TODO]
```

## Isaac side
- `extensions/pegasus.simulator/.../logic/vehicles/vtol.py` — generic `VTOL` +
  `VTOLConfig` + `RotorConfig`/`SurfaceConfig`; airframe = data (lists of rotors/
  surfaces). `dual_tiltrotor_config()` is the target airframe. `apply_force` uses
  the link-**local** frame, so thrust on a tilted prop vectors automatically.
  `_validate_channels()` warns once if a command's channels don't match the airframe.
- **Rotor + servo dynamics** (opt-in `VTOLConfig.rotor_dynamics=True`, default off):
  each rotor has a speed state omega that chases `throttle*rotor_omega_max` with a
  first-order lag (`rotor_tau_spin`); thrust = `max_thrust*(omega/omega_max)^2` (lags
  throttle, ~RPM^2); yaw reaction torque = `rotor_moment_ratio*thrust*(-spin_dir)`;
  tilt servos become a first-order lag (`tilt_tau`) capped at the slew rate. Off =
  legacy instantaneous `throttle*max_thrust` + visual-only spin. Demo:
  `application/vtol/06_vtol_rotor_physics.py` (step inputs, logs omega/thrust/tilt).
- `.../logic/dynamics/vtol_aerodynamics.py` — lumped lift/drag/moment model.
- `.../logic/backends/vtol_mavlink_backend.py` — `PX4VTOLMavlinkBackend` maps PX4
  `HIL_ACTUATOR_CONTROLS.controls[]` → VTOL channel dict via a channel_map.
- `.../rotorcraft/vtol_utils.py` — `spawn_vtol` / `spawn_vtol_with_mavlink`.
  The mavlink spawner auto-selects a channel_map matching the config (raises on no
  match) and takes `publish_state_ros2=` (opt-in state publisher, see Mocap).
- Scenes `application/vtol/`: `01_basic_vtol` (joints only), `02_vtol_hover`
  (forces + stand-in controller), `03_vtol_px4_hil` (full HIL; reads `VTOL_ROS2_STATE`).

## PX4 side — airframe `4100_fsc_dual_tiltrotor`
Source of truth: `px4/airframes/` (+ README). Installed into
`$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/`, registered in that dir's
`CMakeLists.txt`; rebuild with `make px4_sitl` (or use the pre-dropped copies in
`build/px4_sitl_default/{etc,rootfs}/etc/...`). Boots via `PX4_SYS_AUTOSTART=4100`.
**Actuator output mapping is designed to match `default_dual_tiltrotor_channel_map()`
AND the ROS2 node's MOTOR/SERVO order** — verify with `PEGASUS_HIL_DEBUG=1`:

| `PWM_MAIN_FUNC` | function | HIL `controls[]` | channel |
|---|---|---|---|
| 1=101,2=102 | Motor1,2 | 0,1 | motor_left, motor_right |
| 3=201,4=202 | Servo1,2 | 2,3 | tilt_left, tilt_right |
| 5=203,6=204,7=205 | Servo3,4,5 | 4,5,6 | aileron, elevator, rudder |

CA_ (control allocation) geometry is best-effort and **bypassed** by direct_actuator;
a 2-rotor tiltrotor is under-actuated for PX4 hover allocation, so arming may need
preflight relaxation. Airframe bakes `UXRCE_DDS_SYNCT=0`, `COM_DISARM_LAND/PRFLT=0`.

## ROS 2 side (`~/ros2_ws`, Humble, px4_msgs 2.0.1)
- `ros2/px4_vtol_direct_actuator/` — streams `ActuatorMotors`+`ActuatorServos`
  (MOTOR_ORDER=[motor_left,motor_right], SERVO_ORDER=[tilt_left,tilt_right,aileron,
  elevator,rudder]) and drives OFFBOARD+ARM. Symlink into `~/ros2_ws/src`, build.
- `isaacsim_optitrack_ros2_emulator` — mocap tool; subscribes `/uav_0/state/*`,
  publishes `/uav_0/mocap` (`fsc_autopilot_ros2_msgs/Mocap` = header+pose+twist).

## Mocap pipeline (feeds the controller)
`VTOL_ROS2_STATE=1` (env-gated, **off by default**) makes `spawn_vtol_with_mavlink`
attach a state-only `ROS2Backend` (namespace `uav_`) → `/uav_0/state/{pose,twist,
twist_inertial}`. Then run the emulator → `/uav_0/mocap`. The controller consuming
`/uav_0/mocap` → direct_actuator setpoints is **not written yet (TODO)**.

## Scripts (`scripts/`)
- `start_vtol_hil.sh <cfg>` — one command, 4-pane tmux HIL: DDS agent + PX4 + Isaac
  + ROS2 node. Env overrides: `AUTO_ARM`, `MOTOR_*`, `TILT_*`, `VTOL_ROS2_STATE`,
  `PEGASUS_HIL_DEBUG`, `ROS_DISTRO`, `ROS2_WS`, `PX4_SIM_MODEL`. Opens a **new
  terminal** by default (`--here` to run in the current one).
- `stop_vtol_hil.sh` — teardown (VTOL-anchored targets; `--dry-run`/`--keep-agent`/
  `--tmux-only`).
- `start_vtol_mocap.sh` — runs the mocap emulator (needs `VTOL_ROS2_STATE=1` in Isaac).
- `start_vtol_sitl.sh` — PX4+Isaac only (subset of the HIL launcher).

## Importing a CAD airframe (mesh -> USD -> config-driven links)
`vtol.py` links can use imported CAD meshes instead of procedural boxes. Each
`RotorConfig`/`SurfaceConfig` has `mesh` + `mesh_scale`/`mesh_offset`/`mesh_rpy_deg`,
and `VTOLConfig` has `body_mesh*`. A rotor's `mesh` attaches to its **prop link**
(tilts via the now-invisible nacelle for tiltable rotors, spins, and is the thrust
point); `None` -> procedural box (mix freely for incremental bring-up). Mesh links
are visual-only (no PhysX collision — a dynamic triangle mesh is invalid; add
convex collision in-GUI if you need ground contact). **Each articulated link needs
its own separate mesh** (a link can't share geometry with the body).

Workflow: (1) export each CAD component (body + one file per rotor) as a mesh
(STL/OBJ/FBX/glTF); (2) `application/vtol/convert_meshes_to_usd.py --input <dir|file>
--output <dir|file>` (run with Isaac python, headless) -> USD; (3) use
`quad_tiltrotor_config(body_mesh=, front_left_mesh=, ..., mesh_scale=)` (2 front tilt
+ 2 rear static — matches a "2 tilting + 2 static rotor" airframe) and view/verify
with `application/vtol/04_vtol_cad.py`. `mesh_scale=(0.001,)*3` if the CAD is in mm.
Part origins should sit at each joint pivot; otherwise fix in the mesh tool or use
`mesh_offset`/`mesh_rpy_deg`. Rotor hub positions in `quad_tiltrotor_config()` are
placeholders — measure them off the CAD.

## FSC CAD airframe (wired into HIL)
The real airframe is the CAD tilt-quadplane (2 front tilting + 2 rear static rotors),
built from the imported STL. `fsc_cad_vtol_config()` (fsc_aerial_manipulation.rotorcraft)
= `quad_tiltrotor_config()` wired to the converted part USDs (`$VTOL_CAD_USD_DIR`,
default `~/vtol_cad_usd/parts2_usd`) with tuned hinge (front `position≈(0.459,±0.284,
-0.067)`, `spin_offset≈(0,0,0.09)`) + `mesh_scale=0.001`, `mesh_rpy_deg=(90,0,90)`.
- **STL→parts**: `application/vtol/split_cad_stl.py` (needs trimesh; connected-component
  split + region/servo/plate rules + `HINGE_OFFSET_MM`) → OBJ → `convert_meshes_to_usd.py` → USD.
- **Nacelle vs prop**: `RotorConfig.nacelle_mesh` (arm+motor, tilts, no spin) on the
  nacelle link; `mesh` (prop) on the prop link; `spin_offset` puts the prop link on
  its shaft so it spins in place.
- **Surfaces**: 2 ailerons (opposite `deflect_scale` ±1 -> roll), 1 elevator, 1 rudder
  — procedural boxes in `quad_tiltrotor_config()`, `SurfaceConfig.size`/`deflect_scale`.
- **HIL**: `03_vtol_px4_hil.py` spawns `fsc_cad_vtol_config()`; channel_map auto-selected
  to `default_quad_tiltrotor_channel_map()` (controls[0..3]=motors FL/FR/RL/RR,
  [4..5]=tilt L/R, [6..8]=aileron/elevator/rudder). PX4 airframe `4101_fsc_quad_tiltrotor`
  (px4/airframes/, registered, rebuild PX4). `start_vtol_hil.sh` defaults `PX4_SIM_MODEL=fsc_quad_tiltrotor`; node
  pane params `motor_front_L/R`, `motor_rear_L/R`, `tilt_left/right` (env `MOTOR_FRONT`,
  `MOTOR_REAR`, `TILT_*`). ROS2 node MOTOR/SERVO order updated to match.
- **Viewer**: `06_vtol_rotor_physics.py` (forces off) shows the airframe + a red pivot
  marker at each tilt joint + surfaces wiggling.
- **Physics model**: `07_vtol_cad_physics.py` — forces ON (thrust∝RPM², yaw torque,
  aero, gravity) held up by the shared `hover_controller.py` `HoverControllerBackend`
  (allocation from rotor positions; inverts the RPM² thrust when `rotor_dynamics`).
  Same controller now used by `02_vtol_hover.py` (deduped). Mesh bodies get an
  invisible box collider (`VTOLConfig.body_collision_size`) so they don't fall through.
- **External ROS2 control loop** (no PX4): `08_vtol_hover_control.py` runs the CAD
  VTOL with `VTOLCommandROS2Backend` (subscribes `/uav_0/vtol/actuator_cmd`) +
  state-publishing `ROS2Backend`. The controller is a ROS2 package
  `~/ros2_ws/src/fsc_vtol_controller` (`vtol_hover_controller` node): `/uav_0/mocap`
  -> hover/attitude PID -> `/uav_0/vtol/actuator_cmd` (9-ch Float64MultiArray, tilt
  held fixed, all gains/mass/max_thrust live `ros2 param`s). Launcher
  `scripts/start_vtol_hover_control.sh <cfg>`: tmux Isaac + mocap emulator +
  controller + monitor (Isaac bundled rclpy ↔ system ROS2 over FastDDS, same domain).

## Control-loop options + controllers (position/trajectory tracking)
Three ways to fly the CAD quadplane; all consume the SAME reference topic
`/uav_0/fsc_autopilot_ros2/position_controller/reference` (pos+vel+yaw), so the
ground-station Trajectory tab drives any of them.

- **KEY FINDING (CORRECTED) — direct_actuator DOES track trajectories in HIL; the earlier
  tumble/fly-away was a CONTROL-LAW bug, NOT a latency wall.** Two root causes, both fixed:
  (1) a CG bug (`body_com_offset` 0.174→0.195, see below) caused the −38° liftoff pitch;
  (2) the outer position loops IGNORED the trajectory's velocity/accel FEED-FORWARD, so
  tracking relied entirely on delayed feedback → oscillation/fly-away on the laggy HIL plant
  (this is what got mislabeled a "latency wall"). The fix is **`traj_track_node.py`** — an
  SE(3) geometric attitude law + velocity/accel FEED-FORWARD on the reference. VALIDATED
  live (2026-08): re-arms from the ground, hovers dead-on (errx=0.00), and tracks a 40 m
  out-and-back at **mean|err_x|=0.86 m (max 1.92 m), altitude held 0.18 m mean, stable, no
  fly-away** (log: `/tmp/track_out.txt`). So BOTH paths are viable: the PX4 inner loop is
  tightest for fast *attitude*, but direct_actuator + feed-forward tracks *trajectories* fine.
  - **`traj_track_node.py`** (`ros2/px4_vtol_direct_actuator/...`): `method=se3`,
    `controller_id=geometric`, `auto_arm`, 200 Hz. Position outer loop with vel+accel FF →
    `F_des`; lean-capped; SE(3) attitude (`kR/kW`); allocation `Ainv@[f,Mx,My,Mz]` to 4
    rotors from CG-relative geometry. Robustness: DEFAULTS to a hover ref at the current
    x,y (z=1.5) until a trajectory arrives (publishing NaN actuators with no ref stalls the
    PX4 arm handshake — this was the "flaky arm"); FF clamped against reference jumps.
  - **PID+FF (band-limited integral) — VALIDATED, the best variant.** A naive position
    integral WINDS UP on this laggy plant and diverges (ran away, flipped the vehicle
    inverted). Band-limiting fixes it: `i_band`=1 m (integrate only near the set-point,
    FREEZE during fast tracking so no windup) + `i_max` anti-windup clamp + reset on ref
    jump + gated on armed. Result (same 40 m out-and-back): **mean|errx|=0.32 m (vs 0.86 m
    for PD+FF — 63% better), max 1.40 m, altitude 0.19 m mean; hover dead-on (errx=0.00),
    settles to ±0.02 m at each hold.** `ki_xy=0.4, ki_z=0.8` (log `/tmp/pid_final.txt`).
  - **Tilt-for-forward (rotors tilt during cruise) — VALIDATED.** During a forward traverse
    the FRONT rotors tilt forward instead of the airframe pitching over. It is COORDINATED:
    the tilt is sized to the forward-force demand (`tilt=atan2(F_fwd, ½·F_up)`, capped by
    `tilt_max_deg`=35°), that force is then REMOVED from the SE(3) pitch target (no pitch/tilt
    double-counting), and the front pair's collective is boosted by 1/cos(tilt) so their
    VERTICAL thrust (altitude + pitch balance) is preserved. Gated on forward speed
    (`tilt_v_gate`=0.4 m/s) so hover/hold stays a clean multicopter `tilt=0`. Live: tilt ~18°
    at 1 m/s cruise (scales to the 35° cap for faster), hover `tilt=0`, tracking
    mean|errx|=**0.31 m** (same as no-tilt) + altitude 0.15 m. NOTE this is *tiltrotor forward
    flight* (rotors vector thrust), NOT a wing-borne fixed-wing cruise transition (that still
    needs the tilt+aero NMPC). Set `tilt_max_deg=0` to disable (pure multicopter).
  - **Precise-landing mission — VALIDATED.** Flew takeoff→diagonal traverse (0,0)→(12,6)→
    descend→land: **landed at (11.98, 6.00), 0.02 m from the commanded target** (log
    `/tmp/mission_out.txt`). Point-to-point tracking stayed within 0.15 m; the descent held
    the target line to ±0.02 m. This is the "land at a precise point" goal.
  - **Launcher `scripts/start_vtol_traj_hil.sh <cfg>`** — DDS+PX4+Isaac+mocap+estimator +
    traj_track (ACTIVE, `controller_id=geometric`) + GS. It is GS-drop-in: the Trajectory tab
    reference + the "Safe Geometric" switch button drive it (Play/HOVER/reference-dropdown).
  - **Test without rebooting:** the flaky-arm/tumble grinding traced largely to rebooting the
    whole HIL stack every run. Keep DDS+PX4+Isaac+mocap UP; only relaunch the controller
    against the live stack (arm was reliable every time that way). **To reset a crashed/inverted
    vehicle you MUST restart PX4 (fresh EKF) BEFORE Isaac** — restarting Isaac ALONE leaves
    PX4's EKF diverged ("Preflight Fail: Attitude failure (roll)" → cannot arm). Procedure:
    `pkill -9 -x px4` (kills only the binary, not the tmux bash wrapper → the pane survives),
    then `tmux send-keys -t <sess>:1-px4 "<px4 launch>" Enter`; then relaunch Isaac
    (`03_vtol_px4_hil.py`, it connects to PX4:4560); the mocap emulator reconnects on its own.

- **fsc autopilot (`autopilot_sv_baseline_node`)** — the rock-solid hover (sub-mm at
  (0,0,1.5)). PX4 inner-loop, publishes `VehicleAttitudeSetpoint`+`OffboardControlMode
  (attitude=true, thrust_and_torque=true)`. Launcher `scripts/start_vtol_fsc_hil.sh`.
  Arm via the helper window: `fsc_takeoff 1.5` then `fsc_stopref` to hand the reference
  to the tab. It ALSO tracks a moving reference, so it can fly trajectories too.

- **attitude-setpoint NMPC** (`.../px4_vtol_direct_actuator/.../nmpc_attitude_node.py`) —
  outer-loop CasADi NMPC: state=[pos,vel] (ENU), control = thrust-accel vector (tilt≤30°
  + thrust-mag constrained); converts the optimal thrust vector → desired attitude quat
  (NED, via fsc's `getNEDqFromENUq` = q(Rie)·q_enu·q(Rbv), Rie=[[0,1,0],[1,0,0],[0,0,-1]],
  Rbv=180°-about-x) + normalized collective thrust; publishes the SAME PX4 attitude
  interface. Stable in HIL (hovers ~0.25 m). Launcher `scripts/start_vtol_nmpc_hil.sh`
  (`NMPC_MODE=attitude` default; `direct` = raw actuators, sim-only). Gentle-hover gains:
  w_pos5/w_vel20/max_tilt18 + governor sp_max_vel 0.6. `hover_throttle=0.70` (HIL-measured;
  was 0.46 -> under-thrust), `a_min=7` (0.7g floor; 3.0 let it near-free-fall -> vertical
  limit cycle). Hovers cleanly at 1.5 (x,y ±7 cm) when the GS drives the reference.

- **TWO-WAY inner-loop switch — `scripts/start_vtol_inner_hil.sh` (the "ship" path).** Runs
  BOTH inner-loop controllers (fsc = safe hover, attitude-NMPC = tracking) behind the GS
  switch, with `attitude_setpoint_mux.py` (NEW): each controller's VehicleAttitudeSetpoint is
  launch-time REMAPPED to /<ns>/mux/{fsc,nmpc}/... and the mux forwards ONLY the active one to
  /<ns>/fmu/in/vehicle_attitude_setpoint (both keep sending OffboardControlMode+arm to the real
  topics -> idempotent -> NO controller code change). GS "Safe Geometric"/"NMPC" buttons publish
  "geometric"/"nmpc" (mux maps "geometric"->fsc), so it drops in with no GS change. Mux switch
  VERIFIED (toggles nmpc<->fsc; NMPC hovers at 1.5). **fsc side needs engagement**: run
  `fsc_takeoff 1.5` in the helper window (win 9) once -- the fsc has its own arm/takeoff state
  machine, so a reference alone doesn't make it emit setpoints (else switch->fsc = `forwarded=0`
  -> drift). After that the switch is seamless.

- **SMC outer loop + A/B inner switch (`scripts/start_vtol_smc_hil.sh`).** An outer-loop
  sliding-mode controller (`smc_outer.py`: `v=-λ·e−K·sat(e/Φ)` advancing a CONVERGING stationary
  set-point; LOOSE cruise `vmax=20 m/s` vs STRICT landing, by target z) drives a GS-switchable
  inner loop ("A: Geometric+SMC" / "B: Attitude-NMPC+SMC" -> `active_controller`=geometric|attnmpc):
  - **A = `smc_outer_node.py` → `traj_track_node.py`** (reference remapped to `.../reference_smc`).
    RELIABLE, DEFAULT. Reads state from MOCAP + streams direct_actuator → NEVER uses the px4 EKF →
    immune to HIL EKF flakiness. Flew takeoff→fly-to-spot→hover@spot 0.04 m→land 0.00 m; live it
    holds a steady hover 20 m out.
  - **B = `nmpc_attitude_node.py`** (SMC governor + attitude setpoints, px4 inner loop). Tracks
    against the px4 EKF. Two bugs found: `hover_throttle` 0.70→**0.50** (0.70 over-scaled → could
    never descend → climb-away); and **EKF HEIGHT divergence** (live: px4 `z_ned=+4.86` vs true
    −1.41, ~6 m too low → B over-thrusts → fly-away). NOTE: the EKF *yaw* is FINE (sim mag; live
    mocap-vs-EKF yaw DIFF=0.0° — the earlier "yaw off" diagnosis was wrong). Diag: `/tmp/diag_ekf.py`.
  - **EV bridge `mocap_to_px4_ev.py` (makes B viable).** mocap (ENU/FLU) → `VehicleOdometry`
    (NED/FRD) on `/<ns>/fmu/in/vehicle_visual_odometry`; stamps in PX4 time via a ROS↔PX4 clock
    offset learned from `/fmu/out/vehicle_attitude` (works with `UXRCE_DDS_SYNCT` 0 or 1). Verified:
    emits `z_ned=−1.41` (correct) vs the EKF's diverged +4.86. Engage: `EKF2_EV_CTRL=11`
    (hpos+vpos+yaw) `EKF2_HGT_REF=3` `EKF2_GPS_CTRL=0` `EKF2_EV_DELAY=50` then `ekf2 stop; ekf2 start`
    — EKF2_* need an ekf2 RESTART, so do it PRE-ARM at boot (launcher 8-relax window), NOT mid-flight
    (offboard failsafes on the estimator gap). Launcher wires it in window 4b-ev + the relax window;
    env knobs `EV_BRIDGE=1`, `DISABLE_GEOFENCE=1` (GF_*=0), `SMC_VMAX=20`.
  - **VALIDATED (2026-08-08):** with EV the EKF `z_ned` went +4.86 (6 m wrong) → −1.50 (exact), and
    **B then HELD HOVER 22 s (maxZ 1.94, maxR 0.28 m)** — vs the 160 m climb-away before. Residual: a
    mild ~+0.3 m altitude bias on B (tune `hover_throttle`/`w_pos`,`w_vel`). A↔B switch is clean; A
    returns to a dead-on (0,0,1.5) hover after B. `/tmp/diag_ekf.py` = EKF-vs-mocap diff tool.
  - **CRUISE / TILT-TRANSITION LIMIT (2026-08-08) — A cannot do wing-borne cruise.** The GS "Cruise
    speed" spinbox was capped at 10 m/s (default 1) → the vehicle never reached wing-borne speed →
    the tilt (force-sized `atan2(F_fwd,half_up)`) stayed ~0 → "tilt never changes". Cap raised to 20
    (`trajectory_player.py`), default kept LOW (2). The AERO MODEL IS REAL (full linear fixed-wing,
    `vtol_aerodynamics.py`, applied every step): a 3.4 kg airframe needs **8–14 m/s** for the wing to
    carry its weight (lift∝V²). BUT the geometric controller (A) does NOT model wing lift, so at
    cruise speed it is UNSTABLE: an airspeed-scheduled tilt→70° made the WING OVERPOWER it →
    **climb-away to 110 m**; reverting to force-based tilt (cap 45°) at a 6 m/s traverse made it
    **descend and crash** (lean loses vertical thrust). So the PLAIN geometric controller (A) is a
    hover / ≲4–5 m/s-traverse controller only.
  - **AERO-AWARE TILT TRANSITION — SOLVED (2026-08-09), `aero_transition_node.py`.** The fix is aero
    FEED-FORWARD: subtract the MODELED wing lift/drag (same coeffs as `vtol_aerodynamics.py`) from
    F_des so the rotors back off as the wing takes over → holds altitude through the transition.
    Offline-validated (`/tmp/nmpc_transition_offline.py` = the receding-horizon CasADi NMPC;
    `/tmp/aeroff_offline.py` = the real-time aero-FF feedback law: aero-FF OFF 3.76 m alt loss vs ON
    0.42 m). The online node is the aero-FF law (robust at 200 Hz; it mirrors the NMPC's steady-state).
    Tilt is COORDINATED: airspeed-gated cap (tilt far only once the wing carries weight), servo slew
    limit, and a YAW-ALIGNMENT gate (yaw to face travel first — the body-frame tilt else pushes the
    wrong WORLD way). Mocap-based (no EKF). Launcher window `6b-transition` (controller_id=transition,
    SMC-fed); activate with active_controller=transition. **HIL VALIDATED — FULL MISSION (fly +y at
    vmax=16): takeoff → transition (tilt 12°→52°, accel to 16.4 m/s) → cruise → hover@spot (0,79,3) →
    clean vertical landing at (0.00,78.9,0.15). Altitude held ±0.15 m through the whole transition,
    dead-straight.** Higher vmax → higher tilt (14→~29°, 16→52°, offline 20→80°).
  - **OUT-AND-BACK (return trips) — VALIDATED.** Two additions handle a 180° turn-around: HOLD-AND-YAW
    (when the nose isn't aligned with travel, hover + yaw first instead of chasing) + YAW-CMD SLEW
    (`yaw_rate_deg_s`) so the attitude heading turns gradually — avoiding the SE(3) 180° singularity
    that had dropped it. **HIL: takeoff → transition out to (0,70,3) → 180° yaw-around (altitude held
    3.02–3.07 m) → transition back → land near origin, altitude held ~3.0 m throughout.** Guidance must
    point the reference yaw at the target (see `/tmp/mission_outback.py`). GS button "C: Aero Transition"
    added. Each HIL crash inverts the vehicle → full stack restart to recover.

- **direct_actuator controllers** (`nmpc_quad_node.py` = CasADi NMPC → actuators;
  `tiltrotor_control_node.py` = geometric PID → actuators, tilt-for-forward). These two were
  the ones that "confirmed the latency wall" — but that was the missing-feed-forward bug (see
  KEY FINDING); `traj_track_node.py` supersedes them for tracking. Both take `controller_id`
  + `active_at_start` and
  gate PX4 output on a std_msgs/String `/<ns>/ground_station/active_controller` — only the
  ACTIVE one publishes (the other is standby but keeps computing). This is the **two-way
  controller SWITCH**: `scripts/start_vtol_dual_hil.sh` runs both (geometric active, nmpc
  standby) + estimator + GS; the Trajectory tab has Safe-Geometric/NMPC buttons
  (HOVER→geometric, Play→nmpc, geofence-bail→geometric). Switch infra works. (These two
  legacy direct nodes lacked feed-forward → fly-away; use `traj_track_node.py` — see KEY FINDING.)
  - **ROOT CAUSE FOUND + FIXED — it was a CG bug, not (mostly) latency.** The ~−38° liftoff
    pitch disturbance was `body_com_offset` set to 0.174 (rotor centroid, IGNORING the
    articulated-link masses). Front rotors carry 2 links each (tiltable nacelle + prop) vs 1
    (prop) for the rear, plus 4 aft surfaces — all pull the TOTAL CG ~2 cm AFT of the rotor
    centroid where thrust acts → symmetric thrust = ~0.6 N·m nose-up = the disturbance. FIX:
    `fsc_cad_vtol_config` body_com_offset x → **0.195** (total CG on the rotor centroid; 0.195
    was the correct value wrongly reverted to 0.174). With it, the direct geometric controller
    **hovers at z=1.5 (pitch ±7°) for ~13 s** — vs the immediate tumble before. Open-loop probe
    (`/tmp/probe2.py`) had confirmed all axis mappings are correct (not a sign bug); yaw
    divergence was ground-contact (fixed by decisive liftoff + `kp_yaw`). The ~13 s residual
    fly-away was NOT a latency limit — it was the position loop lacking velocity/accel
    feed-forward (adding it in `traj_track_node.py` tracks a 40 m round trip; see KEY FINDING).
    Geometric node also has structural fixes (velocity-profiled altitude w/ sqrt stopping profile,
    takeoff punch, altitude-gated position hold, attitude-priority mixer desaturation, pitch_ff off).
  - **fsc_autopilot_ros2-packaged variant** (only-adds-files): the NMPC direct_actuator
    node is also placed in the fsc package as `fsc_autopilot_ros2/fsc_autopilot_lib/
    single_vehicle_baseline/modules/position_control/python/nmpc_vtol_control.py` (copy of
    gated `nmpc_quad_node.py`, controller_id=nmpc; run via `python3`, no CMake change).
    Launcher `scripts/start_vtol_nmpc_fsc_hil.sh` = a version of `start_vtol_fsc_hil.sh`
    that runs the geometric direct_actuator as the ACTIVE hover-safety + this NMPC node
    STANDBY (instead of the fsc attitude autopilot) + estimator + GS + helper. GS
    Safe-Geometric/NMPC buttons activate the NMPC to track the reference. Same
    direct_actuator stability caveat as above.

- **★ IN-LOOP TRANSITION NMPC — WORKS, the cruise problem is SOLVED (2026-08-09).**
  `ros2/px4_vtol_direct_actuator/.../nmpc_cruise_node.py` (controller_id=`nmpc_cruise`, launcher
  window `6c-nmpc-cruise`, GS button "D: NMPC Cruise"). Longitudinal NMPC (CasADi/IPOPT, state
  `[s,z,vs,vz]` along the TRUE body heading, control `[Tf,Tr,beta]`, wing aero in the model,
  N=15/DT=0.08, ~8 ms/solve) co-optimizes tilt+thrust, PAIRED with a gentle rate/saturation-limited
  attitude+lateral+pitch loop. **FULL MISSION HIL-VALIDATED: hover → climb → cruise to a 40 m target
  (sustained ~5 m/s via GENUINE TILT 18–21°, altitude held ±0.2 m) → precise arrival at (40.0,0,4.0),
  no overshoot → precise LANDING at (40.1,0,0.3) dead-on.**
  - **★★ CORRECTION (2026-08-10, sim-force-verified): the "wing-borne" claim below was OVERSTATED.**
    Re-ran with the sim's own `[AERO]`/`[PROP]` force prints. What nmpc_cruise actually does at "cruise":
    the WING does carry ~49% (sim `[AERO] Fbody_z=+16.9 N` lift at AoA +5°, V=11.5) and rotors unload to
    ~16.8 N — BUT the **rotors stay VERTICAL (tilt≈0)**, so it is a wing-ASSISTED multicopter drift, NOT
    airplane-mode flight (no forward-thrust-by-tilt), and it is NOT robust: on a long cruise it degrades
    to high AoA and **stalls/crashes** (watched it hit the ground at ~1200 m). So: partial (~50%) wing
    OFFLOAD, stable only ~400-500 m, not airplane-mode. **REAL airplane-mode (rotors tilt forward, wing
    carries ~all the weight, +AoA) is NOT achieved** — `aero_transition_node` reactive mode gives a stable
    tilted cruise but NOSE-DOWN with wing DOWNFORCE (tilting props forward makes a nose-down moment → −AoA);
    its snap-to-80° cruise mode CRASHES at the onset. See the airplane-mode discussion. Original (overstated)
    notes kept below for the mechanics.
  - **★★ WING-BORNE UPGRADE — 5-STATE PITCH NMPC (2026-08-09). GENUINE POSITIVE-AoA WING-BORNE CRUISE
    ACHIEVED + rosbag-verified.** The state is now `[s,z,vs,vz,theta]` (theta = BODY PITCH, nose-up +)
    and control `[Tf,Tr,beta,q]` (q=pitch-rate): the solver CO-OPTIMIZES pitch WITH tilt+thrust (`_f`
    rotates the rotor thrust by theta; aero AoA = theta − flight-path). **HIL result (bag wb10/wb11):
    hover@8 m → accelerate on TILT (16→23°, body level) → at ~11 m/s TRANSITION (tilt→0, pitch nose-UP
    +5°) → sustained WING-BORNE cruise: AoA=+5°, altitude 8.0 m DEAD-FLAT, rotors unloaded to the floor
    (16 N — the WING carries ~53% of weight), 11.5 m/s, ~45 s / 490 m steady, recovers from
    disturbances.** The path to it (all HIL-diagnosed via rosbag, `/tmp/anabag.py`):
    (1) **monotonic SMC** — `SMC_LEAD=500` (reset_dist huge → never re-seeds) killed the 4 s setpoint
    SAWTOOTH that had capped speed ~5 m/s and driven a pitch limit-cycle; smooth velocity FF → smooth
    tilt → speed builds. (2) **speed-gate via theta-weight** (`wp[2]=1.0`) + **slow pitch trim**
    (`Q_MAX=7°/s`, tight bounds `TH_UP=9/TH_DN=3`, `theta_lp` low-pass) — pitch is a gentle trim, not a
    fast altitude actuator (that limit-cycled through the HIL latency). (3) **rotor FLOOR** (`rotor_min`=8 N
    NMPC bound) — rotors never fully unload so they RETAIN differential-thrust pitch/roll authority (full
    unload → pitch ran away to stall). (4) **speed-scheduled, altitude-regulated nose-up TRIM**
    (`cruise_trim_deg`=3, `k_trim_alt`; `k_trim_vz`=0 — the climb-rate feedback set up a limit cycle) →
    the body targets +AoA for wing lift but backs off if it climbs. (5) **DAMPED aero surfaces** — route
    the FULL damped moment `M=−kR·eR−kW·eW` to the surfaces: `elevator=k_elev·M[1]`, `aileron=k_ail·M[0]`,
    `rudder=k_rud·M[2]` (k_elev/ail=−0.15, k_rud=−0.10, airspeed-gated). Position-only (`eR`) surfaces ran
    the pitch AWAY (no damping); the moment-matched DAMPED surfaces give authority WITHOUT runaway, and the
    ailerons fix a slow ROLL divergence that tumbled the elevator-only version at ~50 s. Nose-DOWN is
    tightly capped (`TH_DN=3`) so forward thrust comes from the TILT (tiltrotor), keeping the body nose-up.
    - **Ops:** SMC-fed (`start_vtol_smc_hil.sh`, env `SMC_VMAX=11 SMC_LEAD=500`; 6c passes vmax_fwd/a_fwd/
      pitch_rate + reads reference_smc). Drive by publishing the RAW reference (pubraw → smc_outer → nmpc).
      Cruise altitude must be > `cruise_alt_min`=4. Take off on `geometric` then switch
      `active_controller`=nmpc_cruise (the NMPC does NOT take off cleanly from the ground). NEVER
      mid-flight node restarts — one test = one fresh stack (`bash /tmp/teardown.sh` + relaunch), or a
      corrupted state carries into the next run.
    - **WING-BORNE (`cruise_trim_deg`≈3)**: tilt→0, body nose-up +5°, wing carries ~53%, rotors
      unloaded. Lightly-damped roll limit-cycle (±18°, recovers) can appear on >~500 m legs.
    - **DECELERATION / fly-to-point+land.** Root weakness: a tiltrotor can't reverse-tilt (beta≥0) and
      at tilt=0 the wing-borne pitch trim holds the body LEVEL, so it has little forward-braking authority
      and coasts far past a finite target (commanded 120 m → landed 200-270 m before the fix). Two fixes:
      (1) **`vff` fix** — use the reference velocity directly, don't hold the last non-zero value (holding
      it kept the vehicle cruising when a mission's FF decayed to 0, ~doubling the overshoot). (2)
      **DECELERATION BRAKE** (`k_brake`,`brake_max_deg`) — in HOLD, pitch nose-up ∝ forward speed so the
      thrust tilts BACKWARD and brakes. Fires only in hold (`vs>0.3 or e_fwd>0.5`), so a far-target
      wing-borne CRUISE (hold=0 throughout) is untouched. **Result: overshoot 200-300 m → ~20-30 m**
      (fly-to-(120,0)+land landed 94-116 m across runs, dead-on in y). The brake suppresses full
      wing-borne DURING a point-to-point mission (it slows the traverse to ~3 m/s), so a mission is
      tilt-cruise+precise-ish, while a far constant target is full wing-borne. **STILL OPEN:** the landing
      is not yet cm-precise/consistent (brake oscillates ±20 m around the target); combining fast
      wing-borne cruise WITH a precise stop needs a proper distance-scheduled decel (sqrt profile to the
      true target) or a small reverse tilt. The nmpc doesn't see the true target through the SMC (only
      the SMC setpoint), which is part of why anticipating the stop is hard.
    - **`vff` fix:** on_ref uses the reference velocity DIRECTLY (don't hold the last non-zero value) —
      holding it kept the vehicle cruising when a mission's FF decayed to 0, doubling the overshoot.
  - **Key fixes (all found live, this is why "hover-only" became "full mission"):** (1) lateral
    **roll-sign** was inverted → fly-away; `roll=+(kp·e_lat+kv·v_lat)/g`. (2) **pitch drift-arrest**
    (symmetric to roll) — with `beta≥0` and level pitch there was NO backward force to stop forward
    drift; nose-up pitch cancels it → rock-solid hover. (3) **★ attitude AUTHORITY** — soft `kR=5` +
    moment caps 3/3/1.5 starved the loop; hover was fine but ANY maneuver (even a climb) crashed.
    **`kR=9` + caps `8/8/4` N·m = the breakthrough.** (4) reads the **RAW reference** (NOT
    `reference_smc`) — it self-limits, the SMC's converging setpoint caused tilt chatter; +
    hysteresis (`engage/disengage_dist`=4/1.5) on tilt-cruise vs pitch-hold. (5) **cruise stall** (CG
    pitch-trim cancels the tilt's forward force) → keep the pitch loop active in cruise tracking
    forward SPEED (`vs_cmd`), with a **sqrt decel** on approach to avoid arrival overshoot. (6)
    yaw-to-target disabled by default (`yaw_face_dist=10000`; it flips 180° near a target) — command
    targets along the heading. (7) `z_bail` safety = level-climb if it drops near ground.
  - **Ops:** mocap-based (no EV/EKF needed). Drive by publishing the RAW reference
    (`python3 /tmp/pubraw.py X Y Z`) + `active_controller`=`nmpc_cruise`. Params are LIVE
    (`ros2 param set /nmpc_cruise_control ...`, no restart). NEVER mid-flight-handover to geometric
    (stale SMC-ref velocity crashes it) — relaunch to reset (each crash inverts the vehicle).
    Limits/TODO: >~8 m/s cruise + return-trips-with-yaw not yet validated on this node.

- **★ FULL CLOSED-LOOP AUDIT of nmpc_cruise (2026-08-09), sim-truth-verified.** Traced every stage
  `controller → PX4 HIL → Isaac (channel-map → propulsion → aero → PhysX) → mocap → controller` with
  live instrumentation (`PEGASUS_PROP_DEBUG=1` → `[PROP]`/`[AERO]` in vtol.py) cross-checked against the
  controller log, over a full hover→climb→cruise(0→5 m/s)→arrive(40 m) mission.
  - **(A) Actuator→HIL→channel-map: 1:1 CORRECT, verified live.** mavlink backend log:
    `controls[0..3]→motor_FL/FR/RL/RR, [4,5]→tilt_L/R, [6,7,8]→aileron/elevator/rudder`
    (`default_quad_tiltrotor_channel_map`, all scale=1/offset=0), exactly matching the node's
    `pub(mot=[FL,FR,RL,RR], servo=[tservo,tservo,ail,elev,rud])`. Rotor order + spin dirs
    (FL+1,FR−1,RL−1,RR+1) match config `quad_tiltrotor_config`.
  - **(B) Propulsion: ACCURATE.** `rotor_dynamics=False` → thrust = throttle·max_thrust (18 N),
    INSTANTANEOUS (controller assumes this). Live hover: `mot=[0.47,0.48,0.46,0.47]` → `[PROP]
    front 8.6/rear 8.4 N` (0.47·18=8.5 ✓), all four ≈ weight/4, total 33.9 N ≈ 34.3 N weight. Thrust
    dir = body-frame `[T·sinθ,0,T·cosθ]` at the rotor position (matches the node's Frx=Tf·sinβ). Yaw
    reaction `−spin_dir·moment_ratio·T` about the tilted axis (matches the node's yaw alloc).
    **Tilt slews at 80°/s** (rate-limit, rotor_dynamics OFF → no tilt_tau lag) — keeps up with the
    node's gradual tilt: live CTL tilt 17→18→20→0 vs `[PROP]` 16→18→20→0 (≤1° lag on ramps, exact at
    steady). So tilt/thrust the sim applies == what the controller commands.
  - **(C) Aero: coeffs MATCH, but a MAJOR AoA-definition mismatch is the real cruise limiter.** The
    node's NMPC aero coeffs (RHO 1.225, S 0.55, CL0 0.2, CLα 5.5, CLmax 1.5, CD0 0.04, k 0.04, stall
    22°) EXACTLY mirror the sim `VTOLAerodynamicsConfig`. BUT the node's NMPC computes AoA from the
    **flight-path** (`atan2(−vz, √(vs²+…))` ≈ 0 in level cruise) while the sim uses the **body-frame**
    AoA — and the vehicle cruises **nose-down ~20°** (the pitch-for-forward + tilt-moment), so the real
    AoA is −5→−22°. Live `[AERO]` at V≈5: `alpha=−19.5°, Fbody_z=−11.6 N` (wing DOWNFORCE, CL hits the
    −1.5 stall floor). The node's model predicts `alpha≈0 → +1.5 N up`. **≈14 N of unmodeled downforce**
    (grows as V²) the feedback loops silently absorb (rotor thrust rises 8.6→10–14 N, altitude still
    held ±0.2 m). This is the root of the ~6 m/s sustained-cruise ceiling seen all session: the NMPC
    (point-mass, no body-pitch state) and the attitude loop (pitches the body nose-down) are DECOUPLED,
    so neither predicts the resulting wing downforce. Minor extras: sim aero hard-gates `<0.25 m/s`
    while the node ramps `vs²/(vs²+4)` (node under-models aero at 2–5 m/s); sim also has cl_q / elevator
    lift / side-force / aero-moments the node's longitudinal model ignores. Fix for >6 m/s (future):
    put the body pitch into the NMPC AoA (couple attitude↔NMPC), or command a small +AoA / use the
    elevator to flip the wing to lift — but that reopens the climb-away (needs the coupled controller).
  - **(D) State feedback: CORRECT frames.** `/uav_0/mocap` = `frame_id:map` WORLD pose (matches the
    node's p), `twist.linear`=WORLD velocity (≈0 at hover, +x during +x cruise — the node uses it as
    world v for e_lat/vs ✓), `twist.angular`=BODY rates (the node uses it as eW ✓), orientation quat
    level at hover (w≈1). No frame bug.
  - **(E) Closed-loop consistency: all check out.** hover thrust=weight (34 N), throttle→thrust exact,
    tilt→genuine forward cruise (tilt 18–21° @ ~5 m/s), altitude held ±0.2 m through the transition,
    precise arrival (40.0 m, no overshoot). The loop is ACCURATE; the only model gap is the aero AoA
    (C), handled by feedback within the validated ~5 m/s envelope.
  - **(F) Roll/lateral path verified + a YAW-HOLD limitation found.** A +5 m lateral (y) move: CTL
    `roll=−14°` → `[PROP] front_L 9.6 > front_R 8.1` (left rotors higher → rolls to translate +y),
    reached (40,5.1,4) with altitude held 4.0 — roll allocation is applied correctly in the sim, mocap
    `twist.linear.y` tracks the world +y motion. **BUT the yaw drifted 0°→61° and STAYED** (only
    damped, never restored): with yaw-to-target off (`yaw_face_dist`=1e4) the node sets `psi_tgt=cur_yaw`
    each loop, so `yaw_cmd` tracks the CURRENT yaw → the yaw error is ~0 by construction → M[2] is
    rate-damping only, no position hold → the roll/reaction coupling of a lateral maneuver yaws the
    body and it isn't corrected. Straight-line forward flight is unaffected (no yaw disturbance), which
    is why the validated mission held yaw≈0. **Recommended fix (small, low-risk): latch `yaw_cmd` to the
    heading captured at engage (a fixed reference) instead of tracking `cur_yaw`, so yaw disturbances
    are corrected** — forward cruise (yaw≈0) is unchanged, lateral/combined moves get a real yaw hold.
  - **Net:** the closed loop is faithful end-to-end (commands → sim forces → state → feedback all
    match) — no plumbing/frame/sign bugs. The two real model gaps are (C) the NMPC AoA (flight-path vs
    body → unmodeled wing downforce, the >6 m/s limiter) and (F) the missing yaw position-hold (lateral
    maneuvers only). Both have concrete fixes above; neither affects the validated hover/climb/
    forward-cruise/precise-arrival/landing envelope.
  - **★ BOTH GAPS FIXED + HIL-VALIDATED (2026-08-09).**
    - **(C) fix — aero AoA coupling.** The NMPC now takes the measured body pitch as a parameter
      (`theta`) and uses `alpha = theta − flight_path` in its aero, instead of assuming a level body.
      `theta` is derived from the sim's EXACT AoA: `v_body = Rᵀ·v_world`, `alpha_meas = atan2(w,u)`
      (identical to `vtol_aerodynamics.update`), `theta = alpha_meas + gamma_meas`. **Sign gotcha:**
      the first attempt used `asin(R[2,0])` (body-x elevation) which is the OPPOSITE sign to the aero
      AoA → the NMPC modeled lift where the sim has downforce; fixed by using the body-frame velocity.
      Live cross-check: controller `aoa=` now tracks `[AERO] alpha` (−8° vs −8° at steady cruise).
      **A/B (same node, `use_body_aoa` 1 vs 0, identical leg): ON → body ~8° nose-up, downforce −5 N,
      altitude held 4.0; OFF → ~17° nose-up, downforce −14 N, altitude dips to 3.8.** So modeling the
      aero keeps the attitude less extreme (3× less downforce, tighter altitude). Param `use_body_aoa`
      (default 1) toggles it live.
    - **(F) fix — yaw position-hold.** When not yawing-to-target the node now holds `yaw_cmd` as a
      FIXED latched reference (re-latched to `cur_yaw` only while standby) instead of tracking the
      current yaw, so a yaw disturbance is actively restored. **Validated:** a lateral move that used
      to drift yaw 0→61° permanently now disturbs it to 54° and it RECOVERS to the held 80°.
    - Both fixed in `nmpc_cruise_node.py`; hover/climb/cruise/precise-arrival unchanged (re-flew a
      40 m cruise: arrived (46.0,82.1,4.0), altitude 4.0, yaw held). Instrumentation: controller log
      now prints `aoa=` (measured body AoA) to compare against `[AERO]`.

- **CHASE CAMERA + Fly+Land wiring + rosbag workflow + the +AoA cruise limit (2026-08-09).**
  - **Chase camera:** `application/vtol/03_vtol_px4_hil.py` render loop now follows the VTOL body
    (reads `vehicle._state.position` each frame, low-passed viewport eye). Env: `PEGASUS_CHASE_CAM=0`
    (static), `PEGASUS_CHASE_OFFSET="dx,dy,dz"` (default `-8,-8,4`).
  - **Fly+Land wired to nmpc_cruise:** GS `_on_mission` now `_set_controller("nmpc_cruise")` and sets the
    traverse yaw toward the target; nmpc_cruise now tracks the **reference yaw** (`self.yaw_ref`, from
    on_ref) instead of a latched heading, so the tiltrotor FACES the target and flies FORWARD (cruise).
    Trajectory file `trajectories/cruise_fly_and_land.csv` (takeoff→cruise→land, loadable in the tab).
  - **★ rosbag diagnosis workflow (use this to verify runs):** `/tmp/recbag.sh <name>` records
    `/uav_0/mocap` + `fmu/in/actuator_{motors,servos}` + the reference(+_smc) + active_controller to
    `/tmp/bags/<name>`; `/tmp/anabag.py <name> [stride]` prints a time series with the sim-EXACT AoA
    (`v_body=Rᵀv`, `alpha=atan2(w,u)`), tilt (servo), elevator, motors, ref + altitude/AoA/tilt summary.
    (anabag imports the msg classes directly — the ROS python here has no `yaml` for get_message.)
  - **★ Fly+Land VALIDATED from a bag (mission1):** takeoff → cruise to (60,0) at 4.0 m with **rotors
    tilted forward ~20°** (tilt servo 0.17–0.24) and **altitude held 3.9–4.1 m** → **landed precisely
    at (60.1, 0, 0.2)**. The reference (`refz=4.0`) is tracked throughout.
  - **★ +5–6° AoA wing-borne cruise — FRAGILE / envelope limit.** Added an elevator-driven AoA-hold
    (`cruise_aoa_deg`, `k_aoa_elev`, `aoa_v_gate`, `k_aoa_alt/vz`, + pitch-gate + hysteresis latch;
    default 0=off). Bag/sign truth: in cruise the body flies **nose-up ~5° (`asin(R[2,0])≈+5`) ↔ AoA≈−5°**
    (small wing downforce) — so a POSITIVE AoA needs the body **nose-down**, and that makes the wing LIFT,
    which the rotors can't offset at ~6 m/s (wing carries weight only ~12 m/s) → **climb-away/fly-away**
    (repeatedly flew to >800 m / 66 m). So the reliable cruise trims at ≈−5° AoA (holds altitude, rotors
    tilt ~20°); a true +5–6° AoA wing-borne cruise needs ~12 m/s (punch the transition corridor) and is
    beyond the current stable envelope. `cruise_aoa_deg` is left as a tunable experimental knob.
  - **Ops caution:** mid-flight NODE RESTARTS disturb the vehicle into unrecoverable stall-glides — test
    controller changes from a CLEAN boot (vehicle on geometric) or via LIVE params, not mid-flight
    restarts. `tmux kill-server` if a teardown leaves a stale session (a relaunch can attach to it).

- **★ AERO COEFFICIENTS SCALED to the real airframe + SMC-routing fix (2026-08-09) — CLIMB-AWAY FIXED.**
  The aero geometry was the AVTOL's (a 1325 kg aircraft): `wing_area=0.55 m²`, `wing_span=2.0 m` on a
  ~0.6 m fuselage — unphysical, ~5× too much lift, so the wing carried weight already at ~8 m/s and
  drove the climb-away. Scaled `vtol_aerodynamics.py` to a sensible small quadplane: **S=0.35 m², b=1.4 m,
  c=0.25 m (AR 5.6), CL_max 1.5→1.2, CL_α 5.5→4.8, CL0 0.2→0.15** (wing loading ~98 N/m², stall ~11.5 m/s,
  wing-borne cruise ~18 m/s). **The nmpc_cruise controller aero constants were updated to MATCH** (must
  agree per the loop audit). **RESULT (bag aero1, SMC-fed cruise): altitude HELD dead-on at 8.0-8.1 m the
  whole traverse (x 5→110 m), AoA small (−4..+6°), no climb-away, no fly-away.** The over-lift is gone.
  - **SMC-routing fix (answers "why does nmpc+smc not track fly_cruise"):** nmpc_cruise had been reading
    the RAW reference (bypassing the SMC), so in "nmpc+smc" mode the SMC's smooth converging setpoint on
    `_smc` never fed it — it got the raw (step-like) trajectory, which both isn't the smc-smoothed
    reference and is what triggered the fly-aways. Re-added the launcher remap `reference:=reference_smc`
    so nmpc is SMC-fed → **smooth reference → stable tracking** (aero1). TRADE-OFF: the SMC's converging
    setpoint caps the speed (~4 m/s in aero1) — to reach wing-borne speed the SMC convergence rate must
    be raised, or feed a smooth fast reference from a KNOWN start (the raw fast tests flew away only
    because the mission started from origin while the vehicle was left at x≈110 by a prior run — a
    test-state mismatch, not an aero/control failure).
- **WING-BORNE FLIGHT ATTEMPT (2026-08-09, rosbag-diagnosed) — transition ENGAGES but altitude climbs
  away; PARTLY FIXED by the aero scaling above (SMC-fed cruise now holds altitude).** Goal: genuine wing-borne cruise (rotors unload, wing carries weight) with altitude
  held + precise landing. Added to nmpc_cruise: a z-REFERENCE RAMP (a big z-step commanded a max-thrust
  climb that corrupted the AoA and tumbled the vehicle on takeoff — `_z_ramp` at vzmax fixed that,
  rosbag-confirmed), altitude-gated + level-gated AoA-hold (`aoa_z_min`, `|vz|<1.5` — only engage in
  steady level cruise, not takeoff/climb). **RESULT (bag wb2, mission fly 300 m @ peak ~15 m/s):** clean
  ramped takeoff to 8 m, then **GENUINE WING-BORNE — rear rotors fully OFF (mot=0.00), front near-idle
  (0.25), tilt engaged, ~7-8 m/s.** BUT the **altitude climbs away 8→35→80 m**: once the rotors unload,
  the body AoA runs to the ~22° STALL (bag shows AoA 38-52°) → CL_max lift the (now-idle) rotors can't
  offset, and the **elevator saturates (-1.00) unable to pull the AoA back down** to the target. Live
  sign-flip of `k_aoa_elev` came too late (already a stalled glide, flew to 700 m/80 m). CONCLUSION: the
  elevator-only AoA-hold is INSUFFICIENT authority for wing-borne (the AoA is dominated by the flight
  dynamics once the rotors idle; in BOTH the rotor-on and rotor-off cases the elevator couldn't move the
  AoA to its target). Stable wing-borne needs the AoA/pitch CO-OPTIMIZED with tilt+thrust (the offline
  NMPC holds altitude to 20 m/s because it does this) OR a partial-wing-borne law that keeps a rotor-
  thrust floor for pitch authority. **Reliable deliverable stays the ROTOR-BORNE cruise** (Fly+Land,
  tilt ~20°, altitude ±0.2 m, precise landing); `cruise_aoa_deg=0` default (wing AoA-hold OFF).
  Ops: the launcher BOOT-node has bad state → RESTART the node once (on geometric) before switching to
  nmpc, else the switch flies away. Bags in `/tmp/bags/{mission1,wb2,...}`.

- **CRUISE SMC + wing-borne status (2026-08-09, rosbag-verified) — STABLE tilt-cruise achieved,
  genuine wing-borne NOT reached (speed-limited).** New `smc_cruise_node.py` (separate outer SMC for
  cruise): same SMC law but leads the set-point FAR ahead (`reset_dist=30`) at high `vmax`, with a NEW
  `amax` accel-limit on the velocity cmd (added to `smc_outer.py`; gentle ramp = no lurch). nmpc reads
  `reference_cruise` (remap) + `cruise_alt_min` (only tilt-cruise ABOVE this alt → takes off VERTICALLY
  first; a far set-point else tilted it fwd at z=2 m and crashed). **RESULT (bag cruise3): a STABLE
  tilt-cruise — yaw held 0°, ALTITUDE held 8.0–8.2 m over 300 m, rotors tilted ~34°, no crash/fly-away,
  arrives.** Gentle SMC `amax=2.5` + stronger yaw (`kR_yaw=4`,`m_yaw_max=6`) fixed the yaw-flip. **BUT
  rosbag proves it is NOT wing-borne:** the body flies **NOSE-DOWN, AoA −13..−27° (wing DOWNFORCE)** —
  the rotors carry it and FIGHT the wing; and the speed caps at **~3 m/s** (SMC set-point convergence +
  nmpc's decel-toward-set-point + the cruise engage/disengage hysteresis toggling as the vehicle catches
  the set-point). At 3 m/s the wing makes ~0 lift and the elevator has no authority, so the AoA-hold
  (even with the corrected `k_aoa_elev` sign) can't pull the AoA positive. **★ PITCH-SIGN CORRECTION
  (user-caught):** `asin(R[2,0])` is nose-DOWN-positive (it read +18° while the aero AoA was −17°) — the
  analyzer now NEGATES it (`/tmp/anabag.py`) so + pitch == nose-up == +AoA == lift; the AoA (`atan2(w,u)`,
  matches the sim) is the reliable quantity. **The barrier to genuine wing-borne is SPEED:** need ~15-18
  m/s (scaled wing) so the wing lifts + rotors unload + a +AoA holds; the SMC/nmpc pipeline caps ~3-4 m/s
  and pushing the accel re-triggers the yaw-flip. Next: rework nmpc to TRACK the SMC velocity-FF as the
  cruise speed (not decel toward the moving set-point), + a speed-scheduled yaw authority.

- **★★★ SO(3) PATH-FOLLOWING NMPC PORT + DECOUPLED CASCADE — wing-borne transition AND fly-to-point+land
  WORK in HIL (2026-08-10).** The user's AVTOL SO(3) path-following transition NMPC
  (`~/Documents/PhDStudies/ControlDesign_VTOL/NMPC_Design/AVTOL_Work-main/Aircraft_Model_SO3_pathfollowing/
  nmpc_vtol_reduced_pos_att_so3.py`) was PORTED onto the FSC quad, then run in a cascade to make it real-time.
  - **`nmpc_so3_transition_node.py`** (controller_id `so3_transition`) — the faithful in-loop port: full
    6-DOF RK4 predictor, cost = pos + SO(3) attitude `0.5·trace(I−RᵣᵀR)` + vel + tilt vs a hover→cruise
    `TransitionRef` (horizon anchored on `s_from_beta`). State x(19)=[pos,R(row-major),beta,vel,omega];
    control u(8)=[T_FL,T_FR,T_RL,T_RR,betacmd,de,da,dr]. Isaac tilt convention (beta=0 up, 90 fwd).
    Offline `--selftest` does the perfect transition (alt held, tilt 0→70°, wing takes over).
    - **HOVER-HOLD fixes (all REQUIRED, HIL-diagnosed):** (1) **`_uprev` must init to the HOVER TRIM, not 0**
      — the `w_du` control-rate penalty is centred on uprev, so uprev=0 front-loaded a ~0-thrust first
      control (16 N vs 34 N weight) → droop/"hover drift". (2) **never apply OR keep a FAILED IPOPT
      iterate** (`return_status` not Solve_Succeeded/Acceptable): it can be ~0 thrust → free-fall, and
      saving it as the warm-start (`zsol`) poisons every later solve → fall back to last-good + reseed.
      (3) **`CG_X` 0.195 → 0.177** (MEASURED: the sim's balanced hover is front=8.6>rear=8.4 →
      Ffront·(0.459−cg)=Frear·(cg+0.112) → cg=0.177 ≈ the rotor centroid). With these the SO(3) hover is
      rock-solid; `IXX,IYY,IZZ=0.20,0.28,0.35` match the VTOLConfig default.
    - **WHY in-loop fails the TRANSITION (not a gains problem — a SPEED problem):** the IPOPT solve is
      **0.24 s at hover, up to 0.94 s during the transition** (measured; N=25). In-loop that runs the
      control at ~1–4 Hz — fine for quasi-static hover but far too slow for the FAST tilt-transition
      attitude → pitch tumble. IPOPT can't hit the 33 ms budget even at N=12/max_iter=3 (~40 ms floor);
      no acados on this machine; CasADi `qrqp` SQP-RTI gets ~26 ms but is borderline.
  - **`nmpc_so3_cascade_node.py`** (controller_id `so3_cascade`) — **the DECOUPLE (the fix).** OUTER = the
    same SO(3) NMPC in a **background `threading.Thread`** (CasADi releases the GIL during the solve —
    verified the fast loop holds ~99 Hz while solves run) producing the optimal tilt feed-forward `beta_ff`;
    it NEVER blocks. INNER = a fast geometric tiltrotor POSITION controller on every mocap update
    (conventions reused verbatim from the HIL-validated `traj_track_node.py`: forward force via front-rotor
    TILT sized to the fwd-force demand then removed from F_des so the body stays LEVEL; decel via SE(3)
    backward LEAN; altitude by collective; 4-rotor allocation `Ainv@[f,Mx,My,Mz]`, front pair boosted 1/cos).
    - **Wing-borne transition (HIL):** hover → tilt ramps 0→20° → accel 0→10 m/s, ALTITUDE HELD →
      `[AERO]` CL=1.2, +25 N lift (~73% of weight), rotors unload (rear 8.3→0.5 N) → wing-borne cruise.
    - **Fly-to-point + LAND mission (HIL, the deliverable):** a `Mission` reference (trapezoidal-speed
      straight line to `target_x,target_y` at `cruise_alt`/`cruise_speed`, then a controlled vertical
      descent — FSC-scaled, in the spirit of their `TransitionRefInterp`, which is unusable directly
      because it's scaled to the 1325 kg AVTOL). **Result (target (60,0), Vc=12, alt 15): hover → accel
      to ~10 m/s (straight, alt held) → cruise → decel → LANDED at (59.9, 0.0), 0.1 m from the target.**
      Speed control + precise point-landing both work (that run: `beta_ff_gain=0`, NMPC tilt-FF OFF —
      the geometric inner loop's own tilt-for-forward carried it; set `beta_ff_gain>0` to blend the NMPC's
      optimal tilt in). Original rough spot: a ~5 m altitude BULGE (15→20→15) + a +32° pitch transient
      during the ~10 m/s cruise as the wing suddenly lifts (the position loop lagged the uncommanded lift).
    - **WING-LIFT FEED-FORWARD added (to kill the bulge) — code done, HIL re-test PENDING (interrupted).**
      `aero_world(R,v)` computes the modeled wing force and the inner loop uses `F_des -= wing_ff_gain*aero`
      so the altitude loop anticipates the lift. **Sign gotcha (fixed):** the ported `aero_body` in
      `nmpc_so3_transition_node.py` has the LIFT sign FLIPPED vs the sim (fine offline, wrong for a real-sim
      FF); `aero_world` instead mirrors the SIM's decomposition exactly (`lift_dir=[-sin a,0,cos a]`, drag
      along −velocity) — verified it reproduces the sim `[AERO]`: at α=+18.3/V=10.3 → Fbody=(−10.7,0,+25.2)
      vs the sim's (−10.6,0,+25.1). `wing_ff_gain` default 0.8. NOT yet HIL-confirmed to remove the bulge.
  - **GS-WIRED (code done, end-to-end HIL re-test PENDING):** (1) GS button **"E: SO(3) Cascade"** added
    to the controller-switch row (`trajectory_player.py`, publishes `so3_cascade`); (2) `_on_mission` now
    runs on the CURRENTLY-SELECTED controller (`_active_ctrl()`) instead of hard-coded `nmpc_cruise`, so the
    "🛬 Fly to point + LAND" button (Target X/Y, Cruise alt/speed) drives whichever is selected; (3) the
    cascade node now SUBSCRIBES to the RAW GS reference `…/position_controller/reference`
    (`PositionControllerReference`) and a fresh stream OVERRIDES the internal `Mission` — so the GS
    Trajectory tab / Fly-to-point drive it like any controller; (4) launcher window **`6d-cascade`** added
    to `start_vtol_smc_hil.sh` (STANDBY, reads the RAW reference NOT `_smc`; env `CASC_VC/CASC_BFF/CASC_WFF`).
    GS is launched from SOURCE (`/usr/bin/python3 src/single_drone_ground_control.py`) → no colcon rebuild.
    Usage: take off (A), click **E: SO(3) Cascade**, set Target X/Y + Cruise alt/speed, click **Fly to
    point + LAND**. See [[so3-cascade-transition]].
  - **Run (standalone, no GS):** `/tmp/run_cascade.sh` env `TX/TY/ZC/VC/ACC/VD/BFF/WFF` (pass DOUBLES —
    `TX=60.0`; an int override crashes rclpy param typing). Take off on `geometric` (`pubraw.py 0 0 15 0`),
    then publish `active_controller=so3_cascade`. Offline solver probes + selftest in the transition node.

## Ground-station Trajectory tab (`~/ros2_ws/src/ros2_ground_station_gui`)
Extra "Trajectory" tab (added by `src/single_drone_ground_control.py`, logic in
`src/ROS_Node/trajectory_player.py`, numpy-only — GS python has no working scipy).
Loads `.csv/.npz` reference files (`t,x,y,z,yaw_deg[,vx,vy,vz]`, ENU), streams them as
`PositionControllerReference` with **Play/Pause/Stop, spatial+time scale, auto-fit to
geofence, "start at current position", loop**, a **big HOVER (hold) safety button**
(hold current pos, always available, geofence-guarded), and the **controller switch**.
Exporter `tools/export_reference_trajectory.py` turns the AVTOL `transition_reference.py`
generators into trajectory files (run with the isaac_env venv — needs scipy; the
generators model a FIXED-WING transition so even "quad" presets are ~166 m/30 m/s → the
tab's auto-fit shrinks the shape to the geofence). Samples in `trajectories/`.

## Environment gotchas — controllers / GS / HIL (this session)
- **casadi/scipy live in the isaac_env venv** (`~/envs/isaac_env/bin/python3` = the
  ROS-node python; pip-bootstrapped via ensurepip). NMPC/exporter must use it. **System
  `/usr/bin/python3` (the GS python) has BROKEN scipy** (numpy 2.x vs apt scipy) → GS-side
  code must be numpy-only. `pyqtgraph` installed --user for the GS (plots + tab preview).
- **A leftover `autopilot_sv_baseline_node` from a prior session** publishes the SAME PX4
  setpoint topics → two controllers fight → tumble. The plain px4/isaac/agent process
  check misses it. `pkill -9 -f autopilot_sv_baseline` before running any NMPC/direct node
  (the launchers do this in the BODY, not the node pane — the pane's own cmdline contains
  the pattern and would self-kill).
- **tmux: use separate WINDOWS, not tiled split-panes** — 5 tiled panes overflow the min
  pane height and the last split silently drops (the controller pane never launches).
- **PX4 4101 airframe**: set `param set-default SYS_HAS_NUM_ASPD 0` (both ROMFS + the
  `build/px4_sitl_default/etc` copy) or arming fails "Airspeed selector module down". VTOL
  spawns at z=0.3 (on ground). Isaac ROS2-bridge env for VTOL_ROS2_STATE=1: export
  `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` + `LD_LIBRARY_PATH+=.../exts/isaacsim.ros2.bridge/humble/lib`.
- **Self-kill**: never put a `pkill -f <pat>` (or a heredoc containing `<pat>`) in a Bash
  command whose own command line matches `<pat>` — it kills the invoking shell. Put kills
  in a Write-created /tmp script, run as `bash /tmp/x.sh`.
- **Teardown from an automated Bash sandbox silently no-ops** — `pkill -f`, `kill -9 <pid>`
  and `tmux kill-server` return exit 1 and the px4/isaac/DDS/tmux stack SURVIVES (this is why a
  crashed/inverted vehicle "carried into the next run"). Fix: run teardown with the sandbox
  DISABLED, and even then `pkill -f`/`tmux kill-server` are unreliable → gather pids with
  `pgrep -f` and `kill -9` each EXPLICITLY. Use `bash /tmp/kill_hil.sh` (does exactly this),
  sandbox-off, before every relaunch. Launcher: `start_vtol_smc_hil.sh --here pieper_machine`
  (needs the config NAME arg; run standalone, not chained after the teardown in one command).

## Environment gotchas (this machine)
- **GUI terminal launch**: `~/.bashrc` sources ROS + a venv (`isaac_env`) into every
  shell; that polluted env crashes gnome-terminal ("undefined symbol:
  `__libc_pthread_init`") when a script spawns it. `terminal_utils.sh
  open_new_terminal` launches the terminal with a **pristine env** (`env -i` + only
  DISPLAY/DBUS/XDG/HOME/PATH), like the desktop shell; `bash -lc` re-sources the
  profile for the work command.
- **rclpy inside Isaac**: run scenes from a **clean** shell (`PYTHONPATH`/conda unset,
  as the scripts do) so Isaac's bundled rclpy is used, not system ROS py3.10 (ABI clash).
  Isaac's ROS2 bridge only provides `rclpy` if its env is set (`ROS_DISTRO`,
  `RMW_IMPLEMENTATION`, `AMENT_PREFIX_PATH`, bridge `LD_LIBRARY_PATH`); otherwise
  `import rclpy` fails. Therefore **do not hard-import rclpy at module top in code an
  Isaac scene imports** — guard it (`fsc_aerial_manipulation/utils/__init__.py` now
  wraps `ROS2RigidBodyBackend` in try/except; a bare `import rclpy` there previously
  crashed scene 03 at import via `add_dome_lighting`).
- **Mocap (`VTOL_ROS2_STATE=1`) needs Isaac's ROS2 bridge working** (see above) —
  the plain HIL path (`VTOL_ROS2_STATE=0`) does not use rclpy and is unaffected.
- Status: scene 01 runs; scene 03 imports fixed. Follow the runbook's staged 01→02→03.

## ★★ FIXED-WING CRUISE via the SCALED AVTOL REFERENCES (2026-08-10) — D validated, full analysis in docs/fixed_wing_control_analysis.md
The AVTOL reference generators (`transition_reference[_consistent].py`, profile builders) are
WIRED IN + SCALED to this airframe: `ros2/px4_vtol_direct_actuator/.../fsc_transition_ref.py`
(m=3.5, S=0.35, T_max=60 → force-balanced trim schedule V→(tilt, α, θ, Ttot, de); self-tested:
hover 34.9 N → cruise@15: tilt 90°, α 6.9°, T=3.4 N, wing ~100%; env `AVTOL_REF_DIR`,
`FSC_TILT_CAP_DEG` default 65).
- **D (`nmpc_cruise`, GS button D) = the VALIDATED FIXED-WING fly-to-point** (bags fwD8/9/10:
  3/3 250 m arrivals; clean hover→fixed-wing transition — tilt 0→65°+, rear pair unloads →0,
  altitude dead-flat through the hand-off; residual ±10 m cruise phugoid = open issue). Profile
  drives the horizon θ/β refs + a THRUST-hand-off term (`w_T_ref` — without it the solver sits in
  a rotor-borne KKT trap) + a PITCH-BALANCE term `w_pm·(Tf·cosβ·xf − Tr·xr)²` (point-mass model is
  blind to the split's pitch moment → rear-loaded → nose-over, bag fwD2; cg_x=0.177 MEASURED) +
  schedule-bounded β (`bref+6°`) + applied-β slew 35°/s (β bang-banged 80↔0 at the wing-borne
  boundary, fwD4) + AoA gates at 2 m/s (0.2 let hover-drift noise fake θ=9° → doomed runs, fwD7).
  **KEY BUG (explains the old "rotors stay vertical in cruise" mystery): the NMPC's du penalty was
  anchored to a HARDCODED hover Uprev → the first-step tilt was dragged to 0 every cycle =
  receding-horizon procrastination; uprev now = last applied control via P.**
- **E (`so3_cascade`, GS button E) = validated geometric fly-to-point** (fwE8: landed 0.12 m from
  the 60 m target) in its MC mode, improved by Mission LEAD-CLAMP (ref ≤10 m ahead), z-RAMP, and a
  CLOCK-FREEZE (trapezoid pauses while lagging). Its profile fixed-wing branch is EXPERIMENTAL
  (`CASC_FW=1`): traverse-engage transient unresolved (bags fwE2–7: MC↔FW tilt flapping → dive;
  backward slide from the one-sided alt-corr; FW self-trap at v≤0 — each fixed, engage still not robust).
- **GHOST PUBLISHER (fixed):** standby `aero_transition_node` (window 6b) co-published actuators at
  200 Hz against the active controller (constant 45°-tilt + zero-elevator stream, bags fwD1/fwE2) —
  6b now DISABLED by default (`RUN_TRANSITION_C=1` to re-enable); `nmpc_so3_cascade` added to
  `/tmp/kill_hil.sh` (it survived every teardown before).
- **GS STOP+SAFE-HOVER**: now actually forces "geometric" (it silently kept the active controller
  before) with a STAGED handover (active controller decelerates on the hover ref; switch <4 m/s /
  12 s). **LIMIT: an instant STOP from full fixed-wing cruise (>10 m/s tilted) is NOT survivable**
  (stop1–3: tilt-up at speed → wing+rotors=2× weight → zoom-climb → stall-drop). Abort from cruise =
  let the mission decelerate, or a far-ahead hover point. Multicopter-envelope STOP unchanged/safe.
- **Why fixed-wing kept failing (the user's question)** — six stacked causes, five fixed
  (procrastinating du anchor; no force-consistent reference; pitch-blind point-mass split; bistable
  boundary replanning; garbage low-speed AoA + the ghost publisher), one characterised+open (the
  LAST 15% of the lift hand-off: one-sided rotor pitch authority + phugoid with no fast
  lift-shedding actuator → needs a 6-state longitudinal NMPC with pitch dynamics + elevator in the
  loop). Full write-up: `docs/fixed_wing_control_analysis.md`.
- GS Reference dropdown now has FSC-SCALED AVTOL trajectories (`fsc_fixed_wing_transition.csv`,
  `fsc_transition_climb_turn.csv`); exporter `--consistent` velocity-index bug fixed (30-state
  layout carries v at [17:20], not [16:19]).
- Test drivers: `/tmp/run_{d,e,stop}_test.sh` (verified param-set + bag + monitor); launcher knobs
  `FW_VCRUISE=15 SMC_VMAX=20 FSC_TILT_CAP_DEG=65 CASC_FW=false RUN_TRANSITION_C=0`.

## ★★★ THE USER'S SO(3) NMPC FLYING 90° PURE-PULLER IN-LOOP (2026-08-10 late) — see docs/fixed_wing_control_analysis.md (UPDATE section)
Directive: fly the AVTOL NMPC derivations directly. `nmpc_vtol_reduced_pos_att_so3.py` (via its
port `nmpc_so3_transition_node.py`) now runs IN-LOOP at 25 Hz: **JIT-compiled IPOPT (N12/DT0.1/RK1)
= 20 ms mean / 36 p95 per solve** (was 240–940 ms), cached via jit_serialize=embed
(`~/.cache/fsc_so3_nmpc_*`; cache key MUST include weights+max_iter — serialized solvers bake
options; a stale cache silently no-ops changes). **MAX_ITER=60 REQUIRED** (50 = deterministic
corridor failure cascade). fatrop present but slow here; qrqp-RTI diverges.
- **Fixes that made it fly:** (1) ★ LIFT-SIGN in the ported aero_body was FLIPPED vs the sim
  (port trimmed at α=−9 offline = downforce in HIL → dive; bag so3a) — now mirrors
  vtol_aerodynamics verbatim; (2) reference schedules from the FSC-scaled TransitionRefInterp
  incl. TRIM-PITCH attitude ref (level ref fights the wing at high tilt); (3) w_om=3 planned rate
  damping (roll blew up where front differential = yaw authority) + w_att 120 + w_du 6; (4)
  TIME-RATCHET path anchor with 0.15 leash (tilt-only anchoring parks in a rotor-borne local
  optimum at ~30°); (5) rear floor relaxes with s; tilt integrator freezes on failed solves.
- **OFFLINE: full 90° transition + INDEFINITE steady pure-puller cruise** (β=90.0, V=13.8, z flat,
  AoA +7.2 = true trim, rotors 6 N tot). Latency: needs true ≲130 ms, comp ≥ true (under-comp −30 ms
  fatal; over-comp = safe partial-tilt stall).
- **HIL (bag so3c, SO3_TF=20): transition completed + ~22 s / 200 m STABLE 90° cruise** (tilt servo
  1.00, rear ~1 N, front ~4 N ≈ drag, alt flat 8.1–8.4, V 11.9, straight) — ended by a slow
  roll/yaw departure (~6 s) = the lightly-damped lateral mode at full unload. tf=30 lingers in the
  corridor and crashes (so3d) — FASTER through the corridor is better. Corridor in HIL sags
  nose-down (elevator saturates) with run-to-run variance — latency calibration is the lever.
- Ops: window `6e-so3`, knobs SO3_VC/BC/TF/LAT (launcher FORCES DOUBLES — rclpy int-for-double
  rejection killed one run). Activation: verify the ENGAGED log (one-shot pub misses discovery).
  EVERY sim launch records a bag; read it after each test (user directive). Bags so3a–so3d.
- NEXT: per-axis w_om (roll), da/dr gains 1.0, latency calibration from commanded-vs-realized tilt
  slew in a bag; then long-cruise soak.

## ★★★ 90° WING-BORNE CRUISE SOLVED — 6-STATE PITCH-MOMENT NMPC (2026-08-12)
The rotor→wing handoff corridor that defeated EVERY controller all session (SO(3) mission-tracking,
point-mass nmpc_cruise, cascade) is CRACKED by a new node that co-optimizes the PITCH MOMENT.

- **ROOT CAUSE of the whole-session failure (found by auditing the REFERENCE):** plugging each
  reference (xref,uref) into `build_ode()` showed `omega_dot(pitch) = -7..-8 rad/s²` through the whole
  corridor — the reference was NOT a trim. The old front/rear thrust split was a heuristic `rear_frac`
  fade that never PITCH-BALANCED the FSC 4-rotor geometry. **Fix: solve the split so `My_thrust +
  My_aero = 0`:** `Tf = (xr*Ttot - My_aero)/(2*(zf*sin b - xf*cos b + xr))` (xf=0.282, zf=0.02,
  xr=-0.289 rel CG_X=0.177). Drove the pitch residual -7 → ~0 everywhere. Wired into
  `nmpc_so3_transition_node.TransitionRef.get()` AND `fsc_mission_ref.py`.

- **`ros2/px4_vtol_direct_actuator/.../nmpc6_cruise_node.py` — the SOLUTION.** A 6-state LONGITUDINAL
  NMPC: state `X=[s,z,vs,vz,theta,q]`, control `U=[Tf,Tr,beta,de]`, `qdot=My/Iyy` = thrust-split
  moment + aero (`cm0+cma*alpha+cmq*qhat+cmde*de`), ELEVATOR IN THE LOOP. Every other controller plans
  body pitch as if directly commandable; this one predicts+counters the pitch moment. `build_ode6()`
  is VALIDATED byte-identical to the sim plant (`--validate`, err 0.0000; aero/geom imported from
  nmpc_so3_transition_node so they can't drift). **Offline vs the TRUE plant WITH 100 ms latency:
  hover → transition → sustained tilt=90°, V=16, AoA=+6.4°, altitude DEAD-FLAT, rear rotors OFF, front
  3.8 N (drag), wing carries the weight.** The 5 keys (all necessary): (1) 6-state pitch dynamics;
  (2) STATE bounds on the stiff pitch (theta∈[-20,30]°, q∈[-3,3] — unbounded, IPOPT explores qdot~-200
  → ill-conditioned → solve fails); (3) reference-feedforward FALLBACK on solve failure (a frozen
  control diverges); (4) SOFT tilt guide (`w_beta=12` — the NMPC self-limits the tilt through the
  corridor, then commits ~V=10; forcing the schedule tilt drove the nose-down trap); (5) delay
  compensation. N=14/DT=0.06/RK4×2, JIT-compiled (`jit_serialize:embed`) → **9 ms solve**, cached to
  `~/.cache/nmpc6_cruise_*.casadi` (first build ~2 min, then Function.load 0.1 s). Reference =
  `Schedule.at(V)` (FSCTransitionProfile trim + pitch-balanced split, SPEED-gated pitch=`theta_cmd` so
  the engage is a LEVEL hover). Lateral (roll/yaw) = a TILT-AWARE stabilizer (differential-thrust roll
  scaled by cos(beta) → fades to 0 by 90° where the front rotors are horizontal + the rear is
  unloaded; AERO surfaces take over at cruise).
  - **Wiring:** launcher window `6f-nmpc6` in `start_vtol_smc_hil.sh` (STANDBY; env `NMPC6_VC/CAP/TTR/
    LAT/ALT`); GS button **"✈ 6-state Wing-borne (90°)"** in `trajectory_player.py` (publishes
    `nmpc6_cruise`). Take off on geometric, then engage. Runbook `docs/wing_borne_90_runbook.md`.
  - **HIL hardening (each root-caused offline; NONE reproduce in the non-real-time offline test):**
    (a) solve 53 ms > 40 ms 25 Hz budget → loop falls behind → latency balloons → tumble: fixed by
    JIT (9 ms). (b) +8° hover-pitch engage → nose-up with no airspeed → tumble: fixed by speed-gating
    the pitch. (c) `self.beta` was UNDEFINED (the 6-state model has tilt as a DIRECT control, no
    tilt-state) → loop threw AttributeError before pub() → 'does nothing / just lands': use the local
    `beta`. (d) ★ RUDDER SIGN backwards (`CNDR=-0.12` NEGATIVE → +dr gives -yaw): `+k_rud*Mz` AMPLIFIED
    the yaw → ran to -100°, huge arc (HIL bag myrun); fixed to `-k_rud*Mz`. (e) engaged at 1.5 m → sank
    to 0.2 m repeatedly: param `cruise_alt` (default 15) climbs to a safe altitude BEFORE transitioning.
  - **MOCAP pitch VERIFIED consistent** (user asked): mocap = FLU-body rel ENU-world, angular_velocity
    = FLU BODY frame (State def). `theta=-asin(R[2,0])` EXACTLY equals the sim AoA (`v_body=Rᵀv`), and
    driving to the reference theta=+8 makes the plant CLIMB (+2.07 m/s²) — pitch measurement + reference
    are correct. But the pitch is RAZOR-sensitive at 90° (theta=0 → -8 m/s² dive), so any lateral
    divergence that drops it → nose-dive. Empirical check: `/tmp/mocap_pitch_check.py`.
  - **OPEN:** combined roll+yaw disturbance still crashes offline (`--roll0 10 --yaw0 20`) = residual
    lateral robustness; pure roll or pure yaw recover. NOT yet cleanly HIL-flown (3 attempts, each fixed
    a distinct bug).

- **FULL MISSION (≤14 m/s tilt-assisted) on the SO(3) node — `mission_mode` (Track A).** `fsc_mission_ref.py`
  (FSCMissionProfile) builds a TIME-parameterised climb→transition→cruise→back-transition→settle→land
  trajectory through the AVTOL `_solve_coupled_thrust_and_beta` trim (force-EXACT tilt(V), wing-load
  gate, pitch-balanced split). `nmpc_so3_transition_node._mission_step` (param `mission_mode:=1`, a
  SELF-CONTAINED branch — the HIL-validated legacy path is untouched) flies it: state-gated clock
  (speed-gated trans_out) + `solve_mission(anchor_xy=)` + strong attitude weights (`mission_w_att=220`)
  + elevator pitch-hold. Offline: landed 0.4 m from a 200 m target. Launcher env `SO3_MISSION MISSION_TX/
  TY/VC/CAP/ZC`; runbook `docs/mission_hil_runbook.md`. (>14 m/s reintroduces the corridor → that's what
  the nmpc6 node is for.)

- **Bag tooling for nmpc6:** `scripts/recbag_nmpc6.sh <name>` (mocap + actuator_motors/servos +
  active_controller + px4 out estimate); analyze with `/tmp/ana6.py <name>` (NOT anabag.py — it chokes
  on the px4 out-topics). Healthy cruise = pitch≈AoA≈+6°, roll/yaw≈0, tilt→90°, rear motors→0. Full
  topic table + failure signatures: `docs/nmpc6_rosbag.md`.

## ★★★ FULL-STATE (19) NMPC + OUTER POSITIONAL SMC (2026-08-12 late) — supersedes nmpc6 for robustness
`ros2/px4_vtol_direct_actuator/.../nmpc_full_smc_node.py` (controller_id `nmpc_full`, launcher window
`6g-full`, GS button "★ Full-state NMPC + SMC (90°)").

**WHY:** nmpc6 cracked the corridor by co-optimizing the PITCH MOMENT, but its lateral axes were a
hand-rolled PD OUTSIDE the solver — fragile exactly at 90° (front rotors horizontal → differential
thrust makes YAW not roll; rear unloaded → no roll authority), and a **combined roll+yaw disturbance
still crashed it**. This node keeps the pitch-moment insight and puts the FULL attitude + tilt + body
rates INSIDE one NMPC over the sim-exact 6-DOF plant, so the solver allocates 4 rotors + 3 surfaces
against ALL coupled moments at once.

- **Model:** `so3.build_ode()` (the 19-state sim-exact plant) — x=[pos, R(row-major), beta, vel,
  omega], u=[T_FL,T_FR,T_RL,T_RR, betacmd, de, da, dr]. No new dynamics written = cannot drift.
- **Cost:** pos + vel + **SO(3) attitude `0.5·tr(I − Rᵣᵀ R)`** against `Rᵣ = Rz(heading)·Ry(θ_trim)`
  (roll=0, yaw=heading → **lateral is TRACKED, not patched**) + soft tilt + rate damping (all 3 axes)
  + control-ref/du. N=12, DT=0.08, RK1, JIT (`jit_serialize:embed`) cached to `~/.cache/nmpc_full_*`.
- **Outer SMC:** subscribes `reference_smc` (launcher remap) and CLAMPS the along-path reference at the
  SMC destination, so the NMPC flies the wing-borne schedule TOWARD the commanded point.
- **Reference:** the same validated `nmpc6.Schedule.at(V)` (FSCTransitionProfile trim + pitch-balanced
  split + SPEED-GATED pitch → level engage) wrapped into a full 19-state xref/uref.
- **The 5 nmpc6 keys retained:** full moment dynamics; STATE bounds on the stiff rates (now all 3 axes,
  |ω|≤3, |vz|≤12); reference-FF fallback on solve failure; SOFT tilt guide (`w_beta=12`); delay comp.
- **OFFLINE vs the TRUE plant @100 ms latency:** sustained **tilt=90°, V=16, AoA=+6.4°, altitude
  DEAD-FLAT 20.0 m, rotors 3.8 N (rear≈0), roll=0, yaw=0** — AND it **RECOVERS from roll10+yaw20,
  roll15+yaw−25, yaw30**, every one of which the 6-state node could not survive.
- **Timing:** solve **22 ms mean / 31 ms max** → default `rate_hz=20` (50 ms budget). 25 Hz leaves only
  ~9 ms and an overrunning solve is what killed an earlier HIL run.
- **Ops:** `FULL_VC/CAP/TTR/ALT/RATE/LAT` env; take off on geometric, then activate `nmpc_full`.
  `cruise_alt` (default 15) climbs to a safe altitude before transitioning. Bag with
  `scripts/recbag_nmpc6.sh` and read with **`scripts/ana_bag6.py`**. NOT yet HIL-flown.

**★ /tmp IS WIPED ON REBOOT** — it took out every rosbag AND all the /tmp helper scripts
(anabag.py, ana6.py, recbag.sh, kill_hil.sh, pubraw.py, the offline probes). Keep reusable tooling in
the REPO: `scripts/recbag_nmpc6.sh` + `scripts/ana_bag6.py` (persistent). Check `ls -dt /tmp/bags/*/`
before claiming to analyze a run.

### FIRST HIL FLIGHT of the full-state NMPC (2026-08-13, bag `full1`) — FAILED, 4 causes fixed
Engage/hover was ROCK SOLID (z held, roll=0, yaw=0, ok=1) — the SO(3) attitude cost does hold the
lateral at hover. The TRANSITION failed: repeated attempts, each pitching **nose-down −20..−26°**
(AoA −21..−25 = wing DOWNFORCE), speed never past ~3.6 m/s, then tumble to inverted (roll ±178°),
fall, auto-recover via `cruise_alt`, retry — until it crashed at x=71.6. Fixes applied (offline
regression re-passed after: 90°/AoA 6.4/roll,yaw 0, and roll10+yaw20 still recovers):
1. **Loop overrun** — solve **43 ms mean / 69 ms max** vs the 50 ms budget @20 Hz (offline it is
   22/31 ms; HIL CPU load from Isaac+PX4+GS ~doubles it). → `rate_hz` 20 → **15** (67 ms budget).
2. **Latency UNDER-compensation** — `latency_s` 0.10 was below the true loop latency (43 ms solve +
   DDS + PX4 + Isaac). Prior finding: under-comp is FATAL, over-comp is safe. → **0.18**.
3. **da/dr were UNPENALIZED in the cost** → IPOPT parked them anywhere; the bag shows the **rudder
   pinned at +1.00** all run = a standing yaw disturbance at speed. → added `w_surf=3.0`.
4. **Tilt-state estimate vs the sim's RATE-LIMITED servo** — commanded tilt thrashed 32→60→45→90°;
   the sim servo rate-limits (~80°/s) while our first-order estimate LEADS it → the NMPC plans with
   the wrong thrust direction. → applied-tilt **slew limit** (`tilt_slew_deg_s`=60) + the estimate
   now integrates the SLEW-LIMITED command with the same 80°/s cap.
**Ops gotchas learned:** (a) `scripts/pubref.py` (raw-reference streamer) SURVIVES teardown like
pubraw — now in `kill_hil.sh`'s pattern list; a stale one had the geometric controller chasing a
400 m target at 20 m/s on the next launch. (b) Don't run the OFFLINE test while tearing down —
`kill_hil.sh` matches `nmpc_full_smc_node` and kills it. (c) Helper scripts now persist in
`scripts/` (`kill_hil.sh`, `switch_ref.sh`, `pubref.py`, `recbag_nmpc6.sh`, `ana_bag6.py`).
**STATUS: the 4 fixes are in + offline-validated but NOT yet re-flown in HIL.** Next run: launch,
climb on geometric, bag, engage `nmpc_full`, then release the far target; read the bag and check
whether the nose-down persists (if it does, the tilt estimate is still the prime suspect).

### 2nd/3rd HIL runs of nmpc_full (bags full1/full2) — TWO ROOT CAUSES FOUND
**(A) MY SMC-CLAMP BUG (bag full1, "never accelerates past 3.6 m/s").** The clamp did
`xr[13:16]=0` — ZEROING the velocity reference for every horizon stage past the SMC set-point. The
SMC set-point is a CONVERGING one leading by only `reset_dist` (8-30 m) that re-seeds, so the NMPC
got a CONTRADICTORY reference: the schedule said "tilt to 16 m/s cruise" while pos/vel said "stop
here, v=0" → could never accelerate, attitude fought the tilt → nose-down → tumble. **FIX:** brake
only when genuinely approaching the destination, via a sqrt stopping profile
(`v_cap=sqrt(2·a_br·rem)`, param `brake_accel`); outside braking distance the schedule runs free.
Confirmed in flight: after the fix it accelerated 0.5→4.6 m/s (vs the old 3.6 cap).

**(B) OVER-TILT AT ZERO AIRSPEED → NO PITCH AUTHORITY (bag full2, the actual crash).** Timeline
from engage: stable hover → tilt ramps 0.04→0.24→0.34 → **pitch −2→−5→−8→−17→−80° in ~6 s** with the
**elevator saturated at +1.00** fighting it and the rotors front-heavy → inverted (roll −171°) →
ground. Mechanism: the solver tilted to **82-90° while V was still ~1 m/s**; at high tilt the front
rotors are horizontal (almost no pitch moment) AND the elevator has no dynamic pressure → **zero
pitch authority** → the nose simply falls. `w_beta=12` is too soft and the 0.8 s horizon can't see
the authority loss. **FIX: the AIRSPEED TILT CLAMP** — applied tilt ≤ trim_tilt(V_now) + 12°
(`tilt_margin_deg`), self-relaxing to 90° as speed builds. This is the SAME clamp the SO(3) node
already needed (bag so3i) and which I failed to carry over into this node.

**(C) REAL-TIME: the 19-state NMPC is too big for this machine under HIL load.** N=12/iter60 =
22 ms on an IDLE machine but **48-96 ms** with Isaac+PX4+GS competing (budget 67 ms @15 Hz), with
`ok=0` solve failures. **FIX: N 12→8, DT .08→.10 (same ~0.8 s horizon), MAX_ITER 60→25.**
Real-time margin matters more than horizon length here — measure solve time IN HIL, never offline.

### nmpc_full HIL campaign — the plateau is a STALL DEPARTURE, not a lift wall (2026-09-07, bag full_tr12)
Level-flight speed across the campaign: 5.6 -> 8.45 -> 8.71 -> 10.5 -> **12.42 m/s** (tilt 71.2 deg,
wing 54.4%). The 0.8 -> **1.6 s horizon** (`N=8, DT=0.20, NRK4=1` -- same symbolic size, ~5 min JIT
build; DT=0.25/NRK4=2 does NOT build inside 15 min) broke the 10.5 m/s plateau, as predicted.
**MEASURE LEVEL FLIGHT, NEVER PEAK SPEED.** `fly_transition.py` now reports only samples with
|vz|<1.5 and flags >5 m of altitude loss -- flight 11's "TRUE FIXED-WING 15.83 m/s" was a DIVE, and
flight 12's 14.80 m/s peak likewise. The honest numbers are the LEVEL ones.
**Root cause of the departure (quantitative, bag-confirmed):** the effective CL peaks at **AoA
exactly 10.0 deg (CL 0.790)** and is down to **0.334 at 25 deg** (`stall_alpha_rad`=13 deg, the
sigmoid blend bites well before it). Flight 12 ran to AoA +25.3 deg -> wing share collapsed
52.4% -> 23.8% (42% predicted / 45% measured -- the model and the bag AGREE) -> sank -> and the sink
RAISED AoA further (descending at speed tilts the flight path down) -> unrecoverable.
**It was NOT short of force:** at 12.4 m/s the wing can carry 77% of weight and the rotors have
23.4 N of vertical capacity at 71 deg tilt vs the 7.8 N needed. Level flight there was feasible; it
was lost to attitude, not capacity. Stall speed is **14.13 m/s** (wing carries 100% at the CL peak),
so 90 deg tilt needs ~14 m/s and the whole problem reduces to holding AoA on the peak while speed builds.
**Fix under test:** `w_aoa` 0.5 -> **2.0** (`aoa_lim` stays 10 deg -- it sits ON the CL peak by
design). The corridor requires riding a knife edge with no margin either side, so the guard has to be
a wall, not a suggestion.

**★ DO NOT RETRY (bag full_tr13): `w_aoa` 0.5 -> 2.0.** It is a REGRESSION and the reason is
numerical, not aerodynamic. `(alpha/aoa_lim)^6` reaches 244 at alpha=25 deg, so at w=2.0 the cost
term is ~488 with gradients to match; IPOPT cannot converge that inside `MAX_ITER=25`.
**Every solve returned ok=0 from V=6 m/s / tilt 46 deg onward** and the solve time went 21-34 ->
**50-80 ms against the 67 ms budget** at 15 Hz. The node fell back to `safe_hover()` (visible as
`mot=[0.47,0.47,0.47,0.47]`, rotors pinned at 33.8 N), the vehicle coasted, pitched to +38 deg,
stalled, slid backwards (AoA -> 179 deg) and tumbled to the ground.
**READ THE SUMMARY'S PROVENANCE, NOT JUST ITS NUMBERS:** that run reported "50 level samples, wing
80.3%" -- both were recorded AFTER the controller had dropped out, during an uncontrolled glide at
tilt=0. A level-flight filter does not distinguish flying from gliding. Check WHO WAS FLYING (ok=1
and a non-hover motor pattern) before crediting any number.
**CONSEQUENCE: the AoA hold is limited by the SOLVER BUDGET, not by tuning.** 0.5 is the largest
weight that still solves. Making the guard stiffer requires making the solve cheaper first (lower
exponent for better conditioning, fewer states, or a faster QP) -- not a larger weight.

### ★★ EXPONENT 4 FIXES THE SOLVER — 79.5 deg TILT, ROTORS TO 9.5 N (2026-09-07, bag full_tr15)
`_aoa_excess` is now `(alpha/aoa_lim)^4` with **`w_aoa`=1.5** (was ^6 at 0.5). The ^6 form blew up
in the FAR TAIL (244 at alpha=25 deg), so raising the weight to make it hold at the limit made the
gradient unsolvable. ^4 reaches only 39 there: the guard is **3x STRONGER at the limit** (1.5 vs
0.5, where it must hold the CL peak) with a **12x SMALLER gradient at 25 deg** than the ^6/2.0 form
that broke -- and ~3x smaller than even the ^6/0.5 that already solved. **`ok=1` held right through
the corridor** (V=6.6 tilt 48, V=8.3 tilt 60, V=10.3 tilt 73, all 26-37 ms) where full_tr13 collapsed
at V=6 / tilt 46.
**BEST CONTROLLED STATE OF THE CAMPAIGN** (rotors unloaded, altitude held, ~4 s each):
```
t  76.0- 79.9 (3.9s)  V  9.9 avg/11.8 max  tilt 79.1  wing 64.1%  rotors 10.0N  alt 18.9-22.7
t 121.1-125.3 (4.2s)  V 10.9 avg/13.5 max  tilt 79.5  wing 91.2%  rotors  9.5N  alt 19.0-21.1
```
vs flight 12's 71 deg / 54% / 16.8 N: **+8 deg of tilt and the rotor load nearly halved** at
comparable speed. Still NOT sustained -- it drops back to the fallback after ~4 s and eventually
departs. Cruise AoA offline is unchanged at 5.7 deg, so the stiffer guide does not bias the trim.

**★★★ "LEVEL" IS NOT ENOUGH -- THE SAMPLE MUST ALSO BE FLOWN.** When a solve fails the node falls
back to `safe_hover()`: equal thrust ~= weight (33.8 N), tilt 0. The vehicle then GLIDES, and a
glide at speed is level, fast AND wing-loaded. full_tr15 reported **"15.31 m/s, 79 deg, 120.7% wing,
TRUE FIXED-WING"** -- but **6697 of its 8411 level samples were at fallback thrust**, and both the
15.31 m/s and the 120.7% were recorded while gliding. The genuine controlled maximum was 12.0 m/s.
`fly_transition.py` now requires `thrust < 32 N` (not the hover fallback) for a sample to count,
prints the CONTROLLED count next to the level count, and prints the single best SIMULTANEOUS sample
-- the old line printed per-column maxima that were never simultaneous. Three runs in a row
(11 dive, 13 post-dropout, 15 glide) produced spectacular false headlines; check WHO WAS FLYING.

**TWO SILENT STACK FAULTS cost a flight each, both looking perfectly healthy:**
  * **PX4's SIM BATTERY drained to its 50% floor and `COM_LOW_BAT_ACT`=3 (Land) fired.** The
    controller commanded correct hover thrust throughout while PX4 walked the vehicle down at
    0.5 m/s. Fix: `SIM_BAT_MIN_PCT`=100, `COM_LOW_BAT_ACT`=0 (saved in the airframe params). Every
    ~5 min JIT rebuild leaves the vehicle idling airborne, so this WILL recur without the fix.
  * **STALE DDS DISCOVERY after a node restart:** the node listed the right subscriptions but
    received nothing, so `self.state` stayed None and its loop returned silently every tick -- no
    logs, no engagement, no error. The whole run was flown by the geometric controller (tilt 0).
    **Always verify the node LOGS and `ok=1` before committing to a flight**, not just that it
    appears in `ros2 node list`.

### ★★★ THE 1 m ALTITUDE BAND WAS ENDING EVERY CRUISE (2026-09-07, bags full_tr16/17)
`settling` was re-evaluated EVERY TICK as
`(self._eng < settle_s*rate) or (z < self.z0 - 1.0)`. That is meant as a PRE-transition gate
("do not tilt over until settled and at altitude"), but in cruise a **1 m altitude dip threw the
vehicle straight back to the hover reference**: tilt commanded to 0 at 12 m/s (the documented
unsurvivable manoeuvre) and `_v_cmd` re-anchored to the achieved speed. full_tr15's two wing-borne
episodes each died this way after ~4 s while the flight was holding altitude to **within 1 m**
(19.0-21.1 with z0=20) -- the REFERENCE punished it for touching the edge of its own band.
**This is the FOURTH instance of a reference surrendering to its own tracking error** (SMC re-seed
C7, GS mission clock, `build_refs` vtar, now the settle gate). Look for this pattern FIRST.
**FIX: the gate LATCHES OFF once satisfied** (`self._settled`); in cruise it only reverts to hover
on a real `cruise_alt_band` (6 m) loss. **RESULT (full_tr16): episode 4.2 -> 6.2 s, tilt 79.5 ->
83.0 deg** -- the highest CONTROLLED tilt of the campaign.

**★ DO NOT RE-ENABLE THE VERTICAL-FORCE TILT CLAMP AS WRITTEN (`lift_margin`, default 0 = OFF).**
The idea -- allow tilt only while `T_max*cos(beta) + L_wing(V) >= k*W` -- is sound and needs no
tuning (it self-relaxes to 90 deg at exactly 14.1 m/s). As implemented it made things WORSE
(full_tr17: level wing share 74.5% -> 24.7%, verdict back to MULTICOPTER):
  1. it credits the wing with its PEAK CL, which only holds at alpha=10 deg -- during acceleration
     the real lift is far lower, so the clamp was most permissive exactly when the wing was
     under-delivering;
  2. **restricting the TILT does not remove the demand for forward force, it MOVES it.** Denied
     tilt, the solver accelerated by pitching DOWN: it reached 89.4 deg tilt at 15.0 m/s with the
     wing at 95% -- while SINKING 6.8 m. Another gravity-not-lift reading.
To make it work it must use the ACTUAL modelled lift at the current state AND be paired with
something that stops the solver trading altitude for speed.

**WHERE THE CAMPAIGN STANDS:** best CONTROLLED wing-borne state is full_tr16 --
**tilt 83.0 deg, wing 86.8%, rotors 9.4 N (28% of hover), 6.2 s**. The corridor is genuinely
ENERGY-LIMITED below the 14.13 m/s stall speed: forward force must come from tilt (costs vertical
thrust) or from pitching down (costs altitude), and neither is free. Sustained 90 deg needs the
vehicle to arrive at ~14 m/s WITH altitude intact; that is the open problem.

### ★★ TWO STACK HAZARDS FOUND WHILE RE-FLYING (2026-09-07)
**(a) THE HIL LINK DIES SILENTLY AFTER A LONG SESSION.** bag full_tr21 recorded a vehicle whose
position was CONSTANT TO FOUR DECIMALS while the controller happily varied tilt 1.6->13.3 deg and
thrust 31-38 N. PX4 was printing `ERROR [simulator_mavlink] poll timeout` and Isaac was holding
the last stale controls -- after ~6.7 h of sim time and 2.1 km of travel. Nothing in the controller
or the harness noticed. **A frozen position with LIVE, VARYING commands means the LINK is gone**;
check the PX4 pane for poll timeouts before believing any flight result.

**(b) ★ THE SAFE CONTROLLER REFUSED TO RECOVER A RUNAWAY -- MY OWN ENVELOPE GUARD.** After the
restart the geometric controller still had flight 21's LATCHED target (2500 m ahead) and flew
toward it at 34 m/s, descending. Commanding a hover at the vehicle's position did NOTHING because
the guard accepts a target only if it is within `radius_max` (300 m) of HOME **or** `radius_local`
(100 m) of the VEHICLE -- and at 34 m/s the vehicle outran the 100 m local ball between reading
its position and the command taking effect. It reached 11 m altitude before `radius_max`/
`radius_local` were raised, after which it recovered instantly.
**`radius_local` MUST scale with speed** (100 m is 3 s of flight at cruise) -- or the recovery
target must be placed AHEAD of the vehicle, not where it was. This is the SECOND time this guard
has blocked the one controller whose job is to save the aircraft (the first was the home-relative
check after 1430 m of cumulative flight). A safety fallback that can refuse is not a fallback.
**Also: controllers RETAIN their last reference across a PX4/Isaac restart** -- always command a
local hover BEFORE relaunching, or the vehicle bolts for a stale target the moment the link comes up.

### CAMPAIGN CLOSE-OUT: what damps the phugoid, and four things that DO NOT (2026-09-07)
**STATE:** 90 deg wing-borne flight is REPEATABLE. full_tr28 (w_vz=2.5 build, flight-20 params):
four episodes of 8-11 s at **tilt 90.0, rotors 8.8 N of 33.8 N weight, wing 195-350%, V 13.7-17.5
avg**. full_tr20 (w_vz=0) gave the LONGEST episode (41.7 s) at lower speed (14.1 avg). The phugoid
(20-40 m altitude swing) is the one remaining defect.

**THE ONE THING THAT TOUCHED IT: `w_vz` -- weight on (vz - vz_ref)^2.**
  * w_vz=0  (full_tr20): 41.7 s episodes, 27 m phugoid, V 14.1
  * w_vz=2.5 (full_tr24/28): 8-11 s episodes, V up to 17.5, still 20-40 m
  * w_vz=6  (full_tr23): phugoid 27 -> **7.6 m**, but the episode collapses after 14.7 s -- holding
    altitude that rigidly stops the vehicle trading height for speed, V decays below the 14.13 m/s
    stall and it drops into an 8 m/s mush.
  The trade is real and bracketed; the sweet spot is between 2.5 and 6 and was not found.

**FOUR THINGS THAT DO NOT WORK (each cost a flight; do not retry as written):**
1. **TECS energy/balance** (full_tr22). B = g*dz - 0.5*d(V^2) is the textbook phugoid term, but the
   ROLLING TRANSITION IS ITSELF a potential->kinetic exchange, so it charges for the corridor
   crossing. Never crossed: tilt 73 vs 90, wing 61% vs 238%.
2. **Longer horizon `DT` 0.20 -> 0.30** (offline gate). 2.4 s vs the ~4 s phugoid is the right idea,
   but RK1 at 0.30 s cannot integrate the vertical dynamics: the offline test drifted z 20 -> 50 m
   and the last solve failed. The correct version is DT=0.25/NRK4=2, which does NOT build inside
   the 15-minute timeout. NOT FLOWN.
3. **Vertical-force tilt clamp** (full_tr17). Credits the wing with its PEAK CL (only true at
   alpha=10 deg) so it is most permissive when the wing under-delivers; and restricting TILT does
   not remove the demand for forward force, it MOVES it -- the solver dove instead. Wing 74->25%.
4. **★ COMMANDING THRUST IN THE REFERENCE** (full_tr25/27). The trim asks only 5-19% throttle at
   ANY speed (drag is tiny; 80% would balance at ~80 m/s), so the vehicle coasts at 12-14 m/s with
   72 N unused. But you cannot fix that by asking for more:
     - demanding more SPEED (`v_lead` 15) -> the solver bought speed with GRAVITY: **0.0 N thrust,
       81 m dive**;
     - overriding `uref` thrust to 28.8 N -> 28.8 N front against 2 N rear is a huge pitch moment
       and a vertical over-thrust, so (xref,uref) stopped being a balanced state. The solver sat in
       FALLBACK for the entire flight (625/625 level samples at hover thrust), lost 84 m.
   **SAME ERROR AS full_tr7: A TRIM IS A PACKAGE.** For a given speed in level flight the trim IS
   the trim -- thrust is not independently commandable. The correct mechanism already exists:
   `so3.trim_split(..., Fx_req=M*a_cmd)` solves the TILT as a third unknown while keeping vertical
   and pitch balance. That is a SCHEDULE change (nmpc6.Schedule.at), not a uref override, and it is
   the recommended next step.

**AIRFRAME FACT worth stating: ONLY THE FRONT PAIR TILTS.** At 90 deg the rear rotors still point
straight up, so the maximum FORWARD thrust is the front pair alone -- 2 x 18 = 36 N = **50% of
installed thrust**. "80% throttle in cruise" is not a thing this airframe can do.

**PROCESS: an unasserted string edit cost a flight.** Four substitutions were applied to the node
and only two carried `assert`; the `accel_thr` parameter declaration silently did not match
(`descent_budget` defaults to 20.0, the patch said 45.0), so the node raised
ParameterNotDeclaredException every loop and the vehicle fell 108 m. **The offline `--test` passed
throughout because it uses the class defaults and never touches the ROS parameter path.** ASSERT
EVERY EDIT, and check the node pane for tracebacks + `ok=1` before every flight -- the offline gate
cannot see this class of bug.

## ★ STANDALONE `vtol_nmpc.py` — TRANSITION SOLVED offline (2026-08-13)
`ros2/px4_vtol_direct_actuator/.../vtol_nmpc.py`: ONE file, ZERO cross-imports (plant + trim table +
NMPC + ROS node + selftest all inlined; trim baked as a TABLE so the AVTOL generators aren't needed).
Built to escape the tangle after 4 failed HIL runs. Controller id `vtol_nmpc`.
**SELFTEST RESULT (vs the same plant, lat=0.25): hover -> 90 deg wing-borne, ok=1 EVERY solve.**
The AIRSPEED TILT CLAMP is visibly doing its job: tilt HELD at 11-12 deg while speed builds 0->10 m/s
(refuses to over-tilt while the wing can't carry), then COMMITS 15->41->62->75->90 as the wing loads.
**Altitude DEAD-FLAT 20.0 m through the whole transition, roll=0, yaw=0**; cruise tilt=90, V=16,
AoA +6.3, Tf=3.7 N / Tr=0.2 N (rotors carry only drag).
**OPEN — sustained cruise wanders after ~10 s:** tilt oscillates 83-89-79-73-87-64, AoA swings
0.7-11.9-2.2-12.5, altitude drifts 20.0->20.8, Tr touches 0/negative. At full unload there is almost
no control authority and 10 Hz + 0.25 s delay-comp is marginal for that lightly-damped mode. Fixes to
try: a rear-rotor FLOOR (keep ~1-2 N for authority, as nmpc6 needed), more w_om, or a higher rate if
the HIL solve budget allows. TRANSITION is the part that was broken and it is now fixed.
The 7 hard-won invariants are documented in the file header (tilt clamp, real-time sizing, never zero
the velocity ref, state bounds, ref-FF fallback, penalise surfaces, ghost publishers).
NOT yet HIL-flown -- fly it on the MINIMAL stack (`scripts/start_vtol_minimal.sh`, verify
`Publisher count: 2`); the last 2 HIL crashes were ghost publishers flipping it before engage.

### vtol_nmpc HIL bags vt1/vt2 — two AMPLIFIER bugs in the failure paths (2026-08-14)
**vt1: the NODE DIED.** Hover was rock solid (z=15.0, roll/pitch/yaw 0, tilt clamp working exactly:
held ~14 deg = trim(V)+12). Then pitch drifted -5 -> -20 and at t=7.3 s it inverted in one step. The
tell: **actuators FROZE byte-identical (`mot=[0.60,0.60,0.81,0.81] tilt=0.28`) for 17 s** while it
fell, and every monitor line after engage was blank -> an uncaught exception killed the loop, PX4
HELD the last command. FIX: the loop body is now wrapped (`_loop()`), logging the traceback ONCE and
falling back to hover. A frozen actuator stream is the worst failure mode; it must be impossible.
**vt2: the FALLBACK amplified an upset into a death spiral.** Every solve returned `ok=0`, and the
solve-failure path applied `urefs[0]` = the CRUISE TRIM, which at speed is **tilt=90**. So while the
vehicle was inverted at AoA +75 it kept commanding full forward tilt -> deeper upset -> next solve
also failed -> unrecoverable (roll +-180, altitude 5.8-53.3, AoA 70-97). FIX: the trim feed-forward
is applied ONLY while the state is near the reference; if `diverged()` (|roll|>50, |pitch|>35, or
R[2,2]<0.4) the fallback is `safe_hover()` (rotors UP, equal thrust, neutral surfaces) -- the only
command recoverable from an upset.
Also: `w_beta` 12 -> 60 (the airspeed TILT CLAMP now owns corridor safety, so the soft tilt guide is
no longer needed; soft w_beta left a flat tilt-vs-pitch optimum -> cruise tilt hunting 43<->89).
**STILL OPEN: why does every solve fail (`ok=0`) once it is moving?** MAX_ITER=25 may be too low
after the w_beta change. That is the entry cause; the two fixes above only stop the amplification.
Ops: /tmp was wiped again -- `scripts/rosenv.sh` + `scripts/stop_bag.sh` now live in the repo. A bag
killed with -9 is UNREADABLE (sqlite I/O error, no metadata.yaml): always SIGINT the recorder.
