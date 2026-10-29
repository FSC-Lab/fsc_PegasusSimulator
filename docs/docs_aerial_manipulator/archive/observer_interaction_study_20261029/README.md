# Is the contact sway the disturbance observer or the interaction-mode logic? (2026-10-28/29)

Isolated research study. Nothing in the flight stack, the valve-turning version, any yaml,
generator or shared tool was changed. No Isaac, PX4 or ROS 2 was run. Everything here is offline,
on the recorded 10-28 hardware valve flights (`../wb_valve_turning_flight_20261028/npz/`) and on an
offline bench built from the exact law and observer (`controller.py` + `l1_observer.py` through
`wb_entry_sim.Law`).

## Conclusion

**The sway does not come from the disturbance observer, and the interaction-mode logic (the chi
switch, the rendered F_hat_y, the free-axis anchor, the room-held claw) is at most a secondary
contributor. Do not modify the observer or the interaction logic for this.** No redesign is
proposed, so there is no proof note.

The hardware data show where the energy enters (figure `figures/energy_split.png`):

1. The law's own lateral force command damps the sway strongly: −203 mW in the large flight.
2. The base's **roll attitude tracking error** about the nose (5.0° amplitude) tilts the thrust
   sideways in phase with the lateral velocity. That feeds +264 mW, which is more than the
   command removes. The net is +71 mW into the lateral motion.
3. That roll error is what the attitude loop makes of a **~0.28 N·m roll moment at the sway
   frequency**. The observer reads this moment in its unfiltered residual. The loop transfer
   function `1/|k_R − Iω² + j k_w ω|` turns 0.28 N·m at 3.1 rad/s into 4.3° with ~86° of lag. The
   measured value is 5.0°, ~90° behind the lateral displacement.
4. PX4's attitude estimate is not the cause (0.32° error, +11 mW). The observer's consumed outputs
   are small: −d_hat_t +6 mW, and a consumed roll compensation of 0.07 of the 0.28 N·m.

Why the observer cannot be the fix: its rotational channel runs at ω_c,r = 0.74 rad/s, below the
3.1 rad/s sway by design. Raising it past ~1–1.5 rad/s is the instability already measured three
times (7.15.8, 2026-09-09, and the rotor-lag pole).

**Where the roll moment comes from (measured kinematics; the bench did not reproduce the growth,
so this is not a proven mechanism):**

- The claw stays pinned on the fin: its task error across the nose is only 1.6 mm while the base
  sways ±59 mm.
- The arm absorbs that sway with q1. On the flown pose, ±16.4° of q1 moves the claw ±61 mm
  sideways relative to the base, which is the sway itself.
- q4 swings ±18° to hold the claw's heading.
- At the flown fold (q2+q3 = 30°, close to the wrist singularity) that q1/q4 pair still turns the
  claw ~9° about the nose axis relative to the base (forward kinematics,
  `tools/claw_roll_fk.py`).
- The clamped fin resists that roll, so the moment lands on the base as a roll moment that grows
  with the lateral displacement.
- The attitude loop answers it ~90° late, which turns it into lateral negative damping.
- The 4-D task (claw position + heading) has no claw-roll row, so nothing in the law acts on this
  coupling except the soft heading channel.

The arm side follows the same direction but is smaller. The observer-driven share of u3 (the
rendered F_hat_y plus d_int^f) is in phase with the joint velocity on q1 and q4, so it acts as
negative damping there. It injects +7.6 / +14.4 mW, out of the law's total +39 / +27 mW on those
joints. That share is real but secondary.

## Evidence

### A. Hardware energy split (lock-in at the sway frequency)

Windows: 180652 at 41.0–47.5 s, inside the turn, which ended 0.2 s later in the rotor-tilt guard.
181833 at 41.0–44.0 s.

| term (mean power, mW; + feeds the sway) | 180652 (59 mm, 0.49 Hz) | 181833 (14 mm, 0.63 Hz) |
|---|---|---|
| base lateral force **delivered** (u1 × true tilt) | **+71.1** | **+16.0** |
| = law COMMAND (u1 × commanded tilt) | −203.3 | −20.5 |
| + roll TRACKING error (−u1 e_R,nose) | **+263.7** (e_R 5.0°) | **+37.0** (e_R 2.2°) |
| + PX4 attitude ESTIMATE error | +10.7 (0.32°) | −0.5 (0.34°) |
| observer −d_hat_t consumed (part of the command) | +6.1 | +0.5 |
| q1 law joint torque (all terms) | +39.0 | +8.2 |
| of which observer-driven (rendered F_hat_y + d_int^f) | +7.6 | +1.6 |
| q4 law joint torque (all terms) | +27.1 | +9.1 |
| of which observer-driven | +14.4 | +2.3 |

Roll at the sway, 180652:
- The unfiltered roll residual is 0.28 N·m. The consumed d_hat_r is 0.07 N·m.
- The law's roll torque is 0.47 N·m.
- The base rolls 3.1° (true), and its tracking error is 5.0°.

Joints over the same window:
- q1 and q4 swing sinusoidally: the fundamental carries 85–94 % of the variance, and they are stuck
  only 4–7 % of the time. They are not stick-slipping.
- q2 and q3 are stuck 50–64 % of the time.

### B. The offline bench cannot reproduce the mode

The bench (`tools/valve_bench.py`) holds the claw clamped on a valve fin whose rotor has the
measured bearing friction. Everything else is the shipped valve configuration:
- the exact law and the 4-D L1 observer;
- the chi switch and the room hold;
- the free-axis anchor at 0.75;
- the mirror plant, with rotor lag, a 16 ms motor delay and arm friction;
- hardware-like raw-mocap feedback, plus the 0.32 N drift force.

Two seeds per case. Lateral rms is given per seed:

| round | what was varied | lateral rms, mm | growing 0.5 Hz mode? |
|---|---|---|---|
| 1 | as flown (V0) | 24.1 / 29.1 | no (dominant 0.11–0.23 Hz, growth ≈ 0) |
| 1 | contact flag OFF (no chi, no F_hat_y rendering) | 23.4 / 27.9 | no, same as V0 |
| 1 | no drift force | 17.6 / 18.1 | decays |
| 1 | arm delay 20 / 40 ms, mocap 40 ms, stiction ×2, PX4-like tilt error ±0.25·a/g, soft grip | ≈ V0 | no |
| 1 | F_hat_y noise 1.0 / 0.3 N | 24.4 / 35.9; q3 goes negative | no (worse, but in the drift band) |
| 2 | j1/j4 net negative damping 0.1–0.3 N·m·s/rad | aborts, **also in free flight** | not contact-only |
| 2 | j1/j4 relay 0.02 / 0.05 N·m | 16.6 / 16.2 | no |
| 3 | small negative damping 0.02–0.08, free vs clamped | 11.5 / 16–17 | no |
| 4 | rotationally stiff grasp, k = 1–30 N·m/rad about the fin axes | 14–21; arm to stops at 30 | no |
| 4 | stiff grasp, contact flag OFF | 20.7 / 17.2 | no, same as ON |

What the bench rules out:
- a law, observer or chi-logic instability against a clamped valve;
- the modelled delays;
- PX4-like acceleration tilt error, of either sign;
- stiction;
- F_hat_y noise;
- small arm energy injection.

What it leaves open is outside the bench:
- the real gripper and fin compliance about the claw-roll axis;
- the real arm servo response on q1/q4: the duty and back-EMF feed-forward on the observer
  velocity, the friction relays, the j4 error damping;
- rotor and ESC dynamics.

Chi on/off never mattered in any bench case.

`tools/wrench_fit.py` tried to fit the hardware residual to a claw wrench. It is **inconclusive**:
the unfiltered hardware residual is too noisy in the translational rows. It is kept only so nobody
repeats it.

## Recommendation

Do not change the observer or the interaction logic for the sway. On hardware, cheapest first:

1. **No flight.** With the jaws closed on the fin, twist the claw by hand about the tangential axis
   (the claw's own axis). If the pads resist the twist, the roll coupling described above is
   present.
2. **Clamped hold without a turn, `push_pull_contact` true vs false** (chi off in contact). If the
   sway persists with the flag false, the interaction logic is cleared on hardware directly. The
   bench predicts no difference.
3. **If it persists:** try a handle the claw can roll about, such as a round pin along the
   tangential axis gripped with clearance. That removes the claw-roll constraint, and with it the
   roll moment. Changing the observer will not fix it.
4. **Optional, offline:** run `tools/energy_flow2.py` on the 10-24 evening and 10-27 box-push bags.
   Their 0.47 Hz along-push oscillation may be the same chain about the pitch axis.

## Files

- `figures/energy_split.png`: the power split above.
- `tools/energy_flow.py`: lateral and arm power terms at the sway frequency, from the hardware npz.
- `tools/energy_flow2.py`: the delivered lateral force split into command, roll tracking error and
  estimate error, plus the roll moment balance.
- `tools/wrench_fit.py`: the claw-wrench fit of the residual (inconclusive, see above).
- `tools/valve_bench.py`: the clamped-valve bench, with options for delays, stiction, attitude
  error, F_hat_y noise, arm energy injection and a 3-axis grasp stiffness (`grip_rot`).
- `tools/smoke.py`, `tools/round3.py`, `tools/round4.py`: the bench rounds.
- `round1.txt` … `round4.txt`: their outputs.
- `tools/claw_roll_fk.py`: forward kinematics of the flown q1/q4 swing (the claw's ~9° nose-axis turn).
- `tools/fig_energy.py`: draws the figure.

Run everything with
`PYTHONNOUSERSITE=1 nice -n 19 taskset -c 16-31 /usr/bin/python3 tools/<script>.py`.
The hardware tools need only `../wb_valve_turning_flight_20261028/npz/`.
