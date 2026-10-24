# Valve turning on the bench-measured bearing (2026-10-23)

The user asked for this after the free-spin bench analysis (`../valve_bearing_spin_20261023/`). The
final valve configuration of 10-22 was flown on the bearing friction fitted from the bench bag. It
uses the free-axis anchor (0.75, on_world) and the 10-22 planner fixes, and its yaml is
byte-identical to the one the 10-22 FINAL runs flew. The flights are paired with the 10-22 FINAL
runs at the scene default.

**A stress case.** The scene default (0.048 N·m, the user's conservative setting) is unchanged.
Every run sets the bearing through the environment, recorded in `runs/<tag>.args`. Isaac printed
the applied values: `bearing damping 0.00077 N.m.s/rad, Coulomb friction 0.0065 N.m`, rotor 204 g.

| case | `PEGASUS_VALVE_FRICTION` | `_DAMPING` | torque at the turn's 24 deg/s |
|---|---|---|---|
| bench (the fitted law: 2.52 rad/s² + 0.30/s·ω, × I 0.002569 kg·m²) | 0.0065 N·m | 0.00077 N·m·s/rad | 6.8 mN·m |
| low (the 95 % floor at a 170 g rotor) | 0.004 N·m | 0.00077 N·m·s/rad | 4.3 mN·m |
| 10-22 reference (scene default) | 0.048 N·m | 0 | 48 mN·m |

Rig: whole-body 4-D L1, raw mocap at 120 Hz, headless at RTF 1, and the arm GS's Valve Turning tab
flying the steps (`--gs-auto`). "drift" is the hardware-like drift model
`PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,<seed>`. The harness is `tools/chain.sh`, which calls
`valve_turning_20261015/tools/try_valve.sh`.

## Results (`tools/table.py`, `analysis_table.txt`)

| run | bearing | drift | outcome | beam | rotor pushed before the close [deg] | close along the fin [mm] | q3 min | on a stop | peak tilt |
|---|---|---|---|---|---|---|---|---|---|
| vb_nom_1 | bench | -- | ok | +355 | 0.0 | 0.1 | 17.5 | 0 % | 6.6 |
| vb_nom_2 | bench | -- | ok | +356 | 0.0 | 0.1 | 19.1 | 0 % | 5.7 |
| vb_nom_3 | bench | -- | ok | +357 | -0.1 | -4.1 | 7.1 | 0 % | 5.8 |
| vb_low_nom_1 | low | -- | ok | +361 | 0.0 | 1.6 | 20.3 | 0 % | 5.6 |
| **vb_hw_2** | bench | s2 | **no close**: 20 s timeout, abort clean | -- | **18** | (fin 30-68 mm away) | -- | -- | 2.6 |
| vb_hw_3 | bench | s3 | ok (the close waited 6.6 s) | +368 | **14.8** | **-20.0** (the fin's end) | 15.8 | 0 % | 6.7 |
| **vb_hw_4** | bench | s4 | **tilt guard** 4.3 s into the turn, clean | +7 | 4.6 | -15.8 | 4.2 | 0 % | 7.7 |
| **vb_low_hw_3** | low | s3 | **q3 guard** at q3 -12.3 deg, 2/3 through the turn; clean | +235 | **9.9** | -15.5 | **-14.5** | 1.8 % | 5.3 |
| s22_final_nom_1 | 0.048 | -- | ok | +354 | 0.0 | 0.0 | 12.9 | 0 % | 4.5 |
| s22_final_nom_2 | 0.048 | -- | ok | +355 | 0.0 | 9.1 | 3.9 | 4.2 % | 6.3 |
| s22_fa0_nom_1 | 0.048 | -- | ok | +351 | 0.0 | 6.0 | 10.5 | 0 % | 4.5 |
| s22_final_hw_2 | 0.048 | s2 | ok | +365 | 6.9 | -15.7 | 13.8 | 0 % | 7.3 |
| s22_final_hw_3 | 0.048 | s3 | ok | +369 | 2.3 | -4.0 | 4.1 | 0 % | 4.4 |
| s22_final_hw_4 | 0.048 | s4 | ok | +364 | 0.4 | 13.8 | 13.7 | 0 % | 4.3 |

**Nominal: no difference** (4/4 complete, against 3/3 at 0.048). The light bearing is, if anything,
kinder to the arm: q3 never went below 7°, and no run touched a stop.

**Under drift: 1 of 4 complete, against 3 of 3 at 0.048 on the same seeds.** That is not
significant on four flights (Fisher p ≈ 0.14). Two mechanisms show directly in the data, though:

1. **The open jaws push the free rotor away before the grasp.** With the vehicle drifting, a finger
   brushes the fin during the Ready descent. At 0.0065 N·m the rotor then decelerates at only
   2.5 rad/s² (at 0.048 it was 18.7), so it coasts about 7x farther. It was turned 4.6–18° before
   the close, against 0.4–6.9° at 0.048. The planner's live-handle close gate did its job every
   time:
   - it never closed on a fin 30–68 mm away (vb_hw_2);
   - otherwise it waited until the claw drifted back, and closed 15–20 mm along a fin whose
     half-length is 20 mm.

   The claw's target does NOT follow the moved fin, so a large push ends in the 20 s timeout.
2. **The q1/q4 (arm yaw / wrist roll) ~1 Hz antiphase mode is no longer damped.** In vb_hw_4, q1
   swung 1 → 13 → 7 → 20 → 6 → 18° with q4 opposite, growing from the first second of the turn
   until the 10° tilt guard. On the same seed at 0.048 it stayed at ±5–10° and decayed. This is the
   mode the 10-18 study saw grow on a frictionless bearing. vb_low_hw_3 also started off-centre
   (−15.5 mm). Its arm then folded onto q3's +50° stop at +14 s, and at +22 s crossed q3 = 0 to
   −14.5° before the static q3 guard (0.2 s persistence) aborted.

**Every failure ended safely**: DIRECT kept, peak true tilt ≤ 7.7° after the abort, landed upright.
But vb_low_hw_3 put the arm on the elbow-singular branch, the state that crashed the vehicle in
earlier campaigns.

## What it means

The 0.048 default was conservative for the force the claw needs. It was not conservative for
stability: it supplied damping and grasp tolerance that a ~7 mN·m bearing does not.

Options, none applied:
- **Hardware first**: add friction to the valve shaft, for example an O-ring or felt washer under
  the hub, to bring it into the range the final config is validated at. The 10-22 record already
  suggested this below ~5 mN·m. Then re-measure with a hanging weight at the fin.
- **Planner**: re-aim the claw at the live handle before closing, instead of only gating the close
  on it. Also descend with more side clearance to the fin.
- **Law**: damp the q1/q4 mode. It is the observed instability on the light bearing.
- Re-measure the real bearing: weigh the rotor, then the thread-and-weight test at the fin. The
  sim law here rests on the CAD inertia at 204 g.

Not covered: rotor downwash on the real beam (the sim has no aerodynamics, and a ~0.05 N push at
the fin is enough to turn the real valve); repeats beyond one flight per seed.

## Files

`tools/chain.sh` (the 8 flights), `tools/score.py` (the 10-22 scorer on these runs, `--ref` for the
10-22 ones), `tools/table.py` (the table), `runs/` (npz, logs, as-flown yaml, args; the npz are
gitignored).
