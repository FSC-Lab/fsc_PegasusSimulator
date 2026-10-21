# Box pushing: a light box, and the box going over the table's SIDE edge while clamped (2026-10-19)

User questions, in order: (1) is the case "the box is pushed off the table's side edge while
the jaws still hold the fin" dangerous, and does it need two guide rails along the table's
long edges? (2) is the real box emptied to ~240 g a safer first hardware test than 500 g?
(3) "test the robustness of this worst case".

Setup = the 10-16/10-17 final box-pushing configuration (whole-body 4-D L1, raw mocap, the
CAD box with its 5 mm fin + the sim's 20 mm grip shim, T650 gear, q2 stops [-20, 45], push
pose [0, 32, 28, 0], 0.50 m in 24 s min-snap, mu 0.25 / 0.25, box start y 0.28 on the
table's centre line, the 10-17 release sequence), with the HARDWARE guard values: planner
tilt guard 10 deg, abort release hold 1.5 s (the sim yaml has 15 / 1.0). Headless, RTF 1.

## The side-edge fall, how it is injected

The table is 150 mm deep and the box 160 mm, so the box overhangs 5 mm on each side and its
CoM is over the edge after 75 mm of sideways travel. Nothing in normal flight gets near
that (whole-body sideways drift <= 7 mm in every completed push since 10-16). To force it,
the box is placed at **15 deg yaw** (`PEGASUS_PUSH_BOX_YAW_DEG=15`) and the planner's
plan-time on-table check is switched off (`push_pull_box_half_length 0`): the push follows
the box's yaw, so the clamped claw drags the box diagonally and its CoM crosses the +x side
edge ~15 s into the push (0.29 m of travel). The cause does not matter for the consequence:
this is any sideways drift of a clamped claw that reaches the edge.

## Runs

`tools/chain.sh`, `chain2.sh`, `chain3.sh` (fly.sh = a copy of the 10-16 campaign's);
scored with `tools/sidefall_score.py` (guard, CoM vs the edge at the trip, PhysX contact
truth: how much of the box's weight the claw carried, jaws, vehicle, arm, where the box
ended) and `tools/table.py` (the 10-16 scorer). `runs/base500_*` link the 10-17 evening
results runs (500 g, same setup, the sim guard values) as the 500 g baseline.

### Side-edge fall, box guards ON (the configuration that will fly)

| run | box | guard (first) | CoM past the edge at the trip | claw carried (of the weight) | jaws open after | vehicle peak tilt | outcome |
|---|---|---|---|---|---|---|---|
| sf240_1 | 240 g | box tilt 2.3 deg | 7 mm (0.9 s after crossing) | 0.26 of 2.35 N | 0.32 s | 3.1 deg | abort, box fell to the floor later |
| sf240_2 | 240 g | box tilt 2.5 deg | 13 mm (1.5 s) | 0.06 of 2.35 N | 0.24 s | 3.3 deg | same |
| sf500_1 | 500 g | box tilt 3.5 deg | 8 mm (0.7 s) | 0.05 of 4.91 N | 0.40 s | 2.5 deg | same |
| sf500_2 | 500 g | box tilt 3.9 deg | 9 mm (0.9 s) | 0.01 of 4.91 N | 0.57 s | 2.8 deg | same |

4 / 4: the box-tilt guard (2 deg / 0.2 s) catches the tip at its ONSET, before any weight
reaches the claw; the vehicle never notices; mass makes no difference. q3 touched its
+50 deg stop for a moment after the open in 3 / 4 (the release, the folding direction --
not the singular one). After the release the box rested on the table edge, leaning on the
OPEN fingers, and fell to the floor when the abort's climb lifted the claw away (~6 s).

### Side-edge fall, box guards OFF (what is left if the box guard missed)

`push_pull_guard_box_lag 0`, `push_pull_guard_box_tilt_deg 0` (the box-feed staleness
check is off with them). Left: the vehicle-tilt guard (10 deg), the claw-force guard (5 N),
the q3 guard (10 deg), the flight node's watchdog.

- **sf240_ng1 (240 g): CRASH.** The CoM crossed the edge at 14.8 s and the box did not fall:
  it leaned 6-7 deg on the clamped claw for ~9 s while the push went on (the claw carried
  <= 0.35 N). Near the push's end the box rocked (its centre rose 18 mm) and, rotating
  about the table edge, dragged the claw down and round: q2 onto +45, q3 through 0 at
  ~60 deg/s to -43 deg (the elbow-singular branch). The q3 guard tripped at **-2 deg** (its
  0.2 s persistence is too slow for that rate -- the 10-17 results crash did the same),
  the jaws opened 0.57 s later, the node's watchdog took SAFETY at 17 deg tilt, the box fell
  to the floor -- and **3.5 s later, in SAFETY with the arm still on the wrong branch
  (q3 -30..-40), the vehicle flipped** (176 deg). Arm torques reached the 3.0 N.m sim clamp
  on all four joints (the hardware caps are lower: 0.50 / 2.81 / 1.92 / 0.39).
- **sf500_ng1 (500 g): completed.** The box leaned up to 12 deg on the claw past the edge until
  the release; the arm stayed in range (q3 >= 16.8); the box fell after the claw left.

One run each, so this does not rank the masses -- but the light box is NOT safer here, and
the danger is not the weight (the claw never carried more than 0.8 N in any run): it is the
box ROTATING about the table edge while clamped, which drives the arm into the elbow-singular
branch. The box-tilt guard is the layer that prevents it.

### The light box in a normal push (240 g)

| run | slide e_imp [N rms] | contact force [N] | claw deflection [mm] | rho slide | vehicle tilt | q2 max | q3 range | box sideways / yaw |
|---|---|---|---|---|---|---|---|---|
| base500_1..3 (10-17) | 0.61 / 0.63 / 0.80 | 1.29 / 1.19 / 1.06 | 10.3-10.8 | 0.055-0.067 | 2.8-3.3 | 39.1-42.3 | 14.9..43.9 | 3-5 mm / 7-9 deg |
| l240_1 | 0.82 | 0.55 | 7.5 | 0.063 | 3.2 | 36.9 | 21.1..43.3 | 6 mm / 5.7 deg |
| l240_2b | 0.63 | 0.50 | 5.7 | 0.059 | 2.9 | 40.0 | 17.0..42.4 | 5 mm / 4.1 deg |
| l240_3 | 0.57 | 0.59 | 5.4 | 0.047 | 3.7 | 42.3 | 14.9..39.3 | 4 mm / 5.6 deg |

l240_2 never flew: DIRECT entry refused before take-off (the arm stack did not come up --
"arm joint states stale; arm reference never received"), re-flown as l240_2b. 3 / 3 complete.
Contact force halves, the claw deflects less, the impedance residual does not shrink (the
controller's own floor, ~0.6-0.8 N -- on hardware the force will sit below the ~1.3 N
reading noise, so a 240 g flight is a shakedown, not a result). Box tilt while clamped
(before any edge): 240 g <= 0.67 deg (3 runs), 500 g <= 0.36 except one 2.22 deg spike at the grasp
(base500_3, < 0.2 s, no trip) -- the light box does not rock more in a normal push.

## Conclusions

1. A light box halves the push loads and is a reasonable FIRST test, but it does not make the
   side-edge case safer: what is dangerous there is the box tipping over the edge while
   clamped, and that is independent of the mass.
2. **The box-tilt guard is load-bearing** for the side edge: 4 / 4 caught at the onset with
   the vehicle at <= 3.3 deg; without it 1 of 2 crashed. On hardware it reads obj_0's tilt:
   a lost / frozen box feed trips the box-feed guard (safe), a flipping body trips the tilt
   guard (safe, false abort). Check before the flight that the box body's tilt reads quiet
   (well under 1 deg) with the claw hovering over the fin -- the 10-09 basket body flipped
   near the claw.
3. The q3 guard is too slow as a backstop for a fast buckle (tripped at -2 deg at ~60 deg/s).
   A rate term (q3 < ~20 deg AND falling faster than ~30 deg/s) would catch it earlier --
   not built.
4. Rails: still not required for the first test (the box guard covers the side edge), but
   this is the one failure in the campaign that crashed, and a rail removes it physically.
