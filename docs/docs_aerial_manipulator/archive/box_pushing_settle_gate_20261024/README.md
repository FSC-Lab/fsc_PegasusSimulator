# 2026-10-24 evening — the hand-over settle and the calm exit, flown in Isaac

Asked 2026-10-24 ("have you tested it in Isaac? at least 1-3 tests") after the changes in
`../wb_box_pushing_flight_20261024/README.md` section 8:
- planner: after Ready the claw stays on the CoM anchor until it is within 25 mm of the grasp point,
  8 mm across the fin and slower than 30 mm/s for 0.3 s, then the room hold
  (`push_pull_handle_settle_*`); Ready is refused until the claw has been slower than 30 mm/s for
  0.5 s at Go To Start's hold (`push_pull_ready_max_speed` / `_settle_s`);
- arm GS: the close gate opens, and its 2.5 s limit starts, at that hand-over (6 s settle limit,
  then ABORT); the automatic Exit waits for the vehicle slower than 40 mm/s for 0.3 s (at most 3 s).

## Setup

The 10-24 morning test's condition exactly (`../box_pushing_guard_gate_20261024/`): hardware-like
drift (force noise 0.32 N per axis at 0.12 Hz), hardware tilt guard 10° and abort hold 1.5 s, raw
mocap at 120 Hz, the arm GS running Ready / the close / Push / Exit (`--gs-auto`), the shipped box
yaml (q3 floor 10°). Paired seeds: 2 and 5 (the morning's two "claw arrived 52–89 mm off" failures)
and 1 (a morning completion, the regression check). Seeds 1, 2: 240 g / 0.10 m; 5: 500 g / 0.50 m.

Tools: copies of the morning's `runner.sh` / `fly.sh` / `run_pl.sh` / `pl_mission.py` /
`phase_metrics.py`, `compare.py` (now this morning vs tonight, `compare.txt`) and the new
`events.py` (hand-over time from the driver's status log + the GS log lines, `events.txt`).

## Results

| seed | 10-24 morning | tonight | Ready refusals | hand-over after Ready | close after hand-over | exit calm wait |
|---|---|---|---|---|---|---|
| 2 | ABORT, left DIRECT | **complete** | 6 | 0.37 s | 0.00 s (9 mm, 16 mm/s) | 0.41 s |
| 5 | q3 rate-guard abort | **complete** | 7 | 0.46 s | 0.00 s (16 mm, 4 mm/s) | 1.12 s |
| 1 | complete | complete | 14 | **6.15 s** | 1.09 s (13 mm, 22 mm/s) | 1.94 s |

- **3 of 3 complete** (morning 1 of 3 on the same seeds). Box travel 99 / 489 / 122 mm, vehicle tilt
  ≤ 3.8°, q3 never below 20.3° (release minimum 20.4–28.1°).
- **The sequence ran as designed every time**: Ready waited for a still claw, the planner handed
  over to the room hold, the station closed at the hand-over, the exit waited for calm.
- **Seed 1 came within 0.15 s of the station's 6 s settle limit.** The claw sat 8–12 mm ACROSS the fin
  (median 10 mm; within 25 mm and slower than 30 mm/s most of the time) and the settle gate's 8 mm
  across tolerance held the hand-over. The CoM anchor cannot remove an across offset — the claw only
  follows the vehicle. In seeds 2 and 5 the across error was under 2 mm. The A/B/C hardware pulls were
  ALONG the arm (28–36 mm), which is what the settle gate exists for; the across error is closed by the
  room hold and then checked by the station's close gate (8 mm hardware / 3 mm Isaac).
- **Ready refusals took 3–7 s** of the station's 10 s automatic-Ready retry budget (the claw swings
  24–54 mm/s at the hold under this drift model). If it runs out, the station stops and asks for the
  press; the vehicle only hovers.
- Box moved 0 / 1 / 10 mm before the close (seed 1's 10 mm during its long settle).

## The across tolerance 8 → 15 mm, and the re-fly (same evening, user's choice)

`push_pull_handle_settle_axis_tol` 0.008 → **0.015** in `make_push_pull_yaml.py` (sim, hardware and
coordinated-admittance box yamls regenerated); both whole-body hardware stacks expect 0.015 (raw dry
run: 60/60 tuned values). The station's close gate keeps 8 mm across on hardware (3 mm in Isaac),
checked after the room hold has pulled the claw across.

Seed 1 re-flown (`sg15_s1`, everything else identical):
- **The grasp sequence is fixed**: 1 Ready refusal (was 14), hand-over **1.19 s** after Ready (was
  6.15 s), the close 0.39 s later with the claw 4 mm off and 1.3 mm across, push and release clean,
  exit calm after 0.31 s.
- **Then the planner's 10° tilt guard aborted the way home** (Go To Land Start, 24.2 s into its 25.4 s,
  24 s after the release). The abort climbed and reported "holding higher"; the node was still in
  DIRECT when the recording ended 4 s later (tilt still up to 12°; nothing recorded after that).
- The cause is the known **sim-only 1.3–1.7 Hz attitude mode** (roll 1.33 Hz, `way_home_tilt.txt`).
  It was present faintly in Go To Start (1.36 Hz, 0.7° std) and grew with the arm parked at home
  (joint p-p 1–3°): tilt peaks 3.7° → 12° over 28 s. Every other way home tonight and this morning
  peaked at 2.5–5.8° (median 1.2–2.4°). The same mode tripped the guard in hover on 10-05 and 10-07; on
  hardware it holds 7–9 % of the attitude-error energy against 23–47 % in Isaac. Nothing changed tonight
  acts after the release (the settle runs between Ready and the close, the calm wait before the exit).

Tally tonight: 4 grasp sequences of 4 clean; 3 missions complete, 1 aborted on the way home by the
sim-only mode.
