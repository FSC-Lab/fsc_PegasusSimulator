# 2026-10-24 — the guard changes and the new close gate, flown in Isaac

Asked 2026-10-24 after the 10-23 hardware abort (analysis:
`../wb_box_pushing_flight_20261023/README.md` sections 9–11). Three changes were tested together:

1. **Planner:** the q3 rate guard credits q2's 45° wall (`push_pull_guard_q2_max_deg` 45).
2. **Box-pushing yamls:** `push_pull_guard_q3_min_deg` 10 → 5. One key sets both the static
   floor and the rate rule's threshold.
3. **Arm GS, Box Pushing tab:** a new close gate.
   - Within 25 mm of the grasp point.
   - Across the fin within 8 mm on hardware, 3 mm in Isaac (the sim claw's grip shim).
   - The claw slower than 30 mm/s over 0.3 s.
   - Gate not met within 2.5 s → ABORT, with the jaws still open.

   The 30 mm/s limit was chosen after checking strictness on both hardware holds
   (`../wb_box_pushing_flight_20261023/close_gate_strictness.txt`): from any moment of the hold
   it opens within 1.2 s. At 25 mm/s it was met only 23 % of the 10-21 hold.

## Setup

The 10-22 hardware-like condition, paired seed for seed with that campaign's shipped
configuration ("free axis from the room hold"):
- `../box_pushing_contact_stability_tune_20261021/runs/` `g_fw_1..3`, `h_fw_4/5`, `z_final_6..8`.
- Force noise 0.32 N per axis at 0.12 Hz, seeds 1–8.
- Seeds 1–3, 7, 8: 240 g / 0.10 m. Seeds 4–6: 500 g / 0.50 m. Push speed 21 mm/s.
- Hardware tilt guard (10°) and abort hold (1.5 s); the nominal layout; raw mocap at 120 Hz.
- The arm GS runs Ready, the close, Push and Exit by itself (`--gs-auto`).

Tools: `tools/runner.sh` + `queue/`, `tools/fly.sh` / `run_pl.sh` / `pl_mission.py` (the 10-22
copies; `run_pl.sh` pins `PEGASUS_PUSH_LAYOUT=nominal`), `tools/compare.py` (`compare.txt`),
`tools/guard_split_replay.py` (`guard_split_replay.txt`). The GS logs are
`~/.ros/log/joint_plot_inverted_*.log`, one per flight.

## Results (`compare.txt`)

| seed | 10-22 baseline | 10-24 | GS close (from DONE ready) |
|---|---|---|---|
| 1 | complete | complete | 0.94 s, claw 9 mm, 13 mm/s |
| 2 | complete | **CRASH** after the GS's wait-limit ABORT | not met: claw 56–89 mm off |
| 3 | complete | complete | 0.96 s, 10 mm, 27 mm/s |
| 4 | complete | complete | 0.89 s, 6 mm, 25 mm/s |
| 5 | complete | q3 rate-guard abort, safe (q3 min 7.7°) | — (claw 52–57 mm off) |
| 6 | complete | complete | 0.10 s, 5 mm, 18 mm/s |
| 7 | q3-guard abort | no push: the jaws closed on nothing | 1.55 s, 20 mm, 10 mm/s |
| 8 | no grasp: closed at **43 mm/s** on nothing | **complete** | 1.02 s, 15 mm, 14 mm/s |

Totals: baseline 6/8 complete; 10-24 5/8 complete, one crash. In both sets every failure starts
the same way: the claw arrives far from the fin, or the open jaws shove the box before the grasp.

- **The close gate does what it should.**
  - Every close came 0.1–1.55 s after Ready, with the claw at 10–27 mm/s.
  - Seed 8, which in the baseline closed mid-sweep at 43 mm/s on nothing, now waited 1.0 s for a
    calm moment and completed.
  - Hardware's 8 mm across tolerance is not exercised here (Isaac keeps 3 mm).
- **Seed 7: a moving box defeats the gate.** Before the close the open jaws shoved the box 70 mm
  along the push. The gate checks the claw against the box's CAPTURED position, so it closed
  20 mm from that point but ~50 mm from the fin. In Isaac a full close is "nothing in the jaws":
  no push, then the driver's timeout abort, safe. On hardware a full close counts as the grasp, so
  the push would start empty and the box-lag guard should catch it.
- **Seed 2 crashed. The cause is the shared 5° threshold**, not the gate (`guard_split_replay.txt`).
  - The descent ended with the claw 56 mm off. The room hold stretched the arm onto q2's 45° stop.
  - q3 then sank SLOWLY, ~10°/s from 16° to 5.5°, while the wrist rolled to −75° and the open jaws
    dragged the box ~40 mm.
  - At a 5° threshold the rate rule predicts ~7° and does not fire, and the static floor is not
    reached. The GS's 2.5 s limit aborted from that wound-up arm and the vehicle tipped.
  - With the rate rule at **10°** it would have fired 1.82 s after Ready, at q3 11.4° (0.7 s
    earlier), as seed 5 did (tripped at 10°, recovered with q3 min 7.7°).
  - The 69-run replay behind the 10 → 5 change contained fast buckles and false trips but no slow
    fold on the wall, so it missed this.

## Recommendation (NOT applied)

1. **For the hardware flight: put `push_pull_guard_q3_min_deg` back to 10** (yaml only). Keep the
   q2-aware rule (it alone clears the 10-23 trip) and the new GS gate.
   - The 5° floor's one gain was a level stretch onto the q2 wall (q3 parks at 7.6°), which has
     not been flown.
   - Alternative: split the key (static 5°, rate 10°). That needs a planner change; on these runs
     it catches the same events as 10/10.
2. **Next, the root cause of every failure in both sets: the arrival.** Don't engage the room hold
   when the claw arrives far off. The GS can ABORT at DONE ready if the claw is > ~40 mm off,
   before the arm winds up (the hardware arrivals were ~15 and ~30 mm). Better: a planner arrival
   gate and a retry. Also: check the box hasn't moved since Get before closing.
