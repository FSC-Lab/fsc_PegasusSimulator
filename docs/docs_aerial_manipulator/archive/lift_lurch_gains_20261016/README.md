# Lift / release lurch vs. rotation-observer bandwidth and attitude gains (2026-10-16)

Question (user, after the 1015 pick-and-drop analysis): can a faster rotational observer
(`wb_l1_omega_c_r`) or stiffer attitude gains (`wb_k_r` / `wb_k_w`) shrink the airframe lurch when
the 190 g basket is lifted and released? The payload stays model uncertainty (no feed-forward).

Setup: whole-body 4-D L1 rig, Isaac headless at RTF 1, raw mocap, `WB_SIM_PROFILE=pick_and_drop`,
`PNP_PAYLOAD_MASS=0.190`, the full pick-and-drop mission driven by
`pick_place_controllers_20261003/tools/pnp_mission_v2.py --drop-place --grip-via-action --grip-axis-tol 0.003`.
Variants are generated from the sim `_sim_pick_and_drop.yaml` by `tools/make_variants.py`. Each one
changes only the named keys, plus `pick_place_drop_auto: false` in all of them, so the driver times every
release the same way. Two flights per variant, A1 B1 C1 D1 A2 B2 C2 D2 (`tools/run_lurch_gains.sh`).

| variant | change | completed | lift lurch (mm, deg p-p) | release lurch (mm, deg p-p) | 1.5 Hz pitch (deg rms) | peak tilt | basket in tray |
|---|---|---|---|---|---|---|---|
| A (shipped) | omega_c_r 0.7428, k_R/k_w 1.6/1.2 | 2/2 | 127 / 124 (6.3) | 160 / 153 (11.5) | 0.27 / 0.27 | 6.4 / 7.0 | 46 / 45 mm |
| B | omega_c_r 1.0 | 2/2 | 108 / 135 (5.7) | 141 / 160 (14.6) | 0.26 / 0.45 | 8.6 / 9.1 | 31 / 41 mm |
| C | omega_c_r 1.25 | **1/2** | – / 101 | – / 136 | 0.25 / 0.41 | **33.4** / 7.9 | – / 28 mm |
| D | k_R/k_w 2.134/1.567 (free-flight H1b) | 2/2 | **87 / 104 (5.1)** | **127 / 137 (11.2)** | 0.29 / **0.46** | 7.4 / 7.8 | **18 / 17 mm**, flat |

Lurch = peak horizontal CoM tracking error (the law's own x_c − x_cd) in the 6 s after the lift
(`exit_pick` start) and after the gripper opens (the window includes the exit, which starts 1.74 s
after the open; all variants fly the same plan). The vertical part is 15–21 mm in every run. The
1.5 Hz band is pitch rms over the whole DIRECT phase. Scorer: `tools/lurch_score.py` (output in
`runs/lurch_score.txt`).

Findings
- The repeat scatter is small: A1/A2 agree within 3–7 mm.
- **C (1.25) is past the stability limit.** C1 grew a 1.38 Hz roll oscillation from DIRECT entry,
  1 → 23° in ~20 s while hovering over the pick (no payload yet), and left DIRECT. C2 completed.
  It is the rig's ~1.5 Hz roll/pitch mode on the rotor-lag pole, the same family as the 09-09
  omega_c_r 2.0 abort.
- **B (1.0) gives no consistent gain** (lift 108 then 135 mm), and B2 raised the 1.5 Hz band.
- **D gives the most consistent reduction**: lift −25 %, release −15 %, basket closest to the tray
  centre and flat in both runs. Cost: D2's 1.5 Hz band rose to 0.46° rms. That mode is why
  pick-and-place was softened to 1.6/1.2 on 10-02 (it grew to 8–12° in 2 of 8 hovers there). Two
  flights cannot clear that risk, but these are the gains the hardware already flies in free flight
  (circle / figure-8).
- None of these changes the lurch by more than ~25 %. The sim lift lurch (95–126 mm) is about half
  the 1015 hardware value (241 mm). The sim has no j4 back-EMF over-compensation, so it does not
  show the hardware's loaded wrist-roll swing.

## D repeated: D3–D6 (user request, same day)

Four more D flights, so six in all against A's two. Scores in `runs/lurch_score_D.txt` and
`runs/wobble_D.txt`. Wobble tool: `tools/wobble_decay.py` (the 1.2–1.8 Hz pitch band per 1 s after the
release, in the two quiet hovers, and over the carry from 4 s after the lift to the release). Ranking
tool: `tools/wobble_where.py` (worst 5 s windows, named by mission step).

| | A (2 flights) | D (6 flights) |
|---|---|---|
| completed | 2/2 | **6/6**, no guard, no abort |
| lift lurch | 124–127 mm | **87–104 mm, mean 93 (−26 %)** |
| release lurch | 153–160 mm | **121–137 mm, mean 127 (−19 %)** |
| basket in the tray | 45–46 mm off, once 7° tilted | **17–42 mm off, all six flat** |
| peak tilt | 6.4–7.0° | 5.3–7.8° |
| 1.5 Hz ring after the open, first 2 s | 1.3–1.6° rms | 0.8–2.3° rms, gone by ~4 s in all six |
| 1.5 Hz over the carry | 0.17–0.19° rms | 0.20–0.44° rms (D2 0.44, D3 0.34, D5 0.30) |
| 1.5 Hz at the Drop Start hover | 0.06–0.07° rms | 0.05–0.49° rms (D2 0.49, D3 0.44) |

- **The worst 1.5 Hz window is the release in every flight of every variant.** Opening the gripper
  kicks the pitch mode. It decays in 3–4 s under both A and D. D rings bigger and up to ~1 s longer.
- **D does not grow the mode, but it damps it less with the basket on.** In 2 of 6 flights (D2, D3) the
  wobble left by the lift did not die out during the 25 s carry. It sat at 0.4–0.5° rms (about ±0.7°)
  through the Drop Start hover. Under A it fades to ~0.1° within 4 s of the lift. No D flight came
  near the 15° guard.
- **The lurch reduction and the flat drops repeat in all six flights.**

## Applied 2026-10-16 (user decision): D in the PICK-AND-DROP, simulation and hardware

The pick-and-drop yamls no longer override the attitude pair. They fly the free-flight (H1b) wb_k_r /
wb_k_w 2.134 / 1.567, exactly variant D. The pick-and-place yamls keep 1.6 / 1.2 (the paper's
comparison was flown with it). Changed:
- `pick_place_tune_20261001/tools/make_pick_place_yaml.py`: the gain override is per profile
  (`PROFILES[...]["gains"]`): pick_and_place {1.6, 1.2}, pick_and_drop {}. Regenerated
  `..._t650_sim_pick_and_drop.yaml`; the diff is exactly the two values plus the header line.
  `_sim_pick_and_place.yaml` is byte-identical (`--check` current).
- `pick_place_top_hat_20261008/tools/make_hw_pick_and_place_yamls.py`: `wb_drop` overrides {} plus a
  header note. Regenerated `..._t650_pick_and_drop.yaml` (hardware); the diff is exactly the two values
  plus the header. The whole-body and geometric `_pick_and_place.yaml` are byte-identical.
- fsc_autopilot_ros2 `scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator{,_fused}.sh`:
  the tuned-law check expects 1.6 / 1.2 only for `AM_HW_PROFILE=pick_and_place`. pick_and_drop and
  free flight expect 2.134 / 1.567. Run standalone on both hardware task yamls: 28 of 28 OK each.
- Nothing to rebuild: the gains are yaml values the node already reads. Not flown on hardware.
