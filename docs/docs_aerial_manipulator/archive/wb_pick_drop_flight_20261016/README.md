# 2026-10-16 pick-and-drop hardware flights (T650-AM, whole-body 4-D L1, fused) — first flights with the 10-16 changes

Bags: `docs/experimental_data_ros2_bag/1016 - T650-AM whole-body Pick&Drop-*/`. Report artifact "1016 Pick-and-Drop
Flights": https://claude.ai/artifact/SFA2LczEPpbYLSkyJkN2gD, built from `report.html` by `tools/build_artifact_page.py <out.html>`.

| run | bag | outcome |
|---|---|---|
| E-1 13:44 | `..._empty_20261016_134438` | full mission, no payload (control) |
| D-1 13:51 | `..._20261016_135100` | full mission; gripper stalled at -0.0010 (wrong grip); basket rest 112 mm off, 25 deg |
| D-2 14:00 | `..._20261016_140020` | full mission; released with the claw 59 mm off; basket rest 30 mm off, flat |

Tools (`tools/`, `PYTHONNOUSERSITE=1 /usr/bin/python3`; extract first with `run_extract.sh`): the 10-15 set
(`legs.py`, `arm_terms.py` with the new caps, `carry.py`, `drop_hold.py`) plus `rosout_dump.py`, `drop16.py`
(grasp, hang, drop point = claw target - Rz(yaw) ee_offset - drop height), `grasp16.py`, `caps_audit.py` (new caps
and PWM limits), `health16.py`, `fig16.py`. Outputs in `analysis/`, figures in `figures/`.

Findings:
- Carry pose = pick pose, the j4 fix and the wider caps worked: loaded q4 no longer swings (std 0.0 deg over the
  drop leg; 10-15 +-18 deg at 0.5 Hz), carry pitch 0.21 / 0.50 deg rms (10-15 1.52), caps exceeded 0.02 s total.
- k_R/k_w 2.134/1.567: lift CoM peak 181/157 mm (10-15 241/184); no 1.5 Hz pitch wobble (0.07-0.10 deg rms).
- Automatic drop and the basket aim were running but NOT exercised: Drop clicked 0.48-0.82 s after the leg (inside
  the 2 s wait); Ready To Drop pressed 0.5-0.6 s after Drop Start (aim needs >= 6 s hover). Replayed numbers: D-1's
  hang was 79 mm off the model (aim max 60 -> refused, no auto release), D-2's 28 mm (aimed).
- j2 stick + wind-up under load: D-2 j2 held 1.1 deg low for 18 s while law torque 1.25 -> 2.44 N.m, d_rho j2
  -0.54 -> -1.50, current 190 -> 340 counts; breakaway +12.7 deg, pitch kick 4.7 deg. Worst 10 s mean 321 counts
  (08-26 overload ~250). No servo error.
- Empty wrist still wanders (E-1 -25..+18 deg after 69.9 s, D-1 -45 deg after release), as the 10-16 j4 model predicted.

**2026-10-19 question, Drop Margin 0.20 -> 0.15?** `tools/drop_margin_check.py` (output
`analysis/drop_margin_check.txt`): the basket's lowest point (mocap pose on the CAD's low points) over the plane it stood
on at the cover Get, along Ready To Drop. The leg CLIMBS (Drop Start body 1.0 m -> drop hold 1.39 m), so the tightest
point is the near rim crossing, not the release: at 0.20 the lowest point cleared the cover plane by 107-119 mm (D-1,
wrong grip, basket tilted 19 deg) / 135-145 mm (D-2) with the basket centre 150-250 mm from the drop point; shifted to
0.15 (rest 0.05 lower, Drop Start unchanged, a straight leg lowered by 0.05 * s): 64-75 / 92-101 mm; at the hold 104 /
132 mm. Released 182 / 193 mm above its cover height (hangs lower than the EE-offset model). Pitch over the tray at
the hold (body 1.39 m, rim ~0.89) 0.20-0.36 deg rms 0.3-2 Hz = the carry's: no 10-14-bin effect at 0.49 m; 0.44 m at
0.15 not flown. Assumes the rim ~ the cover plane and the box's near wall within ~0.2 m of the Get point.
