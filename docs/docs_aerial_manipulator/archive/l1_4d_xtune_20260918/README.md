# Along-arm (world-x) settling tune of the whole-body + L1 4-D rig — 2026-09-18

User report the night before the flight test: "the x channel, where the arm heads
forward, takes quite a long time to stabilize." Quantified on the 2026-09-16/17
campaign's two baseline flights (`../l1_4d_20260916/l1_4d_4d_best_{A,B}.npz`, same
driver, 16 s holds) with `step_score.py`: after a 0.5 m step ALONG the arm the CoM
error keeps a ~0.15 Hz, lightly damped oscillation (envelope ratio 0.4-0.6 between
hold seconds 8-12 and 0-4) and is still above 20 mm at the end of the 16 s hold;
the LATERAL step settles under 20 mm in ~5 s (envelope 0.25).

Twelve single-change flights (`run_xtune.sh`, 4-leg step mission, ~5 min each;
`step_scores.txt` has the full table). Nothing in the gain space fixes it:

| change | along-arm (x) | lateral (y) | verdict |
|---|---|---|---|
| baseline (2 flights) | t20 14-16 s, settled 14-28 mm | t20 5-11 s, 3-6 mm | — |
| `wb_l1_omega_x` 0.25 -> 0.1 | 15-16 s, 21-37 mm | 5 s, 2-3 mm | no |
| `wb_l1_omega_c_t` 2 -> 1.0 | **10 s, 7-8 mm** | **16 s, 23-25 mm** | trades x for y |
| `wb_l1_omega_c_t` 2 -> 1.5 | 12-16 s, 14-23 mm | 5-6 s, 3-4 mm | marginal, within scatter |
| `wb_k_r` 2 -> 3, `wb_k_w` 1.5 -> 1.8 | CRASH (36 deg tilt, 4 s after the first step) | | no |
| `wb_k_v` 20 -> 28 | diverging oscillation on every leg | | no |
| `wb_l1_omega_c_r` 0.5 -> 0.25 | 15-16 s, 16-40 mm | 5-10 s | no |
| `wb_k_w` 1.5 -> 1.1 | 16 s, 58-63 mm | worse (27 mm) | no |
| `wb_k_x` 32 -> 24 | 16 s, 19-26 mm | 6 s, 3-4 mm | no |
| `wb_mrd_x` 0.1165 -> 0.16 | 16 s, 30-43 mm | 5 s, 4-6 mm | no |
| `wb_mrd_x` 0.1165 -> 0.19 | reverted to SAFETY 3.8 s after entering DIRECT | | no |

Readings:
- Raising loop gain (attitude k_R/k_w, position k_v) destabilises: the loop is at
  its delay margin (rotor lag 0.1 s + transport), as the 2026-08-23 M_r_d study
  already found. Lowering attitude damping (k_w 1.1) is worse on both axes.
- The only lever that moves the arm-axis mode is the TRANSLATIONAL observer
  bandwidth, and it moves the lateral axis the other way: a world-frame per-axis
  split would not survive a yawing trajectory, so it is not a fix.
- The per-axis effective attitude gain (k_R M_r/M_r_d = 2.27 on the arm axis vs
  1.65 lateral) was equalised with M_r_d_x = 0.16 and did NOT help.
- The offline `wb_entry_sim.py`/`wb_step_sim.py` model tracks the same step with a
  17 mm peak (flight: ~200 mm) and aborts at 40 ms of transport delay; the flown
  plant carries ~1.1-1.4 s of effective closed-loop lag on every trajectory
  (measured on the circle/figure-8 runs in `../wb_vs_decoupled_20260918/`). The
  x/y asymmetry is a property of that lag interacting with the arm-axis coupling,
  and it needs a model change (e.g. the observer predicting the ROTOR-LAGGED
  applied wrench, or a reference lead in the planner), not a gain.

**Shipped config: unchanged** (omega_c_t 2.0). One flight each; the 2026-08-10
scatter lesson applies to every row above.
