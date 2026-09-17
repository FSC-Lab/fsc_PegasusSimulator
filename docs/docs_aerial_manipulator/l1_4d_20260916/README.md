# 4-D vs 6-D attribution for the whole-body L1 observer — 2026-09-16/17

**Question (user):** the working note `disturbance_observer_draft.tex` was rewritten
around a FOUR-dimensional interaction wrench (attribution on the joint rows, Sept 2026)
in place of the six-dimensional metric/projector/identifier design flown in 7.15. Build
the 4-D rig, tune it, and compare it with the 6-D one under the same disturbances and
uncertainties on the hover and step test in free flight; report tracking errors and the
end-effector phantom forces.

**Rig:** `scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_adaptive_4D_direct_actuation_sitl.sh`
+ fsc_autopilot_ros2 `scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh`
+ `config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml`.
Same executable as the 6-D rig; `wb_l1_four_d` is the switch. Plant = the standing
config A (+15 % allocator kf, body mass/inertia ×1.10 + 10/10/5 mm CoM shift, MN4010
rotor lag, current-loop residual, gearbox friction ×1.05, arm mass ×1.05).

| run | design | `omega_x` (t/r/q) | result |
|---|---|---|---|
| `l1_6d_A`, `l1_6d_B` | 6-D (shipped 7.15 yaml) | 20 (wrench reading only) | baseline, reproduces the 09-14 run |
| `l1_4d_4d_wx6` / `_wx2` / `_wx0p5` | 4-D | 6 / 2 / 0.5 scalar | 0.94 Hz j2/j3 ripple 339 / 259 / 196 mN·m |
| `l1_4d_4d_blk` / `_blk_slow` | 4-D | 2/0.5/0.5, 2/0.25/0.25 per block | 167 / 182 — the translational block excites it |
| `l1_4d_4d_wx0p25`, `_best_A`, `_best_B` | **4-D, shipped** | **0.25 scalar** | ripple 5–6 mN·m, best CoM/EE/tilt, phantom 0 |

```bash
docs/docs_aerial_manipulator/l1_4d_20260916/run_4d.sh shiqi_machine            # RUNS="..." picks tags
cd docs/docs_aerial_manipulator/l1_4d_20260916
/usr/bin/python3 summarize_4d.py            # metrics.txt, legs.txt, summary.json
/usr/bin/python3 ripple_4d.py               # ripple.txt: band-passed torque/tilt ripple, |e_psi|, phantom
/usr/bin/python3 traj_errors.py --csv traj_errors.csv   # EVERY tracked state, per leg, peak/rms/settled
/usr/bin/python3 ripple_4d.py --soak        # hover.txt: the 20 s hover only
PYTHONNOUSERSITE=1 /usr/bin/python3 plot_4d.py --six l1_6d_A.npz l1_6d_B.npz --four l1_4d_4d_best_A.npz l1_4d_4d_best_B.npz --out compare_6d_4d.png
```

**Tracking errors are scored PER CHANNEL** (`traj_errors.py`): CoM position and
velocity, platform attitude, EE position, EE heading and joints, each in its own
unit, peak / rms / settled-rms per leg. Do not use the 4-norm of `e_y` as an "EE
error" — it adds three components in metres to `sin(heading error)`, which turned
8.7 deg of heading into "151 mm" of position error in the first version of this
campaign's table. `wb_compare_metrics.py` was fixed on 2026-09-17 to report the two
separately.

Results, mechanism and every table: Command.md §7.17.3–7.17.4. Raw `.npz` stay out of
git; launcher/driver logs in `logs/`; `campaign*.log` are the four back-to-back runs.
