#!/usr/bin/env bash
# Rebuild the published two-part page (report.html) and the earlier long-form
# page (report_detailed.html) from the analysis/ outputs.
#   report.html          <- tools/build_polished.py, reading
#                           analysis/states_{a2_r6,f18_r4}.json   (tools/state_rmse.py, below)
#                           ../circle_tune_20260927/analysis/circle_states.json (its tools/circle_states.py)
#   report_detailed.html <- tools/build_report.py (the 2026-09-26/27 long form)
set -e
cd "$(dirname "$0")/.."
if [[ "${1:-}" == "--recompute" ]]; then
  /usr/bin/python3 tools/state_rmse.py data/a2.npz data/sim_a2_r6.npz a2_r6 --rigid leg2_move=leg2_move,leg2_settle
  /usr/bin/python3 tools/state_rmse.py data/f18.npz data/sim_f18_r4.npz f18_r4 --exclude "leg2_settle=flight mocap dropout 44.6-46.9 s"
  /usr/bin/python3 ../circle_tune_20260927/tools/circle_states.py
fi
/usr/bin/python3 tools/build_polished.py report.html
/usr/bin/python3 tools/build_report.py report_detailed.html \
  "f18=0918 mission — DIRECT hold, arm out, home, +0.61 m base step, arm out, home (raw-mocap stack, replay f18_r4)=analysis/compare_f18_r4.json" \
  "a2=0924 F2 — 0.5 m EE circle, one lap at time scale 1, fold 60°, q2 30 ± 10° (EKF2-fused stack, replay a2_r6)=analysis/compare_a2_r6.json"
