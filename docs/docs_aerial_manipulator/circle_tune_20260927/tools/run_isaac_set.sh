#!/usr/bin/env bash
# Isaac confirmation flights for the 2026-09-27 circle tuning, one at a time,
# through sim2real_tuning_20260926/tools/replay.sh (a2 = the EE-circle replay:
# fused stack, s = RTF, wall-clock lags, yaml restored on exit), then extract +
# score_run.py.
#
#   tools/run_isaac_set.sh <spec-file>
#
# spec-file: one flight per line, fields separated by '|':
#   profile | tag | law gains "wb_key=v,..." (empty = shipped) | planner "key=v,..." | s_max
# e.g.  mirror|ship_show|  |ee_traj_circle_radius=0.75,ee_traj_ccw=false,...|1.1312
set -uo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
S2R="$HERE/../sim2real_tuning_20260926"
EX="$HERE/../wb_l1_4d_flight_20260924/tools/extract_bag.py"
SPEC="${1:?spec file}"
STAMP="$(date +%H%M)"
NPZ=()
while IFS='|' read -r prof tag gains planner smax; do
  prof="$(echo "$prof" | xargs)"; tag="$(echo "$tag" | xargs)"; [[ -z "$prof" || "$prof" == \#* ]] && continue
  gains="$(echo "$gains" | xargs)"; planner="$(echo "$planner" | xargs)"; smax="$(echo "$smax" | xargs)"
  T="${tag}_${STAMP}"
  if pgrep -f "[e]e_trajectory_sim_driver" >/dev/null; then echo "[isaac-set] a driver is still running -- stop"; exit 1; fi
  echo "[isaac-set] $(date +%T) $T: $prof | gains [$gains] | planner [$planner] | s_max ${smax:-1.1312}"
  WB_REPLAY_PROFILE="$prof" WB_REPLAY_GAINS="$gains" WB_REPLAY_PLANNER="$planner" WB_REPLAY_SMAX="${smax:-1.1312}" \
    "$S2R/tools/replay.sh" a2 "$T" > "$HERE/logs/replay_${T}.log" 2>&1
  echo "[isaac-set] $(date +%T) $T: $(grep -m1 'driver rc=' "$HERE/logs/replay_${T}.log")"
  sleep 20
  (set +u; source /opt/ros/humble/setup.bash; source "$HOME/ros2_ws/install/setup.bash"; set -u
   /usr/bin/python3 "$EX" "$S2R/bags/$T" "$HERE/data/isaac_${T}.npz" > "$HERE/logs/extract_${T}.log" 2>&1) \
    && echo "[isaac-set] $T extracted" || echo "[isaac-set] $T EXTRACT FAILED"
  NPZ+=("$HERE/data/isaac_${T}.npz")
done < "$SPEC"
/usr/bin/python3 "$S2R/tools/score_run.py" "${NPZ[@]}" --json "$HERE/analysis/isaac_${STAMP}.json"
echo "[isaac-set] done $(date +%T)"
