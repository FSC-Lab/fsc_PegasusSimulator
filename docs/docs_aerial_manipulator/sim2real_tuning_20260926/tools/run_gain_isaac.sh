#!/usr/bin/env bash
# Isaac confirmation of the offline gain study (2026-09-26): the 0924 F2 circle
# on the MIRROR plant and on the ROBUSTNESS plant, shipped hardware law vs the
# candidate, one flight at a time, then extract + score.
#
#   tools/run_gain_isaac.sh [candidate gains]      default "wb_k_w=0.9,wb_l1_omega_c_t=6.0"
#
# Every flight goes through replay.sh (wall-clock lag rescaling, yaml restored
# on exit); the robustness flights overlay the hardware law on the stress plant.
set -uo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CAND="${1:-wb_k_w=0.9,wb_l1_omega_c_t=6.0}"
EX="$HERE/../wb_l1_4d_flight_20260924/tools/extract_bag.py"
STAMP="$(date +%H%M)"
run() {   # profile tag gains
  local prof="$1" tag="$2" gains="$3"
  if pgrep -f "[e]e_trajectory_sim_driver" >/dev/null; then
    echo "[gain-isaac] a driver is still running -- refusing to start $tag"; return 1
  fi
  echo "[gain-isaac] $(date +%T) $tag: profile $prof gains [${gains}]"
  WB_REPLAY_PROFILE="$prof" WB_REPLAY_GAINS="$gains" \
    "$HERE/tools/replay.sh" a2 "$tag" > "$HERE/logs/replay_${tag}.log" 2>&1
  echo "[gain-isaac] $(date +%T) $tag: replay rc=$? ($(grep -m1 'driver rc=' "$HERE/logs/replay_${tag}.log"))"
  sleep 20
  (set +u; source /opt/ros/humble/setup.bash; source "$HOME/ros2_ws/install/setup.bash"; set -u
   /usr/bin/python3 "$EX" "$HERE/bags/$tag" "$HERE/data/sim_${tag}.npz" > "$HERE/logs/extract_${tag}.log" 2>&1) \
    && echo "[gain-isaac] $tag: extracted" || echo "[gain-isaac] $tag: EXTRACT FAILED"
}
TAGS=()
for spec in "mirror:g_mir_hw:" "mirror:g_mir_cand:$CAND" "robustness:g_rob_hw:" "robustness:g_rob_cand:$CAND"; do
  IFS=: read -r prof tag gains <<<"$spec"
  tag="${tag}_${STAMP}"
  run "$prof" "$tag" "$gains"; TAGS+=("$HERE/data/sim_${tag}.npz")
done
/usr/bin/python3 "$HERE/tools/score_run.py" "${TAGS[@]}" --json "$HERE/analysis/gain_isaac_${STAMP}.json"
echo "[gain-isaac] done $(date +%T)"
