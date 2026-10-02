#!/usr/bin/env bash
# Matched circle flights at RTF 1 (2026-10-01): headless, real-time pacer on,
# Isaac pinned to the P-cores (shiqi_machine.conf ISAAC_CPUS), no wall-clock
# rescale (PEGASUS_SIM_RTF=1.0), planner time scale 1.0.
#
#   run_rt1.sh <rig>:<tag> [<rig>:<tag> ...]      e.g. run_rt1.sh wb:rt1b modular:rt1a
#
# After each cycle the Isaac pane (the pacer's RTF readouts) is saved to
# logs/isaac_pane_<rig>_<tag>.txt, since the next cycle's clean slate kills it.
# START_POS_TOL (m, default 0.10) widens the planner Start gate -- the geometric+L1 rig
# settles F/K_p ~ 0.185 m off the start point (2026-10-01) and flies with 0.25.
set -uo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../../.." && pwd)"
export DISPLAY="${DISPLAY:-:1}" PEGASUS_HEADLESS="${PEGASUS_HEADLESS:-1}" PEGASUS_REALTIME=1 PEGASUS_SIM_RTF=1.0
export AM_CMP_OUT="$HERE/data"
mkdir -p "$HERE/logs"
for spec in "$@"; do
  rig="${spec%%:*}"; tag="${spec#*:}"
  echo "=== $rig $tag (headless=$PEGASUS_HEADLESS) $(date +%T) ==="
  "$PEG/application/robotic_arm/utils/am_compare_cycle.sh" "$rig" "$tag" shiqi_machine -- \
    --shape circle --radius 0.5 --lap-time 24 --laps 1 --q2-period 6 --fold-deg 55 \
    --q2-center-deg 25 --q2-amp-deg 15 --time-scale 1.0 --start-pos-tol "${START_POS_TOL:-0.10}" --gate-speed 0.10 \
    > "$HERE/logs/cycle_${rig}_${tag}.log" 2>&1
  echo "rc=$? $(tail -2 "$HERE/logs/cycle_${rig}_${tag}.log" | head -1)"
  tmux capture-pane -J -p -t px4_isaac:0.1 -S -5000 > "$HERE/logs/isaac_pane_${rig}_${tag}.txt" 2>/dev/null
  echo "RTF: $(grep -o 'RTF [0-9.]*' "$HERE/logs/isaac_pane_${rig}_${tag}.txt" | awk '{print $2}' | tr '\n' ' ')"
done
echo "=== all done $(date +%T) ==="
