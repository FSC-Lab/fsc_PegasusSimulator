#!/usr/bin/env bash
# Whole-body figure-8 SPEED SWEEP (2026-10-07): the hardware's first-flight shape
# A 0.70 / B 0.35 m (path 4.268 m), mean EE speed FIG8_V, q2 = 25 +- 15 deg at lap/4,
# fold 55 deg, 1 lap, s = 1 -- the 10-05 sweep's launch condition (run_fig8.sh:
# headless, RTF 1 pacer, P-core pinning, mirror plant, raw mocap stack, takeoff yaw 45
# deg = long axis on world x with no go-to-start yaw). WB rig only.
#
#   run_speed.sh <v>:<tag> [<v>:<tag> ...]          e.g. run_speed.sh 0.16:v016_a
#
# EE-run planner bounds default to the HARDWARE yaml's (a 0.40 m/s^2, yaw 1.0 rad/s,
# v = shared 0.30 m/s); EE_V_MAX / EE_A_MAX / EE_W_MAX override them for sim-only
# margin probes above 0.20 m/s (A 0.70 is CoM-speed bound at 0.20: s_max 1.02).
set -uo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../../../.." && pwd)"
export DISPLAY="${DISPLAY:-:1}" PEGASUS_HEADLESS="${PEGASUS_HEADLESS:-1}" PEGASUS_REALTIME=1 PEGASUS_SIM_RTF=1.0
export AM_CMP_OUT="$HERE/data" AM_CMP_FEEDBACK="${AM_CMP_FEEDBACK:-raw}"
unset WB_SIM_YAML
A=0.70; B=0.35
for spec in "$@"; do
  V="${spec%%:*}"; tag="${spec#*:}"
  read -r LAP Q2P < <(python3 -c "
import numpy as np
th=np.linspace(0,2*np.pi,400001); A,B,v=$A,$B,$V
L=np.trapezoid(np.hypot(A*np.cos(th),2*B*np.cos(2*th)),th); print(f'{L/v:.4f} {L/v/4:.5f}')")
  extra=(--ee-a-max "${EE_A_MAX:-0.40}" --ee-w-max "${EE_W_MAX:-1.0}")
  [[ -n "${EE_V_MAX:-}" ]] && extra+=(--ee-v-max "$EE_V_MAX")
  echo "=== wb $tag v $V -> lap $LAP s, q2 period $Q2P s, bounds ${extra[*]} $(date +%T) ==="
  "$PEG/application/robotic_arm/utils/am_compare_cycle.sh" wb "$tag" shiqi_machine -- \
    --shape figure8 --fig8-a "$A" --fig8-b "$B" --lap-time "$LAP" --laps 1 --q2-period "$Q2P" \
    --fold-deg 55 --q2-center-deg 25 --q2-amp-deg 15 --yaw-deg 45 "${extra[@]}" \
    --time-scale 1.0 --start-pos-tol "${START_POS_TOL:-0.10}" --gate-speed 0.10 \
    > "$HERE/logs/cycle_wb_${tag}.log" 2>&1
  echo "rc=$? $(tail -2 "$HERE/logs/cycle_wb_${tag}.log" | head -1)"
  tmux capture-pane -J -p -t px4_isaac:0.1 -S -5000 > "$HERE/logs/isaac_pane_wb_${tag}.txt" 2>/dev/null
  echo "RTF: $(grep -o 'RTF [0-9.]*' "$HERE/logs/isaac_pane_wb_${tag}.txt" | awk '{print $2}' | tr '\n' ' ')"
done
echo "=== all done $(date +%T) ==="
