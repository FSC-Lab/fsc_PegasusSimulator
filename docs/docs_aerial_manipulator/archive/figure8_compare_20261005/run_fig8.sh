#!/usr/bin/env bash
# Matched FIGURE-8 flights at RTF 1 (2026-10-05): the launch condition of the
# circle comparison (rtf_profile_20261001/run_rt1.sh) -- headless, real-time pacer,
# Isaac pinned to the P-cores (shiqi_machine.conf ISAAC_CPUS), no wall-clock
# rescale, planner time scale 1.0 -- flying the figure-8 chosen for 0.20 m/s:
#   A = 0.75 m (half-length), B = 0.375 m (half-width), path 4.5729 m,
#   lap 22.8646 s (= 4.5729 / 0.20), q2 = 25 +- 15 deg at lap/4 = 5.71615 s,
#   fold 55 deg, takeoff heading 45 deg (long axis on world x),
#   EE-run-only planner bounds a 0.40 m/s^2, yaw rate 1.0 rad/s.
#
#   run_fig8.sh <rig>:<tag> [<rig>:<tag> ...]     e.g. run_fig8.sh wb:f8a decoupled:f8a
#
# decoupled = the geometric+L1 rig on its TUNED comparison gains (the circle
# report's yaml); override with WB_SIM_YAML for anything else.
set -uo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../../../.." && pwd)"
GEO_YAML="$HERE/../geometric_l1_tune_20261001/variants/geometric_l1_mirror_sim_tuned.yaml"
export DISPLAY="${DISPLAY:-:1}" PEGASUS_HEADLESS="${PEGASUS_HEADLESS:-1}" PEGASUS_REALTIME=1 PEGASUS_SIM_RTF=1.0
export AM_CMP_OUT="$HERE/data"
mkdir -p "$HERE/logs"
# Shape and speed (2026-10-05 sweep): FIG8_A half-length [m], FIG8_B half-width (default A/2),
# FIG8_V mean EE speed [m/s]; the lap = path / FIG8_V and the q2 period = lap / 4.
FIG8_A="${FIG8_A:-0.75}"; FIG8_B="${FIG8_B:-$(python3 -c "print($FIG8_A/2)")}"; FIG8_V="${FIG8_V:-0.20}"
read -r LAP Q2P < <(python3 -c "
import numpy as np
th=np.linspace(0,2*np.pi,400001); A,B,v=$FIG8_A,$FIG8_B,$FIG8_V
L=np.trapezoid(np.hypot(A*np.cos(th),2*B*np.cos(2*th)),th); print(f'{L/v:.4f} {L/v/4:.5f}')")
echo "figure-8 A $FIG8_A B $FIG8_B v $FIG8_V -> lap $LAP s, q2 period $Q2P s"
for spec in "$@"; do
  rig="${spec%%:*}"; tag="${spec#*:}"
  if [[ "$rig" == decoupled ]]; then export WB_SIM_YAML="${GEO_YAML_OVERRIDE:-$GEO_YAML}"; else unset WB_SIM_YAML; fi
  echo "=== $rig $tag (headless=$PEGASUS_HEADLESS, yaml=${WB_SIM_YAML:-default}) $(date +%T) ==="
  "$PEG/application/robotic_arm/utils/am_compare_cycle.sh" "$rig" "$tag" shiqi_machine -- \
    --shape figure8 --fig8-a "$FIG8_A" --fig8-b "$FIG8_B" --lap-time "$LAP" --laps 1 --q2-period "$Q2P" \
    --fold-deg 55 --q2-center-deg 25 --q2-amp-deg 15 --yaw-deg 45 \
    --ee-a-max 0.40 --ee-w-max 1.0 \
    --time-scale 1.0 --start-pos-tol "${START_POS_TOL:-0.10}" --gate-speed 0.10 \
    > "$HERE/logs/cycle_${rig}_${tag}.log" 2>&1
  echo "rc=$? $(tail -2 "$HERE/logs/cycle_${rig}_${tag}.log" | head -1)"
  tmux capture-pane -J -p -t px4_isaac:0.1 -S -5000 > "$HERE/logs/isaac_pane_${rig}_${tag}.txt" 2>/dev/null
  echo "RTF: $(grep -o 'RTF [0-9.]*' "$HERE/logs/isaac_pane_${rig}_${tag}.txt" | awk '{print $2}' | tr '\n' ' ')"
done
echo "=== all done $(date +%T) ==="
