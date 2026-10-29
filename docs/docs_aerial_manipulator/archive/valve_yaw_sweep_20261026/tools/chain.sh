#!/bin/bash
# 2026-10-26 valve-turning HEADING-CHANNEL SWEEP (README.md beside this folder): the shipped valve
# config (valve_turning_20261015 make_valve_turning_yaml.py, as on 10-22) with
#   * a 90 deg turn in 12 s (push_pull_turn_deg 90, push_pull_push_time_s 12, the 1.5 s settle kept),
#     released 17 s after the push step starts (settle + turn + 3.5 s on the fin), the valve's own
#     exit (open with the claw world-held, Exit 3 s later -- --open-first-exit, asked by name),
#     the run ENDED 2 s after the exit (--stop-after-exit: no way home, no landing);
#   * the heading impedance K_psi from the run spec, D_psi = 2.6 sqrt(0.05 K_psi) (zeta ~1.3),
#     M_psi 0.05 (the yaml's); x / y / z unchanged;
#   * the bearing: Coulomb friction tau_f from the run spec in the STICK-SLIP model
#     (PEGASUS_VALVE_FRICTION_MODEL=drive -- the default tanh model cannot hold the rotor, see
#     tools/friction_check.py); load, ratchet, damping off.
# Runs land in valve_turning_20261015/runs/yw_*.
#   setsid nohup bash chain.sh tag:K:tau [tag:K:tau ...] > ../chain.log 2>&1 < /dev/null &
# K = "ref" flies the yaml's own heading gains (0.2484 / 0.2903). touch ../STOP to stop before the next run.
V=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R=$V/runs
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1
          ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() {   # run <tag> <K|ref> <tau_f>
  local tag=$1 K=$2 tau=$3
  [ -e "$R/$tag.npz" ] && { echo "skip $tag (flown)"; return; }
  [ -e "$HERE/../STOP" ] && { echo "STOP file: skipping $tag"; return; }
  clean
  local gains=()
  if [[ $K != ref ]]; then
    local Kf Df
    Kf=$(/usr/bin/python3 -c "print(f'{float($K):.4f}')")
    Df=$(/usr/bin/python3 -c "import math; print(f'{2.6 * math.sqrt(0.05 * float($K)):.4f}')")
    gains=("wb_ky_psi=$Kf" "wb_dy_psi=$Df")
  fi
  echo "[$(date +%T)] start $tag K=$K tau_f=$tau ${gains[*]}"
  PEGASUS_VALVE_FRICTION="$tau" PEGASUS_VALVE_FRICTION_MODEL=drive RELEASE_AT_OPEN=0 \
    PL_DRIVER_ARGS="--timetable --slot-release 17.0 --stop-after-exit --stop-hold 2.0" \
    bash $V/tools/try_valve.sh "$tag" push_pull_turn_deg=90.0 push_pull_push_time_s=12.0 "${gains[@]}"
  echo "FRICTION_MODEL=drive" >> "$R/$tag.args"
  echo "[$(date +%T)] end $tag: $(grep -E '^completed|^ABORTED' "$R/$tag.log" 2>/dev/null | head -1 | cut -c1-220)"
}
for a in "$@"; do IFS=: read -r t k f <<< "$a"; run "$t" "$k" "$f"; done
clean
echo "[$(date +%T)] CHAIN DONE"
