#!/bin/bash
# TWO PARAMETER SETS (2026-11-01, user: "two sets of parameters for the free-flight and the
# interaction phases ... make sure they work in the simulation"). The valve A/B: B = the
# regenerated valve sim yaml (the grip schedule now also switches the EE impedance 211.9/26.82
# -> 120/20.2 and omega_c_t 2.927 -> 2.0 at the grasp, holds the interaction set until the exit
# is clear of the fin, and the leash / guard aborts hand the claw back over 0.25 s); A = the
# Step 1 runs of archive/valve_room_hold_guards_20261031 (rh_*: the same build minus today's
# changes, flown last night on the same seeds).
# Harness = chain2.sh there: v2 valve at 0.072 N.m (drive), raw mocap, headless, the ARM GS
# flying Ready / Turn / Exit (--gs-auto), stopped 4 s after the exit; drift model
# PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,<seed>.
#   setsid nohup bash chain_valve.sh [job ...] > ../chain_valve.out 2>&1 &   (touch ../runs/STOP to skip the rest)
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/../runs"; mkdir -p "$R"
export PL_RUNS_DIR="$R"
export PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda
export PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0
DRV=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/wb_valve_turning_flight_20261028/tools/driver
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
fly() { tag=$1; shift
  echo "$(date +%H:%M:%S) start $tag noise=${PEGASUS_PLANT_FORCE_NOISE:-none} $*"
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" bash $C/tools/try_valve.sh "$tag" "$@"; }
gs() { [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $1"; return; }; clean; fly "$@"; }
# A on today's build (the one-set yaml: every free-flight key off, the set follows the jaws)
ONESET="wb_ky_x_before_grip=0.0 wb_ky_y_before_grip=0.0 wb_ky_z_before_grip=0.0 wb_dy_x_before_grip=0.0 wb_dy_y_before_grip=0.0 wb_dy_z_before_grip=0.0 wb_l1_omega_c_t_before_grip=0.0 wb_grip_hold_set_while_free_axis=false"

run() {
  case $1 in
    nom1|nom2) unset PEGASUS_PLANT_FORCE_NOISE; gs ts_$1 ;;
    s2|s3|s4) export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,${1#s}"; gs ts_$1 ;;
    s4b) export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,4"; gs ts_s4b ;;
    # A on today's build (a control for the planner / node changes, one seed)
    one_s2) export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,2"; gs ts_one_s2 $ONESET ;;
    # FORCED: the leash at 15 mm with the gate at 10 mm (seed 2) -- now the FAST hand-back
    leash) export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,2"
           gs ts_leash push_pull_room_hold_leash=0.015 push_pull_ready_max_com_err=0.010 ;;
    # FORCED: the static q3 guard at 26 deg in the room hold -- the abort's anchor drop, FAST
    q3) unset PEGASUS_PLANT_FORCE_NOISE; gs ts_q3 push_pull_guard_q3_min_deg=26.0 ;;
    *) echo "unknown job $1" ;;
  esac
}
for j in "${@:-nom1 s2 s3 s4 leash q3 nom2 s4b one_s2}"; do for k in $j; do run "$k"; done; done
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
