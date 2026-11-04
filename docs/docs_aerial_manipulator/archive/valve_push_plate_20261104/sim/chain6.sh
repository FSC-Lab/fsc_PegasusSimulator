#!/bin/bash
# chain 6 (2026-11-04): THE FOUR-EDGE SAFETY GUARDS + THE ABORT THAT CLIMBS AT ONCE, the plate push at
# [0, 5, 25, 0] (chain 4's p10-p13 configuration) plus SAFE:
#   push_pull_guard_fin_depth_max 0.060  (the plate's BOTTOM edge; new planner key)
#   push_pull_abort_release_wait_s 0.0 + push_pull_abort_climb_on_open false (no pause: climb at once)
# The other three edges: depth_min 0.012 (top), across_max 0.035 (both sides) -- already in PLATE.
#   s1 nominal | s2 drift seed 3 (no false trip) | each edge with the claw aimed a few mm INSIDE its limit,
#   so the hand-over hold's and the turn's own motion carry it over:
#   s3 outer side (claw +31 mm, the turn drifts outward) | s4 inner side (-34 mm) | s5 top (13.5 mm under it)
#   s6 bottom (58.5 mm under the top)
#   setsid nohup bash chain6.sh > chain6.out 2>&1 &      (touch runs/STOP to skip the rest)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
A=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive
ASSETS=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets
R="$HERE/runs"; mkdir -p "$R"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
stopped() { [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $1"; return 0; }; return 1; }
V=$A/valve_turning_20261015
DRV=$A/valve_upush_20261102/tools/driver
valve() {
  local tag=$1; shift
  stopped "$tag" && return
  clean
  echo "$(date +%H:%M:%S) start $tag usd=$(basename ${USD:-v21}) noise=${PEGASUS_PLANT_FORCE_NOISE:-none} gs=${GS:-} $*"
  PL_RUNS_DIR="$R" PEGASUS_VALVE_CONTACT_LOG="$R/$tag.contacts.csv" PEGASUS_VALVE_FIN_GRIP_MM=0 \
  PEGASUS_VALVE_USD=${USD:-$ASSETS/Valve_Assembly_v21.usda} PEGASUS_VALVE_ROTOR_MASS=${ROTOR_MASS:-0.260} \
  PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S=0.0 RELEASE_AT_OPEN=0 \
  ARM_GS_EXTRA_ARGS="${GS:-}" \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" \
    bash $V/tools/try_valve.sh "$tag" "$@"
}
CUP="-p push_pull_valve_cup_position:=-0.010"
GATES=("push_pull_ready_max_speed=0.06" "push_pull_handle_settle_tol=0.035" "push_pull_handle_settle_max_speed=0.05")
GSGATES="-p push_pull_valve_grip_axis_tol:=0.008 -p push_pull_valve_settle_wait_max_s:=12.0 -p push_pull_valve_grip_wait_max_s:=5.0"
PLATE=("push_pull_valve_fin=[0.1600, -0.0219, 0.2040, 0.0790, 0.0025, 0.0600]"
       "push_pull_guard_fin_bracket_back=-0.0169" "push_pull_guard_fin_depth_min=0.012"
       "push_pull_guard_fin_across_max=0.035" "push_pull_turn_deg=97.0" "push_pull_exit_world_held_dz=0.08"
       "push_pull_follow_lead_deg=[14.0, 24.0]" "push_pull_turn_tangent_lead=0.0369")
EE="push_pull_ee_offset=[0.1600, -0.0613, 0.1704]"
EE_LAT="push_pull_ee_offset=[0.1720, -0.0613, 0.1704]"
POSE="push_pull_push_pose_deg=[0.0, 0.0, 30.0, 0.0]"
COM=("push_pull_world_anchor_push=false" "push_pull_world_anchor_on_handle=false")
unset PEGASUS_PLANT_FORCE_NOISE PEGASUS_VALVE_FAULT USD GS ROTOR_MASS
POSE525="push_pull_push_pose_deg=[0.0, 5.0, 25.0, 0.0]"
SAFE=("push_pull_guard_fin_depth_max=0.060" "push_pull_abort_release_wait_s=0.0" "push_pull_abort_climb_on_open=false")
GS="$CUP" valve s1_nominal "${PLATE[@]}" "$EE" "$POSE525" "${SAFE[@]}"
PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,3 GS="$CUP $GSGATES" valve s2_drift3 "${PLATE[@]}" "$EE" "$POSE525" "${SAFE[@]}" "${GATES[@]}"
GS="$CUP" valve s3_outer "${PLATE[@]}" "push_pull_ee_offset=[0.1910, -0.0613, 0.1704]" "$POSE525" "${SAFE[@]}"
GS="$CUP" valve s4_inner "${PLATE[@]}" "push_pull_ee_offset=[0.1260, -0.0613, 0.1704]" "$POSE525" "${SAFE[@]}"
GS="$CUP" valve s5_top "${PLATE[@]}" "push_pull_ee_offset=[0.1600, -0.0613, 0.1905]" "$POSE525" "${SAFE[@]}"
GS="$CUP" valve s6_bottom "${PLATE[@]}" "push_pull_ee_offset=[0.1600, -0.0613, 0.1455]" "$POSE525" "${SAFE[@]}"
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
