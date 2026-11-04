#!/bin/bash
# chain 2 = chain 1 + push_pull_turn_tangent_lead 0.0369 (chain 1 was refused at Plan: the tangent check at the claw point).
# 2026-11-04, THE PUSH PLATE (Pegasus archive/valve_push_plate_20261104): the CLOSED claw pushes a 120 x 79 x 5 mm
# plate on the fin's pushing edge with its nose (Valve_Assembly_v21.usda), against the U push (v2, jaws open).
# The plate push = the generated valve yaml + KEY=VAL overrides (try_valve.sh) + the station's cup position CLOSED:
#   the nose 20 mm behind the plate face at the grasp (the slack), 165 mm above the plate bottom (39 mm under the
#   plate top), the claw on the fin's radius; planned 97 deg; the fin-in-the-U guard re-described as the PLATE
#   (push_pull_valve_fin = the plate box, the push point 16.9 mm AHEAD of the claw point).
#   p1 nominal, pose [0, 0, 30, 0], the shipped approach (CoM-anchored descent -> settle -> room hold, free axis)
#   p2 nominal, pose [0, 0, 30, 0], the claw CoM-ANCHORED through the turn (no room hold): the plate takes the wander
#   p3-p5 drift seeds 2/3/4 (room hold) | p6-p7 drift 2/3 (CoM-anchored)
#   p8 the claw aimed 12 mm off the fin's radius (the hardware's FK error class), room hold
#   p9 THE U PUSH (v2, shipped) aimed 12 mm off -- the comparison | p10 the old pose [0, 5, 25, 0], room hold
#   setsid nohup bash chain1.sh > chain1.out 2>&1 &      (touch runs/STOP to skip the rest)
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
GS="$CUP" valve p1_nominal "${PLATE[@]}" "$EE" "$POSE"
GS="$CUP" valve p2_nominal_com "${PLATE[@]}" "$EE" "$POSE" "${COM[@]}"
for k in 2 3 4; do PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,$k GS="$CUP $GSGATES" valve p$((1 + k))_drift$k "${PLATE[@]}" "$EE" "$POSE" "${GATES[@]}"; done
for k in 2 3; do PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,$k GS="$CUP $GSGATES" valve p$((4 + k))_drift${k}_com "${PLATE[@]}" "$EE" "$POSE" "${COM[@]}" "${GATES[@]}"; done
GS="$CUP" valve p8_lat12 "${PLATE[@]}" "$EE_LAT" "$POSE"
USD=$ASSETS/Valve_Assembly_v2.usda ROTOR_MASS=0.204 GS="" valve p9_upush_lat12 "push_pull_ee_offset=[0.1720, -0.0244, 0.1240]"
GS="$CUP" valve p10_pose5_25 "${PLATE[@]}" "$EE" "push_pull_push_pose_deg=[0.0, 5.0, 25.0, 0.0]"
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
