#!/bin/bash
# 2026-11-02, the U-push campaign (Pegasus archive/valve_upush_20261102), chain 4 (after chain 3 exposed two
# driver artifacts of a U that stays open -- fixed in tools/driver -- and the approach gates under drift):
# the BARE fin, the DEEP U with the jaws kept OPEN, the claw point 80 mm under the fin top / 25 mm behind its
# centre, the turn planned 95 deg, the exit room-held 0.12 m.
#   k1 nominal | k2-k4 the hardware-like drift (seeds 2, 3, 4) WITH THE APPROACH GATES RELAXED (Ready when the
#   claw is under 60 mm/s, the hand-over within 35 mm / 50 mm/s, the station's gate 8 mm across and 12 s to
#   settle): under the drift the shipped gates abort before any contact (h2, h3), this tests the CONTACT phase |
#   k5 the operator's ABORT 10 s into the turn | k6 a GUARD abort mid-turn (q3 floor 22 deg) | k7 the
#   path-following governor (lead [10, 20] deg) | k8 the governor on the SHALLOW grasp (the U cannot push) |
#   k9 70 mm under / 20 mm behind, turn 90
#   setsid nohup bash chain4.sh > chain4.out 2>&1 &      (touch runs/STOP to skip the rest)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
A=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive
R="$HERE/runs"; mkdir -p "$R"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
stopped() { [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $1"; return 0; }; return 1; }
V=$A/valve_turning_20261015
DRV=$A/valve_upush_20261102/tools/driver
OPEN="-p push_pull_valve_cup_position:=0.019"
valve() {
  local tag=$1; shift
  stopped "$tag" && return
  clean
  echo "$(date +%H:%M:%S) start $tag noise=${PEGASUS_PLANT_FORCE_NOISE:-none} gs=${GS:-} extra=${EXTRA:-} $*"
  PL_RUNS_DIR="$R" PEGASUS_VALVE_CONTACT_LOG="$R/$tag.contacts.csv" PEGASUS_VALVE_FIN_GRIP_MM=0 \
  PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda \
  PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S=0.0 RELEASE_AT_OPEN=0 \
  ARM_GS_EXTRA_ARGS="$OPEN ${GS:-}" \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4 ${EXTRA:-}" \
    bash $V/tools/try_valve.sh "$tag" "$@"
}
DEEP80="push_pull_ee_offset=[0.1600, -0.0244, 0.1240]"
EXH="push_pull_exit_world_held_dz=0.12"
T95="push_pull_turn_deg=95.0"
GATES=("push_pull_ready_max_speed=0.06" "push_pull_handle_settle_tol=0.035" "push_pull_handle_settle_max_speed=0.05")
GSGATES="-p push_pull_valve_grip_axis_tol:=0.008 -p push_pull_valve_settle_wait_max_s:=12.0 -p push_pull_valve_grip_wait_max_s:=5.0"
GOV=("push_pull_follow=true" "push_pull_follow_lead_deg=[10.0, 20.0]")
unset PEGASUS_PLANT_FORCE_NOISE
valve k1_open80 "$DEEP80" "$EXH" "$T95"
for k in 2 3 4; do PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,$k GS="$GSGATES" valve k${k}_open80_drift$k "$DEEP80" "$EXH" "$T95" "${GATES[@]}"; done
EXTRA="--inject abort@10" valve k5_open80_abort "$DEEP80" "$EXH" "$T95"
valve k6_open80_q3trip "$DEEP80" "$EXH" "$T95" "push_pull_guard_q3_min_deg=22.0"
valve k7_open80_gov "$DEEP80" "$EXH" "$T95" "${GOV[@]}"
valve k8_shallow_gov "$EXH" "${GOV[@]}"
valve k9_open70 "push_pull_ee_offset=[0.1600, -0.0194, 0.1340]" "$EXH"
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
