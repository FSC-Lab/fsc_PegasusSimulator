#!/bin/bash
# 2026-11-02, the U-push campaign (Pegasus archive/valve_upush_20261102), chain 3: the BARE fin, the DEEP U with the
# jaws kept OPEN (the station's push_pull_valve_cup_position 0.019 = open: the GS's "close" moves nothing), the
# claw point 80 mm under the fin top and 25 mm behind its centre, the turn planned 95 deg (the ~13 mm of slack
# before the hub meets the fin's back edge costs ~5 deg). The driver is this campaign's copy (a U that stays open
# never "closes": the GS's Push is the close event).
#   h1 nominal | h2-h4 the hardware-like drift (seeds 2, 3, 4) | h5 the operator's ABORT 10 s into the turn |
#   h6 a GUARD abort mid-turn (q3 floor raised to 22 deg) | h7 the path-following governor on (lead [10, 20] deg) |
#   h8 the governor on, the SHALLOW grasp of f0 (the U cannot push: the governor must hold and abort before the
#   landing gear reaches the fin) | h9 70 mm under / 20 mm behind, turn 90
#   setsid nohup bash chain3.sh > chain3.out 2>&1 &      (touch runs/STOP to skip the rest)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
A=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive
R="$HERE/runs"; mkdir -p "$R"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
stopped() { [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $1"; return 0; }; return 1; }
V=$A/valve_turning_20261015
DRV=$A/valve_upush_20261102/tools/driver
valve() {
  local tag=$1; shift
  stopped "$tag" && return
  clean
  echo "$(date +%H:%M:%S) start $tag noise=${PEGASUS_PLANT_FORCE_NOISE:-none} extra=${EXTRA:-} $*"
  PL_RUNS_DIR="$R" PEGASUS_VALVE_CONTACT_LOG="$R/$tag.contacts.csv" PEGASUS_VALVE_FIN_GRIP_MM=0 \
  PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda \
  PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S=0.0 RELEASE_AT_OPEN=0 \
  ARM_GS_EXTRA_ARGS="-p push_pull_valve_cup_position:=0.019" \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4 ${EXTRA:-}" \
    bash $V/tools/try_valve.sh "$tag" "$@"
}
DEEP80="push_pull_ee_offset=[0.1600, -0.0244, 0.1240]"
EXH="push_pull_exit_world_held_dz=0.12"
T95="push_pull_turn_deg=95.0"
unset PEGASUS_PLANT_FORCE_NOISE
valve h1_open80 "$DEEP80" "$EXH" "$T95"
for k in 2 3 4; do PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,$k valve h${k}_open80_drift$k "$DEEP80" "$EXH" "$T95"; done
EXTRA="--inject abort@10" valve h5_open80_abort "$DEEP80" "$EXH" "$T95"
valve h6_open80_q3trip "$DEEP80" "$EXH" "$T95" "push_pull_guard_q3_min_deg=22.0"
GOV=("push_pull_follow=true" "push_pull_follow_lead_deg=[10.0, 20.0]")
valve h7_open80_gov "$DEEP80" "$EXH" "$T95" "${GOV[@]}"
valve h8_shallow_gov "$EXH" "${GOV[@]}"
valve h9_open70 "push_pull_ee_offset=[0.1600, -0.0194, 0.1340]" "$EXH"
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
