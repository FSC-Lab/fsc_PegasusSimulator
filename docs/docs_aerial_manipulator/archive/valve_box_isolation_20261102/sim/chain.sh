#!/bin/bash
# The 2026-11-02 isolation, flown in Isaac (Pegasus archive/valve_box_isolation_20261102):
#   1. BOX REGRESSION -- the 10-28 comparison campaign's whole-body task (280 g, 0.40 m at 21 mm/s, its slots,
#      the release-first exit) on the box yaml that is byte-identical to the one flown 2026-10-27; compared
#      against that campaign's five kept whole-body runs (box_pushing_comparison_20261028)
#   2. VALVE -- the v2 valve at 0.072 N.m (drive model), raw mocap, headless, the ARM GS flying every step by
#      itself (--gs-auto, as on hardware), each run stopped 4 s after the exit:
#      a static hold (the grasp event, no turn), the 90 deg turn, the turn under the hardware-like drift, and
#      every abort path left for the valve (q3 guard, sway guard, the emergency abort, a mocap freeze)
#   setsid nohup bash chain.sh [job ...] > chain.out 2>&1 &      (touch runs/STOP to skip the rest)
# Jobs: box valve aborts (default: all three)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
A=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive
R="$HERE/runs"; mkdir -p "$R"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
stopped() { [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $1"; return 0; }; return 1; }

box() {   # the 10-28 campaign's whole-body flight (results/utils/run_box_pushing_campaign.sh's settings)
  local tag=$1
  stopped "$tag" && return
  clean
  echo "$(date +%H:%M:%S) start $tag (box regression)"
  MASS=0.280 DIST=0.40 SPEED=0.021 \
    PL_DRIVER_ARGS="--slot-ready 35.0 --slot-close 8.0 --slot-release 22.05 --slot-land-step 40.0 --release-at-open" \
    bash $A/box_pushing_campaign_20261016/tools/fly.sh "$tag"
  cp $A/box_pushing_campaign_20261016/runs/"$tag".* "$R/" 2>/dev/null
}

V=$A/valve_turning_20261015
DRV=$HERE/../tools/driver
valve() {   # one valve flight, the GS flying every step; EXTRA = more driver args, HOLD = the static hold [s]
  local tag=$1; shift
  stopped "$tag" && return
  clean
  echo "$(date +%H:%M:%S) start $tag hold=${HOLD:-0} noise=${PEGASUS_PLANT_FORCE_NOISE:-none} $* ${EXTRA:-}"
  local hold_args=""; [[ -n "${HOLD:-}" ]] && hold_args="--gs-hold $HOLD"
  PL_RUNS_DIR="$R" \
  PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda \
  PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S="${HOLD:-0.0}" RELEASE_AT_OPEN=0 \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4 $hold_args ${EXTRA:-}" \
    bash $V/tools/try_valve.sh "$tag" "$@"
}

run() {
  case $1 in
    box) for k in 1 2 3; do box iso_box_$k; done ;;
    valve)
      unset PEGASUS_PLANT_FORCE_NOISE
      HOLD=10.0 valve iso_vhold_1
      valve iso_vturn_1
      valve iso_vturn_2
      PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,2" valve iso_vturn_s2 ;;
    aborts)
      unset PEGASUS_PLANT_FORCE_NOISE
      # the q3 floor raised to 25 deg so it trips in the turn: the claw follows the vehicle at once
      valve iso_q3trip push_pull_guard_q3_min_deg=25.0
      # the sway guard at 0.05 m/s on drift seed 2
      PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,2" valve iso_swaytrip push_pull_guard_sway_speed=0.05
      # the emergency abort: the soft tilt watchdog at 3.5 deg so it trips in the turn
      valve iso_emerg system_wd_max_tilt_deg=3.5
      # the drone's mocap frozen 2.5 s, 4 s into the turn
      EXTRA="--mocap-fault freeze@push+4:2.5" valve iso_mfreeze
      # an abort DURING THE STATIC HOLD (the drone's mocap frozen 2.5 s, 3 s after the grasp event): the hold
      # is retired (no release event, no Exit), the planner's abort opens and climbs, the jaws close after it
      HOLD=10.0 EXTRA="--mocap-fault freeze@push+3:2.5" valve iso_holdfreeze ;;
    refly)   # (after the first chain: the planner publishes info on a grasp event; the driver leaves the
             # gripper to the GS in --gs-auto -- the first holds' runs are in runs/superseded/)
      unset PEGASUS_PLANT_FORCE_NOISE
      HOLD=10.0 valve iso_vhold_2
      valve iso_vturn_3
      HOLD=10.0 EXTRA="--mocap-fault freeze@push+3:2.5" valve iso_holdfreeze_2 ;;
    *) echo "unknown job $1" ;;
  esac
}
for j in "${@:-box valve aborts}"; do for k in $j; do run "$k"; done; done
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
