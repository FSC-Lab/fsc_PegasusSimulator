#!/bin/bash
# 2026-10-27 -- the v2 valve (Valve_Assembly_v2.usda: + the static half-rings holding the
# fabric pad) on its MEASURED bench law (archive/valve_damper_spin_20261027): Coulomb
# 28 rad/s^2 x I 0.00257 = 0.072 N.m, viscous ~0 -> PEGASUS_VALVE_FRICTION 0.072 in the
# stick-slip `drive` model (cloth on plastic: it holds at rest), damping 0, rotor 204 g.
# A = the 10-22 FINAL valve config as shipped; B = A + the box push's hand-over settle gate
# (its 2026-10-24 values). Nominal x2 and the hardware-like drift on seeds 2/3/4 each,
# interleaved. The GS's Valve Turning tab flies the steps (--gs-auto), as on hardware.
# The turn is the experiment's 90 deg in 12 s (user, 2026-10-27; the generator's default since);
# each run ends 4 s after the exit (--stop-after-exit: the interaction phase is what is tested).
# (A first chain at 360 deg was stopped mid-flight: runs/stopped_360/.)
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/../runs"
export PL_RUNS_DIR="$R"
export PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda
export PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
gs() { tag=$1; shift; [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; echo "$(date +%H:%M:%S) start $tag"
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4 ${EXTRA:-}" bash $C/tools/try_valve.sh "$tag" "$@"; }
SETTLE="push_pull_handle_settle_tol=0.025 push_pull_handle_settle_axis_tol=0.015 push_pull_handle_settle_max_speed=0.030 push_pull_handle_settle_dwell_s=0.3 push_pull_ready_max_speed=0.030 push_pull_ready_settle_s=0.5"
gs t90A_nom_1;                                                   gs t90B_nom_1 $SETTLE
for k in 2 3 4; do
  PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k" gs t90A_hw_$k
  PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k" gs t90B_hw_$k $SETTLE
done
gs t90A_nom_2;                                                   gs t90B_nom_2 $SETTLE
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
