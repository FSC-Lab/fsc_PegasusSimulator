#!/bin/bash
# 2026-10-23 -- the FINAL valve config (10-22: free axis 0.75 on_world + the planner fixes) flown on the
# bearing law FITTED from the 10-23 free-spin bench bag (archive/valve_bearing_spin_20261023):
#   Coulomb 3.48 rad/s^2 overall, split 2.52 rad/s^2 + 0.30 /s * w  x  I = 0.002569 kg m^2 (204 g, CAD k)
#   -> PEGASUS_VALVE_FRICTION 0.0065 N.m + PEGASUS_VALVE_DAMPING 0.00077 N.m.s/rad (6.8 mN.m at 24 deg/s)
#   low end: 0.004 + 0.00077 (the turn-speed band's 95 % floor at a lighter, 170 g rotor)
# A STRESS CASE against the scene default (0.048 / 0.0 = the user's conservative setting, unchanged).
# Paired with the 10-22 FINAL runs at 0.048: s22_final_nom_1/2 + s22_fa0_nom_1, s22_final_hw_2/3/4.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/../runs"
export PL_RUNS_DIR="$R"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
gs() { tag=$1; shift; [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; echo "$(date +%H:%M:%S) start $tag"
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_DRIVER_ARGS="--gs-auto ${EXTRA:-}" bash $C/tools/try_valve.sh "$tag" "$@"; }
# prefix assignments on a function call reach try_valve.sh's environment (the 10-22 chains' pattern)
F=0.0065; FL=0.004; B=0.00077
for k in 1 2 3; do PEGASUS_VALVE_FRICTION=$F PEGASUS_VALVE_DAMPING=$B gs vb_nom_$k; done
for k in 2 3 4; do PEGASUS_VALVE_FRICTION=$F PEGASUS_VALVE_DAMPING=$B PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,$k" gs vb_hw_$k; done
PEGASUS_VALVE_FRICTION=$FL PEGASUS_VALVE_DAMPING=$B gs vb_low_nom_1
PEGASUS_VALVE_FRICTION=$FL PEGASUS_VALVE_DAMPING=$B PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,3" gs vb_low_hw_3
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
