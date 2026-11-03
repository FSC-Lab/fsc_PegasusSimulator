#!/bin/bash
# chain 3 (2026-11-03): after chain 2, REBUILD the planner (no stack runs then: the clean slate's pgrep patterns
# match compiler lines) with two fixes from chain 1, then re-fly the runs they concern:
#   - push_pull_guard_fin_q3_max_deg 40: the forced JAM kept the fin in the U while the vehicle pressed on it and
#     q3 went 32 -> 50 deg (the stop); the tilt guard fired only there and the abort from the stops crashed (g5)
#   - the fin-AHEAD check only while the turn flies (g11_drift4: a false abort in the hand-over hold, 62 mm)
#   j1 nominal | j2 jam at 30 deg | j3 drift seed 4 (the relaxed approach gates) | j4 drift seed 2
#   setsid nohup bash chain3.sh > chain3.out 2>&1 &
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
until grep -q "CHAIN DONE" "$HERE/chain2.out" 2>/dev/null; do sleep 15; done
echo "$(date +%H:%M:%S) building fsc_trajectory_planner"
( cd ~/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select fsc_trajectory_planner 2>&1 | tail -2 )
( cd ~/ros2_ws && ./build/fsc_trajectory_planner/test_push_pull 2>&1 | grep -E "^\[  (PASSED|FAILED)" )
/usr/bin/python3 ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015/tools/make_valve_turning_yaml.py | tail -1
sed -n '/^A=/,/^}/p' "$HERE/chain1.sh" > "$HERE/.chain3_funcs.sh"   # (generated, removed after the run)
R="$HERE/runs"
source "$HERE/.chain3_funcs.sh"
GATES=("push_pull_ready_max_speed=0.06" "push_pull_handle_settle_tol=0.035" "push_pull_handle_settle_max_speed=0.05")
GSGATES="-p push_pull_valve_grip_axis_tol:=0.008 -p push_pull_valve_settle_wait_max_s:=12.0 -p push_pull_valve_grip_wait_max_s:=5.0"
unset PEGASUS_PLANT_FORCE_NOISE PEGASUS_VALVE_FAULT FRICTION GS
valve j1_nominal
PEGASUS_VALVE_FAULT=jam,30 valve j2_jam
PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,4 GS="$GSGATES" valve j3_drift4 "${GATES[@]}"
PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,2 GS="$GSGATES" valve j4_drift2 "${GATES[@]}"
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
