#!/usr/bin/env bash
# Extract the six hardware bags (0918, 0921 #3, 0924 F1-F4) with the 0924 flattener.
set -uo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
B="$HERE/../../experimental_data_ros2_bag"
EX="$HERE/../wb_l1_4d_flight_20260924/tools/extract_bag.py"
set +u
source /opt/ros/humble/setup.bash
source "$HOME/workspaces/isaacsim/install/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"
set -u
declare -A BAGS=(
 [f18]="0918 - T650-AM whole-body-L1-4D-20260919T003515Z-1-001/0918 - T650-AM whole-body-L1-4D/flight_wb_l1_4d_20260918_144828"
 [c3]="0921 - T650-AM whole-body-L1-4D-20260921T170115Z-1-001/0921 - T650-AM whole-body-L1-4D/flight_wb_l1_4d_circle_20260921_123637"
 [a1]="0924 - T650-AM whole-body-L1-4D Circle-20260925T003220Z-1-001/0924 - T650-AM whole-body-L1-4D Circle/flight_wb_l1_4d_circle_20260924_120228"
 [a2]="0924 - T650-AM whole-body-L1-4D Circle-20260925T003220Z-1-001/0924 - T650-AM whole-body-L1-4D Circle/flight_wb_l1_4d_circle_20260924_120546"
 [a3]="0924 - T650-AM whole-body-L1-4D Circle-20260925T003220Z-1-001/0924 - T650-AM whole-body-L1-4D Circle/flight_wb_l1_4d_circle_20260924_165654"
 [a4]="0924 - T650-AM whole-body-L1-4D Circle-20260925T003220Z-1-001/0924 - T650-AM whole-body-L1-4D Circle/flight_wb_l1_4d_circle_20260924_172101"
)
for k in f18 c3 a1 a2 a3 a4; do
  [[ -f "$HERE/data/$k.npz" ]] && { echo "$k: exists"; continue; }
  echo "== $k"
  /usr/bin/python3 "$EX" "$B/${BAGS[$k]}" "$HERE/data/$k.npz" > "$HERE/logs/extract_$k.log" 2>&1 && echo "$k: ok" || echo "$k: FAILED (see logs)"
done
