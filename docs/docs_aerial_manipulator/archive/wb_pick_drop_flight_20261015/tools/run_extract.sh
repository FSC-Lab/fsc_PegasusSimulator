#!/usr/bin/env bash
# Extract the three 2026-10-15 bags in parallel into ../npz and make them numpy-1 readable.
set +u   # the ROS setup scripts read unset variables
HERE="$(cd "$(dirname "$0")" && pwd)"
BAGS="$HERE/../../../../experimental_data_ros2_bag/1015 - T650-AM whole-body Pick&Drop-20261016T013913Z-1-001/1015 - T650-AM whole-body Pick&Drop"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
declare -A RUN=([e1]=flight_wb_l1_4d_pick_and_drop_empty_20261015_172318 [d1]=flight_wb_l1_4d_pick_and_drop_20261015_175059 [d2]=flight_wb_l1_4d_pick_and_drop_20261015_180850)
for k in "${!RUN[@]}"; do
  ( /usr/bin/python3 "$HERE/extract.py" "$BAGS/${RUN[$k]}" "$HERE/../npz/$k.npz" > "$HERE/../npz/$k.extract.log" 2>&1 \
    && /usr/bin/python3 "$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py" "$HERE/../npz/$k.npz" >> "$HERE/../npz/$k.extract.log" 2>&1; echo "$k done rc=$?" ) &
done
wait
