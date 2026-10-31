#!/usr/bin/env bash
# Extract the four 2026-10-28 pick-and-drop bags (drop margin 12 / 8 / 6 / 5 cm) in parallel into ../npz and make
# them numpy-1 readable. Uses the 2026-10-16 extractor (the 10-09 one + the gripper action status).
set +u   # the ROS setup scripts read unset variables
HERE="$(cd "$(dirname "$0")" && pwd)"
BAGS="$HERE/../../../../experimental_data_ros2_bag/1028 - T650-AM whole-body Tracking, Pick&Drop, Valve-Turning-20261029T003205Z-1-001/1028 - T650-AM whole-body Tracking, Pick&Drop, Valve-Turning"
X="$HERE/../../wb_pick_drop_flight_20261016/tools/extract.py"
NP1="$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
declare -A RUN=([m12]=flight_wb_l1_4d_pick_and_drop_12offset_20261028_161640 [m8]=flight_wb_l1_4d_pick_and_drop_8offset_20261028_162711
                [m6]=flight_wb_l1_4d_pick_and_drop_6offset_20261028_164816 [m5]=flight_wb_l1_4d_pick_and_drop_5offset_20261028_165603)
for k in "${!RUN[@]}"; do
  ( nice /usr/bin/python3 "$X" "$BAGS/${RUN[$k]}" "$HERE/../npz/$k.npz" > "$HERE/../npz/$k.extract.log" 2>&1 \
    && /usr/bin/python3 "$NP1" "$HERE/../npz/$k.npz" >> "$HERE/../npz/$k.extract.log" 2>&1; echo "$k done rc=$?" ) &
done
wait
