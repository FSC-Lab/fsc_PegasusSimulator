#!/usr/bin/env bash
# Extract the five 2026-10-21 box-pushing bags into ../npz (parallel, E-cores, nice) and make
# them numpy-1 readable.
set +u   # the ROS setup scripts read unset variables
HERE="$(cd "$(dirname "$0")" && pwd)"
BAGS="$HERE/../../../../experimental_data_ros2_bag/1021 - T650-AM whole-body Box-Pushing [Success]-20261021T223148Z-1-001/1021 - T650-AM whole-body Box-Pushing [Success]"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
# the bags sit in class sub-folders (organised 2026-10-21): stale/, free_flight/, box_pushing/
declare -A DIR=([161745]=stale [162604]=stale [165311]=stale [164338]=free_flight [170123]=box_pushing)
for b in 161745 162604 164338 165311 170123; do
  ( nice -n 19 taskset -c 16-31 /usr/bin/python3 "$HERE/extract.py" "$BAGS/${DIR[$b]}/flight_wb_l1_4d_box_pushing_20261021_$b" "$HERE/../npz/b$b.npz" > "$HERE/../npz/b$b.extract.log" 2>&1 \
    && /usr/bin/python3 "$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py" "$HERE/../npz/b$b.npz" >> "$HERE/../npz/b$b.extract.log" 2>&1; echo "b$b done rc=$?" ) &
done
wait
