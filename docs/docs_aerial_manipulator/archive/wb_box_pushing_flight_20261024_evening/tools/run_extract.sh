#!/usr/bin/env bash
# Extract the 2026-10-24 evening box-pushing bags (280 g, 510 g) into ../npz with the 10-21 extractor,
# numpy-1 readable.  Usage: run_extract.sh [194310 195657]
set +u   # the ROS setup scripts read unset variables
HERE="$(cd "$(dirname "$0")" && pwd)"
BAGS="$HERE/../../../../experimental_data_ros2_bag/1024 - T650-AM whole-body Box-Pushing-20261024T192954Z-1-001/1024 - T650-AM whole-body Box-Pushing"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
X="$HERE/../../wb_box_pushing_flight_20261021/tools/extract.py"
C="$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py"
declare -A NAME=([194310]=flight_wb_l1_4d_box_pushing_280g_20261024_194310 [195657]=flight_wb_l1_4d_box_pushing_500g_20261024_195657)
TAGS="${*:-194310 195657}"
for tag in $TAGS; do
  nice -n 19 taskset -c 16-31 /usr/bin/python3 "$X" "$BAGS/${NAME[$tag]}" "$HERE/../npz/b$tag.npz" > "$HERE/../npz/b$tag.extract.log" 2>&1 \
    && /usr/bin/python3 "$C" "$HERE/../npz/b$tag.npz" >> "$HERE/../npz/b$tag.extract.log" 2>&1; echo "b$tag rc=$?"
done
