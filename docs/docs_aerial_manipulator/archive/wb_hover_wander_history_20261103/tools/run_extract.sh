#!/usr/bin/env bash
# Extract the whole-body bags not yet extracted elsewhere (tracking 10-27..10-29, pick-and-drop 10-20/10-28, mail 10-31)
# into ../npz with the 10-29 valve extractor (a superset of the box-pushing one).
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$HERE/../../../../experimental_data_ros2_bag"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
X="$HERE/../../wb_valve_turning_flight_20261029/tools/extract.py"
C="$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py"
find "$ROOT"/1020* "$ROOT"/1027* "$ROOT"/1028* "$ROOT"/1029* "$ROOT"/1031* -name metadata.yaml | while IFS= read -r m; do
  d="$(dirname "$m")"; b="$(basename "$d")"
  case "$b" in flight_wb_l1_4d_*) ;; *) continue ;; esac
  case "$b" in *valve_turning*|*box_pushing*) continue ;; esac
  ts="${b##*_}"; [ -f "$HERE/../npz/b$ts.npz" ] && continue
  nice -n 19 /usr/bin/python3 "$X" "$d" "$HERE/../npz/b$ts.npz" > "$HERE/../npz/b$ts.extract.log" 2>&1 \
    && /usr/bin/python3 "$C" "$HERE/../npz/b$ts.npz" >> "$HERE/../npz/b$ts.extract.log" 2>&1; echo "$b -> b$ts rc=$?"
done
echo EXTRACT DONE
