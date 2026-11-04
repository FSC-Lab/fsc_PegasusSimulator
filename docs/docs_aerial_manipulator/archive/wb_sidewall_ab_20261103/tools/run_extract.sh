#!/usr/bin/env bash
# Extract the 11-02 free-flight bags (wall up) that have their .db3, into ../npz with the 10-29 valve extractor.
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$HERE/../../../../experimental_data_ros2_bag/1102 - T650-AM Final Movies"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
X="$HERE/../../wb_valve_turning_flight_20261029/tools/extract.py"
C="$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py"
find "$ROOT" -name metadata.yaml | while IFS= read -r m; do
  d="$(dirname "$m")"; b="$(basename "$d")"; ts="${b##*_}"
  ls "$d"/*.db3 >/dev/null 2>&1 || { echo "$b: no .db3, skipped"; continue; }
  [ -f "$HERE/../npz/b$ts.npz" ] && continue
  nice -n 19 /usr/bin/python3 "$X" "$d" "$HERE/../npz/b$ts.npz" > "$HERE/../npz/b$ts.extract.log" 2>&1 \
    && /usr/bin/python3 "$C" "$HERE/../npz/b$ts.npz" >> "$HERE/../npz/b$ts.extract.log" 2>&1; echo "$b -> b$ts rc=$?"
done
echo EXTRACT DONE
