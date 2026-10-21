#!/usr/bin/env bash
# The box-pushing workflow page's auto-exit picture (2026-10-21): the REAL Box Pushing
# panel, offscreen, against a FAKE planner (no Isaac, no PX4, no planner node) on a
# private ROS domain. Builds the harness outside the repo, then crops the Mission row.
#   bash gs_wf_snapshot.sh [out dir]      (default: ../shots)
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-$HERE/../shots}"
BUILD="${GS_WF_BUILD:-/tmp/claude-1000/gs_wf_harness_build}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_CHECK:-81}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
mkdir -p "$BUILD" "$OUT"
cmake -S "$HERE/gs_wf_harness" -B "$BUILD" > "$BUILD.cmake.log" 2>&1 && cmake --build "$BUILD" -j > "$BUILD.build.log" 2>&1 \
  || { echo "harness build failed: $BUILD.build.log"; exit 1; }
QT_QPA_PLATFORM=offscreen timeout 30 "$BUILD/gs_wf_snapshot" "$OUT" 2>"$BUILD.run.err" | tee "$BUILD.run.log" | grep GSVAL
rect=$(awk '/GSVAL mission_rect/ {print $3, $4, $5, $6}' "$BUILD.run.log")
[[ -n "$rect" && "$rect" != "none" ]] || { echo "no mission rectangle"; exit 1; }
PYTHONNOUSERSITE=1 /usr/bin/python3 -I - "$OUT/gs_wf_auto_exit_full.png" "$OUT/gs_auto_exit.png" $rect <<'EOF'
import sys
from PIL import Image
src, dst, x, y, w, h = sys.argv[1], sys.argv[2], *map(int, sys.argv[3:7])
im = Image.open(src)
pad = 4   # the 2026-10-19 crop: the panel's full width, the status row through the step buttons
im.crop((0, max(0, y - pad), im.width, min(im.height, y + h + pad))).save(dst)
print("wrote", dst, im.crop((0, max(0, y - pad), im.width, min(im.height, y + h + pad))).size)
EOF
