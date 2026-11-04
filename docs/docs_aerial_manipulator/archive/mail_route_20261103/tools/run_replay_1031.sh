#!/usr/bin/env bash
# The "no surprises" regression after the mail route (2026-11-03): today's planner and flight node
# replayed against the four 10-31 hardware mail-delivery bags, plus the route's wrist check.
#
#   run_replay_1031.sh [OUT_DIR]      (default ../runs/replay_1031)
#
#   [1] check_route_wrist_loopback.py -- the experiment route yaml commands the pick pose
#       (wrist 90 deg) at every letter's hold above the letter        (domain 93)
#   [2] replay_bag_planner.py, the yaml that FLEW (62cbf03), all four bags: the replayed
#       reference must equal the recorded one                           (domain 91)
#   [3] replay_bag_planner.py, today's committed mail yaml, 3cm: the same up to the place
#       approach, which is 0.10 m deeper (Insert Margin 0.15 -> 0.25) by design
#   [4] replay_bag_law.py, the flown yaml, 3cm TWICE (the repeat = the timing floor), then 4/5/6cm (domain 92)
#   [5] replay_bag_law.py, the experiment route yaml, 3cm (its controller section = the flown one)
# Each step first checks that no Isaac / PX4 / flight stack / build is running and stops if one is.
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OUT="${1:-$HERE/../runs/replay_1031}"; mkdir -p "$OUT"
BAGS=~/Desktop/keilan_work/ros2_bag_mail
CFG=~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2/config
MAIL=params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_mail_delivery.yaml
( cd ~/Workspaces/fsc_autopilot_ws/src/fsc_autopilot_ros2 && git show 62cbf03:config/$MAIL > "$OUT/flown_1031_$MAIL" \
  && git show HEAD:config/$MAIL > "$OUT/head_$MAIL" ) || { echo "could not extract the yamls" >&2; exit 2; }
FLOWN="$OUT/flown_1031_$MAIL"; HEADY="$OUT/head_$MAIL"
ROUTE="$CFG/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_mail_route.yaml"
source /opt/ros/humble/setup.bash
source ~/Workspaces/fsc_autopilot_ws/install/setup.bash
PY=/usr/bin/python3
export ROS_LOCALHOST_ONLY=1

busy() {   # someone else's simulation, stack or build
  pgrep -af 'isaacsim/kit/python|build/px4_sitl_default/bin/px4|MicroXRCEAgent|colcon build|start_.*(stack|sitl).*\.sh|run_mail|run_route' \
    | grep -v -E "pgrep|run_replay_1031" | head -3
}
guard() {
  local b; b="$(busy)"
  if [[ -n "$b" ]]; then
    echo "STOPPED before '$1': other work is running:" >&2; echo "$b" >&2; exit 75
  fi
}
step() { echo; echo "=================== $1 ($(date +%T))"; }

if [[ "${SKIP_ROUTE_CHECK:-0}" != 1 ]]; then
  guard "route wrist check"; step "[1] route wrist check (experiment route yaml)"
  ROS_DOMAIN_ID=93 $PY "$HERE/check_route_wrist_loopback.py" "$ROUTE" 2>&1 | grep -v "Opened database" | tee "$OUT/route_wrist.txt"
fi

for b in 3cm_20261031_170629 4cm_20261031_163410 5cm_20261031_161915 6cm_20261031_153032; do
  guard "planner replay $b"; step "[2] planner replay, flown yaml: $b"
  $PY "$HERE/replay_bag_planner.py" "$BAGS/flight_wb_l1_4d_mail_delivery_$b" --yaml "$FLOWN" --out "$OUT/planner_flown_${b%%_*}" --domain 91 2>&1 \
    | grep -v "Opened database" | tee "$OUT/planner_flown_${b%%_*}.txt" | tail -14
done

guard "planner replay today"; step "[3] planner replay, today's committed mail yaml: 3cm"
$PY "$HERE/replay_bag_planner.py" "$BAGS/flight_wb_l1_4d_mail_delivery_3cm_20261031_170629" --yaml "$HEADY" --out "$OUT/planner_head_3cm" --domain 91 2>&1 \
  | grep -v "Opened database" | tee "$OUT/planner_head_3cm.txt" | tail -14

for run in 3cm_20261031_170629:3cm_a 3cm_20261031_170629:3cm_b 4cm_20261031_163410:4cm 5cm_20261031_161915:5cm 6cm_20261031_153032:6cm; do
  b="${run%%:*}"; tag="${run##*:}"
  guard "law replay $tag"; step "[4] law replay, flown yaml: $tag"
  $PY "$HERE/replay_bag_law.py" "$BAGS/flight_wb_l1_4d_mail_delivery_$b" --yaml "$FLOWN" --out "$OUT/law_flown_$tag" --domain 92 2>&1 \
    | grep -v "Opened database" | tee "$OUT/law_flown_$tag.txt" | tail -32
done

guard "law replay route yaml"; step "[5] law replay, experiment route yaml: 3cm"
$PY "$HERE/replay_bag_law.py" "$BAGS/flight_wb_l1_4d_mail_delivery_3cm_20261031_170629" --yaml "$ROUTE" --out "$OUT/law_route_3cm" --domain 92 2>&1 \
  | grep -v "Opened database" | tee "$OUT/law_route_3cm.txt" | tail -32
echo; echo "done $(date +%T): $OUT"
