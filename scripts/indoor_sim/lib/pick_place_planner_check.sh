#!/usr/bin/env bash
# Sourced by the two pick-and-place scene wrappers. Defines
#
#   pick_place_planner_check <planner yaml> [<controller yaml>]
#
# With a controller yaml it also reads the RUNNING controller's vehicle_name
# (/uav_0/fsc_autopilot_ros2) and requires the one that yaml sets -- every
# generated pick-and-place controller yaml carries its own (since 2026-10-03
# the mirror carries the task block too, so the planner alone no longer tells
# the pick-and-place profile, with its own gains, from a plain mirror launch).
#
# which dumps the RUNNING planner's parameters ONCE and compares its arm poses
# and drone waypoints with the pick-and-place yaml the stack should have been
# started on. Returns 0 when they match, 1 when any differs (the stack was
# started without the pick-and-place planner block: the planner then flies
# its built-in defaults -- pick / place pose [0, 0, 0, 0], carry = the folded
# home pose, every yaw 0, so no clockwise turns), 2 when the planner could not
# be read. A mismatch always wins over a failed read.
#
# 2026-10-03: replaces a check on pick_place_approach_dz <= 0, which stopped
# detecting anything once that parameter's built-in default became 0.20 m.
# One `ros2 param dump` (with a 5 s spin and retries) rather than one
# `param get` per key: separate discoveries timed out one key in six.
# arm poses: compared exactly (Adjust never moves them). Drone waypoints
# [x, y, z, yaw]: z and the yaw RELATIVE TO pick_place_start -- Adjust writes a
# common x / y / yaw shift into all four, and the relative yaws are what make
# the clockwise turns.
PP_CHECK_KEYS="pick_place_pick_pose_deg pick_place_place_pose_deg pick_place_carry_pose_deg pick_place_start pick_place_place_start pick_place_land_start pick_place_land"

pick_place_planner_check() {
  local yaml="$1" ctl_yaml="${2:-}" node="${PP_CHECK_NODE:-/uav_0/whole_body_trajectory_planner}"
  local ctl_node="${PP_CHECK_CTL_NODE:-/uav_0/fsc_autopilot_ros2}" ctl_bad=0 want_name got_name
  local udp="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
  local dump="" i
  [[ -r "$yaml" ]] || { echo "pick-and-place check: planner yaml not readable: $yaml"; return 2; }
  for i in 1 2 3; do
    dump=$( ( [[ -r "$udp" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$udp"
              timeout 30 ros2 param dump "$node" --no-daemon --spin-time 5 2>/dev/null ) || true)
    [[ "$dump" == *pick_place_carry_pose_deg* ]] && break
    dump=""
  done
  [[ -n "$dump" ]] || { echo "pick-and-place check: could not dump the parameters of $node"; return 2; }
  if [[ -n "$ctl_yaml" ]]; then
    want_name=$(sed -nE 's/^[[:space:]]+vehicle_name:[[:space:]]*"?([^"#]*)"?.*/\1/p' "$ctl_yaml" | head -1 | xargs)
    for i in 1 2 3; do
      got_name=$( ( [[ -r "$udp" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$udp"
                    timeout 30 ros2 param get "$ctl_node" vehicle_name --no-daemon --spin-time 5 2>/dev/null ) \
                  | sed -nE 's/.*value is: *(.*)$/\1/p' | xargs)
      [[ -n "$got_name" ]] && break
    done
    if [[ -z "$got_name" ]]; then
      echo "pick-and-place check: could not read vehicle_name off $ctl_node"; return 2
    elif [[ "$got_name" != "$want_name" ]]; then
      echo "pick-and-place check: controller vehicle_name '$got_name' vs '$want_name' ($(basename "$ctl_yaml"))"
      ctl_bad=1
    fi
  fi
  PP_DUMP="$dump" /usr/bin/python3 - "$yaml" $PP_CHECK_KEYS <<'EOF'
import os, sys
import yaml


def planner_params(doc):
    for k, v in (doc or {}).items():
        if isinstance(v, dict) and str(k).endswith("whole_body_trajectory_planner"):
            return v.get("ros__parameters", {})
    return {}


want = planner_params(yaml.safe_load(open(sys.argv[1])))
got = planner_params(yaml.safe_load(os.environ["PP_DUMP"]))
WAYPOINTS = ("pick_place_start", "pick_place_place_start", "pick_place_land_start", "pick_place_land")


def view(d, k):
    v = d.get(k)
    if v is None or k not in WAYPOINTS:
        return v
    s = d.get("pick_place_start")
    return [v[2], v[3] - s[3]] if s is not None and len(v) >= 4 and len(s) >= 4 else v


bad = 0
for k in sys.argv[2:]:
    w, g = view(want, k), view(got, k)
    tag = " (z, yaw from Start)" if k in WAYPOINTS else ""
    if w is None:
        print(f"pick-and-place check: {k} is not in {os.path.basename(sys.argv[1])}"); bad = 1
    elif g is None:
        print(f"pick-and-place check: {k} not reported by the running planner"); bad = 1
    elif len(w) != len(g) or any(abs(float(a) - float(b)) > 1e-6 for a, b in zip(w, g)):
        print(f"pick-and-place check: {k}{tag} running {list(g)} vs yaml {list(w)}"); bad = 1
sys.exit(bad)
EOF
  local rc=$?
  (( rc == 0 && ctl_bad == 1 )) && rc=1
  return $rc
}
