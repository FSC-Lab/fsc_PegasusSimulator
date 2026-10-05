#!/usr/bin/env bash
# Sourced by the push-and-pull scene wrapper. Defines
#
#   push_pull_planner_check <yaml>
#
# which dumps the RUNNING planner's parameters ONCE and compares its
# push-and-pull block -- the EE offset, the push pose, the table keep-out, the
# push distance, the yaw relation to the box -- with the push-and-pull yaml the
# stack should have been started on, and reads the RUNNING controller's
# vehicle_name (/uav_0/fsc_autopilot_ros2), which that yaml sets. Returns 0 when
# they match, 1 when any differs (the stack was started without
# WB_SIM_PROFILE=push_pull, or the planner binary predates push-and-pull and
# has no push_pull_* parameters at all), 2 when either node could not be read.
# A mismatch always wins over a failed read. (Adjust moves push_pull_land and
# push_pull_start_mark, so those two are not compared.)
PL_CHECK_KEYS="push_pull_ee_offset push_pull_push_pose_deg push_pull_table push_pull_push_distance push_pull_yaw_from_box_deg"

push_pull_planner_check() {
  local yaml="$1" node="${PL_CHECK_NODE:-/uav_0/whole_body_trajectory_planner}"
  local ctl_node="${PL_CHECK_CTL_NODE:-/uav_0/fsc_autopilot_ros2}" ctl_bad=0 want_name got_name
  local udp="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
  local dump="" i
  [[ -r "$yaml" ]] || { echo "push-and-pull check: yaml not readable: $yaml"; return 2; }
  for i in 1 2 3; do
    dump=$( ( [[ -r "$udp" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$udp"
              timeout 30 ros2 param dump "$node" --no-daemon --spin-time 5 2>/dev/null ) || true)
    [[ "$dump" == *ee_traj_lap_time* ]] && break
    dump=""
  done
  [[ -n "$dump" ]] || { echo "push-and-pull check: could not dump the parameters of $node"; return 2; }
  if [[ "$dump" != *push_pull_ee_offset* ]]; then
    echo "push-and-pull check: the running planner has NO push_pull_* parameters -- its binary predates push-and-pull (rebuild fsc_trajectory_planner)"
    return 1
  fi
  want_name=$(sed -nE 's/^[[:space:]]+vehicle_name:[[:space:]]*"?([^"#]*)"?.*/\1/p' "$yaml" | head -1 | xargs)
  for i in 1 2 3; do
    got_name=$( ( [[ -r "$udp" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="$udp"
                  timeout 30 ros2 param get "$ctl_node" vehicle_name --no-daemon --spin-time 5 2>/dev/null ) \
                | sed -nE 's/.*value is: *(.*)$/\1/p' | xargs)
    [[ -n "$got_name" ]] && break
  done
  if [[ -z "$got_name" ]]; then
    echo "push-and-pull check: could not read vehicle_name off $ctl_node"; return 2
  elif [[ "$got_name" != "$want_name" ]]; then
    echo "push-and-pull check: controller vehicle_name '$got_name' vs '$want_name' ($(basename "$yaml"))"
    ctl_bad=1
  fi
  PL_DUMP="$dump" /usr/bin/python3 - "$yaml" $PL_CHECK_KEYS <<'PY'
import os, sys
import yaml


def planner_params(doc):
    for k, v in (doc or {}).items():
        if isinstance(v, dict) and str(k).endswith("whole_body_trajectory_planner"):
            return v.get("ros__parameters", {})
    return {}


want = planner_params(yaml.safe_load(open(sys.argv[1])))
got = planner_params(yaml.safe_load(os.environ["PL_DUMP"]))
bad = []
for k in sys.argv[2:]:
    w, g = want.get(k), got.get(k)
    if w is None:
        continue                      # the node default is what the yaml means
    wl = w if isinstance(w, list) else [w]
    gl = g if isinstance(g, list) else [g]
    if g is None or len(wl) != len(gl) or any(abs(float(a) - float(b)) > 1e-9 for a, b in zip(wl, gl)):
        bad.append(f"{k}: running {g} vs yaml {w}")
if bad:
    print("push-and-pull check: " + "; ".join(bad))
    sys.exit(1)
PY
  local rc=$?
  [[ $rc == 0 && $ctl_bad == 1 ]] && rc=1
  return $rc
}
