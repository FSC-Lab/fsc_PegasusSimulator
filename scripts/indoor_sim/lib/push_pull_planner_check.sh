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
# WB_SIM_PROFILE=box_pushing, or the planner binary predates push-and-pull and
# has no push_pull_* parameters at all), 2 when either node could not be read.
# A mismatch always wins over a failed read. (Adjust moves push_pull_land and
# push_pull_start_mark, so those two are not compared.)
# push_pull_push_time_s (2026-10-07): 24 s, not 12 -- a pull at 12 s failed 0/3.
# push_pull_world_anchor_on_handle (2026-10-07): a planner built before it has
# no such parameter, and with world_anchor_ready false would fly the whole grasp
# CoM-anchored (one run never centred) -- a key missing on the node is a mismatch.
# push_pull_turn_deg / push_pull_obstacle (2026-10-15, the valve-turning scene): a
# yaml that sets them is compared on them too (the push yaml does not, so its
# check is unchanged); a planner built before them has no such parameter.
# push_pull_push_ramp_s (2026-10-17, the cruise): a planner built before it would
# fly push_pull_push_time_s as the old min-snap move -- 1.6 x the cruise's peak
# speed -- so a key missing on the node is a mismatch.
# push_pull_push_mean_speed / _min_time_s (2026-10-21, THE PUSH SPEED): a straight push's
# time is the distance at this speed; a planner built before it would fly its built-in
# push_pull_push_time_s instead -- a key missing on the node is a mismatch.
# push_pull_mocap_guard_topic (2026-10-21, THE RAW-MOCAP GUARD, archive/
# mocap_safety_guard_20261021): the box-pushing yaml points the planner at the estimator's
# guard health (a confirmed mocap fault aborts into a hover); a planner built before it
# ignores the key -- a key missing on the node is a mismatch. A STRING key: compared exactly
# (every other key here is numeric / boolean and compared to 1e-9).
# The box guards / abort (2026-10-16) are compared too (2026-10-19): a planner built before
# them passed this check while ignoring them. PL_CHECK_REQUIRE: keys the RUNNING planner must
# declare whatever the yaml says -- push_pull_capture_max_yaw_spread_deg is the 2026-10-19
# build (the frozen / flipping box capture gates), which also implies push_pull/release (10-17).
# push_pull_box_gate_jump_m / _deg (2026-10-21, THE BOX SAMPLE GATE, archive/box_mocap_gate_20261021):
# a planner built before it lets an obj_0 mis-solve reach the box guards -- a key missing on the
# node is a mismatch.
# push_pull_abort_unbuckle (2026-10-21, archive/box_abort_unbuckle_20261021): a planner built
# before it holds a buckled arm through the whole abort -- a key missing on the node is a mismatch.
# push_pull_guard_q3_lookahead_s (2026-10-22, THE q3 RATE GUARD, archive/
# box_pushing_contact_stability_tune_20261021 section 6): a planner built before it catches the
# arm's fold only after q3 passed the limit -- a key missing on the node is a mismatch.
# push_pull_table_from_box / _table_size / _table_box_gap / _box_half_height / _start_middle_tol,
# push_pull_go_to_start_bypass / _bypass_clearance (2026-10-22, the table placed from the box and
# Go To Start round it): a planner built before them checks the room-fixed table and flies Go To
# Start straight over the table -- a key missing on the node is a mismatch.
# push_pull_guard_q2_max_deg (2026-10-24, the q2 wall in the q3 rate guard, archive/
# wb_box_pushing_flight_20261023): an older planner ignores it and keeps the plain rate rule.
# push_pull_handle_settle_* / push_pull_ready_max_speed / _ready_settle_s (2026-10-24 afternoon,
# the hand-over settle, archive/wb_box_pushing_flight_20261024 section 8): an older planner
# holds the claw in the room the moment Ready ends
# push_pull_valve_axis_offset (2026-10-28, the valve's mocap pivot at its markers' centroid,
# archive/valve_turning_v2_20261027): an older planner ignores it and turns about obj_0 --
# 72.6 mm off the shaft -- so a key missing on the node is a mismatch (the box yaml does not
# set it: not compared there)
PL_CHECK_KEYS="push_pull_ee_offset push_pull_push_pose_deg push_pull_table push_pull_push_distance push_pull_yaw_from_box_deg push_pull_push_time_s push_pull_push_mean_speed push_pull_push_min_time_s push_pull_push_ramp_s push_pull_world_anchor_ready push_pull_world_anchor_on_handle push_pull_turn_deg push_pull_obstacle push_pull_guard_box_lag push_pull_guard_box_tilt_deg push_pull_guard_q3_min_deg push_pull_box_half_length push_pull_abort_from_measured push_pull_guard_tilt_deg push_pull_guard_force_n push_pull_mocap_guard_topic push_pull_box_gate_jump_m push_pull_box_gate_jump_deg push_pull_abort_unbuckle push_pull_guard_q3_lookahead_s push_pull_guard_q2_max_deg push_pull_table_from_box push_pull_table_size push_pull_table_box_gap push_pull_box_half_height push_pull_start_middle_tol push_pull_go_to_start_bypass push_pull_bypass_clearance push_pull_handle_top_above_grasp push_pull_handle_settle_tol push_pull_handle_settle_axis_tol push_pull_handle_settle_max_speed push_pull_handle_settle_dwell_s push_pull_ready_max_speed push_pull_ready_settle_s push_pull_valve_axis_offset"
PL_CHECK_REQUIRE="${PL_CHECK_REQUIRE:-push_pull_capture_max_yaw_spread_deg}"

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
  PL_DUMP="$dump" PL_REQUIRE="$PL_CHECK_REQUIRE" /usr/bin/python3 - "$yaml" $PL_CHECK_KEYS <<'PY'
import os, sys
import yaml


def planner_params(doc):
    for k, v in (doc or {}).items():
        if isinstance(v, dict) and str(k).endswith("whole_body_trajectory_planner"):
            return v.get("ros__parameters", {})
    return {}


want = planner_params(yaml.safe_load(open(sys.argv[1])))
got = planner_params(yaml.safe_load(os.environ["PL_DUMP"]))


def differs(a, b):
    # strings (push_pull_mocap_guard_topic) exactly; numbers / booleans to 1e-9, as before
    if isinstance(a, str) or isinstance(b, str):
        return str(a) != str(b)
    return abs(float(a) - float(b)) > 1e-9


bad = []
for k in sys.argv[2:]:
    w, g = want.get(k), got.get(k)
    if w is None:
        continue                      # the node default is what the yaml means
    wl = w if isinstance(w, list) else [w]
    gl = g if isinstance(g, list) else [g]
    if g is None or len(wl) != len(gl) or any(differs(a, b) for a, b in zip(wl, gl)):
        bad.append(f"{k}: running {g} vs yaml {w}")
for k in os.environ.get("PL_REQUIRE", "").split():
    if k not in got:
        bad.append(f"the running planner has no {k} -- its build is older than the box-pushing "
                   f"safety changes (rebuild fsc_trajectory_planner)")
if bad:
    print("push-and-pull check: " + "; ".join(bad))
    sys.exit(1)
PY
  local rc=$?
  [[ $rc == 0 && $ctl_bad == 1 ]] && rc=1
  return $rc
}
