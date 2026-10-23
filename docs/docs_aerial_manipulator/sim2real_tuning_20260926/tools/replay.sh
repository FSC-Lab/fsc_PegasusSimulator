#!/usr/bin/env bash
# Replay one hardware flight in Isaac on the MIRROR plant (sim2real_tuning_20260926).
#
#   replay.sh <f18|a2|a3> <tag> [machine-config]
#
#   f18  the 0918 mission: DIRECT hold, arm out (+0.03 x, -0.07 z), home, base
#        +0.61 m x, arm out again, home. Raw-mocap stack (l1_4d), the campaign
#        driver's --mission list.
#   a2   the 0924 F2 circle: r 0.5 m, 24 s lap, one lap, fold 60, q2 30 +- 10
#        at 48 s, time scale 1. Fused stack (l1_4d_fused), the planner's own
#        EE-trajectory driver.
#   a3   the 0924 F3 circle: fold 55, q2 25 +- 15 at 12 s. Fused stack.
#
# Per flight it exports that day's plant (kf at DIRECT entry + sag, table 1.6
# of the mirror yaml) over the yaml's reference values, and -- because Isaac
# runs at RTF < 1 while the law and planner run on the wall clock -- scales
# the planner's kinematic bounds by RTF (a TEMPORARY edit of the mirror yaml,
# restored on exit) and requests EE-trajectory time scale s = RTF, so the
# PLANT sees the flown pace. Everything else is the mirror yaml as committed.
#
# Records a rosbag of the hardware flights' topic set plus Isaac ground
# truth, so wb_l1_4d_flight_20260924/tools/extract_bag.py and this
# directory's compare.py run on it unchanged. Measures RTF from the bag.
set -uo pipefail
FLIGHT="${1:?f18|a2|a3}"; TAG="${2:?tag}"; CFG="${3:-shiqi_machine}"
# RTF: env > the machine config's SIM_RTF > 0.48. Was a hard-coded 0.48 (measured 0.477 on the f18_r1
# attempt); since 2026-10-01 shiqi_machine runs at 1.0 under the real-time pacer (SIM_REALTIME=1,
# Command.md 7.23), where everything below collapses to the identity (lags unscaled, s = 1).
_CONF_RTF="$(grep -oP '^SIM_RTF=\K[0-9.]+' "$(dirname -- "${BASH_SOURCE[0]}")/../../../../scripts/config/${CFG}.conf" 2>/dev/null || true)"
RTF="${RTF:-${_CONF_RTF:-0.48}}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PEG="$(cd -- "$HERE/../../.." && pwd)"
CFGDIR="$HOME/ros2_ws/src/fsc_autopilot_ros2/config"
Y="$CFGDIR/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml"
mkdir -p "$HERE/logs" "$HERE/bags" "$HERE/runs"

# WB_REPLAY_PROFILE=robustness flies the ROBUSTNESS plant instead (the stress
# gate a tuned candidate must also pass): the _sim_robustness.yaml plant and
# allocator stress (+17.6 % kf belief, mass/inertia x1.10, CoM 10/10/5 mm, arm
# x1.05) with the HARDWARE law overlaid (WB_REPLAY_OVERLAY_HW=1, default in that
# profile), so the gate tests the gains that would fly, not the file's old ones.
# WB_REPLAY_GAINS="key=value,..." then overrides any law key in either profile.
PROFILE="${WB_REPLAY_PROFILE:-mirror}"
if [[ "$PROFILE" == robustness ]]; then
  Y="$CFGDIR/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_robustness.yaml"
  export WB_SIM_PROFILE=robustness
  export WB_REPLAY_OVERLAY_HW="${WB_REPLAY_OVERLAY_HW:-1}"
else
  export WB_SIM_PROFILE=mirror
fi
case "$FLIGHT" in
  f18) RIG=l1_4d;       export PEGASUS_PLANT_KF_SCALE=1.066 PEGASUS_PLANT_KF_SAG_PER_MIN=0.034 ;;
  # a2: kf at LIFT-OFF, not at the DIRECT hold -- the SAFETY UDE found -3.0 N on
  # the bench map (kf 4.17e-05 = x1.030); at the hold value 0.997 the sim cannot
  # leave the ground (bench map 4.1 N short, UDE gated below 0.4 m: attempt a2_r1).
  a2)  RIG=l1_4d_fused
       # the robustness plant keeps its own kf (x1.0); only the mirror carries F2's pack state
       [[ "$PROFILE" == robustness ]] || export PEGASUS_PLANT_KF_SCALE=1.030 PEGASUS_PLANT_KF_SAG_PER_MIN=0.040 ;;
  a3)  RIG=l1_4d_fused; export PEGASUS_PLANT_KF_SCALE=1.062 PEGASUS_PLANT_KF_SAG_PER_MIN=0.037 ;;
  *) echo "unknown flight $FLIGHT"; exit 2 ;;
esac

# PLANT TIME CONSTANTS THE CONTROLLER PERCEIVES. The law runs on the wall clock,
# so a lag of tau plant-seconds looks like tau/RTF to it: the MN4010 rotor lag
# (99.7 ms) became ~210 ms and the arm's 48 ms Present-Velocity lag ~100 ms on
# the first attempt, and the attitude loop -- delay-margin limited at ~32 ms on
# hardware timing -- diverged in a growing roll oscillation. Express those two
# lags in WALL time so the controller sees what it saw on hardware (the
# rigid-body dynamics stay in plant time; that residual is documented). The
# yaml keeps the physical, plant-time values.
export PEGASUS_PLANT_ROTOR_LAMBDA="$(python3 -c "print(round(10.0265/$RTF, 4))")"
if [[ "$PROFILE" == mirror ]]; then
  export PEGASUS_ARM_VEL_LAG_S="$(python3 -c "print(round(0.048*$RTF, 4))")"
else
  export PEGASUS_ARM_VEL_LAG_S=0.0       # the robustness plant has no Present-Velocity model
fi
export PEGASUS_ARM_CURRENT_NOISE_BW_HZ="$(python3 -c "print(round(5.0/$RTF, 3))")"
echo "[replay] wall-clock lags: rotor lambda $PEGASUS_PLANT_ROTOR_LAMBDA 1/s, arm vel lag $PEGASUS_ARM_VEL_LAG_S s, noise bw $PEGASUS_ARM_CURRENT_NOISE_BW_HZ Hz (RTF $RTF)"

BK="$HERE/logs/yaml_backup_${TAG}_${PROFILE}.yaml"
cp -p "$Y" "$BK"
restore() { cp -p "$BK" "$Y"; echo "[replay] $PROFILE yaml restored"; }
trap restore EXIT

# RTF-scaled planner bounds (wall-clock planner, half-speed plant): rates and
# accelerations x RTF, times / RTF. Only the whole_body_trajectory_planner
# section is touched; the values printed here are what the run flew.
python3 - "$Y" "$RTF" "$FLIGHT" <<'EOF'
import re, sys
p, rtf, fl = sys.argv[1], float(sys.argv[2]), sys.argv[3]
s = open(p).read()
head, sep, tail = s.partition("/**/whole_body_trajectory_planner:")
def sub(key, f):
    global tail
    m = re.search(rf"^(\s*{key}:\s*)([0-9.]+)", tail, re.M)
    v = float(m.group(2)); nv = f(v)
    # ALWAYS a float literal: "55" is an integer to rclcpp and the planner
    # refuses it for a double parameter and exits (a3 attempt a3_jdiag_obs_k80).
    txt = f"{nv:g}"
    if re.fullmatch(r"-?\d+", txt):
        txt += ".0"
    tail = tail[:m.start(2)] + txt + tail[m.end(2):]
    print(f"[replay] {key}: {v:g} -> {nv:g}")
# The EE trajectory's s_max is the yaw-rate bound (w_max), and its pace is set
# by the requested s (= RTF, below), so for the circle replays only the
# translational bounds are scaled; scaling w_max there quartered the run's
# pace (attempt a2_r2: s_max 0.259, T 222 s wall).
# s_max IS the CoM/EE-speed bound (a2_r3: v_max x0.48 -> s_max 0.208), so the
# circle replays keep every planner bound and set the pace with s alone; only
# the go-to-start transition then runs ~2x fast in plant time (documented).
if fl == "f18":
    for k in ("v_max", "a_max", "w_max", "dw_max"):
        sub(k, lambda v: v * rtf)
    for k in ("t_min", "t_max"):
        sub(k, lambda v: v / rtf)
else:
    print("[replay] circle: planner bounds unchanged; pace from s = RTF")
# AS-FLOWN CONTROLLER (2026-09-26): the mirror's section 2 follows the hardware
# yaml, which moved to K_y/D_y 80/24, the joint-diagonal armature + rescaled
# M_r_d and the observer-velocity topic on the evening of 2026-09-26; the
# 0918-0924 flights were flown with the previous values. WB_REPLAY_ASFLOWN=1
# restores those for a sim-to-real replay (temporary edit, restored on exit).
import os
if os.environ.get("WB_REPLAY_ASFLOWN", "0") == "1":
    for k, nv in (("wb_mrd_x", 0.116522), ("wb_mrd_y", 0.136107), ("wb_mrd_z", 0.125102),
                  ("wb_ky_x", 20.0), ("wb_ky_y", 20.0), ("wb_ky_z", 20.0),
                  ("wb_dy_x", 12.0), ("wb_dy_y", 12.0), ("wb_dy_z", 12.0)):
        m = re.search(rf"^(\s*{k}:\s*)([0-9.]+)", head, re.M)
        head = head[:m.start(2)] + f"{nv:g}" + head[m.end(2):]
        print(f"[replay] as-flown {k} = {nv:g}")
    head = re.sub(r"^(\s*wb_armature_joint_diag:\s*)true", r"\1false", head, flags=re.M)
    head = re.sub(r'^(\s*wb_arm_velocity_topic:\s*)"[^"]*"', r'\1""', head, flags=re.M)
    print("[replay] as-flown: link armature, joint_states velocity")
def put(key, val, where):
    """Set a law key in the head (before the planner section); insert it after
    wb_mrd_z if the file does not carry it. Floats stay float literals."""
    global head
    if isinstance(val, bool):
        txt = "true" if val else "false"
    elif isinstance(val, str):
        txt = '"' + val + '"'
    else:
        txt = f"{float(val):g}"
        if re.fullmatch(r"-?\d+", txt):
            txt += ".0"
    m = re.search(rf"^(\s*{key}:[ \t]*)(\S[^#\n]*?)([ \t]*(#.*)?)$", head, re.M)
    if m:
        head = head[:m.start(2)] + txt + head[m.end(2):]
    else:
        a = re.search(r"^(\s*)wb_mrd_z:.*$", head, re.M)
        head = head[:a.end()] + f"\n{a.group(1)}{key}: {txt}" + head[a.end():]
    print(f"[replay] {where} {key} = {txt}")
if os.environ.get("WB_REPLAY_OVERLAY_HW", "0") == "1":
    # the hardware law (= the mirror's section 2) on top of the robustness file;
    # its allocator, thrust map and base_com stay (they belong to the stress plant)
    for k, v in (("wb_mrd_x", 0.130710), ("wb_mrd_y", 0.135962), ("wb_mrd_z", 0.134261),
                 ("wb_ky_x", 80.0), ("wb_ky_y", 80.0), ("wb_ky_z", 80.0),
                 ("wb_dy_x", 24.0), ("wb_dy_y", 24.0), ("wb_dy_z", 24.0),
                 ("wb_armature_joint_diag", True), ("wb_armature_j1", 0.010), ("wb_armature_j2", 0.0194),
                 ("wb_armature_j3", 0.0097), ("wb_armature_j4", 0.0097),
                 ("wb_arm_velocity_topic", "fsc_open_manipulator/external_torque_controller/velocity_observer"),
                 ("wb_arm_velocity_timeout_s", 0.05), ("wb_u3_estimate_max_j4", 0.06)):
        put(k, v, "hw-overlay")
for kv in filter(None, os.environ.get("WB_REPLAY_GAINS", "").split(",")):
    k, v = kv.split("=")
    put(k.strip(), float(v), "gain")
# planner overrides (2026-09-27), e.g. WB_REPLAY_PLANNER="ee_traj_circle_radius=0.75"
for kv in filter(None, os.environ.get("WB_REPLAY_PLANNER", "").split(",")):
    k, v = kv.split("="); k = k.strip(); v = v.strip()
    if v in ("true", "false"):
        tail, nsub = re.subn(rf"^(\s*{k}:\s*)(true|false)", rf"\g<1>{v}", tail, flags=re.M)
        assert nsub == 1, k
        print(f"[replay] {k} -> {v}")
    else:
        sub(k, lambda old, nv=float(v): nv)
if fl == "a3":
    for k, nv in (("ee_traj_fold_deg", 55.0), ("ee_traj_q2_center_deg", 25.0),
                  ("ee_traj_q2_amp_deg", 15.0), ("ee_traj_q2_period_s", 12.0)):
        sub(k, lambda v, nv=nv: nv)
open(p, "w").write(head + sep + tail)
EOF

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"
set -u

# clean slate + Fast DDS shm + daemon (the shiqi-desktop traps)
"$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh" >/dev/null 2>&1
"$PEG/scripts/kill_stale_sim_processes.sh" -y >/dev/null 2>&1
sleep 5
timeout 30 fastdds shm clean >/dev/null 2>&1 || true
for f in /dev/shm/sem.fastrtps_port*_mutex; do [[ -e "$f" ]] || continue; fuser -s "$f" 2>/dev/null || rm -f "$f"; done
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true
sleep 1
timeout 10 ros2 daemon start >/dev/null 2>&1 || true

P=/uav_0
TOPICS=( $P/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug
  $P/fsc_autopilot_ros2/whole_body_direct_actuation/reference
  $P/fsc_autopilot_ros2/whole_body_direct_actuation/mode
  $P/fsc_autopilot_ros2/whole_body_direct_actuation/motors_debug
  $P/fsc_autopilot_ros2/controller_type $P/fsc_autopilot_ros2/vehicle_info
  $P/fsc_autopilot_ros2/position_controller/ude $P/fsc_autopilot_ros2/position_controller/state
  $P/fsc_autopilot_ros2/position_controller/reference $P/fsc_autopilot_ros2/attitude_setpoint_debug
  $P/state_estimator/local_position/odom $P/state_estimator/enu/imu/data
  $P/mocap $P/state/pose $P/state/twist_inertial
  $P/fsc_open_manipulator/joint_states
  $P/fsc_open_manipulator/external_torque_controller/law_debug
  $P/fsc_open_manipulator/external_torque_controller/joint_torque_command
  $P/fsc_open_manipulator/external_torque_controller/reference_joint_trajectory
  $P/fsc_open_manipulator/external_torque_controller/smoothed_reference_joint_trajectory
  $P/fsc_open_manipulator/external_torque_controller/velocity_observer
  $P/whole_body_planner/status $P/whole_body_planner/current_ee $P/whole_body_planner/current_ee_body
  $P/whole_body_planner/ee_target $P/whole_body_planner/pending_base
  $P/whole_body_planner/target_joints $P/whole_body_planner/viz_path $P/whole_body_planner/viz_pose
  $P/whole_body_planner/ee_trajectory/status $P/whole_body_planner/ee_trajectory/info
  $P/fmu/out/vehicle_odometry $P/fmu/in/vehicle_visual_odometry $P/fmu/out/vehicle_status_v1
  $P/fmu/out/battery_status $P/fmu/out/vehicle_attitude $P/fmu/out/sensor_combined
  $P/fmu/out/estimator_status_flags $P/fmu/in/actuator_motors /rosout )
rm -rf "$HERE/bags/$TAG"
FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/../q2_sine_sim_20260924/tools/fastdds_udp_only.xml" \
  ros2 bag record -o "$HERE/bags/$TAG" "${TOPICS[@]}" > "$HERE/logs/bag_${TAG}.log" 2>&1 &
REC=$!

# hold/settle times are WALL seconds; the flights' plant-time holds (10-18 s)
# become these at RTF.
W() { python3 -c "print(round($1/$RTF, 1))"; }
if [[ "$FLIGHT" == f18 ]]; then
  MISSION_ARGS="--mission ee:0.03,0,-0.07,0;home;base:0.61,0.02,0,0;ee:0.044,-0.022,-0.062,0;home"
  export WB_L1_MISSION=standard
  export WB_L1_DRIVER_ARGS="$MISSION_ARGS --hover-z 1.0 --direct-settle $(W 18) --hold-between $(W 10) --plan-timeout 40 --exec-timeout $(W 30) --land-z 0.35"
else
  export WB_L1_MISSION=ee_circle
  # the driver asks for a FRACTION of the planner's s_max; s_max is the yaw-rate
  # bound, 1.1312 at a 24 s lap and proportional to the lap time (WB_REPLAY_SMAX
  # overrides), so s = RTF flies the plant at the planned pace for any lap.
  export WB_L1_EE_SCALE="$(python3 -c "print(round($RTF/${WB_REPLAY_SMAX:-1.1312}, 6))")"
  export WB_L1_DRIVER_ARGS="--hover-z 1.0 --direct-settle $(W 9) --post-hold $(W 7)"
fi
echo "[replay] flight $FLIGHT profile $PROFILE rig $RIG RTF $RTF kf x${PEGASUS_PLANT_KF_SCALE:-yaml} sag ${PEGASUS_PLANT_KF_SAG_PER_MIN:-yaml}/min gains [${WB_REPLAY_GAINS:-}] planner [${WB_REPLAY_PLANNER:-}]"
echo "[replay] driver: $WB_L1_MISSION ${WB_L1_EE_SCALE:+scale $WB_L1_EE_SCALE} $WB_L1_DRIVER_ARGS"

WB_L1_OUT="$HERE/runs" WB_L1_CLIENT_DDS_PROFILE="$HERE/../q2_sine_sim_20260924/tools/fastdds_udp_only.xml" \
  "$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh" "$RIG" "$TAG" "$CFG"
rc=$?

kill -INT "$REC" 2>/dev/null; wait "$REC" 2>/dev/null
echo "[replay] driver rc=$rc; bag: $(du -sh "$HERE/bags/$TAG" 2>/dev/null | cut -f1)"
"$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh" >/dev/null 2>&1
"$PEG/scripts/kill_stale_sim_processes.sh" -y >/dev/null 2>&1
exit $rc
