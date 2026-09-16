#!/usr/bin/env bash

# The flight controller moved to fsc_autopilot_ros2. Resolve it through the
# workspace symlink so a future move of that repo needs no change here.
: "${FSC_CONTROLLER_DIR:=$HOME/ros2_ws/src/px4_vtol_direct_actuator}"
# Full HIL stack restart -- run this BEFORE EVERY FLIGHT TEST.
#
# WHY (all of these cost a flight in the 2026-09-07 campaign, and NONE of them
# announced itself -- each looked like a controller failure):
#   * PX4's SIM BATTERY drains to its floor and COM_LOW_BAT_ACT=3 (Land) fires:
#     the controller commands correct thrust while PX4 walks the vehicle down.
#   * The HIL link dies silently after a long session -- position frozen to four
#     decimals while the controller varies tilt and thrust; PX4 prints
#     "simulator_mavlink poll timeout" and Isaac holds the last stale controls.
#   * Controllers RETAIN their last reference across a restart, so the vehicle
#     bolts for a stale target the moment the link comes up (seen at 34 m/s).
#   * The vehicle drifts kilometres from origin over successive flights.
#
# ORDER MATTERS: kill nodes -> PX4 -> Isaac; then start Isaac (it waits for the
# heartbeat) and only THEN PX4, which connects to it.  Restarting Isaac alone
# leaves the PX4 EKF diverged.
set -u
SESSION=${SESSION:-vtol_smc_hil}
ROOT=/home/pieper/envs/fsc_PegasusSimulator
ISAAC_PY=/home/pieper/workspaces/isaacsim/python.sh
ISAAC_ROOT=/home/pieper/workspaces/isaacsim
PX4_BIN="$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4"
PX4_WD="$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs_vtol0"

say(){ echo "[restart_hil] $*"; }

# `--if-needed` skips the whole restart when the stack is verifiably healthy: link up, no poll
# timeouts, both controllers alive, no NMPC traceback.  The restart-every-time rule exists because
# STALE stacks cost four flights -- but a stack that passed these checks minutes ago is not stale,
# and restarting anyway costs ~3.5 min per data point.
if [ "${1:-}" = "--if-needed" ]; then
  # Delegate to scripts/hil_health.sh, which checks the STACK (link, poll timeouts, both
  # controllers, isaac/px4 alive, no NMPC traceback) *and* the VEHICLE (upright, low, stopped,
  # near origin).  The vehicle half matters: a healthy stack can still be holding an aircraft
  # that is inverted or parked a kilometre downrange from the last flight, and starting a
  # mission from there is not a validated case.
  if OUT=$(bash "$ROOT/scripts/hil_health.sh"); then
    echo "[restart_hil] $OUT"
    echo "[restart_hil] stack and vehicle are fit — SKIPPING the restart"
    echo "[restart_hil] READY."
    exit 0
  fi
  echo "[restart_hil] $OUT"
  echo "[restart_hil] not fit to fly — doing a full restart"
fi

# COLD START: this script only sends keys to EXISTING tmux windows, so with no session there
# is nothing to restart -- every send-keys fails silently and it reports the link dead.  Hand
# off to the full launcher instead.  (This branch was deleted once by an over-wide edit to the
# --if-needed block above; if the script ever reports "link not healthy" from a cold machine,
# check that it is still here.)
if ! tmux has-session -t "$SESSION" 2>/dev/null; then
  say "no '$SESSION' tmux session — cold-starting via start_vtol_smc_hil.sh"
  CONTROLLERS="${CONTROLLERS:-geometric nmpc_full cascade}" VTOL_SLOSH="${VTOL_SLOSH:-0}" VTOL_TANK="${VTOL_TANK:-1}" VTOL_TANK_KG="${VTOL_TANK_KG:-0.502}" VTOL_TANK_R="${VTOL_TANK_R:-}" VTOL_TANK_DX="${VTOL_TANK_DX:-0}" VTOL_CG_DX="${VTOL_CG_DX:-0}" CASC_L1="${CASC_L1:-true}" \
  CASC_NODE_PY="${CASC_NODE_PY:-}" \
    bash "$ROOT/scripts/start_vtol_smc_hil.sh" --here "${CFG:-pieper_machine}"
  RC=$?
  # ★ THE COLD PATH USED TO SKIP STEP 5 ENTIRELY.
  # Everything below this branch -- including the PX4 params that do not survive a
  # restart -- belongs to the WARM path, so a cold start came up with the sim
  # BATTERY live and COM_LOW_BAT_ACT at its default. That failsafe has already
  # cost this project a flight: the controller commands correct thrust while PX4
  # walks the aircraft down, and the only symptom is "battery warning (fast)"
  # scrolling in a pane nobody is reading. The launcher's own 8-relax window sets
  # the arming and geofence params but NOT these.
  # NOTE the launcher exits non-zero on a cold start (its last line is
  # `tmux attach-session`, which cannot succeed without a terminal), so this must
  # not be gated on RC.
  say "cold start: applying the PX4 params the launcher does not set"
  for p in "SIM_BAT_MIN_PCT 100" "COM_LOW_BAT_ACT 0" "BAT_LOW_THR 0.05" \
           "COM_DISARM_LAND 0" "COM_DISARM_PRFLT 0"; do
    tmux send-keys -t "$SESSION:1-px4" "param set $p" Enter 2>/dev/null; sleep 0.4
  done
  exit $RC
fi

say "1/6 killing controller nodes"
for pat in nmpc_full_smc_node nmpc_cascade_node geometric_hover_node smc_outer_node pubref.py; do
  for pid in $(pgrep -f "$pat" 2>/dev/null); do
    c=$(cat /proc/$pid/comm 2>/dev/null); case "$c" in python3|python) kill -9 "$pid" 2>/dev/null;; esac
  done
done
say "2/6 killing PX4 + Isaac"
for pid in $(pgrep -x px4 2>/dev/null); do kill -9 "$pid" 2>/dev/null; done
for pid in $(pgrep -f "03_vtol_px4_hil" 2>/dev/null); do
  c=$(cat /proc/$pid/comm 2>/dev/null); case "$c" in python3|python|kit) kill -9 "$pid" 2>/dev/null;; esac
done
sleep 6

say "3/6 starting Isaac (waits for the PX4 heartbeat)"
tmux kill-window -t "$SESSION:2-isaac" 2>/dev/null
tmux new-window -d -t "$SESSION:" -n 2-isaac; sleep 1
tmux send-keys -t "$SESSION:2-isaac" \
 "unset PYTHONHOME PYTHONPATH PYTHONEXE VIRTUAL_ENV VIRTUAL_ENV_PROMPT CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL; \
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; \
export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}:$ISAAC_ROOT/exts/isaacsim.ros2.bridge/humble/lib\"; \
VTOL_ROS2_STATE=1 VTOL_SLOSH=${VTOL_SLOSH:-0} VTOL_TANK=${VTOL_TANK:-1} VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} VTOL_TANK_R=${VTOL_TANK_R:-} VTOL_TANK_F1=${VTOL_TANK_F1:-} VTOL_TANK_ZETA=${VTOL_TANK_ZETA:-} VTOL_TANK_DX=${VTOL_TANK_DX:-0} VTOL_CG_DX=${VTOL_CG_DX:-0} VTOL_GPS_HZ=${VTOL_GPS_HZ:-} VTOL_GPS_SIGMA=${VTOL_GPS_SIGMA:-} VTOL_GPS_VSIGMA=${VTOL_GPS_VSIGMA:-} VTOL_GPS_WALK=${VTOL_GPS_WALK:-} VTOL_WIND=${VTOL_WIND:-} VTOL_TURB=${VTOL_TURB:-} VTOL_TURB_TAU=${VTOL_TURB_TAU:-} PEGASUS_SLOSH_DEBUG=${PEGASUS_SLOSH_DEBUG:-} PYTHONEXE=\"$ISAAC_ROOT/kit/python/bin/python3\" $ISAAC_PY $ROOT/application/vtol/03_vtol_px4_hil.py" Enter
# POLL, do not sleep a flat 100 s: Isaac usually prints its banner well before that and the
# fixed wait was pure dead time on every single test.
say "    waiting for Isaac to load (polling, <=140 s)"
for i in $(seq 1 70); do
  sleep 2
  tmux capture-pane -p -S -400 -t "$SESSION:2-isaac" 2>/dev/null \
    | grep -qE "Waiting for first hearbeat|Received first hearbeat|mavlink_backend" && break
done
say "    Isaac up after ~$((i*2)) s"

say "4/6 starting PX4"
tmux send-keys -t "$SESSION:1-px4" \
 "PX4_SYS_AUTOSTART=4101 PX4_SIM_MODEL=fsc_quad_tiltrotor PX4_UXRCE_DDS_NS=uav_0 \"$PX4_BIN\" -i 0 -w \"$PX4_WD\"" Enter
# poll for the PX4 shell prompt rather than a fixed 32 s
for i in $(seq 1 30); do
  sleep 2
  tmux capture-pane -p -S -60 -t "$SESSION:1-px4" 2>/dev/null | grep -q "pxh>" && break
done
say "    PX4 up after ~$((i*2)) s"

say "5/6 re-applying PX4 params (they do not all survive a restart)"
for p in "SIM_BAT_MIN_PCT 100" "COM_LOW_BAT_ACT 0" "BAT_LOW_THR 0.05" \
         "GF_ACTION 0" "GF_MAX_HOR_DIST 0" "GF_MAX_VER_DIST 0" \
         "COM_DISARM_LAND 0" "COM_DISARM_PRFLT 0"; do
  tmux send-keys -t "$SESSION:1-px4" "param set $p" Enter; sleep 0.4
done

say "6/6 verifying the link"
# RETRY, do not sample once: the MAVLink handshake completes well after PX4's shell is up,
# so a single check a few seconds in always fails ("Waiting for first hearbeat").  Accept
# either the backend tag (printed per HIL_ACTUATOR_CONTROLS frame when PEGASUS_HIL_DEBUG is
# on) or Isaac's own "Received first hearbeat".  NOTE `-S -N` for scrollback: `-S N` counts
# from the TOP of the history and silently matches nothing.
CTRL=0
for _ in $(seq 1 20); do
  sleep 3
  CTRL=$(tmux capture-pane -p -S -200 -t "$SESSION:2-isaac" 2>/dev/null \
         | grep -cE "VTOL_HIL vid=|Received first hearbeat")
  [ "$CTRL" -ge 1 ] && break
done
PT=$(tmux capture-pane -p -S -15 -t "$SESSION:1-px4" 2>/dev/null | grep -c "poll timeout")
echo "    px4 poll-timeouts (want 0): $PT"
echo "    isaac HIL link established (want >=1): $CTRL"
if [ "$PT" -gt 0 ] || [ "$CTRL" -lt 1 ]; then
  echo "    !! HIL LINK NOT HEALTHY -- do not fly; re-run this script"; exit 1
fi
say "7/8 restarting the SAFE controller (this script killed it)"
tmux send-keys -t "$SESSION:6-geometric" C-c 2>/dev/null; sleep 1
tmux send-keys -t "$SESSION:6-geometric" \
 "source $ROOT/scripts/rosenv.sh; VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} VTOL_TANK_F1=${VTOL_TANK_F1:-} VTOL_TANK_ZETA=${VTOL_TANK_ZETA:-} VTOL_TANK_DX=${VTOL_TANK_DX:-0} VTOL_TANK_DZ=${VTOL_TANK_DZ:-0} \"\$HOME/envs/isaac_env/bin/python3\" \
${FSC_CONTROLLER_DIR}/px4_vtol_direct_actuator/geometric_hover_node.py \
--ros-args -p px4_ns:=uav_0 -p auto_arm:=false -p controller_id:=geometric -p active_at_start:=true -p state_source:=${GEO_STATE:-mocap}" Enter
sleep 20

# NB: everything from the opening quote below to the final `Enter` is ONE QUOTED STRING sent
# into the tmux pane. A `#` comment placed inside it is not a shell comment -- it becomes part of
# the command the pane runs, and because the preceding line ends in `\`, it swallows the rest of
# the node's arguments. `bash -n` cannot see this: the whole thing is a single valid argument.
# Keep notes OUT here.
#
# rate_hz is 10, not 15: MEASURED, the node only achieves 10.0 Hz with the corrected reference
# (scripts/check_timing.py on bag rep1 -- LOOP median 101 ms against a 67 ms budget, 91% of ticks
# more than 25% late). Commanding a rate it cannot meet is worse than commanding a slower one:
# latency_s compensates for a FIXED period, so a silently slipping loop under-compensates delay on
# every tick, and an overrunning loop is a documented tumble cause here.
say "8/8 restarting the NMPC node"
tmux kill-window -t "$SESSION:6g-full" 2>/dev/null
tmux new-window -d -t "$SESSION:" -n 6g-full; sleep 1
tmux send-keys -t "$SESSION:6g-full" \
 "source $ROOT/scripts/rosenv.sh; VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} VTOL_TANK_F1=${VTOL_TANK_F1:-} VTOL_TANK_ZETA=${VTOL_TANK_ZETA:-} VTOL_TANK_DX=${VTOL_TANK_DX:-0} VTOL_TANK_DZ=${VTOL_TANK_DZ:-0} \"\$HOME/envs/isaac_env/bin/python3\" \
${FSC_CONTROLLER_DIR}/px4_vtol_direct_actuator/nmpc_full_smc_node.py \
--ros-args -p px4_ns:=uav_0 -p auto_arm:=false -p controller_id:=nmpc_full -p active_at_start:=false \
-p vcruise:=16.0 -p cap_deg:=90.0 -p ttr:=18.0 -p cruise_alt:=20.0 -p rate_hz:=10.0 -p latency_s:=0.12 \
-r /uav_0/fsc_autopilot_ros2/position_controller/reference:=/uav_0/fsc_autopilot_ros2/position_controller/reference_smc" Enter
sleep 35
TB=$(tmux capture-pane -p -S -40 -t "$SESSION:6g-full" 2>/dev/null | grep -ci traceback)
echo "    nmpc node tracebacks (want 0): $TB"
[ "$TB" -gt 0 ] && { echo "    !! node threw on startup"; exit 1; }
# ---------------------------------------------------------------------------------------------
# CASCADE: 6-state longitudinal NMPC outer + geometric inner (controller_id `cascade`).
# Runs ALONGSIDE nmpc_full as a second standby; the GS/active_controller topic picks between them.
#
# rate_hz is NOT set here -- the cascade has no single rate. Its inner loop runs on every mocap
# sample and its outer NMPC runs in a background thread at outer_hz (the solve is 9-12 ms on the
# bench, so 20 Hz is roughly 4x margin; the 19-state needed 27-93 ms and could not hold 15 Hz).
#
# kR_roll/kR_pitch stay at the node default 3.0. Offline across the latency range: kR 9.0 is on the
# ground at 7 s with 150 ms of latency while kR 3.0 flies the full 120 s, and at 100 ms kR 3.0 also
# has FEWER solve failures (0/2401 vs 5/4001) and settles on the exact +5.7 deg trim AoA. If HIL
# measures the true loop latency above ~150 ms, soften kR further rather than raising latency_s --
# the ceiling is the attitude loop's, not the solver's.
#
# latency_s is 0.049 = the MEASURED loop transport. The outer now ADDS its own solve time at
# runtime, so the total compensation tracks 49 ms + whatever IPOPT just took (10-24 ms nominal).
# It used to be 0.04, which was correct only by accident: predict6 quantised to whole outer
# periods, so 0.04 snapped UP to 50 ms. With sub-period compensation 0.04 means 40 ms and
# under-compensates. Do not put it back without also reverting predict6.
# (historical note) the old value came from cross-correlating the elevator command
# against the resulting pitch ACCELERATION over 20 s of steady HIL cruise (bag casc1) peaks
# at lag 30 ms. It was 0.12, inherited from the 19-state node, which over-compensates 4x --
# the outer then plans for a state the aircraft never reaches. Re-measure with
# scripts/casc_timing.py if the stack's rates change.
#
# As with the block above: keep notes OUT of the send-keys string, it is one single argument.
say "8b/8 starting the CASCADE node (standby)"
tmux kill-window -t "$SESSION:6h-cascade" 2>/dev/null
tmux new-window -d -t "$SESSION:" -n 6h-cascade; sleep 1
tmux send-keys -t "$SESSION:6h-cascade" \
 "source $ROOT/scripts/rosenv.sh; VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} VTOL_TANK_F1=${VTOL_TANK_F1:-} VTOL_TANK_ZETA=${VTOL_TANK_ZETA:-} VTOL_TANK_DX=${VTOL_TANK_DX:-0} VTOL_TANK_DZ=${VTOL_TANK_DZ:-0} CASC_EST=${CASC_EST:-steso} STESO_L=${STESO_L:-} STESO_LF=${STESO_LF:-} STESO_MODE=${STESO_MODE:-} STESO_EPS=${STESO_EPS:-} LESO_WO=${LESO_WO:-} LESO_WOF=${LESO_WOF:-} MDOB_K=${MDOB_K:-} MDOB_KF=${MDOB_KF:-} IMU_WC=${IMU_WC:-} IMU_DR=${IMU_DR:-} IMU_DELAY=${IMU_DELAY:-} SLOSH_PRED=${SLOSH_PRED:-} SLOSH_PRED_LAG=${SLOSH_PRED_LAG:-} SLOSH_PRED_CANCEL=${SLOSH_PRED_CANCEL:-} SLOSH_PRED_OUTER=${SLOSH_PRED_OUTER:-} SLOSH_PRED_UMAX=${SLOSH_PRED_UMAX:-} SLOSH_PRED_Q=${SLOSH_PRED_Q:-} SLOSH_PRED_RF=${SLOSH_PRED_RF:-} SLOSH_PRED_RM=${SLOSH_PRED_RM:-} SLOSH_PRED_WALL_S=${SLOSH_PRED_WALL_S:-} SLOSH_PRED_F1_SCALE=${SLOSH_PRED_F1_SCALE:-} \"\$HOME/envs/isaac_env/bin/python3\" \
${CASC_NODE_PY:-${FSC_CONTROLLER_DIR}/px4_vtol_direct_actuator/nmpc_cascade_node.py} \
--ros-args -p px4_ns:=uav_0 -p auto_arm:=false -p controller_id:=cascade -p active_at_start:=false \
-p vcruise:=16.0 -p cap_deg:=90.0 -p ttr:=10.0 -p cruise_alt:=30.0 -p outer_hz:=20.0 -p latency_s:=0.049 -p l1_on:=${CASC_L1:-true} -p state_source:=${CASC_STATE:-mocap} \
-r /uav_0/fsc_autopilot_ros2/position_controller/reference:=/uav_0/fsc_autopilot_ros2/position_controller/reference_smc" Enter
sleep 35
TC=$(tmux capture-pane -p -S -40 -t "$SESSION:6h-cascade" 2>/dev/null | grep -ci traceback)
echo "    cascade node tracebacks (want 0): $TC"
[ "$TC" -gt 0 ] && { echo "    !! cascade node threw on startup"; exit 1; }

# ---------------------------------------------------------------------------------------------
# 8c/8 THE ADAPTIVE CASCADE (2026-09-14): the STESO + slosh-predictor variant as its OWN
# controller, controller_id `cascade_adaptive`, node `nmpc_cascade_control_adaptive`, guidance on
# /uav_0/cascade_adaptive/{cmd,status}. Standby beside the base cascade: a standby cascade runs
# no inner step and no outer solve until active_controller names it, so two cost nothing. The
# GS's "Adaptive NMPC + Geo" button / ctrl choice select it. ADAPTIVE_NODE_PY overrides the file;
# ADAPTIVE=0 skips the window (or drop cascade_adaptive from CONTROLLERS).
# ---------------------------------------------------------------------------------------------
ADAPTIVE_NODE="${ADAPTIVE_NODE_PY:-${FSC_CONTROLLER_DIR}/px4_vtol_direct_actuator/nmpc_cascade_steso_node.py}"
if [ "${ADAPTIVE:-1}" != "0" ] && [ -f "$ADAPTIVE_NODE" ]; then
say "8c/8 starting the ADAPTIVE cascade (STESO + slosh predictor, standby)"
tmux kill-window -t "$SESSION:6i-adaptive" 2>/dev/null
tmux new-window -d -t "$SESSION:" -n 6i-adaptive; sleep 1
tmux send-keys -t "$SESSION:6i-adaptive" \
 "source $ROOT/scripts/rosenv.sh; VTOL_TANK_KG=${VTOL_TANK_KG:-0.502} VTOL_TANK_F1=${VTOL_TANK_F1:-} VTOL_TANK_ZETA=${VTOL_TANK_ZETA:-} VTOL_TANK_DX=${VTOL_TANK_DX:-0} VTOL_TANK_DZ=${VTOL_TANK_DZ:-0} CASC_EST=${CASC_EST:-steso} STESO_L=${STESO_L:-} STESO_LF=${STESO_LF:-} STESO_MODE=${STESO_MODE:-} STESO_EPS=${STESO_EPS:-} LESO_WO=${LESO_WO:-} LESO_WOF=${LESO_WOF:-} MDOB_K=${MDOB_K:-} MDOB_KF=${MDOB_KF:-} IMU_WC=${IMU_WC:-} IMU_DR=${IMU_DR:-} IMU_DELAY=${IMU_DELAY:-} SLOSH_PRED=${SLOSH_PRED:-} SLOSH_PRED_LAG=${SLOSH_PRED_LAG:-} SLOSH_PRED_CANCEL=${SLOSH_PRED_CANCEL:-} SLOSH_PRED_OUTER=${SLOSH_PRED_OUTER:-} SLOSH_PRED_UMAX=${SLOSH_PRED_UMAX:-} SLOSH_PRED_Q=${SLOSH_PRED_Q:-} SLOSH_PRED_RF=${SLOSH_PRED_RF:-} SLOSH_PRED_RM=${SLOSH_PRED_RM:-} SLOSH_PRED_WALL_S=${SLOSH_PRED_WALL_S:-} SLOSH_PRED_F1_SCALE=${SLOSH_PRED_F1_SCALE:-} \"\$HOME/envs/isaac_env/bin/python3\" \
$ADAPTIVE_NODE \
--ros-args -p px4_ns:=uav_0 -p auto_arm:=false -p controller_id:=cascade_adaptive -p active_at_start:=false \
-p vcruise:=16.0 -p cap_deg:=90.0 -p ttr:=10.0 -p cruise_alt:=30.0 -p outer_hz:=20.0 -p latency_s:=0.049 -p l1_on:=true -p state_source:=${CASC_STATE:-mocap} \
-r /uav_0/fsc_autopilot_ros2/position_controller/reference:=/uav_0/fsc_autopilot_ros2/position_controller/reference_smc" Enter
sleep 20
TC=$(tmux capture-pane -p -S -40 -t "$SESSION:6i-adaptive" 2>/dev/null | grep -ci traceback)
echo "    adaptive cascade tracebacks (want 0): $TC"
[ "$TC" -gt 0 ] && { echo "    !! adaptive cascade threw on startup"; exit 1; }
fi

# ---------------------------------------------------------------------------------------------
# RESTART THE BAG RECORDER, with a FRESH name.
#
# WHY: this script restarts nodes, PX4 and Isaac -- everything except the one thing that RECORDS
# the flight. The recorder's bag name is fixed when the launcher starts, so after a restart it is
# either still appending to the previous flight's bag or (if the session was rebuilt) gone
# entirely. Both happened: a flight flown after two restarts left NO bag at all, and the newest
# one on disk was an hour old from a different flight. Reading that as "the run" is precisely the
# instrumentation failure this project keeps paying for -- so "restart before every flight" has to
# include the recorder.
#
# SIGINT, NEVER kill -9: a SIGKILLed bag has no metadata.yaml and is UNREADABLE.
BAG_DIR=${BAG_DIR:-$HOME/flight_bags}   # not /tmp: it is wiped on reboot (twice, with every bag in it)
BAGNAME=${BAG:-casc_$(date +%H%M%S)}
say "8c/8 restarting the bag recorder -> $BAG_DIR/$BAGNAME"
if tmux list-windows -t "$SESSION" 2>/dev/null | grep -q '10-bag'; then
  tmux send-keys -t "$SESSION:10-bag" C-c 2>/dev/null
  sleep 3
fi
tmux kill-window -t "$SESSION:10-bag" 2>/dev/null
tmux new-window -d -t "$SESSION:" -n 10-bag; sleep 1
tmux send-keys -t "$SESSION:10-bag" \
 "source $ROOT/scripts/rosenv.sh; mkdir -p $BAG_DIR; cd $BAG_DIR; ros2 bag record -a -o $BAGNAME" Enter
sleep 4
# ★ the recorder must be RECORDING, not merely launched: 2026-09-15 a launcher abort upstream left
# no 10-bag window, READY was declared, the flight landed, and there was no bag to score it with
# (the results row then pointed at the PREVIOUS flight's bag). ros2 bag creates the directory at
# start; its absence after 4 s is a dead recorder.
if tmux capture-pane -p -S -20 -t "$SESSION:10-bag" 2>/dev/null | grep -qi 'error\|Failed' || [ ! -d "$BAG_DIR/$BAGNAME" ]; then
  echo "    !! bag recorder did not start ($BAG_DIR/$BAGNAME absent) -- the flight would NOT be recorded"; exit 1
else
  echo "    recording to $BAG_DIR/$BAGNAME"
fi

say "READY. Fresh stack, vehicle at origin, geometric + nmpc_full + cascade up, recording to $BAG_DIR/$BAGNAME."
