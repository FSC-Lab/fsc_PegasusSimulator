#!/usr/bin/env bash
#
# Fly the validated 30 m cascade mission on whatever model is currently up, then
# score it FROM THE BAG.  This is the A/B instrument for the tank work: run it on
# the slosh stack and on the baseline stack and compare the two TSV lines.
#
#   bash scripts/fly_slosh_mission.sh                  # name from the clock
#   NAME=slosh_1 DIST=800 CRUISE_T=70 bash scripts/fly_slosh_mission.sh
#
# The pre-flight sequence is not ceremony -- every step below cost a flight in
# the night campaign and is documented there:
#   * RESET THE EKF, but only once mocap is flowing.  A fresh PX4 refuses to arm
#     after a session of crashes ("High Accelerometer Bias"), and restarting
#     ekf2 into silence leaves it unconverged ("heading estimate not stable").
#   * WIDEN THE SAFE CONTROLLER'S ENVELOPE before flying downrange.  It refuses
#     targets outside radius_max/radius_local SILENTLY -- the aircraft simply
#     does not move, and the log still says "approach".
#   * VERIFY ARMED FROM PX4 ITSELF.  While disarmed PX4 inhibits the outputs and
#     every upstream indicator looks perfectly healthy; one trial spent its whole
#     climb window on the ground commanding 2x weight.
set -u
ROOT=/home/pieper/envs/fsc_PegasusSimulator
SESSION=${SESSION:-vtol_smc_hil}
NAME=${NAME:-flight_$(date +%H%M%S)}
BAG_DIR=${BAG_DIR:-$HOME/flight_bags/slosh}
ALT=${ALT:-30}; DIST=${DIST:-800}; CRUISE_T=${CRUISE_T:-70}
# GUIDE="T:cmd;T:cmd" flies the cascade's guidance protocol during the cruise (turns, loiter,
# rth); with an rth the aircraft stops itself over home, so land there: LAND_X/LAND_Y default
# to the target for the straight mission and to home (0, 0) when GUIDE has an rth.
GUIDE=${GUIDE:-}
case "$GUIDE" in *rth*) LAND_X=${LAND_X:-0}; LAND_Y=${LAND_Y:-0} ;; *) LAND_X=${LAND_X:-$DIST}; LAND_Y=${LAND_Y:-0} ;; esac
CLIMB_T=${CLIMB_T:-150}; CTRL=${CTRL:-cascade}
OUT=${OUT:-$BAG_DIR/$NAME.flight}
NODE_PY=${NODE_PY:-$HOME/envs/isaac_env/bin/python3}

say(){ echo "[fly_slosh $(date +%H:%M:%S)] $*"; }
set +u; source "$ROOT/scripts/rosenv.sh" 2>/dev/null; set -u

# --- 0. which model is flying? record it with the flight --------------------
MODEL=baseline_VTOL
tmux capture-pane -p -S -3000 -t "$SESSION:2-isaac" 2>/dev/null | grep -q "VTOL_SLOSH set" && MODEL=VTOLSlosh
say "model in the sim: $MODEL"

# --- 0.5 IS THE VEHICLE FIT?  (not: is the stack up) ------------------------
# The stack can be perfectly healthy while the aircraft is inverted, airborne, or
# a kilometre downrange from a previous flight -- and this driver will then
# happily "fly" a wreck and report 0.00 m/s for three minutes, which is exactly
# what happened on 2026-09-12: a full auto mission ran during stack boot, the
# aircraft departed in roll at 350 m, and the flight commanded afterwards was
# flown on the resulting wreck. Isaac does not destroy a crashed vehicle, so
# nothing downstream notices.
VS=$(timeout 60 /usr/bin/python3 "$ROOT/scripts/vehicle_state.py" 2>/dev/null | tail -1)
case "$VS" in
  ok*) say "vehicle: $VS" ;;
  *)   echo "[fly_slosh] !! VEHICLE NOT FIT TO FLY: $VS"
       echo "[fly_slosh]    Expected it at the origin, upright, on the ground and stopped."
       echo "[fly_slosh]    Something has already flown this aircraft -- check whether a"
       echo "[fly_slosh]    mission was run from the ground station, then restart the stack."
       echo "[fly_slosh]    FORCE_UNFIT=1 overrides (the flight will not be comparable)."
       [ "${FORCE_UNFIT:-0}" = "1" ] || exit 1 ;;
esac

# --- 0.7 ONE DRIVER PER FLIGHT ------------------------------------------------
# The ground station re-publishes its last-selected controller every second, forever
# (trajectory_player.py: "so a node that starts later still picks it up"). A selection
# made there while this script is climbing on `geometric` becomes a 1 Hz fight over
# active_controller -- the cascade re-engaged every second at 20 m and the climb
# stalled (bag slosh_20260912_181628). The scripted flight therefore closes the GS
# window; the next launch brings it back. GS_OFF=0 keeps it.
if [ "${GS_OFF:-1}" = "1" ] && tmux list-windows -t "$SESSION" 2>/dev/null | grep -q '9-groundstn'; then
  tmux kill-window -t "$SESSION:9-groundstn" 2>/dev/null
  for pid in $(pgrep -f single_drone_ground_control 2>/dev/null); do kill -9 "$pid" 2>/dev/null; done
  say "ground station closed for the scripted flight (GS_OFF=0 keeps it)"
fi

# --- 1. mocap, then the EKF -------------------------------------------------
say "waiting for mocap"
for _ in $(seq 1 20); do
  timeout 8 ros2 topic echo /uav_0/mocap --once >/dev/null 2>&1 && { say "mocap up"; break; }
  sleep 5
done
say "resetting the EKF"
tmux send-keys -t "$SESSION:1-px4" "ekf2 stop" Enter 2>/dev/null; sleep 3
tmux send-keys -t "$SESSION:1-px4" "ekf2 start" Enter 2>/dev/null; sleep 25

# --- 2. envelope + arming ---------------------------------------------------
for kv in "radius_max 8000.0" "radius_local 2000.0" "z_max 130.0" "auto_arm true"; do
  timeout 20 ros2 param set /geometric_hover_control ${kv} >/dev/null 2>&1
done
# CASC_SETTLE_S: how long the cascade holds its engage hover before transitioning
# (node default 2 s). 30 s turns the engage hover into the L1 experiment the offline
# gate runs with --hover_only: a standing pitch that has time to converge.
if [ -n "${CASC_SETTLE_S:-}" ]; then
  # ★ FORCE A DOUBLE AND READ THE REPLY. `ros2 param set` exits 0 even when the node
  # REJECTS the value -- and rclpy rejects an integer for a double parameter -- so
  # `settle_s 30` printed "Setting parameter failed", returned success, and the
  # cascade transitioned after its default 2 s (flight l1off_hov30). CLAUDE.md has
  # this trap on record for the launchers; it applies to every param set.
  V=$(printf '%s' "$CASC_SETTLE_S" | grep -q '\.' && echo "$CASC_SETTLE_S" || echo "$CASC_SETTLE_S.0")
  # the node that flies: cascade -> /nmpc_cascade_control, cascade_adaptive -> ..._adaptive
  CNODE=/nmpc_cascade_control; [ "$CTRL" != cascade ] && CNODE="/nmpc_cascade_control_${CTRL#cascade_}"
  R=$(timeout 20 ros2 param set "$CNODE" settle_s "$V" 2>&1)
  case "$R" in
    *successful*) say "cascade settle_s = $V s (engage hover held that long)" ;;
    *) echo "[fly_slosh] !! settle_s not applied: $R"; exit 1 ;;
  esac
fi
say "envelope widened; waiting for PX4 to report ARMED"
ARMED=0
for _ in $(seq 1 16); do
  timeout 12 ros2 topic echo /uav_0/fmu/out/vehicle_status_v1 --once 2>/dev/null \
    | grep -qE '^arming_state: 2' && { ARMED=1; break; }
  sleep 6
done
if [ "$ARMED" -eq 0 ]; then
  say "!! PX4 never armed -- not flying. Recent preflight messages:"
  tmux capture-pane -p -t "$SESSION:1-px4" -S -40 2>/dev/null | grep -iE 'preflight|arming denied' | tail -3
  exit 1
fi
say "ARMED"

# --- 3. fly -----------------------------------------------------------------
say "flying: $CTRL, alt $ALT, dist $DIST, cruise ${CRUISE_T}s, back-transition + land at ($LAND_X, $LAND_Y)${GUIDE:+; guide: $GUIDE}"
timeout 900 "$NODE_PY" "$ROOT/scripts/fly_transition.py" \
    --alt "$ALT" --dist "$DIST" --controller "$CTRL" --climb-time "$CLIMB_T" \
    --cruise-time "$CRUISE_T" --back-transition --land --land-x "$LAND_X" --land-y "$LAND_Y" \
    ${GUIDE:+--guide "$GUIDE"} \
    > "$OUT" 2>&1 &
FPID=$!
# ★ SUPERVISE. The stack has died mid-flight twice today and the driver flew the last
# mocap sample for minutes each time -- the freeze detector in fly_transition only
# runs in cruise. Here: every 10 s, is PX4 up, is Isaac up, and has the mocap sample
# changed? Three identical reads (30 s) or a dead process ends the flight with exit 2,
# which run_slosh_flight.sh treats as "tear down and go again".
STACK_DEAD=0; SAME=0; PREV=""
while kill -0 "$FPID" 2>/dev/null; do
  sleep 10
  if ! pgrep -x px4 >/dev/null 2>&1 || ! pgrep -f "03_vtol_px4_hil" >/dev/null 2>&1; then
    say "!! PX4 or Isaac is GONE mid-flight"; STACK_DEAD=1; break
  fi
  CUR=$(timeout 8 ros2 topic echo /uav_0/mocap --once 2>/dev/null | grep -A3 "position:" | tr -d ' \n')
  if [ -n "$CUR" ] && [ "$CUR" = "$PREV" ]; then SAME=$((SAME+1)); else SAME=0; fi
  PREV="$CUR"
  if [ "$SAME" -ge 3 ]; then say "!! SIMULATOR FROZEN (mocap unchanged for 30 s)"; STACK_DEAD=1; break; fi
done
if [ "$STACK_DEAD" = "1" ]; then
  kill -INT "$FPID" 2>/dev/null; sleep 2; kill -9 "$FPID" 2>/dev/null
  say "flight INVALID -- the stack died or froze. Exit 2 (re-run)."
  exit 2
fi
wait "$FPID"
say "flight driver exited; summary:"
tail -20 "$OUT"

# --- 4. close the bag and score it ------------------------------------------
# SIGINT, never -9: a SIGKILLed rosbag has no metadata.yaml.
say "closing the bag"
tmux send-keys -t "$SESSION:10-bag" C-c 2>/dev/null
for _ in $(seq 1 10); do sleep 1; pgrep -f "ros2 bag record" >/dev/null || break; done

BAG=$(ls -dt "$BAG_DIR"/*/ 2>/dev/null | head -1)
if [ -z "$BAG" ]; then say "!! no bag found in $BAG_DIR -- the flight is unscored"; exit 1; fi
say "scoring $BAG"
LERR=$(grep -oE "LANDING ERROR +[0-9.]+" "$OUT" 2>/dev/null | tail -1 | tr -dc '0-9.')
echo
# One results file for the campaign: the A/B is two lines in the same table, and
# it outlives the terminal this ran in.
RES=${RES:-$BAG_DIR/results.tsv}
[ -f "$RES" ] || printf 'trial\tparams\tverdict\twb_s\tpeakV\tmaxtilt\tmaxroll\tmaxyaw\tzmin\tzmax\tfails\tland_err\tbag\n' > "$RES"
printf 'trial\tparams\tverdict\twb_s\tpeakV\tmaxtilt\tmaxroll\tmaxyaw\tzmin\tzmax\tfails\tland_err\tbag\n'
/usr/bin/python3 "$ROOT/scripts/night_metrics.py" "${BAG%/}" "$NAME" "model=$MODEL water=${VTOL_TANK_KG:-0.502}kg${VTOL_TANK_R:+ R=${VTOL_TANK_R}m}${VTOL_TANK_DX:+ dx=${VTOL_TANK_DX}m}${GUIDE:+ guided}${CASC_NODE_PY:+ node=$(basename "$CASC_NODE_PY" .py)} ctrl=$CTRL${CASC_EST:+ est=$CASC_EST}${ADAPTIVE_NODE_PY:+ anode=$(basename "$ADAPTIVE_NODE_PY" .py)}${CASC_STATE:+ state=$CASC_STATE}${EKF_SOURCE:+ ekf=$EKF_SOURCE}${VTOL_GPS_HZ:+ gps_hz=$VTOL_GPS_HZ}" "${LERR:-}" | tee -a "$RES"
say "results -> $RES"
