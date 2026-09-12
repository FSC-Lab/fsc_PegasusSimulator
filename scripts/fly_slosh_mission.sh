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
say "flying: $CTRL, alt $ALT, dist $DIST, cruise ${CRUISE_T}s, back-transition + land"
timeout 900 "$NODE_PY" "$ROOT/scripts/fly_transition.py" \
    --alt "$ALT" --dist "$DIST" --controller "$CTRL" --climb-time "$CLIMB_T" \
    --cruise-time "$CRUISE_T" --back-transition --land --land-x "$DIST" --land-y 0 \
    > "$OUT" 2>&1
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
/usr/bin/python3 "$ROOT/scripts/night_metrics.py" "${BAG%/}" "$NAME" "model=$MODEL" "${LERR:-}" | tee -a "$RES"
say "results -> $RES"
