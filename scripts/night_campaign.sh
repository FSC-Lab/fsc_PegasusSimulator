#!/usr/bin/env bash
# Unattended flight-test campaign: run trials back-to-back, one parameter set each, and record
# a one-line verdict per trial so the morning's question ("which configuration flew?") is a
# single file rather than forty rosbags.
#
# WHY IT LOOKS LIKE THIS
#   * A FULL STACK RESTART BEFORE EVERY TRIAL is not optional. Four flights in the 2026-09-07
#     campaign were lost to stale-stack faults that each looked like a controller failure:
#     the PX4 sim battery hitting COM_LOW_BAT_ACT, a silently dead HIL link, a retained
#     reference sending the vehicle off at 34 m/s, and a controller deaf after a DDS
#     rediscovery. Unattended, a stale stack would poison every remaining trial.
#   * EVERY TRIAL IS VERIFIED BEFORE IT FLIES (nodes present, no traceback, link up, no poll
#     timeouts). A trial flown on a broken stack is worse than no trial: it produces a
#     plausible-looking bag that argues against a change that was never actually tested.
#   * ONE VARIABLE PER TRIAL where possible. This project has twice drawn a conclusion from a
#     run that changed two things at once (bag full_tr32) and had to unpick it later.
#
# Trials are read from a file, one per line:   <name> <p=v> <p=v> ...
# Results append to $OUT/results.tsv.
set -u
cd "$(dirname "$0")/.."
REPO=$PWD
TRIALS=${1:-scripts/night_trials.txt}
OUT=${OUT:-/tmp/night_campaign}
ALT=${ALT:-30}
DIST=${DIST:-800}
CRUISE_T=${CRUISE_T:-70}
# LAND=1: finish each trial by flying to (DIST, 0) and touching down there. The mission is
# not complete until the aircraft is ON THE GROUND WHERE IT WAS ASKED TO BE -- killing the
# forward velocity with flare + aft tilt exists precisely so it can stop somewhere specific.
LAND=${LAND:-1}
mkdir -p "$OUT"
RES="$OUT/results.tsv"
[ -f "$RES" ] || printf 'trial\tparams\tverdict\twb_s\tpeakV\tmaxtilt\tmaxroll\tmaxyaw\tzmin\tzmax\tfails\tland_err\tbag\n' > "$RES"

say() { echo "[$(date +%H:%M:%S)] $*"; }

health_line() { timeout 90 bash scripts/hil_health.sh 2>/dev/null | tail -1; }

wait_healthy() {   # $1 = attempts
  local n=${1:-20} h
  for _ in $(seq 1 "$n"); do
    h=$(health_line)
    # ★ THE VEHICLE MUST BE FIT, NOT JUST THE STACK.
    # wait_healthy checked nodes/traceback/poll_timeouts and IGNORED the `vehicle=` field that
    # hil_health already computes -- so a trial would happily launch from wherever the previous
    # one left the wreck. rep1 spent its entire 150 s climb window sitting on the ground at
    # (173.2, 16.8) with yaw -99, engaged at 13.8 m instead of 30, and then could never
    # transition because the settle gate (z < z0 - 1) is permanently true below the target
    # altitude. Every downstream number from such a trial is meaningless.
    case "$h" in
      *"nodes=2/2"*traceback=0*vehicle=ok*)
        case "$h" in *poll_timeouts=0*) echo "$h"; return 0;; esac;;
    esac
    sleep 15
  done
  echo "$h"; return 1
}

run_trial() {
  local name="$1"; shift
  local params="$*"
  say "=== TRIAL $name : ${params:-<defaults>}"

  # ★ DISK GUARD.  Rosbags are the single biggest unattended risk: /tmp/bags reached 69 GB in a
  # day of manual testing, and a full disk does not fail cleanly -- it takes PX4, Isaac and the
  # recorder down together and every remaining trial reports a stack fault instead of a result.
  local freeg
  freeg=$(df -BG --output=avail / | tail -1 | tr -dc '0-9')
  if [ "${freeg:-0}" -lt 20 ]; then
    say "ABORT CAMPAIGN - only ${freeg}G free"
    printf '%s\t%s\tDISK_FULL\t\t\t\t\t\t\t\t\t-\n' "$name" "$params" >> "$RES"
    exit 1
  fi

  # 1. full restart, then verify. Never fly an unverified stack.
  # ATTEMPT CAP: a trial that dies mid-restart never writes a result, so without this the
  # supervisor would relaunch into the same trial forever and the night would produce nothing.
  local acount=0
  [ -f "$OUT/$name.attempts" ] && acount=$(cat "$OUT/$name.attempts")
  acount=$((acount+1)); echo "$acount" > "$OUT/$name.attempts"
  if [ "$acount" -gt 3 ]; then
    say "GIVE UP on $name after $((acount-1)) attempts"
    printf '%s\t%s\tABORTED_RETRIES\t\t\t\t\t\t\t\t\t-\n' "$name" "$params" >> "$RES"
    return 1
  fi

  # ★ RESTART IN ITS OWN SESSION.  Run as a child, the cold-start path takes this script down
  # with it (observed 22:59 -- the stack came up healthy and the campaign vanished mid-restart).
  # setsid puts it in a separate session and process group, so whatever it kills on the way
  # cannot reach the campaign; we then simply WAIT FOR HEALTH rather than for the process.
  say "restarting the stack (attempt $acount, detached)"
  CONTROLLERS="geometric nmpc_full" BAG_ALL=0 setsid nohup timeout 900 bash scripts/restart_hil.sh \
      > "$OUT/$name.restart" 2>&1 < /dev/null &
  sleep 20
  local h
  if ! h=$(wait_healthy 24); then
    say "SKIP $name - stack unhealthy: $h"
    printf '%s\t%s\tSTACK_UNHEALTHY\t\t\t\t\t\t\t\t\t%s\n' "$name" "$params" "-" >> "$RES"
    return 1
  fi
  say "healthy: $h"

  # 2. arm. The operator normally owns arming; unattended it has to be automatic, and this
  #    is a simulator. It is set back to false by the campaign's teardown.
  set +u; source scripts/rosenv.sh 2>/dev/null
  # ★ ARM AND CONFIRM LIFT-OFF BEFORE FLYING.  Arming is not instantaneous: PX4 needs offboard
  # setpoints streaming and its own preflight to pass, and the first trial sat at z=0.1 m with
  # the controller commanding 70 N (2x weight) for the whole 45 s climb window -- output was
  # inhibited because it was not armed yet.  It lifted at ~55 s, long after the driver had given
  # up on the climb and moved on to engage a vehicle still on the ground.
  # ★ RESET THE EKF BEFORE ARMING.  A fresh PX4 still refuses to arm after a session of
  # crashes: "Preflight Fail: High Accelerometer Bias", then "Arming denied: Resolve system
  # health failures first", forever.  The bias estimate goes bad while the vehicle is being
  # spawned and settling, and nothing clears it on its own -- trial 1 sat on the ground for the
  # entire climb window commanding 70 N (2x weight) because output was inhibited.
  # `ekf2 stop; ekf2 start` after the stack has settled clears it; verified live, arming_state
  # went 1 (DISARMED) -> 2 (ARMED) immediately afterwards.
  # ★ RESET THE EKF ONLY ONCE IT HAS SOMETHING TO EAT.
  # A fixed sleep after the stack restart races the mocap/EV bridge coming up: restarting ekf2
  # into silence leaves it unconverged and PX4 then refuses to arm with "heading estimate not
  # stable" / "ekf2 missing data" (rep1). Wait for pose to actually be flowing first, then reset,
  # then give it time to converge. This race is re-run on EVERY trial because the stack is torn
  # down and rebuilt each time -- which is why it shows up so often.
  say "waiting for mocap before touching the EKF"
  local mok=0
  for _ in $(seq 1 20); do
    if timeout 8 ros2 topic echo /uav_0/mocap --once >/dev/null 2>&1; then mok=1; break; fi
    sleep 5
  done
  [ "$mok" -eq 1 ] || say "WARNING: no mocap; the EKF reset will probably not converge"
  say "resetting EKF"
  tmux send-keys -t vtol_smc_hil:1-px4 "ekf2 stop" Enter 2>/dev/null; sleep 3
  tmux send-keys -t vtol_smc_hil:1-px4 "ekf2 start" Enter 2>/dev/null; sleep 25

  # ★ WIDEN THE SAFE CONTROLLER'S ENVELOPE BEFORE FLYING DOWNRANGE.
  # It accepts a target only within radius_max of HOME or radius_local of the vehicle, and it
  # refuses SILENTLY -- the aircraft simply does not move. With the 300 m default, the landing
  # command to (800, 0) was ignored and the vehicle held station at (948, 13) at 30 m while the
  # log happily printed "approach" (trial bt_flare20). This is the same guard that twice blocked
  # the one controller whose job is to recover the aircraft.
  timeout 20 ros2 param set /geometric_hover_control radius_max 8000.0 >/dev/null 2>&1
  timeout 20 ros2 param set /geometric_hover_control radius_local 2000.0 >/dev/null 2>&1
  timeout 20 ros2 param set /geometric_hover_control z_max 130.0 >/dev/null 2>&1
  timeout 20 ros2 param set /geometric_hover_control auto_arm true >/dev/null 2>&1
  # VERIFY ARMED, from PX4 itself. "the controller is commanding thrust" is not evidence: while
  # disarmed PX4 inhibits the outputs and everything upstream looks perfectly healthy.
  local armed=0
  for _ in $(seq 1 16); do
    if timeout 12 ros2 topic echo /uav_0/fmu/out/vehicle_status_v1 --once 2>/dev/null \
         | grep -qE '^arming_state: 2'; then armed=1; break; fi
    sleep 6
  done
  if [ "$armed" -eq 0 ]; then
    say "SKIP $name - PX4 refused to arm"
    tmux capture-pane -p -t vtol_smc_hil:1-px4 -S -40 2>/dev/null \
      | grep -iE 'preflight|arming denied' | tail -3 >> "$OUT/$name.armfail"
    printf '%s\t%s\tNEVER_ARMED\t\t\t\t\t\t\t\t\t-\n' "$name" "$params" >> "$RES"
    return 1
  fi
  say "ARMED"

  # 3. trial parameters (all live; none of these force a solver rebuild)
  local ok_p=0 bad_p=""
  for kv in $params; do
    local k=${kv%%=*} v=${kv##*=}
    if timeout 20 ros2 param set /nmpc_full_smc_control "$k" "$v" 2>/dev/null | grep -qi success; then
      ok_p=$((ok_p+1))
    else
      bad_p="$bad_p $k"
    fi
  done
  if [ -n "$bad_p" ]; then
    say "SKIP $name - parameters refused:$bad_p"
    printf '%s\t%s\tPARAM_REFUSED\t\t\t\t\t\t\t\t\t%s\n' "$name" "$params" "-" >> "$RES"
    return 1
  fi
  say "applied $ok_p parameter(s)"

  # 4. fly
  # ★ RECORD OUR OWN BAG.  The launcher's recorder window belongs to the stack, and a restart
  # does not necessarily give it a fresh one -- trial 1 was scored NO_BAG because the newest bag
  # on disk was from a stack started half an hour earlier.  Own the recording per trial.
  # ★ ONE RECORDER, AND A FRESH PATH.  A leftover recorder still holding the sqlite file makes
  # the new one die with "database is locked" -- bt_aft25 recorded 18 s of a 300 s flight and
  # scored NO_TRANSITION on a mission that actually flew 68.5 s wing-borne and landed on target.
  # SIGINT (never -9) any recorder still alive, then record to a path nothing else can own.
  for rp in $(pgrep -f 'ros2 bag record' 2>/dev/null); do kill -INT "$rp" 2>/dev/null; done
  sleep 4
  local bag="$OUT/bags/${name}_$(date +%H%M%S)"
  mkdir -p "$OUT/bags"
  setsid nohup ros2 bag record -o "$bag" \
      /rosout /uav_0/mocap \
      /uav_0/fmu/in/actuator_motors /uav_0/fmu/in/actuator_servos \
      > "$OUT/$name.bagrec" 2>&1 < /dev/null &
  local rec=$!
  sleep 3
  timeout 900 "$HOME/envs/isaac_env/bin/python3" scripts/fly_transition.py \
      --alt "$ALT" --dist "$DIST" --controller nmpc_full --climb-time 150 \
      --cruise-time "$CRUISE_T" --back-transition \
      ${LAND:+--land --land-x $DIST --land-y 0} > "$OUT/$name.flight" 2>&1
  # SIGINT, never -9: a bag killed with -9 is unreadable (no metadata.yaml, sqlite I/O error)
  kill -INT "$rec" 2>/dev/null; sleep 4; kill -9 "$rec" 2>/dev/null
  [ -d "$bag" ] || bag="-"

  # 5. verdict from the BAG, not from the driver's own summary
  # /usr/bin/python3 explicitly: the shebang resolves to the isaac_env venv, which has no
  # rosidl_runtime_py, so the scorer failed with ModuleNotFoundError on a good flight.
  local lerr
  lerr=$(grep -oE "LANDING ERROR +[0-9.]+" "$OUT/$name.flight" 2>/dev/null | tail -1 | tr -dc "0-9.")
  /usr/bin/python3 "$REPO/scripts/night_metrics.py" "$bag" "$name" "$params" "${lerr:-}" >> "$RES" 2>>"$OUT/$name.metrics.err"
  say "result: $(tail -1 "$RES" | cut -f3-)"
}

# ★ PID FILE, NOT pgrep.  The supervisor used `pgrep -f "bash scripts/night_campaign.sh"`,
# which matches ANY process whose command line merely CONTAINS that string -- including a
# monitoring shell that greps for it.  That is exactly what happened: the watchdog matched the
# watcher, concluded the campaign was alive, and sat there for 22 minutes while nothing ran.
echo $$ > "$OUT/campaign.pid"
trap 'rm -f "$OUT/campaign.pid"' EXIT
say "campaign starting (pid $$); trials from $TRIALS; results -> $RES"
while read -r line; do
  case "$line" in ''|'#'*) continue;; esac
  # ★ RESUMABLE.  The cold-start path inside restart_hil.sh takes its caller down with it
  # (observed 22:59: the stack came up healthy and the campaign process vanished mid-restart).
  # Rather than depend on surviving that, record every finished trial and skip it on re-entry,
  # so a supervisor can simply relaunch this script and the campaign carries on where it was.
  tname=${line%% *}
  if cut -f1 "$RES" 2>/dev/null | grep -qx "$tname"; then
    say "skip $tname (already recorded)"
    continue
  fi
  # shellcheck disable=SC2086
  run_trial $line
done < "$TRIALS"
say "campaign finished"
set +u; source scripts/rosenv.sh 2>/dev/null
timeout 20 ros2 param set /geometric_hover_control auto_arm false >/dev/null 2>&1
say "auto_arm set back to false"
