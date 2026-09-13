#!/usr/bin/env bash
#
# Start the HIL stack on the FREE-SURFACE TANK model (branch `slosh`).
#
#   scripts/start_slosh_hil.sh                 # full restart, tank model
#   scripts/start_slosh_hil.sh --if-needed     # skip the restart if the stack is fit
#   scripts/start_slosh_hil.sh --baseline      # the SAME path, baseline VTOL -- the A/B
#   scripts/start_slosh_hil.sh --dry-run       # checks only; launches nothing
#
# WHY THIS EXISTS AND IS NOT JUST `VTOL_SLOSH=1 restart_hil.sh`
# ------------------------------------------------------------
# Because the env var can silently fail to arrive.  The launchers pass Isaac's
# environment INLINE on a tmux send-keys command line -- a pane does not inherit
# the exporting shell's environment -- so a forgotten forward means the stack
# comes up on the BASELINE aircraft while the operator believes the tank is
# flying, and every number from that flight is mislabelled.  This project has
# already paid for mislabelled flights: a glide read as wing-borne flight, a
# frozen sim read as a successful manoeuvre, a whole run flown by the wrong
# controller after a stale DDS discovery.  So this script does not trust the
# variable -- it VERIFIES that the model it asked for is the model that came up,
# and refuses to report READY otherwise.
#
# It also runs the phase-1 gates first (2 s, no Isaac) and puts bags somewhere
# that survives a reboot -- /tmp/bags is the default everywhere else and is how
# the post-extraction verification bag was lost.
set -u
ROOT=/home/pieper/envs/fsc_PegasusSimulator
SESSION=${SESSION:-vtol_smc_hil}          # ONE HIL stack at a time: PX4 4560 and the DDS
                                          # domain are fixed, so the tank stack reuses the
                                          # session (and all the tooling keyed to that name).
                                          # The MODEL is what differs, and that is verified below.
# overridable so the guard itself can be tested without switching branches
MODEL_PY=${MODEL_PY:-$ROOT/extensions/pegasus.simulator/pegasus/simulator/logic/vehicles/vtol_slosh.py}
LAUNCH_LOG=${LAUNCH_LOG:-$HOME/flight_bags/LAUNCH_LOG.txt}

say(){ echo "[slosh_hil] $*"; }

WANT_SLOSH=1
DRY=0
ARGS=()
for a in "$@"; do
  case "$a" in
    --baseline) WANT_SLOSH=0 ;;
    # --dry-run runs every check that does not need a stack -- the branch guard,
    # the gates, the bag path -- and stops before touching Isaac or PX4.  It is
    # how this script itself is tested without burning four minutes, and how you
    # check a machine is ready to fly before committing to a run.
    --dry-run) DRY=1 ;;
    *) ARGS+=("$a") ;;
  esac
done

# --- 1. am I even on a branch that HAS the tank model? ----------------------
# `git checkout dev_branch_logical` and this file is gone; without this check the
# run would fall back to the baseline aircraft and look completely normal.
if [ "$WANT_SLOSH" = "1" ] && [ ! -f "$MODEL_PY" ]; then
  echo "[slosh_hil] !! $MODEL_PY does not exist."
  echo "[slosh_hil]    The tank model lives on branch 'slosh'. You are on "
  echo "[slosh_hil]    '$(git -C "$ROOT" branch --show-current)'.  git checkout slosh"
  exit 1
fi
BRANCH=$(git -C "$ROOT" branch --show-current 2>/dev/null)
COMMIT=$(git -C "$ROOT" rev-parse --short HEAD 2>/dev/null)
DIRTY=$(git -C "$ROOT" status --porcelain 2>/dev/null | wc -l)

# --- 2. the phase-1 gates, before burning 4 minutes on a stack --------------
if [ "${SKIP_GATES:-0}" != "1" ]; then
  say "1/4 gates: slosh scaffold + import"
  for t in test_slosh_scaffold test_slosh_import; do
    if ! OUT=$(/usr/bin/python3 "$ROOT/tests/$t.py" 2>&1); then
      echo "$OUT" | tail -12
      echo "[slosh_hil] !! $t FAILED -- not launching. (SKIP_GATES=1 overrides.)"
      exit 1
    fi
  done
  say "    both gates pass"
else
  say "1/4 gates SKIPPED (SKIP_GATES=1)"
fi

# --- 3. hand off to the one real launcher -----------------------------------
# Durable bag directory. Everything else defaults to /tmp/bags, which is wiped on
# reboot -- that is exactly how the 72.7 s post-extraction verification flight was
# lost, and a tank campaign that cannot be compared against its baseline is worthless.
export BAG_DIR=${BAG_DIR:-$HOME/flight_bags/slosh}
export BAG=${BAG:-$([ "$WANT_SLOSH" = 1 ] && echo slosh || echo base)_$(date +%Y%m%d_%H%M%S)}
export VTOL_SLOSH=$WANT_SLOSH
export VTOL_TANK=${VTOL_TANK:-1}
export VTOL_TANK_DX=${VTOL_TANK_DX:-0}
export VTOL_CG_DX=${VTOL_CG_DX:-0}
export CASC_L1=${CASC_L1:-false}
# The tank AIRCRAFT flies the tank CASCADE (nmpc_cascade_tank_node: the base node with
# the back-transition schedule replaced -- rotors upright before the aft tilt, flare
# capped under the CL peak). Same node name and controller_id, so nothing downstream
# changes. CASC_NODE_PY=<path> overrides; CASC_NODE_PY=base forces the base node.
CTRL_DIR=${FSC_CONTROLLER_DIR:-$HOME/ros2_ws/src/px4_vtol_direct_actuator}
if [ "$WANT_SLOSH" = "1" ] && [ "${VTOL_TANK:-1}" != "0" ]; then
  case "${CASC_NODE_PY:-}" in
    "")   export CASC_NODE_PY="$CTRL_DIR/px4_vtol_direct_actuator/nmpc_cascade_tank_node.py" ;;
    base) unset CASC_NODE_PY ;;
  esac
fi
say "    cascade: $(basename "${CASC_NODE_PY:-nmpc_cascade_node.py}")  l1_on=$CASC_L1  tank_dx=$VTOL_TANK_DX  cg_dx=$VTOL_CG_DX"
mkdir -p "$BAG_DIR"

say "2/4 model: $([ "$WANT_SLOSH" = 1 ] && echo "VTOLSlosh (VTOL_TANK=$VTOL_TANK)" || echo 'baseline VTOL')"
say "    branch $BRANCH @ $COMMIT$([ "$DIRTY" -gt 0 ] && echo " (+$DIRTY uncommitted)")"
say "    bags -> $BAG_DIR/$BAG"
if [ "$DRY" = "1" ]; then
  say "DRY RUN -- stopping here. Nothing was launched and NOTHING IS VERIFIED:"
  say "  would run: VTOL_SLOSH=$WANT_SLOSH BAG_DIR=$BAG_DIR BAG=$BAG \\"
  say "             bash $ROOT/scripts/restart_hil.sh ${ARGS[*]-}"
  say "  then verify Isaac logged the model banner before reporting READY."
  exit 0
fi

# ★ DETACHED, THEN POLL -- do not run restart_hil.sh as a plain child.
# On a COLD machine it hands off to start_vtol_smc_hil.sh, whose last line is
# `tmux attach-session`, and whose teardown kills processes in this process
# group: run as a child it takes THIS script down with it, so the verification
# below would never execute and the run would look like it simply vanished
# (night_campaign.sh hit exactly this at 22:59 and solved it the same way).
# setsid puts it in its own session; we then wait for its own READY line.
RESTART_LOG=${RESTART_LOG:-$BAG_DIR/restart_$(date +%H%M%S).log}
WAIT_S=${WAIT_S:-1200}
say "3/4 restarting the stack, detached (~4 min cold) -> $RESTART_LOG"
setsid nohup timeout "$WAIT_S" bash "$ROOT/scripts/restart_hil.sh" \
  ${ARGS[@]+"${ARGS[@]}"} > "$RESTART_LOG" 2>&1 < /dev/null &
RPID=$!

# WAIT ON HEALTH, NOT ON THE PROCESS, and accept either signal:
#   * warm path: restart_hil.sh prints "READY." after its own gates (link up, zero
#     poll timeouts, controller nodes alive, no traceback);
#   * COLD path: it hands off to start_vtol_smc_hil.sh, whose last line is
#     `tmux attach-session` -- which cannot succeed without a terminal, so the
#     launcher exits NON-ZERO and never prints READY even when the stack is
#     perfectly up. Waiting on the exit status there would fail every cold start.
# hil_health.sh is the authority in both cases: it checks the stack (link, poll
# timeouts, both controllers, no traceback) AND the vehicle (upright, low,
# stopped, near origin).
OK=0
for _ in $(seq 1 $((WAIT_S / 15))); do
  sleep 15
  if grep -q "READY\." "$RESTART_LOG" 2>/dev/null; then
    OK=1; say "    restart_hil.sh reported READY"; break
  fi
  if H=$(timeout 90 bash "$ROOT/scripts/hil_health.sh" 2>/dev/null); then
    OK=1; say "    healthy: $H"; break
  fi
  if grep -q "HIL LINK NOT HEALTHY\|threw on startup" "$RESTART_LOG" 2>/dev/null; then
    echo "[slosh_hil] !! the restart reported a fault:"
    grep -n "!!" "$RESTART_LOG" | tail -5
    exit 1
  fi
done
if [ "$OK" != "1" ]; then
  echo "[slosh_hil] !! timed out after ${WAIT_S}s with no healthy stack. Last health: ${H:-<none>}"
  echo "[slosh_hil]    restart log tail:"; tail -20 "$RESTART_LOG"
  exit 1
fi

# --- 4. VERIFY the model that actually came up ------------------------------
# 03_vtol_px4_hil.py logs this banner when, and only when, it spawns the tank model.
say "4/4 verifying which model Isaac spawned"
PANE=$(tmux capture-pane -p -S -3000 -t "$SESSION:2-isaac" 2>/dev/null)
BANNER=$(echo "$PANE" | grep -c "VTOL_SLOSH set")
if [ "$WANT_SLOSH" = "1" ] && [ "$BANNER" -lt 1 ]; then
  echo "[slosh_hil] !! ASKED FOR THE TANK MODEL AND DID NOT GET IT."
  echo "[slosh_hil]    Isaac never logged 'VTOL_SLOSH set -> VTOLSlosh model'."
  echo "[slosh_hil]    Check, in order:"
  echo "[slosh_hil]      * VTOL_SLOSH is forwarded on the Isaac command line in BOTH"
  echo "[slosh_hil]        restart_hil.sh and start_vtol_smc_hil.sh (a pane does not"
  echo "[slosh_hil]        inherit this shell's environment)"
  echo "[slosh_hil]      * --if-needed skipped the restart, so the RUNNING Isaac is an"
  echo "[slosh_hil]        older baseline one: re-run without --if-needed"
  echo "[slosh_hil]      * the scene imported fsc_cad_vtol_slosh_config (tmux window 2-isaac)"
  echo "[slosh_hil]    DO NOT FLY THIS AND CALL IT A TANK RESULT."
  exit 1
fi
# Phase 2: with the tank mounted the vehicle logs its mass fold at construction.
# Verify THAT too -- "VTOLSlosh with no tank" and "VTOLSlosh with the tank" are
# different aircraft, and the launcher must say which one it verified.
TANK_LINE=$(echo "$PANE" | grep -o "\[TANK\] dead mass.*" | tail -1 | cut -c1-90)
if [ "$WANT_SLOSH" = "1" ]; then
  if [ "${VTOL_TANK:-1}" != "0" ] && [ -z "$TANK_LINE" ]; then
    echo "[slosh_hil] !! the tank model came up but never logged '[TANK] dead mass' --"
    echo "[slosh_hil]    the tank was NOT mounted. Check VTOL_TANK and the scene."
    exit 1
  fi
  [ -n "$TANK_LINE" ] && say "    $TANK_LINE"
  # phase 3: asked for the free surface -> the banner must say so
  if [ "${VTOL_TANK:-1}" = "slosh" ]; then
    P3=$(echo "$PANE" | grep -o "SLOSH (phase 3, pendulum)" | tail -1)
    if [ -z "$P3" ]; then
      echo "[slosh_hil] !! VTOL_TANK=slosh but Isaac logged a dead-mass tank, not the pendulum."
      exit 1
    fi
    say "    free surface: $P3 confirmed in Isaac's log"
  fi
fi
if [ "$WANT_SLOSH" = "0" ] && [ "$BANNER" -ge 1 ]; then
  echo "[slosh_hil] !! ASKED FOR THE BASELINE AND GOT THE TANK MODEL (banner present)."
  echo "[slosh_hil]    Re-run without --if-needed to force a fresh Isaac."
  exit 1
fi

# ★ PIPE THE PANES TO FILES.  When the stack died on 2026-09-12 there was nothing
# to look at: tmux was gone and with it every pane's scrollback, so the cause is
# still unknown. A pane log costs nothing and is the only forensic trail a dead
# stack leaves.
for w in 2-isaac 1-px4 6h-cascade; do
  tmux pipe-pane -o -t "$SESSION:$w" "cat >> $BAG_DIR/pane_${w}_$(date +%H%M%S).log" 2>/dev/null
done
say "    pane logs -> $BAG_DIR/pane_*.log"

# and VERIFY which cascade came up: the variant prints its schedule banner on start.
CPANE=$(tmux capture-pane -p -S -400 -t "$SESSION:6h-cascade" 2>/dev/null)
CBANNER=$(echo "$CPANE" | grep -o "\[cascade_tank\] schedule:.*" | tail -1 | cut -c1-110)
case "${CASC_NODE_PY:-}" in
  *tank*)
    if [ -z "$CBANNER" ]; then
      echo "[slosh_hil] !! asked for the TANK cascade and its banner never appeared in 6h-cascade."
      echo "[slosh_hil]    The base cascade (or nothing) is running. Not READY."
      exit 1
    fi
    say "    $CBANNER" ;;
  *)
    [ -n "$CBANNER" ] && { echo "[slosh_hil] !! the TANK cascade is running but was not asked for."; exit 1; }
    say "    cascade: base node confirmed" ;;
esac

# provenance, next to the bags, because the tmux pane is gone by the time anyone asks
printf '%s  model=%-10s branch=%-18s commit=%-9s dirty=%-3s bag=%s\n' \
  "$(date '+%Y-%m-%d %H:%M:%S')" \
  "$([ "$WANT_SLOSH" = 1 ] && echo VTOLSlosh || echo VTOL)" \
  "$BRANCH" "$COMMIT" "$DIRTY" "$BAG_DIR/$BAG  cascade=$(basename "${CASC_NODE_PY:-nmpc_cascade_node.py}")" >> "$LAUNCH_LOG"

say "READY -- $([ "$WANT_SLOSH" = 1 ] && echo "TANK MODEL (VTOLSlosh, tank $([ -n "$TANK_LINE" ] && echo MOUNTED || echo none))" || echo 'baseline VTOL') confirmed in Isaac."
say "        bag $BAG_DIR/$BAG   provenance appended to $LAUNCH_LOG"
say "        tear down with: bash $ROOT/scripts/kill_slosh_hil.sh"
