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
mkdir -p "$BAG_DIR"

say "2/4 model: $([ "$WANT_SLOSH" = 1 ] && echo 'VTOLSlosh (tank=None, phase 1)' || echo 'baseline VTOL')"
say "    branch $BRANCH @ $COMMIT$([ "$DIRTY" -gt 0 ] && echo " (+$DIRTY uncommitted)")"
say "    bags -> $BAG_DIR/$BAG"
if [ "$DRY" = "1" ]; then
  say "DRY RUN -- stopping here. Nothing was launched and NOTHING IS VERIFIED:"
  say "  would run: VTOL_SLOSH=$WANT_SLOSH BAG_DIR=$BAG_DIR BAG=$BAG \\"
  say "             bash $ROOT/scripts/restart_hil.sh ${ARGS[*]-}"
  say "  then verify Isaac logged the model banner before reporting READY."
  exit 0
fi

say "3/4 restarting the stack (this is restart_hil.sh; ~4 min cold)"
bash "$ROOT/scripts/restart_hil.sh" ${ARGS[@]+"${ARGS[@]}"} || {
  echo "[slosh_hil] !! restart_hil.sh failed -- see above. Nothing was verified."
  exit 1
}

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
if [ "$WANT_SLOSH" = "0" ] && [ "$BANNER" -ge 1 ]; then
  echo "[slosh_hil] !! ASKED FOR THE BASELINE AND GOT THE TANK MODEL (banner present)."
  echo "[slosh_hil]    Re-run without --if-needed to force a fresh Isaac."
  exit 1
fi

# provenance, next to the bags, because the tmux pane is gone by the time anyone asks
printf '%s  model=%-10s branch=%-18s commit=%-9s dirty=%-3s bag=%s\n' \
  "$(date '+%Y-%m-%d %H:%M:%S')" \
  "$([ "$WANT_SLOSH" = 1 ] && echo VTOLSlosh || echo VTOL)" \
  "$BRANCH" "$COMMIT" "$DIRTY" "$BAG_DIR/$BAG" >> "$LAUNCH_LOG"

say "READY -- $([ "$WANT_SLOSH" = 1 ] && echo 'TANK MODEL (VTOLSlosh)' || echo 'baseline VTOL') confirmed in Isaac."
say "        bag $BAG_DIR/$BAG   provenance appended to $LAUNCH_LOG"
say "        tear down with: bash $ROOT/scripts/kill_slosh_hil.sh"
