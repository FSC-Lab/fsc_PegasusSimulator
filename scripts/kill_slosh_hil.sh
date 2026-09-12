#!/usr/bin/env bash
#
# Tear down the HIL stack, with the bag closed cleanly and the teardown VERIFIED.
#
#   bash scripts/kill_slosh_hil.sh
#
# It is a wrapper around scripts/kill_hil.sh, not a copy of it: the process
# pattern list has ONE home, because a controller added to one list and not the
# other survives teardown and co-publishes into the next flight (nmpc_so3_cascade
# and pubref.py each did exactly that).  What this adds is the two things the
# plain teardown gets wrong:
#
#   1. IT CLOSES THE BAG WITH SIGINT FIRST.  kill_hil.sh matches "ros2 bag record"
#      and kill -9s it, and a SIGKILLed rosbag has NO metadata.yaml and cannot be
#      opened with `ros2 bag` at all -- which is why every bag in ~/flight_bags is
#      db3-only today.  The recorder gets a Ctrl-C and time to finalise before
#      anything is killed.
#   2. IT CHECKS THAT THE STACK IS ACTUALLY DEAD.  kill_hil.sh PRINTS survivors and
#      exits 0 regardless; kills also silently no-op under an automation sandbox.
#      A teardown that reports success while PX4 is still up is how a "fresh" stack
#      turns out to be the previous flight's.  This exits non-zero if anything is
#      left, so a script that chains on it stops.
set -u
ROOT=/home/pieper/envs/fsc_PegasusSimulator
SESSION=${SESSION:-vtol_smc_hil}
BAG_DIR=${BAG_DIR:-$HOME/flight_bags/slosh}

say(){ echo "[kill_slosh] $*"; }

# --- 0. never from inside the session being killed --------------------------
# kill_hil.sh ends with `tmux kill-server`, which would take this shell with it
# and skip every check below.
if [ -n "${TMUX:-}" ] && [ "${FORCE:-0}" != "1" ]; then
  echo "[kill_slosh] !! running inside tmux. kill_hil.sh calls 'tmux kill-server',"
  echo "[kill_slosh]    which kills THIS shell before the verification runs."
  echo "[kill_slosh]    Run it from a plain terminal, or FORCE=1 to override."
  exit 1
fi

# --- 1. close the bag properly ---------------------------------------------
if tmux list-windows -t "$SESSION" 2>/dev/null | grep -q '10-bag'; then
  say "1/3 SIGINT to the bag recorder (a SIGKILLed bag is unreadable)"
  tmux send-keys -t "$SESSION:10-bag" C-c 2>/dev/null
  for _ in $(seq 1 10); do
    sleep 1
    pgrep -f "ros2 bag record" >/dev/null 2>&1 || break
  done
  if pgrep -f "ros2 bag record" >/dev/null 2>&1; then
    say "    recorder still running after 10 s -- it will be killed, and THAT BAG"
    say "    WILL HAVE NO metadata.yaml"
  else
    say "    recorder stopped"
  fi
else
  say "1/3 no bag recorder window"
fi

NEWEST=$(ls -dt "$BAG_DIR"/*/ 2>/dev/null | head -1)
if [ -n "$NEWEST" ]; then
  if [ -f "$NEWEST/metadata.yaml" ]; then
    say "    newest bag $(basename "$NEWEST"): metadata.yaml present (ros2 bag can open it)"
  else
    say "    newest bag $(basename "$NEWEST"): NO metadata.yaml -- read the .db3 directly"
  fi
fi

# --- 2. the one teardown ----------------------------------------------------
say "2/3 kill_hil.sh"
bash "$ROOT/scripts/kill_hil.sh"

# --- 3. verify, and FAIL if anything survived -------------------------------
say "3/3 verifying"
sleep 1
LEFT=""
for p in "bin/px4" "03_vtol_px4_hil" "isaacsim" "MicroXRCEAgent" \
         "geometric_hover_node" "nmpc_cascade_node" "nmpc_full_smc_node" \
         "smc_outer_node" "ros2 bag record" "pubref.py"; do
  for pid in $(pgrep -f -- "$p" 2>/dev/null); do
    # A SHELL whose command line merely CONTAINS the pattern is not a survivor --
    # an editor, a grep, or the very script that writes these patterns out will
    # match `pgrep -f` and make a clean teardown look incomplete forever.  The
    # real processes are px4, python and kit; filter on comm, as restart_hil.sh
    # does when it picks which pids to kill.
    c=$(cat /proc/"$pid"/comm 2>/dev/null) || continue
    case "$c" in
      bash|sh|dash|zsh|pgrep|grep|tail|less|vim|nvim|nano|node|claude) continue ;;
    esac
    echo "    STILL ALIVE: $p -> pid $pid ($c)"
    LEFT="$LEFT $p"
  done
done
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "    STILL ALIVE: tmux session $SESSION"
  LEFT="$LEFT tmux/$SESSION"
fi

if [ -n "$LEFT" ]; then
  echo "[kill_slosh] !! TEARDOWN INCOMPLETE:$LEFT"
  echo "[kill_slosh]    Do NOT launch on top of this -- the next flight would inherit it."
  echo "[kill_slosh]    If kills are silently no-opping, you are in a sandboxed shell;"
  echo "[kill_slosh]    re-run it from a real terminal."
  exit 1
fi
say "done -- stack is down, nothing left."
