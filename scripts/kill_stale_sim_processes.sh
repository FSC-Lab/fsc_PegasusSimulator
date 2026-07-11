#!/usr/bin/env bash
# Find (and optionally kill) lingering PX4 SITL / Isaac Sim processes left over from a
# crashed or partially-closed scenario launch. The start_*.sh scripts now kill each
# other's tmux pane on exit (see scripts/README.md), but this catches anything that
# slips through: processes started outside the launch scripts, or a tmux session that
# was killed/detached without its child processes actually dying with it.
#
# Usage:
#   ./scripts/kill_stale_sim_processes.sh          # scan and ask before killing
#   ./scripts/kill_stale_sim_processes.sh -y        # scan and kill without asking
#   ./scripts/kill_stale_sim_processes.sh --dry-run # scan only, never kill

set -uo pipefail  # no -e: keep scanning/reporting even if one pgrep/kill call fails

MODE="ask"
case "${1:-}" in
  -y|--yes)     MODE="yes" ;;
  --dry-run)    MODE="dry" ;;
esac

echo "Scanning for lingering PX4 SITL / Isaac Sim processes..."
echo

# The actual PX4 SITL binary — matches regardless of which PX4_DIR it was built under.
PX4_PIDS="$(pgrep -f 'build/px4_sitl_default/bin/px4' || true)"

# Isaac Sim's underlying Omniverse Kit process, however it was launched (python_r_fsc.sh
# execs into it, so by the time it's running the cmdline shows the kit binary, not the
# wrapper script name — matching both here in case that assumption is wrong on some setup).
ISAAC_PIDS="$(pgrep -f 'isaacsim|/kit/kit|python_r_fsc\.sh' || true)"

FOUND=0

if [[ -n "$PX4_PIDS" ]]; then
  FOUND=1
  echo "PX4 SITL:"
  # shellcheck disable=SC2086
  ps -o pid,%cpu,etime,cmd -p $(echo $PX4_PIDS | tr '\n' ',' | sed 's/,$//')
  echo
fi

if [[ -n "$ISAAC_PIDS" ]]; then
  FOUND=1
  echo "Isaac Sim:"
  # shellcheck disable=SC2086
  ps -o pid,%cpu,etime,cmd -p $(echo $ISAAC_PIDS | tr '\n' ',' | sed 's/,$//')
  echo
fi

SESSION_PRESENT=0
if command -v tmux >/dev/null 2>&1 && tmux has-session -t px4_isaac 2>/dev/null; then
  SESSION_PRESENT=1
  FOUND=1
  echo "tmux session 'px4_isaac' is still present."
  echo
fi

if [[ $FOUND -eq 0 ]]; then
  echo "Nothing found — no lingering PX4/Isaac Sim processes or tmux session."
  exit 0
fi

if [[ "$MODE" == "dry" ]]; then
  echo "(--dry-run: not killing anything)"
  exit 0
fi

if [[ "$MODE" == "ask" ]]; then
  read -r -p "Kill all of the above? [y/N] " REPLY
  if [[ ! "$REPLY" =~ ^[Yy]$ ]]; then
    echo "Aborted, nothing killed."
    exit 0
  fi
fi

for pid in $PX4_PIDS $ISAAC_PIDS; do
  echo "Killing PID $pid"
  kill "$pid" 2>/dev/null || true
done

if [[ $SESSION_PRESENT -eq 1 ]]; then
  echo "Killing tmux session 'px4_isaac'"
  tmux kill-session -t px4_isaac 2>/dev/null || true
fi

echo "Done."
