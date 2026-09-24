#!/usr/bin/env bash
# Report whether the HIL stack AND the vehicle are fit to fly.  Prints one line and exits 0
# (fit) or 1 (not).  Used by restart_hil.sh --if-needed.
#
# The VEHICLE matters as much as the stack: a perfectly healthy stack can be holding an
# aircraft that is inverted, airborne, or a kilometre downrange from the previous flight.
# Starting a mission from there is not a validated case -- it is how the vehicle ended up
# upside down at (581, 1.4, 0.10) after two back-to-back missions on one stack.
SESSION=${SESSION:-vtol_smc_hil}
ROOT=/home/pieper/envs/fsc_PegasusSimulator
# NB: source the ROS env with nounset OFF.  ROS's setup.bash references unset variables
# (AMENT_TRACE_SETUP_FILES, COLCON_TRACE, ...), so `set -u` makes sourcing it fatal and the
# script dies silently right there.
set +u
source "$ROOT/scripts/rosenv.sh" 2>/dev/null
set -u

LK=$(tmux capture-pane -p -S -200 -t "$SESSION:2-isaac" 2>/dev/null | grep -cE "VTOL_HIL vid=|Received first hearbeat")
PT=$(tmux capture-pane -p -S -20  -t "$SESSION:1-px4"   2>/dev/null | grep -c  "poll timeout")
# Tracebacks in EITHER wing-borne pane, and the cascade node must be ALIVE, not merely have
# printed its banner: on 2026-09-13 (flight turns_loiter_rth_a1) the tank cascade printed its
# schedule banner and then died in its constructor; this check counted geometric+nmpc_full as
# "2/2", the launcher saw the banner, and the mission was flown by the geometric controller
# reporting "cruise" at 1 m/s. If a 6h-cascade window exists the node is required.
TB=$(for w in 6g-full 6h-cascade 6i-adaptive 6j-indi 6k-aindi; do tmux capture-pane -p -S -60 -t "$SESSION:$w" 2>/dev/null; done | grep -ci traceback)
NL=$(ros2 node list 2>/dev/null)
ND=$(echo "$NL" | grep -cE "geometric_hover_control|nmpc_full_smc_control")
NDW=2
if tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx 6h-cascade; then
  NDW=$((NDW + 1)); ND=$((ND + $(echo "$NL" | grep -cx "/nmpc_cascade_control")))
fi
# the adaptive cascade (STESO + slosh predictor, 2026-09-14) has its own node name
if tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx 6i-adaptive; then
  NDW=$((NDW + 1)); ND=$((ND + $(echo "$NL" | grep -cx "/nmpc_cascade_control_adaptive")))
fi
# the INDI cascade (2026-09-21) likewise
if tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx 6j-indi; then
  NDW=$((NDW + 1)); ND=$((ND + $(echo "$NL" | grep -cx "/nmpc_cascade_control_indi")))
fi
# Adaptive MPC + INDI (2026-09-23 night) likewise
if tmux list-windows -t "$SESSION" -F '#W' 2>/dev/null | grep -qx 6k-aindi; then
  NDW=$((NDW + 1)); ND=$((ND + $(echo "$NL" | grep -cx "/nmpc_cascade_control_aindi")))
fi
IS=$(pgrep -cf "03_vtol_px4_hil" 2>/dev/null | head -1); IS=${IS:-0}
PX=$(pgrep -cx px4 2>/dev/null | head -1); PX=${PX:-0}

VS=$(/usr/bin/python3 "$ROOT/scripts/vehicle_state.py" 2>/dev/null | tail -1)
[ -z "$VS" ] && VS="nostate"

echo "health: link=$LK poll_timeouts=$PT nodes=$ND/$NDW isaac=$IS px4=$PX traceback=$TB vehicle=$VS"
[ "$LK" -ge 1 ] && [ "$PT" -eq 0 ] && [ "$ND" -ge "$NDW" ] && [ "$IS" -ge 1 ] \
  && [ "$PX" -ge 1 ] && [ "$TB" -eq 0 ] && [ "$VS" = "ok" ]
