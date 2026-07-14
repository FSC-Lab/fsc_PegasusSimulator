#!/usr/bin/env bash
# Applies and saves the X650-specific PX4 rate/attitude gain tuning to a running PX4 SITL
# instance's pxh> console, via tmux send-keys. This exists because adding realistic rotor
# spin-up lag (LaggedQuadraticThrustCurve, lambda=10.51 1/s) to the X650 thrust curve made it
# unable to fly on PX4's generic none_iris default gains (diverging rate/attitude oscillation) -
# see CLAUDE.md's "X650 PX4 gain tuning" section for the full derivation/diagnostic history.
#
# Values here are the empirically-confirmed-working combination (2026-07-14), tested against
# the true CAD-derived body inertia (not mass-ratio-scaled) and lambda=10.51:
#   MC_ROLLRATE_K = MC_PITCHRATE_K = MC_YAWRATE_K = 0.3   (stock default: 1.0)
#   MC_ROLL_P     = MC_PITCH_P                    = 3.25  (stock default: 6.5)
#   MC_YAW_P      = 1.4                                    (stock default: 2.8)
#
# Usage: apply_x650_px4_gains.sh <tmux_session> <tmux_pane> [delay_seconds]
# Meant to be backgrounded (`&`) from an X650 launch script right after the PX4 pane is created -
# delay_seconds must be long enough for PX4 to have already booted to its pxh> prompt, since
# tmux send-keys has no way to detect readiness and keys sent before the prompt exists are
# silently dropped.
set -euo pipefail

SESSION="$1"
PANE="$2"
DELAY="${3:-8}"

sleep "$DELAY"

tmux send-keys -t "$SESSION:$PANE" "param set MC_ROLLRATE_K 0.3" Enter
sleep 0.3
tmux send-keys -t "$SESSION:$PANE" "param set MC_PITCHRATE_K 0.3" Enter
sleep 0.3
tmux send-keys -t "$SESSION:$PANE" "param set MC_YAWRATE_K 0.3" Enter
sleep 0.3
tmux send-keys -t "$SESSION:$PANE" "param set MC_ROLL_P 3.25" Enter
sleep 0.3
tmux send-keys -t "$SESSION:$PANE" "param set MC_PITCH_P 3.25" Enter
sleep 0.3
tmux send-keys -t "$SESSION:$PANE" "param set MC_YAW_P 1.4" Enter
sleep 0.3
tmux send-keys -t "$SESSION:$PANE" "param save" Enter
