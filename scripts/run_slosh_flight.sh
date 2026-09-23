#!/usr/bin/env bash
#
# ONE COMMAND, SELF-HEALING: fresh stack -> fly -> score -> tear down, and if the stack
# dies or freezes on the way, tear it down and go again (up to ATTEMPTS).
#
#   bash scripts/run_slosh_flight.sh              # tank aircraft, tank cascade
#   bash scripts/run_slosh_flight.sh --baseline   # baseline aircraft, base cascade
#   NAME=x ATTEMPTS=3 DIST=800 CRUISE_T=70 bash scripts/run_slosh_flight.sh
#   VTOL_TANK=slosh VTOL_TANK_KG=0.7 NAME=heavy bash scripts/run_slosh_flight.sh   # water mass (0.1 / 0.502 / 0.7)
#
# Why a wrapper rather than one long script: each stage already verifies itself
# (launch refuses READY on the wrong model or controller; the flight refuses an unfit
# vehicle and aborts on a dead or frozen stack; teardown exits non-zero if anything
# survived). This just sequences them and retries on the two exit codes that mean
# "not the aircraft's fault".
set -u
ROOT=/home/pieper/envs/fsc_PegasusSimulator
ATTEMPTS=${ATTEMPTS:-3}
TAG=${NAME:-flight}
say(){ echo "[run_slosh $(date +%H:%M:%S)] $*"; }
# ★ THE WRAPPER KEEPS ITS OWN LOG. Its timestamped lines are the only record of WHICH attempt
# tore down or launched a stack and why; on 2026-09-23 they went to a pipe and were lost when
# the run was interrupted, and a stack torn down mid-flight could not be attributed afterwards.
RUN_LOG=${RUN_LOG:-$HOME/flight_bags/slosh/run_${TAG}_$(date +%H%M%S).log}
exec > >(tee -a "$RUN_LOG") 2>&1
say "wrapper log: $RUN_LOG"

for n in $(seq 1 "$ATTEMPTS"); do
  say "=== attempt $n/$ATTEMPTS ($*) ==="
  bash "$ROOT/scripts/kill_slosh_hil.sh" >/dev/null 2>&1 || true
  if ! bash "$ROOT/scripts/start_slosh_hil.sh" "$@"; then
    say "launch failed -- retrying"; continue
  fi
  NAME="${TAG}_a${n}_$(date +%H%M%S)" bash "$ROOT/scripts/fly_slosh_mission.sh"
  RC=$?
  if [ "$RC" = "2" ]; then
    say "stack died or froze during the flight -- tearing down and going again"
    continue
  fi
  bash "$ROOT/scripts/kill_slosh_hil.sh" >/dev/null 2>&1 || true
  if [ "$RC" = "0" ]; then
    say "done: $(tail -1 "$HOME/flight_bags/slosh/results.tsv" | cut -f1-12 | tr '\t' ' ')"
    exit 0
  fi
  say "flight exited $RC (not a stack fault) -- not retrying"; exit "$RC"
done
say "!! gave up after $ATTEMPTS attempts"
bash "$ROOT/scripts/kill_slosh_hil.sh" >/dev/null 2>&1 || true
exit 1
