#!/bin/bash
# N release-window flights back to back (2026-10-17). Stop it with the process group
# (kill -- -PGID), never this bash alone (orphaned run_pl.sh polls the planner by name).
#   setsid nohup bash chain.sh <prefix> <first> <last> [RELEASE=0|1] > ../runs_chain_<prefix>.log 2>&1 < /dev/null &
C="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
P=$1; A=$2; B=$3; REL=${4:-1}
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
for k in $(seq "$A" "$B"); do
  clean
  echo "[$(date +%T)] ${P}_$k RELEASE=$REL"
  env RELEASE=$REL bash "$C/tools/fly.sh" "${P}_$k"
done
clean
echo "[$(date +%T)] CHAIN DONE"
