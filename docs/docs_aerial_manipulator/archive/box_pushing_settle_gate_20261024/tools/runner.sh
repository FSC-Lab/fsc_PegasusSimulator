#!/bin/bash
# (copied 2026-10-24 evening from ../../box_pushing_guard_gate_20261024/tools for the hand-over settle + calm-exit check)
# (copied 2026-10-24 from ../../box_pushing_contact_stability_tune_20261021/tools for the guard + close-gate check)
# Queue runner for the contact-stability tune: flies queue/*.job oldest-name first, one at a time,
# a clean slate before each flight, and closes the simulation whenever the queue runs dry.
#   job file (bash): TAG is the file name; ENV="K=V ..." (fly.sh environment); ARGS="k=v ..."
#   touch ../PAUSE  holds before the next clean slate;  touch ../STOP  ends the runner
# Never edit this file while it runs (bash reads it by byte offset); add jobs instead.
C="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
Q=$C/queue
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
idle=0
while true; do
  [[ -e $C/STOP ]] && { echo "[$(date +%T)] STOP"; break; }
  [[ -e $C/PAUSE ]] && { sleep 10; continue; }
  job=$(ls $Q/*.job 2>/dev/null | sort | head -1)
  if [[ -z "$job" ]]; then
    if (( idle == 0 )); then clean; echo "[$(date +%T)] queue empty -- simulation closed"; idle=1; fi
    sleep 15; continue
  fi
  idle=0
  tag=$(basename "$job" .job); tag=${tag#*__}       # files are <order>__<tag>.job
  if [[ -e $C/runs/$tag.npz ]]; then echo "skip $tag (flown)"; mv "$job" $Q/done/; continue; fi
  ENV=""; ARGS=""; . "$job"
  clean
  echo "[$(date +%T)] start $tag | $ENV | $ARGS"
  env $ENV bash $C/tools/fly.sh "$tag" $ARGS
  mv "$job" $Q/done/
done
clean
echo "[$(date +%T)] RUNNER DONE"
