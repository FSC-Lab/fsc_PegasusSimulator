#!/usr/bin/env bash
# Keep the overnight campaign alive. The campaign is RESUMABLE (it skips trials already in
# results.tsv), so relaunching it is always safe -- it never repeats a trial or flies twice.
# This supervisor deliberately does nothing except watch and relaunch: it never calls the
# restart path itself, which is what kills the campaign process.
set -u
cd "$(dirname "$0")/.."
LOG=/tmp/night_campaign/supervisor.log
TRIALS=${1:-scripts/night_trials.txt}
mkdir -p /tmp/night_campaign
# ★ PIDFILE FOR THE SUPERVISOR TOO.  Checking "is the supervisor running?" with
# `pgrep -cf night_supervisor.sh` counts any shell whose command line merely CONTAINS that
# string -- including the shell asking the question. That false positive left the campaign
# unsupervised for 15 minutes while the check cheerfully reported "supervisors: 1".
echo $$ > /tmp/night_campaign/supervisor.pid
trap 'rm -f /tmp/night_campaign/supervisor.pid' EXIT
while true; do
  alive=0
  if [ -f /tmp/night_campaign/campaign.pid ]; then
    kill -0 "$(cat /tmp/night_campaign/campaign.pid 2>/dev/null)" 2>/dev/null && alive=1
  fi
  if [ "$alive" -eq 0 ]; then
    # stop when every trial has a result
    want=$(grep -cvE '^\s*(#|$)' "$TRIALS")
    have=$(( $(wc -l < /tmp/night_campaign/results.tsv 2>/dev/null || echo 1) - 1 ))
    if [ "$have" -ge "$want" ]; then
      echo "[$(date +%H:%M:%S)] all $want trials recorded - supervisor exiting" >> "$LOG"
      exit 0
    fi
    echo "[$(date +%H:%M:%S)] campaign not running ($have/$want done) - relaunching" >> "$LOG"
    setsid nohup bash scripts/night_campaign.sh "$TRIALS" \
        >> /tmp/night_campaign/driver.log 2>&1 < /dev/null &
  fi
  sleep 60
done
