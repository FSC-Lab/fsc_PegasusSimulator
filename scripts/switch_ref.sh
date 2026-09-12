#!/usr/bin/env bash
# Swap the streaming raw-reference target. Run as a FILE (a pkill -f typed into a tool call
# would match its own command line). Usage: switch_ref.sh X Y Z [YAW]
source /tmp/rosenv.sh
PAT="scripts/pubref.py"
for pid in $(pgrep -f -- "$PAT" 2>/dev/null); do
  [ "$pid" = "$$" ] && continue
  kill -9 "$pid" 2>/dev/null && echo "  stopped old ref publisher $pid"
done
sleep 1
nohup /home/pieper/envs/isaac_env/bin/python3 \
  /home/pieper/envs/fsc_PegasusSimulator/scripts/pubref.py "$1" "$2" "$3" "${4:-0}" --rate 5 \
  > /tmp/pubref.log 2>&1 &
sleep 2
echo "  now streaming target ($1,$2,$3) yaw=${4:-0}"
tail -2 /tmp/pubref.log
