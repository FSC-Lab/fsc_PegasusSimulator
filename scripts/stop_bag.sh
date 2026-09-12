#!/usr/bin/env bash
# SIGINT the rosbag recorder so the .db3 is finalized and metadata.yaml is written.
# Must be run as a FILE: a `pkill -f "bag record"` typed into a tool call matches its own command
# line and would kill the invoking shell. An unfinalized bag fails to open with a sqlite I/O error.
PAT="bag record"
SELF=$$
for pid in $(pgrep -f -- "$PAT" 2>/dev/null); do
  [ "$pid" = "$SELF" ] && continue
  kill -INT "$pid" 2>/dev/null && echo "  SIGINT -> $pid"
done
sleep 4
for pid in $(pgrep -f -- "$PAT" 2>/dev/null); do
  [ "$pid" = "$SELF" ] && continue
  kill -9 "$pid" 2>/dev/null
done
echo "  recorder stopped"
