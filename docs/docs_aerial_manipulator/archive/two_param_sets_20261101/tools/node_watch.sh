#!/bin/bash
# Capture the whole-body node's grip-schedule / anchor log lines from the running stack's tmux pane
# (the pane is not written to any file) into runs/node_events.log, tagged with the newest run.
#   setsid nohup bash node_watch.sh > /dev/null 2>&1 &      (touch runs/STOP_WATCH to end)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"; R="$HERE/../runs"
OUT="$R/node_events.log"; touch "$OUT"
PAT="TWO SETS|GRIP: parameters|GRIP: jaws|EE reference anchor ->|free-axis anchor (LATCHED|released)|DIRECT WATCHDOG|emergency abort"
while [[ ! -e "$R/STOP_WATCH" ]]; do
  tag=$(ls -t "$R"/*.args 2>/dev/null | head -1 | xargs -r basename | sed 's/\.args$//')
  for p in $(tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index}' 2>/dev/null); do
    tmux capture-pane -p -J -S -4000 -t "$p" 2>/dev/null | grep -aE "$PAT" | sed 's/\x1b\[[0-9;]*m//g' | sed "s/^/[$tag] /"
  done | cut -c1-260 | while IFS= read -r line; do grep -qF -- "$line" "$OUT" || echo "$line" >> "$OUT"; done
  sleep 15
done
