#!/bin/bash
# Snapshot the Isaac pane (the scene's own prints: FIRST CONTACT, ROLL FIN, BEAM, RTF) into
# runs/<current tag>.isaac.log every 5 s while chain_main.out names a running flight (2026-10-31).
R="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../runs" && pwd)"
while ! grep -q "CHAIN DONE" "$R/${1:-chain_main.out}" 2>/dev/null; do
  tag=$(grep -E " start " "$R/${1:-chain_main.out}" | tail -1 | awk '{print $3}')
  if [[ -n "$tag" ]] && tmux has-session -t px4_isaac 2>/dev/null; then
    tmux capture-pane -p -J -t px4_isaac:0.1 -S -30000 2>/dev/null | grep -E "AM-T650-VALVE|AM-T650-WB\] RTF" > "$R/$tag.isaac.tmp" && \
      [[ -s "$R/$tag.isaac.tmp" ]] && mv "$R/$tag.isaac.tmp" "$R/$tag.isaac.log"
  fi
  sleep 5
done
