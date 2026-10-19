#!/bin/bash
# Render the Isaac half of a REPLAYED video (2026-10-18): replay_scene.py in Isaac's python,
# windowed on DISPLAY :1, pinned to the P-cores like the live scene (the capture runs on 16-31).
#   replay.sh TAKE [--start EVENT[+-s]] [--end EVENT[+-s]] [--preroll s]
# After a clean slate: no stack, no PX4, no other Isaac. Log: ../logs/replay_TAKE.log.
TAKE="${1:?take}"; shift
V="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
L="$V/../logs"; mkdir -p "$L"
export DISPLAY="${DISPLAY:-:1}"
taskset -c 0-15 /home/shiqi/isaacsim/python_r_fsc.sh "$V/replay_scene.py" "$TAKE" "$@" 2>&1 | tee "$L/replay_$TAKE.log"
