#!/bin/bash
# after chain2 (its queue pid given): drift seeds 7 and 8 for v2 / free / locked (the 2026-10-31 shim-raise
# diagnostic, queue3.sh, was dropped: the sim fingertips grip at 0..+16 mm along the claw, inside both fins)
while kill -0 "$1" 2>/dev/null; do sleep 10; done
cd "$(dirname -- "${BASH_SOURCE[0]}")"
setsid nohup bash pane_keeper.sh chain3_main.out > /dev/null 2>&1 < /dev/null &
bash chain2.sh free:s7:rf_s7 lock:s7:rl_s7 v2:s7:v2_s7 free:s8:rf_s8 lock:s8:rl_s8 v2:s8:v2_s8 > ../runs/chain3_main.out 2>&1 < /dev/null
