#!/bin/bash
# wait for chain.sh (pid given) to exit, then fly chain2.sh's jobs with a pane keeper
while kill -0 "$1" 2>/dev/null; do sleep 10; done
cd "$(dirname -- "${BASH_SOURCE[0]}")"
setsid nohup bash pane_keeper.sh chain2_main.out > /dev/null 2>&1 < /dev/null &
bash chain2.sh v2:s2:v2_s2 v2:s3:v2_s3 free:s5:rf_s5 lock:s5:rl_s5 v2:s5:v2_s5 free:s6:rf_s6 lock:s6:rl_s6 v2:s6:v2_s6 > ../runs/chain2_main.out 2>&1 < /dev/null
