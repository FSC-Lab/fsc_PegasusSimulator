#!/bin/bash
# 2026-10-23 -- the mail-delivery repeats: mail3 (raw mocap), mail4 (EKF2-fused, the hardware path).
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/../runs"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
fly() { tag=$1; shift; [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $tag"; return; }
  clean; env "$@" timeout 1500 bash "$HERE/run_mail.sh" "$tag" > "$R/$tag.run.log" 2>&1
  /usr/bin/python3 "$HERE/mail_score.py" "$R/$tag.npz" > "$R/$tag.score.txt" 2>&1; }
fly mail3 MAIL_FEEDBACK=raw
fly mail4 MAIL_FEEDBACK=fused
clean
echo "CHAIN DONE"
