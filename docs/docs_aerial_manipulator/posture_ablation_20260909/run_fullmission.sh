#!/usr/bin/env bash
# FULL 8-LEG MISSION with the posture PID OFF and the compensation ON.
# Every earlier compensation flight was a hover soak; this is the gate before
# the wb_posture_* keys are removed from the L1 sim yaml. The yaml is used AS
# IT STANDS (no edits, no restore) so it flies exactly the shipped config.
#   settle -> x steps -> y steps -> yaw steps -> compatible trajectory -> abort -> land
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/posture_ablation_20260909"
export WB_L1_DRIVER_ARGS="--hold-between 16"        # 7.15.7: 6 s does not settle
for tag in full_A full_B; do
  echo; echo "############ $tag  full mission, posture OFF, compensation ON ############"
  "$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh" l1 "mission_$tag" "$CFG"
  echo "############ $tag rc=$? ############"
done
echo "FULL MISSION DONE"
