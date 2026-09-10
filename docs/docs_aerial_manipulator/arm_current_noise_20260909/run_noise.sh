#!/usr/bin/env bash
# THE ARM'S CURRENT-LOOP RESIDUAL, against the L1 whole-body law (2026-09-09).
#
# The arm controller now closes a 1.5 Hz software current loop around Dynamixel
# Mode 16, so the back-EMF droop the plant used to inject is gone and what is
# left is a zero-mean residual current error: 7 mA rms band-limited at 5 Hz on
# every joint, = [16.3, 15.0, 17.7, 16.3] mN.m through each joint's Kt.
# Everything else is the shipped _sim config -- +15% allocator kf, plant mass
# and inertia x1.10 with a 10/10/5 mm CoM shift, MN4010 rotor lag, posture term
# absent, wb_ee_anchor_com + wb_u3_internal_ff on, K_y 20 / D_y 12, k_x 32.
#
#   run_noise.sh [machine-config]
#
# --hold-between 16 ON PURPOSE: it is what l1_mission_kx32.npz (the ideal-arm
# baseline this is compared against) flew, and 7.15.7 measured that a 6 s hold
# does not settle on this rig, so a settled error scored across two different
# hold lengths is not a comparison at all.
#
# TWO RUNS of the same config, because one completed run proves nothing here --
# a matched-kf configuration once passed only 1 soak in 3 (Command.md 7.14.6).
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/arm_current_noise_20260909"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---hold-between 16}"

for TAG in ${RUNS:-noise_B noise_C}; do
  echo
  echo "############ $TAG  (current-loop residual ON) ############"
  "$CYCLE" l1 "$TAG" "$CFG"
  echo "############ $TAG rc=$? ############"
  echo "$TAG servo=current 7mA@5Hz driver='$WB_L1_DRIVER_ARGS'" >> "$WB_L1_OUT/runs.txt"
done
