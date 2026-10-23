#!/usr/bin/env bash
# SEPARATING THE TWO THINGS THAT CHANGED (2026-09-11).
#
# run_calib.sh flew per-joint residuals at sigma_total = sqrt(2) x the measured
# IN-BAND rms, and aborted 2/2. That run moved TWO things at once:
#
#   1. the values became PER JOINT, which is the measurement (the current loop
#      runs on j2/j3 only, so j1/j4 keep the untrimmed 11 mA);
#   2. the CONVENTION changed. The shipped model set sigma_total equal to the
#      measured in-band rms directly (7 mA was j3's in-band figure, used as a
#      total). sqrt(2) is the right factor IF the real spectrum keeps falling
#      like the model's first-order one above 5 Hz -- but the bench filtered
#      there and servo_model.py says in as many words that content above 5 Hz
#      "is not characterised". So the measurement does NOT settle it, and it is
#      a 41% inflation of every channel.
#
# This runs (1) ALONE, on the original convention: sigma = [11, 4.4, 6.8, 11]
# mA. Against the 09-09 baseline's uniform 7 mA that is per-joint torque noise
# x[1.55, 0.71, 0.95, 1.64] instead of x[2.19, 1.00, 1.34, 2.32].
#
# Everything else is identical to run_calib.sh, including the calibrated kappa
# (so Kt is the new one in both) and the shipped 20 deg tilt watchdog.
set -uo pipefail
CFG="${1:-fsc_lab_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/arm_calib_20260911"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---hold-between 16}"
# env > yaml, so this wins over the yaml's sim_arm_current_noise_a_j* without
# editing the config a campaign is being scored against.
export PEGASUS_ARM_CURRENT_NOISE_A="0.011,0.0044,0.0068,0.011"

for TAG in ${RUNS:-conva_A conva_B}; do
  echo
  echo "############ $TAG  (per-joint, in-band convention) ############"
  "$CYCLE" l1 "$TAG" "$CFG"
  echo "############ $TAG rc=$? ############"
  echo "$TAG servo=current per-joint[11,4.4,6.8,11]mA@5Hz kappa=calib-0911 driver='$WB_L1_DRIVER_ARGS'" \
    >> "$WB_L1_OUT/runs.txt"
done
