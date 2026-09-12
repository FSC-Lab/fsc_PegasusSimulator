#!/usr/bin/env bash
# Trimmed 2026-09-11: `ideal` aborted, so the residual is exonerated and
# unif7_B's information value collapsed. Keep ideal_B -- one abort can be
# scatter, two is systematic -- and leave the machine free for the pre-pull
# arm-controller test, which is the variable this session introduced.
set -uo pipefail
CFG="${1:-fsc_lab_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/arm_calib_20260911"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---hold-between 16}"
echo "############ ideal_B ############"
PEGASUS_ARM_SERVO_MODEL=ideal "$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh" l1 ideal_B "$CFG"
echo "############ ideal_B rc=$? ############"
echo "ideal_B servo=ideal (no residual) kappa=calib-0911" >> "$WB_L1_OUT/runs.txt"
