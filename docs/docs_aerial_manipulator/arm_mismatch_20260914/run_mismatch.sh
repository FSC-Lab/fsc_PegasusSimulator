#!/usr/bin/env bash
# IMPERFECT FRICTION + GRAVITY COMPENSATION on the arm, against the whole-body
# L1 law (2026-09-14, user request off the calibration report: those two
# compensations dominate on the real arm, so the sim plant now carries both,
# with a controlled 5 % mismatch against what the arm controller compensates).
#
#   run_mismatch.sh [machine-config]            (RUNS=... to pick a subset)
#
# Three data points, same mission / driver / 16 s holds as every 7.15 campaign:
#   mismatch_A   plant friction x1.05 + arm mass x1.05, compensation ON at 1.00
#                (the shipped _sim yaml as of 2026-09-14) -- THE QUESTION
#   matched_B    plant friction x1.00 + arm mass x1.00, compensation ON
#                (perfect compensation: what "5 %" is measured against)
#   uncomp_C     plant friction x1.05 + arm mass x1.05, compensation OFF
#                (what the compensation buys; the observer carries everything)
# Everything else is the shipped plant: +15 % allocator kf, body mass/inertia
# x1.10 with the 10/10/5 mm CoM shift, MN4010 rotor lag, the current-loop
# residual, posture term absent, wb_ee_anchor_com + wb_u3_internal_ff.
#
# Env overrides beat the yaml (the launcher's precedence rule), which is what
# makes B and C one-line variants of A without editing anything.
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/arm_mismatch_20260914"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---hold-between 16}"

run() {  # $1 tag, $2 friction scale, $3 mass scale, $4 compensation 0|1
  echo; echo "############ $1  friction x$2  arm mass x$3  compensation $4 ############"
  PEGASUS_ARM_FRICTION_SCALE="$2" PEGASUS_ARM_MASS_SCALE="$3" PEGASUS_ARM_FRICTION_COMP="$4" \
    "$CYCLE" l1 "$1" "$CFG"
  echo "############ $1 rc=$? ############"
  echo "$1 friction=$2 mass=$3 comp=$4 driver='$WB_L1_DRIVER_ARGS'" >> "$WB_L1_OUT/runs.txt"
}

for TAG in ${RUNS:-mismatch_A matched_B uncomp_C}; do
  case "$TAG" in
    mismatch_A) run "$TAG" 1.05 1.05 1 ;;
    matched_B)  run "$TAG" 1.00 1.00 1 ;;
    uncomp_C)   run "$TAG" 1.05 1.05 0 ;;
    *) echo "unknown run tag $TAG" >&2; exit 2 ;;
  esac
done
