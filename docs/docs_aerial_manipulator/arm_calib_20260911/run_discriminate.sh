#!/usr/bin/env bash
# IS IT THE PLANT CHANGE, OR SOMETHING ELSE THAT MOVED TODAY? (2026-09-11)
#
# calib_A/B (per-joint, sqrt(2) convention) and conva_A (per-joint, in-band
# convention) all aborted, all with the same signature and all on the first leg
# that MOVES THE ARM. Before concluding that the measured per-joint residual is
# beyond this tune, rule out the two OTHER things that differ from the 09-09
# baseline campaign and have nothing to do with the residual:
#
#   * THE ARM CONTROLLER WAS REBUILT from the 2026-09-11 upstream
#     (external_torque_controller.cpp +251, torque_controller_base.cpp +147).
#     Those changes are in the Mode-16 duty path, which torque_controller_isaac_
#     aerial.yaml does not select -- but "should not reach it" is not a
#     measurement.
#   * system_wd_max_tilt_deg 40 -> 20 and system_wd_max_drift_m 0.75, added
#     2026-09-11 and NOT present in the 09-09 campaign.
#
#   ideal   the arm as an exact torque source. This plant is IDENTICAL to the
#           09-09 ideal-arm baseline l1_mission_kx32.npz, which flew the full
#           10-leg mission clean. If this aborts, the residual is exonerated
#           and the cause is the rebuild or the guard.
#   unif7   the 09-09 residual exactly: a uniform 7 mA. Isolates the PER-JOINT
#           change from the fact of having a residual at all. (Kt is the new
#           calibrated one, so torque noise is [16.0, 16.9, 17.3, 17.0] mN.m
#           against the baseline's [16.3, 15.0, 17.7, 16.3] -- within 13% per
#           joint, and j1/j4 are NOT raised.)
set -uo pipefail
CFG="${1:-fsc_lab_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/arm_calib_20260911"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---hold-between 16}"

echo "############ ideal  (exact torque source = the 09-09 clean baseline plant) ############"
PEGASUS_ARM_SERVO_MODEL=ideal "$CYCLE" l1 ideal "$CFG"
echo "############ ideal rc=$? ############"
echo "ideal servo=ideal (no residual) kappa=calib-0911" >> "$WB_L1_OUT/runs.txt"

echo "############ unif7  (the 09-09 residual: uniform 7 mA) ############"
PEGASUS_ARM_CURRENT_NOISE_A=0.007 "$CYCLE" l1 unif7 "$CFG"
echo "############ unif7 rc=$? ############"
echo "unif7 servo=current uniform 7mA@5Hz kappa=calib-0911" >> "$WB_L1_OUT/runs.txt"
