#!/usr/bin/env bash
# THE 2026-09-11 ARM CALIBRATION + PER-JOINT CURRENT LOOP, against the L1
# whole-body law.
#
# What changed in the PLANT since arm_current_noise_20260909 (nothing else --
# no gain, no law key, no injection moved):
#
#   1. THE MOTOR CALIBRATION. nm_to_effort_joints [160.0, 173.8, 146.7, 160.0]
#      -> [162.4, 154.0, 150.5, 153.4] counts/N.m and motor_resistance_ohm
#      [4.90, 4.90, 4.30, 4.90] -> [5.26, 4.90, 4.53, 4.88], both measured by
#      the arm repo's lever/RLS campaign (the j1/j4 UNITS were swapped into the
#      j2/j3 brackets to give them a gravity lever). Cross-check that makes
#      them trustworthy: eta = Kt/Ke is now [0.943, 0.943, 0.942, 0.943]
#      against the same session's measured Ke -- four motors agreeing to 0.1 %
#      on a sensible gearbox loss, where the old set implied eta > 1 on j2.
#      In the sim this moves Kt, so it moves the torque each mA of residual
#      current makes.
#
#   2. THE CURRENT LOOP IS PER JOINT. current_loop_bandwidth_hz_joints ships
#      [0.0, 1.5, 1.5, 0.0]: the trim helps the two loaded joints and HURTS
#      j1/j4, whose commanded current is mostly noise. So the plant's residual
#      stops being one number and becomes [15.6, 6.2, 9.6, 15.6] mA rms
#      total = [35.7, 15.0, 23.7, 37.8] mN.m, where it was a uniform 7 mA =
#      [16.3, 15.0, 17.7, 16.3] mN.m. THE UNTRIMMED JOINTS MORE THAN DOUBLE,
#      and they are exactly the two that carry the EE-heading task.
#
# Everything else is the shipped _sim config: +15% allocator kf, plant mass
# and inertia x1.10 with a 10/10/5 mm CoM shift, MN4010 rotor lag, posture term
# absent, wb_ee_anchor_com + wb_u3_internal_ff on, K_y 20 / D_y 12, k_x 32.
#
#   run_calib.sh [machine-config]
#
# --hold-between 16 for the same reason 09-09 used it: 7.15.7 measured that a
# 6 s hold does not settle on this rig, so a settled error scored across two
# different hold lengths is not a comparison. The baselines to read this
# against are in ../arm_current_noise_20260909/metrics.txt (l1_noise_B/C, the
# uniform 7 mA on the OLD constants, and l1_mission_kx32, the ideal arm).
#
# TWO RUNS of one config, because one completed run proves nothing here -- a
# matched-kf configuration once passed only 1 soak in 3 (Command.md 7.14.6).
set -uo pipefail
CFG="${1:-fsc_lab_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/arm_calib_20260911"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---hold-between 16}"

for TAG in ${RUNS:-calib_A calib_B}; do
  echo
  echo "############ $TAG  (calibrated kappa + per-joint residual) ############"
  "$CYCLE" l1 "$TAG" "$CFG"
  echo "############ $TAG rc=$? ############"
  echo "$TAG servo=current per-joint[15.6,6.2,9.6,15.6]mA@5Hz kappa=calib-0911 driver='$WB_L1_DRIVER_ARGS'" \
    >> "$WB_L1_OUT/runs.txt"
done
