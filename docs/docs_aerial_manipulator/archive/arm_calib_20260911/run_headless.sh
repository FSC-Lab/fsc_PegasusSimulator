#!/usr/bin/env bash
# THE REAL-TIME-FACTOR TEST (2026-09-11).
#
# Six runs on this machine aborted, INCLUDING an ideal arm and including the
# exact 09-09 plant, so the arm residual was never the cause. Measured live
# during a flight: REAL-TIME FACTOR 0.336, Isaac CPU-bound at 430% with the GPU
# at 40%.
#
# Why that alone is fatal here: these rigs run PEGASUS_PX4_LOCKSTEP=0, so the
# external controller runs on the WALL CLOCK at 250 Hz while the plant advances
# at 0.336x. Every millisecond of real transport delay is therefore ~3 ms of
# SIMULATED delay, and this tune is delay-margin limited -- wb_hover_stability.py
# picked M_r_d on a ~32 ms margin against the 10.03 1/s rotor pole, and the whole
# 2026-08-22 retune exists because in-process gains did not survive the DDS loop.
# A 3x delay inflation is far outside that.
#
# 06 already implements world.step(render=not HEADLESS) -- the fix the repo
# measured at 4-5x on the slung-load scenario -- but nothing ever set the knob,
# and an export could not reach the Isaac pane anyway (the tmux-server-env trap).
# start_single_drone_x650.sh now bakes it into the pane command line.
#
#   ideal_hl   ideal arm, headless. Isolates RTF: same plant as `ideal`, which
#              aborted at leg 3.
#   calib_hl   the SHIPPED per-joint residual, headless. This is the actual
#              validation of the 2026-09-11 alignment.
#
# NOTE: CLAUDE.md records Isaac crashing at ~800 ms under PEGASUS_HEADLESS=1 on
# a lab machine (2026-08-05, breakpad out of pthread_create). If that happens
# the run dies immediately and visibly -- it is not a silent degradation.
set -uo pipefail
CFG="${1:-fsc_lab_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/arm_calib_20260911"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---hold-between 16}"
export PEGASUS_HEADLESS=1

echo "############ ideal_hl (headless, ideal arm) ############"
PEGASUS_ARM_SERVO_MODEL=ideal "$CYCLE" l1 ideal_hl "$CFG"
echo "############ ideal_hl rc=$? ############"
echo "ideal_hl servo=ideal HEADLESS=1" >> "$WB_L1_OUT/runs.txt"

echo "############ calib_hl (headless, shipped per-joint residual) ############"
"$CYCLE" l1 calib_hl "$CFG"
echo "############ calib_hl rc=$? ############"
echo "calib_hl servo=current per-joint[15.6,6.2,9.6,15.6]mA HEADLESS=1" >> "$WB_L1_OUT/runs.txt"
