#!/usr/bin/env bash
# MINIMAL HIL stack: the SAFE hover controller plus ONE controller under test.
#
# WHY THIS EXISTS: /uav_0/fmu/in/actuator_motors once had SIX publishers (traj_track,
# nmpc_so3_transition, nmpc_so3_cascade, nmpc_cruise, nmpc6_cruise, nmpc_full). A standby
# node co-publishing actuators against the active one is a documented failure mode here and
# flips the vehicle at takeoff (bags full3/full4: inverted on the ground before the
# controller was ever engaged). Standby nodes also SOLVE at full rate and starve the active
# loop of CPU, which is the other documented cause of a HIL tumble.
#
# It used to launch everything and then kill the windows it did not want. The main launcher
# now takes a CONTROLLERS list, so this is simply that list -- nothing is started and then
# torn down, and there is no race between the two.
#
# Usage:  scripts/start_vtol_minimal.sh [config] [controller]
#           scripts/start_vtol_minimal.sh pieper_machine              -> geometric alone
#           scripts/start_vtol_minimal.sh pieper_machine nmpc_full    -> geometric + nmpc_full
#         Controllers: nmpc_cruise nmpc6 nmpc_full so3 cascade transition attnmpc
#
# Verify before flying:  ros2 topic info -v /uav_0/fmu/in/actuator_motors
#   -> expect 1 publisher (geometric alone) or 2 (geometric + the one under test).
set -uo pipefail
CFG="${1:-pieper_machine}"
UNDER_TEST="${2:-}"
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

LIST="geometric${UNDER_TEST:+ $UNDER_TEST}"
echo "[minimal] controllers: $LIST"
CONTROLLERS="$LIST" \
SMC_LEAD="${SMC_LEAD:-30.0}" SMC_VMAX="${SMC_VMAX:-6.0}" \
FULL_VC="${FULL_VC:-16.0}" FULL_CAP="${FULL_CAP:-90.0}" FULL_ALT="${FULL_ALT:-15.0}" \
  exec bash "$REPO/scripts/start_vtol_smc_hil.sh" "$CFG"
