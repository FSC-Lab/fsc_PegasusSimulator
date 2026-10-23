#!/usr/bin/env bash
# The whole L1-observer campaign, back to back.
#
#   wb_l1_campaign.sh [machine-config]
#
# Every data point is a FULL relaunch (params are read at controller startup
# and PX4 never disarms this rig), so this just sequences wb_l1_tune_cycle.sh
# and edits the yaml between runs. Roughly 7 minutes per run.
#
# THE SWEEP, and why these points.
#   gmo_baseline   the control case: the flight-validated GMO, K_o =
#                  0.5/0.1/0.1, on exactly this plant.
#   l1_matched     omega_c set EQUAL to K_o. The GMO's estimate and the L1's
#                  reach the control loops through the same first-order
#                  omega/(s+omega), so this should fly indistinguishably --
#                  it is the sanity check that the swap changed nothing it
#                  should not have, not a result.
#   l1_wc2         the first step up. The GMO cannot follow here: 1.0 on its
#                  body channels crashed this rig.
#   l1_wc6         aggressive, into the region where the 99.7 ms rotor lag
#                  (a pole at 10.03 rad/s) starts to matter.
#   l1_lumped      the best surviving omega_c with the ATTRIBUTION OFF, so the
#                  note's two ideas -- a better estimate and a correct
#                  attribution -- are separated instead of measured together.
#   l1_minnorm     the best surviving omega_c with the note's OWN minimum-norm
#                  L_c metric (all prior variances 1.0), which offline
#                  analysis says should make the phantom force worse. Flying
#                  it is what turns that from a prediction into a measurement.
set -uo pipefail

CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
SETG="$PEG/application/robotic_arm/utils/wb_l1_set_gains.py"
OUT="${WB_L1_OUT:-$PEG/docs/docs_aerial_manipulator/l1_observer_20260906}"
mkdir -p "$OUT"

# Restore the shipped gains on the way out, whatever happens -- an interrupted
# campaign must not leave the yaml carrying a sweep point.
BASE_YAML="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml"
cp "$BASE_YAML" "$OUT/.yaml_backup"
trap 'cp "$OUT/.yaml_backup" "$BASE_YAML"; echo "restored $BASE_YAML"' EXIT

run() {  # run <gmo|l1> <tag>
  echo
  echo "############ $2 ############"
  "$CYCLE" "$1" "$2" "$CFG"
  echo "############ $2 rc=$? ############"
}

case "${WB_L1_ONLY:-all}" in
  all|gmo) run gmo baseline ;;
esac

if [[ "${WB_L1_ONLY:-all}" == gmo ]]; then exit 0; fi

/usr/bin/python3 "$SETG" omega_c_t=0.5 omega_c_r=0.1 omega_c_q=0.1 decompose=true \
  lc_var_f=100.0 lc_var_m=0.25 lc_var_q=0.0025
run l1 matched

/usr/bin/python3 "$SETG" omega_c_t=2.0 omega_c_r=0.5 omega_c_q=0.5
run l1 wc2

/usr/bin/python3 "$SETG" omega_c_t=6.0 omega_c_r=2.0 omega_c_q=1.0
run l1 wc6

echo
echo "Sweep done. Score with:"
echo "  /usr/bin/python3 $PEG/application/robotic_arm/utils/wb_l1_metrics.py $OUT/*.npz"
