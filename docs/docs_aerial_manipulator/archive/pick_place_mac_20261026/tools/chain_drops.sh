#!/usr/bin/env bash
# MAC pick-and-drop flights in sequence (2026-10-26), drain on, the generated MAC pick-and-drop yaml.
#   chain_drops.sh <tag>...
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
Y="$HERE/../configs/params_single_aerial_manipulator_modular_adaptive_direct_actuation_t650_sim_pick_and_drop.yaml"
for tag in "$@"; do
  PNP_DRIVER_ARGS="--drop-place --grip-via-action --grip-axis-tol 0.003" \
    "$HERE/fly_one.sh" mod "$tag" PEGASUS_ARM_SPIN_DRAIN=1 WB_SIM_YAML="$Y" PEGASUS_PNP_TASK=drop
done
echo "[$(date +%T)] chain done"
