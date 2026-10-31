#!/usr/bin/env bash
# MAC pick-and-place (the 10-09 set-down task, hook, 200 g, --hook-place --timetable) with the CORRECTED
# arm-bridge drain (2026-10-26 pm): fly_one.sh per tag, each between clean slates.
#   chain_fixed_pnp.sh <tag>...
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for tag in "$@"; do
  "$HERE/fly_one.sh" mod "$tag" PEGASUS_ARM_SPIN_DRAIN=1
done
echo "[$(date +%T)] chain done"
