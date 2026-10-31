#!/usr/bin/env bash
# MAC pick-and-place, drain on, a list of payload masses in sequence (2026-10-26).
#   chain_masses.sh <tag-prefix> <mass_kg>...
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
P="$1"; shift
for m in "$@"; do
  g=$(printf '%.0f' "$(echo "$m*1000" | bc)")
  tag="${P}_m${g}"
  [[ -e "$HERE/../runs/$tag.npz" ]] && tag="${tag}_b"
  "$HERE/fly_one.sh" mod "$tag" PEGASUS_ARM_SPIN_DRAIN=1 PNP_PAYLOAD_MASS="$m" PEGASUS_PNP_PAYLOAD_MASS="$m"
done
echo "[$(date +%T)] chain done"
