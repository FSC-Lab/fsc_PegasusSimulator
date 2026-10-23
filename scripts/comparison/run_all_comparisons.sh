#!/usr/bin/env bash
# Fly all three comparison tasks on both controllers, then score them.
#
#   scripts/comparison/run_all_comparisons.sh [machine-config] [task ...]
#
# Six full launches, ~8 min each.  Every case gets its own clean slate: params
# are read at controller startup and PX4 never disarms this rig, so two
# missions on one launch is not an option (see Comparison Command.md 3).
#
# A failed case does NOT stop the campaign -- the run is recorded (the driver
# writes its npz even on an abort) and the next one starts, so one unstable
# configuration cannot cost the whole night.
set -uo pipefail

PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG="${1:-shiqi_machine}"
shift || true
TASKS=("$@")
if [[ ${#TASKS[@]} -eq 0 ]]; then
  TASKS=(hover_arm_swing circle_ee_hold figure8_ee_updown)
fi

rc_all=0
for task in "${TASKS[@]}"; do
  for which in wb l1; do
    echo
    echo "############################################################"
    echo "### $task / $which"
    echo "############################################################"
    "$PEG/scripts/comparison/run_comparison_case.sh" "$which" "$task" "$CFG"
    rc=$?
    [[ $rc -ne 0 ]] && rc_all=1 && echo "### $task/$which returned $rc (continuing)"
  done
  echo "### scoring $task"
  PYTHONNOUSERSITE=1 /usr/bin/python3 \
      "$PEG/application/robotic_arm/comparison_plots.py" --task "$task" || rc_all=1
done

echo
echo "### campaign done (rc=$rc_all).  Results under $PEG/results/"
exit $rc_all
