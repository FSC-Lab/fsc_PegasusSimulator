#!/usr/bin/env bash
# THE MODEL-ERROR ENVELOPE (hardware-transfer piece 2, 2026-09-15): the full mission (ke_sweep's line:
# climb -> corridor -> cruise -> back-transition at 60 s -> stop) with the PLANT perturbed one constant
# at a time and the controller at nominal, 16 seeds per cell, the recommended configuration (the
# estimator variant, mdob + EKF, L1). Cells are sequential (each cell's 16 seeds run in parallel).
#   bash scripts/robust_sweep.sh <tag> [cells file]     -> ~/flight_bags/robust/<tag>/<cell>/ + summary.txt
# then  scripts/robust_metrics.py ~/flight_bags/robust/<tag>  -> the tolerance table.
# Never while a flight is in the air (it drops the sim's real-time factor).
set -u
TAG=$1; CELLS=${2:-}
[ -n "$CELLS" ] && CELLS=$(readlink -f "$CELLS")
ROOT=/home/pieper/envs/fsc_PegasusSimulator
OUT=${ROBUST_DIR:-$HOME/flight_bags/robust}/$TAG; mkdir -p "$OUT"
SEEDS_LO=${SEEDS_LO:-1}; SEEDS_HI=${SEEDS_HI:-16}
# The control law under test. Defaults = piece 2's line (the steso node's velocity force twin), so
# that table reproduces; ROBUST_LINE / ROBUST_NODE run the sweep on another one -- the flown
# "Adaptive NMPC + Geo" is ROBUST_NODE=nmpc_cascade_imu_node.py with --noise_acc 0.02 on the line.
LINE=${ROBUST_LINE:-"--est mdob --l1 --ttr 10 --lat 0.03 --comp 0.049 --slosh"}
NODE=${ROBUST_NODE:-nmpc_cascade_steso_node.py}
cd /home/pieper/envs/fsc_autopilot_ros2-master
if ! git diff --quiet -- px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_node.py px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_so3_transition_node.py px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_cascade_steso_node.py; then
  echo "!! controller tree dirty -- commit first"; git status --short; exit 1
fi
echo "$NODE | $LINE" > "$OUT/law"; git rev-parse --short HEAD > "$OUT/ctrl_commit"; git -C "$ROOT" rev-parse --short HEAD > "$OUT/sim_commit"
# cell name | harness flags  (the plant side only; --l1_tau_spin pins the controller's lag when --tau_spin moves the plant's)
default_cells() { cat <<'C'
nominal|
mass_0.90|--plant_mass_scale 0.90
mass_0.95|--plant_mass_scale 0.95
mass_1.05|--plant_mass_scale 1.05
mass_1.10|--plant_mass_scale 1.10
mass_1.15|--plant_mass_scale 1.15
mass_1.20|--plant_mass_scale 1.20
inertia_0.6|--plant_inertia_scale 0.6
inertia_0.8|--plant_inertia_scale 0.8
inertia_1.25|--plant_inertia_scale 1.25
inertia_1.6|--plant_inertia_scale 1.6
cq_0.06|--plant_cq 0.06
cq_0.09|--plant_cq 0.09
cq_0.15|--plant_cq 0.15
cq_0.18|--plant_cq 0.18
lift_0.7|--plant_aero_scale lift=0.7
lift_0.85|--plant_aero_scale lift=0.85
lift_1.15|--plant_aero_scale lift=1.15
lift_1.3|--plant_aero_scale lift=1.3
pitch_0.6|--plant_aero_scale pitch=0.6
pitch_0.8|--plant_aero_scale pitch=0.8
pitch_1.25|--plant_aero_scale pitch=1.25
pitch_1.5|--plant_aero_scale pitch=1.5
ctrl_0.6|--plant_aero_scale ctrl=0.6
ctrl_0.8|--plant_aero_scale ctrl=0.8
ctrl_1.25|--plant_aero_scale ctrl=1.25
ctrl_1.5|--plant_aero_scale ctrl=1.5
lat_0.5|--plant_aero_scale lat=0.5
lat_1.5|--plant_aero_scale lat=1.5
drag_0.7|--plant_aero_scale drag=0.7
drag_1.4|--plant_aero_scale drag=1.4
thrust_0.80|--plant_thrust_scale 0.80
thrust_0.90|--plant_thrust_scale 0.90
thrust_1.10|--plant_thrust_scale 1.10
thrust_1.20|--plant_thrust_scale 1.20
j0_0.60|--plant_J0 0.60
j0_0.66|--plant_J0 0.66
j0_0.80|--plant_J0 0.80
j0_0.90|--plant_J0 0.90
cg_-0.03|--plant_cg_dx -0.03
cg_-0.015|--plant_cg_dx -0.015
cg_0.015|--plant_cg_dx 0.015
cg_0.03|--plant_cg_dx 0.03
cg_0.045|--plant_cg_dx 0.045
spin_0.06|--tau_spin 0.06 --l1_tau_spin 0.12
spin_0.20|--tau_spin 0.20 --l1_tau_spin 0.12
spin_0.30|--tau_spin 0.30 --l1_tau_spin 0.12
surf_0.06|--surf_tau 0.06
surf_0.10|--surf_tau 0.10
C
}
if [ -n "$CELLS" ]; then cat "$CELLS"; else default_cells; fi | grep -v '^#' | while IFS='|' read -r name flags; do
  [ -z "$name" ] && continue
  if [ -f "$OUT/$name/summary.txt" ]; then echo "skip $name (done)"; continue; fi
  echo "== $name: $flags   $(date '+%H:%M')"
  # shellcheck disable=SC2086
  KE_DIR="$OUT" KE_NODE="$NODE" bash "$ROOT/scripts/ke_sweep.sh" "$name" "$SEEDS_LO" "$SEEDS_HI" $flags $LINE > /dev/null 2>&1
  ~/envs/isaac_env/bin/python3 "$ROOT/scripts/ke_metrics.py" "$OUT/$name" > "$OUT/$name/summary.txt" 2>&1
  echo "$name|$flags|$(grep SUMMARY "$OUT/$name/summary.txt")" >> "$OUT/cells.txt"
  tail -1 "$OUT/$name/summary.txt" | cut -c1-160
done
echo "robust sweep $TAG done -> $OUT"; ~/envs/isaac_env/bin/python3 "$ROOT/scripts/robust_metrics.py" "$OUT" | tee "$OUT/table.txt"
