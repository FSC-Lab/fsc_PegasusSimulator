#!/usr/bin/env bash
# THE SENSOR AUDIT'S SWEEP (2026-09-16): the observer + controller on the turning mission with the
# accelerometer, gyro, state source and pitot degraded the way a real airframe's are, one effect per
# cell and then the "hardware" combinations; imu-mdob + EKF (the recommended channel) unless a cell
# names another kind. Each cell is one est_sweep.sh tag (sens_<cell>); the merged table at the end.
#
#   bash scripts/sensor_sweep.sh [seed_lo seed_hi] [cells...]      (default seeds 1-3, all cells)
#   env: JOBS=10 EST_DIR=~/flight_bags/est_sweeps
set -u
LO=1; HI=3
if [ $# -ge 2 ] && [[ "$1" =~ ^[0-9]+$ ]] && [[ "$2" =~ ^[0-9]+$ ]]; then LO=$1; HI=$2; shift 2; fi
ROOT=/home/pieper/envs/fsc_PegasusSimulator
export JOBS=${JOBS:-10}
IMU_HW="--noise_acc 0.05 --bias_acc 0.2,0.2,0.2 --acc_walk 0.01 --acc_scale 1.01 --vib 0.3,93 --imu_lag 0.01 --bias_gyro 0.2,0.2,0.2"
GPS_HW="--pos_hz 10 --vel_hz 10 --pv_lag 0.1 --imu_dr 1"
PITOT_HW="--airspeed --noise_aspd 0.3"
declare -A CELL=(
  [base]="--noise_acc 0.02"
  [acc_n05]="--noise_acc 0.05" [acc_n10]="--noise_acc 0.10"
  [bias02]="--noise_acc 0.02 --bias_acc 0.2,0.2,0.2" [bias05]="--noise_acc 0.02 --bias_acc 0.5,0.5,0.5"
  [walk01]="--noise_acc 0.02 --acc_walk 0.01" [walk03]="--noise_acc 0.02 --acc_walk 0.03"
  [scale101]="--noise_acc 0.02 --acc_scale 1.01" [scale103]="--noise_acc 0.02 --acc_scale 1.03"
  [vib03_93]="--noise_acc 0.02 --vib 0.3,93" [vib10_93]="--noise_acc 0.02 --vib 1.0,93"
  [vib03_30]="--noise_acc 0.02 --vib 0.3,30" [vib10_30]="--noise_acc 0.02 --vib 1.0,30"
  [lag10]="--noise_acc 0.02 --imu_lag 0.01" [lag20]="--noise_acc 0.02 --imu_lag 0.02" [lag40]="--noise_acc 0.02 --imu_lag 0.04"
  [lag40_comp]="--noise_acc 0.02 --imu_lag 0.04 --imu_delay 0.04"   # the same age, TOLD to the delay buffer
  [gyro02]="--noise_acc 0.02 --bias_gyro 0.2,0.2,0.2" [gyro10]="--noise_acc 0.02 --bias_gyro 1.0,1.0,1.0"
  [hw_imu]="$IMU_HW"
  [hw_all]="$IMU_HW $GPS_HW $PITOT_HW"
  # the state source and the pitot on their own, and the REALISTIC hardware state: EKF2's odometry at
  # its 100 Hz (200 after the PX4 rebuild) with its ~35 ms of lag (flight 11) and 0.05 m/s of velocity noise
  # not a bare 10 Hz GPS (hw_all's GPS_HW is the no-INS worst case)
  [gps10]="--noise_acc 0.02 --pos_hz 10 --vel_hz 10 --imu_dr 1"
  [gps10_lag01]="--noise_acc 0.02 $GPS_HW"
  [pitot03]="--noise_acc 0.02 $PITOT_HW"
  [pvlag035]="--noise_acc 0.02 --pos_hz 100 --vel_hz 100 --pv_lag 0.035"   # the lag alone
  [vnoise05]="--noise_acc 0.02 --noise_vel 0.05"                                  # the velocity noise alone
  [ekf2_like]="--noise_acc 0.02 --pos_hz 100 --vel_hz 100 --pv_lag 0.035 --noise_vel 0.05"
  [hw_ekf2]="$IMU_HW --pos_hz 100 --vel_hz 100 --pv_lag 0.035 --noise_vel 0.05 $PITOT_HW"
)
# which kind flies a cell (imu-mdob unless listed): the velocity twins on the hardware set, for the comparison
declare -A KIND=( [hw_all_mdob]="mdob" [hw_all_steso]="steso" )
CELL[hw_all_mdob]="$IMU_HW $GPS_HW $PITOT_HW"; CELL[hw_all_steso]="$IMU_HW $GPS_HW $PITOT_HW"
CELLS=${*:-"base acc_n05 acc_n10 bias02 bias05 walk01 walk03 scale101 scale103 vib03_93 vib10_93 vib03_30 vib10_30 lag10 lag20 lag40 lag40_comp gyro02 gyro10 hw_imu gps10 gps10_lag01 pitot03 pvlag035 vnoise05 ekf2_like hw_ekf2 hw_all hw_all_mdob hw_all_steso"}
PAR=${PAR:-3}     # cells in flight at once (each runs its seeds in parallel)
n=0
for c in $CELLS; do
  [ -n "${CELL[$c]+x}" ] || { echo "unknown cell $c"; exit 2; }
  echo "== cell $c: ${CELL[$c]} (${KIND[$c]:-imu-mdob})"
  ( KINDS="${KIND[$c]:-imu-mdob}" PREDS="on" bash "$ROOT/scripts/est_sweep.sh" "sens_$c" turns "$LO" "$HI" ${CELL[$c]} | grep -v "^  done" ) &
  n=$((n + 1)); if [ $((n % PAR)) -eq 0 ]; then wait; fi
done
wait
~/envs/isaac_env/bin/python3 "$ROOT/scripts/sensor_metrics.py" $CELLS
