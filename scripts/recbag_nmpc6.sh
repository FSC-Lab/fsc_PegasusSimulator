#!/usr/bin/env bash
# Record the key diagnosis topics for the 6-state wing-borne NMPC (nmpc6_cruise).
#   Usage:  scripts/recbag_nmpc6.sh <name>          (default name: nmpc6)
#   Then:   ~/envs/isaac_env/bin/python3 /tmp/anabag.py <name>
# See docs/nmpc6_rosbag.md for the topic meanings + how to read the result.
# Run from a CLEAN shell (this unsets the isaac_env/conda pollution so system ROS python is used).
unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT LD_PRELOAD
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
NAME="${1:-nmpc6}"
NS="${PX4_NS:-uav_0}"
BAG_DIR=${BAG_DIR:-$HOME/flight_bags}     # not /tmp: wiped on reboot
rm -rf "$BAG_DIR/$NAME"; mkdir -p "$BAG_DIR"
echo "[recbag_nmpc6] recording -> $BAG_DIR/$NAME   (Ctrl-C to stop)"
exec ros2 bag record -o "$BAG_DIR/$NAME" \
  "/$NS/mocap" \
  "/$NS/fmu/in/actuator_motors" \
  "/$NS/fmu/in/actuator_servos" \
  "/$NS/ground_station/active_controller" \
  "/$NS/fmu/out/vehicle_status_v1" \
  "/$NS/fmu/out/vehicle_attitude" \
  "/$NS/fmu/out/vehicle_local_position"
