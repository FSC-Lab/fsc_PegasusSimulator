#!/usr/bin/env bash
# Move the SIMULATED localization frame in the 07 mail-delivery scene, as the operator moves the
# printed one on the day (2026-10-30):
#
#   mail_frame.sh hat       on the pick hat (skirts over the rim, the envelope in its slot) -> Envelope Get
#   mail_frame.sh mailbox   on the red mailbox's top (+x into the mouth)                  -> Mailbox Get
#   mail_frame.sh off       lifted off (the scene also does this by itself at takeoff)
#
# obj_0 is the frame's five markers' centroid while it is placed; the planner's Get adds
# pick_place_pick_locator / pick_place_place_locator (the same in the sim and the hardware yaml).
set -eo pipefail
case "${1:-}" in
  hat)     SRV=on_hat ;;
  mailbox) SRV=on_mailbox ;;
  off)     SRV=remove ;;
  *) echo "usage: $0 hat|mailbox|off" >&2; exit 2 ;;
esac
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
timeout 20 ros2 service call "/mail_frame/$SRV" std_srvs/srv/Trigger | grep -E "success|message" || {
  echo "no answer from /mail_frame/$SRV -- is the 07 mail-delivery scene running?" >&2; exit 1; }
