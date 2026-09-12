#!/usr/bin/env bash
# Live one-line view of what the HIL stack and the vehicle are doing.
# Useful because a full test is ~3.5 min of stack restart before ~6 min of flight, during
# which a filtered log prints nothing and it looks hung.  Ctrl-C to stop.
source /home/pieper/envs/fsc_PegasusSimulator/scripts/rosenv.sh 2>/dev/null
while true; do
  ISA=$(pgrep -cf 03_vtol_px4_hil 2>/dev/null || echo 0)
  PX=$(pgrep -cx px4 2>/dev/null || echo 0)
  NODES=$(ros2 node list 2>/dev/null | grep -cE "geometric_hover_control|nmpc_full_smc_control")
  LINK=$(tmux capture-pane -p -S -200 -t vtol_smc_hil:2-isaac 2>/dev/null \
         | grep -cE "VTOL_HIL vid=|Received first hearbeat")
  POS=$(timeout 2 ros2 topic echo /uav_0/mocap --once 2>/dev/null \
        | sed -n '/position:/,/z:/p' | grep -oE "[-0-9.e]+" | tr '\n' ' ' \
        | awk '{printf "x=%.0f y=%.0f z=%.1f", $1, $2, $3}')
  TILT=$(timeout 2 ros2 topic echo /uav_0/fmu/in/actuator_servos --once 2>/dev/null \
         | grep -A1 "^control:" | tail -1 | grep -oE "[-0-9.e]+" | awk '{printf "%.0f", $1*90}')
  CTRL=$(tmux capture-pane -p -S -3 -t vtol_smc_hil:6g-full 2>/dev/null \
         | grep -oE "ok=[01]" | tail -1)
  printf "\r%s | isaac=%s px4=%s nodes=%s/2 link=%s | %s tilt=%s %s        " \
         "$(date +%H:%M:%S)" "$ISA" "$PX" "$NODES" "$([ "$LINK" -ge 1 ] && echo up || echo --)" \
         "${POS:-no mocap}" "${TILT:-–}" "$CTRL"
  sleep 2
done
