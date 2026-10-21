#!/usr/bin/env bash
# grab the simulator window every 2 s (full window PNG, timestamped) until it closes or 30 min pass
export DISPLAY=:1
W=0x620000b
D=/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/8b92ac4e-24d1-4f70-8431-374faf1f2d27/scratchpad/frames
end=$((SECONDS + 1800))
while (( SECONDS < end )); do
  xwininfo -id $W >/dev/null 2>&1 || { echo "window closed $(date +%T)"; exit 0; }
  gst-launch-1.0 -q ximagesrc xid=$W num-buffers=1 use-damage=false ! videoconvert ! pngenc ! filesink location="$D/f_$(date +%H%M%S).png" >/dev/null 2>&1
  sleep 2
done
echo "30 min done $(date +%T)"
