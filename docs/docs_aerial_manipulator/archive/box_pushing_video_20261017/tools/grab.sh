#!/bin/bash
# grab.sh TITLE OUT.png -- one frame of a window (composite contents; covered windows are fine)
id=$(/usr/bin/python3 "$(dirname "$0")/xwin.py" find "$1") || exit 1
gst-launch-1.0 -q ximagesrc xid=$id use-damage=false show-pointer=false num-buffers=1 ! videoconvert ! pngenc ! filesink location="$2"
