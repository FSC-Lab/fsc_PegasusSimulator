#!/bin/bash
# Wait for a PID to exit (a check that shares the ROS graph), then run chain_next.sh with the given jobs.
#   setsid nohup bash after.sh <pid> [job ...] > chain.out 2>&1 &
P=$1; shift
while kill -0 "$P" 2>/dev/null; do sleep 5; done
echo "$(date +%H:%M:%S) pid $P gone -- starting the chain"
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/chain_next.sh" "$@"
