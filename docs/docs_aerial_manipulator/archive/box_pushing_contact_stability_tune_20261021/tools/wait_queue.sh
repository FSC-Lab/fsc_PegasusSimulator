#!/bin/bash
# Waits for the runner to empty the queue (a "queue empty" line written after line $1 of the
# log), then stops it and waits for its exit. Run in the background.
C="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
N=$1
while ls $C/queue/*.job > /dev/null 2>&1 || ! tail -n +"$N" $C/runner.log | grep -q "queue empty"; do sleep 30; done
touch $C/STOP
while ! tail -n +"$N" $C/runner.log | grep -q "RUNNER DONE"; do sleep 10; done
rm -f $C/STOP
tail -n +"$N" $C/runner.log | grep -E "^\[|ABORT|COMPLETE|left DIRECT" | tail -40
