#!/usr/bin/env bash
# est_merge.sh <out tag> <src tag>...  : one directory of symlinks under $EST_DIR so est_metrics.py prints
# the runs of several sweeps (e.g. the velocity twins and the IMU twin at one rate) as ONE table.
E=$HOME/flight_bags/est_sweeps; OUT=$E/$1; shift; rm -rf "$OUT"; mkdir -p "$OUT"
for src in "$@"; do for f in $E/$src/*_p*_s*.txt $E/$src/*_p*_s*.csv; do ln -sf "$f" "$OUT/$(basename "$f")"; done; done
echo "merged: $*" > "$OUT/config"; cat $E/$1/ctrl_commit > "$OUT/ctrl_commit" 2>/dev/null || true
~/envs/isaac_env/bin/python3 /home/pieper/envs/fsc_PegasusSimulator/scripts/est_metrics.py "$OUT" | tee "$OUT/summary.txt"
