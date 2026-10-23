#!/usr/bin/env bash
# One command for the GMO-vs-L1 comparison figure.
#
#   wb_compare_figure.sh <gmo.npz> <l1.npz> <out.png> ["title"]
#
# Two interpreters on purpose. The npz files are written by a numpy 2 process
# (the driver runs under /usr/bin/python3, which picks up ~/.local numpy
# 2.2.6), so their OBJECT arrays -- leg_marks, events -- only unpickle under
# numpy 2. But the apt matplotlib on this machine is built against numpy 1 and
# fails to import when numpy 2 is on the path. So: read the leg marks with the
# user-site numpy, then plot with PYTHONNOUSERSITE=1 and pass the marks in on
# the command line.
set -euo pipefail
GMO="$1"; L1="$2"; OUT="$3"; TITLE="${4:-Whole-body DIRECT: GMO vs L1}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

MARKS="$(/usr/bin/python3 - "$L1" <<'PY'
import sys, numpy as np
z = np.load(sys.argv[1], allow_pickle=True)
dbg = z["dbg"]; d = dbg[:, 1:]
m = np.nan_to_num(d[:, 0]) > 0.5
t0 = dbg[m, 0][0]
out = []
for s in z["leg_marks"]:
    a, b = str(s).split(" ", 1)
    out.append(f"{b}:{float(a)-t0:.2f}")
print(",".join(out))
PY
)"

PYTHONNOUSERSITE=1 /usr/bin/python3 "$HERE/wb_compare_plot.py" \
    --gmo "$GMO" --l1 "$L1" --out "$OUT" --title "$TITLE" --marks "$MARKS"
