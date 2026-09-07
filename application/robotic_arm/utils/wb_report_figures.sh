#!/usr/bin/env bash
#   wb_report_figures.sh <gmo.npz> <l1.npz> <outdir>
# Two interpreters, same reason as wb_compare_figure.sh: the npz object arrays
# only unpickle under numpy 2, the apt matplotlib only imports under numpy 1.
set -euo pipefail
G="$1"; L="$2"; OUT="$3"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MARKS="$(/usr/bin/python3 - "$L" <<'PY'
import sys, numpy as np
z=np.load(sys.argv[1],allow_pickle=True); dbg=z["dbg"]; d=dbg[:,1:]
m=np.nan_to_num(d[:,0])>0.5; t0=dbg[m,0][0]
print(",".join(f"{str(s).split(' ',1)[1]}:{float(str(s).split(' ',1)[0])-t0:.2f}"
                for s in z["leg_marks"]))
PY
)"
PYTHONNOUSERSITE=1 /usr/bin/python3 "$HERE/wb_report_figures.py" \
    --gmo "$G" --l1 "$L" --outdir "$OUT" --marks "$MARKS"
