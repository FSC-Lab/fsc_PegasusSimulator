#!/usr/bin/env python3
"""The compute budget as a regression floor (hardware-transfer piece 6, 2026-09-16).

  ~/envs/isaac_env/bin/python3 tests/test_bench_budget.py [--ticks N] [--outer N]

Runs scripts/bench_loops.py --load --json and holds the desktop numbers where they were measured
on 2026-09-16 (inner tick 0.41 ms median / 0.44 p95, 0.43 / 0.57 under the outer's load; outer solve
11.5 ms median) with room for machine noise -- the floor is there to catch a regression to what the
tick cost BEFORE the predictor's hot path was made cheap (1.32 ms: np.cross at 30 us a call and
a forward integration for a log nobody read), not to pin a decimal. The same script on the Jetson
Orin produces the budget table that goes in CLAUDE.md; the pass line there is inner p95 < 3 ms at
200 Hz, outer p95 < 25 ms at 20 Hz (the plan's piece 6)."""
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
PY = os.path.expanduser("~/envs/isaac_env/bin/python3")
ticks = int(sys.argv[sys.argv.index("--ticks") + 1]) if "--ticks" in sys.argv else 800
outer = int(sys.argv[sys.argv.index("--outer") + 1]) if "--outer" in sys.argv else 20
out = os.path.join(os.environ.get("TMPDIR", "/tmp"), f"bench_budget_{os.getpid()}.json")

# the desktop floor (a 2x margin on the medians, ~3x on the p95s: a loaded desktop is noisy,
# a 2.5x regression is what this catches)
FLOOR = {
    "inner_step (all)":                  {"median_ms": 0.85, "p95_ms": 1.3},
    "inner_step (all) UNDER OUTER LOAD": {"median_ms": 0.90, "p95_ms": 1.6},
    "outer_solve":                       {"median_ms": 20.0, "p95_ms": 25.0},
    "outer_solve UNDER INNER LOAD":      {"median_ms": 22.0, "p95_ms": 28.0},
}

r = subprocess.run([PY, os.path.join(ROOT, "scripts", "bench_loops.py"), "--ticks", str(ticks), "--outer", str(outer), "--load", "--json", out],
                   capture_output=True, text=True, timeout=900)
if r.returncode != 0:
    print(r.stdout[-3000:]); print(r.stderr[-3000:]); print("FAIL: the bench did not run"); sys.exit(1)
res = json.load(open(out)); os.unlink(out)
fails = 0
for k, lim in FLOOR.items():
    if k not in res:
        print(f"  FAIL {k}: not measured"); fails += 1; continue
    med, p95 = res[k]["median_ms"], res[k]["p95_ms"]
    ok = med <= lim["median_ms"] and p95 <= lim["p95_ms"]
    fails += 0 if ok else 1
    print(f"  {'ok  ' if ok else 'FAIL'} {k:36s} median {med:6.3f} ms (floor {lim['median_ms']:5.2f})  p95 {p95:6.3f} (floor {lim['p95_ms']:5.2f})")
inner = res["inner_step (all) UNDER OUTER LOAD"]; osv = res["outer_solve UNDER INNER LOAD"]
print(f"  Orin x2 projection: inner tick {2 * inner['median_ms']:.2f} / p95 {2 * inner['p95_ms']:.2f} ms of a 5 ms tick at 200 Hz "
      f"({100 * 2 * inner['p95_ms'] / 5:.0f}% p95); outer {2 * osv['median_ms']:.1f} / p95 {2 * osv['p95_ms']:.1f} ms of 50 ms at 20 Hz")
print("BENCH BUDGET: " + ("ALL PASS" if fails == 0 else f"{fails} FAILED"))
sys.exit(1 if fails else 0)
