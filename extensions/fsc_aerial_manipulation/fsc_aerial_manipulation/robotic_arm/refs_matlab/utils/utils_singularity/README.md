# Singularity analysis — modified aerial-manipulator arm

Operational singularity of the impedance-controlled arm, certified via the smallest
singular value of `J_3y0(q)`. Everything lives in [singularity_sweep.m](singularity_sweep.m);
run it as a script. Toggle the three `run_*` flags near the top to pick what executes.

## Structure

| Section | `run_*` flag | Output |
|---|---|---|
| 1. Parameters | — | link lengths, masses, measured CoMs `Ccom`, `(q2,q3)` box |
| 2. Preliminary | `run_preliminary` | one-time `(q2,q3)` calibration (EE below shoulder) + geometry checks |
| 3. Simplified case (collinear CoM `c_k = γ_k l_k`) | `run_simplified` | **Fig 1** `σ_min\|σ_max\|κ` fields, **Fig 2** analytic branches, **Fig 3** task-space workspace |
| 4. Actual case (full measured CoM) | `run_actual` | **Fig 4** fields (top `q4=0` / bottom worst-`q4`), **Fig 5** task-space workspace |
| 5. Comparison | both on | **Fig 6** simplified vs actual-worst `σ_min` + `\|Δσ_min\|` |

Metrics are non-dimensionalized (translational rows scaled by `Lchar`). The **safe region**
is the super-level set `{q : σ_min(J_3y0) ≥ ε}`, with margin `ε = 0.10`.

## Main results

- **`det J_3y0 = det B_xz · det B_yψ`**, exactly `q1`-invariant, so the singular set lives in
  the `(q2,q3)` plane as **two branches**: an *elbow* branch (`det B_xz = 0`, fixed `q3`) and a
  *yaw* branch (`det B_yψ = 0`), plotted as reference overlays.
- **`q4` dependence (actual case only):** the true off-axis `c_4` reintroduces `q4`. Its effect
  is a **≤1.1 % of margin** modulation of `σ_min`, extremized near `q4 = ±180°`, and it flips
  **0** points across the safe/unsafe boundary. The per-point worst `q4` is bimodal
  (~`-180°`/~`0°`), but a single hardcoded `q4_worst = -180` reproduces the worst-case field to
  within that tolerance. Set `run_q4search = true` once to re-verify the gap.
- **Simplified vs actual-worst:** `σ_min` differs by at most **~2 % of the margin**, i.e. the
  clean collinear-CoM model is an accurate stand-in for the measured arm.

Net: over the calibrated joint box (and payload range via `mp`), a modest keep-out margin
around the two branches certifies a non-degenerate arm configuration throughout.
