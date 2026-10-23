# Reconstructed world assets

Real-world scenes reconstructed from camera capture and rendered natively by Isaac
Sim's **NuRec** neural-volume renderer.

## Gate splat (A2RL x DCL practice gate)

3D Gaussian splat of the drone racing gate, captured on an iPhone and reconstructed
with [spirulae-splat](https://github.com/harry7557558/spirulae-splat).

| | |
|---|---|
| `assets/gate_metric.usda` | 558 B wrapper: metric scale + gravity alignment + origin |
| `assets/gate.usdz` | 105 MB payload: 924,710 Gaussians (NuRec volume) |

### These files are not in git

`.gitignore:52` excludes `extensions/fsc_aerial_manipulation/**/assets/`, the same
rule that keeps `x650_new.usd` and `AM_xfwd.usda` out. **A fresh clone has neither
the gate scene nor the vehicle USD**, so this asset is no more of an obstacle to a
new user than the drone itself already is — both arrive in the same out-of-band
bundle (currently Google Drive).

Drop both files into `assets/` side by side. `gate_metric.usda` references
`@./gate.usdz@` *relatively*: if the payload is missing the scene composes to an
empty Xform and renders nothing **with no USD error**, which is why
`gate_splat.gate_splat_usd_path()` checks for it explicitly.

Alternatively point `GATE_SPLAT_USD` at a copy anywhere on disk.

### Scene frame

Metric (metres), Z up, gravity-aligned, floor at `z = 0`.

```
    +Z  up
     |          gate plane = YZ plane at x = 0
     |          flying THROUGH the gate = motion along X
     o------ +Y
    /
  +X   capture side; the drone spawns here facing -X
```

| quantity | value |
|---|---|
| opening centre | `(0, 0, 1.746)` m |
| opening | 1.493 m wide x 1.165 m tall |
| sill / lintel | `z = 1.102` / `z = 2.328` m |
| default spawn | `(3, 0, 0.07)`, yaw 180 deg |

Import these from `gate_splat.py` rather than hardcoding them — that module *is* in
git, so it works on a fresh clone before the asset is downloaded.

**No collision geometry.** The splat is vision-only; a NuRec volume has no physics
representation. The scenario script adds an invisible PhysX ground plane at `z = 0`.
The drone will pass straight through the gate frame unless a mesh collision proxy is
added.

### Verified against the asset (2026-08-19)

The table above was re-derived from `gate.usdz` itself, not from the build-time
JSON, after a flight test suggested the gate sat lower than documented. It does
not: transforming all 924,710 gaussians into world coordinates puts the **floor
at z = -0.046 m** and a clean **1.40 m-wide aperture spanning z = 1.33 .. 2.28 m**,
centred on y = 0 -- consistent with the recorded sill 1.102 / centre 1.746 /
lintel 2.328 / width 1.493 to within the 5 cm measuring grid. **The constants in
`gate_splat.py` are correct**; a vehicle hovering below ~1.1 m is under the sill,
and ~1.75 m is the centre of the opening.

**The trap, if you ever re-measure.** The world transform is NOT
`gate_metric.json`'s `M_world_from_ply` alone. `gate.usdz` carries its own
axis-swap on `/World/gauss/gauss`,

```
xformOp:transform = ((-1,0,0,0), (0,0,-1,0), (0,-1,0,0), (0,0,0,1))   # (x,y,z) -> (-x,-z,-y)
```

so the full chain is **`M_world_from_ply . S`**. Applying `M` on its own to
`splat.ply` scatters the scene (the gate plane at x = 0 comes out empty, and the
"floor" lands at z = 0.79); composing the swap first reproduces the recorded
opening corners to 0.000 m. Note also that `gate_scale.json`'s
`scale_m_per_unit` (0.896, the mean of all four measured sides) is NOT the one
that was baked -- `make_metric_usd.py` re-measures and uses the width alone
(1.5 / 1.845 = 0.813).

### Scale caveat -- RESOLVED 2026-08-19

Metric scale comes from the gate's inner opening **width**, measured in the field as
1.50 m. The reconstructed opening is landscape (1.49 x 1.17 m). The open question was
whether that 1.50 m was actually the **height**, which would rescale every distance by
1.287. **It was the width** (confirmed by Shiqi 2026-08-19), so the shipped scale
0.8130 m/unit stands and the scene does NOT need regenerating.

Three things corroborate it, worth keeping if the question is ever reopened:

* **Capture-camera height.** `gate_metric.json` records `camera_mean_height_m`
  **1.613 m** -- where a handheld phone rides. The height hypothesis would put it at
  2.08 m, i.e. filmed overhead for the whole capture.
* **The opening really is landscape**, visible directly in
  `measure_debug/heldout_*.jpg`, so the aspect is not inverted.
* **Width is the well-measured edge pair.** In `gate_scale.json` the top and bottom
  sides agree to 0.04% (1.8369 / 1.8376 units) while left and right disagree by 10%
  (1.4335 / 1.5859). Anchoring scale to the width was therefore the right choice --
  and it is why `gate_scale.json`'s own `scale_m_per_unit` (0.896, the mean of all
  four sides) is not the number that was baked.

### This gate is TIGHT for a 650-class quad

Not a defect, but it surprises people and reads as a scaling error. The T650 is
**1.031 m tip to tip** (0.650 m motor diagonal + 15" props) against a 1.493 x 1.165 m
opening, so it fills **69% of the width and 88% of the height** -- roughly 7 cm of
clearance above and below when perfectly centred. Fly it through at the opening
centre, `z ~ 1.68 m`.

The rigorous fix is an AprilTag re-capture (>= 3 non-collinear tags with measured
centres + a `markers.yaml`), which recovers scale, gravity and origin in one solve
via spirulae-splat's `align_apriltag.py`.

### Regenerating

See `tools/gate_splat/`. Requires the original capture dataset (COLMAP
`transforms.json` + the trained `splat.ply`), which is also not in git.

```bash
# 1. measure the opening in reconstruction units -> gate_scale.json
~/miniconda3/envs/py312/bin/python tools/gate_splat/measure_gate_opening.py
# 2. bake scale + gravity + origin -> gate_metric.usda (+ .json)
~/miniconda3/envs/3dgrut/bin/python tools/gate_splat/make_metric_usd.py
```

Then copy `gate_metric.usda` and `gate.usdz` into `assets/` and update the constants
in `gate_splat.py` from the regenerated `gate_metric.json`.
