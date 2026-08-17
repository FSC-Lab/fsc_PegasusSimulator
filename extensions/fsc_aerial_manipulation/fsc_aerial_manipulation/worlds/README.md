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

### Scale caveat

Metric scale comes from the gate's inner opening **width**, measured in the field as
1.50 m. The reconstructed opening is landscape (1.49 x 1.17 m). If that 1.50 m
measurement was actually the **height**, every distance rescales by ~1.22 and the
scene must be regenerated. Sanity check against reality: the opening height should
be ~1.17 m and the sill ~1.10 m off the floor.

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
