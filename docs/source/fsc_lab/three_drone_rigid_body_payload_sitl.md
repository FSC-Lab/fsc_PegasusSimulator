# Multi Drone SITL — Rigid-Body Payload (3 drones)

```{note}
`scripts/start_3_drone_rigid_body_payload_sitl.sh` predates the machine-config system used by every other script on this page and has **not** been migrated to it. It hardcodes `PX4_DIR`, `PEGASUS_SCRIPT`, and `ISAAC_PY` for the original author's machine (including a path under the pre-fork `PegasusSimulator/examples/` layout that no longer matches this repository's `application/` layout), does not source `scripts/common_config.sh`, and takes no config-name argument.

**Do not run it as-is.** The corresponding scenario now lives at `application/slungload/04_px4_multi_drone_rigidbody_payload.py`. Until this script is migrated, either launch that entrypoint manually, or copy `scripts/start_3_drone_point_mass_payload_sitl.sh` and swap in the rigid-body entrypoint — that script is already on the config-driven pattern used everywhere else in this folder.
```

**Intended scenario:** three PX4 SITL instances (`uav_0`, `uav_1`, `uav_2`) carrying a shared rigid-body payload, analogous to [the point-mass variant](three_drone_point_mass_payload_sitl.md).
