# Multi Drone SITL — Point-Mass Payload (3 drones)

Same three-instance SITL setup as [Multi Drone SITL](multi_drone_sitl.md), carrying a shared point-mass payload.

```bash
./scripts/start_3_drone_point_mass_payload_sitl.sh <machine_config_name>
```

Example:

```bash
./scripts/start_3_drone_point_mass_payload_sitl.sh longhao_machine
```

**Pegasus entrypoint:** `application/slungload/03_px4_multi_drone_point_mass_payload.py`

**What it does:** identical `tmux` / per-vehicle rootfs / PX4 launch as `start_multi_drone_sitl.sh` — only the Pegasus entrypoint differs.

**Prerequisites:** run `make px4_sitl_default` in `$PX4_DIR` at least once before first use; `scripts/config/<machine_config_name>.conf` must exist (see [FSC Lab Launch Scripts](index.md)).
