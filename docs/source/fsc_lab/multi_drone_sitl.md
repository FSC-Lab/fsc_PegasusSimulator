# Multi Drone SITL (3 drones, no payload)

Launches three independent PX4 SITL instances (`uav_0`, `uav_1`, `uav_2`) alongside a multi-vehicle Pegasus scene.

```bash
./scripts/indoor_sim/start_multi_drone_sitl.sh <machine_config_name>
```

Example:

```bash
./scripts/indoor_sim/start_multi_drone_sitl.sh longhao_machine
```

**Pegasus entrypoint:** `application/px4_base/02_px4_multi_drone.py`

**What it does:**
1. Opens a new terminal and creates (or replaces) a `tmux` session named `px4_isaac` with 4 evenly-split panes.
2. Detects the PX4 SITL rootfs (`build/px4_sitl_default/rootfs` or `.../tmp/rootfs` — requires `make px4_sitl_default` to have been run at least once).
3. Creates a persistent per-vehicle rootfs copy (`rootfs_uav0/1/2`) if one doesn't already exist, so each instance keeps its own EEPROM/parameters across runs.
4. Panes 0–2: one `./build/px4_sitl_default/bin/px4 -i {0,1,2} -w rootfs_uavN` each, model `iris`, autostart `10016`, on TCP ports 4560/4561/4562.
5. Pane 3: waits 2 s, then runs the Pegasus entrypoint through `$ISAAC_PY`.

**Prerequisites:** run `make px4_sitl_default` in `$PX4_DIR` at least once before first use; `scripts/config/<machine_config_name>.conf` must exist (see [FSC Lab Launch Scripts](index.md)).
