# Single Drone SITL with Slung-Load Payload

Launches one PX4 SITL instance carrying a slung-load payload.

```bash
./scripts/indoor_sim/start_single_drone_sitl_payload.sh <machine_config_name>
```

Example:

```bash
./scripts/indoor_sim/start_single_drone_sitl_payload.sh longhao_machine
```

**Pegasus entrypoint:** `application/slungload/01_px4_single_drone_payload.py`

**What it does:**
1. Opens a new terminal and creates (or replaces) a `tmux` session named `px4_isaac` with two side-by-side panes.
2. Left pane: `cd $PX4_DIR && make px4_sitl none_iris`, with `PX4_UXRCE_DDS_NS=uav_0`, mavlink remote `127.0.0.1:14540`.
3. Right pane: waits 2 s, then runs the Pegasus entrypoint through `$ISAAC_PY`.

**Prerequisites:** `PX4_DIR` must be a PX4-Autopilot checkout; `scripts/config/<machine_config_name>.conf` must exist (see [FSC Lab Launch Scripts](index.md)).
