# Single Drone SITL

Launches one PX4 SITL instance flying the default Iris multirotor via `pegasus.simulator`, no payload.

```bash
./scripts/start_single_drone_sitl.sh <machine_config_name>
```

Example:

```bash
./scripts/start_single_drone_sitl.sh longhao_machine
```

**Pegasus entrypoint:** `application/px4_base/01_px4_single_drone.py`

**What it does:**
1. Opens a new terminal and creates (or replaces) a `tmux` session named `px4_isaac` with two side-by-side panes.
2. Left pane: `cd $PX4_DIR && make px4_sitl none_iris`, with `PX4_UXRCE_DDS_NS=uav_0`, mavlink remote `127.0.0.1:14540`.
3. Right pane: waits 2 s, then runs the Pegasus entrypoint through `$ISAAC_PY`.

**Prerequisites:** `PX4_DIR` must be a PX4-Autopilot checkout (build happens on first run via `make`); `scripts/config/<machine_config_name>.conf` must exist (see [FSC Lab Launch Scripts](index.md)).
