# Single Drone SITL Payload — Testing Variant

Same layout as [Single Drone SITL with Slung-Load Payload](single_drone_sitl_payload.md), pointed at the payload testing scenario (e.g. a variable-length cable / testing setup) instead of the standard one.

```bash
./scripts/indoor_sim/start_single_drone_sitl_payload_test.sh <machine_config_name>
```

Example:

```bash
./scripts/indoor_sim/start_single_drone_sitl_payload_test.sh longhao_machine
```

**Pegasus entrypoint:** `application/slungload/05_px4_single_drone_payload_testing.py`

**What it does:** identical `tmux` layout and PX4 launch as `start_single_drone_sitl_payload.sh` — only the Pegasus entrypoint differs.

**Prerequisites:** `PX4_DIR` must be a PX4-Autopilot checkout; `scripts/config/<machine_config_name>.conf` must exist (see [FSC Lab Launch Scripts](index.md)).
