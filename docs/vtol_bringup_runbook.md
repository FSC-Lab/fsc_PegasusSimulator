# VTOL bring-up runbook (dual independent tiltrotor)

Sequenced checklist to bring the dual-tiltrotor VTOL up in Isaac Sim / Pegasus,
from "does the articulation build" to full PX4 `direct_actuator` HIL. Do the
stages **in order** — each proves one layer so a later failure is easy to localise.

Legend: 🖥️ = run in Isaac Sim (clean shell, **no ROS sourced**) · 🤖 = ROS 2 shell.

> Machine paths come from `scripts/config/<machine>.conf` (e.g. `pieper_machine`):
> `PX4_DIR=~/PX4-Autopilot`, `ISAAC_PY=~/workspaces/isaacsim/python.sh`,
> `FSC_PEGASUS_ROOT=/home/pieper/envs/fsc_PegasusSimulator`.

---

## 0. One-time setup

1. **PX4 airframe built in.** The `4100_fsc_dual_tiltrotor` airframe is installed +
   registered (see [`px4/airframes/README.md`](../px4/airframes/README.md)). Bake it
   into the SITL ROMFS:
   ```sh
   cd ~/PX4-Autopilot && make px4_sitl        # regenerates build/.../etc + rootfs
   rm -rf ~/PX4-Autopilot/build/px4_sitl_default/rootfs_vtol0   # drop stale per-run workdir
   ```
2. **ROS 2 node built.** (Humble, in `~/ros2_ws`; the package is symlinked into `src/`.)
   ```sh
   cd ~/ros2_ws && source /opt/ros/humble/setup.bash
   colcon build            # full build so the top-level setup.bash lists the pkg
   ```
   > If you use `--packages-select`, the top-level `install/setup.bash` order file
   > goes stale and `ros2 run` can't find the pkg. A plain full `colcon build` fixes it.
3. **Micro XRCE-DDS Agent** available on the PATH (`MicroXRCEAgent`).

---

## Stage 1 — Articulation only 🖥️ (`01_basic_vtol.py`)

Gravity off, no forces. Proves the procedural PhysX articulation builds and the
joints animate. **No PX4, no ROS 2.**

```sh
cd /home/pieper/envs/fsc_PegasusSimulator
env -u PYTHONHOME -u PYTHONPATH -u CONDA_PREFIX -u VIRTUAL_ENV \
  PYTHONEXE=~/workspaces/isaacsim/kit/python/bin/python3 \
  ~/workspaces/isaacsim/python.sh application/vtol/01_basic_vtol.py
```

**PASS:** vehicle spawns and floats; front rotors sweep hover↔cruise (0→90°),
props spin, elevons wiggle. No prims explode / no articulation errors in the log.

**If it fails:** this is the riskiest structural assumption (ArticulationRoot on
the top Xform + child rigid `/body`). Check the console for PhysX articulation
warnings; verify each rotor/surface link + joint under `/dual_tiltrotor_0/`.

---

## Stage 2 — Forces + stand-in controller 🖥️ (`02_vtol_hover.py`)

Gravity + `enable_forces=True` + a throwaway hover controller (stand-in for PX4).
Proves propulsion (local-+Z thrust vectoring) and the aero model. **No PX4/ROS 2.**

```sh
cd /home/pieper/envs/fsc_PegasusSimulator
env -u PYTHONHOME -u PYTHONPATH -u CONDA_PREFIX -u VIRTUAL_ENV \
  PYTHONEXE=~/workspaces/isaacsim/kit/python/bin/python3 \
  ~/workspaces/isaacsim/python.sh application/vtol/02_vtol_hover.py
```

**PASS:** holds ~1.5 m and stays roughly level (doesn't flip). Set
`FORWARD_PULSE=True` in the scene to see a brief forward tilt → accelerate → return.

**Tune here (not later):** hover instability → controller gains (`KP_*` in the
scene) and/or rotor `max_thrust` in `dual_tiltrotor_config()`. Drift/oscillation
in cruise → aero coefficients in `VTOLAerodynamicsConfig`.

---

## Stage 3 — Full PX4 HIL 🖥️🤖 (`03_vtol_px4_hil.py`)

**One-command option** — brings up all four pieces in one tmux session (DDS agent
+ PX4 + Isaac + node, self-sequenced):
```sh
cd /home/pieper/envs/fsc_PegasusSimulator
scripts/start_vtol_hil.sh pieper_machine
```
Defaults are safe (motors off, tilts at hover, `auto_arm=false`, `PEGASUS_HIL_DEBUG=1`).
Override on the command line, e.g. `AUTO_ARM=true MOTOR_LEFT=0.6 MOTOR_RIGHT=0.6
scripts/start_vtol_hil.sh pieper_machine`. Optional config/env vars: `ROS_DISTRO`,
`ROS2_WS`, `DDS_AGENT_PORT`, `PX4_UXRCE_DDS_NS`, `PX4_SIM_MODEL`, `PX4_SYS_AUTOSTART`.
(`scripts/start_vtol_sitl.sh` remains the PX4+Isaac-only subset.)

**Teardown:** `scripts/stop_vtol_hil.sh` kills the whole stack (tmux sessions + PX4
+ Isaac + node, and the DDS agent unless `--keep-agent`). It targets VTOL-unique
markers, so a co-running single-drone SITL is left alone. `--dry-run` to preview,
`--tmux-only` for a soft stop.

The manual breakdown below is the same four pieces, for understanding / debugging.
**Start them in this order:**

**3a. DDS agent** 🤖
```sh
MicroXRCEAgent udp4 -p 8888
```

**3b. PX4 SITL + Isaac Sim** (tmux: PX4 pane + Isaac pane) 🖥️
```sh
cd /home/pieper/envs/fsc_PegasusSimulator
PEGASUS_HIL_DEBUG=1 scripts/start_vtol_sitl.sh pieper_machine
```
Wait until Isaac is up and the PX4 pane shows the vehicle booted
(`INFO [init] found ... SYS_AUTOSTART=4100`) and the HIL sensor link connected.

**3c. Direct-actuator node** 🤖 (streams setpoints, then auto OFFBOARD + ARM)
```sh
cd ~/ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch px4_vtol_direct_actuator vtol_direct_actuator.launch.py \
    px4_ns:=uav_0 motor_left:=0.6 motor_right:=0.6
```
Live-edit setpoints without restarting:
```sh
ros2 param set /vtol_direct_actuator motor_left 0.7
ros2 param set /vtol_direct_actuator tilt_left 0.0      # -1 hover … +1 cruise
```

**PASS:** node logs `arming_state=2 (armed=True)` + `nav_state=14 (offboard=True)`;
the props spin and the airframe responds in Isaac.

> `direct_actuator` is fully open-loop (no stabilization) — equal thrust will climb
> and tumble. That's correct; a stable hover needs your own per-actuator controller.

---

## Verify the actuator chain (do this the FIRST time)

The one thing that isn't provable by inspection: the actuator **index order**
(ROS 2 `SERVO_ORDER` ↔ PX4 output assignment ↔ Pegasus `channel_map`).

With `PEGASUS_HIL_DEBUG=1`, the Isaac pane logs every ~0.5 s:
```
[VTOL_HIL vid=0] armed=True mode=129 controls[:8]=[...] -> {'motor_left': .., 'tilt_left': .., ...}
```
Command **one actuator at a time** and confirm the expected index moves. Target map
(designed into airframe `4100_fsc_dual_tiltrotor`):

| set param      | expected `controls[i]` | vtol channel |
|----------------|:----------------------:|--------------|
| `motor_left`   | 0                      | motor_left   |
| `motor_right`  | 1                      | motor_right  |
| `tilt_left`    | 2                      | tilt_left    |
| `tilt_right`   | 3                      | tilt_right   |
| `aileron`      | 4                      | aileron      |
| `elevator`     | 5                      | elevator     |
| `rudder`       | 6                      | rudder       |

**If an index is wrong:** the airframe `PWM_MAIN_FUNCn` assignment and the Pegasus
map disagree. Fix the fast side — `default_dual_tiltrotor_channel_map()` in
[`vtol_mavlink_backend.py`](../extensions/pegasus.simulator/pegasus/simulator/logic/backends/vtol_mavlink_backend.py)
`(hil_index, scale, offset)` — no rebuild needed. If a channel never appears,
`VTOL._validate_channels()` will already have warned that nothing matches.

---

## Mocap feed (optional — for the controller)

Produces the `/uav_0/mocap` feedback a controller needs, via the OptiTrack
emulator. Two pieces:

1. **Isaac publishes state** — start the HIL stack with state publishing on:
   ```sh
   VTOL_ROS2_STATE=1 scripts/start_vtol_hil.sh pieper_machine
   ```
   This attaches a state-only `ROS2Backend` (namespace `uav_`) to the VTOL →
   `/uav_0/state/{pose,twist,twist_inertial}`. Off by default (plain HIL).
2. **Run the emulator** (separate ROS 2 shell):
   ```sh
   scripts/start_vtol_mocap.sh          # BODIES="uav_0" by default
   ```

Data flow:
```
Isaac VTOL ─[ROS2Backend]→ /uav_0/state/* ─[emulator]→ /uav_0/mocap ─→ [controller: TODO]
```
Verify: `ros2 topic hz /uav_0/mocap` and `ros2 topic echo /uav_0/state/pose`.
The control integration (a node subscribing `/uav_0/mocap` and driving the
direct_actuator setpoints) is not wired yet.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `ERROR [init] Unknown model fsc_dual_tiltrotor` | airframe not in the built ROMFS | rebuild PX4 (§0.1) or use the pre-dropped copies in `build/.../airframes/` |
| Won't arm / "control allocation" preflight fail | 2-rotor tiltrotor is under-actuated for PX4 hover allocation | `direct_actuator` bypasses allocation for outputs — relax preflight (`param set COM_PREARM_MODE 0`, disable the offending arming check) in the `pxh>` console |
| `Switching to Offboard is currently not available` | DDS timestamp re-stamping under HIL sim-time | `param set UXRCE_DDS_SYNCT 0` (baked into the airframe; verify it took) |
| Disarms ~2 s after arming | auto-disarm (never "took off") | baked `COM_DISARM_LAND/PRFLT 0`; verify, or command lift-off thrust |
| `simulator_mavlink poll timeout`, Isaac freezes | HIL lockstep deadlock | ensure `enable_lockstep: False` (already set in `spawn_vtol_with_mavlink`) |
| `ros2 run … package not found` | stale top-level `install/setup.bash` | full `colcon build` (§0.2) or source the pkg prefix directly |
| Isaac crashes on launch (rclpy ABI) | launched from a ROS-sourced shell | run scenes from a **clean** shell with `PYTHONPATH` unset (as in the Stage 1/2 commands) |
| `gnome-terminal … undefined symbol: __libc_pthread_init` | sourced env put snap libs on `LD_LIBRARY_PATH`, breaking the GUI terminal | fixed in `terminal_utils.sh` (spawns terminal with `LD_LIBRARY_PATH`/`LD_PRELOAD` unset); or run the launcher from a shell without Isaac/conda sourced |
| `/uav_0/mocap` silent / no output | Isaac not publishing `/uav_0/state/*` | launch with `VTOL_ROS2_STATE=1` (see Mocap feed) |
| No actuator moves; `[VTOL_HIL] ... -> {}` | disarmed (empty command is the safe state) | arm in OFFBOARD; command non-zero `motor_*` |

See [`direct_actuator_offboard.md`](direct_actuator_offboard.md) for the deep DDS /
offboard-mode rationale.
