# Direct-actuator offboard control (PX4 SITL + Isaac Sim / Pegasus HIL)

This document describes how to drive a PX4 vehicle in **offboard
`direct_actuator` mode** — sending raw per-motor commands that bypass PX4's
position/attitude/rate controllers — while the vehicle is simulated in Isaac
Sim through the Pegasus MAVLink HIL backend.

It also records the changes that were required to make this work, and the
issues encountered along the way, so the setup is reproducible.

---

## Architecture

```
                 HIL_SENSOR / HIL_ACTUATOR_CONTROLS (MAVLink, TCP 4560)
  PX4 SITL  <───────────────────────────────────────────────────────>  Isaac Sim / Pegasus
 (none_iris)                                                            (PX4MavlinkBackend)
      │
      │  uXRCE-DDS  (Micro XRCE-DDS Agent, UDP 8888)
      ▼
  /uav_0/fmu/in/offboard_control_mode   ◄── px4_direct_actuator_test (ROS 2 node)
  /uav_0/fmu/in/actuator_motors         ◄──
  /uav_0/fmu/in/vehicle_command         ◄──
  /uav_0/fmu/out/vehicle_status         ──►
```

* The **HIL link** (TCP 4560) carries sensors *to* PX4 and the computed actuator
  outputs *back* to the simulator. It is **mode-agnostic** — PX4 sends the same
  `HIL_ACTUATOR_CONTROLS` message regardless of flight mode.
* **`direct_actuator` cannot be commanded over MAVLink.** PX4's MAVLink receiver
  only maps `SET_POSITION_TARGET_*` / `SET_ATTITUDE_TARGET` offboard setpoints,
  none of which set `OffboardControlMode.direct_actuator`. It is only reachable
  over **uXRCE-DDS (ROS 2)** by publishing `OffboardControlMode{direct_actuator:
  true}` together with `ActuatorMotors`.

---

## Required changes (summary)

| # | Change | Location | Why |
|---|--------|----------|-----|
| 1 | `enable_lockstep: False` | `extensions/fsc_aerial_manipulation/.../rotorcraft/rotorcraft_utils.py` | The HIL lockstep handshake deadlocked the moment DDS offboard traffic made PX4 skip a control cycle. |
| 2 | Robust arming check + optional `PEGASUS_HIL_DEBUG` logging | `extensions/pegasus.simulator/.../backends/px4_mavlink_backend.py` | Bitmask arming test (`mode & MAV_MODE_FLAG_SAFETY_ARMED`) instead of an exact `== 129`; runtime visibility into the actuator path. |
| 3 | `px4_direct_actuator_test` ROS 2 node | `ros2/px4_direct_actuator_test/` | Streams the offboard setpoints, switches mode, and arms. |
| 4 | PX4 parameters (runtime) | PX4 SITL (`~/PX4-Autopilot`) | `UXRCE_DDS_SYNCT=0` is the key fix; `COM_DISARM_*` allow ground testing. |

---

## PX4 parameters (one-time, in the `pxh>` console)

```sh
param set UXRCE_DDS_SYNCT 0      # do not re-stamp incoming DDS timestamps (see note)
param set COM_DISARM_LAND 0      # don't auto-disarm 2 s after "landed" (ground testing)
param set COM_DISARM_PRFLT 0     # don't auto-disarm 10 s after arming without takeoff
param save
```

> **Why `UXRCE_DDS_SYNCT 0` matters.** PX4 SITL runs on *simulation time*, while a
> ROS 2 node stamps messages in wall-clock time. With timestamp sync **on**
> (the default), PX4's uXRCE-DDS client re-stamps every incoming message using a
> timesync offset. In a HIL setup sim-time never tracks wall-time cleanly, so the
> re-stamped `offboard_control_mode.timestamp` comes out **stale**, the
> `data_is_recent` check in `offboardCheck.cpp` fails, and PX4 refuses the
> offboard switch (`"Switching to Offboard is currently not available"`). With
> sync **off**, PX4 uses the raw timestamp as-is; since wall-clock ≫ sim-time the
> recency test always passes and offboard becomes available.

These persist in the SITL parameter file after `param save`, so they only need
to be set once per build.

---

## Running it

Four pieces, in order:

**1. Micro XRCE-DDS Agent**
```sh
MicroXRCEAgent udp4 -p 8888
```

**2. PX4 SITL + Isaac Sim** (the project tmux launcher; `PEGASUS_HIL_DEBUG=1`
   surfaces the actuator path in the Isaac pane):
```sh
PEGASUS_HIL_DEBUG=1 scripts/start_single_drone_sitl.sh <machine-config>
```

**3. The test node** (build once, then run):
```sh
cd ~/ros2_ws
# symlink or copy ros2/px4_direct_actuator_test into ~/ros2_ws/src, then:
colcon build --packages-select px4_direct_actuator_test
source install/setup.bash
ros2 run px4_direct_actuator_test direct_actuator_test --ros-args -p motor_value:=0.6
```

**4. (Manual variant)** Stream setpoints first, then switch in PX4 by hand —
   the most reliable bring-up:
```sh
ros2 run px4_direct_actuator_test direct_actuator_test --ros-args -p motor_value:=0.6 -p auto_arm:=false
```
then in the `pxh>` console:
```sh
commander mode offboard
commander arm
```

### Verifying

In the Isaac pane (`PEGASUS_HIL_DEBUG=1`):
```
[HIL_DBG vid=0] mode=129 armed=True flags=1 ... controls[:8]=[0.6, 0.6, 0.6, 0.6, ...] rotor_ref=[...]
```
`armed=True` + `controls[:4]` matching `motor_value` = the direct-actuator
command is reaching the motors unmodified. The props spin up and the vehicle
lifts.

> `direct_actuator` is fully **open-loop** — equal thrust with no stabilization,
> so the vehicle will climb and tumble. That is correct behaviour for raw
> actuator control; a stable hover requires your own controller computing
> per-motor values.

---

## Troubleshooting (issues encountered)

| Symptom | Cause | Fix |
|---------|-------|-----|
| `ERROR [simulator_mavlink] poll timeout 0, 111` on the PX4 side, Isaac freezes | HIL lockstep deadlock: Pegasus blocks waiting for an actuator message PX4 skipped while handling the mode change | `enable_lockstep: False` (change #1). The `111` is a stale `errno`; the `0` (poll timeout) is the real signal. |
| `Matching flight task was not able to run, Nav state: 2, Task: 1` | Vehicle armed in **POSCTL** (nav_state 2), whose position task can't run without a local-position estimate in HIL | Switch to OFFBOARD *before* arming (the node does this when `auto_arm:=true`). |
| `Switching to Offboard is currently not available` | `offboard_control_mode` seen as stale due to DDS timestamp re-stamping | `param set UXRCE_DDS_SYNCT 0` (change #4). |
| Disarms ~2 s after arming, reason "auto disarm preflight" | `COM_DISARM_LAND` / `COM_DISARM_PRFLT` auto-disarm because the vehicle never took off | Set both to `0` for ground testing, or command enough thrust to lift (Iris hover ≈ 0.5). |
| Nothing moves at `motor_value:=0.0` | Expected — zero thrust | Use a non-zero `motor_value`. |

---

## Node parameters

See [`ros2/px4_direct_actuator_test/README.md`](../ros2/px4_direct_actuator_test/README.md).
Key ones: `px4_ns` (default `uav_0`), `motor_value` (`[-1, 1]` per motor),
`auto_arm` (auto OFFBOARD+ARM vs. pure streamer), `num_motors`.
