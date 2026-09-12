# PX4 SITL airframes (FSC VTOL)

Version-controlled copies of the custom PX4 airframes this project needs. PX4
reads airframes from *its own* tree (`$PX4_DIR/ROMFS/...`), so these files must be
installed into `$PX4_DIR` and built into the SITL ROMFS — the copy here is the
source of truth for review / re-installation on a new machine.

## `4100_fsc_dual_tiltrotor`

PX4 counterpart of the Isaac-side `vtol.dual_tiltrotor_config()` (2 motors +
2 independent tilt servos + aileron/elevator/rudder). Intended for
**`direct_actuator` offboard HIL** — see [`docs/direct_actuator_offboard.md`](../../docs/direct_actuator_offboard.md).

The output assignment (`PWM_MAIN_FUNC1..7`) is chosen so PX4's
`HIL_ACTUATOR_CONTROLS.controls[]` indices line up exactly with
`default_dual_tiltrotor_channel_map()` and the `ros2/px4_vtol_direct_actuator`
node's `MOTOR_ORDER` / `SERVO_ORDER`:

| controls[] | PX4 function | channel     |
|-----------:|--------------|-------------|
| 0          | Motor1 (101) | motor_left  |
| 1          | Motor2 (102) | motor_right |
| 2          | Servo1 (201) | tilt_left   |
| 3          | Servo2 (202) | tilt_right  |
| 4          | Servo3 (203) | aileron     |
| 5          | Servo4 (204) | elevator    |
| 6          | Servo5 (205) | rudder      |

> The allocator geometry (`CA_*`) is best-effort and **bypassed** in
> `direct_actuator` mode; a 2-rotor tiltrotor is under-actuated for PX4's
> standard hover allocation, so stabilized-mode flight would need extra work.

## Install into PX4 and build

```sh
PX4_DIR=~/PX4-Autopilot        # match scripts/config/<machine>.conf
AF=$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes

# 1. copy the airframe file
cp px4/airframes/4100_fsc_dual_tiltrotor "$AF/"

# 2. register it (add the line "4100_fsc_dual_tiltrotor" to the airframes list)
#    in $AF/CMakeLists.txt  (px4_add_romfs_files(...))

# 3. rebuild the SITL ROMFS
cd "$PX4_DIR" && make px4_sitl
```

Then launch with `scripts/start_vtol_sitl.sh <machine-config>` (defaults to
`PX4_SIM_MODEL=fsc_dual_tiltrotor`). Verify the actuator path with
`PEGASUS_HIL_DEBUG=1`.

> If you edit an already-built tree, `make px4_sitl` regenerates
> `build/px4_sitl_default/{etc,rootfs}/etc/init.d-posix/airframes/`. Delete any
> cached per-run workdir (`build/px4_sitl_default/rootfs_vtol0`) so the start
> script re-copies the refreshed rootfs.
