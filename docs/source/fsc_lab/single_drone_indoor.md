# Indoor single-drone PX4 SITL

The indoor Iris and X650 launchers run standard PX4 flight control with an
OptiTrack/external-vision estimator profile and the Isaac Sim GUI:

```bash
cd "$HOME/Source/fsc_PegasusSimulator"
./scripts/indoor_sim/start_single_drone_iris.sh fsc_lab_machine
./scripts/indoor_sim/start_single_drone_x650.sh fsc_lab_machine
```

Run only one at a time because both use PX4 instance 0, TCP port 4560,
namespace `/uav_0`, and tmux session `px4_isaac`.

## Persistent parameter profiles

The Iris uses `build/px4_sitl_default/rootfs_fsc_indoor`; the X650 uses
`build/px4_sitl_default/rootfs_fsc_indoor_x650`. These are distinct from one
another and from the outdoor PX4 rootfs, so changing environments does not
require manually switching estimator or controller parameters.

Both indoor profiles set external vision as the height reference, fuse vision
position, velocity, height, and yaw, disable GPS and magnetometer aiding, and
permit arming without GPS. Override `EKF2_EV_DELAY` when necessary with:

```bash
PX4_INDOOR_EV_DELAY_MS=20 \
  ./scripts/indoor_sim/start_single_drone_x650.sh fsc_lab_machine
```

The X650 profile additionally applies its validated PX4 rate and attitude gains.
Its calibrated plant uses corrected `x650_new.usd` rotation and PX4 Quad-X motor
ordering with `LaggedQuadraticThrustCurve` and the measured motor response
`lambda=10.51 1/s`. A runtime override preserves the established 3.5 kg total
mass and CAD-derived diagonal inertia rather than using the new asset's authored
body mass.

## Ground truth

Isaac publishes ENU pose/quaternion on `/uav_0/state/pose`, ENU inertial linear
velocity on `/uav_0/state/twist_inertial`, and FLU body velocity/angular rate on
`/uav_0/state/twist`. Each launcher type-checks the required inertial topics.
The X650 result is stored in `/tmp/indoor_x650_groundtruth.log`.

Stop either simulation with:

```bash
tmux kill-session -t px4_isaac
```
