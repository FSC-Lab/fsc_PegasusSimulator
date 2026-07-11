"""
| File: x650_vehicle.py
| Author: Shiqi Gao (shiqi.gao907@gmail.com)
| License: BSD-3-Clause
| Description: FSC aerial-manipulator Vehicle. A thin *subclass* of the stock
|              pegasus ``Vehicle`` (extensions/pegasus.simulator/.../vehicles/vehicle.py)
|              that adds:
|                • usd_prim_path — reference a specific source prim inside the USD
|                  (instead of the whole default prim), and
|                • body_path     — a configurable rigid-body sub-path used when
|                  reading the vehicle state (defaults to "/body").
|              The stock base class is NOT modified: this class reuses the base
|              __init__ through its ``_init_prim`` hook and only overrides
|              ``_init_prim`` and ``update_state``. Every other method
|              (apply_force, apply_torque, sim_start_stop, sensor/backend
|              callbacks, get_dc_interface, ...) is inherited unchanged.
"""

import numpy as np
from scipy.spatial.transform import Rotation

from pxr import Sdf

from pegasus.simulator.logic.vehicles.vehicle import Vehicle, get_world_transform_xform


class VehicleMod(Vehicle):
    """Vehicle variant with configurable USD source prim and body sub-path."""

    def __init__(
        self,
        stage_prefix: str,
        usd_path: str = None,
        usd_prim_path: str = None,
        body_path: str = "/body",
        init_pos=(0.0, 0.0, 0.0),
        init_orientation=(0.0, 0.0, 0.0, 1.0),
        sensors=None,
        graphical_sensors=None,
        graphs=None,
        backends=None,
    ):
        """Initialize the vehicle.

        Args:
            stage_prefix (str): stage path the vehicle is spawned under.
            usd_path (str): USD file describing the vehicle.
            usd_prim_path (str, optional): if given, the USD is referenced from
                this specific source prim (e.g. "/gripper_bat") rather than the
                file's default prim.
            body_path (str): rigid-body sub-path used to read the vehicle state
                (relative to stage_prefix). Defaults to "/body".
            init_pos (list): initial position in the inertial (ENU) frame.
            init_orientation (list): initial orientation quaternion [qx, qy, qz, qw].
        """
        # Stash the extra configuration the stock Vehicle.__init__ does not know
        # about. This must happen BEFORE super().__init__, because the base
        # __init__ calls self._init_prim() (our override below) partway through.
        self._body_path = body_path
        self._usd_prim_path = usd_prim_path
        self._source_usd_file = usd_path

        # When a specific source prim is requested we add the USD reference
        # ourselves in _init_prim (targeting that prim). Pass usd_path=None to the
        # base in that case so it does not also add a plain, whole-file reference
        # (which would stack a second, unwanted reference on the prim).
        super().__init__(
            stage_prefix=stage_prefix,
            usd_path=(None if usd_prim_path else usd_path),
            init_pos=init_pos,
            init_orientation=init_orientation,
            sensors=sensors,
            graphical_sensors=graphical_sensors,
            graphs=graphs,
            backends=backends,
        )

        # Restore the true USD file for record-keeping (the base stored None above
        # whenever usd_prim_path was supplied).
        self._usd_file = usd_path

    def _init_prim(self, prim):
        """Base-class hook: runs after the root Xform is created and before
        Robot.__init__. If a specific source prim was requested, add the USD
        reference targeting that prim instead of the whole-file default."""
        if self._usd_prim_path and self._source_usd_file:
            prim.GetReferences().AddReference(
                self._source_usd_file, primPath=Sdf.Path(self._usd_prim_path)
            )

    def update_state(self, dt: float):
        """Read and update the vehicle state each physics step, using the
        configurable ``body_path`` and guarding against the referenced USD not
        yet being valid for the dynamic-control interface during the first frames.

        Args:
            dt (float): time elapsed since the previous call (s).
        """
        if dt <= 0.0:
            return

        # Body frame used to read position, orientation, velocities.
        body_rel_path = getattr(self, "_body_path", "/body")
        body_abs_path = self._stage_prefix + body_rel_path
        body = self.get_dc_interface().get_rigid_body(body_abs_path)

        # The referenced USD may not be fully valid for DC during the first frames.
        prim = self._world.stage.GetPrimAtPath(body_abs_path)
        if not prim or not prim.IsValid():
            return

        # Current position and orientation in the inertial frame
        try:
            pose = self.get_dc_interface().get_rigid_body_pose(body)
        except RuntimeError:
            return

        # Attitude according to the convention [w, x, y, z]
        try:
            rotation_quat = get_world_transform_xform(prim).GetQuaternion()
        except RuntimeError:
            return
        rotation_quat_real = rotation_quat.GetReal()
        rotation_quat_img = rotation_quat.GetImaginary()

        # Angular velocity (inertial frame) and linear velocity (inertial frame)
        ang_vel = self.get_dc_interface().get_rigid_body_angular_velocity(body)
        linear_vel = self.get_dc_interface().get_rigid_body_linear_velocity(body)

        # Linear acceleration by finite difference (Isaac does not expose it directly)
        linear_acceleration = (np.array(linear_vel) - self._state.linear_velocity) / dt

        # X = [x, y, z]
        self._state.position = np.array(pose.p)

        # Quaternion in the [qx, qy, qz, qw] standard
        self._state.attitude = np.array(
            [rotation_quat_img[0], rotation_quat_img[1], rotation_quat_img[2], rotation_quat_real]
        )

        # Inertial-frame velocity X_dot = [x_dot, y_dot, z_dot]
        self._state.linear_velocity = np.array(linear_vel)

        # Body-frame linear velocity V = [u, v, w] (x_dot = Rot * V)
        self._state.linear_body_velocity = (
            Rotation.from_quat(self._state.attitude).inv().apply(self._state.linear_velocity)
        )

        # omega = [p, q, r] expressed in the body frame
        self._state.angular_velocity = (
            Rotation.from_quat(self._state.attitude).inv().apply(np.array(ang_vel))
        )

        # Inertial-frame acceleration X_ddot = [x_ddot, y_ddot, z_ddot]
        self._state.linear_acceleration = linear_acceleration
