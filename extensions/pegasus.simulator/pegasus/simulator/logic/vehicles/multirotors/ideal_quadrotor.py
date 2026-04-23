"""
| File: ideal_quadrotor.py
| Description: Ideal quadrotor (single-link floating-base articulation, no external
|   USD) driven by thrust f [N] and moment M [N·m] from fsc_geometric_controller.
|
|   PhysX requires ArticulationRootAPI and RigidBodyAPI to be on the SAME prim for
|   a single-link floating articulation.  The collision box is a scaled Cube child.
|   Because the physics prim is at stage_prefix (not stage_prefix/body), update_state
|   is overridden and apply_force/torque are called with body_part="".
|
|   ROS 2 interface (via GeometricControllerROS2Backend):
|     Subscribes: ~control/output/f (Float64), ~control/output/M (Vector3Stamped)
|     Publishes:  ~state/pose, ~state/twist, ~state/twist_inertial,
|                 ~state/accel, ~state/jerk
"""

import numpy as np
from scipy.spatial.transform import Rotation
from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema

from pegasus.simulator.logic.vehicles.vehicle import Vehicle, get_world_transform_xform
from pegasus.simulator.logic.dynamics import LinearDrag
from pegasus.simulator.logic.backends.geometric_controller_ros2_backend import GeometricControllerROS2Backend


class IdealQuadrotorConfig:
    """Configuration for the ideal quadrotor.

    Control input is [f, M_x, M_y, M_z] from fsc_geometric_controller
    (collective thrust along body z, and body-frame moments).
    """

    def __init__(self):
        self.stage_prefix = "ideal_quadrotor"

        # Rigid-body properties
        self.mass    = 1.806                           # kg
        self.inertia = (0.016, 0.017, 0.024)  # (Ixx, Iyy, Izz)  kg·m²

        # Visual/collision box  (length × width × height) in metres
        self.box_size = (0.47, 0.47, 0.11)

        # Aerodynamic drag
        self.drag = LinearDrag([0.0, 0.0, 0.0]) # no drag

        # Set to True to disable PhysX gravity on this body (useful for attitude-only testing)
        self.disable_gravity: bool = False

        # Sensors / backends
        self.sensors           = []
        self.graphical_sensors = []
        self.graphs            = []
        self.backends          = [GeometricControllerROS2Backend(vehicle_id=0)]


class IdealQuadrotor(Vehicle):
    """Ideal quadrotor driven by [f, M] from fsc_geometric_controller.

    PhysX 5 recognises a single-body floating articulation only when
    ArticulationRootAPI and RigidBodyAPI are applied to the *same* prim.
    The root prim therefore IS the physics body; state and forces are read/written
    at self._stage_prefix (not self._stage_prefix + "/body").
    """

    def __init__(
        self,
        vehicle_id: int = 0,
        init_pos=(0.0, 0.0, 0.055),
        init_orientation=(0.0, 0.0, 0.0, 1.0),
        config: IdealQuadrotorConfig = None,
    ):
        if config is None:
            config = IdealQuadrotorConfig()

        # Must be stored before super().__init__ because _init_prim() runs inside it.
        self._mass           = config.mass
        self._inertia        = config.inertia
        self._box_size       = config.box_size
        self._disable_gravity = config.disable_gravity

        super().__init__(
            stage_prefix=config.stage_prefix,
            usd_path=None,
            init_pos=init_pos,
            init_orientation=init_orientation,
            sensors=config.sensors,
            graphical_sensors=config.graphical_sensors,
            graphs=config.graphs,
            backends=config.backends,
        )

        self._drag = config.drag

    # ------------------------------------------------------------------
    # USD prim construction (called from Vehicle.__init__ before Robot.__init__)
    # ------------------------------------------------------------------

    def _init_prim(self, root_prim):
        """Build a single-link floating-base articulation.

        ArticulationRootAPI + RigidBodyAPI on the SAME prim is required by PhysX 5
        for a single-link floating articulation.  The collision box is a scaled
        Cube child prim whose CollisionAPI is inherited by the parent rigid body.
        """
        Lx, Ly, Lz = self._box_size

        # Single-link floating-base articulation
        UsdPhysics.ArticulationRootAPI.Apply(root_prim)
        UsdPhysics.RigidBodyAPI.Apply(root_prim)

        mass_api = UsdPhysics.MassAPI.Apply(root_prim)
        mass_api.GetMassAttr().Set(float(self._mass))
        mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*self._inertia))

        if self._disable_gravity:
            PhysxSchema.PhysxRigidBodyAPI.Apply(root_prim).GetDisableGravityAttr().Set(True)

        # Collision + visual shape as a child cube
        col_path = self._stage_prefix + "/collision"
        col_cube = UsdGeom.Cube.Define(self._current_stage, col_path)
        col_cube.GetSizeAttr().Set(1.0)
        UsdGeom.Xformable(col_cube).AddScaleOp().Set(Gf.Vec3f(Lx, Ly, Lz))
        UsdPhysics.CollisionAPI.Apply(col_cube.GetPrim())

    # ------------------------------------------------------------------
    # State update — reads directly from stage_prefix (no /body suffix)
    # ------------------------------------------------------------------

    def update_state(self, dt: float):
        """Override: the physics prim is at stage_prefix, not stage_prefix/body."""

        body = self.get_dc_interface().get_rigid_body(self._stage_prefix)
        pose = self.get_dc_interface().get_rigid_body_pose(body)

        prim = self._world.stage.GetPrimAtPath(self._stage_prefix)
        rotation_quat     = get_world_transform_xform(prim).GetQuaternion()
        rotation_quat_real = rotation_quat.GetReal()
        rotation_quat_img  = rotation_quat.GetImaginary()

        ang_vel    = self.get_dc_interface().get_rigid_body_angular_velocity(body)
        linear_vel = self.get_dc_interface().get_rigid_body_linear_velocity(body)

        linear_acceleration = (np.array(linear_vel) - self._state.linear_velocity) / dt

        self._state.position = np.array(pose.p)
        self._state.attitude = np.array([
            rotation_quat_img[0], rotation_quat_img[1],
            rotation_quat_img[2], rotation_quat_real,
        ])
        self._state.linear_velocity      = np.array(linear_vel)
        self._state.linear_body_velocity = (
            Rotation.from_quat(self._state.attitude).inv().apply(self._state.linear_velocity)
        )
        self._state.angular_velocity = (
            Rotation.from_quat(self._state.attitude).inv().apply(np.array(ang_vel))
        )
        self._state.linear_acceleration = linear_acceleration

    # ------------------------------------------------------------------
    # Physics update — called every physics step
    # ------------------------------------------------------------------

    def update(self, dt: float):
        """Read [f, M_x, M_y, M_z] from fsc_geometric_controller and apply to body.

        f   – collective thrust [N] along body z (control/output/f)
        M   – body-frame moments [N·m]            (control/output/M)
        body_part="" resolves to self._stage_prefix (the physics prim itself).
        """
        if self._backends:
            cmd = self._backends[0].input_reference()
        else:
            cmd = [0.0, 0.0, 0.0, 0.0]

        f   = float(cmd[0]) if len(cmd) > 0 else 0.0
        M_x = float(cmd[1]) if len(cmd) > 1 else 0.0
        M_y = float(cmd[2]) if len(cmd) > 2 else 0.0
        M_z = float(cmd[3]) if len(cmd) > 3 else 0.0

        self.apply_force( [0.0, 0.0, f],      body_part="")
        self.apply_torque([M_x, M_y, M_z],    body_part="")

        drag = self._drag.update(self._state, dt)
        self.apply_force(drag.tolist(),            body_part="")

        for backend in self._backends:
            backend.update(dt)
