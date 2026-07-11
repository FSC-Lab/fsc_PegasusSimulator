"""
| File: x650_multirotor.py
| Author: Shiqi Gao (shiqi.gao907@gmail.com)
| License: BSD-3-Clause
| Description: FSC aerial-manipulator Multirotor. A *subclass* of the stock pegasus
|              ``Multirotor`` (for the thrust model / is-a relationship) and of the
|              FSC ``VehicleMod`` (for the configurable USD source prim + body path).
|              It overrides only the methods that must know the vehicle's configurable
|              body/rotor prim paths and rotor joint names:
|                • __init__                       (extra body/rotor/usd_prim kwargs)
|                • update                         (per-rotor force + drag on those paths)
|                • handle_propeller_visual        (rotor joint names)
|                • force_and_torques_to_velocities(allocation on those paths)
|              Everything else (start/stop, apply_force/torque, sensor & backend
|              callbacks, get_dc_interface, ...) is inherited. The stock base
|              classes are NOT modified.
"""

import numpy as np
import carb

from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor as BaseMultirotor, MultirotorConfig

from fsc_aerial_manipulation.robotic_arm.x650_vehicle import VehicleMod

# Re-export so callers can do `from ...multirotor_mod import MultirotorMod, MultirotorConfig`.
__all__ = ["MultirotorMod", "MultirotorConfig"]


class MultirotorMod(BaseMultirotor, VehicleMod):
    """Multirotor whose body/rotor rigid-body prim paths and rotor joint names are
    configurable (needed for custom USD assets such as the aerial manipulator).

    MRO: MultirotorMod -> Multirotor (base) -> VehicleMod -> Vehicle (base) -> Robot.
    ``update_state`` therefore resolves to VehicleMod's body-path-aware version,
    while ``start``/``stop`` come from the stock Multirotor.
    """

    def __init__(
        self,
        stage_prefix: str = "quadrotor",
        usd_file: str = "",
        vehicle_id: int = 0,
        init_pos=(0.0, 0.0, 0.07),
        init_orientation=(0.0, 0.0, 0.0, 1.0),
        body_path: str = "/body",
        rotor_paths=None,
        rotor_joint_names=None,
        config=None,
        usd_prim_path: str = None,
    ):
        """Initialize the multirotor.

        Args:
            stage_prefix (str): stage path the vehicle is spawned under.
            usd_file (str): USD file describing the vehicle.
            vehicle_id (int): vehicle id (kept for API parity; unused here).
            init_pos (list): initial position in the inertial (ENU) frame.
            init_orientation (list): initial orientation quaternion [qx, qy, qz, qw].
            body_path (str): body rigid-body sub-path (default "/body").
            rotor_paths (list, optional): rotor rigid-body sub-paths. Defaults to
                ["/rotor0", ... ] sized to the thrust curve's rotor count.
            rotor_joint_names (list, optional): rotor joint names for the spin
                animation. Defaults to ["joint0", ... ].
            config (MultirotorConfig, optional): defaults to a fresh MultirotorConfig().
            usd_prim_path (str, optional): specific source prim to reference in the USD.
        """
        # Resolve config here (not as a mutable default argument) so each vehicle
        # gets its own fresh instance and its own fresh default backend.
        if config is None:
            config = MultirotorConfig()
        if config.backends is None:
            config.backends = [PX4MavlinkBackend(config=PX4MavlinkBackendConfig())]

        # Initialize through VehicleMod directly. The stock Multirotor.__init__
        # targets the stock Vehicle signature (no usd_prim_path/body_path), so we
        # bypass it and drive the VehicleMod -> Vehicle -> Robot chain ourselves.
        VehicleMod.__init__(
            self,
            stage_prefix=stage_prefix,
            usd_path=usd_file,
            usd_prim_path=usd_prim_path,
            body_path=body_path,
            init_pos=init_pos,
            init_orientation=init_orientation,
            sensors=config.sensors,
            graphical_sensors=config.graphical_sensors,
            graphs=config.graphs,
            backends=config.backends,
        )

        # Dynamics: thrust curve + drag, and the configurable rotor prim paths.
        self._thrusters = config.thrust_curve
        self._drag = config.drag
        self._rotor_paths = (
            list(rotor_paths) if rotor_paths is not None
            else [f"/rotor{i}" for i in range(self._thrusters._num_rotors)]
        )
        self._rotor_joint_names = (
            list(rotor_joint_names) if rotor_joint_names is not None
            else [f"joint{i}" for i in range(self._thrusters._num_rotors)]
        )

    def update(self, dt: float):
        """Compute and apply rotor forces, rolling moment and drag each physics step,
        using the configurable body/rotor prim paths.

        Args:
            dt (float): time elapsed since the previous call (s).
        """
        # Some USD assets expose the articulation on the body prim rather than the
        # container root; fall back to the root if the body prim has none.
        articulation = self.get_dc_interface().get_articulation(f"{self._stage_prefix}{self._body_path}")
        if not articulation:
            articulation = self.get_dc_interface().get_articulation(self._stage_prefix)

        if not hasattr(self, "_articulation_logged"):
            if articulation is None or not articulation:
                carb.log_error(
                    f"[MULTIROTOR] CRITICAL: Could not find valid articulation at "
                    f"'{self._stage_prefix}{self._body_path}' or '{self._stage_prefix}'"
                )
            else:
                carb.log_info("[MULTIROTOR] Articulation found and valid")
            self._articulation_logged = True

        # Desired rotor angular velocities [rad/s] from the first backend.
        if len(self._backends) != 0:
            desired_rotor_velocities = self._backends[0].input_reference()
        else:
            desired_rotor_velocities = [0.0 for _ in range(self._thrusters._num_rotors)]

        if not hasattr(self, "_backend_log_count"):
            self._backend_log_count = 0
        if self._backend_log_count < 5:
            print(f"[BACKEND] input_reference() returned: {desired_rotor_velocities}", flush=True)
            self._backend_log_count += 1

        # Feed the thruster model and get per-rotor forces + rolling moment.
        self._thrusters.set_input_reference(desired_rotor_velocities)
        forces_z, _, rolling_moment = self._thrusters.update(self._state, dt)

        if not hasattr(self, "_force_log_count"):
            self._force_log_count = 0
        if self._force_log_count < 5:
            print(f"[THRUST] Forces: {[f'{f:.2f}N' for f in forces_z]} Moment: {rolling_moment:.4f}", flush=True)
            self._force_log_count += 1

        if not hasattr(self, "_logged_rotor_paths"):
            print(f"[MULTIROTOR] Stage prefix: {self._stage_prefix}", flush=True)
            print(f"[MULTIROTOR] Using rotor paths: {self._rotor_paths}", flush=True)
            print(f"[MULTIROTOR] Using body path: {self._body_path}", flush=True)
            print(f"[MULTIROTOR] Using rotor joint names: {self._rotor_joint_names}", flush=True)
            for i, rotor_path in enumerate(self._rotor_paths):
                full_path = f"{self._stage_prefix}{rotor_path}"
                joint_name = self._rotor_joint_names[i]
                print(f"[MULTIROTOR] Rotor {i}: path={rotor_path} -> Full: {full_path}, joint={joint_name}", flush=True)
            self._logged_rotor_paths = True

        # Apply the Z force on each rotor frame + spin the propeller visual.
        for i in range(self._thrusters._num_rotors):
            self.apply_force([0.0, 0.0, forces_z[i]], body_part=self._rotor_paths[i])
            self.handle_propeller_visual(i, forces_z[i], articulation)

        # Rolling moment about the body Z axis.
        self.apply_torque([0.0, 0.0, rolling_moment], self._body_path)

        # Linear drag on the body.
        drag = self._drag.update(self._state, dt)
        self.apply_force(drag, body_part=self._body_path)

        # Update all backends.
        for backend in self._backends:
            backend.update(dt)

    def handle_propeller_visual(self, rotor_number, force: float, articulation):
        """Spin the rotor joint for the visual effect, keyed by the configurable
        rotor joint name.

        Args:
            rotor_number (int): index of the rotor to animate.
            force (float): force currently applied on that rotor.
            articulation: the articulation the rotor joints belong to.
        """
        if articulation is None or not articulation:
            carb.log_warn(
                f"[MULTIROTOR] Articulation is invalid for propeller visual animation on rotor {rotor_number}"
            )
            return

        joint_name = self._rotor_joint_names[rotor_number]
        joint = self.get_dc_interface().find_articulation_dof(articulation, joint_name)

        if not joint or joint == -1:
            carb.log_warn(
                f"[MULTIROTOR] Invalid DOF handle for rotor {rotor_number} with joint name '{joint_name}'. "
                f"Available joint names should be verified in the USD asset."
            )
            return

        if 0.0 < force < 0.1:
            self.get_dc_interface().set_dof_velocity(joint, 5 * self._thrusters.rot_dir[rotor_number])
        elif 0.1 <= force:
            self.get_dc_interface().set_dof_velocity(joint, 100 * self._thrusters.rot_dir[rotor_number])
        else:
            self.get_dc_interface().set_dof_velocity(joint, 0)

    def force_and_torques_to_velocities(self, force: float, torque: np.ndarray):
        """Target rotor angular velocities for a desired body thrust + torque,
        using the configurable body/rotor prim paths for the allocation geometry.

        Args:
            force (np.ndarray): desired body-frame force [N].
            torque (np.ndarray): desired body-frame torque [Nm].

        Returns:
            list: per-rotor angular velocities [rad/s].
        """
        # Body frame + rotor frames (on the configurable prim paths).
        rb = self.get_dc_interface().get_rigid_body(self._stage_prefix + self._body_path)
        rotors = [
            self.get_dc_interface().get_rigid_body(self._stage_prefix + self._rotor_paths[i])
            for i in range(self._thrusters._num_rotors)
        ]

        # Rotor positions relative to the body frame (orientation ignored for now).
        relative_poses = self.get_dc_interface().get_relative_body_poses(rb, rotors)

        # Allocation matrix: [T, tau_x, tau_y, tau_z] = A @ omega^2
        aloc_matrix = np.zeros((4, self._thrusters._num_rotors))
        aloc_matrix[0, :] = np.array(self._thrusters._rotor_constant)
        aloc_matrix[1, :] = np.array(
            [relative_poses[i].p[1] * self._thrusters._rotor_constant[i] for i in range(self._thrusters._num_rotors)]
        )
        aloc_matrix[2, :] = np.array(
            [-relative_poses[i].p[0] * self._thrusters._rotor_constant[i] for i in range(self._thrusters._num_rotors)]
        )
        aloc_matrix[3, :] = np.array(
            [self._thrusters._rolling_moment_coefficient[i] * self._thrusters._rot_dir[i]
             for i in range(self._thrusters._num_rotors)]
        )

        aloc_inv = np.linalg.pinv(aloc_matrix)
        squared_ang_vel = aloc_inv @ np.array([force, torque[0], torque[1], torque[2]])

        # No negative squared angular velocities.
        squared_ang_vel[squared_ang_vel < 0] = 0.0

        # Saturate while preserving the relation between rotors (normalization).
        max_thrust_vel_squared = np.power(self._thrusters.max_rotor_velocity[0], 2)
        max_val = np.max(squared_ang_vel)
        if max_val >= max_thrust_vel_squared:
            normalize = np.maximum(max_val / max_thrust_vel_squared, 1.0)
            squared_ang_vel = squared_ang_vel / normalize

        return np.sqrt(squared_ang_vel)
