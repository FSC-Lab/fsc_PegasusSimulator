#!/usr/bin/env python
"""
07_px4_direct_t650_aerial_manipulator_interaction.py  (2026-09-27)

Author: Shiqi Gao (shiqi.gao907@gmail.com)

THE 06 PLANT WITH AN END-EFFECTOR FORCE INJECTOR -- for the physical-
interaction campaign of the whole-body 4-D L1 law
(docs/docs_aerial_manipulator/interaction_20260927/).

It does not copy 06: it loads 06 as a module (so 06 stays byte-identical and
every plant knob, the servo model, the spawn/re-seat and the ROS 2 arm bridge
are exactly the ones every other whole-body flight used) and subclasses its
sim class to add ONE thing: a world-frame force applied at the GRASP POINT of
the gripper, on the wrist-roll link, every physics step.

  command  /uav_0/isaacsim_manipulator/ee_force_cmd   geometry_msgs/Vector3Stamped
           world (ENU) frame, newtons. Latched for EE_FORCE_STALE_S of SIM time
           after the last message, then zero -- a dead driver never leaves a
           force on the vehicle. Stream it (the campaign driver does, ~50 Hz).
  truth    /uav_0/isaacsim_manipulator/ee_force_state std_msgs/Float64MultiArray
           [t_sim, Fx, Fy, Fz, r_ex, r_ey, r_ez] -- the force APPLIED this step
           and the grasp point it was applied at (world), every 2nd step.

The grasp point is the one 06's EE marker cube uses and the whole-body model's
EE is: the two finger pads' CoM midpoint laterally, 0.108 m out along the
wrist's -z (transition_planner.GRIPPER_OFF_WRIST). A force there is the
controller's "task force" F: the 4-D attribution should read it one-for-one.

This is how the campaign emulates the two interaction tasks without PhysX
contact: a PUSH/PULL on a box is a horizontal force, a grasped PAYLOAD is its
weight (-m g along z). Nothing here enters any control law.

Run with:
  scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_interaction_sitl.sh <config>
"""

import importlib.util
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location(
    "am06_plant", os.path.join(_HERE, "06_px4_t650_aerial_manipulator_free_flight.py"))
am06 = importlib.util.module_from_spec(_spec)
sys.modules["am06_plant"] = am06
_spec.loader.exec_module(am06)          # starts the SimulationApp, defines the plant

import numpy as np                      # noqa: E402  (after SimulationApp, see memory note)
import carb                             # noqa: E402
from pxr import UsdPhysics              # noqa: E402

EE_FORCE_CMD_TOPIC = "/uav_0/isaacsim_manipulator/ee_force_cmd"
EE_FORCE_STATE_TOPIC = "/uav_0/isaacsim_manipulator/ee_force_state"
EE_FORCE_STALE_S = 0.5                  # [s, sim] command latch
EE_FORCE_MAX_N = 20.0                   # [N] hard cap on the injected force
WRIST_LINK = am06.EE_MARKER_WRIST_LINK  # "/manip_base", manip_joint4's child
PAD_LINKS = am06.EE_MARKER_PAD_LINKS
GRASP_DEPTH = am06.EE_MARKER_GRASP_DEPTH


class AmT650Interaction(am06.AmT650WholeBodyArmSim):

    def _setup_arm_ros2_bridge(self):
        super()._setup_arm_ros2_bridge()
        from geometry_msgs.msg import Vector3Stamped
        from std_msgs.msg import Float64MultiArray
        self._Fma = Float64MultiArray
        self._F_cmd = np.zeros(3)
        self._F_cmd_t = None
        self._F_n = 0
        self._F_step = 0
        self._wrist_h = None
        self._grasp_off = None          # grasp point in the WRIST frame
        self._F_sub = self._arm_node.create_subscription(
            Vector3Stamped, EE_FORCE_CMD_TOPIC, self._on_ee_force, 10)
        self._F_pub = self._arm_node.create_publisher(Float64MultiArray, EE_FORCE_STATE_TOPIC, 10)
        print(f"[AM-T650-INT] EE FORCE INJECTOR: {EE_FORCE_CMD_TOPIC} (world N, latched "
              f"{EE_FORCE_STALE_S}s sim, cap {EE_FORCE_MAX_N} N) -> applied at the grasp point; "
              f"truth on {EE_FORCE_STATE_TOPIC}", flush=True)

    def _on_ee_force(self, msg):
        f = np.array([msg.vector.x, msg.vector.y, msg.vector.z], float)
        n = float(np.linalg.norm(f))
        if not np.isfinite(n):
            return
        if n > EE_FORCE_MAX_N:
            f *= EE_FORCE_MAX_N / n
        self._F_cmd = f
        self._F_cmd_t = self._t
        self._F_n += 1

    def _grasp_point(self):
        """World position of the grasp point (lazy one-time offset, the marker
        cube's construction: pad CoM midpoint laterally, -GRASP_DEPTH along z)."""
        dc = self._dc
        if self._wrist_h is None:
            self._wrist_h = dc.get_rigid_body(self.drone_path + WRIST_LINK)
            if not self._wrist_h:
                self._wrist_h = None
                return None

        def pose(h):
            P = dc.get_rigid_body_pose(h)
            return (np.array([P.p.x, P.p.y, P.p.z]),
                    am06.C.quat_to_rot(P.r.w, P.r.x, P.r.y, P.r.z))

        p_w, R_w = pose(self._wrist_h)
        if self._grasp_off is None:
            pads = [dc.get_rigid_body(self.drone_path + p) for p in PAD_LINKS]
            if not all(pads):
                return None
            mids = []
            for path, h in zip(PAD_LINKS, pads):
                p, R = pose(h)
                com = UsdPhysics.MassAPI(self.stage.GetPrimAtPath(self.drone_path + path)) \
                    .GetCenterOfMassAttr().Get()
                mids.append(R_w.T @ ((p + R @ np.array([com[0], com[1], com[2]], float)) - p_w))
            mid = 0.5 * (mids[0] + mids[1])
            self._grasp_off = np.array([mid[0], mid[1], -GRASP_DEPTH])
            print(f"[AM-T650-INT] grasp point in the wrist frame: {self._grasp_off.round(4)} m", flush=True)
        return p_w + R_w @ self._grasp_off

    def _control_step_inner(self, dt):
        super()._control_step_inner(dt)
        live = self._F_cmd_t is not None and (self._t - self._F_cmd_t) <= EE_FORCE_STALE_S
        f = self._F_cmd if live else np.zeros(3)
        r_e = self._grasp_point()
        if r_e is None:
            return
        if np.any(f):
            self._dc.apply_body_force(self._wrist_h, carb._carb.Float3(*map(float, f)),
                                      carb._carb.Float3(*map(float, r_e)), True)
        self._F_step += 1
        if self._F_step % 2 == 0:
            m = self._Fma()
            m.data = [float(self._t), *map(float, f), *map(float, r_e)]
            self._F_pub.publish(m)


def main():
    AmT650Interaction().run()


if __name__ == "__main__":
    main()
