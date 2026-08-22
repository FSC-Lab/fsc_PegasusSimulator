#!/usr/bin/env python
"""
| File: 06_px4_single_drone_t650_gate_splat.py
| Description: Single T650 quadrotor SITL inside the RECONSTRUCTED DRONE-GATE SPLAT
|   scene (3D Gaussian splat of the A2RL x DCL practice gate, captured by phone and
|   reconstructed with spirulae-splat, rendered natively by Isaac Sim's NuRec).
|
|   Same vehicle, calibration and PX4 wiring as 05_px4_single_drone_t650.py. What
|   differs:
|
|     1. ENVIRONMENT: loads gate_metric.usda instead of the stock Curved Gridroom.
|        The scene is metric (scale fixed from the gate's measured 1.50 m inner
|        opening width), gravity-aligned, floor at z=0, gate opening centred at
|        (x=0, y=0, z=1.75). The gate plane is the YZ plane: flying "through the
|        gate" is motion along world X. Cameras/capture side is +X.
|     2. PHYSICS: the splat is VISION-ONLY - it has no collision geometry. An
|        invisible PhysX ground plane at z=0 gives the drone something to sit on.
|        Nothing else collides (the drone can fly through the gate frame; add a
|        mesh collision proxy later if crashes should be physical).
|     3. CAMERA: a forward-looking MonocularCamera is attached to the drone body
|        and shown in its own "Drone Camera" viewport next to the main view.
|     4. MAIN VIEW: a CHASE CAMERA rides behind and above the drone, looking
|        forward past it toward the gate, so the main viewport follows the
|        flight instead of staying parked at the world origin. It tracks the
|        body's POSITION and YAW only -- roll and pitch are deliberately not
|        inherited, so the horizon stays level while the drone manoeuvres.
|
|   The scene asset resolves from the repo itself, via
|   fsc_aerial_manipulation.worlds.gate_splat. Like every other FSC USD it is not in
|   git; see that package's README.md for how to obtain it.
|
|   Environment overrides:
|     GATE_SPLAT_USD    path to the metric splat USD (default: the in-repo asset)
|     PEGASUS_CAM_POS   camera position in body FLU, "x,y,z" metres (default 0.25,0,0.10)
|     PEGASUS_CAM_PITCH camera down-pitch in degrees (default 0 = level)
|     PEGASUS_CAM_ROS2  "1" publishes camera on the ROS 2 backend (default off)
|     PEGASUS_CAM_HFOV  onboard camera HORIZONTAL field of view, deg (default 120);
|                       intrinsics and diagonal FOV are derived from it
|     PEGASUS_CHASE_FOV main viewport horizontal field of view, deg (default 90;
|                       Isaac's own default perspective camera is only 60)
|     PEGASUS_FLOOR_DROP how far the TAKEOFF SURFACE sits below the splat's own
|                       floor, m (default 0.324) -- see "Floor drop" below
|     PEGASUS_RECORD    "1" writes PNG frames for a video (default off)
|     PEGASUS_RECORD_DIR      output directory (default /tmp/gate_capture)
|     PEGASUS_RECORD_FPS      frames per SIMULATED second (default 30)
|     PEGASUS_RECORD_VIEW     "chase" (main view) or "fpv" (onboard camera)
|     PEGASUS_RECORD_MIN_Z    only capture above this altitude, m (default 1.0),
|                       so the parked/climbing frames stay out of the clip
|     PEGASUS_CHASE       "0" disables the chase camera, leaving the main viewport
|                         free for manual navigation (default on)
|     PEGASUS_CHASE_BACK  chase distance behind the drone, m (default 2.5)
|     PEGASUS_CHASE_UP    chase height above the drone, m (default 0.8)
|     PEGASUS_CHASE_AHEAD how far ahead of the drone the view aims, m (default 1.0);
|                         larger shows more of the gate, smaller centres the drone
|
|   plus the usual PEGASUS_HEADLESS / PEGASUS_STEPS / PEGASUS_PX4_LOCKSTEP.
"""

# Imports to start Isaac Sim from this script
import os

import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
HEADLESS = os.environ.get("PEGASUS_HEADLESS", "0") == "1"
STEP_LIMIT = int(os.environ.get("PEGASUS_STEPS", "0"))
PX4_LOCKSTEP = os.environ.get("PEGASUS_PX4_LOCKSTEP", "1") == "1"
CAM_POS = [float(x) for x in os.environ.get("PEGASUS_CAM_POS", "0.25,0.0,0.10").split(",")]
CAM_PITCH = float(os.environ.get("PEGASUS_CAM_PITCH", "0.0"))
CAM_ROS2 = os.environ.get("PEGASUS_CAM_ROS2", "0") == "1"
CHASE_CAM = os.environ.get("PEGASUS_CHASE", "1") == "1"
CHASE_BACK = float(os.environ.get("PEGASUS_CHASE_BACK", "2.5"))
CHASE_UP = float(os.environ.get("PEGASUS_CHASE_UP", "0.8"))
CHASE_AHEAD = float(os.environ.get("PEGASUS_CHASE_AHEAD", "1.0"))
CAM_HFOV = float(os.environ.get("PEGASUS_CAM_HFOV", "120.0"))
CHASE_FOV = float(os.environ.get("PEGASUS_CHASE_FOV", "90.0"))
FLOOR_DROP = float(os.environ.get("PEGASUS_FLOOR_DROP", "0.576"))
RECORD = os.environ.get("PEGASUS_RECORD", "0") == "1"
RECORD_DIR = os.environ.get("PEGASUS_RECORD_DIR", "/tmp/gate_capture")
RECORD_FPS = float(os.environ.get("PEGASUS_RECORD_FPS", "30.0"))
RECORD_VIEW = os.environ.get("PEGASUS_RECORD_VIEW", "chase")   # chase | fpv
RECORD_MIN_Z = float(os.environ.get("PEGASUS_RECORD_MIN_Z", "1.0"))
simulation_app = SimulationApp({"headless": HEADLESS})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import numpy as np
import omni.timeline
from scipy.spatial.transform import Rotation
from omni.isaac.core.world import World
from omni.isaac.core.objects import GroundPlane
import omni.usd

# Import the Pegasus API for simulating drones
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera

# The T650's own spawn helper and constants -- independent of the X650 modules.
from fsc_aerial_manipulation.rotorcraft.t650_bare_frame_utils import spawn_t650_with_mavlink
from fsc_aerial_manipulation.rotorcraft import t650_params
from fsc_aerial_manipulation.utils import add_dome_lighting

# The reconstructed gate scene: in-repo asset path + its metric geometry.
from fsc_aerial_manipulation.worlds import gate_splat


class FscGateSplatSim:
    """T650 SITL inside the metric gate-splat scene, with an onboard camera view."""

    def __init__(self):

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Resolves env override -> in-repo asset, and raises with download/regenerate
        # instructions if the (ungitted) payload is absent.
        scene_usd = gate_splat.gate_splat_usd_path()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # ---------- The reconstructed gate scene ----------
        # NuRec splat: photoreal visuals, ZERO collision. Floor at z=0, gate plane
        # = YZ plane, opening centred at OPENING_CENTRE.
        # NOTE load_environment is asyncio.ensure_future -- the /World/layout prim
        # does NOT exist until the app steps, so nothing may query it before reset().
        self.pg.load_environment(scene_usd)
        print(
            f"[gate] scene {scene_usd}\n"
            f"[gate] opening centre {np.round(gate_splat.OPENING_CENTRE, 3).tolist()} m, "
            f"{gate_splat.OPENING_WIDTH:.3f} x {gate_splat.OPENING_HEIGHT:.3f} m, "
            f"sill {gate_splat.SILL_HEIGHT:.3f} m | fly-through axis = X",
            flush=True,
        )

        # The splat bakes the hangar's real lighting into its radiance; the dome
        # light is for the DRONE asset, which is a mesh and needs actual lights.
        stage = omni.usd.get_context().get_stage()
        add_dome_lighting(
            stage=stage,
            dome_path="/World/DomeLight",
            intensity=2500.0,
            exposure=0.0,
            color=(1.0, 1.0, 1.0),
        )

        # Invisible physics floor: the splat cannot collide, so give PhysX the
        # tarp plane the scene was aligned to (z=0) -- less FLOOR_DROP.
        #
        # Floor drop. QGroundControl's guided modes will not command below 2 m
        # above home, but the gate's opening centre is only 1.746 m above the
        # reconstructed floor, so a guided hover sits ~0.32 m over the opening.
        # Dropping the takeoff surface (and the spawn with it) puts a 2 m guided
        # hover exactly at the opening centre.
        #     drop = rest height + hover altitude - opening centre
        #          = 0.312       + 2.000          - 1.746          = 0.566 m
        # MEASURE THE REST HEIGHT, DO NOT ASSUME IT IS THE SPAWN z. PX4's home
        # altitude is where the airframe SETTLES, i.e. its landing-gear height
        # above the ground plane (0.312 m here), not the 0.07 m spawn placement.
        # Using the spawn value gave 0.324 and left the hover 25 cm high --
        # measured in flight as world z 1.998 against a 1.746 opening centre.
        # The shipped 0.576 is that flight-corrected figure (0.324 + 0.252).
        # The splat is ONE rigid volume holding both the gate and the floor, so
        # it cannot be raised without raising its own floor too -- and raising
        # everything uniformly changes nothing, because PX4 altitude is relative
        # to home, which rides along. Compensating on the physics side instead
        # keeps gate_metric.usda and gate_splat.py describing the real
        # reconstructed scene; the fudge lives here, where it is visible.
        # COST: parked on the ground the vehicle appears sunk FLOOR_DROP into
        # the visible tarp. In flight nothing looks wrong (the drone is 1.746 m
        # over the visible floor, level with the gate). Accepted deliberately --
        # this rig is for filming the hover, not the takeoff. Set
        # PEGASUS_FLOOR_DROP=0 to restore the honest geometry.
        GroundPlane(prim_path="/World/groundPlane", size=25.0,
                    z_position=-FLOOR_DROP, visible=False)

        # ---------- Onboard camera ----------
        # Body-frame FLU position from PEGASUS_CAM_POS. Orientation: the Pegasus
        # example default [0, 0, 180] yields a level forward-facing upright image
        # (examples/8_camera_vehicle.py); down-pitch composes as [0, -pitch, 180].
        #
        # Intrinsics are DERIVED from PEGASUS_CAM_HFOV so the three numbers that
        # have to agree cannot drift apart: a pinhole with horizontal field of
        # view h has fx = (W/2)/tan(h/2), and the diagonal FOV that Isaac's lens
        # model wants as `max_fov` follows from the same focal length. The stock
        # 90 deg (fx = W/2 = 640, diagonal 98) was too narrow to see the gate on
        # approach; 120 deg is the usual FPV-camera figure.
        cam_w, cam_h = 1280, 720
        cam_fx = (cam_w / 2.0) / np.tan(np.radians(CAM_HFOV) / 2.0)
        cam_dfov = np.degrees(2.0 * np.arctan(np.hypot(cam_w, cam_h) / (2.0 * cam_fx)))
        camera = MonocularCamera(
            "camera",
            config={
                "position": np.array(CAM_POS),
                "orientation": np.array([0.0, -CAM_PITCH, 180.0]),
                "resolution": (cam_w, cam_h),
                "frequency": 30,
                # principal point centred; fx = fy (square pixels)
                "intrinsics": np.array([[cam_fx, 0.0, cam_w / 2.0],
                                        [0.0, cam_fx, cam_h / 2.0],
                                        [0.0, 0.0, 1.0]]),
                "distortion_coefficients": [0.0] * 8,
                "diagonal_fov": float(cam_dfov),
            },
        )
        print(
            f"[camera] onboard {CAM_HFOV:.0f} deg horizontal / {cam_dfov:.1f} deg diagonal "
            f"(fx = {cam_fx:.1f} px at {cam_w}x{cam_h})",
            flush=True,
        )

        # ---------- Vehicle ----------
        # Spawn on the capture (+X) side, yawed 180 so body-forward (and the camera)
        # faces the gate. Both come from the scene module, not hardcoded here;
        # only the height is shifted, by the same FLOOR_DROP as the ground plane.
        spawn_pos = (
            gate_splat.DEFAULT_SPAWN_POS[0],
            gate_splat.DEFAULT_SPAWN_POS[1],
            gate_splat.DEFAULT_SPAWN_POS[2] - FLOOR_DROP,
        )
        self.drone_path = spawn_t650_with_mavlink(
            px4_path=self.pg.px4_path,
            px4_default_airframe=self.pg.px4_default_airframe,
            vehicle_id=0,
            spawn_pos=spawn_pos,
            spawn_euler=gate_splat.DEFAULT_SPAWN_EULER,
            enable_lockstep=PX4_LOCKSTEP,
            graphical_sensors=[camera],
            pub_graphical_sensors=CAM_ROS2,
        )
        print(
            f"[T650] total mass {t650_params.MASS:.6f} kg | "
            f"expected hover command {t650_params.HOVER_COMMAND:.4f} | "
            f"spawn {np.round(spawn_pos, 3).tolist()} (floor drop {FLOOR_DROP:.3f} m) "
            f"facing -X toward the gate | "
            f"camera at body FLU {CAM_POS}, pitch {CAM_PITCH} deg down",
            flush=True,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # ---------- Chase camera (main viewport) ----------
        # Grab the MAIN viewport's api HERE, before the "Drone Camera" window
        # below exists: set_camera_view() falls back to get_active_viewport(),
        # and once a second window is open that is no longer guaranteed to be
        # the main one. The perspective camera prim itself is always
        # /OmniverseKit_Persp.
        self._chase_vp = None
        self._chase_fwd = None
        self._chase_warned = False
        if not HEADLESS and CHASE_CAM:
            try:
                from omni.kit.viewport.utility import get_active_viewport

                self._chase_vp = get_active_viewport()
            except Exception as exc:
                carb.log_warn(f"Chase camera disabled (no viewport): {exc}")

            # Isaac's stock perspective camera is 60 deg horizontal (focal 18.147
            # over a 20.955 aperture), which is tight for following a drone into
            # a gate. Widen by shortening the focal length; the aperture is left
            # alone so the framing stays centred. set_camera_view() only writes
            # the transform, so this survives every chase update.
            if CHASE_FOV > 0.0:
                try:
                    from pxr import UsdGeom

                    persp = UsdGeom.Camera(
                        omni.usd.get_context().get_stage().GetPrimAtPath("/OmniverseKit_Persp")
                    )
                    aperture = persp.GetHorizontalApertureAttr().Get()
                    persp.GetFocalLengthAttr().Set(
                        aperture / (2.0 * np.tan(np.radians(CHASE_FOV) / 2.0))
                    )
                    print(f"[chase] main viewport widened to {CHASE_FOV:.0f} deg horizontal",
                          flush=True)
                except Exception as exc:
                    carb.log_warn(f"Could not set main viewport FOV: {exc}")

        # ---------- Drone-camera viewport ----------
        # A second viewport window showing what the onboard camera sees.
        if not HEADLESS:
            try:
                from omni.kit.viewport.utility import create_viewport_window

                cam_prim = self.drone_path + "/body/camera"
                vp = create_viewport_window("Drone Camera", width=640, height=360)
                vp.viewport_api.set_active_camera(cam_prim)
                self._fpv_vp = vp.viewport_api
                print(f"[camera] 'Drone Camera' viewport bound to {cam_prim}", flush=True)
            except Exception as exc:  # viewport API differences should not kill the sim
                carb.log_warn(f"Could not create drone-camera viewport: {exc}")

        # The spawn helper returns the stage prefix; the vehicle object (and its
        # live state) lives in the manager under that key.
        self.vehicle = self.pg.vehicle_manager.vehicles.get(self.drone_path)
        if self._chase_vp is not None and self.vehicle is None:
            carb.log_warn(f"Chase camera disabled: no vehicle at {self.drone_path}")
            self._chase_vp = None

        # Place the chase view before the first frame so the sim opens looking
        # at the drone rather than at the world origin.
        self._update_chase_camera()
        if self._chase_vp is not None:
            print(
                f"[chase] main viewport follows the drone: {CHASE_BACK:.2f} m back, "
                f"{CHASE_UP:.2f} m up, aiming {CHASE_AHEAD:.2f} m ahead "
                f"(PEGASUS_CHASE=0 to free the view)",
                flush=True,
            )

        # ---------- Frame recorder ----------
        # Captures are driven by SIMULATED time, not wall time: the splat renders
        # at roughly a third of real time (measured RTF 0.35), so a screen
        # recorder would produce slow motion. One frame per 1/RECORD_FPS of sim
        # time, encoded at RECORD_FPS, plays back at true speed whatever the
        # simulator manages.
        self._rec_n = 0
        self._rec_next_t = 0.0
        self._rec_vp = None
        self._rec_warned = False
        if RECORD and not HEADLESS:
            os.makedirs(RECORD_DIR, exist_ok=True)
            if RECORD_VIEW == "fpv":
                self._rec_vp = getattr(self, "_fpv_vp", None)
                if self._rec_vp is None:
                    carb.log_warn("PEGASUS_RECORD_VIEW=fpv but the onboard viewport "
                                  "is unavailable; recording the main view instead.")
            if self._rec_vp is None:
                self._rec_vp = self._chase_vp
            if self._rec_vp is None:
                carb.log_warn("Recording disabled: no viewport to capture.")
            else:
                print(f"[record] {RECORD_VIEW} view -> {RECORD_DIR}/frame_%05d.png "
                      f"at {RECORD_FPS:.0f} fps of SIM time, above z = {RECORD_MIN_Z} m",
                      flush=True)

        self.stop_sim = False

    def _record_frame(self):
        """Write one PNG per 1/RECORD_FPS of SIMULATED time.

        Skips while the vehicle is below RECORD_MIN_Z so the parked and climbing
        frames -- where the floor drop leaves the airframe sunk in the tarp --
        never reach the clip.
        """
        if self._rec_vp is None:
            return
        try:
            now = self.world.current_time
            if now < self._rec_next_t:
                return
            if self.vehicle is not None and self.vehicle.state is not None:
                if float(self.vehicle.state.position[2]) < RECORD_MIN_Z:
                    return
            from omni.kit.viewport.utility import capture_viewport_to_file

            capture_viewport_to_file(
                self._rec_vp, f"{RECORD_DIR}/frame_{self._rec_n:05d}.png")
            self._rec_n += 1
            # Advance from the deadline, not from `now`, so a slow frame does not
            # let the capture clock drift behind simulated time.
            self._rec_next_t = max(now, self._rec_next_t) + 1.0 / RECORD_FPS
        except Exception as exc:
            if not self._rec_warned:
                self._rec_warned = True
                carb.log_warn(f"Frame capture failed, disabling it: {exc}")
            self._rec_vp = None

    def _update_chase_camera(self):
        """Park the main viewport behind the drone, looking forward past it.

        Follows POSITION and YAW only. Taking the body's full attitude would
        roll and pitch the horizon with every manoeuvre, which is exactly what
        the onboard "Drone Camera" viewport is already for.
        """
        if self._chase_vp is None:
            return

        try:
            from isaacsim.core.utils.viewports import set_camera_view

            state = self.vehicle.state
            if state is None:
                return
            position = np.asarray(state.position, dtype=float)

            # Body forward (+X FLU) flattened into the horizontal plane. Near
            # vertical it degenerates, so keep the last good heading.
            forward = Rotation.from_quat(state.attitude).apply([1.0, 0.0, 0.0])
            forward[2] = 0.0
            norm = float(np.linalg.norm(forward))
            if norm > 1e-6:
                self._chase_fwd = forward / norm
            elif self._chase_fwd is None:
                return
            forward = self._chase_fwd

            eye = position - forward * CHASE_BACK + np.array([0.0, 0.0, CHASE_UP])
            target = position + forward * CHASE_AHEAD
            set_camera_view(
                eye=eye,
                target=target,
                camera_prim_path="/OmniverseKit_Persp",
                viewport_api=self._chase_vp,
            )
        except Exception as exc:
            # One warning, then stop trying -- a broken chase view must never
            # take the flight down with it.
            if not self._chase_warned:
                self._chase_warned = True
                carb.log_warn(f"Chase camera update failed, disabling it: {exc}")
            self._chase_vp = None

    def run(self):
        """Application main loop - identical to the stock T650 scenario."""

        self.timeline.play()
        steps = 0

        while (
            simulation_app.is_running()
            and not self.stop_sim
            and (not STEP_LIMIT or steps < STEP_LIMIT)
        ):
            self.world.step(render=not HEADLESS)
            self._update_chase_camera()
            self._record_frame()
            steps += 1

        # capture_viewport_to_file is ASYNCHRONOUS -- it schedules the write and
        # returns. Closing immediately loses the last frames, so give the queued
        # captures some updates to land.
        if self._rec_n:
            for _ in range(30):
                simulation_app.update()
            print(f"[record] {self._rec_n} frames in {RECORD_DIR}\n"
                  f"[record] ffmpeg -framerate {RECORD_FPS:.0f} -i "
                  f"{RECORD_DIR}/frame_%05d.png -c:v libx264 -crf 18 "
                  f"-pix_fmt yuv420p gate.mp4", flush=True)

        carb.log_warn("FscGateSplatSim Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()


def main():
    sim = FscGateSplatSim()
    sim.run()


if __name__ == "__main__":
    main()
