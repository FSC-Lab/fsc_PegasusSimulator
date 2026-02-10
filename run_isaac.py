import argparse
import os
import pathlib
import subprocess
import sys
import textwrap

import distro

UBUNTU_VERSION_MAP = {
    "22.04": "humble",
    "24.04": "jazzy",
}


def _join_paths(*paths: str) -> str:
    return os.pathsep.join(p for p in paths if p)


def _native_ros2_visible():
    # Try importing rclpy first. If it exists and is from /opt/ros, we know ROS 2 is visible.
    try:
        import rclpy

        if rclpy.__file__.startswith("/opt/ros/"):
            return True
    finally:
        # If rclpy import fails, we can also check sys.path for any entries that point to /opt/ros
        return any(it.startswith("/opt/ros") for it in sys.path)


def run_isaac_sim():
    parser = argparse.ArgumentParser(
        description="Run a Python script within the Isaac Sim environment."
    )
    _ = parser.add_argument(
        "script", type=str, help="The Python script to run within Isaac Sim."
    )
    args, script_args = parser.parse_known_args()
    # 1. Locate the installed isaacsim package
    # We try to import it to find its location
    try:
        import isaacsim

        isaac_module_dir = pathlib.Path(isaacsim.__file__).parent
    except ImportError:
        print("Error: 'isaacsim' module not found. Is it installed via pip?")
        return 1

    # 2. Set necessary environment variables
    # These mimic the exports in your bash script
    kit_path = isaac_module_dir / "kit"

    env_vars = os.environ.copy()
    env_vars["CARB_APP_PATH"] = str(kit_path)
    env_vars["ISAAC_PATH"] = str(isaac_module_dir)
    env_vars["EXP_PATH"] = os.path.join(isaac_module_dir, "apps")
    env_vars["RESOURCE_NAME"] = "IsaacSim"
    try:
        ros_codename = UBUNTU_VERSION_MAP[distro.version()]
    except KeyError:
        print("Unsupported Ubuntu version. Only 22.04 and 24.04 are supported.")
        return 1

    env_vars["ROS_DISTRO"] = ros_codename
    env_vars["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"

    PREBUNDLE_LOCATIONS = {"exts/", "extscache/"}
    prebundle_dirs: list[str] = []
    for prebundle_location in PREBUNDLE_LOCATIONS:
        found = (isaac_module_dir / prebundle_location).glob("*/pip_prebundle")
        prebundle_dirs.extend(map(str, found))

    additional_paths = os.pathsep.join(
        str(isaac_module_dir / path)
        for path in [
            "kit/python/lib/python3.11",
            "kit/python/lib/python3.11/site-packages",
            "exts/isaacsim.simulation_app",
            "extsDeprecated/omni.isaac.kit",
            "kit/kernel/py",
            "kit/plugins/bindings-python",
        ]
    )

    current_path = env_vars.get("PYTHONPATH", "")
    env_vars["PYTHONPATH"] = _join_paths(
        current_path, additional_paths, *prebundle_dirs
    )

    # 3. Handle LD_PRELOAD for Linux (The "WAR for missing libcarb.so")
    if not sys.platform.startswith("linux"):
        print("Non-linux platforms not supported.")
    libcarb_path = kit_path / "libcarb.so"
    current_preload = env_vars.get("LD_PRELOAD", "")
    if str(libcarb_path) not in current_preload:
        env_vars["LD_PRELOAD"] = f"{libcarb_path}{os.pathsep}{current_preload}"

    additional_library_paths = os.pathsep.join(
        str(isaac_module_dir / path)
        for path in [
            f"exts/isaacsim.ros2.bridge/{ros_codename}/lib",
            "exts/isaacsim.robot.schema/plugins/lib",
            "kit",
            "kit/kernel/plugins",
            "kit/libs/iray",
            "kit/plugins",
            "kit/plugins/bindings-python",
            "kit/plugins/carb_gfx",
            "kit/plugins/rtx",
            "kit/plugins/gpu.foundation",
        ]
    )

    current_ld = env_vars.get("LD_LIBRARY_PATH", "")
    env_vars["LD_LIBRARY_PATH"] = _join_paths(
        current_ld, str(isaac_module_dir), additional_library_paths, *prebundle_dirs
    )

    target_script = str(args.script)

    # 5. Execute
    # We use the same python interpreter that is running this script
    cmd = [sys.executable, target_script] + script_args

    print(f"Launching Isaac Sim environment from: {isaac_module_dir}")

    try:
        # Replace the current process with the new one (Unix) or run subprocess (Windows)
        if sys.platform.startswith("linux"):
            os.execvpe(sys.executable, cmd, env_vars)
        else:
            _ = subprocess.run(cmd, env=env_vars, check=True)
    except subprocess.CalledProcessError as e:
        return e.returncode


if __name__ == "__main__":
    if _native_ros2_visible():
        print(
            textwrap.dedent(
                """
                Warning: Detected native ROS 2 installation in /opt/ros/. 
                This can cause conflicts with the ROS 2 environment provided by Isaac Sim.
                Clear your PYTHONPATH (and optionally LD_LIBRARY_PATH) of any /opt/ros entries before proceeding.
                """
            ).strip("\n")
        )
        sys.exit(1)
    sys.exit(run_isaac_sim())
