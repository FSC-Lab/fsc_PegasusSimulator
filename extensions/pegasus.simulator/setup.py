"""
| File: setup.py
| Author: Marcelo Jacinto (marcelo.jacinto@tecnico.ulisboa.pt)
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: File that defines the installation requirements for this python package.
"""
import os

from setuptools import setup
from setuptools.command.egg_info import egg_info

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
# Read the extension.toml file

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # generic
    "numpy",
    "pymavlink",
    "scipy",
    "pyyaml",
]

# Auxiliar class to patch the Isaac Sim Kit App (to launch the omni.isaac.replicator extension required by the People extension)
class PatchIsaacSimKitApp(egg_info):

    def has_extension(self, file_path: str, extension_name: str) -> bool:
        """
        Checks if a given file contains specific Isaac Sim extension name
        """
        with open(file_path, 'r') as f:
            for line in f:
                if extension_name in line:
                    return True
        return False
    
    def add_extensions(self, file_path: str, extensions: list, marker_line="[dependencies]"):
        """
        Adds a list of extensions to the given file
        """

        with open(file_path, "r") as f:
            lines = f.readlines()

        try:
            index = next(i for i, line in enumerate(lines) if marker_line in line)
        except StopIteration:
            print(f"Could not find marker line '{marker_line}' in file.")
            return
        
        # Insert the extension line right after [dependencies]
        for extension_line in extensions:
            lines.insert(index + 1, extension_line + "\n")
            print(f"Extension '{extension_line}' added to '{file_path}'.")
            index += 1

        # Write the modified content back to the file
        with open(file_path, "w") as f:
            f.writelines(lines)


    @staticmethod
    def _locate_isaacsim():
        """Best-effort resolution of the Isaac Sim install root.

        Prefers ISAACSIM_PATH (standalone zip layout); otherwise falls back to
        the pip-installed `isaacsim` package directory. Returns None if neither
        is available (e.g. during isolated builds before deps are installed)."""
        isaacsim_path = os.environ.get("ISAACSIM_PATH")
        if isaacsim_path:
            return isaacsim_path
        try:
            import isaacsim
            return os.path.dirname(isaacsim.__file__)
        except Exception:
            return None

    def run(self):
        egg_info.run(self)

        # Resolve the Isaac Sim install root (env var or pip package).
        isaacsim_path = self._locate_isaacsim()
        if not isaacsim_path:
            print("PatchIsaacSimKitApp: Isaac Sim not found (ISAACSIM_PATH unset and "
                  "`isaacsim` not importable); skipping kit-app patch.")
            return

        # Check if the isaacsim.exp.base.kit file contains the "isaacsim.replicator.agent.core" extension
        isaac_sim_kit_app = os.path.join(isaacsim_path, "apps", "isaacsim.exp.base.kit")

        if not os.path.isfile(isaac_sim_kit_app):
            print(f"PatchIsaacSimKitApp: kit file not found at {isaac_sim_kit_app}; "
                  "skipping kit-app patch.")
            return

        if not self.has_extension(isaac_sim_kit_app, "isaacsim.replicator.agent.core"):
            self.add_extensions(isaac_sim_kit_app, ["\"isaacsim.replicator.agent.core\" = {}"], marker_line="[dependencies]")

        
# Installation operation
setup(
    name="pegasus-simulator",
    author="Marcelo Jacinto",
    maintainer="Marcelo Jacinto",
    maintainer_email="marcelo.jacinto@tecnico.ulisboa.pt",
    url="https://github.com/PegasusSimulator/PegasusSimulator/tree/main/extensions/pegasus.simulator",
    version="5.1.0",
    description="Extension providing the main framework interfaces for simulating aerial vehicles using PX4, Python or ROS 2 as a backend",
    keywords=["drone", "quadrotor", "multirotor", "UAV", "px4", "sitl", "robotics"],
    license="BSD-3-Clause",
    include_package_data=True,
    python_requires=">=3.7",
    install_requires=INSTALL_REQUIRES,
    cmdclass={'egg_info': PatchIsaacSimKitApp},
    packages=["pegasus.simulator"],
    classifiers=["Natural Language :: English", "Programming Language :: Python :: 3.7"],
    zip_safe=False,
)