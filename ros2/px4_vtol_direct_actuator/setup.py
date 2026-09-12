from setuptools import find_packages, setup

package_name = "px4_vtol_direct_actuator"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/vtol_direct_actuator.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Keilan Pieper",
    maintainer_email="pieperkeilan@gmail.com",
    description="Direct-actuator offboard node for a dual independent-tiltrotor VTOL over uXRCE-DDS.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "vtol_direct_actuator = px4_vtol_direct_actuator.vtol_direct_actuator_node:main",
            "tiltrotor_control = px4_vtol_direct_actuator.tiltrotor_control_node:main",
        ],
    },
)
