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
    install_requires=["setuptools", "numpy"],
    # casadi is required only by the NMPC controllers; the safe geometric
    # hover path, the SMC outer loop and the bridges are numpy-only.
    extras_require={"nmpc": ["casadi"]},
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
            "geometric_hover = px4_vtol_direct_actuator.geometric_hover_node:main",
            "nmpc_cascade = px4_vtol_direct_actuator.nmpc_cascade_node:main",
            "nmpc6_cruise = px4_vtol_direct_actuator.nmpc6_cruise_node:main",
            "nmpc_full_smc = px4_vtol_direct_actuator.nmpc_full_smc_node:main",
            "nmpc_so3_transition = px4_vtol_direct_actuator.nmpc_so3_transition_node:main",
            "nmpc_so3_cascade = px4_vtol_direct_actuator.nmpc_so3_cascade_node:main",
            "vtol_nmpc = px4_vtol_direct_actuator.vtol_nmpc:main",
            "smc_outer = px4_vtol_direct_actuator.smc_outer_node:main",
            "smc_cruise = px4_vtol_direct_actuator.smc_cruise_node:main",
            "nmpc_attitude = px4_vtol_direct_actuator.nmpc_attitude_node:main",
            "nmpc_cruise = px4_vtol_direct_actuator.nmpc_cruise_node:main",
            "nmpc_quad = px4_vtol_direct_actuator.nmpc_quad_node:main",
            "aero_transition = px4_vtol_direct_actuator.aero_transition_node:main",
            "mocap_to_px4_ev = px4_vtol_direct_actuator.mocap_to_px4_ev:main",
            "attitude_setpoint_mux = px4_vtol_direct_actuator.attitude_setpoint_mux:main",
        ],
    },
)
