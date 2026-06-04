from setuptools import find_packages, setup

package_name = "px4_direct_actuator_test"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/direct_actuator_test.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Keilan Pieper",
    maintainer_email="pieperkeilan@gmail.com",
    description="Standalone test node for PX4 direct_actuator offboard control over uXRCE-DDS.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "direct_actuator_test = px4_direct_actuator_test.direct_actuator_test_node:main",
        ],
    },
)
