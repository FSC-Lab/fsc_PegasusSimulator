"""
| File: setup.py
| Author: Longhao Qian (longhao.qian@mail.utoronot.ca)
| License: BSD-3-Clause. Copyright (c) 2025, Longhao Qian. All rights reserved.
| Description: File that defines the installation requirements for this python package.
"""
import os
from setuptools import setup, find_packages

if not os.environ.get("ISAACSIM_PATH"):
    raise RuntimeError(
        "ISAACSIM_PATH is not set. Export ISAACSIM_PATH=/path/to/isaacsim before installing."
    )

INSTALL_REQUIRES = [
    "numpy",
    "scipy",
    "pyyaml",
    # This is the key: depend on the Pegasus python package
    "pegasus-simulator==5.1.0",
]

setup(
    name="fsc-aerial-manipulation",
    version="0.1.0",
    description="FSC aerial manipulation modules built on top of Pegasus Simulator / Isaac Sim.",
    author="Longhao Qian",
    license="BSD-3-Clause",
    python_requires=">=3.11",
    packages=find_packages(exclude=("tests", "docs")),
    include_package_data=True,
    install_requires=INSTALL_REQUIRES,
    zip_safe=False,
)