import os
import sys
from pathlib import Path

import isaacsim


def main():
    isaacsim_path = Path(isaacsim.__file__).parent
    if not isaacsim_path:
        print("Error: ISAACSIM_PATH is not set.")
        sys.exit(1)

    kit_file = os.path.join(isaacsim_path, "apps", "isaacsim.exp.base.kit")
    extension_line = '"isaacsim.replicator.agent.core" = {}'

    # Check and Add logic (Same as your original setup.py)
    with open(kit_file, "r") as f:
        lines = f.readlines()

    if any("isaacsim.replicator.agent.core" in line for line in lines):
        print("Isaac Sim is already patched.")
        return

    # Find [dependencies] and insert
    try:
        idx = next(i for i, line in enumerate(lines) if "[dependencies]" in line)
        lines.insert(idx + 1, extension_line + "\n")
        with open(kit_file, "w") as f:
            f.writelines(lines)
        print("Successfully patched Isaac Sim Kit.")
    except StopIteration:
        print("Could not find [dependencies] section.")
