"""FSC VTOL flight controllers.

★ TRANSITIONAL sys.path SHIM -- read before removing.

Eleven modules in this package import their siblings by BARE name
(`from smc_outer import SMCOuter`, `import nmpc_so3_transition_node as so3`,
23 sites in total). That resolves only when the package's own directory is on
sys.path, which is true when a node is launched the way this project has always
launched them -- `python3 nmpc_cascade_node.py` from inside the package -- and
false as soon as the package is installed and imported properly. The result was
that the package built and installed fine but could not be IMPORTED:

    >>> import px4_vtol_direct_actuator.smc_outer_node
    ModuleNotFoundError: No module named 'smc_outer'

which is a hard blocker on shipping the controller as a release artefact.

Putting the package directory on sys.path here fixes all 23 sites at once, in
both invocation modes, and changes nothing about how the nodes currently run.
It is deliberately a stopgap: the correct fix is absolute package imports
(`from px4_vtol_direct_actuator.smc_outer import ...`), which is scheduled for
the library/node split, when every one of these modules is being renamed and
its imports rewritten anyway. Doing it here would mean editing all 23 twice.

Remove this shim in the same change that converts the imports.
"""
import os as _os
import sys as _sys

_pkg_dir = _os.path.dirname(_os.path.abspath(__file__))
if _pkg_dir not in _sys.path:
    _sys.path.insert(0, _pkg_dir)
