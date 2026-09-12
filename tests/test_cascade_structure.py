#!/usr/bin/env python3
"""Structural check on nmpc_cascade_node: the right callables on the right objects.

WHY: extracting `build_refs` to module level once left it sitting BETWEEN two methods of `Outer`,
so every method after it (`_run` among them) became a nested function inside `build_refs` instead
of a method. The file still parsed, and the offline `--test` gate still PASSED -- because the gate
calls `build_refs` directly and never constructs `Outer`. The node then died on startup with
`'Outer' object has no attribute '_run'`.

A gate that does not exercise the wiring is not a gate for the wiring. This is that check, and it
is deliberately import-only: no ROS, no solver build, so it runs anywhere in under a second.
"""
import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "ros2", "px4_vtol_direct_actuator",
                   "px4_vtol_direct_actuator", "nmpc_cascade_node.py")

# object -> names that must be defined directly on it
EXPECT = {
    None:          ["B_rot", "predict6", "tilt_clamp", "surface_authority",
                    "inner_step", "safe_hover", "build_refs", "run_ros", "offline_test"],
    "RotorAlloc":  ["__init__", "inv"],
    "OuterNMPC":   ["__init__"],
    "Outer":       ["__init__", "post", "get", "reset", "stop", "_refs", "_run"],
}


def main():
    tree = ast.parse(open(SRC).read())
    top_funcs = [n.name for n in tree.body if isinstance(n, ast.FunctionDef)]
    classes = {n.name: [m.name for m in n.body if isinstance(m, ast.FunctionDef)]
               for n in tree.body if isinstance(n, ast.ClassDef)}

    bad = []
    for owner, names in EXPECT.items():
        have = top_funcs if owner is None else classes.get(owner)
        if have is None:
            bad.append(f"class {owner} is missing entirely")
            continue
        for nm in names:
            if nm not in have:
                bad.append(f"{owner or '<module>'}.{nm} is not defined there")
        print(f"  OK   {owner or '<module level>'}: {len(names)} expected names present")

    # `Outer.__init__` starts a thread on self._run -- that attribute MUST resolve on the class.
    for cls, attrs in (("Outer", ["_run", "_refs"]),):
        for a in attrs:
            if a not in classes.get(cls, []):
                bad.append(f"{cls}.{a} referenced by __init__ but not a method of {cls}")

    # ★ BOTH control loops must be exception-wrapped. `inner` runs on every mocap sample and
    # `slow` is a timer callback; an uncaught throw in EITHER kills the rclpy executor and the
    # whole node, and PX4 then holds the last actuator frame forever. Wrapping one and not the
    # other is what put the aircraft into the ground in HIL bag casc2.
    node_cls = None
    for n in ast.walk(tree):
        if isinstance(n, ast.ClassDef) and n.name == "CascadeNode":
            node_cls = n
    if node_cls is None:
        bad.append("CascadeNode class not found")
    else:
        meths = {f.name: f for f in node_cls.body if isinstance(f, ast.FunctionDef)}
        for nm in ("inner", "slow"):
            f = meths.get(nm)
            if f is None:
                bad.append(f"CascadeNode.{nm} missing")
            elif not any(isinstance(x, ast.Try) for x in f.body):
                bad.append(f"CascadeNode.{nm}() is NOT exception-wrapped "
                           f"(an uncaught throw there freezes the actuator stream)")
            else:
                print(f"  OK   CascadeNode.{nm}() is exception-wrapped")

    if bad:
        print("\nSTRUCTURE BROKEN:")
        for b in bad:
            print("   -", b)
        return 1
    print("\nCASCADE STRUCTURE OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
