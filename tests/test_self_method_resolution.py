#!/usr/bin/env python3
"""Static check: every `self._private()` call resolves inside its own class.

WHY THIS EXISTS.  The offline `--test` path constructs the SOLVER class (FullNMPC) directly and
never builds the ROS node class (FullNMPCNode), so a method defined on the wrong class passes
every offline gate and then kills the node on the first line of its constructor:

    AttributeError: 'FullNMPCNode' object has no attribute '_publish_sched_ready'

That cost a flight attempt.  It happened because an edit was anchored on `def _fallback`, which
belongs to the solver, while the new methods were called from the node.  Both classes live in one
file and both have `self`, so nothing complained until runtime.

Only UNDERSCORE-prefixed methods are checked: those are never inherited from rclpy's Node or any
other base, so a call that does not resolve locally is a genuine error rather than inheritance.

Run with either python:  python3 tests/test_self_method_resolution.py
"""
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
# The controllers live in fsc_autopilot_ros2 now; resolve them through
# controller_path rather than by a path relative to this repository.
sys.path.insert(0, ROOT)
from controller_path import inner_dir   # noqa: E402

TARGETS = [os.path.join(inner_dir(), f) for f in (
    "nmpc_full_smc_node.py",
    "nmpc6_cruise_node.py",
    "nmpc_so3_transition_node.py",
    "nmpc_cascade_node.py",
)]


def check(path):
    """Return a list of (class, method, lineno) private calls that do not resolve."""
    with open(path) as fh:
        tree = ast.parse(fh.read(), filename=path)
    problems = []
    for cls in [n for n in ast.walk(tree) if isinstance(n, ast.ClassDef)]:
        defined = {n.name for n in cls.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
        # attributes assigned as callables (self._f = lambda / partial) count as defined
        for n in ast.walk(cls):
            if isinstance(n, ast.Attribute) and isinstance(n.ctx, ast.Store) \
                    and isinstance(n.value, ast.Name) and n.value.id == "self":
                defined.add(n.attr)
        for n in ast.walk(cls):
            if not (isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)):
                continue
            f = n.func
            if not (isinstance(f.value, ast.Name) and f.value.id == "self"):
                continue
            name = f.attr
            if not name.startswith("_") or name.startswith("__"):
                continue
            if name not in defined:
                problems.append((cls.name, name, f.lineno))
    return problems


def main():
    bad = 0
    for rel in TARGETS:
        path = rel        # TARGETS are absolute (resolved via controller_path)
        if not os.path.exists(path):
            print(f"  skip (missing): {rel}")
            continue
        problems = check(path)
        if problems:
            bad += len(problems)
            print(f"FAIL {rel}")
            for cls, name, line in problems:
                print(f"   {rel}:{line}  self.{name}() is not defined in class {cls}")
        else:
            print(f"  OK   {os.path.basename(rel)}")
    if bad:
        print(f"\n{bad} unresolved private self-call(s) — a method is on the wrong class")
        return 1
    print("\nALL SELF-METHOD CALLS RESOLVE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
