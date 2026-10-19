#!/usr/bin/env python3
"""Put the 2026-10-16 abort-on-the-box options into the box-pushing planner block
(make_push_pull_yaml.py) -- run once, after stage 4b (idempotent)."""
import os
p = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "push_pull_20261003", "tools",
                 "make_push_pull_yaml.py")
s = open(p).read()
if "push_pull_abort_from_measured" in s:
    print("already applied"); raise SystemExit(0)
old = """    push_pull_box_half_length: 0.12"""
new = """    # THE ABORT ON THE BOX (2026-10-16): with the claw on the box the abort holds the
    # MEASURED pose (not the reference the push was cut at -- a jammed box leaves the
    # planned claw 40+ mm ahead, and holding it dragged the claw along the fin and
    # wedged it in the open jaw), waits 1.0 s for the jaws (they take 0.75 s), backs
    # 3 cm off against the push, then climbs. Isaac 4b: with the old abort the climbing
    # open claw LIFTED a jammed 1 kg box 58 mm, it dropped off, and the vehicle flipped.
    push_pull_abort_from_measured: true
    push_pull_abort_back: 0.03
    push_pull_abort_release_wait_s: 1.0
    push_pull_box_half_length: 0.12"""
assert s.count(old) == 1
s = s.replace(old, new); open(p, "w").write(s); print("applied")
