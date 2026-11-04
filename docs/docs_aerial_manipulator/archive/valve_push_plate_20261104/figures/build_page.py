#!/usr/bin/env python3
"""Build the updated Valve Turning Design page (artifact BppzF9zYUZoNwrdPymSQoi) = the live page as read on 2026-11-04
+ section 5, THE PUSH PLATE (page_section.html / page_section.js, data page_data.json and sim_data.json).
    python3 build_page.py <the saved live page> [out]
The saved page carries the viewer's skeleton (doctype, head, body); the publish adds its own, so it is stripped."""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
src = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, "valve_turning_design.html")
s = open(src).read()
a = s.index("<title>")
b = s.rindex("</body></html>") if "</body></html>" in s else len(s)
s = s[a:b].rstrip() + "\n"


def sub(old, new, count=1):
    global s
    if s.count(old) != count:
        raise SystemExit(f"expected {count} x {old[:60]!r}, found {s.count(old)}")
    s = s.replace(old, new)


if 'id="plate"' in s:
    raise SystemExit("the page already has section 5")
sub("  --claw: #59606b;", "  --claw: #59606b;\n  --pplate: #a08cf0;      /* the push plate */")
sub("#h3d { width: 100%; height: 520px; }", "#h3d { width: 100%; height: 520px; }\n#p3d { width: 100%; height: 540px; }\n"
    "#pSimTable tr.warn td { color: var(--warn); }\n#pSimTable td:nth-child(2), #pSimTable td:nth-child(3) { min-width: 15em; }")
sub('<a href="#handle">4. A handle the claw can roll on</a>', '<a href="#handle">4. A handle the claw can roll on</a><a href="#plate">5. A push plate</a>')
section = open(os.path.join(HERE, "page_section.html")).read()
anchor = '\n</div>\n\n<script src="https://cdn.jsdelivr.net/npm/plotly'
sub(anchor, "\n" + section + anchor)
js = open(os.path.join(HERE, "page_section.js")).read()
pd = json.load(open(os.path.join(HERE, "page_data.json")))
simf = os.path.join(HERE, "sim_data.json")
sim = json.load(open(simf)) if os.path.exists(simf) else None
js = js.replace("/*PAGE_DATA*/null", json.dumps(pd, separators=(",", ":"))).replace(
    "/*SIM_DATA*/null", json.dumps(sim, separators=(",", ":")) if sim else "null")
last = s.rindex("</script>")
s = s[:last] + js + "\n" + s[last:]
open(out, "w").write(s)
print(f"wrote {out}: {len(s) / 1e6:.2f} MB, sim table {'IN' if sim else 'not yet'}")
