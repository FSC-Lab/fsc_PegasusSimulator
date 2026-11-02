#!/usr/bin/env python3
"""Build ../logic_compare.html ("Valve-turning vs Box-pushing logic") from page_template.html + trace.json
(trace.json comes from build_trace.py), then splice abort_section.html + abort3d.json (the crash flight's abort,
from ../tools/abort3d_data.py) in at <!--ABORT_SECTION--> if the template has that marker, else before </main>.
Run with /usr/bin/python3."""
import os
HERE = os.path.dirname(os.path.abspath(__file__))
s = open(os.path.join(HERE, "page_template.html")).read()
d = open(os.path.join(HERE, "trace.json")).read()
s = s.replace("__DATA__", d)
sec_p, dat_p = os.path.join(HERE, "abort_section.html"), os.path.join(HERE, "abort3d.json")
if os.path.exists(sec_p) and os.path.exists(dat_p):
    sec = open(sec_p).read().replace("__ABORT3D__", open(dat_p).read())
    s = s.replace("<!--ABORT_SECTION-->", sec) if "<!--ABORT_SECTION-->" in s else s.replace("</main>", sec + "\n</main>", 1)
open(os.path.join(HERE, "..", "logic_compare.html"), "w").write(s)
print("wrote logic_compare.html")
