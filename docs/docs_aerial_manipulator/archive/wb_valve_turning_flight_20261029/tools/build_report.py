#!/usr/bin/env python3
"""Build ../report.html (the published 10-29 valve report) from report_template.html, embedding
../fig_flight.png as a data URI.
    PYTHONNOUSERSITE=1 /usr/bin/python3 fig_flight.py   # only if the figure changed
    /usr/bin/python3 build_report.py"""
import base64
import os

HERE = os.path.dirname(os.path.abspath(__file__))
tpl = open(os.path.join(HERE, "report_template.html")).read()
marker = "{{FIG_FLIGHT}}"
assert tpl.count(marker) == 1, marker
png = open(os.path.join(HERE, "..", "fig_flight.png"), "rb").read()
tpl = tpl.replace(marker, "data:image/png;base64," + base64.b64encode(png).decode())
out = os.path.join(HERE, "..", "report.html")
open(out, "w").write(tpl)
print("wrote", out, f"{len(tpl) / 1e6:.2f} MB")
