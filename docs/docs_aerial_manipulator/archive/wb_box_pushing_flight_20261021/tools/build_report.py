#!/usr/bin/env python3
"""Build the published page: report_src.html with the figure embedded as a data URI -> report.html.
Usage: /usr/bin/python3 build_report.py"""
import base64
import os

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
src = open(os.path.join(ROOT, "report_src.html")).read()
fig = base64.b64encode(open(os.path.join(ROOT, "fig_hw_vs_sim.png"), "rb").read()).decode()
assert src.count("{{FIG}}") == 1
open(os.path.join(ROOT, "report.html"), "w").write(src.replace("{{FIG}}", "data:image/png;base64," + fig))
print("wrote", os.path.join(ROOT, "report.html"))
