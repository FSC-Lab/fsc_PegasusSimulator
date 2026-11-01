#!/usr/bin/env python3
"""Build ../report.html (the published 10-31 valve report) from report_template.html and the chart data.
    PYTHONNOUSERSITE=1 /usr/bin/python3 report_data.py > ../report_data.json
    /usr/bin/python3 build_report.py"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
tpl = open(os.path.join(HERE, "report_template.html")).read()
assert tpl.count("/*DATA*/null") == 1
tpl = tpl.replace("/*DATA*/null", open(os.path.join(HERE, "..", "report_data.json")).read().strip())
open(os.path.join(HERE, "..", "report.html"), "w").write(tpl)
print("wrote", os.path.join(HERE, "..", "report.html"))
