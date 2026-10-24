#!/usr/bin/env python3
"""Build ../report.html (the published 10-24 report) from report_template.html and the chart data.
    /usr/bin/python3 report_data.py > ../report_data.json
    /usr/bin/python3 report_data_pm.py > ../report_data_pm.json
    /usr/bin/python3 build_report.py"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))
tpl = open(os.path.join(HERE, "report_template.html")).read()
for marker, name in (("/*DATA*/null", "report_data.json"), ("/*DATA2*/null", "report_data_pm.json")):
    assert tpl.count(marker) == 1, marker
    tpl = tpl.replace(marker, open(os.path.join(HERE, "..", name)).read().strip())
open(os.path.join(HERE, "..", "report.html"), "w").write(tpl)
print("wrote", os.path.join(HERE, "..", "report.html"))
