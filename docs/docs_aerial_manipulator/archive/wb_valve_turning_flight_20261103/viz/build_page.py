#!/usr/bin/env python3
"""Builds fin_in_u_192853.html from fin_in_u_template.html + fin_in_u_192853.json (run tools/viz_data_192853.py first)."""
import os
H = os.path.dirname(os.path.abspath(__file__))
t = open(os.path.join(H, "fin_in_u_template.html")).read()
d = open(os.path.join(H, "fin_in_u_192853.json")).read()
assert t.count("/*DATA*/null") == 1
open(os.path.join(H, "fin_in_u_192853.html"), "w").write(t.replace("/*DATA*/null", d))
print(os.path.join(H, "fin_in_u_192853.html"))
