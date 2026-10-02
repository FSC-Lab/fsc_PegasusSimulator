#!/usr/bin/env python3
"""Embed the two interactive blocks of report section 2 into report.html, in place and idempotently.

    python3 embed_interactive.py <data_dir>

<data_dir> holds ee3d.json (from ee3d_data.py), ee_err.json (from ee_err_data.py) and arm_trk.json (from
arm_trk_data.py). The blocks are
the templates tools/ee3d_section.html (figure 1, four rotatable 3D views) and tools/ee_err_section.html
(figure 2, EE error by channel with signal toggles and y-scale modes); their __DATA__ placeholder is
replaced by the JSON. In report.html each block sits between <!--ee3d:start-->/<!--ee3d:end--> and
<!--eeerr:start-->/<!--eeerr:end-->. Run build_artifact_page.py afterwards.
"""
import os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.join(os.path.dirname(HERE), "report.html")


def block(name, template, data_file):
    t = open(os.path.join(HERE, template), encoding="utf-8").read()
    assert t.count("__DATA__") == 1, template
    return f"<!--{name}:start-->\n" + t.replace("__DATA__", open(data_file, encoding="utf-8").read().strip()) + f"<!--{name}:end-->\n"


def put(src, name, new, legacy):
    marked = re.compile(rf"<!--{name}:start-->.*?<!--{name}:end-->\n?", re.S)
    if marked.search(src):
        return marked.sub(lambda m: new, src, count=1)
    m = legacy.search(src)                       # first run: replace the un-marked original block
    if not m:
        raise SystemExit(f"could not find the {name} block in report.html")
    return src[:m.start()] + new + src[m.end():]


if __name__ == "__main__":
    data = sys.argv[1]
    src = open(REPORT, encoding="utf-8").read()
    src = put(src, "ee3d", block("ee3d", "ee3d_section.html", os.path.join(data, "ee3d.json")),
              re.compile(r'<figure class="wide">\s*<div class="ee3d" id="ee3d">.*?Fig\. 1 —.*?</figcaption></figure>\n?', re.S))
    src = put(src, "eeerr", block("eeerr", "ee_err_section.html", os.path.join(data, "ee_err.json")),
              re.compile(r'<figure class="wide"><img src="figures/f_ee_errors\.png".*?</figure>\n?', re.S))
    src = put(src, "armtrk", block("armtrk", "arm_trk_section.html", os.path.join(data, "arm_trk.json")),
              re.compile(r'(?=<h4 id="s3-flights-1-and-2-the-slow-sinusoid">)'))   # first run: insert before 3.1's first part
    stats = os.path.join(data, "arm_stats.html")
    if os.path.exists(stats):                    # section 3.1's statistics tables, from arm_stats_table.py
        tables = open(stats, encoding="utf-8").read()
        src = re.sub(r"<!--armstats:start-->.*?<!--armstats:end-->\n",
                     lambda m: "<!--armstats:start-->\n" + tables + "<!--armstats:end-->\n", src, count=1, flags=re.S)
    open(REPORT, "w", encoding="utf-8").write(src)
    print("embedded figure 1 (3D), figure 2 (EE error) and figure 3 (arm tracking + torques) into", REPORT)
