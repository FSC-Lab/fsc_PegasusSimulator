#!/usr/bin/env python3
"""Embed figures/*.png into report_src.html as data URIs -> ../report.html (the published page).

    python3 tools/build_report.py      (run from the campaign directory)
"""
import base64, os, re
src = open("tools/report_src.html").read()
out = re.sub(r'src="(figures/[^"]+\.png)"',
             lambda m: 'src="data:image/png;base64,%s"' % base64.b64encode(open(m.group(1), "rb").read()).decode(),
             src)
open("report.html", "w").write(out)
print("report.html", os.path.getsize("report.html") // 1024, "kB")
