"""Build the published page from ../report.html: inline every {{name.png}} from ../figures as a data URI.
usage: build_artifact_page.py <out.html>"""
import base64, os, re, sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "report.html")
FIG = os.path.join(HERE, "..", "figures")


def inline(m):
    with open(os.path.join(FIG, m.group(1)), "rb") as f:
        return "data:image/png;base64," + base64.b64encode(f.read()).decode()


page = re.sub(r"\{\{([\w.-]+\.png)\}\}", inline, open(SRC).read())
open(sys.argv[1], "w").write(page)
print("wrote", sys.argv[1], f"{len(page) / 1e6:.2f} MB")
