#!/usr/bin/env python3
"""Rebuild the Valve Turning Design page with the roll-fin handle as section 4.

Base: valve_turning_design_paddle.html (the published page with the first, hinged-paddle section 4,
kept unchanged). Section 4's HTML and script are replaced with section4.html / section4.js, and the
printed parts from roll_grip_parts.json (tools/roll_grip_design.py) go in as `const RG`. The component
pictures img/*.png (tools/render_components.py) replace the {{IMG:name}} placeholders as data URIs.

    python3 tools/build_page.py      # writes valve_turning_design.html
"""
import base64
import os
import re

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read(name):
    with open(os.path.join(HERE, name), encoding="utf-8") as f:
        return f.read()


def main():
    page = read("valve_turning_design_paddle.html")
    html, js, parts = read("section4.html"), read("section4.js"), read("roll_grip_parts.json")

    def img(m):
        with open(os.path.join(HERE, "img", m.group(1) + ".png"), "rb") as f:
            return "data:image/png;base64," + base64.b64encode(f.read()).decode()
    html = re.sub(r"\{\{IMG:([a-z0-9_]+)\}\}", img, html)
    assert "{{IMG:" not in html

    a = page.index('  <h2 id="handle">')
    b = page.index("\n</div>\n\n<script src=")
    page = page[:a] + html.rstrip("\n") + "\n" + page[b:]

    a = page.index("// ---------- 4. the hinged handle")
    b = page.rindex("</script>")
    page = page[:a] + "const RG = " + parts.strip() + ";\n" + js.rstrip("\n") + "\n" + page[b:]

    for must in ('id="hSect"', 'id="hExp"', "const RG = ", "drawHandleSections()", "plotly.js-gl3d-dist-min@2.35.2", "wght@400"):
        assert must in page, must
    with open(os.path.join(HERE, "valve_turning_design.html"), "w", encoding="utf-8") as f:
        f.write(page)
    print(f"valve_turning_design.html: {len(page.encode()):,} bytes")


if __name__ == "__main__":
    main()
