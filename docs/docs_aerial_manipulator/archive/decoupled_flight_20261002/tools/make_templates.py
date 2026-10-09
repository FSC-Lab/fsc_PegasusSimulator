"""Derive the 1002 section's figure templates from the 0928 ones (same code, own element ids, the four decoupled
runs as keys, violet = the new gains). Every replacement is asserted, so a changed 0928 template fails loudly.
    python3 make_templates.py   -> ee3d_dec_section.html, err_dec_section.html
"""
import os
HERE = os.path.dirname(os.path.abspath(__file__)); SRC = os.path.join(HERE, "..", "..", "wb_vs_decoupled_flight_20260928", "tools")


def sub(t, a, b, n=1):
    assert t.count(a) == n, (a, t.count(a)); return t.replace(a, b)


t = open(os.path.join(SRC, "ee3d_section.html"), encoding="utf-8").read()
for a, b in (('id="ee3d"', 'id="ee3dd"'), ("getElementById('ee3d')", "getElementById('ee3dd')"), ("ee3d-reset", "ee3dd-reset"),
             ("ee3d-fallback", "ee3dd-fallback"), ("ee3d-data", "ee3dd-data")):
    t = t.replace(a, b) if t.count(a) > 1 else sub(t, a, b)
cells = "".join(f'    <div class="ee3d-cell"><div class="ee3d-title" data-title="{k}"></div><div class="ee3d-plot" data-flight="{k}" aria-label="Rotatable 3D plot, {lab}"></div></div>\n'
                for k, lab in (("d1", "decoupled flight 1 of 09-28"), ("d2", "decoupled flight 2 of 09-28"), ("r1", "run 1 of 10-02"), ("r2", "run 2 of 10-02")))
i0 = t.index('    <div class="ee3d-cell">'); i1 = t.index("  </div>\n  <p id=\"ee3dd-fallback\"")
t = t[:i0] + cells + t[i1:]
t = sub(t, "var KEYS=['w1','w2','d1','d2'];", "var KEYS=['d1','d2','r1','r2'];")
t = sub(t, "(D[k].rig==='wb'?' · whole-body':' · decoupled')", "(D[k].gain==='new'?' · new gains':' · old gains')")
t = sub(t, "orange:tok('--orange'),aqua:tok('--aqua')};", "orange:tok('--orange'),aqua:tok('--aqua'),violet:tok('--violet')};")
t = sub(t, "eec=f.rig==='wb'?c.blue:c.orange", "eec=f.gain==='new'?c.violet:c.orange")
t = sub(t, '<span class="sw" style="--c:var(--blue)"></span><span class="sw" style="--c:var(--orange);margin-left:-4px"></span>',
        '<span class="sw" style="--c:var(--orange)"></span><span class="sw" style="--c:var(--violet);margin-left:-4px"></span>')
t = sub(t, "Table 1 below carries", "Table 8 below carries")
i0 = t.index("<figcaption>"); i1 = t.index("</figcaption>")
t = t[:i0] + ("<figcaption>Fig. 4 — The decoupled controller's four circle runs in 3D, world frame, the old gains (09-28, orange EE line) "
              "beside the new gains (10-02, violet EE line). Both 10-02 runs come from one flight. All four views share one set of axis ranges; "
              "each rotates on its own (drag to rotate, scroll or pinch to zoom, hover the EE path for time and error), and the buttons switch a line "
              "on or off in all four. The 10-02 runs are drawn over the 27.2 s they are scored on, the 09-28 runs over the full 28.2 s run.") + t[i1:]
import re
n0 = len(re.findall(r'<script src="https://cdn\.jsdelivr\.net/npm/plotly[^"]*"></script>\n?', t)); assert n0 == 1, n0
t = re.sub(r'<script src="https://cdn\.jsdelivr\.net/npm/plotly[^"]*"></script>\n?', "", t)   # section 1 already loads Plotly
open(os.path.join(HERE, "ee3d_dec_section.html"), "w", encoding="utf-8").write(t)

t = open(os.path.join(SRC, "err_section.html"), encoding="utf-8").read()
for a, b in (('id="errts"', 'id="errtsd"'), ("getElementById('errts')", "getElementById('errtsd')"), ("errts-plot", "errtsd-plot"),
             ("errts-fallback", "errtsd-fallback"), ("errts-data", "errtsd-data"), ("errts-refit", "errtsd-refit")):
    t = t.replace(a, b) if t.count(a) > 1 else sub(t, a, b)
i0 = t.index('      <button type="button" data-f="w1"'); i1 = t.index("    </div>\n    <div class=\"seg\" role=\"group\" aria-label=\"Position rows\">")
btn = ('      <button type="button" data-f="d1" aria-pressed="true"><span class="sw" style="--c:var(--orange)"></span>DEC-1 09-28, old gains</button>\n'
       '      <button type="button" data-f="d2" aria-pressed="true"><span class="sw dash" style="--c:var(--orange)"></span>DEC-2 09-28, old gains</button>\n'
       '      <button type="button" data-f="r1" aria-pressed="true"><span class="sw" style="--c:var(--violet)"></span>Run 1 10-02, new gains</button>\n'
       '      <button type="button" data-f="r2" aria-pressed="true"><span class="sw dash" style="--c:var(--violet)"></span>Run 2 10-02, new gains</button>\n')
t = t[:i0] + btn + t[i1:]
t = sub(t, "KEYS=['w1','w2','d1','d2'];", "KEYS=['d1','d2','r1','r2'];")
t = sub(t, "var show={w1:true,w2:true,d1:true,d2:true}", "var show={d1:true,d2:true,r1:true,r2:true}")
t = sub(t, "blue:tok('--blue'),orange:tok('--orange')};", "orange:tok('--orange'),violet:tok('--violet')};")
t = sub(t, "var f=D[k], wb=f.rig==='wb';", "var f=D[k], nw=f.gain==='new';")
t = sub(t, "line:{color:wb?c.blue:c.orange,width:1.6,dash:(k==='w2'||k==='d2')?'dash':'solid'}",
        "line:{color:nw?c.violet:c.orange,width:1.6,dash:(k==='d2'||k==='r2')?'dash':'solid'}")
t = sub(t, "Tables 1 and 2 carry", "Tables 8 and 9 carry")
t = sub(t, 'aria-label="Error in the circle frame over the circle run, four flights"', 'aria-label="Error in the circle frame over the circle run, four decoupled runs"')
i0 = t.index("<figcaption>"); i1 = t.index("</figcaption>")
t = t[:i0] + ("<figcaption>Fig. 5 — Error over the circle run in the circle's own frame (radial = outward from the circle centre, along-track = "
              "along the direction of travel, positive ahead of the reference, vertical = up), the decoupled controller on the old gains (orange) "
              "and the new gains (violet); the second run of each pair is dashed. Switch the position rows to the airframe to see where the error "
              "originates. Click a run to hide it; the y-axes refit to what is shown, so hiding the two 09-28 runs zooms in on the new gains.") + t[i1:]
open(os.path.join(HERE, "err_dec_section.html"), "w", encoding="utf-8").write(t)
print("wrote ee3d_dec_section.html, err_dec_section.html")

# ---- single-section report (2026-10-03): six circle flights, three controller settings ----------------------------
t = open(os.path.join(SRC, "ee3d_section.html"), encoding="utf-8").read()
for a, b in (('id="ee3d"', 'id="ee3ds"'), ("getElementById('ee3d')", "getElementById('ee3ds')"), ("ee3d-reset", "ee3ds-reset"),
             ("ee3d-fallback", "ee3ds-fallback"), ("ee3d-data", "ee3ds-data")):
    t = t.replace(a, b) if t.count(a) > 1 else sub(t, a, b)
SIX = (("w1", "whole-body flight WB-1"), ("w2", "whole-body flight WB-2"), ("d1", "decoupled flight DEC-1, old gains"),
       ("d2", "decoupled flight DEC-2, old gains"), ("r1", "decoupled flight DEC-3, tuned gains"), ("r2", "decoupled flight DEC-4, tuned gains"))
cells = "".join(f'    <div class="ee3d-cell"><div class="ee3d-title" data-title="{k}"></div><div class="ee3d-plot" data-flight="{k}" aria-label="Rotatable 3D plot, {lab}"></div></div>\n'
                for k, lab in SIX)
i0 = t.index('    <div class="ee3d-cell">'); i1 = t.index("  </div>\n  <p id=\"ee3ds-fallback\"")
t = t[:i0] + cells + t[i1:]
t = sub(t, "var KEYS=['w1','w2','d1','d2'];", "var KEYS=['w1','w2','d1','d2','r1','r2'];")
t = sub(t, "(D[k].rig==='wb'?' · whole-body':' · decoupled')",
        "({wb:' · whole-body',old:' · decoupled, old gains',new:' · decoupled, tuned gains'})[D[k].grp]")
t = sub(t, "orange:tok('--orange'),aqua:tok('--aqua')};", "orange:tok('--orange'),aqua:tok('--aqua'),violet:tok('--violet')};")
t = sub(t, "eec=f.rig==='wb'?c.blue:c.orange", "eec=({wb:c.blue,old:c.orange,new:c.violet})[f.grp]")
t = sub(t, '<span class="sw" style="--c:var(--blue)"></span><span class="sw" style="--c:var(--orange);margin-left:-4px"></span>',
        '<span class="sw" style="--c:var(--blue)"></span><span class="sw" style="--c:var(--orange);margin-left:-4px"></span>'
        '<span class="sw" style="--c:var(--violet);margin-left:-4px"></span>')
t = sub(t, "Table 1 below carries", "Table 5 below carries")
i0 = t.index("<figcaption>"); i1 = t.index("</figcaption>")
t = t[:i0] + ("<figcaption>Fig. 1 — The six circle runs in 3D, world frame, over the scored 27.2 s. The EE line is blue for the "
              "whole-body controller, orange for the decoupled controller on its old gains and violet on its tuned gains. All six "
              "views share one set of axis ranges, so the circles compare directly. Each view rotates on its own: drag to rotate, "
              "scroll or pinch to zoom, hover the EE path for time and error. The buttons switch a line on or off in all six views. "
              "The planned EE circle is anchored at the gripper's bearing and height when the circle was selected, which is why the "
              "start points differ.") + t[i1:]
assert "__DATA__" in t
open(os.path.join(HERE, "summary_ee3d_section.html"), "w", encoding="utf-8").write(t)
print("wrote summary_ee3d_section.html")

# ---- section 2 (2026-10-05): the six figure-8 runs, derived from Fig. 1's template ---------------------------------
t = open(os.path.join(HERE, "summary_ee3d_section.html"), encoding="utf-8").read()
for a in ('id="ee3ds"', "getElementById('ee3ds')", "ee3ds-reset", "ee3ds-fallback", "ee3ds-data"):
    assert t.count(a) >= 1, a
    t = t.replace(a, a.replace("ee3ds", "ee3d8"))
EIGHT = (("w3", "whole-body run WB-3, 0.10 m/s"), ("w4", "whole-body run WB-4, 0.10 m/s"),
         ("r5", "decoupled run DEC-5, 0.10 m/s"), ("r6", "decoupled run DEC-6, 0.13 m/s"),
         ("w5", "whole-body run WB-5, 0.13 m/s"), ("w6", "whole-body run WB-6, 0.13 m/s"))
cells = "".join(f'    <div class="ee3d-cell"><div class="ee3d-title" data-title="{k}"></div><div class="ee3d-plot" data-flight="{k}" aria-label="Rotatable 3D plot, {lab}"></div></div>\n'
                for k, lab in EIGHT)
i0 = t.index('    <div class="ee3d-cell">'); i1 = t.index("  </div>\n  <p id=\"ee3d8-fallback\"")
t = t[:i0] + cells + t[i1:]
t = sub(t, "var KEYS=['w1','w2','d1','d2','r1','r2'];", "var KEYS=['w3','w4','r5','r6','w5','w6'];")
t = sub(t, "EE planned (r = 0.50 m)", "EE planned (figure-8, 1.40 × 0.70 m)")
t = sub(t, '<span class="sw" style="--c:var(--blue)"></span><span class="sw" style="--c:var(--orange);margin-left:-4px"></span>'
        '<span class="sw" style="--c:var(--violet);margin-left:-4px"></span>',
        '<span class="sw" style="--c:var(--blue)"></span><span class="sw" style="--c:var(--violet);margin-left:-4px"></span>')
t = sub(t, "Table 5 below carries", "Table 9 below carries")
# each figure-8 is centred on its own flight's EE hover point: one common span (the same scale in every view), each
# view centred on its own data, instead of one set of axis ranges that would shrink every figure-8
t = sub(t, "  // one set of axis ranges for all four views, so the circles compare at a glance\n"
           "  var lo=[1e9,1e9,1e9], hi=[-1e9,-1e9,-1e9];\n"
           "  KEYS.forEach(function(k){var f=D[k];f.ee.concat(f.ee_ref,f.base,f.base_ref).forEach(function(p){for(var q=0;q<3;q++){if(p[q]<lo[q])lo[q]=p[q];if(p[q]>hi[q])hi[q]=p[q];}});});\n",
        "  // one common span on each axis (the same scale in every view), each view centred on its own figure-8\n"
        "  var LO={}, HI={}, half=[0,0,0];\n"
        "  KEYS.forEach(function(k){var f=D[k],a=[1e9,1e9,1e9],b=[-1e9,-1e9,-1e9];f.ee.concat(f.ee_ref,f.base,f.base_ref).forEach(function(p){for(var q=0;q<3;q++){if(p[q]<a[q])a[q]=p[q];if(p[q]>b[q])b[q]=p[q];}});\n"
        "    for(var q=0;q<3;q++){half[q]=Math.max(half[q],(b[q]-a[q])/2);} LO[k]=a; HI[k]=b;});\n"
        "  KEYS.forEach(function(k){for(var q=0;q<3;q++){var mid=(LO[k][q]+HI[k][q])/2;LO[k][q]=mid-half[q];HI[k][q]=mid+half[q];}});\n")
t = sub(t, "    var ink=tok('--ink'),ink2=tok('--ink2'),line=tok('--line'), pad=0.04;\n",
        "    var ink=tok('--ink'),ink2=tok('--ink2'),line=tok('--line'), pad=0.04, lo=LO[k], hi=HI[k];\n")
n0 = len(re.findall(r'<script src="https://cdn\.jsdelivr\.net/npm/plotly[^"]*"></script>\n?', t)); assert n0 == 1, n0
t = re.sub(r'<script src="https://cdn\.jsdelivr\.net/npm/plotly[^"]*"></script>\n?', "", t)   # section 1 already loads Plotly
i0 = t.index("<figcaption>"); i1 = t.index("</figcaption>")
t = t[:i0] + ("<figcaption>Fig. 2 — The six figure-8 runs in 3D, world frame, over each run's scored span. The EE line is blue for the "
              "whole-body controller and violet for the decoupled controller on its tuned gains. All six views share one scale; each "
              "view is centred on its own figure-8, because the planned figure-8 is centred on the gripper's position when it was "
              "selected and that differs between flights. Each view rotates on its own: drag to rotate, scroll or pinch to zoom, hover "
              "the EE path for time and error. The buttons switch a line on or off in all six views.") + t[i1:]
assert "__DATA__" in t
open(os.path.join(HERE, "fig8_ee3d_section.html"), "w", encoding="utf-8").write(t)
print("wrote fig8_ee3d_section.html")
