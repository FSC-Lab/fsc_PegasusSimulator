"""Layout audit of the published page in headless Chrome at a given viewport width.

    python3 measure_layout.py <page.html> <width>

Prints per-block-type right-edge coverage and overflow. Coverage = block right edge / content-column right edge,
over blocks whose text wraps (a block whose text is short legitimately ends early, so only blocks with >= 2
rendered lines are scored). Tables report their width against the column and whether they scroll sideways.
The figures directory next to report.html is symlinked into the temporary page directory.
"""
import sys, subprocess, re, json, tempfile, os, html as H

page, width = sys.argv[1], int(sys.argv[2])
src = open(page).read()
probe = r"""
<script>
window.addEventListener('load',function(){setTimeout(function(){
 var col=document.body.getBoundingClientRect(), cs=getComputedStyle(document.body);
 var colR=col.right-parseFloat(cs.paddingRight), colL=col.left+parseFloat(cs.paddingLeft);
 var out={vw:innerWidth,colL:colL,colR:colR,docOverflow:document.documentElement.scrollWidth-innerWidth,groups:{}};
 function add(k,v){(out.groups[k]=out.groups[k]||[]).push(v);}
 function lines(el){var lh=parseFloat(getComputedStyle(el).lineHeight)||20;return Math.round(el.getBoundingClientRect().height/lh);}
 document.querySelectorAll('body > p, body > ul > li, body > ol > li, body > .verdict, body > figure, body > h1, body > h2, body > h3, body > h4').forEach(function(el){
   var r=el.getBoundingClientRect(); var tag=el.tagName.toLowerCase()+(el.className?'.'+el.className:'');
   var w=(r.right-colL)/(colR-colL); add(tag,{w:+w.toFixed(3),lines:lines(el),txt:el.textContent.trim().slice(0,40)});});
 document.querySelectorAll('table').forEach(function(t){var w=t.closest('.tscroll')||t; var r=w.getBoundingClientRect();
   var tb=t.querySelector('tbody')||t; var tr=tb.getBoundingClientRect(); add('table',{w:+((tr.right-colL)/(colR-colL)).toFixed(3),wrapW:+((r.right-colL)/(colR-colL)).toFixed(3),scrolls:(w.scrollWidth>w.clientWidth+1),txt:t.textContent.trim().slice(0,30)});});
 var lst={}; document.querySelectorAll('ul,ol').forEach(function(l){var k=l.tagName.toLowerCase(); var p=parseFloat(getComputedStyle(l).paddingLeft); (lst[k]=lst[k]||new Set()).add(p);});
 out.listIndent={}; for(var k in lst) out.listIndent[k]=Array.from(lst[k]);
 var pre=document.createElement('pre'); pre.id='__out'; pre.textContent=JSON.stringify(out); document.body.appendChild(pre);
},2500);});
</script>"""
html = src.replace("</body>", probe + "</body>", 1) if "</body>" in src else src + probe
wrapped = ('<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">'
           '<style>[hidden]:not([hidden=until-found i]){display:none!important}</style></head><body>' + html + '</body></html>') \
    if "<html" not in src.lower() else html
d = tempfile.mkdtemp()
p = os.path.join(d, "index.html"); open(p, "w").write(wrapped)
os.symlink(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "figures"), os.path.join(d, "figures"))
r = subprocess.run(["google-chrome", "--headless=new", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--hide-scrollbars",
                    f"--window-size={width},1000", "--virtual-time-budget=15000", "--dump-dom", "file://" + p],
                   capture_output=True, text=True, timeout=180)
m = re.search(r'<pre id="__out">(.*?)</pre>', r.stdout, re.S)
if not m:
    print("no output", r.stderr[-300:]); sys.exit(1)
o = json.loads(H.unescape(m.group(1)))
print(f"viewport {o['vw']}px, column {o['colL']:.0f}..{o['colR']:.0f} ({o['colR']-o['colL']:.0f}px); page horizontal overflow: {o['docOverflow']}px; list indent px: {o['listIndent']}")
for k, v in sorted(o["groups"].items()):
    if k == "table":
        print(f"  {k:12s} n={len(v)}  table width / column: min {min(x['w'] for x in v):.2f} max {max(x['w'] for x in v):.2f}; "
              f"tables that need sideways scroll: {sum(1 for x in v if x['scrolls'])}; tables ending short of the column (<0.97): {sum(1 for x in v if x['w']<0.97)}")
        for x in v:
            if x['w'] < 0.97 or x['scrolls']:
                print(f"      table '{x['txt']}': width {x['w']:.2f} of column, scrolls={x['scrolls']}")
    else:
        wb = [x for x in v if x['lines'] >= 2]
        if wb:
            print(f"  {k:12s} n={len(v):3d}  wrapping blocks {len(wb):3d}: right edge / column  min {min(x['w'] for x in wb):.2f}  median {sorted(x['w'] for x in wb)[len(wb)//2]:.2f}")
        else:
            print(f"  {k:12s} n={len(v):3d}  (no wrapping blocks) widths {min(x['w'] for x in v):.2f}-{max(x['w'] for x in v):.2f}")
