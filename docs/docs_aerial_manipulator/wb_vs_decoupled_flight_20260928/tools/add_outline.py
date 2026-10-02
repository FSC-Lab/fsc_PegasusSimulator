#!/usr/bin/env python3
"""Regenerate the clickable outline of report.html, in place and idempotently.

    python3 add_outline.py            # from tools/ or anywhere; edits ../report.html

What it maintains (all between markers, so re-running replaces rather than duplicates):
  * an id on every <h1>/<h2>/<h3>/<h4> (slugs from the heading text, prefixed by the section number)
    and on the "Overall" summary box (#overview);
  * a static Contents block under the subtitle (works without JavaScript);
  * a slim bar fixed to the top that appears once the static block scrolls out of view: it names
    the section you are in and opens the same outline as a dropdown, with a "Top" link;
  * the CSS and the small script behind them (smooth jump, current-section highlight).
Headings are found in document order, so a new <h2>/<h3>/<h4> shows up in the outline on the next run
(three levels: h2 sections, h3 subsections, h4 parts of a subsection)
(the artifact builder runs this first).
"""
import html as H
import re
import sys

HERE = __file__.rsplit("/tools/", 1)[0]

CSS = r"""/*toc:start*/
html{scroll-behavior:smooth}
@media (prefers-reduced-motion: reduce){html{scroll-behavior:auto}}
h1,h2,h3,h4,#overview{scroll-margin-top:60px}
.nb{white-space:nowrap}
.toc{border:1px solid var(--line);border-radius:8px;background:var(--panel);padding:12px 16px;margin:14px 0}
.toc-title{font-size:12px;font-weight:600;letter-spacing:.06em;text-transform:uppercase;color:var(--ink2);margin-bottom:8px}
.toc-list,.toc-list ul{list-style:none;margin:0;padding:0}
.toc-list{columns:2 300px;column-gap:32px}
.toc-list>li{break-inside:avoid;margin:0 0 9px}
.toc-list ul{margin:3px 0 0 2px;padding-left:12px;border-left:1px solid var(--line)}
.toc-list ul li{margin:2px 0}
.toc-list a{color:var(--ink);text-decoration:none;overflow-wrap:anywhere}
.toc-list>li>a{font-weight:600}
.toc-list ul a{color:var(--ink2);font-size:13.5px}
.toc-list li.num>a{color:var(--ink);font-weight:600}
.toc-list ul ul{margin-top:2px}
.toc-list ul ul a{font-size:13px}
.toc-list a:hover{text-decoration:underline}
.toc-list a.on{color:var(--blue)}
.toc-bar{position:fixed;top:0;left:0;right:0;z-index:30;padding-top:env(safe-area-inset-top,0px);background:var(--bg);border-bottom:1px solid var(--line);transform:translateY(-110%);opacity:0;pointer-events:none;transition:transform .18s ease,opacity .18s ease}
.toc-bar.show{transform:none;opacity:1;pointer-events:auto}
@media (prefers-reduced-motion: reduce){.toc-bar{transition:none}}
.toc-row{display:flex;align-items:center;gap:12px;max-width:1040px;margin:0 auto;padding:0 16px}
.toc-float{flex:1;min-width:0}
.toc-float summary{display:flex;align-items:center;gap:12px;min-height:40px;cursor:pointer;list-style:none}
.toc-float summary::-webkit-details-marker{display:none}
.toc-btn{flex:none;display:inline-flex;align-items:center;gap:8px;font-size:13px;font-weight:600;border:1px solid var(--line);border-radius:6px;padding:3px 10px 3px 12px;background:var(--panel)}
.toc-btn::after{content:"";width:6px;height:6px;border-right:1.5px solid currentColor;border-bottom:1.5px solid currentColor;transform:rotate(45deg) translateY(-2px);transition:transform .15s}
.toc-float[open] .toc-btn::after{transform:rotate(-135deg) translateY(-1px)}
.toc-now{font-size:13px;color:var(--ink2);white-space:nowrap;overflow:hidden;text-overflow:ellipsis;min-width:0}
.toc-top{flex:none;font-size:13px;color:var(--ink2);text-decoration:none;padding:8px 0}
.toc-top:hover{text-decoration:underline}
.toc-panel{position:absolute;left:0;right:0;top:100%;background:var(--bg);border-bottom:1px solid var(--line);box-shadow:0 14px 24px rgba(0,0,0,.16);max-height:min(72vh,620px);overflow:auto}
.toc-panel-in{max-width:1040px;margin:0 auto;padding:12px 16px 14px}
.toc-float summary:focus-visible,.toc-list a:focus-visible,.toc-top:focus-visible{outline:2px solid var(--blue);outline-offset:2px}
/*toc:end*/"""

JS = r"""<script id="toc-script">
(function(){
 var bar=document.getElementById('toc-bar'),det=document.getElementById('toc-float'),now=document.getElementById('toc-now'),box=document.getElementById('contents');
 if(!bar||!det||!box){return;}
 var heads=[].slice.call(document.querySelectorAll('h1[id],h2[id],h3[id],h4[id],#overview'));
 var links=[].slice.call(document.querySelectorAll('.toc-list a'));
 var instant=function(){return (window.matchMedia&&matchMedia('(prefers-reduced-motion: reduce)').matches)||window.__instantScroll;};
 function jump(id){
  if(id==='top'){window.scrollTo({top:0,behavior:instant()?'instant':'smooth'});return;}
  var el=document.getElementById(id); if(el){el.scrollIntoView({behavior:instant()?'instant':'smooth',block:'start'});}
 }
 document.addEventListener('click',function(e){
  var a=e.target.closest&&e.target.closest('.toc-list a, .toc-top');
  if(a){e.preventDefault();jump(a.getAttribute('href').slice(1));det.open=false;return;}
  if(det.open&&!det.contains(e.target)){det.open=false;}
 });
 document.addEventListener('keydown',function(e){if(e.key==='Escape'){det.open=false;}});
 var queued=false;
 function label(el){return el.textContent.replace(/\s+/g,' ').trim();}
 function update(){
  queued=false;
  bar.classList.toggle('show',box.getBoundingClientRect().bottom<0);
  var cur=heads[0],parent=null,i;
  for(i=0;i<heads.length;i++){if(heads[i].getBoundingClientRect().top<=90){cur=heads[i];}else{break;}}
  var lv=function(el){return /^H[1-4]$/.test(el.tagName)?+el.tagName[1]:2;};
  if(lv(cur)>=3){for(i=heads.indexOf(cur);i>=0;i--){if(lv(heads[i])===lv(cur)-1){parent=heads[i];break;}}}
  now.textContent=(parent?label(parent)+'  ›  ':'')+label(cur);
  links.forEach(function(a){a.classList.toggle('on',a.getAttribute('href')==='#'+cur.id);});
 }
 function queue(){if(!queued){queued=true;requestAnimationFrame(update);}}
 window.addEventListener('scroll',queue,{passive:true});window.addEventListener('resize',queue);
 update();
})();
</script>"""


def slug(text, prefix):
    t = re.sub(r"^\d+(?:\.\d+)*\.?\s+", "", text).lower()
    t = re.sub(r"[^a-z0-9]+", "-", t).strip("-")[:44].strip("-")
    return f"{prefix}-{t}" if prefix else t


def process(src):
    # 1. drop everything this script generated earlier
    src = re.sub(r"<!--toc:start-->.*?<!--toc:end-->\n?", "", src, flags=re.S)
    src = re.sub(r"<!--tocbar:start-->.*?<!--tocbar:end-->\n?", "", src, flags=re.S)
    src = re.sub(r"/\*toc:start\*/.*?/\*toc:end\*/\n?", "", src, flags=re.S)
    src = re.sub(r'<script id="toc-script">.*?</script>\n?', "", src, flags=re.S)
    # 2. ids on headings (fresh, so renamed headings never keep a stale anchor)
    used, sections, sec, sub, prefix = set(), [], None, None, ""

    def add_id(m):
        nonlocal sec, sub, prefix
        tag, inner = m.group(1), m.group(3)
        text = H.unescape(re.sub(r"<[^>]+>", "", inner)).strip()
        if tag == "h1":
            hid = "top"
        elif tag == "h2":
            num = re.match(r"(\d+)\.", text)
            prefix = f"s{num.group(1)}" if num else "s"
            hid = slug(text, prefix)
        else:
            hid = slug(text, prefix)
        base, k = hid, 2
        while hid in used:
            hid, k = f"{base}-{k}", k + 1
        used.add(hid)
        if tag == "h2":
            sec, sub = (hid, text, []), None
            sections.append(sec)
        elif tag == "h3" and sec is not None:
            sub = (hid, text, [])
            sec[2].append(sub)
        elif tag == "h4" and sub is not None:
            sub[2].append((hid, text))
        return f'<{tag} id="{hid}">{inner}</{tag}>'

    src = re.sub(r"<(h[1234])((?:\s[^>]*)?)>(.*?)</\1>", add_id, src, flags=re.S)
    if not sections:
        raise SystemExit("no <h2> headings found")
    # 3. the Overall box gets #overview
    src = re.sub(r'<div class="verdict good"(?: id="overview")?><strong>Overall\.</strong>',
                 '<div class="verdict good" id="overview"><strong>Overall.</strong>', src, count=1)
    # 4. outline markup
    def esc(t):
        return H.escape(t, quote=False)
    items = ['<li><a href="#overview">Overview</a></li>']
    for hid, text, subs in sections:
        li = f'<li><a href="#{hid}">{esc(text)}</a>'
        if subs:
            li += "<ul>"
            for i, t, parts in subs:
                cls = ' class="num"' if re.match(r"\d+\.\d+", t) else ""   # numbered subsection, e.g. "3.1 ..."
                li += f'<li{cls}><a href="#{i}">{esc(t)}</a>'
                if parts:
                    li += "<ul>" + "".join(f'<li><a href="#{j}">{esc(u)}</a></li>' for j, u in parts) + "</ul>"
                li += "</li>"
            li += "</ul>"
        items.append(li + "</li>")
    ul = '<ul class="toc-list">' + "".join(items) + "</ul>"
    nav = (f'<!--toc:start-->\n<nav class="toc" id="contents" aria-label="Contents">'
           f'<div class="toc-title">Contents</div>{ul}</nav>\n<!--toc:end-->\n')
    bar = ('<!--tocbar:start-->\n<div class="toc-bar" id="toc-bar"><div class="toc-row">'
           '<details class="toc-float" id="toc-float"><summary><span class="toc-btn">Contents</span>'
           '<span class="toc-now" id="toc-now"></span></summary>'
           f'<div class="toc-panel"><nav class="toc-panel-in" aria-label="Contents, floating">{ul}</nav></div></details>'
           '<a class="toc-top" href="#top">Top</a></div></div>\n'
           '<noscript><style>.toc-bar{transform:none;opacity:1;pointer-events:auto}</style></noscript>\n<!--tocbar:end-->\n')
    # 5. place: bar first in <body>, contents after the subtitle, css in <style>, script last
    src = src.replace("<body>\n", "<body>\n" + bar, 1) if "<body>\n" in src else src.replace("<body>", "<body>\n" + bar, 1)
    m = re.search(r'(<h1 id="top">.*?</h1>\s*<p class="sub">.*?</p>\n)', src, re.S)
    if not m:
        raise SystemExit("could not find the title + subtitle to place the Contents block after")
    src = src[:m.end()] + nav + src[m.end():]
    src = src.replace("</style>", CSS + "\n</style>", 1)
    src = src.replace("</body>", JS + "\n</body>", 1)
    return src, sections


if __name__ == "__main__":
    path = f"{HERE}/report.html"
    out, secs = process(open(path).read())
    open(path, "w").write(out)
    n3 = sum(len(s[2]) for s in secs); n4 = sum(len(x[2]) for s in secs for x in s[2])
    print(f"outline: {len(secs)} sections, {n3} subsections, {n4} parts, written to {path}")
