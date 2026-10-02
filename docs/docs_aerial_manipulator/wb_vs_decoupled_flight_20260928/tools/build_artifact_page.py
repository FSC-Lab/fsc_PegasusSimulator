#!/usr/bin/env python3
"""Build the publishable artifact page from report.html.

    python3 build_artifact_page.py <out.html>

The Artifact tool wraps the file in its own <!doctype><html><head><body>, so this keeps
only <title>, the font link, <style> and the body content of report.html, after refreshing the
outline (tools/add_outline.py). All layout
rules live in report.html itself; nothing is added here, so the repo copy and the
published page cannot drift apart.
"""
import re, sys
here = __file__.rsplit("/tools/", 1)[0]
sys.path.insert(0, f"{here}/tools")
import add_outline
# keep the outline in sync with the headings before every build (idempotent)
src, _ = add_outline.process(open(f"{here}/report.html").read())
open(f"{here}/report.html", "w").write(src)
title = re.search(r"<title>(.*?)</title>", src, re.S).group(1)
link = re.search(r'<link rel="stylesheet" href="https://fonts\.googleapis\.com[^>]*>', src).group(0)
style = re.search(r"<style>.*?</style>", src, re.S).group(0)
body = re.search(r"<body>(.*)</body>", src, re.S).group(1)
open(sys.argv[1], "w").write(f"<title>{title}</title>\n{link}\n{style}\n{body}")
print("wrote", sys.argv[1], len(body), "chars of body")
