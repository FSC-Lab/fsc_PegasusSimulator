import json, os
H = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(H, "..")
src = open(os.path.join(R, "report_src.html")).read()
data = json.load(open(os.path.join(R, "analysis", "report_data.json")))
out = src.replace("/*__DATA__*/null", json.dumps(data, separators=(",", ":")))
open(os.path.join(R, "report.html"), "w").write(out)
print("report.html", len(out) // 1024, "kB")
