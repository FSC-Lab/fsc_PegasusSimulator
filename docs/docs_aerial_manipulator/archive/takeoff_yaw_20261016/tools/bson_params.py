import struct, sys, re
b = open(sys.argv[1],'rb').read()
n = struct.unpack('<i', b[:4])[0]; i = 4; out = {}
while i < n-1:
    t = b[i]; i += 1
    j = b.index(b'\0', i); k = b[i:j].decode(); i = j+1
    if t == 0x10: v = struct.unpack('<i', b[i:i+4])[0]; i += 4
    elif t == 0x01: v = struct.unpack('<d', b[i:i+8])[0]; i += 8
    elif t == 0x12: v = struct.unpack('<q', b[i:i+8])[0]; i += 8
    elif t == 0x08: v = b[i]; i += 1
    else: raise SystemExit(f"type {t:#x} at {k}")
    out[k] = v
pat = re.compile(sys.argv[2])
for k in sorted(out):
    if pat.search(k): print(k, out[k])
