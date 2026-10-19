#!/usr/bin/env python3
"""Minimal EWMH window control over libX11 (no wmctrl / xdotool / python-xlib on
shiqi-desktop). Used to lay the Isaac window and the two ground stations out for
the box-pushing video.

    xwin.py list                                  every client window: id pid x y w h title
    xwin.py find TITLE_SUBSTRING                  print the first matching window id
    xwin.py place TITLE x y w h [--undecorate]    move/resize the CLIENT area (static gravity)
    xwin.py raise TITLE                           activate (raise + focus)
    xwin.py geom TITLE                            print the client geometry: x y w h
    xwin.py drag TITLE x0 y0 x1 y1                left-button drag in client coords (XTest)
    xwin.py click TITLE x y [n]                   n left clicks (2 = double-click), client coords
"""
import ctypes
import sys
import time

X = ctypes.cdll.LoadLibrary("libX11.so.6")
c_ulong, c_long, c_int, c_void_p = ctypes.c_ulong, ctypes.c_long, ctypes.c_int, ctypes.c_void_p

X.XOpenDisplay.restype = c_void_p
X.XOpenDisplay.argtypes = [ctypes.c_char_p]
X.XDefaultRootWindow.restype = c_ulong
X.XDefaultRootWindow.argtypes = [c_void_p]
X.XInternAtom.restype = c_ulong
X.XInternAtom.argtypes = [c_void_p, ctypes.c_char_p, c_int]
X.XGetWindowProperty.argtypes = [c_void_p, c_ulong, c_ulong, c_long, c_long, c_int, c_ulong,
                                 ctypes.POINTER(c_ulong), ctypes.POINTER(c_int), ctypes.POINTER(c_ulong),
                                 ctypes.POINTER(c_ulong), ctypes.POINTER(ctypes.POINTER(ctypes.c_ubyte))]
X.XFree.argtypes = [c_void_p]
X.XSendEvent.argtypes = [c_void_p, c_ulong, c_int, c_long, c_void_p]
X.XFlush.argtypes = [c_void_p]
X.XSync.argtypes = [c_void_p, c_int]
X.XChangeProperty.argtypes = [c_void_p, c_ulong, c_ulong, c_ulong, c_int, c_int, c_void_p, c_int]
X.XGetGeometry.argtypes = [c_void_p, c_ulong, ctypes.POINTER(c_ulong), ctypes.POINTER(c_int),
                           ctypes.POINTER(c_int), ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint),
                           ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint)]
X.XTranslateCoordinates.argtypes = [c_void_p, c_ulong, c_ulong, c_int, c_int, ctypes.POINTER(c_int),
                                    ctypes.POINTER(c_int), ctypes.POINTER(c_ulong)]
X.XMoveResizeWindow.argtypes = [c_void_p, c_ulong, c_int, c_int, ctypes.c_uint, ctypes.c_uint]


class XClientMessageEvent(ctypes.Structure):
    _fields_ = [("type", c_int), ("serial", c_ulong), ("send_event", c_int), ("display", c_void_p),
                ("window", c_ulong), ("message_type", c_ulong), ("format", c_int), ("l", c_long * 5)]


class XEvent(ctypes.Union):
    _fields_ = [("xclient", XClientMessageEvent), ("pad", c_long * 24)]


D = X.XOpenDisplay(None)
if not D:
    sys.exit("cannot open the X display (DISPLAY unset?)")
# a window can close between listing and reading it: Xlib's default handler would exit
_XERR = ctypes.CFUNCTYPE(c_int, c_void_p, c_void_p)(lambda d, e: 0)
X.XSetErrorHandler(_XERR)
ROOT = X.XDefaultRootWindow(D)


def atom(name):
    return X.XInternAtom(D, name.encode(), 0)


def prop(win, name, kind="CARDINAL"):
    t, f, n, after = c_ulong(), c_int(), c_ulong(), c_ulong()
    data = ctypes.POINTER(ctypes.c_ubyte)()
    req = 0 if kind == "ANY" else atom(kind)
    if X.XGetWindowProperty(D, win, atom(name), 0, 1 << 20, 0, req, ctypes.byref(t), ctypes.byref(f),
                            ctypes.byref(n), ctypes.byref(after), ctypes.byref(data)) != 0 or not data:
        return None
    try:
        if f.value == 32:
            return [ctypes.cast(data, ctypes.POINTER(c_ulong))[i] for i in range(n.value)]
        if f.value == 8:
            return bytes(data[i] for i in range(n.value)).decode("utf-8", "replace")
        return None
    finally:
        X.XFree(data)


def clients():
    return prop(ROOT, "_NET_CLIENT_LIST", "WINDOW") or []


def title(w):
    return prop(w, "_NET_WM_NAME", "UTF8_STRING") or prop(w, "WM_NAME", "STRING") or ""


def geom(w):
    """The CLIENT area in root coordinates."""
    r, x, y = c_ulong(), c_int(), c_int()
    ww, hh, b, d = ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint()
    X.XGetGeometry(D, w, ctypes.byref(r), ctypes.byref(x), ctypes.byref(y), ctypes.byref(ww), ctypes.byref(hh),
                   ctypes.byref(b), ctypes.byref(d))
    rx, ry, ch = c_int(), c_int(), c_ulong()
    X.XTranslateCoordinates(D, w, ROOT, 0, 0, ctypes.byref(rx), ctypes.byref(ry), ctypes.byref(ch))
    return rx.value, ry.value, ww.value, hh.value


def find(sub):
    for w in clients():
        if sub.lower() in title(w).lower():
            return w
    return None


def send(win, msg, data):
    ev = XEvent()
    ev.xclient.type = 33          # ClientMessage
    ev.xclient.send_event = 1
    ev.xclient.display = D
    ev.xclient.window = win
    ev.xclient.message_type = atom(msg)
    ev.xclient.format = 32
    for i, v in enumerate(data):
        ev.xclient.l[i] = v
    X.XSendEvent(D, ROOT, 0, (1 << 20) | (1 << 19), ctypes.byref(ev))   # SubstructureRedirect|Notify
    X.XFlush(D)


def undecorate(w):
    hints = (c_long * 5)(2, 0, 0, 0, 0)     # MWM_HINTS_DECORATIONS, decorations = none
    a = atom("_MOTIF_WM_HINTS")
    X.XChangeProperty(D, w, a, a, 32, 0, hints, 5)
    X.XFlush(D)


def place(w, x, y, wd, ht, undec=False):
    # un-maximize first, or the WM keeps the maximized size
    send(w, "_NET_WM_STATE", [0, atom("_NET_WM_STATE_MAXIMIZED_VERT"), atom("_NET_WM_STATE_MAXIMIZED_HORZ"), 1, 0])
    send(w, "_NET_WM_STATE", [0, atom("_NET_WM_STATE_FULLSCREEN"), 0, 1, 0])
    if undec:
        undecorate(w)
        time.sleep(0.3)
    # gravity 10 = static (x, y = the client's own corner); flags x|y|w|h; source = pager
    send(w, "_NET_MOVERESIZE_WINDOW", [10 | (0xF << 8) | (2 << 12), x, y, wd, ht])
    X.XSync(D, 0)


def activate(w):
    send(w, "_NET_ACTIVE_WINDOW", [2, 0, 0, 0, 0])


def drag(w, x0, y0, x1, y1, steps=30):
    """A left-button drag in the window's CLIENT coordinates (XTest): moves the real pointer."""
    T = ctypes.cdll.LoadLibrary("libXtst.so.6")
    T.XTestFakeMotionEvent.argtypes = [c_void_p, c_int, c_int, c_int, c_ulong]
    T.XTestFakeButtonEvent.argtypes = [c_void_p, ctypes.c_uint, c_int, c_ulong]
    gx, gy, _, _ = geom(w)
    activate(w)
    time.sleep(0.5)
    T.XTestFakeMotionEvent(D, -1, gx + x0, gy + y0, 0)
    X.XFlush(D)
    time.sleep(0.3)
    T.XTestFakeButtonEvent(D, 1, 1, 0)
    X.XFlush(D)
    time.sleep(0.2)
    for i in range(1, steps + 1):
        T.XTestFakeMotionEvent(D, -1, gx + x0 + (x1 - x0) * i // steps, gy + y0 + (y1 - y0) * i // steps, 0)
        X.XFlush(D)
        time.sleep(0.02)
    time.sleep(0.2)
    T.XTestFakeButtonEvent(D, 1, 0, 0)
    X.XFlush(D)


def click(w, x, y, n=1):
    """n left clicks (n = 2: a double-click) in the window's CLIENT coordinates (XTest)."""
    T = ctypes.cdll.LoadLibrary("libXtst.so.6")
    T.XTestFakeMotionEvent.argtypes = [c_void_p, c_int, c_int, c_int, c_ulong]
    T.XTestFakeButtonEvent.argtypes = [c_void_p, ctypes.c_uint, c_int, c_ulong]
    gx, gy, _, _ = geom(w)
    activate(w)
    time.sleep(0.4)
    T.XTestFakeMotionEvent(D, -1, gx + x, gy + y, 0)
    X.XFlush(D)
    time.sleep(0.2)
    for _ in range(n):
        T.XTestFakeButtonEvent(D, 1, 1, 0)
        X.XFlush(D)
        time.sleep(0.04)
        T.XTestFakeButtonEvent(D, 1, 0, 0)
        X.XFlush(D)
        time.sleep(0.06)


def main(argv):
    if not argv or argv[0] == "list":
        for w in clients():
            pid = (prop(w, "_NET_WM_PID") or [0])[0]
            print(hex(w), pid, *geom(w), title(w))
        return 0
    cmd, sub = argv[0], argv[1]
    w = find(sub)
    if w is None:
        print(f"no window titled like '{sub}'", file=sys.stderr)
        return 1
    if cmd == "find":
        print(hex(w))
    elif cmd == "geom":
        print(*geom(w))
    elif cmd == "raise":
        activate(w)
    elif cmd == "drag":
        drag(w, *map(int, argv[2:6]))
    elif cmd == "click":
        click(w, int(argv[2]), int(argv[3]), int(argv[4]) if len(argv) > 4 else 1)
    elif cmd == "place":
        x, y, wd, ht = map(int, argv[2:6])
        place(w, x, y, wd, ht, "--undecorate" in argv)
        time.sleep(0.6)
        print(hex(w), *geom(w))
    else:
        print(__doc__)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
