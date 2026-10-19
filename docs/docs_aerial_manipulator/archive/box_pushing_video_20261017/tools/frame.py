#!/usr/bin/env python3
"""The video frame: 2560 x 1440, the Isaac viewport in the left half and a ground
station in the right half, each window scaled from its 1244 x 1400 client area
(the largest 1280:1440 rectangle inside the GNOME work area) to 1280 x 1440.
Run with PYTHONNOUSERSITE=1 /usr/bin/python3 (cv2 4.5.4 + numpy 1.21).

    frame.py LEFT.png RIGHT.png OUT.png       compose one still (the layout preview)
"""
import sys

import cv2
import numpy as np

FW, FH = 2560, 1440
PW = FW // 2


def fit(img, w=PW, h=FH):
    if img is None:
        return np.zeros((h, w, 3), np.uint8)
    if img.shape[1] == w and img.shape[0] == h:
        return img
    up = img.shape[1] < w
    return cv2.resize(img, (w, h), interpolation=cv2.INTER_CUBIC if up else cv2.INTER_AREA)


def clean_isaac(img):
    """Paint out the small "m" button Kit leaves in the viewport's bottom-left corner
    with the viewport UI hidden (1244 x 1400 capture coordinates)."""
    if img is None or img.shape[:2] != (1400, 1244):
        return img
    x0, y0, x1, y1 = 8, 1368, 52, 1400
    roi = img[y0 - 12:y1, x0:x1 + 12].copy()
    mask = np.zeros(roi.shape[:2], np.uint8)
    mask[12:, :x1 - x0] = 255
    img = img.copy()
    img[y0 - 12:y1, x0:x1 + 12] = cv2.inpaint(roi, mask, 5, cv2.INPAINT_TELEA)
    return img


def compose(left, right):
    out = np.empty((FH, FW, 3), np.uint8)
    out[:, :PW] = fit(clean_isaac(left))
    out[:, PW:] = fit(right)
    return out


if __name__ == "__main__":
    a, b, o = sys.argv[1:4]
    cv2.imwrite(o, compose(cv2.imread(a), cv2.imread(b)))
    print(o)
