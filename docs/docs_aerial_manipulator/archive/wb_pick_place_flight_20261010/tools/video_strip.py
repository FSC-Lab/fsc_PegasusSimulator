"""Four frames of the flight video around the set-down, cropped to the place pillar, 2 x 2 -> ../figures/setdown_strip.jpg.
The video (VID_20261010_190704.mp4, 20 fps) runs 14.0 +- 0.05 s ahead of the bag clock: the basket leaves the hat's rim at
87.6 s in the video and at 73.6 s in the bag, and passes 0.80 m at 88.2 s / 74.2 s.
usage: PYTHONNOUSERSITE=1 /usr/bin/python3 -I video_strip.py <video> <out.jpg>"""
import sys, cv2, numpy as np
OFFSET = 14.0
FR = [(72.70, "72.7 s  touchdown"), (73.30, "73.3 s  flat on the hat"), (73.60, "73.6 s  dragged over the rim"), (74.20, "74.2 s  falling")]
X0, X1, Y0, Y1 = 1130, 1690, 470, 820
c = cv2.VideoCapture(sys.argv[1]); tiles = []
for tb, lab in FR:
    c.set(cv2.CAP_PROP_POS_MSEC, 1000 * (tb + OFFSET)); ok, f = c.read()
    f = cv2.convertScaleAbs(f[Y0:Y1, X0:X1], alpha=1.5, beta=18)
    f = cv2.resize(f, (640, 400), interpolation=cv2.INTER_CUBIC)
    cv2.rectangle(f, (0, 0), (640, 34), (0, 0, 0), -1)
    cv2.putText(f, lab, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (245, 245, 242), 2, cv2.LINE_AA)
    tiles.append(f)
gap = np.zeros((400, 8, 3), np.uint8)
rows = [np.hstack([tiles[0], gap, tiles[1]]), np.hstack([tiles[2], gap, tiles[3]])]
img = np.vstack([rows[0], np.zeros((8, rows[0].shape[1], 3), np.uint8), rows[1]])
cv2.imwrite(sys.argv[2], img, [cv2.IMWRITE_JPEG_QUALITY, 80]); print("wrote", sys.argv[2], img.shape)
