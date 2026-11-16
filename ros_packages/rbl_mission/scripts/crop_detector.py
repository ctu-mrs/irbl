"""Colour-segmentation fruit detector for the greenhouse crops (imported by crop_scanner.py).

One RGB frame + its depth image (same pose and intrinsics) -> fruit detections with pixel position, depth and
size. Per crop, a few colour classes (HSV, OpenCV units: H 0..180, S / V 0..255), measured on renders of
tmux/px4_sim_ugv_uav_greenhouse_dynamic (its generate.py gives the green fruits their own hue, see there):

  tomato      red H<=8|>=170 -> ripe, orange H 12..26 (S>=150: the soil strip has that hue at S~100) -> turning,
              green H 32..41 -> unripe (leaves / stems are H 45..59)
  pepper      red / orange / yellow -> ripe, green H 70..86 -> unripe
  cucumber    H 70..86
  strawberry  red -> ripe, pale H 32..42 / S 18..50 / bright -> unripe (white slabs and gutters: S < 15)

Per colour class: mask -> opening -> connected components. A component's depth is the median depth of its
pixels; at that depth a fruit of the crop's size covers a known area, so a component of k times that area is k
touching fruits (tomato trusses), split by k-means on its pixel coordinates.
"""
import math

import cv2
import numpy as np

# crop -> list of colour classes (name, ripeness, [(h_lo, h_hi), ...], s_lo, s_hi, v_lo)
CLASSES = {
    'tomato': [('red', 'ripe', [(0, 8), (170, 180)], 120, 255, 50),
               ('orange', 'turning', [(12, 26)], 150, 255, 50),
               ('green', 'unripe', [(32, 41)], 90, 255, 50)],
    'pepper': [('red', 'ripe', [(0, 8), (170, 180)], 120, 255, 50),
               ('orange', 'ripe', [(12, 22)], 150, 255, 50),
               ('yellow', 'ripe', [(23, 32)], 150, 255, 50),
               ('green', 'unripe', [(70, 86)], 90, 255, 40)],
    'cucumber': [('green', 'ripe', [(70, 86)], 90, 255, 40)],
    'strawberry': [('red', 'ripe', [(0, 8), (168, 180)], 120, 255, 50),
                   ('pale', 'unripe', [(32, 42)], 18, 50, 150)],
}
# nominal fruit size: (radius across, radius along the long axis) [m]
SIZE = {'tomato': (0.031, 0.031), 'pepper': (0.040, 0.050), 'cucumber': (0.022, 0.125), 'strawberry': (0.015, 0.022)}
MAX_SPLIT = 8
AREA_FACTOR = 0.7    # one fruit covers this share of its full ellipse (leaves / neighbours hide the rest)
SPLIT_ROUND = 0.35   # k = floor(area / one fruit + SPLIT_ROUND) (tuned on renders: fewer double counts than rounding)
# long fruits: a leaf across one cuts it into two blobs -> same-colour detections closer than this [m] are one fruit
MERGE_3D = {'cucumber': 0.10}
# round fruits: one peak of the blob's distance transform per fruit (independent of how much of each fruit leaves
# hide, unlike splitting by area); long ones (cucumbers): by area
SPLIT_BY_PEAKS = set()   # tried {'tomato', 'pepper', 'strawberry'}: not better on renders, untested in flight
PEAK_MIN = 0.45      # a peak must be at least this share of the expected fruit radius [px] deep in the blob


def class_masks(rgb, crop):
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    out = []
    for name, ripeness, hues, s_lo, s_hi, v_lo in CLASSES[crop]:
        m = np.zeros(h.shape, bool)
        for lo, hi in hues:
            m |= (h >= lo) & (h <= hi)
        m &= (s >= s_lo) & (s <= s_hi) & (v >= v_lo)
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        out.append((name, ripeness, m))
    return out


def split_by_peaks(comp, st, pts, r_px):
    """Fruits of one blob = local maxima of its distance transform at least PEAK_MIN * r_px deep and r_px apart;
    every pixel goes to its nearest peak. -> [(centre (u, v), pixels)]"""
    x, y, w, h = st[cv2.CC_STAT_LEFT], st[cv2.CC_STAT_TOP], st[cv2.CC_STAT_WIDTH], st[cv2.CC_STAT_HEIGHT]
    sub = np.pad(comp[y:y + h, x:x + w].astype(np.uint8), 1)
    dist = cv2.distanceTransform(sub, cv2.DIST_L2, 5)
    k = max(3, int(r_px * 0.9) | 1)
    peaks = (dist >= cv2.dilate(dist, np.ones((k, k), np.uint8))) & (dist >= PEAK_MIN * r_px)
    py, px = np.nonzero(peaks)
    if len(px) == 0:
        return [(pts.mean(0), pts)]
    # merge plateau pixels / peaks closer than ~one radius (strongest first)
    order = np.argsort(-dist[py, px])
    cen = []
    for j in order:
        c = np.array([px[j] - 1 + x, py[j] - 1 + y], np.float32)
        if all(np.linalg.norm(c - q) > 0.9 * r_px for q in cen):
            cen.append(c)
        if len(cen) >= MAX_SPLIT:
            break
    if len(cen) == 1:
        return [(pts.mean(0), pts)]
    C = np.array(cen)
    lbl = np.argmin(((pts[:, None, :] - C[None]) ** 2).sum(-1), axis=1)
    return [(C[j], pts[lbl == j]) for j in range(len(C)) if (lbl == j).any()]


def detect(rgb, depth, fx, fy, cx, cy, crop, max_depth=3.0):
    """-> list of dict(color, ripeness, u, v, depth, px, xyz (optical frame, fruit centre)), and the masks."""
    r_across, r_long = SIZE[crop]
    dets = []
    masks = class_masks(rgb, crop)
    for name, ripeness, m in masks:
        n, lab, stats, _ = cv2.connectedComponentsWithStats(m, connectivity=8)
        for i in range(1, n):
            area = stats[i, cv2.CC_STAT_AREA]
            if area < 12:
                continue
            ys, xs = np.nonzero(lab == i)
            d = depth[ys, xs]
            d = d[np.isfinite(d) & (d > 0.05)]
            if len(d) < 3:
                continue
            dz = float(np.median(d))
            if dz > max_depth:
                continue
            # area of one fruit at that depth (a bit less than the full disc: leaves cover parts of most fruits)
            a1 = math.pi * (fx * r_across / dz) * (fx * r_long / dz) * AREA_FACTOR
            if area < 0.15 * a1:
                continue   # a speck: a fruit far behind, a flower, a highlight
            k = int(min(MAX_SPLIT, max(1, math.floor(area / a1 + SPLIT_ROUND))))
            pts = np.stack([xs, ys], 1).astype(np.float32)
            if crop in SPLIT_BY_PEAKS:
                groups = split_by_peaks(lab == i, stats[i], pts, fx * r_across / dz)
            elif k > 1 and len(pts) > 4 * k:
                _, lbl, cen = cv2.kmeans(pts, k, None, (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.5),
                                         2, cv2.KMEANS_PP_CENTERS)
                groups = [(cen[j], pts[lbl.ravel() == j]) for j in range(k)]
            else:
                groups = [(pts.mean(0), pts)]
            for (u, v), gp in groups:
                gd = depth[gp[:, 1].astype(int), gp[:, 0].astype(int)]
                gd = gd[np.isfinite(gd) & (gd > 0.05)]
                dd = float(np.median(gd)) if len(gd) else dz
                z = dd + r_across   # the depth is the fruit's surface: its centre is one radius further
                xyz = ((u - cx) / fx * z, (v - cy) / fy * z, z)
                dets.append(dict(color=name, ripeness=ripeness, u=float(u), v=float(v), depth=dd, px=int(len(gp)),
                                 xyz=xyz))
    gap = MERGE_3D.get(crop)
    if gap:
        merged = []
        for d in sorted(dets, key=lambda d: -d['px']):
            for m in merged:
                if m['color'] == d['color'] and math.dist(m['xyz'], d['xyz']) < gap:
                    w = m['px'] + d['px']
                    m['xyz'] = tuple((a * m['px'] + b * d['px']) / w for a, b in zip(m['xyz'], d['xyz']))
                    m['u'], m['v'] = (m['u'] * m['px'] + d['u'] * d['px']) / w, (m['v'] * m['px'] + d['v'] * d['px']) / w
                    m['px'] = w
                    break
            else:
                merged.append(dict(d))
        dets = merged
    return dets, masks
