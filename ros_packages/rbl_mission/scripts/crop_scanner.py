#!/usr/bin/env python3
"""Crop scanner: counts the fruits (tomato / pepper / cucumber / strawberry, by ripeness) of the crop row a UAV
covers with its lateral RGB-D cameras, logs them and writes a report when the row segment is done.

The greenhouse version of shelf_scanner.py (same mission states, camera choice, gz-direct cameras, output files,
rviz; this class only swaps the detector and the bookkeeping):

  - cameras: per side a colour camera (<side>_gz_topic: <topic>/image, <topic>/camera_info) and a depth camera at
    the same pose (<side>_gz_depth_topic: <topic>/depth_image), paired by stamp; read straight from gz, subscribed
    only while scanning the side facing the row (gz renders them only then).
  - detection: crop_detector.py, colour segmentation per crop + ripeness class, depth per blob, touching fruits split
    by their expected size. Detections are placed in the world frame with TF and kept if they are in the scanned row.
  - counting: per cell (cell_x along the row x cell_z of height) and ripeness, the 70th percentile of the per-frame
    counts over the frames that saw the cell well inside the view: a fruit seen in 20 frames counts once, a fruit
    swinging in the wind into the neighbour cell for a moment does not count twice.
  - ground truth: <session>/worlds/crop_truth.json (its generate.py): every fruit with rest position, ripeness and
    the aisle sides it is visible from. The report scores against the fruits visible from the scanned side.

Output, under data_dir (the session passes <session folder>/scans/<run id>/):
  <uav>_A<a>-S<s>-<L|R>_<HHMMSS>/report.{md,html,json}  counts per ripeness found / expected, recall, over-counts,
                                     height x section grid, position error, annotated frames (html)
  <uav>_A<a>-S<s>-<L|R>_<HHMMSS>/detections.csv          every detection of every frame (world position)
  inventory.csv                      one line per cell x ripeness, every row segment of the run
  index.md                           one line per row segment
  pub ~/report  (JSON summary, latched), ~/state, ~/debug_image (annotated frame, upright), ~/codes (MarkerArray:
      fruits counted, coloured like the fruit; after the report the missed ones grey at their true position)
"""
import base64
import collections
import csv
import datetime
import fcntl
import html
import json
import math
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import crop_detector  # noqa: E402
from shelf_scanner import ShelfScanner, quat_to_rot  # noqa: E402

# marker / drawing colour per (colour class) -- RGB 0..1
FRUIT_RGB = {'red': (0.85, 0.08, 0.05), 'orange': (0.95, 0.5, 0.08), 'yellow': (0.98, 0.82, 0.05),
             'green': (0.3, 0.75, 0.2), 'pale': (0.9, 0.92, 0.7)}


class CropScanner(ShelfScanner):
    def __init__(self):
        super().__init__('crop_scanner')

    # ------------------------------------------------------------------ setup
    def setup_detector(self):
        truth = self.declare_parameter('truth', '').value
        self.cell_x = float(self.declare_parameter('cell_x', 0.5).value)
        self.cell_z = float(self.declare_parameter('cell_z', 0.3).value)
        self.count_pct = float(self.declare_parameter('count_percentile', 70.0).value)
        self.view_margin = float(self.declare_parameter('view_margin', 0.08).value)   # share of the image per side
        self.row_halfwidth = float(self.declare_parameter('row_halfwidth', 0.6).value)  # [m] detections kept
        self.depth_topics = {c: self.declare_parameter(f'{c}_gz_depth_topic', '').value for c in ('left', 'right')}
        self.fruits = []
        if truth and os.path.exists(truth):
            self.fruits = json.load(open(truth))['fruits']
            self.get_logger().info(f'ground truth: {len(self.fruits)} fruits ({truth})')
        else:
            self.get_logger().warn(f'no ground truth ({truth!r}): reports without expected counts')
        self.truth = {}           # (base class: QR manifest) unused
        self.wechat_files = None
        self.rgb_buf, self.depth_buf = {}, {}

    def make_decoder(self):
        return None

    # ------------------------------------------------------------------ cameras: colour + depth, paired by stamp
    def gz_subscribe(self, cam, base):
        from gz.msgs10.image_pb2 import Image as GzImage
        dbase = self.depth_topics.get(cam) or base.replace(f'{cam}_cam', f'{cam}_depth')
        self.rgb_buf[cam], self.depth_buf[cam] = {}, {}
        self.gz_node.subscribe(self.GzCameraInfo, base + '/camera_info', lambda m, c=cam: self.on_gz_info(c, m))
        self.gz_node.subscribe(GzImage, base + '/image', lambda m, c=cam: self.on_rgb(c, m))
        self.gz_node.subscribe(GzImage, dbase + '/depth_image', lambda m, c=cam: self.on_depth(c, m))
        return [base + '/camera_info', base + '/image', dbase + '/depth_image']

    @staticmethod
    def _stamp(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nsec

    def on_rgb(self, cam, msg):
        self.rgb_buf.setdefault(cam, {})[self._stamp(msg)] = msg
        self._pair(cam)

    def on_depth(self, cam, msg):
        self.depth_buf.setdefault(cam, {})[self._stamp(msg)] = msg
        self._pair(cam)

    def _pair(self, cam):
        rb, db = self.rgb_buf.get(cam, {}), self.depth_buf.get(cam, {})
        for st in sorted(set(rb) & set(db)):
            r, d = rb.pop(st), db.pop(st)

            def frame(r=r, d=d, st=st):
                rgb = np.frombuffer(r.data, np.uint8).reshape(r.height, r.width, 3)
                dep = np.frombuffer(d.data, np.float32).reshape(d.height, d.width)
                return rgb, dep, Time(nanoseconds=st)
            self.queue_frame(cam, frame)
        for b in (rb, db):   # drop unpaired leftovers
            for st in sorted(b)[:-3]:
                b.pop(st)

    # ------------------------------------------------------------------ per scan
    def prepare(self, scan):
        """Crop, row and the ground truth of the scanned face (aisle, segment, side)."""
        face = [f for f in self.fruits if f['segment'] == scan.segment and
                any(v['aisle'] == scan.aisle and v['side'] == scan.side for v in f['visible_from'])]
        rows = collections.Counter((f['crop'], f['row_y']) for f in self.fruits if f['segment'] == scan.segment and
                                   any(v['aisle'] == scan.aisle and v['side'] == scan.side for v in f['visible_from']))
        if rows:
            (scan.crop, scan.row_y), _ = rows.most_common(1)[0]
        else:   # no ground truth: the row is 1.9 m to the scanned side of the aisle; crop unknown -> tomato classes
            scan.crop, scan.row_y = 'tomato', float('nan')
        scan.expected = [f for f in face if f['crop'] == scan.crop]
        xs = [f['x'] for f in self.fruits if f['segment'] == scan.segment and f['row_y'] == scan.row_y] or [0.0]
        scan.x0, scan.x1 = min(xs) - 0.25, max(xs) + 0.25
        scan.counts = collections.defaultdict(list)    # (cell, ripeness) -> per-frame counts
        scan.best = {}                                 # (cell, ripeness, count) -> detections of a frame with it
        scan.frames_best = {}                          # 1 m section -> (n detections, jpeg) for the report
        scan.n_det = 0

    def cell_of(self, scan, x, z):
        return (int(math.floor((x - scan.x0) / self.cell_x)), int(math.floor(z / self.cell_z)))

    # ------------------------------------------------------------------ frames
    def process(self, _dec, scan, cam, frame, info):
        if not hasattr(scan, 'crop'):
            self.prepare(scan)
        rgb, dep, stamp = frame
        K, _D = info
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        H, W = dep.shape
        dets, _ = crop_detector.detect(rgb, dep, fx, fy, cx, cy, scan.crop)
        try:
            tf = self.tf.lookup_transform(self.frame, self.cam_topics[cam][2], stamp, timeout=Duration(seconds=0.1))
        except Exception:
            return
        R = quat_to_rot(tf.transform.rotation)
        T = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        mx, my = W * self.view_margin, H * self.view_margin
        inner = lambda u, v: mx <= u <= W - mx and my <= v <= H - my  # noqa: E731
        kept = []
        for d in dets:
            xo, yo, zo = d['xyz']
            p = R @ np.array([zo, -xo, -yo]) + T     # optical -> camera link -> world
            if not (scan.x0 - 0.3 <= p[0] <= scan.x1 + 0.3 and 0.2 <= p[2] <= 3.2):
                continue
            if math.isfinite(scan.row_y) and abs(p[1] - scan.row_y) > self.row_halfwidth:
                continue   # another row, seen through this one
            d['world'] = p
            d['inner'] = inner(d['u'], d['v'])
            kept.append(d)
        # cells well inside this frame's view: their centre (in the row plane) projects into the inner image
        row_y = scan.row_y if math.isfinite(scan.row_y) else (T[1] + (1.9 if scan.side == 'left' else -1.9) - 0.0)
        per = collections.Counter()
        dets_cell = collections.defaultdict(list)
        for d in kept:
            if d['inner']:
                c = self.cell_of(scan, d['world'][0], d['world'][2])
                per[(c, d['ripeness'])] += 1
                dets_cell[(c, d['ripeness'])].append((d['world'].tolist(), d['color']))
        ncx = int(math.ceil((scan.x1 - scan.x0) / self.cell_x))
        ripeness = {r for _, r, *_ in crop_detector.CLASSES[scan.crop]}
        with self.lock:
            for i in range(ncx):
                for j in range(int(0.3 / self.cell_z), int(3.0 / self.cell_z)):
                    w = np.array([scan.x0 + (i + 0.5) * self.cell_x, row_y, (j + 0.5) * self.cell_z])
                    l = R.T @ (w - T)
                    if l[0] < 0.3:
                        continue
                    u, v = fx * -l[1] / l[0] + cx, fy * -l[2] / l[0] + cy
                    if not inner(u, v):
                        continue
                    for r in ripeness:
                        n = per.get(((i, j), r), 0)
                        scan.counts[((i, j), r)].append(n)
                        scan.best.setdefault(((i, j), r, n), dets_cell.get(((i, j), r), []))
            t_img = stamp.nanoseconds * 1e-9
            for d in kept:
                scan.rows.append([f'{t_img:.3f}', cam, scan.crop, d['ripeness'], d['color'], int(d['inner']), d['px'],
                                  f"{d['depth']:.3f}"] + [f'{v:.3f}' for v in d['world']] + [f'{v:.3f}' for v in T])
            scan.n_det += len(kept)
            scan.cams.add(cam)
            # an annotated frame per 1 m section for the report: the one with the most detections
            sec = int(math.floor((T[0] - scan.x0) / 1.0))
            if len(kept) > scan.frames_best.get(sec, (-1, None))[0]:
                vis = self.annotate(rgb, kept, scan, cam, small=True)
                scan.frames_best[sec] = (len(kept), cv2.imencode('.jpg', vis, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes())
        if self.pub_debug.get_subscription_count() > 0:
            self.publish_debug(rgb, kept, cam, stamp, scan)
        self.set_state(f'SCANNING {scan.face} {scan.crop} ({cam}, {len(kept)} fruits in view, {scan.n_det} detections)')

    def annotate(self, rgb, dets, scan, cam, small=False):
        s = 0.4 if small else self.debug_scale
        vis = cv2.resize(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        for d in dets:
            r, g, b = FRUIT_RGB.get(d['color'], (1, 1, 1))
            rad = max(4, int(math.sqrt(d['px'] / math.pi) * s * 1.3))
            cv2.circle(vis, (int(d['u'] * s), int(d['v'] * s)), rad, (0, 0, 0) if d['inner'] else (128, 128, 128), 3)
            cv2.circle(vis, (int(d['u'] * s), int(d['v'] * s)), rad, (int(b * 255), int(g * 255), int(r * 255)), 1)
        rot = {90: cv2.ROTATE_90_CLOCKWISE, -90: cv2.ROTATE_90_COUNTERCLOCKWISE, 180: cv2.ROTATE_180}.get(self.debug_rotation)
        if rot is not None:
            vis = cv2.rotate(vis, rot)
        cv2.putText(vis, f'{self.uav} {cam} | {scan.face} {scan.crop} | {len(dets)} fruits', (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(vis, f'{self.uav} {cam} | {scan.face} {scan.crop} | {len(dets)} fruits', (6, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        return vis

    def publish_debug(self, rgb, dets, cam, stamp, scan):
        vis = self.annotate(rgb, dets, scan, cam)
        msg = Image()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.cam_topics[cam][2]
        msg.height, msg.width = vis.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = vis.tobytes()
        self.pub_debug.publish(msg)

    # ------------------------------------------------------------------ counts
    def tally(self, scan):
        """(cell, ripeness) -> (count, detections of a frame with that count)."""
        out = {}
        with self.lock:
            for key, ns in scan.counts.items():
                n = int(round(np.percentile(ns, self.count_pct))) if ns else 0
                # the detections of a frame that had exactly n (or the closest count seen)
                have = sorted({c for c in ns}, key=lambda c: abs(c - n))
                dets = next((scan.best[(key[0], key[1], c)] for c in have if (key[0], key[1], c) in scan.best), [])
                out[key] = (n, dets)
        return out

    def marker_array(self, scan, missing=()):
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        if not hasattr(scan, 'crop'):
            return arr
        now = self.get_clock().now().to_msg()
        r_across = crop_detector.SIZE[scan.crop][0]
        k = 0
        for (cell, ripe), (n, dets) in self.tally(scan).items():
            for pos, color in dets[:n] if n else []:
                m = Marker(type=Marker.SPHERE, ns=f'{scan.face} {scan.crop} {ripe}', id=k, action=Marker.ADD)
                m.header.frame_id, m.header.stamp = self.frame, now
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 2.4 * r_across
                r, g, b = FRUIT_RGB.get(color, (1, 1, 1))
                m.color = ColorRGBA(r=r, g=g, b=b, a=0.95)
                arr.markers.append(m)
                k += 1
        for f in missing:
            m = Marker(type=Marker.SPHERE, ns=f'{scan.face} missed', id=k, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = self.frame, now
            m.pose.position.x, m.pose.position.y, m.pose.position.z = f['x'], f['y'], f['z']
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 2.0 * max(f['r'])
            m.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.6)
            arr.markers.append(m)
            k += 1
        return arr

    # ------------------------------------------------------------------ report
    def finish_log(self, scan, s, path):
        parts = ', '.join(f"{r} {v['found']}/{v['expected']}" for r, v in s['by_ripeness'].items())
        return (f"REPORT {scan.face} {s['crop']}: {parts}; per 1 m recall {s['section_recall_pct']}% precision "
                f"{s['section_precision_pct']}%, per cell recall {s['recall_pct']}%, "
                f"mean position error {s['position_error_mean']} m -> {path}/report.html")

    def write_report(self, scan):
        if not hasattr(scan, 'crop'):
            self.prepare(scan)
        tal = self.tally(scan)
        name = f'{self.uav}_{scan.face}_{scan.wall_start.strftime("%H%M%S")}'
        out = os.path.join(self.data_dir, name)
        os.makedirs(out, exist_ok=True)
        exp = collections.defaultdict(list)
        for f in scan.expected:
            exp[(self.cell_of(scan, f['x'], f['z']), f['ripeness'])].append(f)
        keys = set(exp) | {k for k, (n, _) in tal.items() if n}
        by_r = collections.defaultdict(lambda: dict(expected=0, found=0, matched=0, over=0))
        cells, errs, missing = [], [], []
        for key in sorted(keys):
            e = exp.get(key, [])
            n, dets = tal.get(key, (0, []))
            m = min(n, len(e))
            br = by_r[key[1]]
            br['expected'] += len(e)
            br['found'] += n
            br['matched'] += m
            br['over'] += max(0, n - len(e))
            # positions: greedy nearest pairs detection <-> truth (rest pose; the wind moves them a little)
            free = list(e)
            for pos, _c in dets[:n]:
                if not free:
                    break
                j = min(range(len(free)), key=lambda j: math.dist(pos, (free[j]['x'], free[j]['y'], free[j]['z'])))
                dist = math.dist(pos, (free[j]['x'], free[j]['y'], free[j]['z']))
                if dist < 0.2:
                    errs.append(dist)
                    free.pop(j)
            missing += free[:max(0, len(e) - n)] if len(e) > n else []
            cells.append(dict(cell_x=key[0][0], cell_z=key[0][1], x=round(scan.x0 + (key[0][0] + 0.5) * self.cell_x, 2),
                              z=round((key[0][1] + 0.5) * self.cell_z, 2), ripeness=key[1], expected=len(e), found=n))
        # per 1 m section along the row, all heights: a fruit swinging across a height-cell border is no miss there
        sec = collections.defaultdict(lambda: [0, 0])
        per_m = max(1, int(round(1.0 / self.cell_x)))
        for c in cells:
            k = (c['cell_x'] // per_m, c['ripeness'])
            sec[k][0] += c['found']
            sec[k][1] += c['expected']
        sec_match = sum(min(f, e) for f, e in sec.values())
        n_exp = sum(v['expected'] for v in by_r.values())
        n_match = sum(v['matched'] for v in by_r.values())
        n_found = sum(v['found'] for v in by_r.values())
        summary = dict(
            uav=self.uav, row_segment=scan.face, aisle=scan.aisle, segment=scan.segment, side=scan.side,
            crop=scan.crop, started=scan.wall_start.isoformat(timespec='seconds'),
            sim_time=[round(scan.t_start, 2), round(scan.t_end or scan.t_start, 2)],
            duration_s=round((scan.t_end or scan.t_start) - scan.t_start, 1), end_state=scan.end_state,
            frames=scan.frames, frames_decoded=scan.frames - scan.frames_dropped, cameras=sorted(scan.cams),
            by_ripeness={r: dict(expected=v['expected'], found=v['found']) for r, v in sorted(by_r.items())},
            expected=n_exp, found=n_found, recall_pct=round(100 * n_match / n_exp, 1) if n_exp else None,
            precision_pct=round(100 * n_match / n_found, 1) if n_found else None,
            section_recall_pct=round(100 * sec_match / n_exp, 1) if n_exp else None,
            section_precision_pct=round(100 * sec_match / n_found, 1) if n_found else None,
            overcount=sum(v['over'] for v in by_r.values()),
            position_error_mean=round(float(np.mean(errs)), 3) if errs else None,
            hidden_by_leaves=sum(1 for f in self.fruits if f['segment'] == scan.segment and f['crop'] == scan.crop and
                                 f['row_y'] == scan.row_y and not any(v['aisle'] == scan.aisle and v['side'] == scan.side
                                                                       for v in f['visible_from'])),
            report_dir=out)
        summary['missing'] = missing
        with open(os.path.join(out, 'report.json'), 'w') as f:
            json.dump(dict(summary={k: v for k, v in summary.items() if k != 'missing'},
                           missed_fruits=[m['id'] for m in missing], cells=cells), f, indent=1)
        with open(os.path.join(out, 'detections.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['sim_time', 'camera', 'crop', 'ripeness', 'color', 'counted', 'px', 'depth_m', 'x', 'y', 'z',
                        'cam_x', 'cam_y', 'cam_z'])
            w.writerows(scan.rows)
        grid = self.grid(scan, cells)
        with open(os.path.join(out, 'report.md'), 'w') as f:
            f.write(self.markdown(summary, grid))
        with open(os.path.join(out, 'report.html'), 'w') as f:
            f.write(self.html(summary, grid, scan))
        self.append_crop_run_files(summary, cells, out)
        return out, summary

    def grid(self, scan, cells):
        """height band (top first) -> 1 m section -> [found, expected] (all ripeness classes)."""
        g = collections.defaultdict(lambda: collections.defaultdict(lambda: [0, 0]))
        per_m = max(1, int(round(1.0 / self.cell_x)))
        for c in cells:
            sec, band = c['cell_x'] // per_m, c['cell_z']
            g[band][sec][0] += c['found']
            g[band][sec][1] += c['expected']
        secs = sorted({s for b in g.values() for s in b})
        bands = sorted(g, reverse=True)
        return secs, bands, g

    def markdown(self, s, grid):
        secs, bands, g = grid
        L = [f"# Crop report {s['row_segment']} {s['crop']} ({s['uav']})", '',
             f"Aisle {s['aisle']}, segment {s['segment']}, {s['side']} row ({s['crop']}). Started {s['started']}, "
             f"{s['duration_s']} s sim time, {s['frames_decoded']}/{s['frames']} frames analysed "
             f"({', '.join(s['cameras']) or 'no'} camera). End: `{s['end_state']}`", '',
             '| ripeness | found | expected (visible) |', '|---|---|---|']
        for r, v in s['by_ripeness'].items():
            L.append(f"| {r} | {v['found']} | {v['expected']} |")
        L += ['', f"- **Total** {s['found']} found / {s['expected']} expected",
              f"- **Per 1 m section** (all heights): recall {s['section_recall_pct']}%, precision {s['section_precision_pct']}%",
              f"- **Per cell** ({self.cell_x} m x {self.cell_z} m): recall {s['recall_pct']}%, precision {s['precision_pct']}%,"
              f" over-counted {s['overcount']}",
              f"- Mean position error (counted fruits vs. their rest position): {s['position_error_mean']} m",
              f"- Fruits of this row hidden by leaves from this side (not expected): {s['hidden_by_leaves']}", '']
        if secs:
            L += ['Found / expected per height band (rows, top first) and 1 m section along the row (columns):', '',
                  '| z \\ x | ' + ' | '.join(f'{s_}' for s_ in secs) + ' |', '|---|' + '---|' * len(secs)]
            for b in bands:
                L.append(f'| {b * self.cell_z:.1f}–{(b + 1) * self.cell_z:.1f} m | ' +
                         ' | '.join(f'{g[b][x][0]}/{g[b][x][1]}' if g[b][x] != [0, 0] else '·' for x in secs) + ' |')
        return '\n'.join(L) + '\n'

    def html(self, s, grid, scan):
        secs, bands, g = grid
        esc = html.escape

        def cls(f, e):
            if e == 0:
                return 'over' if f else 'na'
            return 'ok' if f == e else ('over' if f > e else ('miss' if f < 0.75 * e else 'part'))
        rows = ''.join('<tr><th>%.1f–%.1f</th>' % (b * self.cell_z, (b + 1) * self.cell_z) +
                       ''.join(f"<td class='{cls(*g[b][x])}'>{g[b][x][0]}/{g[b][x][1]}</td>" if g[b][x] != [0, 0]
                               else "<td class='na'>·</td>" for x in secs) + '</tr>' for b in bands)
        rip = ''.join(f"<div class='kpi'><b>{v['found']}/{v['expected']}</b>{esc(r)}</div>" for r, v in s['by_ripeness'].items())
        frames = ''.join(f"<img src='data:image/jpeg;base64,{base64.b64encode(j).decode()}'>"
                         for _, (_, j) in sorted(scan.frames_best.items()) if j)
        return f'''<!doctype html><html><head><meta charset="utf-8"><title>{esc(s['row_segment'])} {esc(s['crop'])}</title>
<style>
:root{{--bg:#fff;--fg:#1d1d1f;--mut:#6e6e73;--ok:#e3f6ea;--part:#fff4d6;--miss:#fce8e6;--over:#e6efff;--line:#e5e5ea}}
@media (prefers-color-scheme:dark){{:root{{--bg:#151517;--fg:#f2f2f7;--mut:#a1a1a6;--ok:#13361f;--part:#3a2f10;--miss:#3d1714;--over:#14244a;--line:#2c2c2e}}}}
body{{font:14px/1.45 system-ui,sans-serif;background:var(--bg);color:var(--fg);margin:24px auto;max-width:1150px;padding:0 16px}}
h1{{font-size:22px;margin:0 0 4px}} .mut{{color:var(--mut)}} .kpis{{display:flex;gap:12px;flex-wrap:wrap;margin:16px 0}}
.kpi{{border:1px solid var(--line);border-radius:10px;padding:10px 14px;min-width:120px}} .kpi b{{font-size:22px;display:block}}
table{{border-collapse:collapse;margin:12px 0;font-size:12px}} td,th{{border:1px solid var(--line);padding:4px 6px;text-align:center}}
td.ok{{background:var(--ok)}} td.part{{background:var(--part)}} td.miss{{background:var(--miss)}} td.over{{background:var(--over)}}
.frames{{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:8px}} .frames img{{width:100%;border-radius:6px}}
.wrap{{overflow-x:auto}}
</style></head><body>
<h1>{esc(s['row_segment'])} — {esc(s['crop'])} ({esc(s['uav'])})</h1>
<div class="mut">Aisle {s['aisle']}, segment {s['segment']}, {s['side']} row · started {esc(s['started'])} · {s['duration_s']} s sim
time · {s['frames_decoded']}/{s['frames']} frames · {esc(s['end_state'])}</div>
<div class="kpis">{rip}
<div class="kpi"><b>{s['found']}/{s['expected']}</b>total</div>
<div class="kpi"><b>{s['section_recall_pct']}% / {s['section_precision_pct']}%</b>recall / precision per 1 m</div>
<div class="kpi"><b>{s['recall_pct']}% / {s['precision_pct']}%</b>recall / precision per cell</div>
<div class="kpi"><b>{s['position_error_mean']} m</b>mean position error</div></div>
<p class="mut">found / expected per height band [m] (rows) and 1 m section along the row (columns); expected = fruits of the
row visible from this aisle ({s['hidden_by_leaves']} more are hidden by leaves). Green exact, yellow &lt;100%, red &lt;75%, blue over-counted.</p>
<div class="wrap"><table><tr><th>z \\ x</th>{''.join(f'<th>{x}</th>' for x in secs)}</tr>{rows}</table></div>
<h2>Frames</h2><p class="mut">per 1 m along the row the frame with the most detections (black ring: counted, grey: at the image edge, not counted)</p>
<div class="frames">{frames}</div>
</body></html>
'''

    def append_crop_run_files(self, s, cells, out):
        os.makedirs(self.data_dir, exist_ok=True)
        with open(os.path.join(self.data_dir, 'inventory.csv'), 'a', newline='') as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            w = csv.writer(f)
            if f.tell() == 0:
                w.writerow(['started', 'uav', 'row_segment', 'crop', 'x', 'z', 'ripeness', 'found', 'expected'])
            for c in cells:
                w.writerow([s['started'], s['uav'], s['row_segment'], s['crop'], c['x'], c['z'], c['ripeness'],
                            c['found'], c['expected']])
            fcntl.flock(f, fcntl.LOCK_UN)
        with open(os.path.join(self.data_dir, 'index.md'), 'a') as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            if f.tell() == 0:
                f.write('# Crop scans\n\n| started | uav | row segment | crop | by ripeness (found/expected) | recall | '
                        'precision | report |\n|---|---|---|---|---|---|---|---|\n')
            rel = os.path.relpath(out, self.data_dir)
            rip = ', '.join(f"{r} {v['found']}/{v['expected']}" for r, v in s['by_ripeness'].items())
            f.write(f"| {s['started']} | {s['uav']} | {s['row_segment']} | {s['crop']} | {rip} | {s['recall_pct']}% | "
                    f"{s['precision_pct']}% | [{rel}]({rel}/report.md) |\n")
            fcntl.flock(f, fcntl.LOCK_UN)


def main():
    rclpy.init()
    node = CropScanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
