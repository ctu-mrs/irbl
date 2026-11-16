#!/usr/bin/env python3
"""Shelf code scanner: reads the QR labels of the shelf a UAV is covering with its lateral cameras, logs
every read and writes a report when the shelf is done.

Driven by the UAV's warehouse_inspection_node state (state_topic, latched std_msgs/String):
  COVERAGE: scanning the aisle <a> segment <s> <left|right> shelf   -> a new shelf scan starts
  COVERAGE / RETURN                                                 -> scanning
  DONE: ... / ABORTED: ...                                          -> cameras off, report written

Cameras: straight from gz-transport (<left|right>_gz_topic, so a camera is rendered only while subscribed:
ros_gz_bridge's lazy mode keeps its gz subscription) or ROS topics (<left|right>_image / _info).
Only the camera that currently faces the shelf is subscribed (chosen from TF: the camera whose view axis
points most towards the shelf face, re-checked at 2 Hz; the UAV's heading changes along the shelf), and
none outside a scan.

Each frame is decoded in a worker thread (dropped if the previous one is still being decoded): OpenCV's
WeChat QR decoder on overlapping tiles (tile / tile_stride [px]; on the whole 1600 x 1200 frame it misses
the small codes). Each code's 4 corners + its known size (manifest qr_size) give its 3D position by
solvePnP (IPPE_SQUARE), brought into the world frame with TF at the image stamp.

Codes are matched against the ground-truth manifest (worlds/shelf_codes.json of the session, written by its
generate.py): LOC:A<a>-S<s>-<L|R>-B<bay>-L<level> location labels and CTN-<id>/SKU-<sku> carton labels,
with their face (aisle, segment, side), bay, level and position. Codes of other faces (seen through the open
racks) are counted apart. Without a manifest every code is reported, without expected / missing lists.

Output, under data_dir (the session passes <session folder>/scans/<run id>/; default ~/robogistic_scans/<time>/):
  <uav>_A<a>-S<s>-<L|R>_<HHMMSS>/report.{md,html,json}   per shelf: found / missing per bay x level,
                                    cartons with their bay / level, position errors, thumbnails (html)
  <uav>_A<a>-S<s>-<L|R>_<HHMMSS>/detections.csv           every single read
  inventory.csv                     one line per code read, every shelf of the run (all UAVs append)
  index.md                          one line per shelf report
  pub ~/report  std_msgs/String (latched)  JSON summary of the last shelf
  pub ~/state   std_msgs/String (latched)  IDLE / SCANNING <shelf> (<camera>, <n> codes) / REPORT <path>
  pub ~/debug_image  sensor_msgs/Image  the frame being decoded, upright (debug_rotation), each code boxed + its text:
                     green location, orange carton, grey other shelf. Only while scanning and only with a subscriber
  pub ~/codes   visualization_msgs/MarkerArray (latched)  codes read at their estimated world position (green / orange,
                     with their text); after the report also the missing ones, red, at their true position
"""
import base64
import csv
import datetime
import fcntl
import html
import json
import math
import os
import queue
import re
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener

SHELF_RE = re.compile(r'aisle (\d+) segment (\d+) (left|right)')
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST)


def quat_to_rot(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class ShelfScan:
    """Everything read during one shelf's coverage."""

    def __init__(self, aisle, segment, side, t_start, wall_start):
        self.aisle, self.segment, self.side = aisle, segment, side
        self.face = f'A{aisle}-S{segment}-{side[0].upper()}'
        self.t_start, self.t_end, self.wall_start = t_start, None, wall_start
        self.codes = {}          # code -> dict(n, first, last, positions [], best_px, thumb (jpg bytes), cams set)
        self.frames = 0
        self.frames_dropped = 0
        self.cams = set()
        self.rows = []           # detections.csv rows
        self.end_state = ''


class ShelfScanner(Node):
    """Also the base of crop_scanner.py (greenhouse fruit counting): it overrides setup_detector / make_decoder /
    gz_subscribe / process / write_report / marker_array / publish_debug / finish_log and keeps the rest (mission
    state, camera choice, gz-direct cameras, report files, rviz)."""

    def __init__(self, name='shelf_scanner'):
        super().__init__(name)
        self.uav = self.declare_parameter('uav', 'uav1').value
        self.data_dir = os.path.expanduser(self.declare_parameter(
            'data_dir', os.path.join('~', 'robogistic_scans', datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))).value)
        state_topic = self.declare_parameter('state_topic', f'/{self.uav}/warehouse_inspection/state').value
        self.frame = self.declare_parameter('world_frame', 'world').value
        self.cam_topics = {c: (self.declare_parameter(f'{c}_image', f'/{self.uav}/sensors/{c}_cam/image').value,
                               self.declare_parameter(f'{c}_info', f'/{self.uav}/sensors/{c}_cam/camera_info').value,
                               self.declare_parameter(f'{c}_frame', f'{self.uav}/{c}_cam_link').value)
                           for c in ('left', 'right')}
        # gz sensor topic of each camera (/world/<w>/model/<m>/link/<c>_cam_link/sensor/<c>_cam): set -> the
        # camera is read straight from gz-transport (<topic>/image, <topic>/camera_info), subscribed only while
        # needed, so gz renders it only then. ros_gz_bridge's lazy:=true keeps its gz subscription and with it
        # the rendering. Empty: the ROS topics above (<c>_image / <c>_info).
        self.gz_topics = {c: self.declare_parameter(f'{c}_gz_topic', '').value for c in ('left', 'right')}
        self.gz_node = None
        if any(self.gz_topics.values()):
            from gz.transport13 import Node as GzNode
            from gz.msgs10.camera_info_pb2 import CameraInfo as GzCameraInfo
            from gz.msgs10.image_pb2 import Image as GzImage
            self.gz_node, self.GzImage, self.GzCameraInfo = GzNode(), GzImage, GzCameraInfo
        self.tile = int(self.declare_parameter('tile', 400).value)
        self.stride = int(self.declare_parameter('tile_stride', 300).value)
        self.min_facing = float(self.declare_parameter('min_facing', 0.5).value)   # cos of view axis vs face normal
        self.max_range = float(self.declare_parameter('max_range', 4.0).value)      # [m] farther reads: ignored
        self.thumb_px = int(self.declare_parameter('thumb_px', 160).value)
        self.setup_detector()
        self.tf = Buffer(cache_time=Duration(seconds=30))
        self.init_ros(state_topic)

    def setup_detector(self):
        # QR: WeChat CNN detector + super-resolution (rbl_mission share/models/wechat_qrcode): full-frame decoding
        # with the codes' real corners. Without them: tiled decoding, corners from cv2.QRCodeDetector where it finds
        # them. And the code manifest (ground truth).
        """QR: WeChat models + the code manifest."""
        manifest = self.declare_parameter('manifest', '').value
        mdir = self.declare_parameter('wechat_model_dir', '').value
        if not mdir:
            try:
                from ament_index_python.packages import get_package_share_directory
                mdir = os.path.join(get_package_share_directory('rbl_mission'), 'models', 'wechat_qrcode')
            except Exception:
                mdir = ''
        files = [os.path.join(mdir, f) for f in ('detect.prototxt', 'detect.caffemodel', 'sr.prototxt', 'sr.caffemodel')]
        self.wechat_files = files if mdir and all(os.path.exists(f) for f in files) else None
        if self.wechat_files:
            self.get_logger().info(f'WeChat QR models: {mdir}')
        else:
            self.get_logger().warn('no WeChat QR models: tiled decoding, positions only where cv2.QRCodeDetector finds corners')

        self.qr_size = 0.126
        self.truth = {}
        if manifest and os.path.exists(manifest):
            m = json.load(open(manifest))
            self.qr_size = float(m.get('qr_size', self.qr_size))
            for c in m['codes']:   # a carton's code is on both of its faces: code -> [entries]
                self.truth.setdefault(c['code'], []).append(c)
            self.get_logger().info(f'manifest: {len(m["codes"])} labels, QR size {self.qr_size} m ({manifest})')
        else:
            self.get_logger().warn(f'no manifest ({manifest!r}): reports without expected / missing codes')

    def init_ros(self, state_topic):
        self.tf_listener = TransformListener(self.tf, self)
        self.pub_report = self.create_publisher(String, '~/report', LATCHED)
        self.pub_state = self.create_publisher(String, '~/state', LATCHED)
        self.pub_debug = self.create_publisher(Image, '~/debug_image', qos_profile_sensor_data)
        self.pub_markers = self.create_publisher(MarkerArray, '~/codes', LATCHED)
        # the lateral cameras are mounted in portrait: rotate the debug image by this (deg, clockwise) to show it upright
        self.debug_rotation = int(self.declare_parameter('debug_rotation', 90).value)
        self.debug_scale = float(self.declare_parameter('debug_scale', 0.5).value)
        self.create_timer(1.0, self.publish_markers)
        self.create_subscription(String, state_topic, self.on_state, LATCHED)

        self.scan = None
        self.lock = threading.Lock()
        self.active_cam = None
        self.subs = []
        self.info = {}
        self.jobs = queue.Queue(maxsize=1)
        self.qrdet = cv2.QRCodeDetector()
        self.worker = threading.Thread(target=self.work, daemon=True)
        self.worker.start()
        self.create_timer(0.5, self.select_camera)
        self.set_state('IDLE: waiting for a shelf coverage (COVERAGE on ' + state_topic + ')')

    # ------------------------------------------------------------------ mission state
    def set_state(self, text):
        self.pub_state.publish(String(data=text))

    def on_state(self, msg):
        text = msg.data
        word = text.split(':', 1)[0].strip()
        now = self.get_clock().now().nanoseconds * 1e-9
        if word == 'COVERAGE':
            m = SHELF_RE.search(text)
            if not m:
                return
            a, s, side = int(m.group(1)), int(m.group(2)), m.group(3)
            with self.lock:
                old = self.scan
                if old and (old.aisle, old.segment, old.side) == (a, s, side):
                    return
                self.scan = ShelfScan(a, s, side, now, datetime.datetime.now())
            if old:   # a new coverage without DONE in between: close the old one first
                old.t_end = now
                self.finish(old, 'interrupted by a new coverage')
            self.get_logger().info(f'scan started: {self.scan.face} ({side} shelf of aisle {a}, segment {s})')
            self.set_state(f'SCANNING {self.scan.face}')
        elif word in ('DONE', 'ABORTED', 'IDLE'):
            with self.lock:
                scan, self.scan = self.scan, None
            self.use_camera(None)
            if scan:
                scan.t_end = now
                self.finish(scan, text)
        # GOTO (before the coverage) / RETURN (after it, still along the shelf): keep the current scan going

    # ------------------------------------------------------------------ camera selection (TF)
    def select_camera(self):
        scan = self.scan
        if scan is None:
            self.use_camera(None)
            return
        want = +1.0 if scan.side == 'left' else -1.0   # direction aisle -> face along world y
        best, best_dot = None, self.min_facing
        for c, (_, _, frame) in self.cam_topics.items():
            try:
                t = self.tf.lookup_transform(self.frame, frame, Time())
            except Exception:
                continue
            axis = quat_to_rot(t.transform.rotation)[:, 0]   # camera link x = view axis
            if axis[1] * want > best_dot:
                best, best_dot = c, axis[1] * want
        self.use_camera(best)

    def use_camera(self, cam):
        if cam == self.active_cam:
            return
        for s in self.subs:
            if isinstance(s, str):
                self.gz_node.unsubscribe(s)
            else:
                self.destroy_subscription(s)
        self.subs = []
        self.active_cam = cam
        if cam and self.gz_topics.get(cam):
            base = self.gz_topics[cam]
            self.subs = self.gz_subscribe(cam, base)
            self.get_logger().info(f'{cam} camera on (gz {base})')
        elif cam:
            img, info, _ = self.cam_topics[cam]
            self.subs = [self.create_subscription(Image, img, lambda m, c=cam: self.on_image(c, m), qos_profile_sensor_data),
                         self.create_subscription(CameraInfo, info, lambda m, c=cam: self.on_info(c, m),
                                                  qos_profile_sensor_data)]
            self.get_logger().info(f'{cam} camera on ({img})')
        scan = self.scan
        if scan:
            self.set_state(f'SCANNING {scan.face} ({cam or "no camera facing the shelf"}, {len(scan.codes)} codes)')

    def gz_subscribe(self, cam, base):
        """Subscribe the gz topics of one camera, return them (unsubscribed when the camera goes off)."""
        self.gz_node.subscribe(self.GzCameraInfo, base + '/camera_info', lambda m, c=cam: self.on_gz_info(c, m))
        self.gz_node.subscribe(self.GzImage, base + '/image', lambda m, c=cam: self.on_gz_image(c, m))
        return [base + '/camera_info', base + '/image']

    # ------------------------------------------------------------------ frames
    # a frame = (gray image, stamp, K 3x3, D) whichever transport it came from
    def on_info(self, cam, msg):
        self.info[cam] = (np.array(msg.k, np.float64).reshape(3, 3),
                          np.array(msg.d, np.float64) if len(msg.d) else np.zeros(5))

    def on_gz_info(self, cam, msg):
        k = list(msg.intrinsics.k)
        if len(k) == 9:
            d = list(msg.distortion.k)
            self.info[cam] = (np.array(k, np.float64).reshape(3, 3), np.array(d, np.float64) if d else np.zeros(5))

    def queue_frame(self, cam, get_frame):
        scan = self.scan
        if scan is None or cam not in self.info:
            return
        scan.frames += 1
        if self.jobs.full():
            scan.frames_dropped += 1
            return
        try:
            self.jobs.put_nowait((scan, cam, get_frame(), self.info[cam]))
        except queue.Full:
            scan.frames_dropped += 1

    def on_image(self, cam, msg):
        self.queue_frame(cam, lambda: (self.to_gray(msg), Time.from_msg(msg.header.stamp)))

    def on_gz_image(self, cam, msg):
        def frame():
            buf = np.frombuffer(msg.data, np.uint8)
            step = msg.step or len(msg.data) // msg.height
            img = buf.reshape(msg.height, step)
            if step >= msg.width * 3:   # RGB_INT8
                img = cv2.cvtColor(img[:, :msg.width * 3].reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2GRAY)
            else:
                img = img[:, :msg.width].copy()
            st = msg.header.stamp
            return img, Time(seconds=st.sec, nanoseconds=st.nsec)
        self.queue_frame(cam, frame)

    def make_decoder(self):
        return cv2.wechat_qrcode_WeChatQRCode(*self.wechat_files) if self.wechat_files else cv2.wechat_qrcode_WeChatQRCode()

    def work(self):
        wechat = self.make_decoder()
        while True:
            scan, cam, frame, info = self.jobs.get()
            try:
                self.process(wechat, scan, cam, frame, info)
            except Exception as e:  # never let one frame kill the worker
                self.get_logger().error(f'frame failed: {e!r}')

    def to_gray(self, msg):
        buf = np.frombuffer(msg.data, np.uint8)
        if msg.encoding in ('mono8', '8UC1'):
            return buf.reshape(msg.height, msg.step)[:, :msg.width]
        img = buf.reshape(msg.height, msg.step // 1)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if msg.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)

    def decode(self, wechat, img):
        """code -> 4 corners (TL, TR, BR, BL) in px, or None when the code was read but not located."""
        if self.wechat_files:
            texts, pts = wechat.detectAndDecode(img)
            return {s: np.asarray(p, np.float64).reshape(4, 2) for s, p in zip(texts, pts) if s}
        return self.decode_tiled(wechat, img)

    def decode_tiled(self, wechat, img):
        H, W = img.shape
        t = min(self.tile, H, W)
        ys = list(range(0, H - t + 1, self.stride)) or [0]
        xs = list(range(0, W - t + 1, self.stride)) or [0]
        if ys[-1] + t < H:
            ys.append(H - t)
        if xs[-1] + t < W:
            xs.append(W - t)
        found = {}
        for y0 in ys:
            for x0 in xs:
                tile = img[y0:y0 + t, x0:x0 + t]
                texts, _ = wechat.detectAndDecode(tile)   # model-less: its points are the tile's corners
                todo = [s for s in texts if s and found.get(s) is None]
                if not todo:
                    continue
                for s in todo:
                    found.setdefault(s, None)
                up = cv2.resize(tile, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
                ok, txt, pts, _ = self.qrdet.detectAndDecodeMulti(up)
                for s2, p in zip(txt if ok else [], pts if ok else []):
                    if s2 in todo:
                        found[s2] = np.asarray(p, np.float64).reshape(4, 2) / 2 + [x0, y0]
        return found

    def process(self, wechat, scan, cam, frame, info):
        img, stamp = frame
        found = self.decode(wechat, img)
        if self.pub_debug.get_subscription_count() > 0:
            self.publish_debug(img, found, cam, stamp, scan)
        if not found:
            return
        K, D = info
        try:
            tf = self.tf.lookup_transform(self.frame, self.cam_topics[cam][2], stamp, timeout=Duration(seconds=0.1))
        except Exception:
            try:
                tf = self.tf.lookup_transform(self.frame, self.cam_topics[cam][2], Time())
            except Exception:
                tf = None
        R = quat_to_rot(tf.transform.rotation) if tf else None
        T = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]) if tf else None
        h = self.qr_size / 2
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], np.float64)
        t_img = stamp.nanoseconds * 1e-9
        for code, p in found.items():
            pos, rng, ok = None, None, False
            if p is not None:
                ok, rvec, tvec = cv2.solvePnP(obj, p, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if ok:
                o = tvec.reshape(3)                       # optical: x right, y down, z along the view
                rng = float(np.linalg.norm(o))
                if rng > self.max_range:
                    continue
                link = np.array([o[2], -o[0], -o[1]])     # camera link: x along the view, y left, z up
                if R is not None:
                    pos = R @ link + T
            px = float(math.sqrt(max(cv2.contourArea(p.astype(np.float32)), 0.0))) if p is not None else 0.0
            with self.lock:
                c = scan.codes.setdefault(code, dict(n=0, first=t_img, last=t_img, positions=[], best_px=0.0,
                                                     thumb=None, cams=set(), ranges=[]))
                c['n'] += 1
                c['last'] = t_img
                c['cams'].add(cam)
                if pos is not None:
                    c['positions'].append(pos.tolist())
                if rng is not None:
                    c['ranges'].append(rng)
                if p is not None and px > c['best_px']:
                    c['best_px'] = px
                    x0, y0 = np.maximum(p.min(0) - px * 0.35, 0).astype(int)
                    x1, y1 = np.minimum(p.max(0) + px * 0.35, [img.shape[1], img.shape[0]]).astype(int)
                    crop = img[y0:y1, x0:x1]
                    if crop.size:
                        s = self.thumb_px / max(crop.shape)
                        crop = cv2.resize(crop, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
                        c['thumb'] = cv2.imencode('.jpg', crop, [cv2.IMWRITE_JPEG_QUALITY, 85])[1].tobytes()
                scan.cams.add(cam)
                scan.rows.append([f'{t_img:.3f}', cam, code, f'{px:.1f}', '' if rng is None else f'{rng:.3f}'] +
                                 (['', '', ''] if pos is None else [f'{v:.3f}' for v in pos]) +
                                 (['', '', ''] if T is None else [f'{v:.3f}' for v in T]))
        self.set_state(f'SCANNING {scan.face} ({cam}, {len(scan.codes)} codes)')

    # ------------------------------------------------------------------ rviz
    def kind(self, code, scan):
        if code.startswith('CTN'):
            t = self.truth.get(code)
            mine = not t or any((e['aisle'], e['segment'], e['side']) == (scan.aisle, scan.segment, scan.side) for e in t)
            return 'carton' if mine else 'other'
        return 'location' if code.startswith(f'LOC:{scan.face}-') or not self.truth else 'other'

    COLORS = {'location': (60, 200, 60), 'carton': (0, 150, 255), 'other': (150, 150, 150), 'missing': (40, 40, 230)}  # BGR

    def publish_debug(self, img, found, cam, stamp, scan):
        s = self.debug_scale
        vis = cv2.cvtColor(cv2.resize(img, None, fx=s, fy=s, interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
        rot = {90: cv2.ROTATE_90_CLOCKWISE, -90: cv2.ROTATE_90_COUNTERCLOCKWISE, 270: cv2.ROTATE_90_COUNTERCLOCKWISE,
               180: cv2.ROTATE_180}.get(self.debug_rotation)
        H, W = vis.shape[:2]

        def rp(p):  # point of the scaled image -> rotated image
            x, y = p
            if rot == cv2.ROTATE_90_CLOCKWISE:
                return (H - 1 - y, x)
            if rot == cv2.ROTATE_90_COUNTERCLOCKWISE:
                return (y, W - 1 - x)
            if rot == cv2.ROTATE_180:
                return (W - 1 - x, H - 1 - y)
            return (x, y)
        if rot is not None:
            vis = cv2.rotate(vis, rot)
        for code, p in found.items():
            if p is None:
                continue
            col = self.COLORS[self.kind(code, scan)]
            q = np.array([rp(pt) for pt in p * s], np.int32)
            cv2.polylines(vis, [q], True, col, 2, cv2.LINE_AA)
            x, y = q[:, 0].min(), q[:, 1].min() - 6
            label = code[4:] if code.startswith('LOC:') else code.split('/')[0]
            cv2.putText(vis, label, (int(x), int(max(y, 12))), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(vis, label, (int(x), int(max(y, 12))), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        cv2.putText(vis, f'{self.uav} {cam} | {scan.face} | {len(scan.codes)} codes', (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        msg = Image()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.cam_topics[cam][2]
        msg.height, msg.width = vis.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = vis.tobytes()
        self.pub_debug.publish(msg)

    def marker_array(self, scan, missing=()):
        arr = MarkerArray()
        arr.markers.append(Marker(action=Marker.DELETEALL))
        now = self.get_clock().now().to_msg()
        with self.lock:
            items = [(c, np.mean(v['positions'], axis=0)) for c, v in scan.codes.items() if v['positions']]
        items += [(c, None) for c in missing]
        for i, (code, pos) in enumerate(items):
            k = 'missing' if pos is None else self.kind(code, scan)
            if k == 'other':
                continue
            if pos is None:
                t = next((e for e in self.truth.get(code, []) if
                          (e['aisle'], e['segment'], e['side']) == (scan.aisle, scan.segment, scan.side)), None)
                if t is None:
                    continue
                pos = (t['x'], t['y'], t['z'])
            b, g, r = self.COLORS[k]
            col = ColorRGBA(r=r / 255, g=g / 255, b=b / 255, a=0.9)
            m = Marker(type=Marker.CUBE, ns=f'{scan.face} {k}', id=i, action=Marker.ADD)
            m.header.frame_id, m.header.stamp = self.frame, now
            m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, pos)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = 0.16, 0.03, 0.19
            m.color = col
            arr.markers.append(m)
            t = Marker(type=Marker.TEXT_VIEW_FACING, ns=f'{scan.face} {k} text', id=i, action=Marker.ADD)
            t.header = m.header
            t.pose.position.x, t.pose.position.y, t.pose.position.z = float(pos[0]), float(pos[1]), float(pos[2]) + 0.17
            t.pose.orientation.w = 1.0
            t.scale.z = 0.09
            t.color = col
            t.text = code[4:] if code.startswith('LOC:') else code.split('/')[0]
            arr.markers.append(t)
        return arr

    def publish_markers(self):
        scan = self.scan
        if scan is not None:
            self.pub_markers.publish(self.marker_array(scan))

    # ------------------------------------------------------------------ report
    def finish(self, scan, end_state):
        """Wait for the frame being decoded, then write the report (in this callback: rare, a few 100 ms)."""
        scan.end_state = end_state
        deadline = time.time() + 2.0
        while self.jobs.qsize() and time.time() < deadline:
            time.sleep(0.05)
        try:
            path, summary = self.write_report(scan)
        except Exception as e:
            self.get_logger().error(f'report of {scan.face} failed: {e!r}')
            return
        self.pub_report.publish(String(data=json.dumps(summary)))
        self.pub_markers.publish(self.marker_array(scan, summary.get('missing', ())))   # stays (latched) until the next scan
        self.set_state(f'REPORT {path}')
        self.get_logger().info(self.finish_log(scan, summary, path))

    def finish_log(self, scan, summary, path):
        loc, ctn = summary['locations'], summary['cartons']
        return (f'REPORT {scan.face}: locations {loc["found"]}/{loc["expected"]}, cartons {ctn["found"]}/{ctn["expected"]}, '
                f'{summary["other_codes"]} codes of other shelves, mean position error '
                f'{summary["position_error_mean"]} m -> {path}/report.html')

    def write_report(self, scan):
        with self.lock:
            codes = {k: dict(v, cams=sorted(v['cams'])) for k, v in scan.codes.items()}
            rows = list(scan.rows)
        stamp = scan.wall_start.strftime('%H%M%S')
        name = f'{self.uav}_{scan.face}_{stamp}'
        out = os.path.join(self.data_dir, name)
        os.makedirs(out, exist_ok=True)

        expected = {c: t for c, ts in self.truth.items() for t in ts
                    if (t['aisle'], t['segment'], t['side']) == (scan.aisle, scan.segment, scan.side)}
        mine = {c: v for c, v in codes.items() if c in expected or (not self.truth)}
        unknown = {c: v for c, v in codes.items() if c not in self.truth and self.truth}
        other = {c: v for c, v in codes.items() if c in self.truth and c not in expected}
        # a code of this face, read but not in the manifest, is still this shelf's: e.g. LOC:<face>-...
        for c, v in unknown.items():
            if c.startswith(f'LOC:{scan.face}-'):
                mine[c] = v

        entries, errs = [], []
        loc_ref = [t for t in expected.values() if t['type'] == 'location']
        for c, v in sorted(mine.items()):
            t = expected.get(c, {})
            est = np.mean(v['positions'], axis=0).tolist() if v['positions'] else None
            err = None
            if est is not None and t:
                err = float(np.linalg.norm(np.array(est) - [t['x'], t['y'], t['z']]))
                errs.append(err)
            est_bay = est_lv = None
            if est is not None and loc_ref:   # nearest location label of this face = the bay / level it sits in
                near = min(loc_ref, key=lambda r: abs(r['x'] - est[0]) + 0.5 * abs(r['z'] - est[2]))
                est_bay = near['bay']
                below = [r for r in loc_ref if r['bay'] == est_bay and r['z'] <= est[2] + 0.15]
                est_lv = max(below, key=lambda r: r['z'])['level'] if below else near['level']
            entries.append(dict(code=c, type=t.get('type', 'carton' if c.startswith('CTN') else 'location'),
                                carton=t.get('carton'), sku=t.get('sku'), bay=t.get('bay'), level=t.get('level'),
                                est_bay=est_bay, est_level=est_lv, reads=v['n'], cameras=v['cams'],
                                first=round(v['first'], 2), last=round(v['last'], 2),
                                est=[round(x, 3) for x in est] if est else None,
                                truth=[t['x'], t['y'], t['z']] if t else None,
                                error=round(err, 3) if err is not None else None,
                                range=round(float(np.median(v['ranges'])), 2) if v['ranges'] else None))
        missing = sorted(set(expected) - set(mine))
        n_loc = sum(1 for t in expected.values() if t['type'] == 'location')
        n_ctn = len(expected) - n_loc
        f_loc = sum(1 for e in entries if e['type'] == 'location' and e['code'] in expected)
        f_ctn = sum(1 for e in entries if e['type'] == 'carton' and e['code'] in expected)
        misplaced = [e['code'] for e in entries if e['type'] == 'carton' and e['bay'] and e['est_bay']
                     and (e['bay'], e['level']) != (e['est_bay'], e['est_level'])]
        summary = dict(
            uav=self.uav, shelf=scan.face, aisle=scan.aisle, segment=scan.segment, side=scan.side,
            started=scan.wall_start.isoformat(timespec='seconds'),
            sim_time=[round(scan.t_start, 2), round(scan.t_end or scan.t_start, 2)],
            duration_s=round((scan.t_end or scan.t_start) - scan.t_start, 1), end_state=scan.end_state,
            frames=scan.frames, frames_decoded=scan.frames - scan.frames_dropped, cameras=sorted(scan.cams),
            locations=dict(expected=n_loc, found=f_loc), cartons=dict(expected=n_ctn, found=f_ctn),
            missing=missing, misplaced_cartons=misplaced, other_codes=len(other), unknown_codes=sorted(unknown),
            position_error_mean=round(float(np.mean(errs)), 3) if errs else None,
            position_error_max=round(float(np.max(errs)), 3) if errs else None,
            report_dir=out)

        with open(os.path.join(out, 'report.json'), 'w') as f:
            json.dump(dict(summary=summary, codes=entries,
                           other_shelves=sorted(other)), f, indent=1)
        with open(os.path.join(out, 'detections.csv'), 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['sim_time', 'camera', 'code', 'code_px', 'range_m', 'x', 'y', 'z', 'cam_x', 'cam_y', 'cam_z'])
            w.writerows(rows)
        grid = self.grid(expected, mine, entries)
        with open(os.path.join(out, 'report.md'), 'w') as f:
            f.write(self.markdown(summary, entries, grid))
        with open(os.path.join(out, 'report.html'), 'w') as f:
            f.write(self.html(summary, entries, grid, codes))
        self.append_run_files(summary, entries, out)
        return out, summary

    def grid(self, expected, mine, entries):
        """level (top first) -> bay -> dict(loc: True/False/None, cartons [(id, found)])"""
        bays = sorted({t['bay'] for t in expected.values()}) or sorted({e['est_bay'] for e in entries if e['est_bay']})
        levels = sorted({t['level'] for t in expected.values()}) or sorted({e['est_level'] for e in entries if e['est_level']})
        g = {lv: {b: dict(loc=None, cartons=[]) for b in bays} for lv in levels}
        for c, t in expected.items():
            cell = g[t['level']][t['bay']]
            if t['type'] == 'location':
                cell['loc'] = c in mine
            else:
                cell['cartons'].append((t['carton'], c in mine))
        return bays, levels[::-1], g

    def markdown(self, s, entries, grid):
        bays, levels, g = grid
        L = [f"# Shelf report {s['shelf']} ({s['uav']})", '',
             f"Aisle {s['aisle']}, segment {s['segment']}, {s['side']} shelf. Started {s['started']}, "
             f"{s['duration_s']} s of sim time, {s['frames_decoded']}/{s['frames']} frames decoded "
             f"({', '.join(s['cameras']) or 'no'} camera). End: `{s['end_state']}`", '',
             f"- **Location labels:** {s['locations']['found']} / {s['locations']['expected']}",
             f"- **Cartons:** {s['cartons']['found']} / {s['cartons']['expected']}",
             f"- **Position error** (estimate vs ground truth): mean {s['position_error_mean']} m, "
             f"max {s['position_error_max']} m",
             f"- **Misplaced cartons** (read in another bay / level than the manifest's): "
             f"{', '.join(s['misplaced_cartons']) or 'none'}",
             f"- Codes of other shelves seen through the racks: {s['other_codes']}", '']
        if bays:
            L += ['| level \\ bay | ' + ' | '.join(f'B{b:02d}' for b in bays) + ' |',
                  '|---|' + '---|' * len(bays)]
            for lv in levels:
                cells = []
                for b in bays:
                    c = g[lv][b]
                    mark = {True: '✅', False: '❌', None: '·'}[c['loc']]
                    ct = ' '.join(f"{cid}{'' if ok else '❌'}" for cid, ok in c['cartons'])
                    cells.append(f'{mark} {ct}'.strip())
                L.append(f'| L{lv} | ' + ' | '.join(cells) + ' |')
            L += ['', '✅ location label read, ❌ not read; carton ids listed per bay / level (❌: not read).', '']
        if s['missing']:
            L += ['## Missing', ''] + [f'- `{m}`' for m in s['missing']] + ['']
        L += ['## Codes read', '', '| code | type | bay / level (manifest → read) | reads | range [m] | error [m] |',
              '|---|---|---|---|---|---|']
        for e in entries:
            bl = f"B{e['bay']} L{e['level']}" if e['bay'] else '?'
            rb = f"B{e['est_bay']} L{e['est_level']}" if e['est_bay'] else '?'
            L.append(f"| `{e['code']}` | {e['type']} | {bl} → {rb} | {e['reads']} | {e['range']} | {e['error']} |")
        return '\n'.join(L) + '\n'

    def html(self, s, entries, grid, codes):
        bays, levels, g = grid
        esc = html.escape
        rows = ''
        for lv in levels:
            tds = ''
            for b in bays:
                c = g[lv][b]
                cls = {True: 'ok', False: 'miss', None: 'na'}[c['loc']]
                ct = ''.join(f"<div class='ctn {'ok' if ok else 'miss'}'>{esc(cid)}</div>" for cid, ok in c['cartons'])
                tds += f"<td class='{cls}'><div class='loc'>{'✓' if c['loc'] else ('✗' if c['loc'] is False else '·')}</div>{ct}</td>"
            rows += f'<tr><th>L{lv}</th>{tds}</tr>'
        head = ''.join(f'<th>B{b:02d}</th>' for b in bays)
        cards = ''
        for e in entries:
            th = codes.get(e['code'], {}).get('thumb')
            img = f"<img src='data:image/jpeg;base64,{base64.b64encode(th).decode()}'>" if th else ''
            cards += (f"<div class='card'>{img}<code>{esc(e['code'])}</code><span>{e['type']}"
                      f"{' · B%s L%s' % (e['bay'], e['level']) if e['bay'] else ''} · {e['reads']} reads"
                      f"{' · err %.2f m' % e['error'] if e['error'] is not None else ''}</span></div>")
        missing = ''.join(f'<li><code>{esc(m)}</code></li>' for m in s['missing']) or '<li>none</li>'
        pct = lambda d: f"{100 * d['found'] / d['expected']:.0f}%" if d['expected'] else '–'
        return f'''<!doctype html><html><head><meta charset="utf-8"><title>Shelf {esc(s['shelf'])}</title>
<style>
:root{{--bg:#fff;--fg:#1d1d1f;--mut:#6e6e73;--ok:#1f9d55;--okbg:#e3f6ea;--miss:#d93025;--missbg:#fce8e6;--line:#e5e5ea}}
@media (prefers-color-scheme:dark){{:root{{--bg:#151517;--fg:#f2f2f7;--mut:#a1a1a6;--okbg:#13361f;--missbg:#3d1714;--line:#2c2c2e}}}}
body{{font:14px/1.45 system-ui,sans-serif;background:var(--bg);color:var(--fg);margin:24px auto;max-width:1100px;padding:0 16px}}
h1{{font-size:22px;margin:0 0 4px}} .mut{{color:var(--mut)}} .kpis{{display:flex;gap:12px;flex-wrap:wrap;margin:16px 0}}
.kpi{{border:1px solid var(--line);border-radius:10px;padding:10px 14px;min-width:150px}} .kpi b{{font-size:22px;display:block}}
table{{border-collapse:collapse;width:100%;margin:12px 0}} td,th{{border:1px solid var(--line);padding:6px;text-align:center;vertical-align:top}}
td.ok{{background:var(--okbg)}} td.miss{{background:var(--missbg)}} .loc{{font-weight:600}}
.ctn{{font-size:11px;font-family:ui-monospace,monospace}} .ctn.miss{{color:var(--miss)}}
.cards{{display:grid;grid-template-columns:repeat(auto-fill,minmax(170px,1fr));gap:10px}}
.card{{border:1px solid var(--line);border-radius:8px;padding:8px;display:flex;flex-direction:column;gap:4px;font-size:12px}}
.card img{{width:100%;image-rendering:pixelated;border-radius:4px}} .card span{{color:var(--mut)}} code{{word-break:break-all}}
</style></head><body>
<h1>Shelf {esc(s['shelf'])} — {esc(s['uav'])}</h1>
<div class="mut">Aisle {s['aisle']}, segment {s['segment']}, {s['side']} shelf · started {esc(s['started'])} ·
{s['duration_s']} s sim time · {s['frames_decoded']}/{s['frames']} frames decoded · {esc(s['end_state'])}</div>
<div class="kpis">
<div class="kpi"><b>{s['locations']['found']}/{s['locations']['expected']}</b>location labels ({pct(s['locations'])})</div>
<div class="kpi"><b>{s['cartons']['found']}/{s['cartons']['expected']}</b>cartons ({pct(s['cartons'])})</div>
<div class="kpi"><b>{s['position_error_mean'] if s['position_error_mean'] is not None else '–'} m</b>mean position error (max {s['position_error_max']})</div>
<div class="kpi"><b>{len(s['misplaced_cartons'])}</b>misplaced cartons</div>
</div>
<table><tr><th>level \\ bay</th>{head}</tr>{rows}</table>
<h2>Missing</h2><ul>{missing}</ul>
<h2>Codes read</h2><div class="cards">{cards}</div>
<p class="mut">{s['other_codes']} codes of other shelves seen through the racks (not counted). Raw reads: detections.csv.</p>
</body></html>
'''

    def append_run_files(self, s, entries, out):
        """inventory.csv / index.md of the run, shared by every UAV's scanner: append under a file lock."""
        os.makedirs(self.data_dir, exist_ok=True)
        inv = os.path.join(self.data_dir, 'inventory.csv')
        with open(inv, 'a', newline='') as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            w = csv.writer(f)
            if f.tell() == 0:
                w.writerow(['started', 'uav', 'shelf', 'code', 'type', 'carton', 'sku', 'bay', 'level', 'read_bay',
                            'read_level', 'reads', 'x', 'y', 'z', 'error_m'])
            for e in entries:
                est = e['est'] or ['', '', '']
                w.writerow([s['started'], s['uav'], s['shelf'], e['code'], e['type'], e['carton'] or '', e['sku'] or '',
                            e['bay'] or '', e['level'] or '', e['est_bay'] or '', e['est_level'] or '', e['reads'],
                            *est, '' if e['error'] is None else e['error']])
            fcntl.flock(f, fcntl.LOCK_UN)
        idx = os.path.join(self.data_dir, 'index.md')
        with open(idx, 'a') as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            if f.tell() == 0:
                f.write('# Shelf scans\n\n| started | uav | shelf | locations | cartons | mean error [m] | report |\n'
                        '|---|---|---|---|---|---|---|\n')
            rel = os.path.relpath(out, self.data_dir)
            f.write(f"| {s['started']} | {s['uav']} | {s['shelf']} | {s['locations']['found']}/{s['locations']['expected']} | "
                    f"{s['cartons']['found']}/{s['cartons']['expected']} | {s['position_error_mean']} | "
                    f"[{rel}]({rel}/report.md) |\n")
            fcntl.flock(f, fcntl.LOCK_UN)


def main():
    rclpy.init()
    node = ShelfScanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
