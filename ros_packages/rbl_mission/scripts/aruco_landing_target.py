#!/usr/bin/env python3
"""ArUco landing-target detector for PX4's built-in precision landing.

Detects one ArUco marker -- or a board of several (board_* parameters) -- in a downward camera
image, estimates the position of the marker / board origin relative to the vehicle and sends it to PX4 as MAVLink TARGET_RELATIVE (development dialect, id 511). With
VTE_EN=0, PX4's mavlink_receiver turns each one directly into landing_target_pose (absolute target
position in the local frame), which drives the navigator's precision landing (AUTO_PRECLAND, e.g.
px4_bridge ~/precland); with VTE_EN=1 it goes to the vision_target_estimator instead
(fiducial_marker_pos_report). No PX4 change is needed: the px4_sitl_default build already has the
development dialect.

Board: board_ids, board_sizes [m] and board_centers (flat x, y list [m], x right / y up of the
printed board, origin = the landing point) describe the markers; one solvePnP over the corners of
every detected board marker gives the origin. A small centre marker keeps it visible until
touchdown, larger outer ones make it detectable from height (tmux/px4_sim_2ugv_2uav_warehouse_stereo_larger/
models/make_deck_boards.py). Without board_ids: the single marker marker_id / marker_size.

  sub  image        sensor_msgs/Image       (rgb8 / bgr8 / mono8)
  sub  camera_info  sensor_msgs/CameraInfo  (intrinsics; until it arrives, nothing is detected)
  sub  <state_topic> std_msgs/String  optional: image / camera_info only while the state starts with
                     one of active_states (default PICKUP, RENDEZVOUS, LAND)
  pub  ~/target     geometry_msgs/PointStamped  marker position in <body_frame> (FLU), for rviz
  pub  ~/debug_image sensor_msgs/Image  annotated camera view (own markers green, others red), for rviz
  UDP  TARGET_RELATIVE -> mavlink_host:mavlink_port (PX4 SITL instance i: onboard link 14580+i)

Without odometry_topic, the measurement goes out in TARGET_OBS_FRAME_BODY_FRD with q_sensor left
zero: PX4 rotates it into NED with its attitude and adds its position *at reception*, not at image
capture -- with the image pipeline's latency, a UAV tilting by a few degrees while it corrects
puts the target 5-10 cm off at 1.5 m, and each such raw sample becomes the landing setpoint.

With odometry_topic (PX4's own fmu/out/vehicle_odometry, NED), each detection is rotated with the
attitude and offset by the position the UAV had when the image was taken, giving the marker's
absolute position in PX4's local frame. The marker is static while the UGV is parked, so the
median over the last filter_window seconds is sent (TARGET_OBS_FRAME_LOCAL_NED, which PX4 hands
to landing_target_pose unchanged). If no odometry sample is close enough to the image stamp it
falls back to the BODY_FRD message. The camera is assumed to look straight down with the
image top towards the vehicle's front (x500_lidar_3d_cam_down): optical (x right, y down,
z forward) -> FRD: x = -y_opt, y = x_opt, z = z_opt, plus camera_offset_frd.

pymavlink's bundled development dialect predates TARGET_RELATIVE, so the MAVLink 2 frame is
packed here (field order / CRC_EXTRA from the message definition, checked against PX4's
development.xml with pymavlink's mavparse: wire length 71, crc_extra 28).
"""

import bisect
import collections
import socket
import struct

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

TARGET_RELATIVE_ID = 511
TARGET_RELATIVE_CRC_EXTRA = 28
TARGET_OBS_FRAME_LOCAL_NED = 0
TARGET_OBS_FRAME_BODY_FRD = 1
LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2


def x25crc(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def quat_to_rot(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def pack_target_relative(seq, sysid, compid, timestamp_us, target_id, pos_frd, pos_std,
                         frame=TARGET_OBS_FRAME_BODY_FRD):
    # wire order: uint64, then floats (x y z pos_std[3] yaw_std q_target[4] q_sensor[4]), then uint8s
    payload = struct.pack(
        '<Q3f3ff4f4f3B',
        timestamp_us,
        *pos_frd,
        *pos_std,
        3.14,                      # yaw_std: no yaw measurement (VTE_YAW_EN 0)
        1.0, 0.0, 0.0, 0.0,        # q_target
        0.0, 0.0, 0.0, 0.0,        # q_sensor: zero -> PX4 uses its attitude (BODY_FRD)
        target_id,
        frame,
        LANDING_TARGET_TYPE_VISION_FIDUCIAL,
    )
    payload = payload.rstrip(b'\x00') or b'\x00'  # MAVLink 2 trailing-zero truncation
    # len, incompat_flags, compat_flags, seq, sysid, compid, msgid (24 bit)
    header = struct.pack('<BBBBBB', len(payload), 0, 0, seq & 0xFF, sysid, compid) + \
        TARGET_RELATIVE_ID.to_bytes(3, 'little')
    crc = x25crc(header + payload)
    crc = x25crc(bytes([TARGET_RELATIVE_CRC_EXTRA]), crc)
    return b'\xfd' + header + payload + struct.pack('<H', crc)


class ArucoLandingTarget(Node):
    def __init__(self):
        super().__init__('landing_target_detector')
        self.marker_id = self.declare_parameter('marker_id', 0).value
        self.marker_size = self.declare_parameter('marker_size', 0.30).value  # [m] black square edge
        ids = list(self.declare_parameter('board_ids', [0]).value)
        sizes = list(self.declare_parameter('board_sizes', [0.0]).value)
        centers = list(self.declare_parameter('board_centers', [0.0]).value)
        if not (len(ids) > 1 or sizes[0] > 0.0):  # no board: the single marker
            ids, sizes, centers = [self.marker_id], [self.marker_size], [0.0, 0.0]
        if len(sizes) != len(ids) or len(centers) != 2 * len(ids):
            raise ValueError('board_sizes needs one entry per board_ids, board_centers two (x, y)')
        # object corners per id, in the detector's corner order (top-left, top-right, bottom-right,
        # bottom-left of the printed marker; y up)
        self.board = {}
        for i, (mid, sz) in enumerate(zip(ids, sizes)):
            cx, cy, h = centers[2 * i], centers[2 * i + 1], sz / 2.0
            self.board[int(mid)] = np.array([[cx - h, cy + h, 0], [cx + h, cy + h, 0],
                                             [cx + h, cy - h, 0], [cx - h, cy - h, 0]], dtype=np.float64)
        self.target_id = int(ids[0])
        dict_name = self.declare_parameter('dictionary', 'DICT_4X4_50').value
        self.cam_offset = np.array(self.declare_parameter('camera_offset_frd', [-0.06, 0.0, 0.06]).value)
        self.body_frame = self.declare_parameter('body_frame', 'uav1/fcu').value
        host = self.declare_parameter('mavlink_host', '127.0.0.1').value
        port = self.declare_parameter('mavlink_port', 14581).value
        self.sysid = self.declare_parameter('mavlink_sysid', 2).value     # = the vehicle's MAV_SYS_ID
        self.compid = self.declare_parameter('mavlink_compid', 191).value  # MAV_COMP_ID_ONBOARD_COMPUTER
        self.std_min = self.declare_parameter('pos_std_min', 0.02).value   # [m]
        self.std_per_m = self.declare_parameter('pos_std_per_meter', 0.02).value
        # PX4 vehicle_odometry (NED) for latency/attitude compensation; '' = send BODY_FRD as before
        odom_topic = self.declare_parameter('odometry_topic', '').value
        self.odom_max_dt = self.declare_parameter('odometry_max_dt', 0.05).value  # [s] image <-> odom
        self.filter_window = self.declare_parameter('filter_window', 0.5).value   # [s] median window
        self.filter_reset = self.declare_parameter('filter_reset_dist', 0.3).value  # [m] jump -> restart
        self.odom = collections.deque(maxlen=400)  # (t [s], position NED, q FRD->NED), ~2 s at 200 Hz
        self.samples = collections.deque()         # (t [s], absolute target NED)
        self.n_comp = 0
        self.n_markers = 0

        self.dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dict_name)) \
            if hasattr(cv2.aruco, 'Dictionary_get') else cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
        self.det_params = cv2.aruco.DetectorParameters_create() \
            if hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters()
        self.det_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.K = None
        self.D = None
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest = (host, port)
        self.seq = 0
        self.n_sent = 0
        self.n_frames = 0

        self.pub = self.create_publisher(PointStamped, '~/target', 10)
        # annotated camera image for rviz (only drawn while someone subscribes): this board's markers
        # green, other ids red, board origin axes + estimate as text
        self.pub_dbg = self.create_publisher(Image, '~/debug_image', qos_profile_sensor_data)
        # state_topic (the pair's mission node ~/state, transient_local): subscribe to the camera only
        # while the mission state starts with one of active_states. With ros_gz_bridge lazy:=true the
        # gz camera then has no subscriber the rest of the time, and gz-sensors skips rendering it
        # (many-vehicle sims). '' = always subscribed.
        state_topic = self.declare_parameter('state_topic', '').value
        self.active_states = tuple(self.declare_parameter(
            'active_states', ['PICKUP', 'RENDEZVOUS', 'LAND']).value)
        self.odom_topic = odom_topic
        self.sub_info = self.sub_image = self.sub_odom = None
        if state_topic:
            from std_msgs.msg import String
            latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(String, state_topic, self.cb_state, latched)
            self.get_logger().info(f'camera only while {state_topic} starts with {list(self.active_states)}')
        else:
            self.set_camera(True)
        if odom_topic:
            self.get_logger().info(f'latency/attitude compensation with {odom_topic}, '
                                   f'{self.filter_window:.2f} s median filter -> LOCAL_NED')
        self.create_timer(5.0, self.cb_status)
        self.get_logger().info(
            f'ArUco {dict_name} ids {sorted(self.board)} -> TARGET_RELATIVE to '
            f'{host}:{port} (sysid {self.sysid})')

    def set_camera(self, on: bool):
        # PX4 odometry (~100 Hz, costly in Python) only while the camera is on, too
        if on and self.sub_image is None:
            if self.odom_topic:
                from px4_msgs.msg import VehicleOdometry
                px4_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5,
                                     reliability=ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.VOLATILE)
                self.sub_odom = self.create_subscription(VehicleOdometry, self.odom_topic, self.cb_odom, px4_qos)
            self.sub_info = self.create_subscription(CameraInfo, 'camera_info', self.cb_info,
                                                     qos_profile_sensor_data)
            self.sub_image = self.create_subscription(Image, 'image', self.cb_image, qos_profile_sensor_data)
            self.get_logger().info('camera on')
        elif not on and self.sub_image is not None:
            self.destroy_subscription(self.sub_image)
            self.destroy_subscription(self.sub_info)
            if self.sub_odom is not None:
                self.destroy_subscription(self.sub_odom)
            self.sub_image = self.sub_info = self.sub_odom = None
            self.odom.clear()
            self.samples.clear()  # the next landing starts a fresh median window
            self.get_logger().info('camera off')

    def cb_state(self, m):
        self.set_camera(m.data.startswith(self.active_states))

    def cb_info(self, m: CameraInfo):
        if self.K is None:
            self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(m.d, dtype=np.float64) if len(m.d) else np.zeros(5)
            self.get_logger().info(f'camera intrinsics: fx {self.K[0, 0]:.1f} cx {self.K[0, 2]:.1f} '
                                   f'cy {self.K[1, 2]:.1f} ({m.width}x{m.height})')

    def cb_odom(self, m):
        if m.pose_frame != 1 or not np.all(np.isfinite(m.position)) or not np.all(np.isfinite(m.q)):
            return  # NED only
        # stamped on reception (sim clock, like the image stamps): the XRCE hop is a few ms
        t = self.get_clock().now().nanoseconds * 1e-9
        self.odom.append((t, np.array(m.position, dtype=np.float64), np.array(m.q, dtype=np.float64)))

    def odom_at(self, t):
        if not self.odom:
            return None
        ts = [o[0] for o in self.odom]
        i = bisect.bisect_left(ts, t)
        best = min((j for j in (i - 1, i) if 0 <= j < len(ts)), key=lambda j: abs(ts[j] - t))
        return self.odom[best] if abs(ts[best] - t) <= self.odom_max_dt else None

    def cb_status(self):
        self.get_logger().info(f'{self.n_frames} frames, marker sent {self.n_sent} times '
                               f'({self.n_comp} compensated, {self.n_markers / max(self.n_sent, 1):.1f} '
                               f'markers each) in the last 5 s')
        self.n_frames = 0
        self.n_sent = 0
        self.n_comp = 0
        self.n_markers = 0

    def cb_image(self, m: Image):
        if self.K is None:
            return
        self.n_frames += 1
        buf = np.frombuffer(m.data, dtype=np.uint8)
        if m.encoding in ('rgb8', 'bgr8'):
            img = buf.reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
            gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if m.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)
        elif m.encoding == 'mono8':
            gray = buf.reshape(m.height, m.step)[:, :m.width]
        else:
            self.get_logger().warn(f'unsupported encoding {m.encoding}', throttle_duration_sec=10.0)
            return

        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.det_params)
        res = self.process(m, corners, ids)
        if self.pub_dbg.get_subscription_count() > 0:
            self.publish_debug(m, gray, corners, ids, res)

    def publish_debug(self, m, gray, corners, ids, res):
        dbg = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        if ids is not None:
            for c, mid in zip(corners, ids.ravel()):
                col = (0, 200, 0) if int(mid) in self.board else (0, 0, 255)
                pts = c.reshape(4, 2).astype(np.int32)
                cv2.polylines(dbg, [pts], True, col, 2)
                cv2.putText(dbg, str(int(mid)), tuple(pts.mean(axis=0).astype(int)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
        status, pose = res
        if pose is not None:
            rvec, tvec, p_frd, p_send = pose
            cv2.drawFrameAxes(dbg, self.K, self.D, rvec, tvec, 0.15, 3)
            txt2 = (f'body FRD x {p_frd[0]:+.2f} y {p_frd[1]:+.2f} z {p_frd[2]:.2f} m'
                    + ('' if p_send is None else f'  -> NED {p_send[0]:+.2f} {p_send[1]:+.2f} {p_send[2]:+.2f}'))
            cv2.putText(dbg, txt2, (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        col = (0, 200, 0) if status.startswith('SENT') else (0, 0, 255)
        cv2.putText(dbg, status, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)
        h, w = dbg.shape[:2]
        cv2.drawMarker(dbg, (w // 2, h // 2), (255, 255, 0), cv2.MARKER_CROSS, 20, 1)  # image centre
        out = Image()
        out.header = m.header
        out.height, out.width, out.encoding, out.is_bigendian, out.step = h, w, 'bgr8', 0, w * 3
        out.data = dbg.tobytes()
        self.pub_dbg.publish(out)

    def process(self, m, corners, ids):
        """-> (status text, (rvec, tvec, p_frd, p_sent_ned or None) or None)"""
        if ids is None:
            return 'no marker', None
        obj, img = [], []
        for c, mid in zip(corners, ids.ravel()):
            if int(mid) in self.board:
                obj.append(self.board[int(mid)])
                img.append(c.reshape(4, 2).astype(np.float64))
        if not obj:
            self.get_logger().warn(f'ArUco ids {sorted(set(ids.ravel().tolist()))} seen, none of this board '
                                   f'{sorted(self.board)} (wrong deck texture / other UGV?)',
                                   throttle_duration_sec=5.0)
            return f'ids {sorted(set(ids.ravel().tolist()))}: NOT this board {sorted(self.board)}', None
        obj, img = np.concatenate(obj), np.concatenate(img)
        # coplanar points: IPPE (4 corners of one marker or more); with several markers refine with LM
        ok, rvec, tvec = cv2.solvePnP(obj, img, self.K, self.D, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return 'solvePnP failed', None
        if len(obj) > 4:
            rvec, tvec = cv2.solvePnPRefineLM(obj, img, self.K, self.D, rvec, tvec)
        self.n_markers += len(obj) // 4
        t = tvec.ravel()
        p_frd = np.array([-t[1], t[0], t[2]]) + self.cam_offset
        dist = float(np.linalg.norm(t))
        std = max(self.std_min, self.std_per_m * dist)

        stamp_us = m.header.stamp.sec * 1_000_000 + m.header.stamp.nanosec // 1000
        t_img = stamp_us * 1e-6
        o = self.odom_at(t_img)
        if o is not None:
            # absolute marker position in PX4's local NED frame, with the pose at capture time
            p_abs = o[1] + quat_to_rot(o[2]) @ p_frd
            while self.samples and (self.samples[0][0] < t_img - self.filter_window or
                                    self.samples[0][0] > t_img):  # old, or sim clock reset
                self.samples.popleft()
            if self.samples:
                med = np.median(np.array([s[1] for s in self.samples]), axis=0)
                if np.linalg.norm(p_abs[:2] - med[:2]) > self.filter_reset:
                    self.samples.clear()  # target moved (UGV not parked) or wrong lock: start over
            self.samples.append((t_img, p_abs))
            p_send = np.median(np.array([s[1] for s in self.samples]), axis=0)
            n = len(self.samples)
            std_f = max(self.std_min, std / np.sqrt(n))
            pkt = pack_target_relative(self.seq, self.sysid, self.compid, stamp_us, self.target_id,
                                       p_send.tolist(), [std_f, std_f, 2.0 * std_f], TARGET_OBS_FRAME_LOCAL_NED)
            self.n_comp += 1
        else:
            p_send = None
            pkt = pack_target_relative(self.seq, self.sysid, self.compid, stamp_us, self.target_id,
                                       p_frd.tolist(), [std, std, 2.0 * std])
        self.seq += 1
        try:
            self.sock.sendto(pkt, self.dest)
            self.n_sent += 1
        except OSError as e:
            self.get_logger().warn(f'MAVLink send failed: {e}', throttle_duration_sec=5.0)

        out = PointStamped()
        out.header.stamp = m.header.stamp
        out.header.frame_id = self.body_frame
        out.point.x, out.point.y, out.point.z = float(p_frd[0]), float(-p_frd[1]), float(-p_frd[2])
        self.pub.publish(out)
        mode = 'LOCAL_NED filt %d' % len(self.samples) if p_send is not None else 'BODY_FRD (no odom)'
        return f'SENT {len(obj) // 4} markers, {mode}', (rvec, tvec, p_frd, p_send)


def main():
    rclpy.init()
    node = ArucoLandingTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
