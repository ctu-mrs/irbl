#!/usr/bin/env python3
"""ArUco landing-target detector for PX4's built-in precision landing.

Detects one ArUco marker in a downward camera image, estimates its position relative to the
vehicle and sends it to PX4 as MAVLink TARGET_RELATIVE (development dialect, id 511). With
VTE_EN=0, PX4's mavlink_receiver turns each one directly into landing_target_pose (absolute target
position in the local frame), which drives the navigator's precision landing (AUTO_PRECLAND, e.g.
px4_bridge ~/precland); with VTE_EN=1 it goes to the vision_target_estimator instead
(fiducial_marker_pos_report). No PX4 change is needed: the px4_sitl_default build already has the
development dialect.

  sub  image        sensor_msgs/Image       (rgb8 / bgr8 / mono8)
  sub  camera_info  sensor_msgs/CameraInfo  (intrinsics; until it arrives, nothing is detected)
  pub  ~/target     geometry_msgs/PointStamped  marker position in <body_frame> (FLU), for rviz
  UDP  TARGET_RELATIVE -> mavlink_host:mavlink_port (PX4 SITL instance i: onboard link 14580+i)

The measurement goes out in TARGET_OBS_FRAME_BODY_FRD with q_sensor left zero, so PX4 rotates it
into NED with its own attitude estimate. The camera is assumed to look straight down with the
image top towards the vehicle's front (x500_lidar_3d_cam_down): optical (x right, y down,
z forward) -> FRD: x = -y_opt, y = x_opt, z = z_opt, plus camera_offset_frd.

pymavlink's bundled development dialect predates TARGET_RELATIVE, so the MAVLink 2 frame is
packed here (field order / CRC_EXTRA from the message definition, checked against PX4's
development.xml with pymavlink's mavparse: wire length 71, crc_extra 28).
"""

import socket
import struct

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

TARGET_RELATIVE_ID = 511
TARGET_RELATIVE_CRC_EXTRA = 28
TARGET_OBS_FRAME_BODY_FRD = 1
LANDING_TARGET_TYPE_VISION_FIDUCIAL = 2


def x25crc(data: bytes, crc: int = 0xFFFF) -> int:
    for b in data:
        tmp = b ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def pack_target_relative(seq, sysid, compid, timestamp_us, target_id, pos_frd, pos_std):
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
        TARGET_OBS_FRAME_BODY_FRD,
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
        dict_name = self.declare_parameter('dictionary', 'DICT_4X4_50').value
        self.cam_offset = np.array(self.declare_parameter('camera_offset_frd', [-0.06, 0.0, 0.06]).value)
        self.body_frame = self.declare_parameter('body_frame', 'uav1/fcu').value
        host = self.declare_parameter('mavlink_host', '127.0.0.1').value
        port = self.declare_parameter('mavlink_port', 14581).value
        self.sysid = self.declare_parameter('mavlink_sysid', 2).value     # = the vehicle's MAV_SYS_ID
        self.compid = self.declare_parameter('mavlink_compid', 191).value  # MAV_COMP_ID_ONBOARD_COMPUTER
        self.std_min = self.declare_parameter('pos_std_min', 0.02).value   # [m]
        self.std_per_m = self.declare_parameter('pos_std_per_meter', 0.02).value

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
        self.create_subscription(CameraInfo, 'camera_info', self.cb_info, qos_profile_sensor_data)
        self.create_subscription(Image, 'image', self.cb_image, qos_profile_sensor_data)
        self.create_timer(5.0, self.cb_status)
        self.get_logger().info(
            f'ArUco {dict_name} id {self.marker_id} ({self.marker_size:.2f} m) -> TARGET_RELATIVE to '
            f'{host}:{port} (sysid {self.sysid})')

    def cb_info(self, m: CameraInfo):
        if self.K is None:
            self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(m.d, dtype=np.float64) if len(m.d) else np.zeros(5)
            self.get_logger().info(f'camera intrinsics: fx {self.K[0, 0]:.1f} cx {self.K[0, 2]:.1f} '
                                   f'cy {self.K[1, 2]:.1f} ({m.width}x{m.height})')

    def cb_status(self):
        self.get_logger().info(f'{self.n_frames} frames, marker sent {self.n_sent} times in the last 5 s')
        self.n_frames = 0
        self.n_sent = 0

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
        if ids is None:
            return
        idx = np.where(ids.ravel() == self.marker_id)[0]
        if len(idx) == 0:
            return
        c = corners[idx[0]].reshape(4, 2).astype(np.float64)
        h = self.marker_size / 2.0
        obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
        ok, _, tvec = cv2.solvePnP(obj, c, self.K, self.D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return
        t = tvec.ravel()
        p_frd = np.array([-t[1], t[0], t[2]]) + self.cam_offset
        dist = float(np.linalg.norm(t))
        std = max(self.std_min, self.std_per_m * dist)

        stamp_us = m.header.stamp.sec * 1_000_000 + m.header.stamp.nanosec // 1000
        pkt = pack_target_relative(self.seq, self.sysid, self.compid, stamp_us, self.marker_id,
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
