#!/usr/bin/env python3
"""
dock_pose_estimator.py
======================
Publishes /detected_dock_pose (geometry_msgs/PoseStamped) consumed by the
opennav_docking SimpleChargingDock plugin.

Phase 1 – ArUco (long range, ~0.3 m → ~3 m):
  Detects ArUco marker ID=771 (DICT_4X4_1000) on the charging base via the
  RGB camera.  Computes the marker centre position in odom frame, then
  publishes a robot approach pose that is STOP_DISTANCE metres in front of
  the marker (so the robot stops ~5 cm from the base face).

Phase 2 – LiDAR (short range, when camera loses marker at < ~0.5 m):
  Analyses the frontal LiDAR sector of /scan to find the dock face cluster.
  Uses a wider sector (close_sector_deg) when min_range < 0.5 m to capture
  the full dock width (0.6 m) for accurate lateral centering.
  Computes face_x = min X of cluster (front face) and center_y = mean Y
  (lateral center of dock) so the robot stops precisely 5 cm from the face
  and is laterally aligned with the dock centre.

Parameters (ROS2, all overridable from the launch file):
  ~marker_id         int    771   ArUco marker ID to track
  ~marker_size       float  0.30  Physical size of printed tag [m]
  ~stop_distance     float  0.32  Offset from marker centre to robot target [m]
                                  ≈ robot half-length (0.27) + 5 cm clearance
  ~aruco_timeout     float  0.5   Seconds without ArUco before LiDAR takes over
  ~sector_half_deg   float  30.0  Half-width of frontal LiDAR sector [degrees]
  ~close_sector_deg  float  60.0  Wider sector used when min_range < 0.5 m
  ~close_range_m     float  0.5   Range threshold to switch to wide sector [m]
"""

import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped

from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs  # registers PoseStamped / PointStamped transformers
from tf2_ros import TransformException


class DockPoseEstimator(Node):

    def __init__(self):
        super().__init__('dock_pose_estimator')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('marker_id',        771)
        self.declare_parameter('marker_size',      0.30)
        self.declare_parameter('stop_distance',    0.32)
        self.declare_parameter('aruco_timeout',    0.5)
        self.declare_parameter('sector_half_deg',  30.0)
        self.declare_parameter('close_sector_deg', 60.0)
        self.declare_parameter('close_range_m',    0.5)

        self.MARKER_ID      = self.get_parameter('marker_id').value
        self.MARKER_SIZE    = self.get_parameter('marker_size').value
        self.STOP_DIST      = self.get_parameter('stop_distance').value
        self.ARUCO_TIMEOUT  = self.get_parameter('aruco_timeout').value
        sector_deg          = self.get_parameter('sector_half_deg').value
        close_sector_deg    = self.get_parameter('close_sector_deg').value
        self.SECTOR_HALF    = math.radians(sector_deg)
        self.CLOSE_SECTOR   = math.radians(close_sector_deg)
        self.CLOSE_RANGE    = self.get_parameter('close_range_m').value

        # ── ArUco detector (DICT_4X4_1000 includes ID 771)
        # OpenCV 4.6 legacy API: Dictionary_get + DetectorParameters_create
        self._aruco_dict   = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_1000)
        self._aruco_params = cv2.aruco.DetectorParameters_create()
        # Tuned for Gazebo rendering: lower threshold constant improves
        # detection when black/white contrast is reduced by ambient lighting.
        self._aruco_params.adaptiveThreshConstant = 3
        self._aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        # 3-D object points for marker corners (marker frame: Z=0 plane,
        # X right, Y up, origin at centre)
        h = self.MARKER_SIZE / 2.0
        self.obj_pts = np.array([
            [-h,  h, 0.0],   # top-left
            [ h,  h, 0.0],   # top-right
            [ h, -h, 0.0],   # bottom-right
            [-h, -h, 0.0],   # bottom-left
        ], dtype=np.float32)

        # ── Internal state ─────────────────────────────────────────────────
        self.camera_matrix = None
        self.dist_coeffs   = np.zeros((5, 1), dtype=np.float32)
        self.last_aruco_t  = None    # rclpy.Time of last successful detection
        self.last_yaw      = 0.0     # remembered approach yaw [rad]
        self._filtered_yaw  = None    # EMA state for yaw smoothing
        self._YAW_ALPHA     = 0.35    # EMA gain: lower = smoother but slower
        self.last_dock_odom = None    # (ox, oy) of dock face in odom from last ArUco

        # ── TF2 ───────────────────────────────────────────────────────────
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── CvBridge ──────────────────────────────────────────────────────
        self.bridge = CvBridge()

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(
            CameraInfo, 'camera/camera_info', self._camera_info_cb, 10)
        self.create_subscription(
            Image, 'camera/image', self._image_cb, 10)
        self.create_subscription(
            LaserScan, 'scan', self._scan_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────────
        self.dock_pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)

        self.get_logger().info(
            f'DockPoseEstimator ready  '
            f'(marker={self.MARKER_ID}, size={self.MARKER_SIZE} m, '
            f'stop_dist={self.STOP_DIST} m, '
            f'aruco_timeout={self.ARUCO_TIMEOUT} s, '
            f'sector={math.degrees(self.SECTOR_HALF):.0f}°/'
            f'{math.degrees(self.CLOSE_SECTOR):.0f}°)')

    # ─────────────────────────────────────────────────────────────────────
    # Camera info callback – store intrinsics once
    # ─────────────────────────────────────────────────────────────────────
    def _camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            if msg.d:
                self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    # ─────────────────────────────────────────────────────────────────────
    # Image callback – ArUco detection (Phase 1)
    # ─────────────────────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        if self.camera_matrix is None:
            return

        # Convert to grey-scale
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self._aruco_dict, parameters=self._aruco_params)
        if ids is None:
            return

        ids_flat = ids.flatten()
        if self.MARKER_ID not in ids_flat:
            return

        idx = int(np.where(ids_flat == self.MARKER_ID)[0][0])
        img_pts = corners[idx].reshape(4, 2).astype(np.float32)

        # Estimate marker pose in camera_link_optical frame
        ok, rvec, tvec = cv2.solvePnP(
            self.obj_pts, img_pts,
            self.camera_matrix, self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return

        tvec = tvec.flatten()

        # Build PointStamped in camera_link_optical
        # Use time=0 (latest available TF) to avoid "extrapolation into the future"
        # errors caused by camera timestamps arriving slightly ahead of the TF buffer.
        pt_cam = PointStamped()
        pt_cam.header.frame_id = 'camera_link_optical'
        pt_cam.header.stamp    = rclpy.time.Time().to_msg()
        pt_cam.point.x = float(tvec[0])
        pt_cam.point.y = float(tvec[1])
        pt_cam.point.z = float(tvec[2])

        # Transform marker centre to odom frame
        try:
            pt_odom = self.tf_buffer.transform(
                pt_cam, 'odom',
                timeout=Duration(seconds=0.15))
        except TransformException as e:
            self.get_logger().warn(f'ArUco TF error: {e}', throttle_duration_sec=2.0)
            return

        mx, my = pt_odom.point.x, pt_odom.point.y

        # Dock face normal is always 0° in odom (dock is static, face perpendicular to X).
        # Using a fixed yaw=0 is more stable than computing from noisy rvec.
        yaw = self._smooth_yaw(0.0)
        self.last_yaw       = yaw
        self.last_aruco_t   = self.get_clock().now()
        self.last_dock_odom = (mx, my)   # ArUco is truth for dock center

        # Target pose: STOP_DIST metres before the marker
        dock_pose = self._make_pose_stamped(
            mx - self.STOP_DIST * math.cos(yaw),
            my - self.STOP_DIST * math.sin(yaw),
            yaw,
            self.get_clock().now().to_msg())

        self.dock_pub.publish(dock_pose)
        self.get_logger().info(
            f'ArUco: dock=({mx:.3f},{my:.3f}) target=({dock_pose.pose.position.x:.3f},'
            f'{dock_pose.pose.position.y:.3f}) yaw={math.degrees(yaw):.1f}°',
            throttle_duration_sec=0.5)

    # ─────────────────────────────────────────────────────────────────────
    # Scan callback – LiDAR fallback (Phase 2)
    # ─────────────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        # If ArUco is currently fresh, let the image callback handle publishing.
        if self.last_aruco_t is not None:
            elapsed = (self.get_clock().now() - self.last_aruco_t).nanoseconds * 1e-9
            if elapsed < self.ARUCO_TIMEOUT:
                return  # ArUco still fresh — image_cb is publishing

        # First pass: find min_range using normal sector (±30°) to locate the dock
        min_range = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > self.SECTOR_HALF:
                continue
            if r < msg.range_min or r > min(msg.range_max, 2.5):
                continue
            min_range = min(min_range, r)

        if not math.isfinite(min_range):
            return

        # When close, use wider sector to capture full dock width (0.6 m) for
        # accurate lateral centering; otherwise use normal sector
        sector = self.CLOSE_SECTOR if min_range < self.CLOSE_RANGE else self.SECTOR_HALF

        # Second pass: collect all valid points in the chosen sector
        cluster_pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > sector:
                continue
            if r < msg.range_min or r > min(msg.range_max, 2.5):
                continue
            cluster_pts.append((r * math.cos(angle), r * math.sin(angle), r))

        if not cluster_pts:
            return

        # ── Face X: center-band average (±20°) for stable distance ────────────
        # With the wide sector (±60°) used at close range, oblique rays produce
        # x = r·cos(θ) values much smaller than the front-face center ray because
        # cos(60°)=0.5.  Using min(x) over all points therefore picks corner/side
        # readings and under-estimates the true face X, causing the target pose to
        # land too far from the dock.
        # Fix: use only rays within ±20° of straight-ahead (atan2(y,x)≈0°) to
        # compute face_x — these see the flat front face cleanly at all Y offsets.
        _CB = math.radians(20.0)
        center_band = [(x, y) for x, y, r in cluster_pts
                       if abs(math.atan2(y, x)) <= _CB]
        if not center_band:
            center_band = [(x, y) for x, y, r in cluster_pts]

        cb_min_r = min(math.hypot(x, y) for x, y in center_band)
        cb_face  = [(x, y) for x, y in center_band
                    if math.hypot(x, y) <= cb_min_r + 0.10]
        face_x   = sum(p[0] for p in cb_face) / len(cb_face)

        # ── Y center from full lateral spread ─────────────────────────────────
        # Keep all near-surface points from the wide sector for Y; more spread
        # gives a better edge-to-edge centre estimate.
        face_pts  = [(x, y) for x, y, r in cluster_pts if r < min_range + 0.20]
        front_pts = [(x, y) for x, y in face_pts if x < face_x + 0.08]
        if len(front_pts) < 2:
            front_pts = face_pts

        y_vals = [p[1] for p in front_pts]
        y_min, y_max = min(y_vals), max(y_vals)
        if y_max - y_min > 0.15:
            center_y = (y_min + y_max) / 2.0
        else:
            center_y = sum(y_vals) / len(y_vals)

        # Build PointStamped for the dock face centre in laser_frame
        pt_laser = PointStamped()
        pt_laser.header.frame_id = 'laser_frame'
        pt_laser.header.stamp    = rclpy.time.Time().to_msg()
        pt_laser.point.x = face_x
        pt_laser.point.y = center_y
        pt_laser.point.z = 0.0

        try:
            pt_odom = self.tf_buffer.transform(
                pt_laser, 'odom',
                timeout=Duration(seconds=0.15))
        except TransformException as e:
            self.get_logger().warn(f'LiDAR TF error: {e}', throttle_duration_sec=2.0)
            return

        # X (stop distance): always from LiDAR face detection
        ox = pt_odom.point.x

        # Y (lateral centering): ArUco is the truth — use last known dock Y.
        # LiDAR edge detection only when ArUco was never seen (e.g., too dark).
        if self.last_dock_odom is not None:
            oy = self.last_dock_odom[1]
        else:
            oy = pt_odom.point.y  # LiDAR fallback (no ArUco ever)

        # Dock face is always at 0° in odom — fixed yaw is more stable than SVD noise.
        yaw = self._smooth_yaw(0.0)

        dock_pose = self._make_pose_stamped(
            ox - self.STOP_DIST * math.cos(yaw),
            oy - self.STOP_DIST * math.sin(yaw),
            yaw,
            self.get_clock().now().to_msg())

        self.dock_pub.publish(dock_pose)
        self.get_logger().info(
            f'LiDAR: face_x={ox:.3f} dock_y={oy:.3f} '
            f'({"ArUco" if self.last_dock_odom else "LiDAR"}) '
            f'range={min_range:.3f} yaw={math.degrees(yaw):.1f}°',
            throttle_duration_sec=0.5)

    # ─────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────
    def _face_normal_yaw(self, face_pts: list) -> 'float | None':
        """Yaw da normal à face da dock calculado por SVD nos pontos LiDAR.

        Usa a menor componente de variância (SVD) para encontrar a direção
        perpendicular à linha de pontos que formam a face frontal da dock.
        Retorna None se houver pontos insuficientes para fitting confiável.
        """
        if len(face_pts) < 3:
            return None
        pts = np.array(face_pts, dtype=float)      # shape (N, 2): [x, y] laser_frame
        centroid = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - centroid, full_matrices=False)
        # vt[1] = direção de menor variância = normal à face da dock
        normal = vt[1]
        # Normal deve apontar DO dock PARA o robô; em laser_frame o dock está
        # à frente (+x), logo a normal correta aponta em −x.
        if normal[0] > 0:
            normal = -normal
        robot_yaw = self._robot_yaw()
        return robot_yaw + math.atan2(normal[1], normal[0])

    def _robot_yaw(self) -> float:
        """Yaw atual do robô em odom via TF odom→base_footprint."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time())
            qz = tf.transform.rotation.z
            qw = tf.transform.rotation.w
            return 2.0 * math.atan2(qz, qw)
        except TransformException:
            return 0.0

    def _smooth_yaw(self, yaw: float) -> float:
        """Filtro EMA no yaw com tratamento correto de wrap-around (±π)."""
        if self._filtered_yaw is None:
            self._filtered_yaw = yaw
            return yaw
        diff = (yaw - self._filtered_yaw + math.pi) % (2 * math.pi) - math.pi
        self._filtered_yaw += self._YAW_ALPHA * diff
        return self._filtered_yaw

    def _approach_yaw(self, target_x: float, target_y: float) -> float:
        """Yaw angle (rad, odom frame) the robot must face to approach target."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time())
            rx = tf.transform.translation.x
            ry = tf.transform.translation.y
        except TransformException:
            rx, ry = 0.0, 0.0
        return math.atan2(target_y - ry, target_x - rx)

    @staticmethod
    def _make_pose_stamped(x: float, y: float, yaw: float, stamp) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id      = 'odom'
        ps.header.stamp         = stamp
        ps.pose.position.x      = x
        ps.pose.position.y      = y
        ps.pose.position.z      = 0.0
        ps.pose.orientation.z   = math.sin(yaw / 2.0)
        ps.pose.orientation.w   = math.cos(yaw / 2.0)
        return ps


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
