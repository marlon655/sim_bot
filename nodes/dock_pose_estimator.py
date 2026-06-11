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

Phase 2 – LiDAR (short range, when camera loses marker at < ~0.3 m):
  Analyses the frontal ±30° sector of /scan to find the closest obstacle
  cluster (the dock face).  Uses the last known approach yaw from the ArUco
  phase so the robot keeps heading straight at the dock.

Parameters (ROS2, all overridable from the launch file):
  ~marker_id        int    771   ArUco marker ID to track
  ~marker_size      float  0.15  Physical size of printed tag [m]
  ~stop_distance    float  0.32  Offset from marker centre to robot target [m]
                                 ≈ robot half-length (0.27) + 5 cm clearance
  ~aruco_timeout    float  1.5   Seconds without ArUco before LiDAR takes over
  ~sector_half_deg  float  30.0  Half-width of frontal LiDAR sector [degrees]
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
        self.declare_parameter('marker_id',       771)
        self.declare_parameter('marker_size',     0.30)
        self.declare_parameter('stop_distance',   0.32)
        self.declare_parameter('aruco_timeout',   1.5)
        self.declare_parameter('sector_half_deg', 30.0)

        self.MARKER_ID     = self.get_parameter('marker_id').value
        self.MARKER_SIZE   = self.get_parameter('marker_size').value
        self.STOP_DIST     = self.get_parameter('stop_distance').value
        self.ARUCO_TIMEOUT = self.get_parameter('aruco_timeout').value
        sector_deg         = self.get_parameter('sector_half_deg').value
        self.SECTOR_HALF   = math.radians(sector_deg)

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
            f'stop_dist={self.STOP_DIST} m)')

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

        # Compute approach yaw = direction from robot toward marker
        yaw = self._approach_yaw(mx, my)
        self.last_yaw  = yaw
        self.last_aruco_t = self.get_clock().now()

        # Target pose: STOP_DIST metres before the marker
        dock_pose = self._make_pose_stamped(
            mx - self.STOP_DIST * math.cos(yaw),
            my - self.STOP_DIST * math.sin(yaw),
            yaw,
            self.get_clock().now().to_msg())

        self.dock_pub.publish(dock_pose)

    # ─────────────────────────────────────────────────────────────────────
    # Scan callback – LiDAR fallback (Phase 2)
    # ─────────────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        # Only activate when ArUco has been seen but is now lost
        if self.last_aruco_t is None:
            return
        elapsed = (self.get_clock().now() - self.last_aruco_t).nanoseconds * 1e-9
        if elapsed < self.ARUCO_TIMEOUT:
            return   # ArUco still fresh

        # Collect valid points in frontal sector (±SECTOR_HALF about angle=0)
        cluster_pts = []
        min_range   = float('inf')

        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > self.SECTOR_HALF:
                continue
            if r < msg.range_min or r > min(msg.range_max, 2.5):
                continue
            min_range = min(min_range, r)
            cluster_pts.append((r * math.cos(angle),
                                 r * math.sin(angle),
                                 r))

        if not cluster_pts:
            return

        # Keep only points within 15 cm of the minimum range (same obstacle)
        same_obj = [(x, y) for x, y, r in cluster_pts if r < min_range + 0.15]
        cx = sum(p[0] for p in same_obj) / len(same_obj)
        cy = sum(p[1] for p in same_obj) / len(same_obj)

        # Build PointStamped in laser_frame, transform to odom
        pt_laser = PointStamped()
        pt_laser.header.frame_id = 'laser_frame'
        pt_laser.header.stamp    = rclpy.time.Time().to_msg()
        pt_laser.point.x = cx
        pt_laser.point.y = cy
        pt_laser.point.z = 0.0

        try:
            pt_odom = self.tf_buffer.transform(
                pt_laser, 'odom',
                timeout=Duration(seconds=0.15))
        except TransformException as e:
            self.get_logger().warn(f'LiDAR TF error: {e}', throttle_duration_sec=2.0)
            return

        ox, oy = pt_odom.point.x, pt_odom.point.y
        yaw = self.last_yaw   # keep orientation from last ArUco phase

        dock_pose = self._make_pose_stamped(
            ox - self.STOP_DIST * math.cos(yaw),
            oy - self.STOP_DIST * math.sin(yaw),
            yaw,
            self.get_clock().now().to_msg())

        self.dock_pub.publish(dock_pose)
        self.get_logger().debug(
            f'LiDAR fallback: cluster at ({ox:.3f}, {oy:.3f})', throttle_duration_sec=1.0)

    # ─────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────
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
