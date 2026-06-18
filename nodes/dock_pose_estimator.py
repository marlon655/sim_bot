#!/usr/bin/env python3
"""
dock_pose_estimator.py — LiDAR-only
=====================================
Publishes /detected_dock_pose (geometry_msgs/PoseStamped) consumed by the
opennav_docking SimpleChargingDock plugin.

Uses only the frontal LiDAR sector of /scan to find the dock face cluster:

  - First pass (±sector_half_deg): finds min_range to the nearest object.
  - Second pass: uses wide sector (±close_sector_deg) when min_range < close_range_m
    to capture the full dock width (0.6 m) for accurate lateral centering.
  - face_x:   center-band average (±20°) — stable against oblique-ray under-estimation.
  - center_y: edge-to-edge midpoint of near-surface points.
  - Yaw fixed at 0.0 (dock face is perpendicular to X in odom) — more stable than SVD.

  When the dock enters the LiDAR blind zone (min_range < 0.30 m), the last
  computed target is re-published with a fresh timestamp so the docking server
  never loses the external detection pose and never falls back to the YAML pose.

Parameters:
  ~stop_distance     float  0.35  m   – target pose offset from dock face
  ~sector_half_deg   float  30.0  deg – frontal sector half-angle (normal)
  ~close_sector_deg  float  60.0  deg – wider sector for lateral edge detection
  ~close_range_m     float  0.5   m   – threshold to switch to wide sector
"""

import math
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PointStamped

import tf2_ros
import tf2_geometry_msgs  # registers PointStamped transformer
from tf2_ros import TransformException


class DockPoseEstimator(Node):

    def __init__(self):
        super().__init__('dock_pose_estimator')

        self.declare_parameter('stop_distance',    0.35)
        self.declare_parameter('sector_half_deg',  30.0)
        self.declare_parameter('close_sector_deg', 60.0)
        self.declare_parameter('close_range_m',    0.5)
        # Fixed dock Y in odom. The dock is static — using the detected center_y
        # introduces a bias equal to the robot's own Y offset at staging time.
        # Set to the dock's odom Y from the world/YAML (0.0 for this setup).
        self.declare_parameter('dock_y_odom', 0.0)

        self.STOP_DIST    = self.get_parameter('stop_distance').value
        self.SECTOR_HALF  = math.radians(self.get_parameter('sector_half_deg').value)
        self.CLOSE_SECTOR = math.radians(self.get_parameter('close_sector_deg').value)
        self.CLOSE_RANGE  = self.get_parameter('close_range_m').value
        self.DOCK_Y_ODOM  = self.get_parameter('dock_y_odom').value

        self.add_on_set_parameters_callback(self._on_param_change)

        self._filtered_yaw = None
        self._YAW_ALPHA    = 0.35   # EMA gain — lower = smoother but slower

        # Cache last computed target so we can keep publishing during LiDAR blind zone.
        self._cached_target_x: float | None = None
        self._cached_target_y: float | None = None
        self._cached_yaw: float | None = None

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(LaserScan, 'scan', self._scan_cb, 10)
        self.dock_pub = self.create_publisher(PoseStamped, 'detected_dock_pose', 10)

        self.get_logger().info(
            f'DockPoseEstimator (LiDAR-only) ready — '
            f'stop_dist={self.STOP_DIST} m  '
            f'sector={math.degrees(self.SECTOR_HALF):.0f}°/'
            f'{math.degrees(self.CLOSE_SECTOR):.0f}°  '
            f'close_range={self.CLOSE_RANGE} m')

    # ──────────────────────────────────────────────────────────────────────────
    def _on_param_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'stop_distance':
                self.STOP_DIST = p.value
                self.get_logger().info(f'stop_distance updated → {self.STOP_DIST}')
            elif p.name == 'dock_y_odom':
                self.DOCK_Y_ODOM = p.value
        return SetParametersResult(successful=True)

    # ──────────────────────────────────────────────────────────────────────────
    def _publish_cached(self):
        """Re-publish last known target with a fresh timestamp (blind-zone keepalive)."""
        if self._cached_target_x is None:
            return
        pose = self._make_pose_stamped(
            self._cached_target_x,
            self._cached_target_y,
            self._cached_yaw,
            self.get_clock().now().to_msg())
        self.dock_pub.publish(pose)
        self.get_logger().info(
            f'LiDAR blind zone — cached target '
            f'({self._cached_target_x:.3f}, {self._cached_target_y:.3f})',
            throttle_duration_sec=0.5)

    # ──────────────────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        # ── First pass: find min_range in normal frontal sector (±sector_half) ──
        # Use <= range_min to also exclude values clamped to range_min by the
        # Gazebo GPU lidar for objects in the blind zone (< 0.30 m from sensor).
        min_range = float('inf')
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > self.SECTOR_HALF:
                continue
            if r <= msg.range_min or r > min(msg.range_max, 2.5):
                continue
            min_range = min(min_range, r)

        # Blind zone: dock face entered LiDAR min_range — publish cached target.
        # Also catches Gazebo clamping all returns exactly to range_min.
        if not math.isfinite(min_range) or min_range <= msg.range_min:
            self._publish_cached()
            return

        # Wide sector when close — captures full dock width (0.6 m) at short range.
        # Use <= so the boundary distance (e.g., 0.5 m) switches to the wide sector.
        sector = self.CLOSE_SECTOR if min_range <= self.CLOSE_RANGE else self.SECTOR_HALF

        # ── Second pass: collect valid points in chosen sector ─────────────────
        cluster_pts = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) > sector:
                continue
            if r <= msg.range_min or r > min(msg.range_max, 2.5):
                continue
            cluster_pts.append((r * math.cos(angle), r * math.sin(angle)))

        if not cluster_pts:
            self._publish_cached()
            return

        # ── face_x: center-band average (±20°) ────────────────────────────────
        # With the wide sector (±60°) used at close range, oblique rays produce
        # x = r·cos(θ) values much smaller than center rays (cos 60°=0.5).
        # Using only rays within ±20° of straight-ahead gives a stable face_x.
        _CB = math.radians(20.0)
        center_band = [(x, y) for x, y in cluster_pts if abs(math.atan2(y, x)) <= _CB]
        if not center_band:
            center_band = cluster_pts

        cb_min_r = min(math.hypot(x, y) for x, y in center_band)
        cb_face  = [(x, y) for x, y in center_band
                    if math.hypot(x, y) <= cb_min_r + 0.10]
        face_x   = sum(p[0] for p in cb_face) / len(cb_face)

        # ── center_y: edge-to-edge midpoint of near-surface points ────────────
        # Keep points within 20 cm of the closest return for edge detection.
        face_pts  = [(x, y) for x, y in cluster_pts
                     if math.hypot(x, y) < min_range + 0.20]
        front_pts = [(x, y) for x, y in face_pts if x < face_x + 0.08]
        if len(front_pts) < 2:
            front_pts = face_pts

        y_vals   = [p[1] for p in front_pts]
        y_min, y_max = min(y_vals), max(y_vals)
        # Use midpoint when spread > 15 cm (flat face detected); mean otherwise.
        center_y = ((y_min + y_max) / 2.0
                    if y_max - y_min > 0.15
                    else sum(y_vals) / len(y_vals))

        # ── Dock face is perpendicular to X-axis in odom — fixed yaw=0 ────────
        yaw = self._smooth_yaw(0.0)

        # Build PointStamped in laser_frame, transform to odom
        pt_laser = PointStamped()
        pt_laser.header.frame_id = 'laser_frame'
        pt_laser.header.stamp    = rclpy.time.Time().to_msg()
        pt_laser.point.x = face_x
        pt_laser.point.y = center_y
        pt_laser.point.z = 0.0

        try:
            pt_odom = self.tf_buffer.transform(
                pt_laser, 'odom', timeout=Duration(seconds=0.15))
        except TransformException as e:
            self.get_logger().warn(f'LiDAR TF error: {e}', throttle_duration_sec=2.0)
            self._publish_cached()
            return

        ox = pt_odom.point.x
        oy = pt_odom.point.y

        target_x = ox - self.STOP_DIST * math.cos(yaw)
        # Use fixed dock Y (dock is static) — detected oy carries the robot's own
        # Y offset from staging and would drive the robot off-center.
        target_y = self.DOCK_Y_ODOM - self.STOP_DIST * math.sin(yaw)

        # Update cache before publishing — so blind-zone keepalive uses this value.
        self._cached_target_x = target_x
        self._cached_target_y = target_y
        self._cached_yaw      = yaw

        dock_pose = self._make_pose_stamped(
            target_x, target_y, yaw,
            self.get_clock().now().to_msg())

        self.dock_pub.publish(dock_pose)
        self.get_logger().info(
            f'LiDAR: face=({ox:.3f},{oy:.3f})  '
            f'target=({target_x:.3f},{target_y:.3f})  '
            f'range={min_range:.3f}  yaw={math.degrees(yaw):.1f}°',
            throttle_duration_sec=0.5)

    # ──────────────────────────────────────────────────────────────────────────
    def _smooth_yaw(self, yaw: float) -> float:
        """EMA com wrap-around correto (±π)."""
        if self._filtered_yaw is None:
            self._filtered_yaw = yaw
            return yaw
        diff = (yaw - self._filtered_yaw + math.pi) % (2 * math.pi) - math.pi
        self._filtered_yaw += self._YAW_ALPHA * diff
        return self._filtered_yaw

    @staticmethod
    def _make_pose_stamped(x: float, y: float, yaw: float, stamp) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id    = 'odom'
        ps.header.stamp       = stamp
        ps.pose.position.x    = x
        ps.pose.position.y    = y
        ps.pose.position.z    = 0.0
        ps.pose.orientation.z = math.sin(yaw / 2.0)
        ps.pose.orientation.w = math.cos(yaw / 2.0)
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
