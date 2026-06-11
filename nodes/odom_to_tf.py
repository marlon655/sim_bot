#!/usr/bin/env python3
"""
odom_to_tf.py – converts /odom (nav_msgs/Odometry, 30 Hz, monotonic)
to a TF broadcast of odom -> base_footprint.

This replaces the gz_bridge TF bridge, which published Pose_V at physics rate
with non-monotonic timestamps, causing endless "jump back in time" TF errors.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self._br = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom', self._cb, 10)
        self.get_logger().info('odom_to_tf ready: publishing odom -> base_footprint')

    def _cb(self, msg: Odometry):
        t = TransformStamped()
        # Use current sim time instead of msg.header.stamp.
        # The odom message timestamp (sim time when pose was measured) can be
        # slightly BEFORE the current sim time; laser scans may arrive just
        # after the corresponding odom, causing slam_toolbox to request a TF
        # at a time beyond the latest cached entry → NoDataForExtrapolationException.
        # Stamping with "now" ensures TF is always available at or ahead of scans.
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg.header.frame_id          # 'odom'
        t.child_frame_id = msg.child_frame_id             # 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self._br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
