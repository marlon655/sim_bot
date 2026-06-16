#!/usr/bin/env python3
"""
One-shot alignment check — coleta dados por MAX_WAIT_S e imprime relatório.
Uso:
  python3 ~/sim_ws/src/sim_bot/nodes/alignment_test.py
  watch -n 1 "python3 ~/sim_ws/src/sim_bot/nodes/alignment_test.py"
"""

import math
import sys
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

DOCK_Y_ODOM = 0.0   # dock em odom(1,0) — spawn world(1,0)=odom(0,0)
DOCK_YAW    = 0.0
DOCK_Z      = 0.0
TOL_Y       = 0.02
TOL_YAW     = 5.0
TOL_Z       = 0.05
MAX_WAIT_S  = 2.0


class _Collector(Node):

    def __init__(self):
        super().__init__('alignment_test_oneshot')
        self.robot_x = self.robot_y = self.robot_z = self.robot_yaw = None
        self.dock_y = None
        self.create_subscription(Odometry,    '/odom',               self._odom_cb, 10)
        self.create_subscription(PoseStamped, '/detected_dock_pose', self._dock_cb, 10)

    def _odom_cb(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_z = msg.pose.pose.position.z
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_yaw = 2.0 * math.atan2(qz, qw)

    def _dock_cb(self, msg):
        self.dock_y = msg.pose.position.y

    def has_data(self):
        return self.robot_y is not None


def main():
    rclpy.init()
    node = _Collector()
    deadline = time.time() + MAX_WAIT_S
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.has_data():
            break

    if not node.has_data():
        print("ERRO: sem dados do /odom (ROS2 rodando?)")
        node.destroy_node()
        rclpy.try_shutdown()
        sys.exit(1)

    rx, ry, rz = node.robot_x, node.robot_y, node.robot_z
    ryaw_rad    = node.robot_yaw
    ryaw_deg    = math.degrees(ryaw_rad)
    dock_y_det  = node.dock_y

    y_err   = ry - DOCK_Y_ODOM
    z_err   = rz - DOCK_Z
    yaw_err = ryaw_deg - math.degrees(DOCK_YAW)

    y_ok   = abs(y_err)   < TOL_Y
    z_ok   = abs(z_err)   < TOL_Z
    yaw_ok = abs(yaw_err) < TOL_YAW
    ok     = y_ok and z_ok and yaw_ok

    status = "ALINHADO ✓" if ok else "FORA DO ALINHAMENTO ✗"
    sep    = "─" * 54

    dock_y_str = f"{dock_y_det:.4f} m" if dock_y_det is not None else "não detectado"

    print(f"\n  ┌{sep}┐")
    print(f"  │  ALINHAMENTO COM DOCK                                │")
    print(f"  ├{sep}┤")
    print(f"  │  robot_x  = {rx:+.4f} m   (X deve diferir da dock)     │")
    print(f"  │  robot_y  = {ry:+.4f} m   dock_y={DOCK_Y_ODOM:.2f}  erro={y_err:+.4f} m  {'✓' if y_ok else '✗':<4}│")
    print(f"  │  robot_z  = {rz:+.4f} m   dock_z={DOCK_Z:.2f}   erro={z_err:+.4f} m  {'✓' if z_ok else '✗':<4}│")
    print(f"  │  robot_yaw= {ryaw_deg:+.2f}°   dock_yaw={math.degrees(DOCK_YAW):.1f}°  erro={yaw_err:+.2f}°  {'✓' if yaw_ok else '✗':<4}│")
    print(f"  │  detect_y = {dock_y_str:<46}│")
    print(f"  ├{sep}┤")
    print(f"  │  STATUS: {status:<45}│")
    print(f"  └{sep}┘\n")

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
