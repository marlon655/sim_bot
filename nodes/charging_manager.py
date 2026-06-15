#!/usr/bin/env python3
"""
charging_manager.py
===================
Orchestrates the full autonomous charging cycle for palmares_bot.

Cycle:
  1. /go_charge (std_msgs/Empty) triggers the sequence
  2. NavigateToPose → staging area 1 m in front of dock (1.0, 0.0, yaw=0)
  3. DockRobot      → dock_pose_estimator guides approach via ArUco + LiDAR
  4. 15 s charging timer
  5. UndockRobot    → robot backs away to staging
  6. NavigateToPose → return to home corner (−3.0, −3.0)

Publishes /charging_manager/state (std_msgs/String) for monitoring.
"""

import math
import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Empty, String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose, DockRobot, UndockRobot


# ── State machine states ───────────────────────────────────────────────────────
IDLE                  = 'IDLE'
NAVIGATING_TO_STAGING = 'NAVIGATING_TO_STAGING'
CENTERING             = 'CENTERING'   # lateral correction before docking approach
DOCKING               = 'DOCKING'
CHARGING              = 'CHARGING'
UNDOCKING             = 'UNDOCKING'
RETURNING             = 'RETURNING'

# Lateral correction parameters
CENTERING_SAMPLE_TIME  = 2.0   # s to collect detected_dock_pose samples
CENTERING_DEAD_BAND    = 0.05  # m — skip correction if error < 5 cm
CENTERING_MAX_CORRECT  = 0.30  # m — clamp correction to avoid over-steering

# ── Key poses (odom frame) ─────────────────────────────────────────────────────
# Robot spawns at world (-3,-3) = odom (0,0). Dock at world (2,0) = odom (5,3).
# Staging: 1 m before dock face in odom X direction → odom (4, 3).
STAGING_X   =  4.0
STAGING_Y   =  2.97   # 3 cm right of dock centre (odom Y=3.0 → 2.97)
STAGING_YAW =  0.0    # face the dock (dock is at odom (5,3), staging at (4,3) → +X)

# Home: near spawn odom (0,0). Yaw faces away from dock (lower-left direction).
HOME_X   =  0.0
HOME_Y   =  0.0
HOME_YAW = -2.356     # ≈ -135° → faces away from dock toward spawn corner

CHARGE_SECONDS = 15.0

# ── Alignment check after docking ─────────────────────────────────────────────
DOCK_Y_ODOM     = 2.97             # expected robot Y when docked (odom frame)
ALIGN_TOL_Y     = 0.02             # m — max lateral error accepted
ALIGN_TOL_YAW   = math.radians(1.0)  # rad — max yaw error (1 degree)
DOCK_MAX_RETRIES = 3               # max realign attempts before aborting


def _make_pose(x: float, y: float, yaw: float) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id  = 'odom'
    ps.pose.position.x  = x
    ps.pose.position.y  = y
    ps.pose.position.z  = 0.0
    ps.pose.orientation.z = math.sin(yaw / 2.0)
    ps.pose.orientation.w = math.cos(yaw / 2.0)
    return ps


class ChargingManager(Node):

    def __init__(self):
        super().__init__('charging_manager')

        self._state = IDLE
        self._charge_timer = None
        self._centering_samples: list = []
        self._centering_timer  = None
        self._centering_sub    = None

        # Alignment tracking
        self._robot_y   : float | None = None
        self._robot_yaw : float | None = None
        self._dock_retries  = 0
        self._after_undock  = RETURNING   # RETURNING (normal) or CENTERING (retry)

        # Use ReentrantCallbackGroup so action callbacks don't block subscribers
        cb = ReentrantCallbackGroup()

        # ── Subscribers ────────────────────────────────────────────────────
        self.create_subscription(Empty, '/go_charge', self._go_charge_cb, 10,
                                 callback_group=cb)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10,
                                 callback_group=cb)

        # ── Publishers ─────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/charging_manager/state', 10)
        self._vel_pub   = self.create_publisher(Twist,  '/cmd_vel', 10)

        # ── Action clients ─────────────────────────────────────────────────
        self._nav_client   = ActionClient(self, NavigateToPose, 'navigate_to_pose',
                                          callback_group=cb)
        self._dock_client  = ActionClient(self, DockRobot,      'dock_robot',
                                          callback_group=cb)
        self._undock_client = ActionClient(self, UndockRobot,   'undock_robot',
                                           callback_group=cb)

        self.get_logger().info('ChargingManager ready. Publish /go_charge to start.')

    # ─────────────────────────────────────────────────────────────────────────
    # Helpers
    # ─────────────────────────────────────────────────────────────────────────
    def _set_state(self, state: str):
        self._state = state
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)
        self.get_logger().info(f'[ChargingManager] → {state}')

    def _abort(self, reason: str):
        self.get_logger().error(f'[ChargingManager] ABORTED: {reason}')
        self._set_state(IDLE)

    # ─────────────────────────────────────────────────────────────────────────
    # Step 1 — trigger
    # ─────────────────────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self._robot_y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._robot_yaw = 2.0 * math.atan2(qz, qw)

    def _is_aligned(self) -> bool:
        if self._robot_y is None or self._robot_yaw is None:
            return True   # no data yet — give benefit of the doubt
        y_err   = abs(self._robot_y - DOCK_Y_ODOM)
        yaw_err = abs((self._robot_yaw + math.pi) % (2 * math.pi) - math.pi)
        return y_err <= ALIGN_TOL_Y and yaw_err <= ALIGN_TOL_YAW

    def _go_charge_cb(self, _msg: Empty):
        if self._state != IDLE:
            self.get_logger().warn(
                f'Received /go_charge but already in state {self._state}, ignoring.')
            return
        self.get_logger().info('Received /go_charge — starting charging cycle.')
        self._dock_retries = 0
        self._navigate_to_staging()

    # ─────────────────────────────────────────────────────────────────────────
    # Step 2 — navigate to staging area
    # ─────────────────────────────────────────────────────────────────────────
    def _navigate_to_staging(self):
        self._set_state(NAVIGATING_TO_STAGING)

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self._abort('NavigateToPose action server not available')
            return

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(STAGING_X, STAGING_Y, STAGING_YAW)
        # Use Time(0) so bt_navigator uses the latest available TF.
        # Using get_clock().now() causes "Initial robot pose not available"
        # when the TF buffer was recently cleared by a SLAM jump-back-in-time.
        goal.pose.header.stamp = rclpy.time.Time().to_msg()

        self.get_logger().info(
            f'Navigating to staging pose ({STAGING_X}, {STAGING_Y})...')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._staging_goal_accepted_cb)

    def _staging_goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._abort('NavigateToPose goal rejected')
            return
        handle.get_result_async().add_done_callback(self._staging_result_cb)

    def _staging_result_cb(self, future):
        result = future.result()
        status = result.status
        # action_msgs/GoalStatus: SUCCEEDED = 4
        if status != 4:
            self._abort(f'NavigateToPose failed with status {status}')
            return
        self.get_logger().info('Reached staging area. Starting lateral centering.')
        self._start_centering()

    # ─────────────────────────────────────────────────────────────────────────
    # Step 2b — lateral centering (pre-align on dock face before approach)
    # ─────────────────────────────────────────────────────────────────────────
    def _start_centering(self):
        self._set_state(CENTERING)
        self._centering_samples = []
        self._centering_sub = self.create_subscription(
            PoseStamped, '/detected_dock_pose', self._centering_pose_cb, 10)
        # Collect for CENTERING_SAMPLE_TIME seconds then apply correction
        self._centering_timer = self.create_timer(
            CENTERING_SAMPLE_TIME, self._centering_done)

    def _centering_pose_cb(self, msg: PoseStamped):
        if self._state != CENTERING:
            return
        self._centering_samples.append(msg.pose.position.y)

    def _centering_done(self):
        self._centering_timer.cancel()
        self._centering_timer = None
        if self._centering_sub is not None:
            self.destroy_subscription(self._centering_sub)
            self._centering_sub = None

        if not self._centering_samples:
            self.get_logger().warn('No dock pose samples during centering — skipping correction.')
            self._start_docking()
            return

        dock_y = sum(self._centering_samples) / len(self._centering_samples)
        error  = dock_y - STAGING_Y
        self.get_logger().info(
            f'Centering: dock_y={dock_y:.3f} m, error={error:+.3f} m '
            f'(n={len(self._centering_samples)})')

        if abs(error) < CENTERING_DEAD_BAND:
            self.get_logger().info('Lateral error within dead-band — no correction needed.')
            self._start_docking()
            return

        # Clamp and apply correction: navigate to corrected Y at staging X
        corrected_y = STAGING_Y + max(-CENTERING_MAX_CORRECT,
                                       min(CENTERING_MAX_CORRECT, error))
        self.get_logger().info(f'Correcting lateral position to y={corrected_y:.3f} m')

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(STAGING_X, corrected_y, STAGING_YAW)
        goal.pose.header.stamp = rclpy.time.Time().to_msg()
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._centering_nav_accepted_cb)

    def _centering_nav_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Centering correction goal rejected — proceeding to dock.')
            self._start_docking()
            return
        handle.get_result_async().add_done_callback(self._centering_nav_result_cb)

    def _centering_nav_result_cb(self, future):
        result = future.result()
        if result.status != 4:
            self.get_logger().warn(
                f'Centering nav ended with status {result.status} — proceeding to dock.')
        else:
            self.get_logger().info('Lateral centering correction complete.')
        self._start_docking()

    # ─────────────────────────────────────────────────────────────────────────
    # Step 3 — dock
    # ─────────────────────────────────────────────────────────────────────────
    def _start_docking(self):
        self._set_state(DOCKING)

        if not self._dock_client.wait_for_server(timeout_sec=10.0):
            self._abort('DockRobot action server not available')
            return

        goal = DockRobot.Goal()
        goal.use_dock_id              = True
        goal.dock_id                  = 'base_carregamento'
        goal.navigate_to_staging_pose = False  # already at staging

        self.get_logger().info('Sending DockRobot goal...')
        future = self._dock_client.send_goal_async(goal)
        future.add_done_callback(self._dock_goal_accepted_cb)

    def _dock_goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._abort('DockRobot goal rejected')
            return
        handle.get_result_async().add_done_callback(self._dock_result_cb)

    def _dock_result_cb(self, future):
        result = future.result()
        status = result.status
        if status != 4:
            error_code = result.result.error_code if result.result else '?'
            self._abort(f'DockRobot failed — status={status}, error_code={error_code}')
            return

        y_err   = abs((self._robot_y or 0.0) - DOCK_Y_ODOM)
        yaw_err = math.degrees(abs((self._robot_yaw or 0.0 + math.pi) % (2 * math.pi) - math.pi))
        if not self._is_aligned():
            self._dock_retries += 1
            self.get_logger().warn(
                f'Docked but misaligned — y_err={y_err:.3f} m (tol {ALIGN_TOL_Y} m), '
                f'yaw_err={yaw_err:.1f}° (tol 1°) — '
                f'retry {self._dock_retries}/{DOCK_MAX_RETRIES}')
            if self._dock_retries >= DOCK_MAX_RETRIES:
                self._abort(f'Alignment failed after {DOCK_MAX_RETRIES} attempts — aborting')
                return
            self._after_undock = CENTERING
            self._start_undocking()
            return

        self.get_logger().info(
            f'Docked and aligned ✓  y_err={y_err:.3f} m  yaw_err={yaw_err:.1f}°  '
            f'Starting charge timer.')
        self._dock_retries = 0
        self._after_undock = RETURNING
        self._start_charging()

    # ─────────────────────────────────────────────────────────────────────────
    # Step 4 — charge for 15 s
    # ─────────────────────────────────────────────────────────────────────────
    def _start_charging(self):
        self._set_state(CHARGING)
        self._charge_elapsed = 0
        self._charge_timer = self.create_timer(1.0, self._charging_tick)

    def _charging_tick(self):
        self._charge_elapsed += 1
        self._vel_pub.publish(Twist())   # keep wheels stopped during charge
        self.get_logger().info(
            f'Charging... {self._charge_elapsed}/{int(CHARGE_SECONDS)} s')
        if self._charge_elapsed >= int(CHARGE_SECONDS):
            self._charge_timer.cancel()
            self._charge_timer = None
            self.get_logger().info('Charge complete. Undocking.')
            self._start_undocking()

    # ─────────────────────────────────────────────────────────────────────────
    # Step 5 — undock
    # ─────────────────────────────────────────────────────────────────────────
    def _start_undocking(self):
        self._set_state(UNDOCKING)

        if not self._undock_client.wait_for_server(timeout_sec=10.0):
            self._abort('UndockRobot action server not available')
            return

        goal = UndockRobot.Goal()
        goal.dock_type = 'charging_dock'

        self.get_logger().info('Sending UndockRobot goal...')
        future = self._undock_client.send_goal_async(goal)
        future.add_done_callback(self._undock_goal_accepted_cb)

    def _undock_goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._abort('UndockRobot goal rejected')
            return
        handle.get_result_async().add_done_callback(self._undock_result_cb)

    def _undock_result_cb(self, future):
        result = future.result()
        status = result.status
        if status != 4:
            self._abort(f'UndockRobot failed with status {status}')
            return
        if self._after_undock == CENTERING:
            self.get_logger().info('Undocked for realignment — restarting centering.')
            self._after_undock = RETURNING
            self._start_centering()
        else:
            self.get_logger().info('Undocking successful. Returning home.')
            self._return_home()

    # ─────────────────────────────────────────────────────────────────────────
    # Step 6 — return to home corner
    # ─────────────────────────────────────────────────────────────────────────
    def _return_home(self):
        self._set_state(RETURNING)

        if not self._nav_client.wait_for_server(timeout_sec=10.0):
            self._abort('NavigateToPose action server not available (return)')
            return

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(HOME_X, HOME_Y, HOME_YAW)
        goal.pose.header.stamp = rclpy.time.Time().to_msg()

        self.get_logger().info(f'Navigating home ({HOME_X}, {HOME_Y})...')
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._home_goal_accepted_cb)

    def _home_goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self._abort('Return NavigateToPose goal rejected')
            return
        handle.get_result_async().add_done_callback(self._home_result_cb)

    def _home_result_cb(self, future):
        result = future.result()
        status = result.status
        if status != 4:
            self.get_logger().warn(f'Return navigation ended with status {status}')
        else:
            self.get_logger().info('Returned home successfully.')
        self._set_state(IDLE)
        self.get_logger().info(
            '\n' + '=' * 68 + '\n'
            '  CICLO CONCLUÍDO — robô em IDLE\n'
            '=' * 68 + '\n'
            '\n'
            '  Terminal 1 (já rodando — não fechar):\n'
            '    cd ~/sim_ws && source install/setup.bash\n'
            '    ros2 launch sim_bot palmares_bot.launch.py\n'
            '\n'
            '  Terminal 2 — disparar novo ciclo de carga:\n'
            '    source ~/sim_ws/install/setup.bash\n'
            "    ros2 topic pub --once /go_charge std_msgs/msg/Empty '{}'\n"
            '\n'
            '  Terminal 2 — monitorar estado do ciclo:\n'
            '    ros2 topic echo /charging_manager/state\n'
            '\n'
            '  Terminal 2 — diagnóstico (opcional):\n'
            '    ros2 topic echo /detected_dock_pose\n'
            '    ros2 topic echo /odom --once\n'
            '    ros2 topic hz /camera/image\n'
            + '=' * 68
        )


def main(args=None):
    rclpy.init(args=args)
    node = ChargingManager()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
