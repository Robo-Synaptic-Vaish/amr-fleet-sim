#!/usr/bin/env python3
"""
Passive Dual-AMR Fleet Monitor (Stage 3.1)
------------------------------------------
A passive, read-only ROS 2 monitoring node for tracking two autonomous
mobile robots (AMR 1 and AMR 2) in a Gazebo warehouse simulation.

Subscribed Topics:
  - /model/amr/odometry  (nav_msgs/msg/Odometry) -> Robot 1
  - /model/amr2/odometry (nav_msgs/msg/Odometry) -> Robot 2

Key Features:
  - Subscribes passively to odometry topics (NO velocity or control publishing).
  - Calculates estimated world/warehouse coordinates using spawn-offset calibration:
      AMR 1: world_x = -2.0 + odom_x, world_y = +0.8 + odom_y
      AMR 2: world_x = -2.0 + odom_x, world_y = -0.8 + odom_y
  - Computes inter-robot Euclidean distance from calibrated coordinates.
  - Enforces data freshness watchdog:
      * WAITING : Initial state before any odometry message is received.
      * STALE   : If no message has arrived for >= 1.0 second.
      * ONLINE  : Active message received within < 1.0 second.
  - Distance displays 'N/A' whenever either robot is not ONLINE.
  - ANSI terminal display refreshed at ~5 Hz without runaway scrolling.
"""

import math
import sys
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

# Spawn offsets as defined in sim.launch.py
AMR1_SPAWN_X = -2.0
AMR1_SPAWN_Y = 0.8

AMR2_SPAWN_X = -2.0
AMR2_SPAWN_Y = -0.8

# Watchdog timeout threshold in seconds
STALE_THRESHOLD_SEC = 1.0

# Display update rate (5 Hz -> 0.2s interval)
DISPLAY_TIMER_SEC = 0.2


class RobotState:
    """Encapsulates the state of an individual AMR."""

    def __init__(self, name: str, topic: str, spawn_x: float, spawn_y: float):
        self.name = name
        self.topic = topic
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y

        self.last_msg_time: Optional[float] = None
        self.odom_x: float = 0.0
        self.odom_y: float = 0.0
        self.linear_vel: float = 0.0
        self.angular_vel: float = 0.0

    def update_odometry(self, msg: Odometry, current_time: float) -> None:
        """Update robot state from received Odometry message."""
        self.last_msg_time = current_time
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

    def get_status(self, current_time: float) -> str:
        """Determine data freshness status: WAITING, STALE, or ONLINE."""
        if self.last_msg_time is None:
            return "WAITING"
        if (current_time - self.last_msg_time) >= STALE_THRESHOLD_SEC:
            return "STALE"
        return "ONLINE"

    def get_calibrated_position(self) -> Tuple[float, float]:
        """
        Calculate estimated world/warehouse position using spawn-offset calibration.
        NOTE: This is based on initial spawn offsets, NOT global sensor localization.
        """
        world_x = self.spawn_x + self.odom_x
        world_y = self.spawn_y + self.odom_y
        return world_x, world_y


class FleetMonitor(Node):
    """Passive ROS 2 node to monitor dual AMR fleet status and separation."""

    def __init__(self):
        super().__init__('fleet_monitor')

        # Initialize robot tracking objects
        self.amr1 = RobotState("AMR 1 (amr)", "/model/amr/odometry", AMR1_SPAWN_X, AMR1_SPAWN_Y)
        self.amr2 = RobotState("AMR 2 (amr2)", "/model/amr2/odometry", AMR2_SPAWN_X, AMR2_SPAWN_Y)

        # Create passive subscribers (QoS depth = 10)
        self._sub_amr1 = self.create_subscription(
            Odometry,
            self.amr1.topic,
            self._amr1_callback,
            10
        )
        self._sub_amr2 = self.create_subscription(
            Odometry,
            self.amr2.topic,
            self._amr2_callback,
            10
        )

        # Periodic timer for terminal display (~5 Hz)
        self._display_timer = self.create_timer(DISPLAY_TIMER_SEC, self._render_display)

        self.get_logger().info("Fleet Monitor initialized (Passive mode). Monitoring odometry streams...")

    def _amr1_callback(self, msg: Odometry) -> None:
        self.amr1.update_odometry(msg, time.monotonic())

    def _amr2_callback(self, msg: Odometry) -> None:
        self.amr2.update_odometry(msg, time.monotonic())

    def _calculate_distance(self, now: float) -> Tuple[Optional[float], str]:
        """
        Calculate inter-robot distance using spawn-offset-calibrated coordinates.
        Returns (distance_in_meters, reason_or_status).
        If either robot is WAITING or STALE, distance is None (displays N/A).
        """
        status1 = self.amr1.get_status(now)
        status2 = self.amr2.get_status(now)

        if status1 == "WAITING" or status2 == "WAITING":
            return None, "WAITING (one or both robots have not sent data yet)"
        if status1 == "STALE" and status2 == "STALE":
            return None, "STALE (both robots odometry streams are stale > 1.0s)"
        if status1 == "STALE":
            return None, "STALE (AMR 1 odometry stream is stale > 1.0s)"
        if status2 == "STALE":
            return None, "STALE (AMR 2 odometry stream is stale > 1.0s)"

        # Both are ONLINE -> Safe to calculate distance using calibrated coordinates
        x1, y1 = self.amr1.get_calibrated_position()
        x2, y2 = self.amr2.get_calibrated_position()

        dx = x2 - x1
        dy = y2 - y1
        dist = math.sqrt(dx * dx + dy * dy)
        return dist, "VALID (both robots ONLINE)"

    def _render_display(self) -> None:
        """Render terminal dashboard at ~5 Hz using in-place cursor repositioning."""
        now = time.monotonic()
        status1 = self.amr1.get_status(now)
        status2 = self.amr2.get_status(now)
        distance, dist_status = self._calculate_distance(now)

        def format_robot_row(robot: RobotState, status: str) -> str:
            if status == "WAITING":
                odom_str = "[   N/A   ,    N/A  ]"
                world_str = "[   N/A   ,    N/A  ]"
                vel_str = "N/A"
            else:
                odom_str = f"[{robot.odom_x:+7.3f}, {robot.odom_y:+7.3f}]"
                wx, wy = robot.get_calibrated_position()
                world_str = f"[{wx:+7.3f}, {wy:+7.3f}]"
                vel_str = f"lin={robot.linear_vel:+5.2f} m/s, ang={robot.angular_vel:+5.2f} rad/s"

            return (
                f"  {robot.name:<14} | {status:<7} | {odom_str:<18} | "
                f"{world_str:<18} | {vel_str}"
            )

        row1 = format_robot_row(self.amr1, status1)
        row2 = format_robot_row(self.amr2, status2)

        if distance is not None:
            dist_value_str = f"{distance:.3f} m"
        else:
            dist_value_str = "N/A"

        output = [
            "\033[H\033[J",  # Reposition cursor to top-left and clear screen
            "=" * 92,
            "                   DUAL-AMR FLEET MONITOR (STAGE 3.1 - PASSIVE)              ",
            "=" * 92,
            " CALIBRATION NOTE: World coordinates use spawn-offset calibration:",
            "                  AMR 1: [-2.0 + odom_x, +0.8 + odom_y]",
            "                  AMR 2: [-2.0 + odom_x, -0.8 + odom_y]",
            "                  (Estimated relative to spawn; not absolute SLAM/sensor localization)",
            "-" * 92,
            f"  {'ROBOT':<14} | {'STATUS':<7} | {'LOCAL ODOM [X, Y]':<18} | {'CALIB WORLD [X, Y]':<18} | VELOCITY",
            "-" * 92,
            row1,
            row2,
            "-" * 92,
            " INTER-ROBOT SEPARATION:",
            f"  Distance: {dist_value_str:<10}  |  Separation Status: {dist_status}",
            "=" * 92,
            " [Ctrl+C] to exit monitor  |  Refresh: 5 Hz  |  Safety: READ-ONLY (No /cmd_vel publishing)",
        ]

        sys.stdout.write("\n".join(output) + "\n")
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    monitor = FleetMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()
        print("\nFleet Monitor stopped safely.")


if __name__ == '__main__':
    main()
