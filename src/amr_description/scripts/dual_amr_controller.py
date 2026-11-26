#!/usr/bin/env python3
"""
Dual AMR Controller Node (Phase 2)
-----------------------------------
A beginner-friendly ROS 2 node to independently and simultaneously control
two autonomous mobile robots (AMR1 and AMR2) in Gazebo warehouse simulation.

Published Topics:
  - /model/amr/cmd_vel  (geometry_msgs/msg/Twist) -> Commands for Robot 1
  - /model/amr2/cmd_vel (geometry_msgs/msg/Twist) -> Commands for Robot 2
"""

import os
import sys
import time
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Configurable default velocities
DEFAULT_LINEAR_SPEED = 0.3    # Forward/backward speed in meters per second (m/s)
DEFAULT_ANGULAR_SPEED = 0.5   # Turning speed in radians per second (rad/s)
COMMAND_PUBLISH_RATE = 10.0   # Timer frequency in Hertz (10 updates per second)


class DualAmrController(Node):
    """
    ROS 2 Node that manages separate publishers for AMR1 and AMR2,
    and publishes continuous velocity commands to keep simulated motion smooth.
    """
    def __init__(self):
        super().__init__('dual_amr_controller')

        # Declare ROS 2 parameters so speeds can be adjusted at launch if needed
        self.declare_parameter('linear_speed', DEFAULT_LINEAR_SPEED)
        self.declare_parameter('angular_speed', DEFAULT_ANGULAR_SPEED)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        # -------------------------------------------------------------
        # ROS 2 Publishers
        # -------------------------------------------------------------
        # In ROS 2, a Publisher sends messages to a specific named topic.
        # Queue size 10 means ROS will buffer up to 10 outgoing messages.
        self.amr1_pub = self.create_publisher(Twist, '/model/amr/cmd_vel', 10)
        self.amr2_pub = self.create_publisher(Twist, '/model/amr2/cmd_vel', 10)

        # Store the current motion state for each robot
        self.amr1_cmd = Twist()
        self.amr2_cmd = Twist()

        # Human-readable state descriptions for the terminal interface
        self.amr1_status = "STOPPED"
        self.amr2_status = "STOPPED"

        # -------------------------------------------------------------
        # Periodic Timer (10 Hz)
        # -------------------------------------------------------------
        # In Gazebo physics, sending a continuous stream of Twist commands
        # ensures motors run smoothly without stuttering or coasting to a stop.
        timer_period = 1.0 / COMMAND_PUBLISH_RATE
        self.publish_timer = self.create_timer(timer_period, self.timer_publish_callback)

        self.get_logger().info('Dual AMR Controller initialized.')
        self.get_logger().info(f'Speeds: Linear = {self.linear_speed} m/s, Angular = {self.angular_speed} rad/s')

    def timer_publish_callback(self):
        """Called automatically every 0.1s by ROS 2 timer."""
        self.amr1_pub.publish(self.amr1_cmd)
        self.amr2_pub.publish(self.amr2_cmd)

    # -----------------------------------------------------------------
    # Robot 1 Control Functions
    # -----------------------------------------------------------------
    def amr1_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        self.amr1_cmd = msg
        self.amr1_status = f"FORWARD ({self.linear_speed:.2f} m/s)"

    def amr1_backward(self):
        msg = Twist()
        msg.linear.x = -self.linear_speed
        self.amr1_cmd = msg
        self.amr1_status = f"BACKWARD (-{self.linear_speed:.2f} m/s)"

    def amr1_turn_left(self):
        msg = Twist()
        msg.angular.z = self.angular_speed
        self.amr1_cmd = msg
        self.amr1_status = f"ROTATING CCW (+{self.angular_speed:.2f} rad/s)"

    def amr1_turn_right(self):
        msg = Twist()
        msg.angular.z = -self.angular_speed
        self.amr1_cmd = msg
        self.amr1_status = f"ROTATING CW (-{self.angular_speed:.2f} rad/s)"

    def amr1_stop(self):
        self.amr1_cmd = Twist()
        self.amr1_status = "STOPPED"

    # -----------------------------------------------------------------
    # Robot 2 Control Functions
    # -----------------------------------------------------------------
    def amr2_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        self.amr2_cmd = msg
        self.amr2_status = f"FORWARD ({self.linear_speed:.2f} m/s)"

    def amr2_backward(self):
        msg = Twist()
        msg.linear.x = -self.linear_speed
        self.amr2_cmd = msg
        self.amr2_status = f"BACKWARD (-{self.linear_speed:.2f} m/s)"

    def amr2_turn_left(self):
        msg = Twist()
        msg.angular.z = self.angular_speed
        self.amr2_cmd = msg
        self.amr2_status = f"ROTATING CCW (+{self.angular_speed:.2f} rad/s)"

    def amr2_turn_right(self):
        msg = Twist()
        msg.angular.z = -self.angular_speed
        self.amr2_cmd = msg
        self.amr2_status = f"ROTATING CW (-{self.angular_speed:.2f} rad/s)"

    def amr2_stop(self):
        self.amr2_cmd = Twist()
        self.amr2_status = "STOPPED"

    # -----------------------------------------------------------------
    # Dual Robot Simultaneous Actions
    # -----------------------------------------------------------------
    def both_forward(self):
        self.amr1_forward()
        self.amr2_forward()

    def both_opposite(self):
        self.amr1_forward()
        self.amr2_backward()

    def stop_all(self):
        """Stops both robots and immediately sends zero velocity."""
        zero = Twist()
        self.amr1_cmd = zero
        self.amr2_cmd = zero
        self.amr1_status = "STOPPED"
        self.amr2_status = "STOPPED"
        # Publish multiple times to ensure Gazebo catches zero velocity
        for _ in range(5):
            self.amr1_pub.publish(zero)
            self.amr2_pub.publish(zero)
            time.sleep(0.02)


def display_menu(node: DualAmrController):
    """Prints the status banner and interactive command menu."""
    print("\n" + "=" * 55)
    print("      MULTI-AMR DUAL CONTROLLER — PHASE 2")
    print("=" * 55)
    print(f" AMR 1 Status: [{node.amr1_status}]")
    print(f" AMR 2 Status: [{node.amr2_status}]")
    print("-" * 55)
    print(" [AMR 1 - Robot 1]")
    print("   1. Move AMR1 Forward")
    print("   2. Move AMR1 Backward")
    print("   3. Rotate AMR1 Left  (Counter-Clockwise)")
    print("   4. Rotate AMR1 Right (Clockwise)")
    print("   5. Stop AMR1")
    print("-" * 55)
    print(" [AMR 2 - Robot 2]")
    print("   6. Move AMR2 Forward")
    print("   7. Move AMR2 Backward")
    print("   8. Rotate AMR2 Left  (Counter-Clockwise)")
    print("   9. Rotate AMR2 Right (Clockwise)")
    print("  10. Stop AMR2")
    print("-" * 55)
    print(" [Simultaneous Dual Actions]")
    print("  11. Move Both Forward Simultaneously")
    print("  12. Move Both in Opposite Directions")
    print("  13. Stop Both Robots")
    print("-" * 55)
    print("   0. Stop All & Exit")
    print("=" * 55)


def main(args=None):
    rclpy.init(args=args)
    controller = DualAmrController()

    # Run the ROS 2 event loop (timer callbacks) in a daemon background thread.
    # This allows the main thread to remain responsive for user keyboard input!
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    print("\nStarting Dual AMR Controller. Both robots ready.")

    try:
        while rclpy.ok():
            display_menu(controller)
            try:
                choice = input("Enter choice (0-13): ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\nExit signal received.")
                break

            if choice == '1':
                controller.amr1_forward()
                print(">> Command sent: AMR1 Moving Forward")
            elif choice == '2':
                controller.amr1_backward()
                print(">> Command sent: AMR1 Moving Backward")
            elif choice == '3':
                controller.amr1_turn_left()
                print(">> Command sent: AMR1 Rotating Left")
            elif choice == '4':
                controller.amr1_turn_right()
                print(">> Command sent: AMR1 Rotating Right")
            elif choice == '5':
                controller.amr1_stop()
                print(">> Command sent: AMR1 Stopped")
            elif choice == '6':
                controller.amr2_forward()
                print(">> Command sent: AMR2 Moving Forward")
            elif choice == '7':
                controller.amr2_backward()
                print(">> Command sent: AMR2 Moving Backward")
            elif choice == '8':
                controller.amr2_turn_left()
                print(">> Command sent: AMR2 Rotating Left")
            elif choice == '9':
                controller.amr2_turn_right()
                print(">> Command sent: AMR2 Rotating Right")
            elif choice == '10':
                controller.amr2_stop()
                print(">> Command sent: AMR2 Stopped")
            elif choice == '11':
                controller.both_forward()
                print(">> Command sent: Both AMRs Moving Forward")
            elif choice == '12':
                controller.both_opposite()
                print(">> Command sent: Both AMRs Moving in Opposite Directions")
            elif choice == '13':
                controller.stop_all()
                print(">> Command sent: Both AMRs Stopped")
            elif choice == '0':
                print(">> Stopping all robots and exiting...")
                break
            else:
                print(">> Invalid choice! Please enter a number between 0 and 13.")

    except KeyboardInterrupt:
        print("\nShutdown requested by user (Ctrl+C).")

    finally:
        # Guarantee both robots are stopped before exiting
        print("Ensuring zero velocities published to both robots...")
        controller.stop_all()
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Controller safely shut down. Goodbye!")


if __name__ == '__main__':
    main()
