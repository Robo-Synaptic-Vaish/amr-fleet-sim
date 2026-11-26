#!/usr/bin/env python3
"""
Dual AMR Keyboard Teleoperation Controller (Phase 2)
---------------------------------------------------
A clean, beginner-friendly ROS 2 node to control two autonomous mobile robots
(AMR 1 and AMR 2) in Gazebo warehouse simulation.

Supports:
  1. AMR 1 independent control (WASD)
  2. AMR 2 independent control (Arrow Keys)
  3. Synchronized fleet control for both AMRs simultaneously (IKJL)
  4. Emergency stopping and safe shutdown (Space, X, Q)

KEY MAPPINGS:
  AMR 1 (Individual):
    [W] Forward
    [S] Backward
    [A] Rotate Left (Counter-Clockwise)
    [D] Rotate Right (Clockwise)

  AMR 2 (Individual):
    [Up Arrow]    Forward
    [Down Arrow]  Backward
    [Left Arrow]  Rotate Left (Counter-Clockwise)
    [Right Arrow] Rotate Right (Clockwise)

  BOTH AMRs (Synchronized Fleet):
    [I] Both Forward
    [K] Both Backward
    [J] Both Rotate Left (Counter-Clockwise)
    [L] Both Rotate Right (Clockwise)

  SAFETY & EXIT:
    [SPACE] or [X] : Emergency Stop BOTH robots immediately
    [Q]            : Stop both robots, restore terminal, and safely exit

Published Topics:
  - /model/amr/cmd_vel  (geometry_msgs/msg/Twist) -> Robot 1
  - /model/amr2/cmd_vel (geometry_msgs/msg/Twist) -> Robot 2
"""

import atexit
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Configurable default parameters
DEFAULT_LINEAR_SPEED = 0.3       # Forward/backward velocity in m/s
DEFAULT_ANGULAR_SPEED = 0.5      # Turning velocity in rad/s
DEFAULT_INACTIVITY_TIMEOUT = 0.5 # Seconds without keypress before auto-stopping
COMMAND_PUBLISH_RATE = 10.0      # Hz (10 velocity updates per second)


class DualAmrTeleopController(Node):
    """
    ROS 2 Node that manages independent velocity publishers for AMR 1 and AMR 2,
    running a 10 Hz timer that publishes continuous velocity and enforces
    automatic inactivity stopping when movement keys are released.
    """
    def __init__(self):
        super().__init__('dual_amr_teleop_controller')

        # Declare configurable parameters
        self.declare_parameter('linear_speed', DEFAULT_LINEAR_SPEED)
        self.declare_parameter('angular_speed', DEFAULT_ANGULAR_SPEED)
        self.declare_parameter('inactivity_timeout', DEFAULT_INACTIVITY_TIMEOUT)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.inactivity_timeout = float(self.get_parameter('inactivity_timeout').value)

        # -------------------------------------------------------------
        # ROS 2 Publishers for Both Robots
        # -------------------------------------------------------------
        self.amr1_pub = self.create_publisher(Twist, '/model/amr/cmd_vel', 10)
        self.amr2_pub = self.create_publisher(Twist, '/model/amr2/cmd_vel', 10)

        # Current velocity command objects
        self.amr1_cmd = Twist()
        self.amr2_cmd = Twist()

        # Activity tracking for key-release / inactivity timeout
        self.amr1_active = False
        self.amr2_active = False
        self.last_amr1_time = 0.0
        self.last_amr2_time = 0.0

        # Human-readable status strings for the terminal display
        self.amr1_status = "STOPPED"
        self.amr2_status = "STOPPED"

        # -------------------------------------------------------------
        # 10 Hz Periodic Timer
        # -------------------------------------------------------------
        # Continuously sends Twist commands to Gazebo DiffDrive and checks
        # whether an active robot should stop due to key inactivity.
        timer_period = 1.0 / COMMAND_PUBLISH_RATE
        self.publish_timer = self.create_timer(timer_period, self.timer_publish_callback)

    def timer_publish_callback(self):
        """Called automatically every 0.1s by the ROS 2 timer."""
        now = time.time()

        # Inactivity check for AMR 1: stop when key is released
        if self.amr1_active and (now - self.last_amr1_time > self.inactivity_timeout):
            self.amr1_cmd = Twist()
            self.amr1_status = "STOPPED"
            self.amr1_active = False

        # Inactivity check for AMR 2: stop when key is released
        if self.amr2_active and (now - self.last_amr2_time > self.inactivity_timeout):
            self.amr2_cmd = Twist()
            self.amr2_status = "STOPPED"
            self.amr2_active = False

        # Publish current command state to each robot's topic
        self.amr1_pub.publish(self.amr1_cmd)
        self.amr2_pub.publish(self.amr2_cmd)

    # -----------------------------------------------------------------
    # AMR 1 Movement Controls (WASD)
    # -----------------------------------------------------------------
    def amr1_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        self.amr1_cmd = msg
        self.amr1_status = f"FORWARD (+{self.linear_speed:.2f} m/s)"
        self.last_amr1_time = time.time()
        self.amr1_active = True

    def amr1_backward(self):
        msg = Twist()
        msg.linear.x = -self.linear_speed
        self.amr1_cmd = msg
        self.amr1_status = f"BACKWARD (-{self.linear_speed:.2f} m/s)"
        self.last_amr1_time = time.time()
        self.amr1_active = True

    def amr1_turn_left(self):
        msg = Twist()
        msg.angular.z = self.angular_speed
        self.amr1_cmd = msg
        self.amr1_status = f"ROTATING CCW (+{self.angular_speed:.2f} rad/s)"
        self.last_amr1_time = time.time()
        self.amr1_active = True

    def amr1_turn_right(self):
        msg = Twist()
        msg.angular.z = -self.angular_speed
        self.amr1_cmd = msg
        self.amr1_status = f"ROTATING CW (-{self.angular_speed:.2f} rad/s)"
        self.last_amr1_time = time.time()
        self.amr1_active = True

    def amr1_stop(self):
        self.amr1_cmd = Twist()
        self.amr1_status = "STOPPED"
        self.amr1_active = False

    # -----------------------------------------------------------------
    # AMR 2 Movement Controls (Arrow Keys)
    # -----------------------------------------------------------------
    def amr2_forward(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        self.amr2_cmd = msg
        self.amr2_status = f"FORWARD (+{self.linear_speed:.2f} m/s)"
        self.last_amr2_time = time.time()
        self.amr2_active = True

    def amr2_backward(self):
        msg = Twist()
        msg.linear.x = -self.linear_speed
        self.amr2_cmd = msg
        self.amr2_status = f"BACKWARD (-{self.linear_speed:.2f} m/s)"
        self.last_amr2_time = time.time()
        self.amr2_active = True

    def amr2_turn_left(self):
        msg = Twist()
        msg.angular.z = self.angular_speed
        self.amr2_cmd = msg
        self.amr2_status = f"ROTATING CCW (+{self.angular_speed:.2f} rad/s)"
        self.last_amr2_time = time.time()
        self.amr2_active = True

    def amr2_turn_right(self):
        msg = Twist()
        msg.angular.z = -self.angular_speed
        self.amr2_cmd = msg
        self.amr2_status = f"ROTATING CW (-{self.angular_speed:.2f} rad/s)"
        self.last_amr2_time = time.time()
        self.amr2_active = True

    def amr2_stop(self):
        self.amr2_cmd = Twist()
        self.amr2_status = "STOPPED"
        self.amr2_active = False

    # -----------------------------------------------------------------
    # Synchronized Fleet Controls (Both AMRs together - IKJL)
    # -----------------------------------------------------------------
    def both_forward(self):
        """Move both AMRs forward at the same speed."""
        self.amr1_forward()
        self.amr2_forward()

    def both_backward(self):
        """Move both AMRs backward at the same speed."""
        self.amr1_backward()
        self.amr2_backward()

    def both_turn_left(self):
        """Rotate both AMRs counter-clockwise at the same speed."""
        self.amr1_turn_left()
        self.amr2_turn_left()

    def both_turn_right(self):
        """Rotate both AMRs clockwise at the same speed."""
        self.amr1_turn_right()
        self.amr2_turn_right()

    # -----------------------------------------------------------------
    # Global Controls (Stop Both)
    # -----------------------------------------------------------------
    def stop_all(self):
        """Immediately halts both robots with a burst of zero velocities."""
        self.amr1_stop()
        self.amr2_stop()
        zero = Twist()
        for _ in range(5):
            self.amr1_pub.publish(zero)
            self.amr2_pub.publish(zero)
            time.sleep(0.01)


def get_key(timeout=0.05):
    """
    Non-blocking / timeout-based key reader using standard Python select.
    Correctly recognizes single keys (WASD, IKJL, Space, X, Q) and multi-byte
    ANSI/VT100 escape sequences for arrow keys.
    """
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if not rlist:
        return None

    try:
        ch = sys.stdin.read(1)
    except Exception:
        return None

    # Handle Escape Sequences (Arrow keys send \x1b[A, \x1b[B, etc.)
    if ch == '\x1b':
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if rlist:
            ch2 = sys.stdin.read(1)
            if ch2 in ['[', 'O']:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist:
                    ch3 = sys.stdin.read(1)
                    if ch3 == 'A':
                        return 'UP'
                    elif ch3 == 'B':
                        return 'DOWN'
                    elif ch3 == 'C':
                        return 'RIGHT'
                    elif ch3 == 'D':
                        return 'LEFT'
        return 'ESC'

    # Normalize AMR 1 keys (WASD)
    if ch in ['w', 'W']:
        return 'W'
    elif ch in ['s', 'S']:
        return 'S'
    elif ch in ['a', 'A']:
        return 'A'
    elif ch in ['d', 'D']:
        return 'D'

    # Normalize Synchronized Fleet keys (IKJL)
    elif ch in ['i', 'I']:
        return 'I'
    elif ch in ['k', 'K']:
        return 'K'
    elif ch in ['j', 'J']:
        return 'J'
    elif ch in ['l', 'L']:
        return 'L'

    # Normalize Safety and Exit keys
    elif ch == ' ':
        return 'SPACE'
    elif ch in ['x', 'X']:
        return 'X'
    elif ch in ['q', 'Q']:
        return 'Q'
    elif ch == '\x03':  # Ctrl+C
        return 'CTRL_C'

    return ch


def print_control_guide():
    """Prints the static teleoperation guide banner."""
    print("=" * 70)
    print("        DUAL-AMR KEYBOARD TELEOP CONTROLLER (PHASE 2)")
    print("=" * 70)
    print("  AMR 1 (Individual):     AMR 2 (Individual):     BOTH AMRs (Fleet):")
    print("    [W] Forward             [↑ Up]    Forward       [I] Both Forward")
    print("    [S] Backward            [↓ Down]  Backward      [K] Both Backward")
    print("    [A] Rotate Left (CCW)   [← Left]  Rotate Left   [J] Both Rotate Left")
    print("    [D] Rotate Right (CW)   [→ Right] Rotate Right  [L] Both Rotate Right")
    print("-" * 70)
    print("  SAFETY & EXIT COMMANDS:")
    print("    [SPACE] or [X] : Emergency Stop BOTH robots immediately")
    print("    [Q]            : Stop both robots, restore terminal, and Quit")
    print("=" * 70)
    print("  Hold key to drive. Release to stop (0.5s inactivity timeout).")
    print("=" * 70)


def main(args=None):
    # Verify terminal stdin is an interactive TTY
    if not sys.stdin.isatty():
        print("Error: dual_amr_controller requires an interactive terminal (TTY).")
        sys.exit(1)

    # Save original terminal settings so we can safely restore them on exit
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def restore_terminal():
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except Exception:
            pass

    # Register exit handler to ensure terminal is NEVER left in raw/cbreak mode
    atexit.register(restore_terminal)

    # Initialize ROS 2
    rclpy.init(args=args)
    controller = DualAmrTeleopController()

    # Run ROS 2 spin in a daemon background thread so the 10 Hz timer executes smoothly
    spin_thread = threading.Thread(target=rclpy.spin, args=(controller,), daemon=True)
    spin_thread.start()

    print_control_guide()

    # Enter cbreak mode: unbuffered character input, standard newline formatting preserved
    tty.setcbreak(fd)

    last_status_display = ""

    try:
        while rclpy.ok():
            key = get_key(timeout=0.05)

            # AMR 1 Individual Control
            if key == 'W':
                controller.amr1_forward()
            elif key == 'S':
                controller.amr1_backward()
            elif key == 'A':
                controller.amr1_turn_left()
            elif key == 'D':
                controller.amr1_turn_right()

            # AMR 2 Individual Control
            elif key == 'UP':
                controller.amr2_forward()
            elif key == 'DOWN':
                controller.amr2_backward()
            elif key == 'LEFT':
                controller.amr2_turn_left()
            elif key == 'RIGHT':
                controller.amr2_turn_right()

            # Both AMRs Synchronized Control (Fleet)
            elif key == 'I':
                controller.both_forward()
            elif key == 'K':
                controller.both_backward()
            elif key == 'J':
                controller.both_turn_left()
            elif key == 'L':
                controller.both_turn_right()

            # Emergency Stop
            elif key in ['SPACE', 'X']:
                controller.stop_all()

            # Safe Exit
            elif key in ['Q', 'CTRL_C']:
                break

            # Update the dynamic status line in-place using \r
            status_line = (
                f"\r  [STATUS] AMR1: {controller.amr1_status:<22} | "
                f"AMR2: {controller.amr2_status:<22}"
            )
            if status_line != last_status_display:
                sys.stdout.write(status_line)
                sys.stdout.flush()
                last_status_display = status_line

    except KeyboardInterrupt:
        pass

    finally:
        # Restore terminal settings first so shutdown messages print properly
        restore_terminal()
        print("\n\nShutting down controller...")
        print("Transmitting stop commands to both AMRs...")
        controller.stop_all()
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("Both AMRs safely stopped. Terminal restored. Goodbye!\n")


if __name__ == '__main__':
    main()
