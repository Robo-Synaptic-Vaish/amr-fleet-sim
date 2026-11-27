#!/usr/bin/env python3
"""
Dual AMR Keyboard Teleoperation Controller (Phase 2)
---------------------------------------------------
A clean, beginner-friendly ROS 2 node to control two autonomous mobile robots
(AMR 1 and AMR 2) in Gazebo warehouse simulation.

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
from std_msgs.msg import Bool

# Configurable default parameters
DEFAULT_LINEAR_SPEED = 0.3       # Forward/backward velocity in m/s
DEFAULT_ANGULAR_SPEED = 0.5      # Turning velocity in rad/s
DEFAULT_INACTIVITY_TIMEOUT = 0.8 # Seconds without keypress before auto-stopping
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
        self.declare_parameter('amr1_cmd_topic', '/model/amr/cmd_vel_raw')
        self.declare_parameter('amr2_cmd_topic', '/model/amr2/cmd_vel_raw')

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.inactivity_timeout = float(self.get_parameter('inactivity_timeout').value)
        self.amr1_cmd_topic = str(self.get_parameter('amr1_cmd_topic').value)
        self.amr2_cmd_topic = str(self.get_parameter('amr2_cmd_topic').value)

        # -------------------------------------------------------------
        # ROS 2 Publishers for Both Robots
        # -------------------------------------------------------------
        self.amr1_pub = self.create_publisher(Twist, self.amr1_cmd_topic, 10)
        self.amr2_pub = self.create_publisher(Twist, self.amr2_cmd_topic, 10)
        self.estop_pub = self.create_publisher(Bool, '/fleet/emergency_stop', 10)
        self.emergency_stop_active = False

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
        self.last_key_label = "NONE"

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

        # If both are stopped, clear last key label
        if not self.amr1_active and not self.amr2_active:
            self.last_key_label = "NONE"

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
        self.last_key_label = "W"

    def amr1_backward(self):
        msg = Twist()
        msg.linear.x = -self.linear_speed
        self.amr1_cmd = msg
        self.amr1_status = f"BACKWARD (-{self.linear_speed:.2f} m/s)"
        self.last_amr1_time = time.time()
        self.amr1_active = True
        self.last_key_label = "S"

    def amr1_turn_left(self):
        msg = Twist()
        msg.angular.z = self.angular_speed
        self.amr1_cmd = msg
        self.amr1_status = f"ROTATING CCW (+{self.angular_speed:.2f} rad/s)"
        self.last_amr1_time = time.time()
        self.amr1_active = True
        self.last_key_label = "A"

    def amr1_turn_right(self):
        msg = Twist()
        msg.angular.z = -self.angular_speed
        self.amr1_cmd = msg
        self.amr1_status = f"ROTATING CW (-{self.angular_speed:.2f} rad/s)"
        self.last_amr1_time = time.time()
        self.amr1_active = True
        self.last_key_label = "D"

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
        self.last_key_label = "UP ARROW"

    def amr2_backward(self):
        msg = Twist()
        msg.linear.x = -self.linear_speed
        self.amr2_cmd = msg
        self.amr2_status = f"BACKWARD (-{self.linear_speed:.2f} m/s)"
        self.last_amr2_time = time.time()
        self.amr2_active = True
        self.last_key_label = "DOWN ARROW"

    def amr2_turn_left(self):
        msg = Twist()
        msg.angular.z = self.angular_speed
        self.amr2_cmd = msg
        self.amr2_status = f"ROTATING CCW (+{self.angular_speed:.2f} rad/s)"
        self.last_amr2_time = time.time()
        self.amr2_active = True
        self.last_key_label = "LEFT ARROW"

    def amr2_turn_right(self):
        msg = Twist()
        msg.angular.z = -self.angular_speed
        self.amr2_cmd = msg
        self.amr2_status = f"ROTATING CW (-{self.angular_speed:.2f} rad/s)"
        self.last_amr2_time = time.time()
        self.amr2_active = True
        self.last_key_label = "RIGHT ARROW"

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
        self.last_key_label = "I (BOTH FWD)"

    def both_backward(self):
        """Move both AMRs backward at the same speed."""
        self.amr1_backward()
        self.amr2_backward()
        self.last_key_label = "K (BOTH BACK)"

    def both_turn_left(self):
        """Rotate both AMRs counter-clockwise at the same speed."""
        self.amr1_turn_left()
        self.amr2_turn_left()
        self.last_key_label = "J (BOTH CCW)"

    def both_turn_right(self):
        """Rotate both AMRs clockwise at the same speed."""
        self.amr1_turn_right()
        self.amr2_turn_right()
        self.last_key_label = "L (BOTH CW)"

    # -----------------------------------------------------------------
    # Global Controls (Stop Both)
    # -----------------------------------------------------------------
    def stop_all(self):
        """Immediately halts both robots with a burst of zero velocities."""
        self.amr1_stop()
        self.amr2_stop()
        self.last_key_label = "STOP"
        zero = Twist()
        for _ in range(5):
            self.amr1_pub.publish(zero)
            self.amr2_pub.publish(zero)
            time.sleep(0.01)

    def trigger_emergency_stop(self):
        """Broadcasts emergency stop across fleet and zeros commands."""
        self.emergency_stop_active = True
        self.stop_all()
        self.last_key_label = "E-STOP (ACTIVE)"
        msg = Bool(data=True)
        for _ in range(3):
            self.estop_pub.publish(msg)
            time.sleep(0.01)

    def reset_emergency_stop(self):
        """Clears emergency stop across the fleet."""
        self.emergency_stop_active = False
        self.last_key_label = "E-STOP CLEARED"
        msg = Bool(data=False)
        for _ in range(3):
            self.estop_pub.publish(msg)
            time.sleep(0.01)


def get_key(fd, timeout=0.05):
    """
    Direct OS-level key reader using select and os.read on file descriptor.
    Bypasses Python TextIOWrapper buffering. Robust against single taps,
    fast typing, and multi-byte escape sequences for arrow keys.
    """
    rlist, _, _ = select.select([fd], [], [], timeout)
    if not rlist:
        return None

    try:
        data = os.read(fd, 1024)
    except Exception:
        return None

    if not data:
        return None

    # Arrow keys in ANSI/VT100 send \x1b[A, \x1b[B, \x1b[C, \x1b[D
    # or \x1bOA, \x1bOB, \x1bOC, \x1bOD
    if b'\x1b[A' in data or b'\x1bOA' in data:
        return 'UP'
    elif b'\x1b[B' in data or b'\x1bOB' in data:
        return 'DOWN'
    elif b'\x1b[C' in data or b'\x1bOC' in data:
        return 'RIGHT'
    elif b'\x1b[D' in data or b'\x1bOD' in data:
        return 'LEFT'

    # Check for Ctrl+C (\x03)
    if b'\x03' in data:
        return 'CTRL_C'

    # Check for Space (' ')
    if b' ' in data:
        return 'SPACE'

    # Letters (case-insensitive check across received byte chunk)
    data_lower = data.lower()
    if b'w' in data_lower:
        return 'W'
    elif b's' in data_lower:
        return 'S'
    elif b'a' in data_lower:
        return 'A'
    elif b'd' in data_lower:
        return 'D'
    elif b'i' in data_lower:
        return 'I'
    elif b'k' in data_lower:
        return 'K'
    elif b'j' in data_lower:
        return 'J'
    elif b'l' in data_lower:
        return 'L'
    elif b'x' in data_lower:
        return 'X'
    elif b'r' in data_lower:
        return 'R'
    elif b'q' in data_lower:
        return 'Q'

    return None


def print_control_guide():
    """Prints the teleoperation guide banner."""
    print("=" * 72)
    print("         DUAL-AMR KEYBOARD TELEOP CONTROLLER (PHASE 2)")
    print("=" * 72)
    print("  AMR 1 (Individual):     AMR 2 (Individual):     BOTH AMRs (Fleet):")
    print("    [W] Forward             [↑ Up]    Forward       [I] Both Forward")
    print("    [S] Backward            [↓ Down]  Backward      [K] Both Backward")
    print("    [A] Rotate Left (CCW)   [← Left]  Rotate Left   [J] Both Rotate Left")
    print("    [D] Rotate Right (CW)   [→ Right] Rotate Right  [L] Both Rotate Right")
    print("-" * 72)
    print("  SAFETY & EXIT COMMANDS:")
    print("    [SPACE] or [X] : Emergency Stop BOTH robots immediately")
    print("    [Q]            : Stop both robots, restore terminal, and Quit")
    print("=" * 72)
    print("  NOTE: Keep focus on THIS terminal window when pressing keys.")
    print("  Hold key to drive. Release to stop (0.8s inactivity timeout).")
    print("=" * 72)


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

    # Register exit handler to ensure terminal is NEVER left in cbreak mode
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
            key = get_key(fd, timeout=0.05)

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

            # Emergency Stop & Reset
            elif key in ['SPACE', 'X']:
                controller.trigger_emergency_stop()
            elif key == 'R':
                controller.reset_emergency_stop()

            # Safe Exit
            elif key in ['Q', 'CTRL_C']:
                break

            # Update the dynamic status line in-place using \r
            status_line = (
                f"\r  [KEY: {controller.last_key_label:<13}] "
                f"AMR1: {controller.amr1_status:<22} | "
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
