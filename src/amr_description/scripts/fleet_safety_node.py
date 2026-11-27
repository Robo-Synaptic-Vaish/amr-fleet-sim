#!/usr/bin/env python3
"""
Decentralized Fleet Safety & Collision Avoidance Node
------------------------------------------------------
Implements a decentralized Edge-AI safety gate for an Autonomous Mobile Robot (AMR).

Key Capabilities:
  1. Communication:
     - Broadcasts local robot state to /fleet/robot_states at 5 Hz.
     - Subscribes to /fleet/robot_states and maintains in-memory peer records.
     - Monitors communication freshness with a 1.5s timeout.
  2. Collision Avoidance & Deterministic Priority:
     - Calculates inter-robot Euclidean distance using spawn-calibrated coordinates.
     - Assesses collision risk: LOW (>2.0m), MEDIUM (1.35m-2.0m), HIGH (<=1.35m).
     - Resolves conflicts using priority:
       * Priority 1 (AMR 1): PROCEED (has right-of-way).
       * Priority 2 (AMR 2): YIELD (inhibits forward motion; reverse allowed).
     - Hysteresis recovery (>1.65m) smoothly resumes operation once conflict clears.
  3. Safety Gate Pipeline:
     - Intercepts teleop commands: cmd_vel_in (raw) -> safety check -> cmd_vel_out (Gazebo).
     - Emergency stop via /fleet/emergency_stop: clamps all velocities to zero.
     - Communication loss: restricts forward velocity if peer comms are lost in proximity.
     - Informative terminal telemetry logged at 1.0 Hz or on status transitions.
"""

import json
import math
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from amr_fleet_msgs.msg import RobotState


class PeerRecord:
    """Stores the latest state and reception timestamp of a peer robot."""

    def __init__(self, state: RobotState, receive_time: float):
        self.state = state
        self.receive_time = receive_time

    def is_fresh(self, current_time: float, timeout_sec: float) -> bool:
        return (current_time - self.receive_time) < timeout_sec


class FleetSafetyNode(Node):
    """Decentralized safety gate, state broadcaster, and collision avoidance engine."""

    def __init__(self):
        super().__init__('fleet_safety_node')

        # -------------------------------------------------------------
        # Configurable Parameters
        # -------------------------------------------------------------
        self.declare_parameter('robot_id', 'amr')
        self.declare_parameter('peer_id', 'amr2')
        self.declare_parameter('odom_topic', '/model/amr/odometry')
        self.declare_parameter('offset_x', -2.0)
        self.declare_parameter('offset_y', 0.8)
        self.declare_parameter('cmd_vel_in', '/model/amr/cmd_vel_raw')
        self.declare_parameter('cmd_vel_out', '/model/amr/cmd_vel')
        self.declare_parameter('priority', 1)  # 1 = High priority (Pass), 2 = Lower priority (Yield)
        self.declare_parameter('caution_distance', 2.0)   # Distance to slow down (m)
        self.declare_parameter('stop_distance', 1.35)     # Distance to stop/yield (m)
        self.declare_parameter('recovery_distance', 1.65) # Distance to clear conflict (m)
        self.declare_parameter('comm_timeout', 1.5)       # Peer timeout in seconds

        self.robot_id: str = str(self.get_parameter('robot_id').value)
        self.peer_id: str = str(self.get_parameter('peer_id').value)
        self.odom_topic: str = str(self.get_parameter('odom_topic').value)
        self.offset_x: float = float(self.get_parameter('offset_x').value)
        self.offset_y: float = float(self.get_parameter('offset_y').value)
        self.cmd_in_topic: str = str(self.get_parameter('cmd_vel_in').value)
        self.cmd_out_topic: str = str(self.get_parameter('cmd_vel_out').value)
        self.priority: int = int(self.get_parameter('priority').value)
        self.caution_dist: float = float(self.get_parameter('caution_distance').value)
        self.stop_dist: float = float(self.get_parameter('stop_distance').value)
        self.recovery_dist: float = float(self.get_parameter('recovery_distance').value)
        self.comm_timeout: float = float(self.get_parameter('comm_timeout').value)

        # -------------------------------------------------------------
        # Internal State Variables
        # -------------------------------------------------------------
        self.latest_odom: Optional[Odometry] = None
        self.peers: Dict[str, PeerRecord] = {}
        self.latest_raw_cmd = Twist()
        self.emergency_stop: bool = False

        # Current decision and risk status
        self.current_risk = "LOW"
        self.current_decision = "PROCEED"
        self.current_gate_action = "ALLOWED"
        self.last_reported_decision = ""
        self.last_diag_log_time = 0.0
        self.in_conflict = False

        # -------------------------------------------------------------
        # Publishers and Subscribers
        # -------------------------------------------------------------
        # 1. Local Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )

        # 2. Raw Teleoperation command subscriber (input to safety gate)
        self.cmd_in_sub = self.create_subscription(
            Twist, self.cmd_in_topic, self.cmd_in_callback, 10
        )

        # 3. Gated Velocity publisher (sends to Gazebo DiffDrive)
        self.cmd_out_pub = self.create_publisher(
            Twist, self.cmd_out_topic, 10
        )

        # 4. Fleet-wide state publisher & subscriber
        self.fleet_pub = self.create_publisher(
            RobotState, '/fleet/robot_states', 10
        )
        self.fleet_sub = self.create_subscription(
            RobotState, '/fleet/robot_states', self.fleet_callback, 10
        )

        # 5. Fleet emergency stop subscriber
        self.estop_sub = self.create_subscription(
            Bool, '/fleet/emergency_stop', self.estop_callback, 10
        )

        # 6. Dedicated Safety Status Publisher
        self.safety_status_pub = self.create_publisher(
            String, f'/model/{self.robot_id}/safety_status', 10
        )

        # -------------------------------------------------------------
        # Timers
        # -------------------------------------------------------------
        # State Broadcast Timer: 5 Hz (0.2s)
        self.state_timer = self.create_timer(0.2, self.publish_fleet_state)

        # Safety Gate & Decision Evaluation Timer: 10 Hz (0.1s)
        self.control_timer = self.create_timer(0.1, self.evaluate_and_control)

        self.get_logger().info(
            f'[{self.robot_id}] Fleet Safety Node started | '
            f'Priority: {self.priority} | Cmd In: {self.cmd_in_topic} -> Out: {self.cmd_out_topic}'
        )

    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------
    def odom_callback(self, msg: Odometry) -> None:
        self.latest_odom = msg

    def cmd_in_callback(self, msg: Twist) -> None:
        self.latest_raw_cmd = msg

    def estop_callback(self, msg: Bool) -> None:
        if msg.data != self.emergency_stop:
            self.emergency_stop = msg.data
            status_str = "ACTIVATED" if self.emergency_stop else "CLEARED"
            self.get_logger().warn(f'[{self.robot_id}] Emergency Stop {status_str} via /fleet/emergency_stop')

    def fleet_callback(self, msg: RobotState) -> None:
        # Ignore our own state broadcasts
        if msg.robot_id == self.robot_id:
            return
        now = time.monotonic()
        self.peers[msg.robot_id] = PeerRecord(msg, now)

    # -----------------------------------------------------------------
    # Helper Computations
    # -----------------------------------------------------------------
    @staticmethod
    def quaternion_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_calibrated_position(self) -> tuple[float, float]:
        """Returns estimated warehouse position (spawn-offset calibrated)."""
        if self.latest_odom is None:
            return self.offset_x, self.offset_y
        x = self.latest_odom.pose.pose.position.x + self.offset_x
        y = self.latest_odom.pose.pose.position.y + self.offset_y
        return x, y

    # -----------------------------------------------------------------
    # Fleet State Broadcast (5 Hz)
    # -----------------------------------------------------------------
    def publish_fleet_state(self) -> None:
        if self.latest_odom is None:
            return

        odom = self.latest_odom
        state = RobotState()
        state.robot_id = self.robot_id
        x, y = self.get_calibrated_position()
        state.position_x = float(x)
        state.position_y = float(y)
        state.heading = float(self.quaternion_to_yaw(odom.pose.pose.orientation))
        state.linear_velocity = float(odom.twist.twist.linear.x)
        state.angular_velocity = float(odom.twist.twist.angular.z)

        # Movement status with deadband
        if abs(state.linear_velocity) > 0.01 or abs(state.angular_velocity) > 0.01:
            state.status = "MOVING"
        else:
            state.status = "IDLE"

        # Broadcast operational task and timestamp (decoupled from safety decisions)
        state.current_task = "UNASSIGNED"
        state.timestamp = self.get_clock().now().to_msg()
        self.fleet_pub.publish(state)

    # -----------------------------------------------------------------
    # Decision Engine & Safety Gate (10 Hz)
    # -----------------------------------------------------------------
    def evaluate_and_control(self) -> None:
        now = time.monotonic()
        my_x, my_y = self.get_calibrated_position()

        # Check peer status
        peer_online = False
        peer_record = self.peers.get(self.peer_id, None)
        peer_dist: Optional[float] = None
        peer_status_str = "WAITING"

        if peer_record is not None:
            if peer_record.is_fresh(now, self.comm_timeout):
                peer_online = True
                peer_status_str = "ONLINE"
                dx = peer_record.state.position_x - my_x
                dy = peer_record.state.position_y - my_y
                peer_dist = math.sqrt(dx * dx + dy * dy)
            else:
                peer_status_str = "STALE/LOST"
                dx = peer_record.state.position_x - my_x
                dy = peer_record.state.position_y - my_y
                peer_dist = math.sqrt(dx * dx + dy * dy)

        # -------------------------------------------------------------
        # Decision Logic
        # -------------------------------------------------------------
        if self.emergency_stop:
            self.current_risk = "CRITICAL"
            self.current_decision = "EMERGENCY_STOP"
            self.current_gate_action = "BLOCKED"

        elif peer_status_str == "STALE/LOST":
            # Peer was active but communication dropped
            self.current_risk = "MEDIUM"
            self.current_decision = "COMM_LOSS_SAFE"
            self.current_gate_action = "LIMITED"

        elif peer_online and (peer_dist is not None):
            # Proximity Hysteresis Evaluation
            if self.in_conflict:
                # Need distance to exceed recovery_dist to clear conflict
                if peer_dist > self.recovery_dist:
                    self.in_conflict = False

            if peer_dist <= self.stop_dist or (self.in_conflict and peer_dist <= self.recovery_dist):
                self.in_conflict = True
                self.current_risk = "HIGH"

                # Deterministic priority resolution: Priority 1 proceeds, Priority 2 yields
                if self.priority == 1:
                    self.current_decision = "PROCEED"
                    self.current_gate_action = "ALLOWED"
                else:
                    self.current_decision = "YIELD"
                    self.current_gate_action = "BLOCKED"

            elif peer_dist <= self.caution_dist:
                self.current_risk = "MEDIUM"
                self.current_decision = "SLOW"
                self.current_gate_action = "SCALED"
            else:
                self.current_risk = "LOW"
                self.current_decision = "PROCEED"
                self.current_gate_action = "ALLOWED"

        else:
            # Waiting for peer discovery
            self.current_risk = "LOW"
            self.current_decision = "PROCEED"
            self.current_gate_action = "ALLOWED"

        # -------------------------------------------------------------
        # Safety Gate Command Filtering
        # -------------------------------------------------------------
        cmd_out = Twist()
        raw = self.latest_raw_cmd

        if self.current_decision == "EMERGENCY_STOP":
            cmd_out.linear.x = 0.0
            cmd_out.angular.z = 0.0

        elif self.current_decision == "YIELD":
            # Inhibit forward motion toward conflict; allow reversing or turning
            if raw.linear.x > 0.0:
                cmd_out.linear.x = 0.0
                cmd_out.angular.z = raw.angular.z
            else:
                # Reversing away is permitted for recovery
                cmd_out.linear.x = raw.linear.x
                cmd_out.angular.z = raw.angular.z

        elif self.current_decision == "SLOW":
            # Scale forward speed by 50%
            cmd_out.linear.x = raw.linear.x * 0.5
            cmd_out.angular.z = raw.angular.z * 0.7

        elif self.current_decision == "COMM_LOSS_SAFE":
            # Restrict speed to safe crawl
            max_crawl = 0.10
            cmd_out.linear.x = math.copysign(min(abs(raw.linear.x), max_crawl), raw.linear.x)
            cmd_out.angular.z = raw.angular.z * 0.5

        else:  # PROCEED
            cmd_out.linear.x = raw.linear.x
            cmd_out.angular.z = raw.angular.z

        self.cmd_out_pub.publish(cmd_out)

        # -------------------------------------------------------------
        # Dedicated Safety Status Broadcast
        # -------------------------------------------------------------
        safety_status_payload = json.dumps({
            'robot_id': self.robot_id,
            'decision': self.current_decision,
            'gate_action': self.current_gate_action,
            'risk': self.current_risk,
            'in_conflict': self.in_conflict,
            'priority': self.priority
        })
        self.safety_status_pub.publish(String(data=safety_status_payload))

        # -------------------------------------------------------------
        # Visible Telemetry Logging (1 Hz or state change)
        # -------------------------------------------------------------
        state_changed = (self.current_decision != self.last_reported_decision)
        if state_changed or (now - self.last_diag_log_time >= 1.0):
            self.last_diag_log_time = now
            self.last_reported_decision = self.current_decision

            dist_str = f"{peer_dist:.2f}m" if peer_dist is not None else "N/A"
            self.get_logger().info(
                f'[{self.robot_id}] Dist: {dist_str:<5} | Risk: {self.current_risk:<8} | '
                f'Peer {self.peer_id}: {peer_status_str:<10} | Decision: {self.current_decision:<14} | '
                f'Gate: {self.current_gate_action}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = FleetSafetyNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
