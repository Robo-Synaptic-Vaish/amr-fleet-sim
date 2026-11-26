#!/usr/bin/env python3
from rclpy.executors import ExternalShutdownException
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from amr_fleet_msgs.msg import RobotState


class FleetStateNode(Node):

    def __init__(self):
        super().__init__('fleet_state_node')

        # Robot configuration
        self.declare_parameter('robot_id', 'amr')
        self.declare_parameter('odom_topic', '/model/amr/odometry')
        self.declare_parameter('offset_x', -2.0)
        self.declare_parameter('offset_y', 0.8)
        self.declare_parameter('publish_rate', 5.0)

        # Read parameters
        self.robot_id = self.get_parameter('robot_id').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.offset_x = self.get_parameter('offset_x').value
        self.offset_y = self.get_parameter('offset_y').value
        self.publish_rate = self.get_parameter('publish_rate').value

        self.latest_odom = None

        self.get_logger().info(
            f'Fleet state node started for {self.robot_id}'
        )

        # Subscribe to this robot's odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )

        # Publish robot state to the fleet
        self.state_pub = self.create_publisher(
            RobotState,
            '/fleet/robot_states',
            10
        )

        # Listen to other robots' states
        self.fleet_sub = self.create_subscription(
            RobotState,
            '/fleet/robot_states',
            self.fleet_callback,
            10
        )

        # Publish at configured frequency
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_state
        )

    def odom_callback(self, msg):
        self.latest_odom = msg

    def quaternion_to_yaw(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def publish_state(self):
        if self.latest_odom is None:
            return

        odom = self.latest_odom
        state = RobotState()

        # Robot identity
        state.robot_id = self.robot_id

        # Position in calibrated warehouse coordinates
        state.position_x = odom.pose.pose.position.x + self.offset_x
        state.position_y = odom.pose.pose.position.y + self.offset_y

        # Heading
        state.heading = self.quaternion_to_yaw(
            odom.pose.pose.orientation
        )

        # Velocity
        state.linear_velocity = odom.twist.twist.linear.x
        state.angular_velocity = odom.twist.twist.angular.z

        # Robot status
        if (
            abs(state.linear_velocity) > 0.01
            or abs(state.angular_velocity) > 0.01
        ):
            state.status = "MOVING"
        else:
            state.status = "IDLE"

        state.current_task = "NONE"
        state.timestamp = self.get_clock().now().to_msg()

        self.state_pub.publish(state)

    def fleet_callback(self, msg):
        # Ignore our own broadcast
        if msg.robot_id == self.robot_id:
            return

        self.get_logger().info(
            f'[{self.robot_id}] Received state from {msg.robot_id} | '
            f'Position: ({msg.position_x:.2f}, {msg.position_y:.2f}) | '
            f'Status: {msg.status} | Task: {msg.current_task}'
        )

def main(args=None):
    rclpy.init(args=args)

    node = FleetStateNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()
