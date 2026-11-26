
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SafetyController(Node):
    def __init__(self):
        super().__init__('safety_controller')

        self.declare_parameter('front_angle_degrees', 30.0)
        self.declare_parameter('stop_distance', 1.0)
        self.declare_parameter('scan_timeout', 0.5)

        self.front_angle = math.radians(
            self.get_parameter('front_angle_degrees').value
        )
        self.stop_distance = self.get_parameter('stop_distance').value
        self.scan_timeout = self.get_parameter('scan_timeout').value

        self.closest_distance = None
        self.last_scan_time = None
        self.obstacle_close = True
        self.last_command = Twist()
        self.blocked = False

        self.scan_sub = self.create_subscription(
            LaserScan, '/lidar', self.scan_callback, 10
        )
        self.command_sub = self.create_subscription(
            Twist, '/cmd_vel_in', self.command_callback, 10
        )
        self.command_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.safety_timer = self.create_timer(
            0.1, self.safety_check
        )

        self.get_logger().info(
            'Safety controller started. Forward motion requires '
            'a fresh, clear LiDAR scan.'
        )

    def scan_callback(self, scan):
        front_distances = []

        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment

            if abs(angle) > self.front_angle:
                continue
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            front_distances.append(distance)

        self.last_scan_time = time.monotonic()

        if not front_distances:
            self.closest_distance = None
            self.obstacle_close = True
        else:
            self.closest_distance = min(front_distances)
            self.obstacle_close = (
                self.closest_distance <= self.stop_distance
            )

        if self.obstacle_close:
            self.stop_if_forward()

    def safety_check(self):
        if self.last_command.linear.x <= 0.0:
            return

        scan_is_fresh = (
            self.last_scan_time is not None
            and time.monotonic() - self.last_scan_time
            <= self.scan_timeout
        )

        if not scan_is_fresh or self.obstacle_close:
            self.command_pub.publish(Twist())
            self.blocked = True

            if not scan_is_fresh:
                self.get_logger().warning(
                    'SAFETY STOP: LiDAR scan missing or stale.'
                )
            elif self.closest_distance is not None:
                self.get_logger().warning(
                    f'SAFETY STOP: obstacle at '
                    f'{self.closest_distance:.2f} m.'
                )

    def command_callback(self, command):
        self.last_command = command
        output = Twist()

        output.linear.x = command.linear.x
        output.linear.y = command.linear.y
        output.linear.z = command.linear.z
        output.angular.x = command.angular.x
        output.angular.y = command.angular.y
        output.angular.z = command.angular.z

        scan_is_fresh = (
            self.last_scan_time is not None
            and time.monotonic() - self.last_scan_time
            <= self.scan_timeout
        )

        if command.linear.x > 0.0 and (
            not scan_is_fresh or self.obstacle_close
        ):
            output.linear.x = 0.0
            self.blocked = True
            self.log_block_reason(scan_is_fresh)
        else:
            self.blocked = False

        self.command_pub.publish(output)

    def stop_if_forward(self):
        if self.last_command.linear.x <= 0.0:
            return

        self.command_pub.publish(Twist())
        self.blocked = True
        self.get_logger().warning(
            'SAFETY STOP: obstacle too close or no valid front scan.'
        )

    def log_block_reason(self, scan_is_fresh):
        if not scan_is_fresh:
            self.get_logger().warning(
                'Forward motion blocked: LiDAR scan missing or stale.'
            )
        else:
            self.get_logger().warning(
                f'Forward motion blocked: obstacle at '
                f'{self.closest_distance:.2f} m.'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SafetyController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.command_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
