
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
        self.scan_unusable = True
        self.last_command = Twist()
        self.blocked = False
        self.last_stop_reason_key = None

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
        clear_rays_count = 0
        front_rays_count = 0

        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment

            if abs(angle) > self.front_angle:
                continue

            front_rays_count += 1

            # Positive infinity or readings beyond range_max mean no obstacle within range (clear ray)
            if (math.isinf(distance) and distance > 0.0) or (
                math.isfinite(distance) and distance > scan.range_max
            ):
                clear_rays_count += 1
                continue

            # Valid finite obstacle detection within sensor range
            if math.isfinite(distance) and (scan.range_min <= distance <= scan.range_max):
                front_distances.append(distance)
                continue

            # Other values (NaN, negative infinity, distance < range_min) are treated as invalid/unusable

        self.last_scan_time = time.monotonic()

        if front_distances:
            # Case 1: Finite obstacle returns present in front sector
            self.closest_distance = min(front_distances)
            self.obstacle_close = (self.closest_distance <= self.stop_distance)
            self.scan_unusable = False
        elif clear_rays_count > 0:
            # Case 2: No finite obstacles, but valid +inf/clear beams confirm open path
            self.closest_distance = None
            self.obstacle_close = False
            self.scan_unusable = False
        else:
            # Case 3: Scan is unusable (no front rays or all front rays are NaN / -inf)
            self.closest_distance = None
            self.obstacle_close = True
            self.scan_unusable = True

        if self.obstacle_close:
            self.stop_if_forward()
        elif self.last_stop_reason_key is not None and self.last_command.linear.x > 0.0:
            self.get_logger().info('Forward path clear. Safety stop cleared.')
            self.last_stop_reason_key = None

    def log_block_reason(self, scan_is_fresh):
        if not scan_is_fresh:
            reason_key = 'stale'
            message = 'SAFETY STOP: LiDAR scan missing or stale.'
        elif self.scan_unusable:
            reason_key = 'unusable'
            message = 'SAFETY STOP: LiDAR scan unusable or no valid front measurements.'
        elif self.obstacle_close:
            reason_key = 'obstacle'
            if self.closest_distance is not None:
                message = f'SAFETY STOP: obstacle at {self.closest_distance:.2f} m.'
            else:
                message = 'SAFETY STOP: obstacle within stop distance.'
        else:
            reason_key = None
            message = None

        if reason_key is not None and reason_key != self.last_stop_reason_key:
            self.get_logger().warning(message)
            self.last_stop_reason_key = reason_key
        elif reason_key is None and self.last_stop_reason_key is not None:
            self.get_logger().info('Forward path clear. Safety stop cleared.')
            self.last_stop_reason_key = None

    def safety_check(self):
        if self.last_command.linear.x <= 0.0:
            return

        scan_is_fresh = (
            self.last_scan_time is not None
            and time.monotonic() - self.last_scan_time <= self.scan_timeout
        )

        if not scan_is_fresh or self.obstacle_close:
            self.command_pub.publish(Twist())
            self.blocked = True
            self.log_block_reason(scan_is_fresh)
        else:
            if self.blocked:
                self.blocked = False
                self.log_block_reason(scan_is_fresh)

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
            and time.monotonic() - self.last_scan_time <= self.scan_timeout
        )

        if command.linear.x > 0.0 and (
            not scan_is_fresh or self.obstacle_close
        ):
            output.linear.x = 0.0
            if not self.blocked:
                self.last_stop_reason_key = None
            self.blocked = True
            self.log_block_reason(scan_is_fresh)
        else:
            if self.blocked:
                self.blocked = False
                self.log_block_reason(scan_is_fresh)

        self.command_pub.publish(output)

    def stop_if_forward(self):
        if self.last_command.linear.x <= 0.0:
            return

        self.command_pub.publish(Twist())
        self.blocked = True
        self.log_block_reason(scan_is_fresh=True)


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

