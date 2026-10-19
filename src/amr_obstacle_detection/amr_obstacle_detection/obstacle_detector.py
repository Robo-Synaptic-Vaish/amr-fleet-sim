import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.declare_parameter('front_angle_degrees', 30.0)
        self.declare_parameter('warning_distance', 1.0)

        self.front_angle = math.radians(
            self.get_parameter('front_angle_degrees').value
        )
        self.warning_distance = (
            self.get_parameter('warning_distance').value
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.scan_callback,
            10
        )

        self.get_logger().info('LiDAR obstacle detector started.')

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

        if not front_distances:
            self.get_logger().warning(
                'No valid LiDAR measurements in the front sector.'
            )
            return

        closest_distance = min(front_distances)

        if closest_distance <= self.warning_distance:
            self.get_logger().warning(
                f'OBSTACLE NEAR: {closest_distance:.2f} m ahead'
            )
        else:
            self.get_logger().info(
                f'Front clearance: {closest_distance:.2f} m'
            )


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
