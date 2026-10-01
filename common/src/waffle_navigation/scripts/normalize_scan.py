#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanNormalizer(Node):
    """Make LDS-03 LaserScan metadata consistent with its sample count."""

    def __init__(self):
        super().__init__('waffle_scan_normalizer')
        self._publisher = self.create_publisher(
            LaserScan, '/scan/normalized', qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, scan):
        count = len(scan.ranges)
        if count == 0 or not math.isfinite(scan.angle_increment):
            return

        normalized = LaserScan()
        normalized.header = scan.header
        normalized.angle_min = scan.angle_min
        normalized.angle_increment = scan.angle_increment
        normalized.angle_max = scan.angle_min + (count - 1) * scan.angle_increment
        normalized.time_increment = scan.time_increment
        normalized.scan_time = scan.scan_time
        normalized.range_min = scan.range_min
        normalized.range_max = scan.range_max
        normalized.ranges = scan.ranges
        normalized.intensities = scan.intensities
        self._publisher.publish(normalized)


def main(args=None):
    rclpy.init(args=args)
    node = ScanNormalizer()
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
