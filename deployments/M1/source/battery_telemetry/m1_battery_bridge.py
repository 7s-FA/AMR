#!/usr/bin/env python3
"""Read-only M1 voltage relay. Requires real firmware telemetry in volts.

This does not read hardware, enable motors, or provide missing voltage values.
No timer republishes old readings: when firmware stops, web freshness expires.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


class M1BatteryBridge(Node):
    def __init__(self):
        super().__init__('m1_battery_bridge')
        source = self.declare_parameter('source_topic', '/stm32/battery_voltage').value
        target = self.declare_parameter('target_topic', '/burger1/battery_state').value
        self.publisher = self.create_publisher(BatteryState, target, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Float32, source, self.on_voltage, qos_profile_sensor_data)
        self.get_logger().info(f'Read-only battery relay: {source} -> {target}; waiting for measured volts')

    def on_voltage(self, sample):
        # Reject missing/error values. A failed read must never look like a fresh
        # empty battery or refresh the age of the last successful measurement.
        if not math.isfinite(sample.data) or sample.data <= 0:
            return
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.voltage = float(sample.data)
        message.present = True
        message.location = 'M1 motor supply'
        # Unknown quantities must not silently default to zero/full/healthy.
        for field in ('temperature', 'current', 'charge', 'capacity',
                      'design_capacity', 'percentage'):
            setattr(message, field, math.nan)
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = M1BatteryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
