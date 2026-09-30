"""Convert a narrow simulated sonar scan to a ROS ultrasound range."""

from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


def nearest_range(
    ranges: list[float], minimum: float, maximum: float
) -> float:
    """Return the nearest valid ray, or positive infinity for open space."""
    valid = (
        value for value in ranges
        if math.isfinite(value) and minimum <= value <= maximum
    )
    return min(valid, default=math.inf)


class ScanToRange(Node):
    """Collapse a narrow LaserScan fan into one ultrasound Range message."""

    def __init__(self) -> None:
        super().__init__('sonar_range_adapter')
        self.declare_parameter('input_topic', '/asv/sonar/scan')
        self.declare_parameter('output_topic', '/asv/sonar/front')
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(Range, output_topic, 10)
        self.create_subscription(LaserScan, input_topic, self._scan, 10)

    def _scan(self, scan: LaserScan) -> None:
        message = Range()
        message.header = scan.header
        message.radiation_type = Range.ULTRASOUND
        message.field_of_view = max(0.0, scan.angle_max - scan.angle_min)
        message.min_range = scan.range_min
        message.max_range = scan.range_max
        message.range = nearest_range(
            list(scan.ranges), scan.range_min, scan.range_max
        )
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ScanToRange()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
