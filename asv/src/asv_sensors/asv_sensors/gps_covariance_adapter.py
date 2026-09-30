"""Add configured position covariance to simulated Gazebo GPS fixes."""

from __future__ import annotations

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


def diagonal_covariance(
    horizontal_stddev: float, vertical_stddev: float
) -> list[float]:
    """Build an ENU diagonal position covariance from standard deviations."""
    horizontal_variance = horizontal_stddev ** 2
    vertical_variance = vertical_stddev ** 2
    return [
        horizontal_variance, 0.0, 0.0,
        0.0, horizontal_variance, 0.0,
        0.0, 0.0, vertical_variance,
    ]


class GpsCovarianceAdapter(Node):
    """Republish Gazebo GPS data with covariance needed by localization."""

    def __init__(self) -> None:
        super().__init__('gps_covariance_adapter')
        self.declare_parameter('input_topic', '/asv/gps/fix_raw')
        self.declare_parameter('output_topic', '/asv/gps/fix')
        self.declare_parameter('horizontal_stddev_m', 0.50)
        self.declare_parameter('vertical_stddev_m', 1.00)
        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.covariance = diagonal_covariance(
            float(self.get_parameter('horizontal_stddev_m').value),
            float(self.get_parameter('vertical_stddev_m').value),
        )
        self.publisher = self.create_publisher(NavSatFix, output_topic, 10)
        self.create_subscription(NavSatFix, input_topic, self._fix, 10)

    def _fix(self, fix: NavSatFix) -> None:
        message = NavSatFix()
        message.header = fix.header
        message.status = fix.status
        message.latitude = fix.latitude
        message.longitude = fix.longitude
        message.altitude = fix.altitude
        message.position_covariance = self.covariance
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GpsCovarianceAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
