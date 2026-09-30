"""Expose Gazebo truth as filtered odometry while sensor fusion is optional."""

from __future__ import annotations

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


class GroundTruthLocalizer(Node):
    """Republish Gazebo truth with the ASV's standard odometry frames."""

    def __init__(self) -> None:
        super().__init__('ground_truth_localizer')
        self.publisher = self.create_publisher(
            Odometry, '/asv/odometry/filtered', 10
        )
        self.dynamic_tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.create_subscription(
            Odometry, '/asv/odometry/ground_truth', self._odom, 10
        )
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'odom'
        map_to_odom.transform.rotation.w = 1.0
        self.static_tf.sendTransform(map_to_odom)

    def _odom(self, message: Odometry) -> None:
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_footprint'
        self.publisher.publish(message)
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = message.child_frame_id
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.dynamic_tf.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RuntimeError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
