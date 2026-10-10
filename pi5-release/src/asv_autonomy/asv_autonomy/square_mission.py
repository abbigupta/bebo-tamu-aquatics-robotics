"""Send a five-metre square mission relative to the current ASV pose."""

from __future__ import annotations

from asv_interfaces.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class SquareMission(Node):
    """Submit a square waypoint action after localization becomes available."""

    def __init__(self) -> None:
        super().__init__('square_mission')
        self.declare_parameter('side_length_m', 5.0)
        self.declare_parameter('cruise_speed_mps', 0.7)
        self.declare_parameter('acceptance_radius_m', 0.5)
        self.declare_parameter(
            'odometry_topic', '/mavros/local_position/odom'
        )
        self.position: tuple[float, float] | None = None
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odometry_topic').value),
            self._odom,
            qos_profile_sensor_data,
        )
        self.client = ActionClient(self, FollowWaypoints, '/asv/follow_waypoints')
        self.goal_handle = None

    def _odom(self, message: Odometry) -> None:
        self.position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )

    def send(self) -> bool:
        self.get_logger().info('Waiting for localization...')
        while rclpy.ok() and self.position is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.position is None or not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Waypoint action server unavailable.')
            return False

        x, y = self.position
        side = float(self.get_parameter('side_length_m').value)
        coordinates = ((x + side, y), (x + side, y + side),
                       (x, y + side), (x, y))
        goal = FollowWaypoints.Goal()
        for px, py in coordinates:
            waypoint = PoseStamped()
            waypoint.header.frame_id = 'map'
            waypoint.pose.position.x = px
            waypoint.pose.position.y = py
            waypoint.pose.orientation.w = 1.0
            goal.waypoints.append(waypoint)
        goal.cruise_speed_mps = float(
            self.get_parameter('cruise_speed_mps').value
        )
        goal.acceptance_radius_m = float(
            self.get_parameter('acceptance_radius_m').value
        )
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error('Mission rejected.')
            return False
        self.goal_handle = handle
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(result.message)
        return bool(result.success)

    def cancel(self) -> None:
        if self.goal_handle is not None:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

    def _feedback(self, message) -> None:
        feedback = message.feedback
        self.get_logger().info(
            f'waypoint={feedback.current_waypoint + 1} '
            f'distance={feedback.distance_remaining_m:.2f} m '
            f'heading_error={feedback.heading_error_rad:.2f} rad',
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SquareMission()
    success = False
    try:
        success = node.send()
    except KeyboardInterrupt:
        node.cancel()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)
