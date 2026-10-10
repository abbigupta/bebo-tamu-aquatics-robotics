"""ROS action server that follows map-frame waypoints."""

from __future__ import annotations

import threading
import time

from asv_interfaces.action import FollowWaypoints
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .guidance import guidance_command, yaw_from_quaternion


class WaypointFollower(Node):
    """Follow one map-frame waypoint action at a time."""

    def __init__(self) -> None:
        super().__init__('waypoint_follower')
        self.declare_parameter('heading_gain', 0.8)
        self.declare_parameter('max_yaw_rate_rad_s', 0.35)
        self.declare_parameter('odometry_timeout_s', 0.75)
        self.declare_parameter(
            'odometry_topic', '/mavros/local_position/odom'
        )
        self.heading_gain = float(self.get_parameter('heading_gain').value)
        self.max_yaw_rate = float(
            self.get_parameter('max_yaw_rate_rad_s').value
        )
        self.odom_timeout_ns = int(
            float(self.get_parameter('odometry_timeout_s').value) * 1e9
        )
        self._lock = threading.Lock()
        self._odom: Odometry | None = None
        self._odom_received_ns: int | None = None
        self._mission_active = False
        group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(
            TwistStamped, '/asv/cmd_vel/autonomy', 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odometry_topic').value),
            self._on_odom,
            qos_profile_sensor_data,
            callback_group=group,
        )
        self.server = ActionServer(
            self,
            FollowWaypoints,
            '/asv/follow_waypoints',
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=group,
        )

    def _on_odom(self, message: Odometry) -> None:
        with self._lock:
            self._odom = message
            self._odom_received_ns = self.get_clock().now().nanoseconds

    def _goal(self, request) -> GoalResponse:
        with self._lock:
            if self._mission_active:
                return GoalResponse.REJECT
        if not request.waypoints or request.cruise_speed_mps <= 0.0:
            return GoalResponse.REJECT
        if request.acceptance_radius_m <= 0.0:
            return GoalResponse.REJECT
        if any(point.header.frame_id not in ('', 'map') for point in request.waypoints):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_command(self, speed: float, yaw_rate: float) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.twist.linear.x = speed
        message.twist.angular.z = yaw_rate
        self.publisher.publish(message)

    def _stop(self) -> None:
        for _ in range(3):
            self._publish_command(0.0, 0.0)

    def _result(self, success: bool, completed: int, message: str):
        result = FollowWaypoints.Result()
        result.success = success
        result.completed_waypoints = completed
        result.message = message
        return result

    def _execute(self, goal_handle):
        with self._lock:
            self._mission_active = True
        completed = 0
        try:
            while completed < len(goal_handle.request.waypoints):
                if goal_handle.is_cancel_requested:
                    self._stop()
                    goal_handle.canceled()
                    return self._result(False, completed, 'Mission canceled.')

                with self._lock:
                    odom = self._odom
                    received_ns = self._odom_received_ns
                now_ns = self.get_clock().now().nanoseconds
                if (
                    odom is None or received_ns is None
                    or now_ns - received_ns > self.odom_timeout_ns
                ):
                    self._stop()
                    goal_handle.abort()
                    return self._result(False, completed, 'Odometry unavailable or stale.')

                target = goal_handle.request.waypoints[completed].pose.position
                position = odom.pose.pose.position
                orientation = odom.pose.pose.orientation
                yaw = yaw_from_quaternion(
                    orientation.x, orientation.y, orientation.z, orientation.w
                )
                speed, yaw_rate, distance, heading_error = guidance_command(
                    target.x - position.x,
                    target.y - position.y,
                    yaw,
                    float(goal_handle.request.cruise_speed_mps),
                    self.heading_gain,
                    self.max_yaw_rate,
                )
                if distance <= goal_handle.request.acceptance_radius_m:
                    completed += 1
                    self._publish_command(0.0, 0.0)
                    continue

                feedback = FollowWaypoints.Feedback()
                feedback.current_waypoint = completed
                feedback.distance_remaining_m = float(distance)
                feedback.heading_error_rad = float(heading_error)
                goal_handle.publish_feedback(feedback)
                self._publish_command(speed, yaw_rate)
                time.sleep(0.1)

            self._stop()
            goal_handle.succeed()
            return self._result(True, completed, 'All waypoints reached.')
        finally:
            with self._lock:
                self._mission_active = False

    def destroy_node(self):
        if rclpy.ok():
            self._stop()
        self.server.destroy()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointFollower()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
