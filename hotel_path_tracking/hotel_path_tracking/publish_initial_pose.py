#!/usr/bin/env python3
"""Publish a reproducible AMCL initial pose several times, then exit."""

from __future__ import annotations

import math
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node


class InitialPosePublisher(Node):
    """Small one-shot helper used by the experiment runner after every reset."""

    def __init__(self) -> None:
        super().__init__('optimization_initial_pose')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('rate_hz', 5.0)
        self.declare_parameter('minimum_subscribers', 1)
        self.declare_parameter('subscriber_wait_timeout_s', 5.0)
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.count = 0
        self.started_wall = time.monotonic()
        self.waiting_logged = False
        self.limit = max(1, int(self.get_parameter('publish_count').value))
        self.timer = self.create_timer(1.0 / max(1.0, float(self.get_parameter('rate_hz').value)), self.publish)

    def publish(self) -> None:
        minimum_subscribers = max(0, int(self.get_parameter('minimum_subscribers').value))
        timeout_s = max(0.0, float(self.get_parameter('subscriber_wait_timeout_s').value))
        if self.publisher.get_subscription_count() < minimum_subscribers and \
                time.monotonic() - self.started_wall < timeout_s:
            if not self.waiting_logged:
                self.get_logger().info(
                    f'Waiting for {minimum_subscribers} /initialpose subscribers before publishing.')
                self.waiting_logged = True
            return
        if self.waiting_logged and self.publisher.get_subscription_count() < minimum_subscribers:
            self.get_logger().warning(
                'Subscriber readiness timeout; publishing initial pose to the available subscribers.')
        yaw = float(self.get_parameter('yaw').value)
        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter('frame_id').value)
        message.pose.pose.position.x = float(self.get_parameter('x').value)
        message.pose.pose.position.y = float(self.get_parameter('y').value)
        message.pose.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(yaw * 0.5)
        message.pose.covariance[0] = 0.05
        message.pose.covariance[7] = 0.05
        message.pose.covariance[35] = 0.0685
        self.publisher.publish(message)
        self.count += 1
        if self.count >= self.limit:
            self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        while rclpy.ok() and node.count < node.limit:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
