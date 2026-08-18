#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseToTF(Node):

    def __init__(self):
        super().__init__('odom_node')

        #  Parámetros configurables
        self.declare_parameter('pose_topic', '/robot1/pose')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')

        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value

        # Suscriptor
        self.subscription = self.create_subscription(
            Pose,
            self.pose_topic,
            self.pose_callback,
            10
        )

        # 🌐 TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Suscrito a: {self.pose_topic}")
        self.get_logger().info(f"Publicando TF: {self.parent_frame} → {self.child_frame}")

    def pose_callback(self, msg: Pose):
        t = TransformStamped()

        # Timestamp actual (porque Pose no tiene header)
        t.header.stamp = self.get_clock().now().to_msg()

        # Frames
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        #Posición
        t.transform.translation.x = msg.position.x/1000.0
        t.transform.translation.y = msg.position.y/1000.0
        t.transform.translation.z = msg.position.z/1000.0

        # Orientación
        t.transform.rotation = msg.orientation

        # Publicar transformada
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()