#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistKeyToStamped(Node):
    def __init__(self):
        super().__init__("twist_key_to_stamped")
        self.pub = self.create_publisher(TwistStamped, "/cmd_vel_key", 10)
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.cb, 10)

    def cb(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"  # no afecta al diffdrive_controller
        out.twist = msg
        self.pub.publish(out)

def main():
    rclpy.init()
    node = TwistKeyToStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
