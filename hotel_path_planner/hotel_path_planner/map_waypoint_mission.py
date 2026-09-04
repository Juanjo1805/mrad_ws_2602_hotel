#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException

import math
import threading


class MissionManager(Node):

    def __init__(self):

        super().__init__('mission_manager')

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/mission_goal',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.goals = []
        self.current_goal = 0

        self.mission_active = False
        self.goal_active = False

        self.reach_threshold = 0.8

        thread = threading.Thread(target=self.console_loop)
        thread.daemon = True
        thread.start()

        self.get_logger().info("Mission manager ready")
        self.get_logger().info("Use RViz to set goals")
        self.get_logger().info("Type 'start' to begin mission")

    def goal_callback(self, msg):

        if self.mission_active:
            return

        self.goals.append(msg)

        self.get_logger().info(
            f"Goal stored ({len(self.goals)})"
        )

    def console_loop(self):

        while rclpy.ok():

            cmd = input()

            if cmd == "start":

                if not self.goals:
                    print("No goals stored")
                    continue

                try:
                    n = int(input("How many laps?: "))
                except:
                    print("Invalid number")
                    continue

                # duplicar metas
                self.goals = self.goals * n

                self.mission_active = True
                self.current_goal = 0
                self.goal_active = False

                print(f"Mission started ({len(self.goals)} goals)")

            elif cmd == "list":

                print(self.goals)

            elif cmd == "clear":

                self.goals.clear()
                print("Goals cleared")

    def timer_callback(self):

        if not self.mission_active:
            return

        try:

            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(seconds=0)
            )

        except TransformException as ex:

            self.get_logger().warn(str(ex))
            return

        rx = transform.transform.translation.x
        ry = transform.transform.translation.y

        goal = self.goals[self.current_goal]

        gx = goal.pose.position.x
        gy = goal.pose.position.y

        dist = math.sqrt((gx - rx)**2 + (gy - ry)**2)

        if not self.goal_active:

            self.goal_pub.publish(goal)

            self.goal_active = True

            self.get_logger().info(
                f"Sending goal {self.current_goal+1}"
            )

            return

        if dist < self.reach_threshold:

            self.current_goal += 1
            self.goal_active = False

            if self.current_goal < len(self.goals):

                next_goal = self.goals[self.current_goal]
                self.goal_pub.publish(next_goal)

                self.goal_active = True

            else:

                self.get_logger().info("Mission completed")
                self.mission_active = False


def main():

    rclpy.init()

    node = MissionManager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()