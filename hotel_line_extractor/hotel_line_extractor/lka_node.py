#!/usr/bin/env python3

import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, Joy
from std_msgs.msg import Float32


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


class LineKeepingAssist(Node):
    def __init__(self):
        super().__init__("line_keeping_assist")

        self.declare_parameter("image_topic", "/left_camera/image")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel_lka")
        self.declare_parameter("debug_image_topic", "/lka/debug_image")
        self.declare_parameter("use_joy_enable", True)
        self.declare_parameter("joy_button_index", 5)
        self.declare_parameter("publish_when_lost", False)

        self.declare_parameter("target_speed", 0.8)
        self.declare_parameter("min_speed", 0.25)
        self.declare_parameter("max_steering", 2.0)
        self.declare_parameter("kp_lateral", 1.5)
        self.declare_parameter("kp_heading", 0.8)
        self.declare_parameter("steering_slowdown", 0.9)
        self.declare_parameter("command_timeout", 0.35)

        self.declare_parameter("roi_top_ratio", 0.55)
        self.declare_parameter("lane_width_px", 260.0)
        self.declare_parameter("black_v_max", 140)
        self.declare_parameter("dark_percentile", 35.0)
        self.declare_parameter("dark_margin", 25)
        self.declare_parameter("min_line_length", 25)
        self.declare_parameter("max_line_gap", 35)
        self.declare_parameter("hough_threshold", 35)
        self.declare_parameter("min_abs_slope", 0.35)
        self.declare_parameter("max_abs_slope", 4.5)
        self.declare_parameter("min_black_area", 80.0)
        self.declare_parameter("min_mask_pixels", 80)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.cmd_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.use_joy_enable = bool(self.get_parameter("use_joy_enable").value)
        self.joy_button_index = int(self.get_parameter("joy_button_index").value)
        self.publish_when_lost = bool(self.get_parameter("publish_when_lost").value)

        self.target_speed = float(self.get_parameter("target_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_steering = float(self.get_parameter("max_steering").value)
        self.kp_lateral = float(self.get_parameter("kp_lateral").value)
        self.kp_heading = float(self.get_parameter("kp_heading").value)
        self.steering_slowdown = float(self.get_parameter("steering_slowdown").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)

        self.roi_top_ratio = float(self.get_parameter("roi_top_ratio").value)
        self.lane_width_px = float(self.get_parameter("lane_width_px").value)
        self.black_v_max = int(self.get_parameter("black_v_max").value)
        self.dark_percentile = float(self.get_parameter("dark_percentile").value)
        self.dark_margin = int(self.get_parameter("dark_margin").value)
        self.min_line_length = int(self.get_parameter("min_line_length").value)
        self.max_line_gap = int(self.get_parameter("max_line_gap").value)
        self.hough_threshold = int(self.get_parameter("hough_threshold").value)
        self.min_abs_slope = float(self.get_parameter("min_abs_slope").value)
        self.max_abs_slope = float(self.get_parameter("max_abs_slope").value)
        self.min_black_area = float(self.get_parameter("min_black_area").value)
        self.min_mask_pixels = int(self.get_parameter("min_mask_pixels").value)

        self.bridge = CvBridge()
        self.rb_pressed = False
        self.prev_rb_pressed = self.rb_pressed
        self.last_valid_time = 0.0
        self.last_image_time = 0.0
        self.last_speed = 0.0
        self.last_steering = 0.0

        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        if self.use_joy_enable:
            self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self.error_pub = self.create_publisher(Float32, "/lka/lateral_error", 10)
        self.heading_pub = self.create_publisher(Float32, "/lka/heading_error", 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)

        self.create_timer(0.05, self.control_timer_callback)

        self.get_logger().info(
            f"Camera LKA started | image={self.image_topic} cmd={self.cmd_topic} "
            f"black_v_max={self.black_v_max} joy_enable={self.use_joy_enable}"
        )

    def joy_callback(self, msg):
        if 0 <= self.joy_button_index < len(msg.buttons):
            self.rb_pressed = msg.buttons[self.joy_button_index] == 1
        else:
            self.rb_pressed = False

        if self.rb_pressed != self.prev_rb_pressed:
            state = "enabled" if self.rb_pressed else "disabled"
            self.get_logger().info(f"LKA deadman {state} using joy button {self.joy_button_index}")
            self.prev_rb_pressed = self.rb_pressed

    def image_callback(self, msg):
        self.last_image_time = time.monotonic()

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"Could not convert image: {exc}")
            return

        lateral_error, heading_error, debug = self.detect_lane(frame)
        if lateral_error is None or heading_error is None:
            self.publish_stop()
            self.last_speed = 0.0
            self.last_steering = 0.0
            self.get_logger().warn(
                "No black lane detected; publishing no drive command",
                throttle_duration_sec=1.0,
            )
            self.publish_debug(0.0, 0.0, debug, msg.header)
            return

        self.last_valid_time = time.monotonic()

        steering = self.kp_lateral * lateral_error + self.kp_heading * heading_error
        steering = clamp(steering, -self.max_steering, self.max_steering)

        speed = self.target_speed / (1.0 + self.steering_slowdown * abs(steering))
        speed = clamp(speed, self.min_speed, self.target_speed)

        if self.use_joy_enable and not self.rb_pressed:
            speed = 0.0
            steering = 0.0
            self.get_logger().info(
                "Dead man activado",
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().info(
                f"LKA cmd speed={speed:.2f} steering={steering:.2f} "
                f"lat={lateral_error:.2f} heading={heading_error:.2f}",
                throttle_duration_sec=0.5,
            )

        self.publish_cmd(speed, steering)
        self.last_speed = speed
        self.last_steering = steering
        self.publish_debug(lateral_error, heading_error, debug, msg.header)

    def detect_lane(self, frame):
        height, width = frame.shape[:2]
        roi_y = int(height * self.roi_top_ratio)
        roi = frame[roi_y:height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        mask = self.build_black_mask(hsv, gray)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        edges = cv2.Canny(mask, 60, 160)
        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=np.pi / 180.0,
            threshold=self.hough_threshold,
            minLineLength=self.min_line_length,
            maxLineGap=self.max_line_gap,
        )

        left_segments = []
        right_segments = []
        debug = frame.copy()
        cv2.rectangle(debug, (0, roi_y), (width - 1, height - 1), (255, 120, 0), 2)

        blob_result = self.detect_lane_by_blobs(mask, debug.copy(), roi_y)

        if lines is not None:
            for raw_line in lines:
                x1, y1, x2, y2 = raw_line[0]
                if x2 == x1:
                    continue

                slope = (y2 - y1) / float(x2 - x1)
                abs_slope = abs(slope)
                if abs_slope < self.min_abs_slope or abs_slope > self.max_abs_slope:
                    continue

                intercept = y1 - slope * x1
                x_bottom = (roi.shape[0] - 1 - intercept) / slope
                segment = (float(slope), float(intercept), float(x_bottom))

                p1 = (int(x1), int(y1 + roi_y))
                p2 = (int(x2), int(y2 + roi_y))
                if slope < 0.0:
                    left_segments.append(segment)
                    cv2.line(debug, p1, p2, (255, 0, 0), 2)
                else:
                    right_segments.append(segment)
                    cv2.line(debug, p1, p2, (0, 0, 255), 2)

        if not left_segments and not right_segments:
            return blob_result

        left_line = self.average_segments(left_segments)
        right_line = self.average_segments(right_segments)
        bottom_y = roi.shape[0] - 1
        lookahead_y = int(roi.shape[0] * 0.45)

        if left_line and right_line:
            left_bottom = self.x_at_y(left_line, bottom_y)
            right_bottom = self.x_at_y(right_line, bottom_y)
            left_ahead = self.x_at_y(left_line, lookahead_y)
            right_ahead = self.x_at_y(right_line, lookahead_y)
            lane_bottom = 0.5 * (left_bottom + right_bottom)
            lane_ahead = 0.5 * (left_ahead + right_ahead)
        elif left_line:
            left_bottom = self.x_at_y(left_line, bottom_y)
            left_ahead = self.x_at_y(left_line, lookahead_y)
            lane_bottom = left_bottom + self.lane_width_px
            lane_ahead = left_ahead + self.lane_width_px
        else:
            right_bottom = self.x_at_y(right_line, bottom_y)
            right_ahead = self.x_at_y(right_line, lookahead_y)
            lane_bottom = right_bottom - self.lane_width_px
            lane_ahead = right_ahead - self.lane_width_px

        image_center = width * 0.5
        lateral_error = (image_center - lane_bottom) / image_center
        heading_error = math.atan2(lane_bottom - lane_ahead, bottom_y - lookahead_y)

        lane_bottom_pt = (int(lane_bottom), int(bottom_y + roi_y))
        lane_ahead_pt = (int(lane_ahead), int(lookahead_y + roi_y))
        cv2.line(debug, lane_bottom_pt, lane_ahead_pt, (0, 255, 0), 4)
        cv2.circle(debug, (int(image_center), int(bottom_y + roi_y)), 5, (255, 255, 255), -1)
        cv2.circle(debug, lane_bottom_pt, 5, (0, 255, 0), -1)

        self.publish_mask_debug(debug, roi_y, mask)
        return float(lateral_error), float(heading_error), debug

    def detect_lane_by_blobs(self, mask, debug, roi_y):
        height, width = debug.shape[:2]
        roi_height = height - roi_y
        ys, xs = np.nonzero(mask)
        if xs.size < self.min_mask_pixels:
            return self.no_lane_result(debug, roi_y, mask)

        image_center = width * 0.5

        bottom_cut = int(roi_height * 0.65)
        ahead_low = int(roi_height * 0.25)
        ahead_high = int(roi_height * 0.55)

        bottom_mask = ys >= bottom_cut
        ahead_mask = (ys >= ahead_low) & (ys <= ahead_high)
        bottom_xs = xs[bottom_mask]
        ahead_xs = xs[ahead_mask]

        if bottom_xs.size >= self.min_mask_pixels:
            lane_bottom = self.lane_center_from_pixels(bottom_xs, image_center)
        else:
            lane_bottom = self.lane_center_from_pixels(xs, image_center)

        if ahead_xs.size >= self.min_mask_pixels:
            lane_ahead = self.lane_center_from_pixels(ahead_xs, image_center)
        else:
            lane_ahead = lane_bottom

        bottom_y = roi_height - 1
        lookahead_y = int(roi_height * 0.45)
        lateral_error = (image_center - lane_bottom) / image_center
        heading_error = math.atan2(lane_bottom - lane_ahead, bottom_y - lookahead_y)

        lane_bottom_pt = (int(lane_bottom), int(bottom_y + roi_y))
        lane_ahead_pt = (int(lane_ahead), int(lookahead_y + roi_y))
        self.publish_mask_debug(debug, roi_y, mask)
        cv2.line(debug, lane_bottom_pt, lane_ahead_pt, (0, 255, 0), 4)
        cv2.circle(debug, (int(image_center), int(bottom_y + roi_y)), 5, (255, 255, 255), -1)
        cv2.circle(debug, lane_bottom_pt, 5, (0, 255, 0), -1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blobs = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_black_area:
                continue

            moments = cv2.moments(contour)
            if moments["m00"] <= 0.0:
                continue

            cx = moments["m10"] / moments["m00"]
            cy = moments["m01"] / moments["m00"]
            x, y, w, h = cv2.boundingRect(contour)
            blobs.append((area, cx, cy, x, y, w, h))

        for blob in sorted(blobs, key=lambda item: item[0], reverse=True)[:4]:
            _, cx, cy, x, y, w, h = blob
            cv2.rectangle(
                debug,
                (int(x), int(y + roi_y)),
                (int(x + w), int(y + h + roi_y)),
                (0, 255, 255),
                2,
            )
            cv2.circle(debug, (int(cx), int(cy + roi_y)), 5, (0, 255, 255), -1)

        return float(lateral_error), float(heading_error), debug

    def lane_center_from_pixels(self, xs, image_center):
        left = xs[xs < image_center]
        right = xs[xs >= image_center]

        if left.size and right.size:
            return 0.5 * (float(np.median(left)) + float(np.median(right)))
        if left.size:
            return float(np.median(left)) + self.lane_width_px
        return float(np.median(right)) - self.lane_width_px

    def build_black_mask(self, hsv, gray):
        adaptive_v = int(np.percentile(gray, clamp(self.dark_percentile, 1.0, 80.0)))
        v_max = int(clamp(adaptive_v + self.dark_margin, 20, self.black_v_max))
        color_mask = cv2.inRange(
            hsv,
            np.array([0, 0, 0]),
            np.array([180, 255, v_max]),
        )
        adaptive = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            31,
            4,
        )
        return cv2.bitwise_or(color_mask, adaptive)

    def average_segments(self, segments):
        if not segments:
            return None
        weights = np.array([max(1.0, abs(seg[2])) for seg in segments], dtype=np.float64)
        slopes = np.array([seg[0] for seg in segments], dtype=np.float64)
        intercepts = np.array([seg[1] for seg in segments], dtype=np.float64)
        return float(np.average(slopes, weights=weights)), float(
            np.average(intercepts, weights=weights)
        )

    def x_at_y(self, line, y):
        slope, intercept = line
        if abs(slope) < 1e-6:
            return 0.0
        return (float(y) - intercept) / slope

    def publish_mask_debug(self, debug, roi_y, mask):
        colored_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        debug[roi_y:, :] = cv2.addWeighted(debug[roi_y:, :], 0.65, colored_mask, 0.35, 0.0)

    def no_lane_result(self, debug, roi_y, mask):
        self.publish_mask_debug(debug, roi_y, mask)
        return None, None, debug

    def control_timer_callback(self):
        if self.last_image_time <= 0.0:
            self.get_logger().warn(
                f"No images received on {self.image_topic}",
                throttle_duration_sec=2.0,
            )
            return

        if self.use_joy_enable and not self.rb_pressed:
            self.publish_stop()
            self.last_speed = 0.0
            self.last_steering = 0.0
            return

        if self.last_valid_time > 0.0:
            age = time.monotonic() - self.last_valid_time
            if age <= self.command_timeout:
                self.publish_cmd(self.last_speed, self.last_steering)
                return

        if self.publish_when_lost:
            self.publish_stop()
            self.last_speed = 0.0
            self.last_steering = 0.0
            return

        if self.last_valid_time <= 0.0:
            return
        if time.monotonic() - self.last_valid_time > self.command_timeout:
            self.publish_stop()
            self.last_speed = 0.0
            self.last_steering = 0.0
            self.last_valid_time = 0.0

    def publish_cmd(self, speed, steering):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        cmd.twist.linear.x = float(speed)
        cmd.twist.angular.z = float(steering)
        self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.publish_cmd(0.0, 0.0)

    def publish_debug(self, lateral_error, heading_error, debug, header):
        lateral_msg = Float32()
        lateral_msg.data = float(lateral_error)
        self.error_pub.publish(lateral_msg)

        heading_msg = Float32()
        heading_msg.data = float(heading_error)
        self.heading_pub.publish(heading_msg)

        image_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        image_msg.header = header
        self.debug_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LineKeepingAssist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
