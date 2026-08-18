#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Header
from geometry_msgs.msg import Point


class LineExtractor(Node):

    def __init__(self):
        super().__init__("line_extractor_node")

        # Parameters
        self.declare_parameter("epsilon", 0.02)
        self.declare_parameter("delta", 0.05)
        self.declare_parameter("Snum", 6)
        self.declare_parameter("Pmin", 8)
        self.declare_parameter("Lmin", 0.1)
        self.declare_parameter("angle_merge_threshold", 0.1)

        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray, "/line_markers", 10
        )

        self.pose_pub = self.create_publisher(
            PoseArray, "/line_segments", 10
        )

    # ============================================================
    # =================== MAIN CALLBACK ===========================
    # ============================================================

    def scan_callback(self, msg):

        points, thetas = self.scan_to_points(msg)

        segments = self.extract_lines(points, thetas)

        segments = self.process_overlap(segments, points)

        self.publish_results(segments, msg.header, points)

    # ============================================================
    # ===================== CORE LOGIC ============================
    # ============================================================

    def extract_lines(self, points, thetas):

        epsilon = self.get_parameter("epsilon").value
        Snum = self.get_parameter("Snum").value
        Pmin = self.get_parameter("Pmin").value
        Lmin = self.get_parameter("Lmin").value

        Np = len(points)
        segments = []
        i = 0

        while i < (Np - Pmin):

            seed = self.detect_seed(i, points, thetas)

            if seed is None:
                i += 1
                continue

            Pb, Pf = self.region_growing(seed, points)

            if (Pf - Pb) >= Pmin:
                segment_points = points[Pb:Pf+1]
                a, b, c = self.fit_tls(segment_points)
                segments.append((Pb, Pf, a, b, c))

            i = Pf + 1

        return segments

    # ============================================================
    # ================== ALGORITHM 1 ==============================
    # ============================================================

    def detect_seed(self, i, points, thetas):

        epsilon = self.get_parameter("epsilon").value
        delta = self.get_parameter("delta").value
        Snum = self.get_parameter("Snum").value

        j = i + Snum
        if j >= len(points):
            return None

        window = points[i:j]
        a, b, c = self.fit_tls(window)

        for k in range(i, j):

            x, y = points[k]
            theta = thetas[k]

            # d2: perpendicular distance
            d2 = abs(a*x + b*y + c)
            if d2 > epsilon:
                return None

            # d1: continuity constraint
            r_pred = self.predicted_range(a, b, c, theta)
            if r_pred is None:
                return None

            x_pred = r_pred * np.cos(theta)
            y_pred = r_pred * np.sin(theta)

            d1 = np.linalg.norm([x - x_pred, y - y_pred])
            if d1 > delta:
                return None

        return (i, j)

    # ============================================================
    # ================== ALGORITHM 2 ==============================
    # ============================================================

    def region_growing(self, seed, points):

        epsilon = self.get_parameter("epsilon").value

        Pb, Pf = seed
        Pf = Pf - 1

        # grow forward
        while Pf + 1 < len(points):
            candidate = points[Pb:Pf+2]
            a, b, c = self.fit_tls(candidate)

            x, y = points[Pf+1]
            d = abs(a*x + b*y + c)

            if d < epsilon:
                Pf += 1
            else:
                break

        # grow backward
        while Pb - 1 >= 0:
            candidate = points[Pb-1:Pf+1]
            a, b, c = self.fit_tls(candidate)

            x, y = points[Pb-1]
            d = abs(a*x + b*y + c)

            if d < epsilon:
                Pb -= 1
            else:
                break

        return Pb, Pf

    # ============================================================
    # ================== ALGORITHM 3 ==============================
    # ============================================================

    def process_overlap(self, segments, points):

        merged = []
        angle_thresh = self.get_parameter("angle_merge_threshold").value

        for seg in segments:
            Pb, Pf, a, b, c = seg
            merged_flag = False

            for idx, m in enumerate(merged):
                Pb2, Pf2, a2, b2, c2 = m

                angle = np.arccos(abs(a*a2 + b*b2))

                if angle < angle_thresh:
                    newPb = min(Pb, Pb2)
                    newPf = max(Pf, Pf2)
                    pts = points[newPb:newPf+1]
                    a_new, b_new, c_new = self.fit_tls(pts)
                    merged[idx] = (newPb, newPf, a_new, b_new, c_new)
                    merged_flag = True
                    break

            if not merged_flag:
                merged.append(seg)

        return merged

    # ============================================================
    # ======================= HELPERS =============================
    # ============================================================

    def scan_to_points(self, msg):

        points = []
        thetas = []

        for i, r in enumerate(msg.ranges):

            if np.isinf(r) or np.isnan(r):
                continue

            theta = msg.angle_min + i * msg.angle_increment
            x = r * np.cos(theta)
            y = r * np.sin(theta)

            points.append([x, y])
            thetas.append(theta)

        return np.array(points), thetas

    def fit_tls(self, pts):

        centroid = np.mean(pts, axis=0)
        centered = pts - centroid
        C = np.dot(centered.T, centered)

        eigvals, eigvecs = np.linalg.eig(C)
        normal = eigvecs[:, np.argmin(eigvals)]

        a, b = normal
        c = -a*centroid[0] - b*centroid[1]

        return a, b, c

    def predicted_range(self, a, b, c, theta):

        denom = a*np.cos(theta) + b*np.sin(theta)
        if abs(denom) < 1e-6:
            return None

        return -c / denom

    # ============================================================
    # ===================== PUBLISH ===============================
    # ============================================================



    def publish_results(self, segments, header, all_points):

        marker_array = MarkerArray()
        pose_array = PoseArray()
        pose_array.header = header

        for idx, seg in enumerate(segments):

            Pb, Pf, a, b, c = seg

            segment_pts = all_points[Pb:Pf+1]

            # ===============================
            # 1️Calcular endpoints crudos
            # ===============================
            p_start_raw = segment_pts[0]
            p_end_raw = segment_pts[-1]

            # ===============================
            # 2️ Proyección ortogonal
            # ===============================
            def project(x0, y0):
                d = (a*x0 + b*y0 + c)
                x_proj = x0 - a*d
                y_proj = y0 - b*d
                return x_proj, y_proj

            x1, y1 = project(p_start_raw[0], p_start_raw[1])
            x2, y2 = project(p_end_raw[0], p_end_raw[1])

            # ===============================
            # 3️ Calcular representación polar
            # ===============================
            rho = -c
            theta = np.arctan2(b, a)
            length = np.linalg.norm([x2 - x1, y2 - y1])

            pose = Pose()
            pose.position.x = rho
            pose.position.y = theta
            pose.position.z = length
            pose_array.poses.append(pose)

            # ===============================
            # 4️Crear Marker
            # ===============================
            marker = Marker()
            marker.header = header
            marker.ns = "line_segments"
            marker.id = idx
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD

            marker.scale.x = 0.03  # grosor línea

            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.points = []

            p1 = Point()
            p1.x = float(x1)
            p1.y = float(y1)
            p1.z = 0.0

            p2 = Point()
            p2.x = float(x2)
            p2.y = float(y2)
            p2.z = 0.0

            marker.points.append(p1)
            marker.points.append(p2)

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.pose_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = LineExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()