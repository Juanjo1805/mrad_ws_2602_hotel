#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

class LidarDataNode(Node):

    def __init__(self):
        super().__init__('lidar_data')

        # ---------- parameters ----------
        self.front_angle = np.deg2rad(20.0)
        self.v_sector = np.deg2rad(30.0)

        self.min_speed = 0.05
        self.max_speed = 5.0

        # ---------- state ----------
        self.prev_ranges = None
        self.prev_stamp = None

        self.vx_filt = 0.0
        self.v_ctrl = 0.0
        self.vctrl_fused = 0.0

        self.d_min = float('inf')

        # ---------- ROS ----------
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 1)
        self.cmdout1_sub = self.create_subscription(TwistStamped, '/cmd_vel_mux', self.cmdout_callback, 10)
        #self.cmdout_sub = self.create_subscription(TwistStamped, '/diffdrive_controller/cmd_vel', self.cmdout_callback, 10)


        self.vx_pub = self.create_publisher(Float32, '/lidar/vx', 10)
        self.dmin_pub = self.create_publisher(Float32, '/lidar/d_min', 10)
        self.front_pub = self.create_publisher(LaserScan, '/lidar/front_scan', 10)
        self.vctrl_pub = self.create_publisher(Float32, '/lidar/vctrl', 10)
        #

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------
    def _angle_to_index(self, scan_msg, a):
        return int(round((a - scan_msg.angle_min) / scan_msg.angle_increment))

    def _valid_range(self, scan_msg, r):
        return np.isfinite(r) and (scan_msg.range_min < r < scan_msg.range_max)

    # --------------------------------------------------
    # Sector extraction
    # --------------------------------------------------
    def _sector(self, msg, ranges, a0, a1):

        i0 = max(0, self._angle_to_index(msg, a0))
        i1 = min(len(ranges) - 1, self._angle_to_index(msg, a1))

        if i1 < i0:
            return None, None, None, None, None

        seg = ranges[i0:i1+1]
        idxs = np.arange(i0, i1+1)

        thetas = msg.angle_min + idxs * msg.angle_increment
        mask = np.isfinite(seg) & (msg.range_min < seg) & (seg < msg.range_max)

        return seg, thetas, mask, i0, i1

    # --------------------------------------------------
    # Kinematics (CORE)
    # --------------------------------------------------
    def _compute_kinematics(self, msg: LaserScan):

        st = msg.header.stamp
        t = float(st.sec) + 1e-9 * float(st.nanosec)

        ranges = np.asarray(msg.ranges, dtype=np.float64)

        # ---------- FRONT SECTOR ----------
        seg, thetas, mask, i0, i1 = self._sector(
            msg, ranges, -self.front_angle, +self.front_angle
        )

        if seg is None:
            return 0.0, None, None, None, float('inf')

        valid = seg[mask]
        self.d_min = float(np.min(valid)) if valid.size else float('inf')

        # ---------- FIRST FRAME ----------
        if self.prev_ranges is None or self.prev_stamp is None:
            self.prev_ranges = ranges.copy()
            self.prev_stamp = t
            return 0.0, seg, thetas, mask, self.d_min

        dt = t - self.prev_stamp

        if dt <= 1e-4 or dt > 0.5:
            self.prev_ranges = ranges.copy()
            self.prev_stamp = t
            return 0.0, seg, thetas, mask, self.d_min

        prev = self.prev_ranges

        # ---------- VELOCITY SECTOR ----------
        i0v = max(0, self._angle_to_index(msg, -self.v_sector))
        i1v = min(len(ranges) - 1, self._angle_to_index(msg, +self.v_sector))

        idxs_v = np.arange(i0v, i1v + 1)
        thetas_v = msg.angle_min + idxs_v * msg.angle_increment

        vr_list = []
        cos_list = []

        for k, i in enumerate(idxs_v):

            r = ranges[i]
            rp = prev[i] if i < len(prev) else np.inf

            if not (self._valid_range(msg, r) and self._valid_range(msg, rp)):
                continue

            c = np.cos(thetas_v[k])

            if abs(c) < 0.5:
                continue

            if abs(r - rp) > 1.5:
                continue

            vr = (rp - r) / dt

            if abs(vr) > self.max_speed:
                continue

            vr_list.append(vr)
            cos_list.append(c)

        if len(vr_list) < 5:
            vx = 0.0
        else:
            A = np.array(cos_list).reshape(-1, 1)
            b = np.array(vr_list)

            vx_ls = float(np.linalg.lstsq(A, b, rcond=None)[0][0])
            vx_med = float(np.median(b / A.flatten()))

            vx = 0.7 * vx_ls + 0.3 * vx_med

        if (not np.isfinite(vx)) or abs(vx) < self.min_speed:
            vx = 0.0

        self.prev_ranges = ranges.copy()
        self.prev_stamp = t

        return vx, seg, thetas, mask, self.d_min

    # --------------------------------------------------
    # CONTROL VELOCITY (from cmd)
    # --------------------------------------------------
    def cmdout_callback(self, msg):

        # misma lógica que tenías en AEB
        self.v_ctrl = msg.twist.linear.x * 2.11 - 2.21
        #self.v_ctrl = msg.twist.linear.x

    # --------------------------------------------------
    # MAIN CALLBACK
    # --------------------------------------------------
    def scan_callback(self, msg):

        vx, seg, thetas, mask, dmin = self._compute_kinematics(msg)

        if seg is None:
            return

        # ---------- FILTER VX ----------
        alpha_lidar = 0.2          # antes 0.7 — suaviza mucho más el LiDAR
        self.vx_filt = (1 - alpha_lidar) * self.vx_filt + alpha_lidar * vx
        vx = self.vx_filt

        if abs(vx) < self.min_speed:
            vx = 0.0

        # ---------- VCTRL FUSION ----------
        if self.v_ctrl < vx and self.v_ctrl < 0.6 or vx < 0.1:
             alpha_ctrl = 0.2
        else:
            alpha_ctrl = 0.85          # 85% diff_drive, 15% LiDAR
        vctrl = alpha_ctrl * self.v_ctrl + (1 - alpha_ctrl) * vx

        if vctrl < self.min_speed:
            vctrl = 0.0

        vctrl = max(0.0, vctrl)
        self.vctrl_fused = vctrl

        # ---------- PUBLISH ----------
        vx_msg = Float32()
        vx_msg.data = float(vx)
        self.vx_pub.publish(vx_msg)

        dmin_msg = Float32()
        dmin_msg.data = float(dmin)
        self.dmin_pub.publish(dmin_msg)

        vctrl_msg = Float32()
        vctrl_msg.data = float(vctrl)
        self.vctrl_pub.publish(vctrl_msg)

        # ---------- FRONT SCAN ----------
        front_scan = LaserScan()
        front_scan.header = msg.header

        filtered = np.where(mask, seg, np.inf)

        front_scan.angle_min = msg.angle_min + self._angle_to_index(msg, -self.front_angle) * msg.angle_increment
        front_scan.angle_max = msg.angle_min + self._angle_to_index(msg, +self.front_angle) * msg.angle_increment
        front_scan.angle_increment = msg.angle_increment

        front_scan.time_increment = msg.time_increment
        front_scan.scan_time = msg.scan_time

        front_scan.range_min = msg.range_min
        front_scan.range_max = msg.range_max

        front_scan.ranges = filtered.tolist()

        self.front_pub.publish(front_scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()