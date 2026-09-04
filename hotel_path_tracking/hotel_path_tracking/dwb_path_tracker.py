#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf2_ros import Buffer, TransformListener


# --------------------------------------------------
# Small helpers
# --------------------------------------------------
def clamp(x: float, lo: float, hi: float) -> float:
    #This clamp a scalar into [lo, hi].
    return max(lo, min(hi, x))


def wrap_to_pi(a: float) -> float:
    #This wrap an angle to [-pi, pi].
    return math.atan2(math.sin(a), math.cos(a))


class SimpleDWBNode(Node):
    #This class implements a very simple DWB-like controller.
    #This version is focused on clarity: sample (v,w), simulate, score, publish TwistStamped.

    def __init__(self):
        super().__init__('dwb_path_tracker')

        # --------------------------------------------------
        # Parameters: topics and frames
        # --------------------------------------------------
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_dwb')
        self.declare_parameter('base_frame', 'base_link')

        # --------------------------------------------------
        # Parameters: control and goal behavior
        # --------------------------------------------------
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('tf_timeout_sec', 0.15)
        self.declare_parameter('goal_tolerance', 0.20)
        self.declare_parameter('yaw_goal_tolerance', 0.20)
        self.declare_parameter('goal_yaw_kp', 1.2)
        self.declare_parameter('eps', 1e-6)

        # --------------------------------------------------
        # Parameters: robot limits
        # --------------------------------------------------
        self.declare_parameter('max_vel_x', 3.0)
        self.declare_parameter('min_vel_x', 0.00)
        self.declare_parameter('max_vel_theta', 0.80)
        self.declare_parameter('acc_lim_x', 8.0)
        self.declare_parameter('acc_lim_theta', 1.5)
        self.declare_parameter('allow_reverse', False)
        self.declare_parameter('robot_radius', 0.20)

        # --------------------------------------------------
        # Parameters: local path and rollout
        # --------------------------------------------------
        self.declare_parameter('horizon_length', 1.0)
        self.declare_parameter('behind_margin', 0.10)
        self.declare_parameter('path_stride', 1)
        self.declare_parameter('sim_time', 1.0)
        self.declare_parameter('sim_dt', 0.05)
        self.declare_parameter('vx_samples', 15)
        self.declare_parameter('wz_samples', 25)

        # --------------------------------------------------
        # Parameters: obstacles and scoring
        # --------------------------------------------------
        self.declare_parameter('scan_stride', 4)
        self.declare_parameter('max_obstacle_points', 180)
        self.declare_parameter('min_obstacle_dist', 0.15)
        self.declare_parameter('obstacle_influence_dist', 0.80)
        self.declare_parameter('w_path', 6.0)
        self.declare_parameter('w_goal', 2.0)
        self.declare_parameter('w_heading', 0.20)
        self.declare_parameter('w_obstacle', 0.8)
        self.declare_parameter('prefer_forward', 1.2)

        # --------------------------------------------------
        # Read parameters
        # --------------------------------------------------
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.tf_timeout_sec = float(self.get_parameter('tf_timeout_sec').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.yaw_goal_tolerance = float(self.get_parameter('yaw_goal_tolerance').value)
        self.goal_yaw_kp = float(self.get_parameter('goal_yaw_kp').value)
        self.eps = float(self.get_parameter('eps').value)

        self.max_vel_x = float(self.get_parameter('max_vel_x').value)
        self.min_vel_x = float(self.get_parameter('min_vel_x').value)
        self.max_vel_theta = float(self.get_parameter('max_vel_theta').value)
        self.acc_lim_x = float(self.get_parameter('acc_lim_x').value)
        self.acc_lim_theta = float(self.get_parameter('acc_lim_theta').value)
        self.allow_reverse = bool(self.get_parameter('allow_reverse').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)

        self.horizon_length = float(self.get_parameter('horizon_length').value)
        self.behind_margin = float(self.get_parameter('behind_margin').value)
        self.path_stride = max(1, int(self.get_parameter('path_stride').value))
        self.sim_time = float(self.get_parameter('sim_time').value)
        self.sim_dt = float(self.get_parameter('sim_dt').value)
        self.vx_samples = max(2, int(self.get_parameter('vx_samples').value))
        self.wz_samples = max(3, int(self.get_parameter('wz_samples').value))

        self.scan_stride = max(1, int(self.get_parameter('scan_stride').value))
        self.max_obstacle_points = int(self.get_parameter('max_obstacle_points').value)
        self.min_obstacle_dist = float(self.get_parameter('min_obstacle_dist').value)
        self.obstacle_influence_dist = float(self.get_parameter('obstacle_influence_dist').value)
        self.w_path = float(self.get_parameter('w_path').value)
        self.w_goal = float(self.get_parameter('w_goal').value)
        self.w_heading = float(self.get_parameter('w_heading').value)
        self.w_obstacle = float(self.get_parameter('w_obstacle').value)
        self.prefer_forward = float(self.get_parameter('prefer_forward').value)

        # --------------------------------------------------
        # State variables
        # --------------------------------------------------
        self.path = []  #This stores the latest global path poses.
        self.path_frame = None  #This stores the frame_id of the current path.
        self.has_path = False  #This indicates whether a valid path exists.

        self.current_v = 0.0  #This stores the current linear speed from odom.
        self.current_w = 0.0  #This stores the current angular speed from odom.
        self.obstacles_base = np.empty((0, 2), dtype=np.float64)  #This stores static obstacles in base frame.

        # --------------------------------------------------
        # ROS I/O
        # --------------------------------------------------
        self.path_sub = self.create_subscription(Path, self.path_topic, self.on_path, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)

        # --------------------------------------------------
        # TF setup
        # --------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --------------------------------------------------
        # Timer
        # --------------------------------------------------
        self.timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self.on_timer)

        self.get_logger().info('Simple DWB node initialized.')

    # --------------------------------------------------
    # Message callbacks
    # --------------------------------------------------
    def on_path(self, msg: Path):
        #This stores the latest path.
        self.path = list(msg.poses)
        self.path_frame = msg.header.frame_id if msg.header.frame_id else None
        self.has_path = (len(self.path) > 0) and (self.path_frame is not None)

    def on_odom(self, msg: Odometry):
        #This stores the current robot velocities.
        self.current_v = float(msg.twist.twist.linear.x)
        self.current_w = float(msg.twist.twist.angular.z)

    def on_scan(self, msg: LaserScan):
        #This converts the scan into a sparse obstacle cloud in base frame.
        ranges = np.asarray(msg.ranges, dtype=np.float64)
        if ranges.size == 0:
            self.obstacles_base = np.empty((0, 2), dtype=np.float64)
            return

        idxs = np.arange(0, len(ranges), self.scan_stride)
        rr = ranges[idxs]
        aa = msg.angle_min + idxs * msg.angle_increment
        mask = np.isfinite(rr) & (msg.range_min < rr) & (rr < msg.range_max)

        if not np.any(mask):
            self.obstacles_base = np.empty((0, 2), dtype=np.float64)
            return

        rr = rr[mask]
        aa = aa[mask]
        pts_scan = np.column_stack((rr * np.cos(aa), rr * np.sin(aa)))
        pts_base = self._transform_points_to_base(pts_scan, msg.header.frame_id)

        if pts_base is None or pts_base.size == 0:
            self.obstacles_base = np.empty((0, 2), dtype=np.float64)
            return

        self.obstacles_base = pts_base[:self.max_obstacle_points, :]

    # --------------------------------------------------
    # TF helpers
    # --------------------------------------------------
    def _quat_to_yaw(self, q) -> float:
        #This extracts yaw from a quaternion.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _lookup_tf(self, target_frame: str, source_frame: str):
        #This gets the latest transform source_frame -> target_frame.
        try:
            return self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=self.tf_timeout_sec))
        except Exception:
            return None

    def _transform_pose_to_base(self, pose_msg: PoseStamped):
        #This transforms a PoseStamped into base_frame using planar TF.
        if not pose_msg.header.frame_id:
            return None

        tf = self._lookup_tf(self.base_frame, pose_msg.header.frame_id)
        if tf is None:
            return None

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        tyaw = self._quat_to_yaw(tf.transform.rotation)
        c = math.cos(tyaw)
        s = math.sin(tyaw)

        px = float(pose_msg.pose.position.x)
        py = float(pose_msg.pose.position.y)
        pyaw = self._quat_to_yaw(pose_msg.pose.orientation)

        bx = tx + c * px - s * py
        by = ty + s * px + c * py
        byaw = wrap_to_pi(tyaw + pyaw)
        return bx, by, byaw

    def _transform_points_to_base(self, pts: np.ndarray, source_frame: str):
        #This transforms an array of 2D points from source_frame to base_frame.
        if pts.size == 0:
            return np.empty((0, 2), dtype=np.float64)

        if source_frame == self.base_frame or source_frame == '':
            return pts.copy()

        tf = self._lookup_tf(self.base_frame, source_frame)
        if tf is None:
            return None

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        tyaw = self._quat_to_yaw(tf.transform.rotation)
        c = math.cos(tyaw)
        s = math.sin(tyaw)
        rot = np.array([[c, -s], [s, c]], dtype=np.float64)
        out = pts @ rot.T
        out[:, 0] += tx
        out[:, 1] += ty
        return out

    # --------------------------------------------------
    # Command publishers
    # --------------------------------------------------
    def _publish_cmd(self, vx: float, wz: float):
        #This publishes a TwistStamped command.
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.base_frame
        out.twist.linear.x = float(vx)
        out.twist.angular.z = float(wz)
        self.cmd_pub.publish(out)

    def _stop(self):
        #This publishes a stop command.
        self._publish_cmd(0.0, 0.0)

    # --------------------------------------------------
    # Path handling
    # --------------------------------------------------
    def _build_local_path(self):
        #This builds a simple local reference in base frame.
        if not self.has_path:
            return None, None, None

        local_pts = []
        final_goal = None

        for i, pose in enumerate(self.path):
            pose_b = self._transform_pose_to_base(pose)
            if pose_b is None:
                return None, None, None

            bx, by, byaw = pose_b
            d = math.hypot(bx, by)

            if i == len(self.path) - 1:
                final_goal = (bx, by, byaw)

            if bx < -self.behind_margin:
                continue
            if d <= self.horizon_length:
                local_pts.append([bx, by])

        if len(local_pts) == 0:
            return None, final_goal, None

        local_pts = np.asarray(local_pts, dtype=np.float64)
        local_pts = local_pts[::self.path_stride]

        if len(local_pts) == 0:
            return None, final_goal, None

        if np.linalg.norm(local_pts[0]) > 1e-3:
            local_pts = np.vstack((np.array([[0.0, 0.0]], dtype=np.float64), local_pts))

        if len(local_pts) >= 2:
            dx = local_pts[-1, 0] - local_pts[-2, 0]
            dy = local_pts[-1, 1] - local_pts[-2, 1]
            local_goal_yaw = math.atan2(dy, dx)
        else:
            local_goal_yaw = 0.0

        local_goal = (float(local_pts[-1, 0]), float(local_pts[-1, 1]), float(local_goal_yaw))
        return local_pts, final_goal, local_goal

    # --------------------------------------------------
    # Goal handling
    # --------------------------------------------------
    def _handle_goal(self, final_goal):
        #This handles stop or final yaw alignment near the goal.
        if final_goal is None:
            return False

        gx, gy, gyaw = final_goal
        d = math.hypot(gx, gy)
        yaw_err = wrap_to_pi(gyaw)

        if d > self.goal_tolerance:
            return False

        if abs(yaw_err) <= self.yaw_goal_tolerance:
            self._stop()
            return True

        wz = clamp(self.goal_yaw_kp * yaw_err, -self.max_vel_theta, self.max_vel_theta)
        self._publish_cmd(0.0, wz)
        return True

    # --------------------------------------------------
    # Dynamic window
    # --------------------------------------------------
    def _build_dynamic_window(self):
        #This computes reachable velocities in one control step.
        dt_ctrl = 1.0 / max(self.control_rate_hz, 1.0)

        v_lo = self.current_v - self.acc_lim_x * dt_ctrl
        v_hi = self.current_v + self.acc_lim_x * dt_ctrl
        w_lo = self.current_w - self.acc_lim_theta * dt_ctrl
        w_hi = self.current_w + self.acc_lim_theta * dt_ctrl

        if self.allow_reverse:
            v_lo = max(-self.max_vel_x, v_lo)
        else:
            v_lo = max(self.min_vel_x, v_lo)

        v_hi = min(self.max_vel_x, v_hi)
        w_lo = max(-self.max_vel_theta, w_lo)
        w_hi = min(self.max_vel_theta, w_hi)

        if v_hi < v_lo:
            v_hi = v_lo
        if w_hi < w_lo:
            w_hi = w_lo

        v_samples = np.linspace(v_lo, v_hi, self.vx_samples)
        w_samples = np.linspace(w_lo, w_hi, self.wz_samples)
        return v_samples, w_samples

    # --------------------------------------------------
    # Candidate simulation
    # --------------------------------------------------
    def _simulate(self, vx: float, wz: float):
        #This simulates a unicycle trajectory in base frame.
        n_steps = max(1, int(math.ceil(self.sim_time / max(self.sim_dt, 1e-3))))
        traj = np.zeros((n_steps + 1, 3), dtype=np.float64)

        x = 0.0
        y = 0.0
        th = 0.0
        traj[0] = np.array([x, y, th], dtype=np.float64)

        for i in range(1, n_steps + 1):
            x += vx * math.cos(th) * self.sim_dt
            y += vx * math.sin(th) * self.sim_dt
            th = wrap_to_pi(th + wz * self.sim_dt)
            traj[i] = np.array([x, y, th], dtype=np.float64)

        return traj

    # --------------------------------------------------
    # Costs
    # --------------------------------------------------
    def _path_cost(self, traj: np.ndarray, local_pts: np.ndarray):
        #This computes how far the simulated trajectory stays from the local path.
        if local_pts is None or len(local_pts) == 0:
            return float('inf')

        idxs = [0, len(traj) // 2, len(traj) - 1]
        total = 0.0
        for i in idxs:
            p = traj[i, :2]
            d = np.linalg.norm(local_pts - p[None, :], axis=1)
            total += float(np.min(d))
        return total / len(idxs)

    def _goal_cost(self, traj: np.ndarray, local_goal):
        #This computes end-point distance to the local goal.
        if local_goal is None:
            return float('inf')
        gx, gy, _ = local_goal
        return math.hypot(gx - traj[-1, 0], gy - traj[-1, 1])

    def _heading_cost(self, traj: np.ndarray, local_goal):
        #This computes heading error at the end of the simulated trajectory.
        if local_goal is None:
            return float('inf')
        _, _, gyaw = local_goal
        return abs(wrap_to_pi(gyaw - traj[-1, 2]))

    def _obstacle_cost(self, traj: np.ndarray):
        #This penalizes candidates that approach static obstacles too much.
        if self.obstacles_base.size == 0:
            return 0.0

        best = float('inf')
        for i in range(len(traj)):
            p = traj[i, :2]
            d = np.linalg.norm(self.obstacles_base - p[None, :], axis=1)
            if d.size > 0:
                best = min(best, float(np.min(d)))

        clearance = best - self.robot_radius

        if clearance <= 0.0:
            return float('inf')
        if clearance < self.min_obstacle_dist:
            return 1e6
        if clearance >= self.obstacle_influence_dist:
            return 0.0

        return 1.0 / max(clearance - self.min_obstacle_dist + self.eps, self.eps)

    def _score(self, traj: np.ndarray, vx: float, local_pts: np.ndarray, local_goal):
        #This combines all critics into one scalar score.
        c_obs = self._obstacle_cost(traj)
        if not np.isfinite(c_obs):
            return float('inf')

        c_path = self._path_cost(traj, local_pts)
        c_goal = self._goal_cost(traj, local_goal)
        c_head = self._heading_cost(traj, local_goal)

        return (
            self.w_path * c_path +
            self.w_goal * c_goal +
            self.w_heading * c_head +
            self.w_obstacle * c_obs -
            self.prefer_forward * max(vx, 0.0)
        )

    # --------------------------------------------------
    # Command selection
    # --------------------------------------------------
    def _select_best_cmd(self, local_pts: np.ndarray, local_goal):
        #This samples candidates, simulates them, scores them, and returns the best command.
        v_samples, w_samples = self._build_dynamic_window()

        best_score = float('inf')
        best_v = 0.0
        best_w = 0.0

        for vx in v_samples:
            if (not self.allow_reverse) and (vx < 0.0):
                continue

            for wz in w_samples:
                traj = self._simulate(float(vx), float(wz))
                score = self._score(traj, float(vx), local_pts, local_goal)
                if score < best_score:
                    best_score = score
                    best_v = float(vx)
                    best_w = float(wz)

        if not np.isfinite(best_score):
            return 0.0, 0.0

        return best_v, best_w

    # --------------------------------------------------
    # Main control loop
    # --------------------------------------------------
    def on_timer(self):
        #This runs the DWB cycle and publishes TwistStamped directly.
        if not self.has_path:
            self._stop()
            return

        local_pts, final_goal, local_goal = self._build_local_path()
        if final_goal is None:
            self._stop()
            return

        if self._handle_goal(final_goal):
            return

        if local_pts is None or local_goal is None or len(local_pts) < 2:
            self._stop()
            return

        vx, wz = self._select_best_cmd(local_pts, local_goal)
        self._publish_cmd(vx, wz)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleDWBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()