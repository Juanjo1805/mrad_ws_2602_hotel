"""CSV instrumentation shared by the two path trackers."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Optional

from .tracking_core import Pose2D, TrackingCommand


TRACE_COLUMNS = [
    "method", "scenario", "trial", "run_id", "execution_mode", "environment", "timestamp_s", "x_robot", "y_robot", "yaw_robot",
    "x_reference", "y_reference", "yaw_reference", "reference_index", "cross_track_error",
    "heading_error", "linear_velocity_command", "angular_velocity_command", "distance_to_goal",
    "saturated_v", "saturated_omega",
]
SUMMARY_COLUMNS = [
    "method", "scenario", "trial", "run_id", "execution_mode", "environment", "success", "reason", "samples", "cte_rmse_m",
    "cte_mae_m", "cte_max_m", "heading_rmse_rad", "completion_time_s", "final_goal_error_m",
    "progress_percent", "omega_saturations", "mean_abs_delta_omega_rad_s", "mean_control_compute_us", "collision_metric",
]


class TrackingCsvLogger:
    def __init__(self, method: str, scenario: str, trial: int, trace_csv: str, summary_csv: str,
                 execution_mode: str = "ros_live", environment: str = "gazebo") -> None:
        self.method, self.scenario, self.trial = method, scenario, trial
        self.execution_mode = execution_mode
        self.environment = environment
        self.trace_path, self.summary_path = Path(trace_csv), Path(summary_csv)
        self.run_id = 0
        self.started_at: Optional[float] = None
        self.cte_values: list[float] = []
        self.heading_values: list[float] = []
        self.omega_deltas: list[float] = []
        self.previous_omega: Optional[float] = None
        self.omega_saturations = 0
        self.compute_times_us: list[float] = []
        self.last_distance = math.inf
        self.max_reference_index = 0
        self.path_size = 0
        self.finished = False

    @staticmethod
    def _append(path: Path, columns, row) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        create_header = not path.exists() or path.stat().st_size == 0
        with path.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            if create_header:
                writer.writeheader()
            writer.writerow(row)

    def start(self, timestamp_s: float, path_size: int) -> None:
        self.run_id += 1
        self.started_at = timestamp_s
        self.cte_values, self.heading_values, self.omega_deltas = [], [], []
        self.previous_omega = None
        self.omega_saturations = 0
        self.compute_times_us = []
        self.last_distance = math.inf
        self.max_reference_index = 0
        self.path_size = max(1, path_size)
        self.finished = False

    def sample(self, timestamp_s: float, robot: Pose2D, command: TrackingCommand, compute_time_us: float = 0.0) -> None:
        if self.started_at is None:
            self.start(timestamp_s, 1)
        self.cte_values.append(command.cross_track_error)
        self.heading_values.append(command.heading_error)
        if self.previous_omega is not None:
            self.omega_deltas.append(abs(command.angular_velocity - self.previous_omega))
        self.previous_omega = command.angular_velocity
        self.omega_saturations += int(command.saturated_omega)
        self.compute_times_us.append(float(compute_time_us))
        self.last_distance = command.distance_to_goal
        self.max_reference_index = max(self.max_reference_index, command.reference_index)
        self._append(self.trace_path, TRACE_COLUMNS, {
            "method": self.method, "scenario": self.scenario, "trial": self.trial, "run_id": self.run_id,
            "execution_mode": self.execution_mode,
            "environment": self.environment,
            "timestamp_s": f"{timestamp_s:.6f}", "x_robot": f"{robot.x:.6f}",
            "y_robot": f"{robot.y:.6f}", "yaw_robot": f"{robot.yaw:.6f}",
            "x_reference": f"{command.reference.x:.6f}", "y_reference": f"{command.reference.y:.6f}",
            "yaw_reference": f"{command.reference.yaw:.6f}", "reference_index": command.reference_index,
            "cross_track_error": f"{command.cross_track_error:.6f}",
            "heading_error": f"{command.heading_error:.6f}",
            "linear_velocity_command": f"{command.linear_velocity:.6f}",
            "angular_velocity_command": f"{command.angular_velocity:.6f}",
            "distance_to_goal": f"{command.distance_to_goal:.6f}",
            "saturated_v": int(command.saturated_v), "saturated_omega": int(command.saturated_omega),
        })

    def finish(self, timestamp_s: float, success: bool, reason: str, collision_metric: str = "not_measured") -> None:
        if self.finished or self.started_at is None:
            return
        samples = len(self.cte_values)
        cte_abs = [abs(item) for item in self.cte_values]
        heading_sq = [item * item for item in self.heading_values]
        cte_sq = [item * item for item in self.cte_values]
        self._append(self.summary_path, SUMMARY_COLUMNS, {
            "method": self.method, "scenario": self.scenario, "trial": self.trial, "run_id": self.run_id,
            "execution_mode": self.execution_mode,
            "environment": self.environment,
            "success": int(success), "reason": reason, "samples": samples,
            "cte_rmse_m": "" if not samples else f"{math.sqrt(sum(cte_sq) / samples):.6f}",
            "cte_mae_m": "" if not samples else f"{sum(cte_abs) / samples:.6f}",
            "cte_max_m": "" if not samples else f"{max(cte_abs):.6f}",
            "heading_rmse_rad": "" if not samples else f"{math.sqrt(sum(heading_sq) / samples):.6f}",
            "completion_time_s": f"{max(0.0, timestamp_s - self.started_at):.6f}",
            "final_goal_error_m": "" if not math.isfinite(self.last_distance) else f"{self.last_distance:.6f}",
            "progress_percent": f"{100.0 * self.max_reference_index / max(1, self.path_size - 1):.3f}",
            "omega_saturations": self.omega_saturations,
            "mean_abs_delta_omega_rad_s": "" if not self.omega_deltas else f"{sum(self.omega_deltas) / len(self.omega_deltas):.6f}",
            "mean_control_compute_us": "" if not self.compute_times_us else f"{sum(self.compute_times_us) / len(self.compute_times_us):.6f}",
            "collision_metric": collision_metric,
        })
        self.finished = True
