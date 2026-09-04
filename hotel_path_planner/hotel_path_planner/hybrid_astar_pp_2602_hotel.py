"""Differential-drive Hybrid A* ROS 2 node for Team Hotel (2602)."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException

from .planner_node_utils import PlannerNodeBase
from .planning_core import HybridAStarPlanner


class HybridAStarPP2602Hotel(PlannerNodeBase):
    method_name = "hybrid_astar_pp_2602_hotel"

    def __init__(self) -> None:
        super().__init__(self.method_name)
        # Spatial states are quantized independently from the OccupancyGrid;
        # use the map resolution in fair A/B experiments.
        self.declare_parameter("xy_resolution", 0.05)
        self.declare_parameter("theta_resolution", math.radians(15.0))
        self.declare_parameter("motion_step", 0.20)
        self.declare_parameter("heuristic_weight", 1.0)
        self.declare_parameter("max_curvature", 1.6)
        self.declare_parameter("allow_reverse", False)
        # A differential drive can rotate about its centre; this is not an
        # Ackermann steering surrogate.  It is penalised in equivalent cost.
        self.declare_parameter("allow_in_place_rotation", True)
        self.declare_parameter("reverse_penalty", 1.4)
        self.declare_parameter("turn_penalty", 0.10)
        self.declare_parameter("direction_change_penalty", 0.25)
        self.declare_parameter("rotation_penalty", 0.12)
        self.declare_parameter("goal_position_tolerance", 0.15)
        self.declare_parameter("goal_yaw_tolerance", math.radians(20.0))
        self.declare_parameter("max_iterations", 100000)
        self.get_logger().info(
            "Differential-drive Hybrid A* ready. Its primitives are unicycle arcs "
            "and optional in-place rotations."
        )

    def build_planner(self) -> HybridAStarPlanner:
        return HybridAStarPlanner(
            self.grid,
            xy_resolution=float(self.get_parameter("xy_resolution").value),
            theta_resolution=float(self.get_parameter("theta_resolution").value),
            motion_step=float(self.get_parameter("motion_step").value),
            heuristic_weight=float(self.get_parameter("heuristic_weight").value),
            max_curvature=float(self.get_parameter("max_curvature").value),
            allow_reverse=bool(self.get_parameter("allow_reverse").value),
            allow_in_place_rotation=bool(self.get_parameter("allow_in_place_rotation").value),
            reverse_penalty=float(self.get_parameter("reverse_penalty").value),
            turn_penalty=float(self.get_parameter("turn_penalty").value),
            direction_change_penalty=float(self.get_parameter("direction_change_penalty").value),
            rotation_penalty=float(self.get_parameter("rotation_penalty").value),
            goal_position_tolerance=float(self.get_parameter("goal_position_tolerance").value),
            goal_yaw_tolerance=float(self.get_parameter("goal_yaw_tolerance").value),
            max_iterations=int(self.get_parameter("max_iterations").value),
            traversal_cost_weight=float(self.get_parameter("traversal_cost_weight").value),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HybridAStarPP2602Hotel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
