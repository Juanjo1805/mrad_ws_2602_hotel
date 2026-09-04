"""Instrumented Dijkstra baseline for the 2602 Hotel planning comparison."""

import rclpy
from rclpy.executors import ExternalShutdownException

from .planner_node_utils import PlannerNodeBase
from .planning_core import DijkstraPlanner


class DijkstraPP2602Hotel(PlannerNodeBase):
    method_name = "dijkstra_pp_2602_hotel"

    def __init__(self) -> None:
        super().__init__(self.method_name)
        self.declare_parameter("use_8_connected", True)
        self.declare_parameter("prevent_corner_cutting", True)
        self.get_logger().info("Dijkstra baseline ready; waiting for /map and /goal_pose.")

    def build_planner(self) -> DijkstraPlanner:
        return DijkstraPlanner(
            self.grid,
            use_8_connected=bool(self.get_parameter("use_8_connected").value),
            prevent_corner_cutting=bool(self.get_parameter("prevent_corner_cutting").value),
            traversal_cost_weight=float(self.get_parameter("traversal_cost_weight").value),
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DijkstraPP2602Hotel()
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
