import math

import numpy as np

from hotel_path_planner.planning_core import (
    DijkstraPlanner,
    GridMap,
    HybridAStarPlanner,
    HybridState,
    Pose2D,
    wrap_to_pi,
)


def free_grid(size=30):
    return GridMap(np.zeros((size, size), dtype=np.int16), 0.1, 0.0, 0.0, inflate_radius=0.0)


def test_world_grid_round_trip_uses_cell_centres():
    grid = free_grid()
    assert grid.world_to_grid(0.24, 0.25) == (2, 2)
    assert grid.grid_to_world(2, 2) == (0.25, 0.25)
    assert grid.world_to_grid(-0.01, 0.0) is None


def test_theta_discretization_wraps_equivalent_angles():
    planner = HybridAStarPlanner(free_grid(), theta_resolution=math.pi / 6.0)
    assert planner.theta_to_bin(-math.pi) == planner.theta_to_bin(math.pi)
    assert abs(wrap_to_pi(planner.bin_to_theta(planner.theta_to_bin(0.0)))) < math.pi / 6.0


def test_arc_collision_checks_intermediate_samples():
    occupancy = np.zeros((30, 30), dtype=np.int16)
    occupancy[10, 11] = 100
    grid = GridMap(occupancy, 0.1, 0.0, 0.0, inflate_radius=0.0)
    planner = HybridAStarPlanner(
        grid, motion_step=0.4, max_curvature=0.0, allow_in_place_rotation=False
    )
    start = Pose2D(0.85, 1.05, 0.0)
    # The arc crosses occupied cell (11,10), so no straight successor is valid.
    assert list(planner.successors(HybridState(start.x, start.y, start.yaw))) == []


def test_hybrid_reconstructs_oriented_path_with_in_place_goal_rotation():
    grid = free_grid()
    planner = HybridAStarPlanner(grid, xy_resolution=0.1, theta_resolution=math.pi / 2.0,
                                 motion_step=0.2, max_curvature=0.0, goal_position_tolerance=0.05,
                                 goal_yaw_tolerance=0.1, max_iterations=1000)
    result = planner.plan(Pose2D(0.25, 0.25, 0.0), Pose2D(0.45, 0.25, math.pi / 2.0))
    assert result.success
    assert abs(wrap_to_pi(result.poses[-1].yaw - math.pi / 2.0)) < 1e-9
    assert len(result.poses) >= 3


def test_dijkstra_refuses_blocked_goal_instead_of_returning_a_fake_path():
    occupancy = np.zeros((10, 10), dtype=np.int16)
    occupancy[5, 5] = 100
    grid = GridMap(occupancy, 0.1, 0.0, 0.0, inflate_radius=0.0)
    result = DijkstraPlanner(grid).plan(Pose2D(0.15, 0.15), Pose2D(0.55, 0.55))
    assert not result.success
    assert result.reason == "goal_occupied"
