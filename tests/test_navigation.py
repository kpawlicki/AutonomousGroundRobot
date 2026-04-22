"""Tests for navigation algorithms and path planner."""

import math
import pytest

from autonomous_ground_robot.navigation.algorithms import (
    AStarAlgorithm,
    DijkstraAlgorithm,
    PotentialFieldsAlgorithm,
    AlgorithmResult,
)
from autonomous_ground_robot.navigation.path_planner import OccupancyGrid, PathPlanner
from autonomous_ground_robot.utils.coordinates import LocalPoint


# ---------------------------------------------------------------------------
# Shared helpers
# ---------------------------------------------------------------------------

def make_empty_grid(rows=10, cols=10):
    return [[False] * cols for _ in range(rows)]


def make_grid_with_wall(rows=10, cols=10, wall_col=5):
    """A grid with a vertical wall at wall_col, except for a gap at row 0."""
    grid = make_empty_grid(rows, cols)
    for r in range(1, rows):
        grid[r][wall_col] = True
    return grid


# ---------------------------------------------------------------------------
# A*
# ---------------------------------------------------------------------------

class TestAStarAlgorithm:
    def test_finds_straight_path(self):
        grid = make_empty_grid(5, 5)
        algo = AStarAlgorithm()
        result = algo.find_path(grid, (0, 0), (4, 4))
        assert result.found
        assert result.path[0] == (0, 0)
        assert result.path[-1] == (4, 4)
        assert result.cost > 0

    def test_no_path_when_goal_blocked(self):
        grid = make_empty_grid(5, 5)
        grid[4][4] = True
        algo = AStarAlgorithm()
        result = algo.find_path(grid, (0, 0), (4, 4))
        assert not result.found

    def test_no_path_when_start_blocked(self):
        grid = make_empty_grid(5, 5)
        grid[0][0] = True
        algo = AStarAlgorithm()
        result = algo.find_path(grid, (0, 0), (4, 4))
        assert not result.found

    def test_finds_path_around_wall(self):
        grid = make_grid_with_wall(rows=10, cols=10, wall_col=5)
        algo = AStarAlgorithm()
        result = algo.find_path(grid, (3, 3), (3, 7))
        assert result.found
        # Path must not pass through the wall (row > 0, col == 5)
        for r, c in result.path:
            assert not grid[r][c], f"Path passes through obstacle at ({r},{c})"

    def test_path_continuity(self):
        grid = make_empty_grid(10, 10)
        algo = AStarAlgorithm()
        result = algo.find_path(grid, (0, 0), (9, 9))
        assert result.found
        for i in range(1, len(result.path)):
            pr, pc = result.path[i - 1]
            cr, cc = result.path[i]
            assert abs(pr - cr) <= 1 and abs(pc - cc) <= 1

    def test_same_start_and_goal(self):
        grid = make_empty_grid(5, 5)
        algo = AStarAlgorithm()
        result = algo.find_path(grid, (2, 2), (2, 2))
        assert result.found
        assert result.path == [(2, 2)]

    def test_weighted_heuristic(self):
        grid = make_empty_grid(10, 10)
        algo = AStarAlgorithm(weight=2.0)
        result = algo.find_path(grid, (0, 0), (9, 9))
        assert result.found

    def test_no_diagonal(self):
        grid = make_empty_grid(5, 5)
        algo = AStarAlgorithm(allow_diagonal=False)
        result = algo.find_path(grid, (0, 0), (4, 4))
        assert result.found
        for i in range(1, len(result.path)):
            pr, pc = result.path[i - 1]
            cr, cc = result.path[i]
            # No diagonal moves
            assert abs(pr - cr) + abs(pc - cc) == 1


# ---------------------------------------------------------------------------
# Dijkstra
# ---------------------------------------------------------------------------

class TestDijkstraAlgorithm:
    def test_finds_straight_path(self):
        grid = make_empty_grid(5, 5)
        algo = DijkstraAlgorithm()
        result = algo.find_path(grid, (0, 0), (4, 4))
        assert result.found
        assert result.path[0] == (0, 0)
        assert result.path[-1] == (4, 4)

    def test_no_path_when_fully_blocked(self):
        # Surround goal with obstacles
        grid = make_empty_grid(5, 5)
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            r, c = 4 + dr, 4 + dc
            if 0 <= r < 5 and 0 <= c < 5:
                grid[r][c] = True
        grid[4][4] = False
        # Also block diagonal routes
        for dr in [-1, 1]:
            for dc in [-1, 1]:
                if 0 <= 4 + dr < 5 and 0 <= 4 + dc < 5:
                    grid[4 + dr][4 + dc] = True
        algo = DijkstraAlgorithm(allow_diagonal=False)
        result = algo.find_path(grid, (0, 0), (4, 4))
        assert not result.found

    def test_finds_path_around_wall(self):
        grid = make_grid_with_wall(rows=10, cols=10, wall_col=5)
        algo = DijkstraAlgorithm()
        result = algo.find_path(grid, (3, 3), (3, 7))
        assert result.found

    def test_optimal_cost_vs_astar(self):
        """Dijkstra and A* should find paths of equal cost on open grids."""
        grid = make_empty_grid(10, 10)
        start, goal = (0, 0), (9, 9)
        astar = AStarAlgorithm()
        dijkstra = DijkstraAlgorithm()
        r1 = astar.find_path(grid, start, goal)
        r2 = dijkstra.find_path(grid, start, goal)
        assert r1.found and r2.found
        assert abs(r1.cost - r2.cost) < 1e-6


# ---------------------------------------------------------------------------
# Potential Fields
# ---------------------------------------------------------------------------

class TestPotentialFieldsAlgorithm:
    def test_moves_towards_goal(self):
        grid = make_empty_grid(20, 20)
        algo = PotentialFieldsAlgorithm()
        start = (10, 10)
        goal = (10, 15)
        result = algo.find_path(grid, start, goal)
        assert len(result.path) >= 1
        # Next step should be closer to goal
        if len(result.path) > 1:
            next_cell = result.path[1]
            dist_before = abs(start[1] - goal[1])
            dist_after = abs(next_cell[1] - goal[1])
            assert dist_after <= dist_before

    def test_reached_when_already_at_goal(self):
        grid = make_empty_grid(10, 10)
        algo = PotentialFieldsAlgorithm(step_size=2.0)
        result = algo.find_path(grid, (5, 5), (5, 5))
        assert result.found

    def test_obstacles_repel(self):
        grid = make_empty_grid(20, 20)
        algo = PotentialFieldsAlgorithm(
            repulsive_gain=200.0,
            influence_radius=3.0,
        )
        algo.set_obstacles([(10, 12)])  # obstacle to the right
        start = (10, 10)
        goal = (10, 18)
        result = algo.find_path(grid, start, goal)
        # Should still produce a path (may go around)
        assert len(result.path) >= 1

    def test_set_obstacles(self):
        algo = PotentialFieldsAlgorithm()
        algo.set_obstacles([(1, 2), (3, 4)])
        assert algo._obstacles == [(1, 2), (3, 4)]


# ---------------------------------------------------------------------------
# Occupancy Grid
# ---------------------------------------------------------------------------

class TestOccupancyGrid:
    def test_local_to_cell_roundtrip(self):
        g = OccupancyGrid(resolution=0.1, width=101, height=101, origin_row=50, origin_col=50)
        point = LocalPoint(x=1.0, y=2.0)
        cell = g.local_to_cell(point)
        back = g.cell_to_local(*cell)
        assert abs(back.x - point.x) < 0.15
        assert abs(back.y - point.y) < 0.15

    def test_mark_occupied_inflates(self):
        g = OccupancyGrid()
        g.mark_occupied(50, 50, inflate=2)
        assert g.is_occupied(50, 50)
        assert g.is_occupied(51, 51)
        assert g.is_occupied(48, 48)

    def test_clear_resets_grid(self):
        g = OccupancyGrid()
        g.mark_occupied(50, 50, inflate=0)
        assert g.is_occupied(50, 50)
        g.clear()
        assert not g.is_occupied(50, 50)

    def test_out_of_bounds_is_occupied(self):
        g = OccupancyGrid(width=10, height=10)
        assert g.is_occupied(-1, 5)
        assert g.is_occupied(5, 100)

    def test_update_from_lidar(self):
        from autonomous_ground_robot.sensors.lidar import LidarReading
        import math
        # 4-point scan with one point straight ahead at 1 m
        reading = LidarReading(
            angles=[0.0, math.pi / 2, math.pi, 3 * math.pi / 2],
            distances=[1.0, 5.0, 5.0, 5.0],
            num_points=4,
            valid=True,
        )
        g = OccupancyGrid(resolution=0.1, width=201, height=201,
                          origin_row=100, origin_col=100)
        g.update_from_lidar(reading, inflate=0)
        # The point at (1.0, 0) in ENU → col=110, row=100
        assert g.is_occupied(100, 110)

    def test_update_from_ultrasonic(self):
        from autonomous_ground_robot.sensors.ultrasonic import UltrasonicReading
        readings = [
            UltrasonicReading(distance=0.2, direction="front", valid=True),
        ]
        g = OccupancyGrid(resolution=0.1, width=201, height=201,
                          origin_row=100, origin_col=100)
        g.update_from_ultrasonic(readings, obstacle_threshold=0.5, inflate=0)
        # front obstacle → y ≈ 0.2 → row=98, col=100
        cell = g.local_to_cell(LocalPoint(x=0.0, y=0.2))
        assert g.is_occupied(*cell)


# ---------------------------------------------------------------------------
# PathPlanner integration
# ---------------------------------------------------------------------------

class TestPathPlanner:
    def test_plan_returns_waypoints(self):
        planner = PathPlanner()
        waypoints = planner.plan(LocalPoint(0, 0), LocalPoint(5, 5))
        assert waypoints is not None
        assert len(waypoints) >= 2

    def test_plan_blocked_returns_none(self):
        g = OccupancyGrid(resolution=0.1, width=21, height=21,
                          origin_row=10, origin_col=10)
        # Block the goal cell
        goal = g.local_to_cell(LocalPoint(x=0.5, y=0.5))
        g.mark_occupied(*goal, inflate=0)
        planner = PathPlanner(grid=g)
        result = planner.plan(LocalPoint(0, 0), LocalPoint(0.5, 0.5))
        assert result is None

    def test_plan_uses_astar_by_default(self):
        planner = PathPlanner()
        assert planner.algorithm.name == "astar"

    def test_plan_dijkstra(self):
        from autonomous_ground_robot.navigation.algorithms import DijkstraAlgorithm
        planner = PathPlanner(algorithm=DijkstraAlgorithm())
        waypoints = planner.plan(LocalPoint(0, 0), LocalPoint(3, 3))
        assert waypoints is not None
        assert len(waypoints) >= 2
