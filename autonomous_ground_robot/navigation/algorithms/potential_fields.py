"""
Artificial Potential Fields (APF) navigation algorithm.

APF is a reactive algorithm.  It does not search a global grid but instead
computes a velocity command by:
  * Attracting the robot towards the goal (attractive potential gradient).
  * Repelling the robot from detected obstacles (repulsive potential gradient).

The algorithm returns a virtual "path" of just two cells – current position
and one step in the direction of the combined force – to stay compatible with
the :class:`BaseAlgorithm` interface used by the path planner.  The motor
controller should call this algorithm at high frequency (e.g. 10–20 Hz) using
the latest sensor data rather than planning a long global path.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .base import BaseAlgorithm, AlgorithmResult


@dataclass
class _Obstacle:
    row: float
    col: float
    influence_radius: float = 5.0  # cells


class PotentialFieldsAlgorithm(BaseAlgorithm):
    """
    Artificial Potential Fields reactive navigation.

    Parameters
    ----------
    attractive_gain : float
        Scalar gain for the attractive force (default ``1.0``).
    repulsive_gain : float
        Scalar gain for the repulsive force (default ``100.0``).
    influence_radius : float
        Distance (grid cells) within which obstacles exert repulsion
        (default ``5.0``).
    step_size : float
        How far to step along the force vector per iteration (default ``1.0``).
    obstacles : list of (row, col)
        Known obstacle positions injected externally (e.g. from LiDAR).
        Updated via :meth:`set_obstacles`.
    """

    name = "potential_fields"

    def __init__(
        self,
        attractive_gain: float = 1.0,
        repulsive_gain: float = 100.0,
        influence_radius: float = 5.0,
        step_size: float = 1.0,
    ) -> None:
        self.attractive_gain = attractive_gain
        self.repulsive_gain = repulsive_gain
        self.influence_radius = influence_radius
        self.step_size = step_size
        self._obstacles: List[Tuple[float, float]] = []

    def set_obstacles(self, obstacles: List[Tuple[float, float]]) -> None:
        """
        Update the list of obstacle positions (row, col) in grid coordinates.
        Call this before each :meth:`find_path` invocation.
        """
        self._obstacles = obstacles

    def find_path(
        self,
        grid: List[List[bool]],
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> AlgorithmResult:
        rows = len(grid)
        cols = len(grid[0]) if rows else 0

        # Collect obstacles from the grid if none were injected externally
        obstacles = list(self._obstacles)
        if not obstacles:
            obstacles = [
                (float(r), float(c))
                for r in range(rows)
                for c in range(cols)
                if grid[r][c]
            ]

        sr, sc = float(start[0]), float(start[1])
        gr, gc = float(goal[0]), float(goal[1])

        # Attractive gradient (towards goal)
        att_r, att_c = self._attractive_gradient(sr, sc, gr, gc)

        # Repulsive gradient (away from obstacles)
        rep_r, rep_c = self._repulsive_gradient(sr, sc, obstacles)

        # Combined force
        force_r = att_r + rep_r
        force_c = att_c + rep_c
        magnitude = math.hypot(force_r, force_c)

        if magnitude < 1e-9:
            # Local minimum – small random perturbation
            force_r, force_c = 0.1, 0.1
            magnitude = math.hypot(force_r, force_c)

        # Normalise and step
        nr = sr + self.step_size * force_r / magnitude
        nc = sc + self.step_size * force_c / magnitude

        # Clamp to grid
        nr = max(0.0, min(float(rows - 1), nr))
        nc = max(0.0, min(float(cols - 1), nc))
        next_cell = (int(round(nr)), int(round(nc)))

        # Check we are not stepping into an obstacle
        if grid[next_cell[0]][next_cell[1]]:
            return AlgorithmResult(path=[start], found=False, cost=0.0)

        path = [start] if next_cell == start else [start, next_cell]
        dist_to_goal = math.hypot(goal[0] - start[0], goal[1] - start[1])
        reached = dist_to_goal <= self.step_size + 0.5

        return AlgorithmResult(
            path=path,
            found=reached,
            cost=dist_to_goal,
        )

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _attractive_gradient(
        self,
        sr: float, sc: float,
        gr: float, gc: float,
    ) -> Tuple[float, float]:
        """Return the attractive force vector (row, col components)."""
        dr = gr - sr
        dc = gc - sc
        dist = math.hypot(dr, dc)
        if dist < 1e-9:
            return 0.0, 0.0
        return self.attractive_gain * dr, self.attractive_gain * dc

    def _repulsive_gradient(
        self,
        sr: float, sc: float,
        obstacles: List[Tuple[float, float]],
    ) -> Tuple[float, float]:
        """Return the total repulsive force vector."""
        rep_r = rep_c = 0.0
        for obs_r, obs_c in obstacles:
            dr = sr - obs_r
            dc = sc - obs_c
            dist = math.hypot(dr, dc)
            if dist <= 0.0 or dist > self.influence_radius:
                continue
            # Repulsive potential gradient magnitude
            mag = self.repulsive_gain * (1.0 / dist - 1.0 / self.influence_radius) / (dist ** 2)
            rep_r += mag * dr / dist
            rep_c += mag * dc / dist
        return rep_r, rep_c
