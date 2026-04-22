"""
A* (A-star) path-finding algorithm.

Uses the octile distance heuristic for grids that allow diagonal movement.
"""

import heapq
import math
from typing import Dict, List, Optional, Tuple

from .base import BaseAlgorithm, AlgorithmResult


class AStarAlgorithm(BaseAlgorithm):
    """
    A* shortest-path algorithm on a 2-D occupancy grid.

    Parameters
    ----------
    allow_diagonal : bool
        Whether diagonal moves are permitted (default ``True``).
    weight : float
        Heuristic weight ≥ 1.  ``1.0`` gives optimal paths; values > 1
        speed up search at the cost of optimality (default ``1.0``).
    """

    name = "astar"

    def __init__(self, allow_diagonal: bool = True, weight: float = 1.0) -> None:
        self.allow_diagonal = allow_diagonal
        self.weight = weight

    def find_path(
        self,
        grid: List[List[bool]],
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> AlgorithmResult:
        rows = len(grid)
        cols = len(grid[0]) if rows else 0

        if grid[start[0]][start[1]] or grid[goal[0]][goal[1]]:
            return AlgorithmResult(found=False)

        # g_score[cell] = cost from start to cell
        g_score: Dict[Tuple[int, int], float] = {start: 0.0}
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        # open_heap entries: (f_score, cell)
        open_heap: List[Tuple[float, Tuple[int, int]]] = []
        heapq.heappush(open_heap, (self._h(start, goal), start))
        in_open = {start}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            in_open.discard(current)

            if current == goal:
                path = self._reconstruct_path(came_from, current)
                return AlgorithmResult(path=path, found=True, cost=g_score[goal])

            for nb in self._neighbors(*current, rows, cols, self.allow_diagonal):
                if grid[nb[0]][nb[1]]:
                    continue  # obstacle
                dr = abs(nb[0] - current[0])
                dc = abs(nb[1] - current[1])
                move_cost = math.sqrt(2) if dr and dc else 1.0
                tentative_g = g_score[current] + move_cost
                if tentative_g < g_score.get(nb, float("inf")):
                    came_from[nb] = current
                    g_score[nb] = tentative_g
                    f = tentative_g + self.weight * self._h(nb, goal)
                    if nb not in in_open:
                        heapq.heappush(open_heap, (f, nb))
                        in_open.add(nb)

        return AlgorithmResult(found=False)

    # ------------------------------------------------------------------
    # Heuristic
    # ------------------------------------------------------------------

    def _h(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Octile distance heuristic."""
        dr = abs(a[0] - b[0])
        dc = abs(a[1] - b[1])
        return (dr + dc) + (math.sqrt(2) - 2) * min(dr, dc)
