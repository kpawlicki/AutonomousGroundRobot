"""
Dijkstra's shortest-path algorithm.

Classic uniform-cost search – equivalent to A* with a zero heuristic.
Guaranteed to find the globally optimal path.
"""

import heapq
import math
from typing import Dict, List, Tuple

from .base import BaseAlgorithm, AlgorithmResult


class DijkstraAlgorithm(BaseAlgorithm):
    """
    Dijkstra shortest-path on a 2-D occupancy grid.

    Parameters
    ----------
    allow_diagonal : bool
        Whether diagonal moves are permitted (default ``True``).
    """

    name = "dijkstra"

    def __init__(self, allow_diagonal: bool = True) -> None:
        self.allow_diagonal = allow_diagonal

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

        dist: Dict[Tuple[int, int], float] = {start: 0.0}
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        heap: List[Tuple[float, Tuple[int, int]]] = [(0.0, start)]

        while heap:
            cost, current = heapq.heappop(heap)

            if cost > dist.get(current, float("inf")):
                continue  # stale entry

            if current == goal:
                path = self._reconstruct_path(came_from, current)
                return AlgorithmResult(path=path, found=True, cost=dist[goal])

            for nb in self._neighbors(*current, rows, cols, self.allow_diagonal):
                if grid[nb[0]][nb[1]]:
                    continue
                dr = abs(nb[0] - current[0])
                dc = abs(nb[1] - current[1])
                move_cost = math.sqrt(2) if dr and dc else 1.0
                new_cost = dist[current] + move_cost
                if new_cost < dist.get(nb, float("inf")):
                    dist[nb] = new_cost
                    came_from[nb] = current
                    heapq.heappush(heap, (new_cost, nb))

        return AlgorithmResult(found=False)
