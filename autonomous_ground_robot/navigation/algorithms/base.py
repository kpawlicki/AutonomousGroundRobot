"""Abstract base class for navigation algorithms."""

import abc
from dataclasses import dataclass, field
from typing import List, Optional, Tuple


@dataclass
class AlgorithmResult:
    """
    The output of a path-finding algorithm.

    Attributes
    ----------
    path : List[Tuple[int, int]]
        Sequence of (row, col) grid cells from start to goal.
        Empty if no path was found.
    found : bool
        True when a path to the goal was successfully computed.
    cost : float
        Total path cost (algorithm-specific units).
    """

    path: List[Tuple[int, int]] = field(default_factory=list)
    found: bool = False
    cost: float = 0.0


class BaseAlgorithm(abc.ABC):
    """
    Abstract navigation algorithm.

    Subclasses receive a binary occupancy grid (2-D list where ``True``
    marks an occupied / impassable cell) and must return an
    :class:`AlgorithmResult`.
    """

    name: str = "base"

    @abc.abstractmethod
    def find_path(
        self,
        grid: List[List[bool]],
        start: Tuple[int, int],
        goal: Tuple[int, int],
    ) -> AlgorithmResult:
        """
        Find a path on *grid* from *start* to *goal*.

        Parameters
        ----------
        grid :
            2-D list ``grid[row][col]`` – ``True`` = obstacle, ``False`` = free.
        start :
            ``(row, col)`` of the starting cell.
        goal :
            ``(row, col)`` of the goal cell.

        Returns
        -------
        AlgorithmResult
        """

    # ------------------------------------------------------------------
    # Helpers shared by grid-based algorithms
    # ------------------------------------------------------------------

    @staticmethod
    def _neighbors(
        row: int,
        col: int,
        rows: int,
        cols: int,
        allow_diagonal: bool = True,
    ) -> List[Tuple[int, int]]:
        """Yield valid (row, col) neighbours."""
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if allow_diagonal:
            directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]
        result = []
        for dr, dc in directions:
            nr, nc = row + dr, col + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                result.append((nr, nc))
        return result

    @staticmethod
    def _reconstruct_path(
        came_from: dict,
        current: Tuple[int, int],
    ) -> List[Tuple[int, int]]:
        """Reconstruct path by following *came_from* back to the start."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path
