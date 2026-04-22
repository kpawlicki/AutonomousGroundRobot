"""
Occupancy grid and path planner.

The :class:`OccupancyGrid` converts raw sensor data (LiDAR, depth camera,
ultrasonics) into a discrete binary grid.
The :class:`PathPlanner` selects an algorithm and plans a path from the
current position to a goal expressed in local ENU coordinates.
"""

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

from .algorithms import BaseAlgorithm, AlgorithmResult, AStarAlgorithm
from ..sensors import LidarReading, DepthFrame, UltrasonicReading
from ..utils.coordinates import LocalPoint
from ..utils.logger import get_logger

_log = get_logger("navigation.path_planner")

# Type alias
Path = List[Tuple[int, int]]


# ---------------------------------------------------------------------------
# Occupancy Grid
# ---------------------------------------------------------------------------

@dataclass
class OccupancyGrid:
    """
    Binary occupancy grid centred on the robot.

    Attributes
    ----------
    resolution : float
        Metres per cell.
    width : int
        Number of columns.
    height : int
        Number of rows.
    origin_row : int
        Row index corresponding to the robot's position.
    origin_col : int
        Column index corresponding to the robot's position.
    """

    resolution: float = 0.1   # metres/cell
    width: int = 201
    height: int = 201
    origin_row: int = 100
    origin_col: int = 100
    _cells: List[List[bool]] = field(default_factory=list, repr=False)

    def __post_init__(self) -> None:
        if not self._cells:
            self._cells = [[False] * self.width for _ in range(self.height)]

    # ------------------------------------------------------------------
    # Coordinate conversions
    # ------------------------------------------------------------------

    def local_to_cell(self, point: LocalPoint) -> Tuple[int, int]:
        """Convert a local ENU point to a (row, col) grid cell."""
        col = int(round(self.origin_col + point.x / self.resolution))
        row = int(round(self.origin_row - point.y / self.resolution))
        row = max(0, min(self.height - 1, row))
        col = max(0, min(self.width - 1, col))
        return row, col

    def cell_to_local(self, row: int, col: int) -> LocalPoint:
        """Convert a (row, col) grid cell to a local ENU point."""
        x = (col - self.origin_col) * self.resolution
        y = (self.origin_row - row) * self.resolution
        return LocalPoint(x=x, y=y)

    # ------------------------------------------------------------------
    # Grid manipulation
    # ------------------------------------------------------------------

    def clear(self) -> None:
        """Reset all cells to free."""
        for r in range(self.height):
            for c in range(self.width):
                self._cells[r][c] = False

    def mark_occupied(self, row: int, col: int, inflate: int = 1) -> None:
        """Mark a cell and its neighbours (inflation) as occupied."""
        for dr in range(-inflate, inflate + 1):
            for dc in range(-inflate, inflate + 1):
                r, c = row + dr, col + dc
                if 0 <= r < self.height and 0 <= c < self.width:
                    self._cells[r][c] = True

    def get_cells(self) -> List[List[bool]]:
        return self._cells

    def is_occupied(self, row: int, col: int) -> bool:
        if 0 <= row < self.height and 0 <= col < self.width:
            return self._cells[row][col]
        return True  # out-of-bounds treated as occupied

    # ------------------------------------------------------------------
    # Sensor data ingestion
    # ------------------------------------------------------------------

    def update_from_lidar(
        self,
        reading: LidarReading,
        inflate: int = 1,
    ) -> None:
        """Project a LiDAR scan into the occupancy grid."""
        self.clear()
        for x, y in reading.to_cartesian():
            cell = self.local_to_cell(LocalPoint(x=x, y=y))
            self.mark_occupied(*cell, inflate=inflate)

    def update_from_depth(
        self,
        frame: DepthFrame,
        hfov_deg: float = 86.0,
        obstacle_threshold: float = 1.0,
        inflate: int = 1,
    ) -> None:
        """
        Project a depth camera frame into the occupancy grid.

        Pixels closer than *obstacle_threshold* metres are marked as
        occupied.  Only the horizontal centre row is used for a simple
        2-D approximation.
        """
        if not frame.valid or frame.width == 0:
            return
        hfov = math.radians(hfov_deg)
        row = frame.height // 2
        for col in range(frame.width):
            d = frame.get_pixel(row, col)
            if 0 < d < obstacle_threshold:
                angle = hfov * (col / frame.width - 0.5)
                x = d * math.sin(angle)
                y = d * math.cos(angle)
                cell = self.local_to_cell(LocalPoint(x=x, y=y))
                self.mark_occupied(*cell, inflate=inflate)

    def update_from_ultrasonic(
        self,
        readings: List[UltrasonicReading],
        obstacle_threshold: float = 0.5,
        inflate: int = 2,
    ) -> None:
        """
        Mark ultrasonic readings as obstacles.

        Direction labels (``"front"``, ``"rear"``, ``"left"``, ``"right"``)
        are mapped to approximate angles.
        """
        direction_angles = {
            "front": 0.0,
            "rear": math.pi,
            "left": -math.pi / 2,
            "right": math.pi / 2,
        }
        for reading in readings:
            if not reading.valid or reading.distance == float("inf"):
                continue
            if reading.distance < obstacle_threshold:
                angle = direction_angles.get(reading.direction, 0.0)
                x = reading.distance * math.sin(angle)
                y = reading.distance * math.cos(angle)
                cell = self.local_to_cell(LocalPoint(x=x, y=y))
                self.mark_occupied(*cell, inflate=inflate)


# ---------------------------------------------------------------------------
# Path Planner
# ---------------------------------------------------------------------------

class PathPlanner:
    """
    High-level path planner.

    Wraps an :class:`BaseAlgorithm` and translates between local ENU
    coordinates and grid cells.

    Parameters
    ----------
    algorithm : BaseAlgorithm
        Path-finding algorithm to use (default: A*).
    grid : OccupancyGrid
        Occupancy grid (shared with the navigation manager).
    """

    def __init__(
        self,
        algorithm: Optional[BaseAlgorithm] = None,
        grid: Optional[OccupancyGrid] = None,
    ) -> None:
        self.algorithm: BaseAlgorithm = algorithm or AStarAlgorithm()
        self.grid: OccupancyGrid = grid or OccupancyGrid()

    def plan(
        self,
        start_local: LocalPoint,
        goal_local: LocalPoint,
    ) -> Optional[List[LocalPoint]]:
        """
        Plan a path from *start_local* to *goal_local* in local ENU coords.

        Returns a list of :class:`LocalPoint` waypoints, or ``None`` if no
        path could be found.
        """
        start_cell = self.grid.local_to_cell(start_local)
        goal_cell = self.grid.local_to_cell(goal_local)

        _log.debug(
            "Planning %s → %s  (algorithm=%s)",
            start_cell, goal_cell, self.algorithm.name,
        )

        result: AlgorithmResult = self.algorithm.find_path(
            self.grid.get_cells(), start_cell, goal_cell
        )

        if not result.found and not result.path:
            _log.warning("No path found from %s to %s", start_cell, goal_cell)
            return None

        waypoints = [self.grid.cell_to_local(r, c) for r, c in result.path]
        _log.debug("Path found: %d waypoints, cost=%.2f", len(waypoints), result.cost)
        return waypoints
