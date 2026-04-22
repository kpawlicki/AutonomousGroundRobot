"""
LiDAR sensor interface.

Supports 2-D spinning LiDARs that provide a 360-degree distance scan via a
serial or USB interface (e.g. RPLIDAR A1/A2/S series).
Falls back to simulation when no port is configured.
"""

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional

from .base import BaseSensor, SensorReading
from ..utils.logger import get_logger

_log = get_logger("sensors.lidar")


@dataclass
class LidarReading(SensorReading):
    """One complete 360-degree LiDAR scan."""

    angles: List[float] = field(default_factory=list)     # radians
    distances: List[float] = field(default_factory=list)  # metres
    num_points: int = 0

    def get_sector_min(self, angle_min_deg: float, angle_max_deg: float) -> float:
        """Return the minimum distance in a given angular sector (degrees)."""
        lo, hi = math.radians(angle_min_deg), math.radians(angle_max_deg)
        dists = [
            d for a, d in zip(self.angles, self.distances)
            if lo <= a <= hi and d > 0.0
        ]
        return min(dists) if dists else float("inf")

    def to_cartesian(self) -> List[tuple]:
        """Return [(x, y), ...] point-cloud in the sensor frame (metres)."""
        return [
            (d * math.cos(a), d * math.sin(a))
            for a, d in zip(self.angles, self.distances)
            if d > 0.0
        ]


class LidarSensor(BaseSensor):
    """
    2-D LiDAR sensor interface.

    Config keys
    -----------
    port : str
        Serial/USB port.  Omit for simulation mode.
    baud : int
        Baud rate (default ``115200``).
    num_points : int
        Points per scan in simulation mode (default ``360``).
    max_range : float
        Maximum simulated range in metres (default ``12.0``).
    obstacle_distance : float
        Distance to a simulated obstacle in front (default ``2.5`` m).
    obstacle_width_deg : float
        Angular width of the simulated obstacle in degrees (default ``30``).
    """

    def __init__(self, config: Optional[dict] = None) -> None:
        super().__init__("lidar", config)
        self._driver = None
        self._sim_mode = "port" not in self.config
        self._num_points = int(self.config.get("num_points", 360))
        self._max_range = float(self.config.get("max_range", 12.0))
        self._obs_dist = float(self.config.get("obstacle_distance", 2.5))
        self._obs_width = float(self.config.get("obstacle_width_deg", 30.0))

    def open(self) -> None:
        if self._sim_mode:
            _log.info("LiDAR sensor opened in simulation mode")
            self._open = True
            return
        try:
            from rplidar import RPLidar  # type: ignore

            self._driver = RPLidar(self.config["port"])
            _log.info("RPLidar opened on %s", self.config["port"])
            self._open = True
        except Exception as exc:  # pragma: no cover
            _log.error("Failed to open LiDAR: %s", exc)
            raise

    def close(self) -> None:
        if self._driver is not None:
            try:
                self._driver.stop()
                self._driver.disconnect()
            except Exception:  # pragma: no cover
                pass
            self._driver = None
        self._open = False
        _log.info("LiDAR sensor closed")

    def read(self) -> LidarReading:
        if self._sim_mode:
            return self._sim_read()
        return self._driver_read()

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------

    def _sim_read(self) -> LidarReading:
        half_obs = math.radians(self._obs_width / 2.0)
        angles, distances = [], []
        for i in range(self._num_points):
            angle = 2 * math.pi * i / self._num_points
            if abs(angle - 0) <= half_obs or abs(angle - 2 * math.pi) <= half_obs:
                dist = self._obs_dist
            else:
                dist = self._max_range
            angles.append(angle)
            distances.append(dist)
        return LidarReading(
            angles=angles,
            distances=distances,
            num_points=self._num_points,
            timestamp=time.time(),
            valid=True,
        )

    # ------------------------------------------------------------------
    # Hardware
    # ------------------------------------------------------------------

    def _driver_read(self) -> LidarReading:  # pragma: no cover
        angles, distances = [], []
        try:
            for scan in self._driver.iter_scans(max_buf_meas=500):
                for _quality, angle_deg, dist_mm in scan:
                    angles.append(math.radians(angle_deg))
                    distances.append(dist_mm / 1000.0)
                break  # one complete scan
        except Exception as exc:
            _log.warning("LiDAR read error: %s", exc)
            return LidarReading(valid=False)
        return LidarReading(
            angles=angles,
            distances=distances,
            num_points=len(angles),
            timestamp=time.time(),
            valid=bool(angles),
        )
