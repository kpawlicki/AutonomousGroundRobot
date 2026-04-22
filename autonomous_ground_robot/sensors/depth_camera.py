"""
Depth camera sensor interface.

Abstracts stereo depth cameras such as Intel RealSense D4xx or similar
devices that provide per-pixel depth images.  Simulation mode generates a
synthetic depth frame with a configurable obstacle.
"""

import time
from dataclasses import dataclass, field
from typing import List, Optional

from .base import BaseSensor, SensorReading
from ..utils.logger import get_logger

_log = get_logger("sensors.depth_camera")


@dataclass
class DepthFrame(SensorReading):
    """One depth image frame."""

    width: int = 0
    height: int = 0
    # Flat row-major list of depth values in metres.
    # Use depth[row * width + col] to access individual pixels.
    depth: List[float] = field(default_factory=list)
    min_depth: float = 0.0  # metres
    max_depth: float = 0.0  # metres

    def get_pixel(self, row: int, col: int) -> float:
        """Return the depth (metres) at pixel (row, col)."""
        if 0 <= row < self.height and 0 <= col < self.width:
            return self.depth[row * self.width + col]
        return 0.0

    def center_distance(self) -> float:
        """Return the depth at the centre of the frame."""
        return self.get_pixel(self.height // 2, self.width // 2)

    def min_in_roi(
        self,
        row_start: int,
        row_end: int,
        col_start: int,
        col_end: int,
    ) -> float:
        """Return the minimum non-zero depth in the given region of interest."""
        values = [
            self.depth[r * self.width + c]
            for r in range(max(0, row_start), min(self.height, row_end))
            for c in range(max(0, col_start), min(self.width, col_end))
            if self.depth[r * self.width + c] > 0
        ]
        return min(values) if values else float("inf")


class DepthCameraSensor(BaseSensor):
    """
    Depth camera sensor interface.

    Config keys
    -----------
    width : int
        Frame width in pixels (default ``640``).
    height : int
        Frame height in pixels (default ``480``).
    fps : int
        Frame rate (default ``30``).
    serial_number : str
        RealSense device serial number; omit to use the first available.
    sim_obstacle_distance : float
        Distance to the simulated central obstacle in metres (default ``1.5``).
    sim_background_distance : float
        Background depth in metres (default ``5.0``).
    """

    def __init__(self, config: Optional[dict] = None) -> None:
        super().__init__("depth_camera", config)
        self._pipeline = None
        self._sim_mode = True  # overridden in open() if pyrealsense2 is available
        self._width = int(self.config.get("width", 640))
        self._height = int(self.config.get("height", 480))
        self._fps = int(self.config.get("fps", 30))
        self._sim_obs_dist = float(self.config.get("sim_obstacle_distance", 1.5))
        self._sim_bg_dist = float(self.config.get("sim_background_distance", 5.0))

    def open(self) -> None:
        try:
            import pyrealsense2 as rs  # type: ignore

            self._pipeline = rs.pipeline()
            cfg = rs.config()
            if "serial_number" in self.config:
                cfg.enable_device(self.config["serial_number"])
            cfg.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)
            self._pipeline.start(cfg)
            self._sim_mode = False
            _log.info("RealSense depth camera opened (%dx%d @ %d fps)", self._width, self._height, self._fps)
        except ImportError:
            _log.info("pyrealsense2 not found – depth camera in simulation mode")
            self._sim_mode = True
        except Exception as exc:  # pragma: no cover
            _log.warning("Could not open RealSense device (%s) – simulation mode", exc)
            self._sim_mode = True
        self._open = True

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:  # pragma: no cover
                pass
            self._pipeline = None
        self._open = False
        _log.info("Depth camera sensor closed")

    def read(self) -> DepthFrame:
        if self._sim_mode:
            return self._sim_read()
        return self._hw_read()

    # ------------------------------------------------------------------
    # Simulation
    # ------------------------------------------------------------------

    def _sim_read(self) -> DepthFrame:
        cx, cy = self._width // 2, self._height // 2
        obs_r = self._height // 6
        depth = []
        for r in range(self._height):
            for c in range(self._width):
                if (r - cy) ** 2 + (c - cx) ** 2 <= obs_r ** 2:
                    depth.append(self._sim_obs_dist)
                else:
                    depth.append(self._sim_bg_dist)
        return DepthFrame(
            width=self._width,
            height=self._height,
            depth=depth,
            min_depth=self._sim_obs_dist,
            max_depth=self._sim_bg_dist,
            timestamp=time.time(),
            valid=True,
        )

    # ------------------------------------------------------------------
    # Hardware
    # ------------------------------------------------------------------

    def _hw_read(self) -> DepthFrame:  # pragma: no cover
        try:
            import pyrealsense2 as rs  # type: ignore

            frames = self._pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                return DepthFrame(valid=False)
            w, h = depth_frame.width, depth_frame.height
            depth_data = [
                depth_frame.get_distance(c, r)
                for r in range(h) for c in range(w)
            ]
            valid_depths = [d for d in depth_data if d > 0]
            return DepthFrame(
                width=w, height=h,
                depth=depth_data,
                min_depth=min(valid_depths) if valid_depths else 0.0,
                max_depth=max(valid_depths) if valid_depths else 0.0,
                timestamp=time.time(),
                valid=True,
            )
        except Exception as exc:
            _log.warning("Depth camera read error: %s", exc)
            return DepthFrame(valid=False)
