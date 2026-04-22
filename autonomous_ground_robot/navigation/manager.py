"""
Navigation manager.

Orchestrates sensors, occupancy-grid updates, path planning and waypoint
following.  The manager runs in its own thread and exposes a simple
command interface.
"""

import math
import threading
import time
from enum import Enum, auto
from typing import List, Optional

from .path_planner import OccupancyGrid, PathPlanner
from ..sensors import GPSSensor, LidarSensor, DepthCameraSensor, UltrasonicSensor, GPSReading
from ..utils.coordinates import GPSCoordinate, LocalPoint, gps_to_local
from ..utils.logger import get_logger

_log = get_logger("navigation.manager")


class NavigationState(Enum):
    IDLE = auto()
    PLANNING = auto()
    NAVIGATING = auto()
    OBSTACLE_AVOIDANCE = auto()
    GOAL_REACHED = auto()
    ERROR = auto()


class NavigationManager:
    """
    Central navigation manager.

    Responsibilities
    ----------------
    * Periodically reads all sensors.
    * Updates the shared occupancy grid from sensor data.
    * Re-plans a path when the occupancy grid changes significantly.
    * Tracks progress along the current path and advances waypoints.
    * Exposes the next heading/speed command for the motor controller.

    Parameters
    ----------
    config : dict
        Sub-keys: ``gps``, ``lidar``, ``depth_camera``, ``ultrasonic`` (list),
        ``navigation``.
    """

    # How close we must be to a waypoint (metres) to consider it reached
    WAYPOINT_RADIUS = 0.5

    def __init__(self, config: Optional[dict] = None) -> None:
        cfg = config or {}
        self.config = cfg

        # Sensors
        self.gps = GPSSensor(cfg.get("gps"))
        self.lidar = LidarSensor(cfg.get("lidar"))
        self.depth_camera = DepthCameraSensor(cfg.get("depth_camera"))
        self.ultrasonics: List[UltrasonicSensor] = [
            UltrasonicSensor(uc) for uc in cfg.get("ultrasonic", [{}])
        ]

        # Navigation
        nav_cfg = cfg.get("navigation", {})
        self.grid = OccupancyGrid(
            resolution=float(nav_cfg.get("grid_resolution", 0.1)),
            width=int(nav_cfg.get("grid_width", 201)),
            height=int(nav_cfg.get("grid_height", 201)),
            origin_row=int(nav_cfg.get("grid_height", 201)) // 2,
            origin_col=int(nav_cfg.get("grid_width", 201)) // 2,
        )

        algorithm_name = nav_cfg.get("algorithm", "astar")
        self.planner = PathPlanner(
            algorithm=_build_algorithm(algorithm_name, nav_cfg),
            grid=self.grid,
        )

        # State
        self.state = NavigationState.IDLE
        self._origin: Optional[GPSCoordinate] = None
        self._current_pos = LocalPoint()
        self._current_heading = 0.0  # degrees
        self._current_speed = 0.0    # m/s
        self._goal: Optional[LocalPoint] = None
        self._waypoints: List[LocalPoint] = []
        self._wp_index = 0
        self._last_gps: Optional[GPSReading] = None

        # Commands (for motor controller)
        self._cmd_heading = 0.0
        self._cmd_speed = 0.0
        self._cmd_lock = threading.Lock()

        # Background thread
        self._update_interval = float(nav_cfg.get("update_interval", 0.1))
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open all sensors and start the navigation loop."""
        self.gps.open()
        self.lidar.open()
        self.depth_camera.open()
        for us in self.ultrasonics:
            us.open()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="nav-manager")
        self._thread.start()
        _log.info("Navigation manager started")

    def stop(self) -> None:
        """Stop the navigation loop and close all sensors."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        self.gps.close()
        self.lidar.close()
        self.depth_camera.close()
        for us in self.ultrasonics:
            us.close()
        self.state = NavigationState.IDLE
        _log.info("Navigation manager stopped")

    def set_goal_gps(self, latitude: float, longitude: float, altitude: float = 0.0) -> None:
        """Command the robot to navigate to a GPS coordinate."""
        target = GPSCoordinate(latitude, longitude, altitude)
        if self._origin is None and self._last_gps is not None:
            self._origin = GPSCoordinate(
                self._last_gps.latitude,
                self._last_gps.longitude,
                self._last_gps.altitude,
            )
        if self._origin:
            self._goal = gps_to_local(target, self._origin)
        else:
            # Store as numeric local point; will recalculate once GPS locks
            self._goal = LocalPoint(x=0, y=0)
            _log.warning("GPS origin not yet established; goal stored for later")
        self.state = NavigationState.PLANNING
        _log.info("New GPS goal set")
        _log.debug("GPS goal coordinates: lat=%.6f lon=%.6f", latitude, longitude)

    def set_goal_local(self, x: float, y: float) -> None:
        """Command the robot to navigate to a local ENU position (metres)."""
        self._goal = LocalPoint(x=x, y=y)
        self.state = NavigationState.PLANNING
        _log.info("New local goal set: x=%.2f y=%.2f", x, y)

    def cancel_navigation(self) -> None:
        """Stop navigating and return to IDLE."""
        self._goal = None
        self._waypoints = []
        self._wp_index = 0
        with self._cmd_lock:
            self._cmd_speed = 0.0
        self.state = NavigationState.IDLE

    def get_command(self) -> dict:
        """Return the latest motion command (heading degrees, speed m/s)."""
        with self._cmd_lock:
            return {"heading": self._cmd_heading, "speed": self._cmd_speed}

    @property
    def position(self) -> LocalPoint:
        return self._current_pos

    @property
    def heading(self) -> float:
        return self._current_heading

    # ------------------------------------------------------------------
    # Navigation loop (background thread)
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        while self._running:
            try:
                self._update()
            except Exception as exc:
                _log.error("Navigation loop error: %s", exc, exc_info=True)
                self.state = NavigationState.ERROR
            time.sleep(self._update_interval)

    def _update(self) -> None:
        # 1. Read sensors
        gps_reading = self.gps.read()
        lidar_reading = self.lidar.read()
        depth_reading = self.depth_camera.read()
        us_readings = [us.read() for us in self.ultrasonics]

        # 2. Update position from GPS
        if gps_reading.valid:
            self._last_gps = gps_reading
            if self._origin is None:
                self._origin = GPSCoordinate(
                    gps_reading.latitude, gps_reading.longitude, gps_reading.altitude
                )
                _log.info("GPS origin established")
                _log.debug(
                    "GPS origin coordinates: lat=%.6f lon=%.6f",
                    self._origin.latitude, self._origin.longitude,
                )
            self._current_pos = gps_to_local(
                GPSCoordinate(gps_reading.latitude, gps_reading.longitude, gps_reading.altitude),
                self._origin,
            )
            self._current_heading = gps_reading.heading
            self._current_speed = gps_reading.speed

        # 3. Update occupancy grid
        if lidar_reading.valid:
            self.grid.update_from_lidar(lidar_reading)
        if depth_reading.valid:
            self.grid.update_from_depth(depth_reading)
        if us_readings:
            self.grid.update_from_ultrasonic(us_readings)

        # 4. Check for close obstacles (emergency stop threshold)
        obstacle_close = self._check_immediate_obstacle(lidar_reading, us_readings)

        # 5. State machine
        if self.state == NavigationState.PLANNING:
            self._do_planning()

        elif self.state == NavigationState.NAVIGATING:
            if obstacle_close:
                self.state = NavigationState.OBSTACLE_AVOIDANCE
                with self._cmd_lock:
                    self._cmd_speed = 0.0
                _log.warning("Immediate obstacle detected – stopping")
            else:
                self._follow_waypoints()

        elif self.state == NavigationState.OBSTACLE_AVOIDANCE:
            if not obstacle_close:
                # Resume after obstacle clears
                self.state = NavigationState.PLANNING
            else:
                with self._cmd_lock:
                    self._cmd_speed = 0.0

    def _do_planning(self) -> None:
        if self._goal is None:
            self.state = NavigationState.IDLE
            return
        waypoints = self.planner.plan(self._current_pos, self._goal)
        if waypoints:
            self._waypoints = waypoints
            self._wp_index = 0
            self.state = NavigationState.NAVIGATING
        else:
            _log.warning("Path planner could not find a path to goal")
            self.state = NavigationState.ERROR

    def _follow_waypoints(self) -> None:
        if self._wp_index >= len(self._waypoints):
            self.state = NavigationState.GOAL_REACHED
            with self._cmd_lock:
                self._cmd_speed = 0.0
            _log.info("Goal reached!")
            return

        target = self._waypoints[self._wp_index]
        dx = target.x - self._current_pos.x
        dy = target.y - self._current_pos.y
        distance = math.hypot(dx, dy)

        if distance < self.WAYPOINT_RADIUS:
            self._wp_index += 1
            return

        desired_heading = (math.degrees(math.atan2(dx, dy)) + 360) % 360
        nav_cfg = self.config.get("navigation", {})
        cruise_speed = float(nav_cfg.get("cruise_speed", 1.0))

        with self._cmd_lock:
            self._cmd_heading = desired_heading
            self._cmd_speed = cruise_speed

    def _check_immediate_obstacle(self, lidar, us_readings) -> bool:
        nav_cfg = self.config.get("navigation", {})
        stop_dist = float(nav_cfg.get("emergency_stop_distance", 0.3))

        # Check LiDAR front sector (±30°)
        if lidar.valid:
            front_dist = lidar.get_sector_min(-30, 30)
            if front_dist < stop_dist:
                return True

        # Check ultrasonic sensors
        for r in us_readings:
            if r.valid and r.direction == "front" and r.distance < stop_dist:
                return True

        return False

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "NavigationManager":
        self.start()
        return self

    def __exit__(self, *_) -> None:
        self.stop()


# ---------------------------------------------------------------------------
# Algorithm factory
# ---------------------------------------------------------------------------

def _build_algorithm(name: str, cfg: dict):
    from .algorithms import AStarAlgorithm, DijkstraAlgorithm, PotentialFieldsAlgorithm

    name = name.lower()
    if name == "astar":
        return AStarAlgorithm(
            allow_diagonal=cfg.get("allow_diagonal", True),
            weight=float(cfg.get("astar_weight", 1.0)),
        )
    if name == "dijkstra":
        return DijkstraAlgorithm(allow_diagonal=cfg.get("allow_diagonal", True))
    if name in ("potential_fields", "apf"):
        return PotentialFieldsAlgorithm(
            attractive_gain=float(cfg.get("apf_attractive_gain", 1.0)),
            repulsive_gain=float(cfg.get("apf_repulsive_gain", 100.0)),
            influence_radius=float(cfg.get("apf_influence_radius", 5.0)),
        )
    _log.warning("Unknown algorithm '%s', falling back to A*", name)
    return AStarAlgorithm()
