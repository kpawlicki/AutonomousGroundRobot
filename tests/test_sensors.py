"""Tests for sensor interfaces (simulation mode only)."""

import math
import time
import pytest

from autonomous_ground_robot.sensors import (
    GPSSensor,
    LidarSensor,
    DepthCameraSensor,
    UltrasonicSensor,
    GPSReading,
    LidarReading,
    DepthFrame,
    UltrasonicReading,
)


# ---------------------------------------------------------------------------
# GPS
# ---------------------------------------------------------------------------

class TestGPSSensor:
    def test_open_close_sim(self):
        s = GPSSensor()
        s.open()
        assert s._open
        s.close()
        assert not s._open

    def test_read_returns_gps_reading(self):
        s = GPSSensor({"sim_latitude": 10.0, "sim_longitude": 20.0})
        with s:
            r = s.read()
        assert isinstance(r, GPSReading)
        assert r.valid
        assert r.fix_quality == 1
        assert r.satellites == 8

    def test_simulated_position_increments(self):
        s = GPSSensor({"sim_latitude": 0.0, "sim_longitude": 0.0})
        with s:
            r1 = s.read()
            r2 = s.read()
        assert r2.latitude > r1.latitude

    def test_context_manager(self):
        with GPSSensor() as s:
            assert s._open

    def test_read_without_open_still_works(self):
        # In simulation mode open() is implicit when we bypass it —
        # but we do need to call open() explicitly.
        s = GPSSensor()
        s.open()
        r = s.read()
        assert isinstance(r, GPSReading)
        s.close()

    def test_nmea_gga_parser(self):
        from autonomous_ground_robot.sensors.gps import _parse_gga
        # $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
        sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
        r = _parse_gga(sentence)
        assert r.valid
        assert abs(r.latitude - 48.117300) < 0.001
        assert abs(r.longitude - 11.516667) < 0.001
        assert r.fix_quality == 1
        assert r.satellites == 8

    def test_nmea_gga_parser_invalid(self):
        from autonomous_ground_robot.sensors.gps import _parse_gga
        r = _parse_gga("$GPGGA,bad,data")
        assert not r.valid

    def test_nmea_rmc_parser(self):
        from autonomous_ground_robot.sensors.gps import _parse_rmc
        sentence = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A"
        r = _parse_rmc(sentence)
        assert r.valid
        assert abs(r.heading - 84.4) < 0.1
        assert r.speed > 0


# ---------------------------------------------------------------------------
# LiDAR
# ---------------------------------------------------------------------------

class TestLidarSensor:
    def test_open_close_sim(self):
        s = LidarSensor()
        s.open()
        assert s._open
        s.close()
        assert not s._open

    def test_read_returns_lidar_reading(self):
        s = LidarSensor({"num_points": 36, "max_range": 10.0, "obstacle_distance": 3.0})
        with s:
            r = s.read()
        assert isinstance(r, LidarReading)
        assert r.valid
        assert r.num_points == 36
        assert len(r.angles) == 36
        assert len(r.distances) == 36

    def test_obstacle_in_front_sector(self):
        s = LidarSensor({
            "num_points": 360,
            "max_range": 10.0,
            "obstacle_distance": 2.0,
            "obstacle_width_deg": 30.0,
        })
        with s:
            r = s.read()
        front_min = r.get_sector_min(-15, 15)
        assert front_min == pytest.approx(2.0, abs=0.01)

    def test_to_cartesian(self):
        s = LidarSensor({"num_points": 4})
        with s:
            r = s.read()
        pts = r.to_cartesian()
        assert len(pts) == 4
        # Each point is (x, y) float pair
        for pt in pts:
            assert len(pt) == 2


# ---------------------------------------------------------------------------
# Depth Camera
# ---------------------------------------------------------------------------

class TestDepthCameraSensor:
    def test_open_close_sim(self):
        s = DepthCameraSensor()
        s.open()
        assert s._open
        assert s._sim_mode  # pyrealsense2 not available in CI
        s.close()
        assert not s._open

    def test_read_returns_depth_frame(self):
        s = DepthCameraSensor({"width": 64, "height": 48, "sim_obstacle_distance": 1.0})
        with s:
            f = s.read()
        assert isinstance(f, DepthFrame)
        assert f.valid
        assert f.width == 64
        assert f.height == 48
        assert len(f.depth) == 64 * 48

    def test_center_obstacle(self):
        s = DepthCameraSensor({
            "width": 64, "height": 48,
            "sim_obstacle_distance": 1.0,
            "sim_background_distance": 5.0,
        })
        with s:
            f = s.read()
        # Centre pixel should be the obstacle distance
        assert f.center_distance() == pytest.approx(1.0)

    def test_get_pixel_bounds(self):
        s = DepthCameraSensor({"width": 10, "height": 10})
        with s:
            f = s.read()
        assert f.get_pixel(-1, -1) == 0.0  # out of bounds
        assert f.get_pixel(100, 100) == 0.0

    def test_min_in_roi(self):
        s = DepthCameraSensor({
            "width": 64, "height": 48,
            "sim_obstacle_distance": 1.0,
            "sim_background_distance": 5.0,
        })
        with s:
            f = s.read()
        # ROI centred on the obstacle should have min ~1.0
        roi_min = f.min_in_roi(20, 28, 28, 36)
        assert roi_min == pytest.approx(1.0)


# ---------------------------------------------------------------------------
# Ultrasonic
# ---------------------------------------------------------------------------

class TestUltrasonicSensor:
    def test_open_close_sim(self):
        s = UltrasonicSensor({"direction": "front", "sim_distance": 2.5})
        s.open()
        assert s._open
        s.close()
        assert not s._open

    def test_read_returns_ultrasonic_reading(self):
        s = UltrasonicSensor({"direction": "front", "sim_distance": 1.5})
        with s:
            r = s.read()
        assert isinstance(r, UltrasonicReading)
        assert r.valid
        assert r.distance == pytest.approx(1.5)
        assert r.direction == "front"

    def test_direction_label(self):
        for direction in ("front", "rear", "left", "right"):
            s = UltrasonicSensor({"direction": direction})
            with s:
                r = s.read()
            assert r.direction == direction

    def test_context_manager(self):
        with UltrasonicSensor() as s:
            assert s._open
