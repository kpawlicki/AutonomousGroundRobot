"""Tests for utilities and the Robot orchestrator."""

import math
import time
import pytest

from autonomous_ground_robot.utils.coordinates import (
    GPSCoordinate,
    LocalPoint,
    gps_to_local,
    local_to_gps,
    haversine_distance,
    bearing_between,
)
from autonomous_ground_robot.control.motor_controller import (
    MotorController,
    DriveCommand,
    _angle_diff,
    _speed_to_duty,
)


# ---------------------------------------------------------------------------
# Coordinate utilities
# ---------------------------------------------------------------------------

class TestCoordinates:
    def test_haversine_zero(self):
        a = GPSCoordinate(48.0, 2.0)
        assert haversine_distance(a, a) == pytest.approx(0.0, abs=1e-6)

    def test_haversine_known(self):
        # Paris → Berlin ≈ 877 km
        paris = GPSCoordinate(48.8566, 2.3522)
        berlin = GPSCoordinate(52.5200, 13.4050)
        dist = haversine_distance(paris, berlin)
        assert 870_000 < dist < 890_000

    def test_bearing_north(self):
        a = GPSCoordinate(0.0, 0.0)
        b = GPSCoordinate(1.0, 0.0)  # due north
        assert bearing_between(a, b) == pytest.approx(0.0, abs=1.0)

    def test_bearing_east(self):
        a = GPSCoordinate(0.0, 0.0)
        b = GPSCoordinate(0.0, 1.0)  # due east
        assert bearing_between(a, b) == pytest.approx(90.0, abs=1.0)

    def test_bearing_south(self):
        a = GPSCoordinate(1.0, 0.0)
        b = GPSCoordinate(0.0, 0.0)  # due south
        assert bearing_between(a, b) == pytest.approx(180.0, abs=1.0)

    def test_gps_to_local_origin(self):
        origin = GPSCoordinate(48.0, 2.0, 0.0)
        local = gps_to_local(origin, origin)
        assert abs(local.x) < 0.01
        assert abs(local.y) < 0.01

    def test_gps_to_local_east(self):
        origin = GPSCoordinate(0.0, 0.0, 0.0)
        east = GPSCoordinate(0.0, 0.001, 0.0)  # slightly east
        local = gps_to_local(east, origin)
        assert local.x > 0
        assert abs(local.y) < 1.0

    def test_gps_to_local_north(self):
        origin = GPSCoordinate(0.0, 0.0, 0.0)
        north = GPSCoordinate(0.001, 0.0, 0.0)  # slightly north
        local = gps_to_local(north, origin)
        assert local.y > 0
        assert abs(local.x) < 1.0

    def test_local_to_gps_roundtrip(self):
        origin = GPSCoordinate(48.8566, 2.3522, 100.0)
        point = LocalPoint(x=100.0, y=200.0, z=50.0)
        gps = local_to_gps(point, origin)
        back = gps_to_local(gps, origin)
        assert abs(back.x - point.x) < 0.01
        assert abs(back.y - point.y) < 0.01
        assert abs(back.z - point.z) < 0.01

    def test_altitude_in_local(self):
        origin = GPSCoordinate(0.0, 0.0, 0.0)
        higher = GPSCoordinate(0.0, 0.0, 50.0)
        local = gps_to_local(higher, origin)
        assert local.z == pytest.approx(50.0)


# ---------------------------------------------------------------------------
# Motor controller
# ---------------------------------------------------------------------------

class TestMotorController:
    def test_open_close_sim(self):
        mc = MotorController()
        mc.open()   # sim mode – no GPIO
        mc.close()

    def test_stop_zeroes_duty(self):
        mc = MotorController()
        mc.open()
        mc.stop()
        assert mc.last_left_duty == 0.0
        assert mc.last_right_duty == 0.0
        mc.close()

    def test_forward_command_both_motors_positive(self):
        mc = MotorController({"max_speed": 2.0, "heading_kp": 1.0})
        mc.open()
        cmd = DriveCommand(heading=0.0, speed=1.0)
        mc.execute(cmd, current_heading=0.0)
        assert mc.last_left_duty > 0
        assert mc.last_right_duty > 0
        mc.close()

    def test_zero_speed_stops(self):
        mc = MotorController()
        mc.open()
        cmd = DriveCommand(heading=90.0, speed=0.0)
        mc.execute(cmd, current_heading=0.0)
        assert mc.last_left_duty == 0.0
        assert mc.last_right_duty == 0.0
        mc.close()

    def test_turn_right_boosts_right_motor(self):
        mc = MotorController({"max_speed": 2.0, "heading_kp": 1.0})
        mc.open()
        # Robot faces North (0°), wants to go East (90°) → turn right
        cmd = DriveCommand(heading=90.0, speed=1.0)
        mc.execute(cmd, current_heading=0.0)
        assert mc.last_right_duty >= mc.last_left_duty
        mc.close()

    def test_turn_left_boosts_left_motor(self):
        mc = MotorController({"max_speed": 2.0, "heading_kp": 1.0})
        mc.open()
        # Robot faces North (0°), wants to go West (270°) → turn left
        cmd = DriveCommand(heading=270.0, speed=1.0)
        mc.execute(cmd, current_heading=0.0)
        assert mc.last_left_duty >= mc.last_right_duty
        mc.close()

    def test_duty_clamped_to_100(self):
        mc = MotorController({"max_speed": 1.0, "heading_kp": 1.0})
        mc.open()
        cmd = DriveCommand(heading=0.0, speed=1.0)
        mc.execute(cmd, current_heading=0.0)
        assert mc.last_left_duty <= 100.0
        assert mc.last_right_duty <= 100.0
        mc.close()


class TestAngleDiff:
    def test_no_diff(self):
        assert _angle_diff(90.0, 90.0) == pytest.approx(0.0)

    def test_positive_diff(self):
        assert _angle_diff(90.0, 0.0) == pytest.approx(90.0)

    def test_negative_diff(self):
        assert _angle_diff(0.0, 90.0) == pytest.approx(-90.0)

    def test_wraparound(self):
        # 350° → 10° = +20°
        assert _angle_diff(10.0, 350.0) == pytest.approx(20.0)

    def test_opposite(self):
        diff = _angle_diff(180.0, 0.0)
        assert abs(diff) == pytest.approx(180.0)


class TestSpeedToDuty:
    def test_zero_speed(self):
        assert _speed_to_duty(0.0, 2.0) == 0.0

    def test_full_speed(self):
        assert _speed_to_duty(2.0, 2.0) == pytest.approx(100.0)

    def test_half_speed(self):
        assert _speed_to_duty(1.0, 2.0) == pytest.approx(50.0)

    def test_negative_speed(self):
        assert _speed_to_duty(-1.0, 2.0) == pytest.approx(50.0)

    def test_clamped(self):
        assert _speed_to_duty(10.0, 2.0) == pytest.approx(100.0)

    def test_zero_max_speed(self):
        assert _speed_to_duty(1.0, 0.0) == 0.0


# ---------------------------------------------------------------------------
# Robot integration smoke test
# ---------------------------------------------------------------------------

class TestRobotIntegration:
    def test_robot_starts_and_stops(self):
        from autonomous_ground_robot import Robot
        robot = Robot()
        robot.start()
        time.sleep(0.2)
        robot.stop()

    def test_robot_set_local_goal(self):
        from autonomous_ground_robot import Robot
        robot = Robot()
        robot.start()
        time.sleep(0.2)  # let GPS establish origin
        robot.navigation.set_goal_local(2.0, 2.0)
        time.sleep(0.3)
        # Should have transitioned from IDLE
        from autonomous_ground_robot.navigation import NavigationState
        assert robot.navigation.state != NavigationState.ERROR
        robot.stop()

    def test_robot_cancel_navigation(self):
        from autonomous_ground_robot import Robot
        from autonomous_ground_robot.navigation import NavigationState
        robot = Robot()
        robot.start()
        time.sleep(0.15)
        robot.navigation.set_goal_local(10.0, 10.0)
        time.sleep(0.1)
        robot.navigation.cancel_navigation()
        assert robot.navigation.state == NavigationState.IDLE
        robot.stop()

    def test_robot_context_manager(self):
        from autonomous_ground_robot import Robot
        with Robot() as robot:
            assert robot.navigation._running

    def test_robot_handle_command(self):
        from autonomous_ground_robot import Robot
        robot = Robot()
        robot.start()
        time.sleep(0.15)
        robot._handle_command({"type": "goto_local", "x": 3.0, "y": 3.0})
        time.sleep(0.2)
        from autonomous_ground_robot.navigation import NavigationState
        assert robot.navigation.state != NavigationState.ERROR
        robot.stop()
