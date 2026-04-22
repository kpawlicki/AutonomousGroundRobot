"""Sensor interfaces for the autonomous ground robot."""

from .base import SensorReading, BaseSensor
from .gps import GPSReading, GPSSensor
from .lidar import LidarReading, LidarSensor
from .depth_camera import DepthFrame, DepthCameraSensor
from .ultrasonic import UltrasonicReading, UltrasonicSensor

__all__ = [
    "SensorReading",
    "BaseSensor",
    "GPSReading",
    "GPSSensor",
    "LidarReading",
    "LidarSensor",
    "DepthFrame",
    "DepthCameraSensor",
    "UltrasonicReading",
    "UltrasonicSensor",
]
