"""
Autonomous Ground Robot
=======================

A modular, carrier-agnostic software stack for autonomous ground navigation.

Sub-packages
------------
sensors         – GPS, LiDAR, depth camera, ultrasonic sensor interfaces
navigation      – occupancy grid, path planning, navigation manager
communication   – LTE / LoRa / Radio carriers + ground station protocol
control         – differential-drive motor controller
utils           – coordinate helpers and logging
config          – YAML configuration loader

Quick start
-----------
>>> from autonomous_ground_robot import Robot
>>> robot = Robot()           # uses default_config.yaml
>>> robot.start()
>>> robot.navigation.set_goal_local(x=5.0, y=5.0)
>>> robot.stop()
"""

from .robot import Robot
from .config import load_config

__version__ = "0.1.0"

__all__ = ["Robot", "load_config"]
