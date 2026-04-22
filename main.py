#!/usr/bin/env python3
"""
Autonomous Ground Robot – main entry point.

Usage
-----
Run with the default (simulation) configuration::

    python main.py

Run with a custom configuration file::

    python main.py --config /path/to/config.yaml

Send the robot to a GPS waypoint (lat/lon, degrees)::

    python main.py --goal-lat 48.860 --goal-lon 2.355

Send the robot to a local ENU waypoint (metres from start)::

    python main.py --goal-x 10 --goal-y 5

Choose a navigation algorithm::

    python main.py --algorithm dijkstra
    python main.py --algorithm potential_fields
"""

import argparse
import signal
import sys
import time

from autonomous_ground_robot import Robot
from autonomous_ground_robot.utils.logger import get_logger

_log = get_logger("main")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Autonomous Ground Robot",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--config", "-c",
        metavar="PATH",
        default=None,
        help="Path to a YAML configuration file (default: built-in defaults)",
    )
    parser.add_argument(
        "--goal-lat",
        type=float,
        default=None,
        metavar="DEG",
        help="Goal latitude in degrees (WGS-84)",
    )
    parser.add_argument(
        "--goal-lon",
        type=float,
        default=None,
        metavar="DEG",
        help="Goal longitude in degrees (WGS-84)",
    )
    parser.add_argument(
        "--goal-x",
        type=float,
        default=None,
        metavar="M",
        help="Goal X position in local ENU frame (metres, East)",
    )
    parser.add_argument(
        "--goal-y",
        type=float,
        default=None,
        metavar="M",
        help="Goal Y position in local ENU frame (metres, North)",
    )
    parser.add_argument(
        "--algorithm",
        choices=["astar", "dijkstra", "potential_fields"],
        default=None,
        help="Override the navigation algorithm specified in the config",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        metavar="SEC",
        help="Run for this many seconds then exit (default: run until Ctrl-C)",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)

    robot = Robot(args.config)

    # Override algorithm from CLI if requested
    if args.algorithm:
        robot.config.setdefault("navigation", {})["algorithm"] = args.algorithm
        _log.info("Algorithm overridden to: %s", args.algorithm)

    robot.start()
    _log.info("Robot running.  Press Ctrl-C to stop.")

    # Set navigation goal
    if args.goal_lat is not None and args.goal_lon is not None:
        # Wait briefly for GPS to establish an origin fix
        time.sleep(1.0)
        robot.navigation.set_goal_gps(args.goal_lat, args.goal_lon)
        _log.info("GPS goal set: lat=%.6f lon=%.6f", args.goal_lat, args.goal_lon)
    elif args.goal_x is not None and args.goal_y is not None:
        robot.navigation.set_goal_local(args.goal_x, args.goal_y)
        _log.info("Local goal set: x=%.2f y=%.2f", args.goal_x, args.goal_y)

    # Graceful shutdown on SIGTERM / SIGINT
    stop_event = [False]

    def _shutdown(signum, frame):
        _log.info("Shutdown signal received")
        stop_event[0] = True

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    start_time = time.time()
    try:
        while not stop_event[0]:
            elapsed = time.time() - start_time
            if args.duration and elapsed >= args.duration:
                _log.info("Duration %.1fs elapsed – stopping", args.duration)
                break
            state = robot.navigation.state.name
            pos = robot.navigation.position
            _log.info(
                "State=%-20s  pos=(%.2f, %.2f)  heading=%.1f°",
                state, pos.x, pos.y, robot.navigation.heading,
            )
            time.sleep(1.0)
    finally:
        robot.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
