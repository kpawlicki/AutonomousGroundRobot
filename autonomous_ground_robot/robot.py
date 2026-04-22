"""
Robot – top-level orchestrator.

Ties together navigation, communication and motor control.
"""

import threading
import time
from typing import Optional

from .config import load_config
from .navigation import NavigationManager, NavigationState
from .communication import GroundStationProtocol, LteComm, LoraComm, RadioComm
from .control import MotorController, DriveCommand
from .utils.logger import get_logger

_log = get_logger("robot")


class Robot:
    """
    Top-level autonomous ground robot controller.

    Usage example::

        from autonomous_ground_robot import Robot

        robot = Robot("config/my_config.yaml")
        robot.start()
        robot.navigation.set_goal_gps(48.860, 2.355)

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            robot.stop()

    Parameters
    ----------
    config_path : str, optional
        Path to a YAML configuration file.  Defaults to the bundled
        ``default_config.yaml``.
    """

    def __init__(self, config_path: Optional[str] = None) -> None:
        self.config = load_config(config_path)
        _log.info("Configuration loaded")

        # Navigation
        self.navigation = NavigationManager(self.config)

        # Motor controller
        self.motor = MotorController(self.config.get("control"))

        # Communication (optional)
        self.comms: Optional[GroundStationProtocol] = None
        comm_cfg = self.config.get("communication", {})
        if comm_cfg.get("enabled", False):
            self.comms = _build_protocol(comm_cfg, self._handle_command)

        # Control loop thread
        self._running = False
        self._ctrl_thread: Optional[threading.Thread] = None
        self._ctrl_interval = float(
            self.config.get("navigation", {}).get("update_interval", 0.1)
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start navigation, motor controller, and (optionally) comms."""
        self.motor.open()
        self.navigation.start()
        if self.comms:
            self.comms.start()
        self._running = True
        self._ctrl_thread = threading.Thread(
            target=self._control_loop,
            daemon=True,
            name="robot-ctrl",
        )
        self._ctrl_thread.start()
        _log.info("Robot started")

    def stop(self) -> None:
        """Gracefully shut down all subsystems."""
        self._running = False
        if self._ctrl_thread:
            self._ctrl_thread.join(timeout=3.0)
        self.motor.stop()
        self.motor.close()
        self.navigation.stop()
        if self.comms:
            self.comms.stop()
        _log.info("Robot stopped")

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        while self._running:
            try:
                cmd = self.navigation.get_command()
                drive = DriveCommand(
                    heading=cmd["heading"],
                    speed=cmd["speed"],
                )
                self.motor.execute(drive, self.navigation.heading)

                # Push telemetry to ground station
                if self.comms:
                    pos = self.navigation.position
                    self.comms.update_telemetry({
                        "x": round(pos.x, 3),
                        "y": round(pos.y, 3),
                        "heading": round(self.navigation.heading, 1),
                        "state": self.navigation.state.name,
                        "cmd_speed": round(drive.speed, 2),
                    })
            except Exception as exc:
                _log.error("Control loop error: %s", exc, exc_info=True)
            time.sleep(self._ctrl_interval)

    # ------------------------------------------------------------------
    # Command handler (from ground station)
    # ------------------------------------------------------------------

    def _handle_command(self, payload: dict) -> None:
        """Dispatch a command received from the ground station."""
        cmd_type = payload.get("type", "").lower()
        _log.info("Received command: %s", payload)

        if cmd_type == "goto_gps":
            self.navigation.set_goal_gps(
                float(payload["latitude"]),
                float(payload["longitude"]),
                float(payload.get("altitude", 0.0)),
            )
        elif cmd_type == "goto_local":
            self.navigation.set_goal_local(
                float(payload["x"]),
                float(payload["y"]),
            )
        elif cmd_type == "stop":
            self.navigation.cancel_navigation()
        elif cmd_type == "set_algorithm":
            _log.info("Algorithm change to '%s' requires restart", payload.get("algorithm"))
        else:
            _log.warning("Unknown command type: %s", cmd_type)

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "Robot":
        self.start()
        return self

    def __exit__(self, *_) -> None:
        self.stop()


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _build_protocol(cfg: dict, on_command) -> GroundStationProtocol:
    carrier_name = cfg.get("carrier", "radio").lower()
    if carrier_name == "lte":
        carrier = LteComm(cfg.get("lte"))
    elif carrier_name == "lora":
        carrier = LoraComm(cfg.get("lora"))
    else:
        carrier = RadioComm(cfg.get("radio"))
    return GroundStationProtocol(
        carrier=carrier,
        telemetry_interval=float(cfg.get("telemetry_interval", 1.0)),
        heartbeat_interval=float(cfg.get("heartbeat_interval", 5.0)),
        on_command=on_command,
    )
