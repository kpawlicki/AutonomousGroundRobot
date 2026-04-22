"""
Differential-drive motor controller.

Converts high-level navigation commands (desired heading + speed) into
individual left/right motor PWM values suitable for an H-bridge or ESC.

Hardware abstraction
--------------------
When ``pwm_left_pin`` and ``pwm_right_pin`` are present in the config the
controller drives GPIO PWM directly (Raspberry Pi / similar).
Otherwise it logs commands (simulation / development mode).

Steering model
--------------
A proportional heading controller computes a turn rate from the heading
error and distributes it between left and right motor speeds:

    left_speed  = base_speed - turn_rate
    right_speed = base_speed + turn_rate

Speeds are clamped to [−max_speed, +max_speed] and linearly mapped to
PWM duty cycles in [0, 100].
"""

import math
from dataclasses import dataclass
from typing import Optional

from ..utils.logger import get_logger

_log = get_logger("control.motor")


@dataclass
class DriveCommand:
    """High-level drive command produced by the navigation manager."""

    heading: float = 0.0   # desired heading, degrees (0 = North, clockwise)
    speed: float = 0.0     # desired forward speed, m/s (negative = reverse)


class MotorController:
    """
    Differential-drive motor controller.

    Parameters
    ----------
    config : dict
        ``pwm_left_pin``, ``pwm_right_pin`` – GPIO BCM pins (optional).
        ``max_speed`` – maximum forward speed m/s (default 2.0).
        ``wheel_base`` – distance between wheels in metres (default 0.3).
        ``heading_kp`` – proportional gain for heading controller (default 1.2).
        ``pwm_frequency`` – GPIO PWM frequency in Hz (default 50).
    """

    def __init__(self, config: Optional[dict] = None) -> None:
        self.config = config or {}
        self._sim_mode = (
            "pwm_left_pin" not in self.config or "pwm_right_pin" not in self.config
        )
        self._max_speed = float(self.config.get("max_speed", 2.0))
        self._wheel_base = float(self.config.get("wheel_base", 0.3))
        self._kp = float(self.config.get("heading_kp", 1.2))
        self._pwm_freq = int(self.config.get("pwm_frequency", 50))
        self._current_heading = 0.0  # updated externally

        self._gpio = None
        self._pwm_left = None
        self._pwm_right = None

        # Last output (for telemetry / testing)
        self.last_left_duty = 0.0
        self.last_right_duty = 0.0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        if self._sim_mode:
            _log.info("Motor controller in simulation mode")
            return
        try:
            import RPi.GPIO as GPIO  # type: ignore

            GPIO.setmode(GPIO.BCM)
            lpin = int(self.config["pwm_left_pin"])
            rpin = int(self.config["pwm_right_pin"])
            GPIO.setup(lpin, GPIO.OUT)
            GPIO.setup(rpin, GPIO.OUT)
            self._pwm_left = GPIO.PWM(lpin, self._pwm_freq)
            self._pwm_right = GPIO.PWM(rpin, self._pwm_freq)
            self._pwm_left.start(0)
            self._pwm_right.start(0)
            self._gpio = GPIO
            _log.info("Motor PWM started on pins %d / %d", lpin, rpin)
        except Exception as exc:  # pragma: no cover
            _log.error("Failed to initialise motor GPIO: %s", exc)
            raise

    def close(self) -> None:
        self.stop()
        if self._pwm_left:
            self._pwm_left.stop()
        if self._pwm_right:
            self._pwm_right.stop()
        if self._gpio:
            try:
                self._gpio.cleanup(
                    [int(self.config["pwm_left_pin"]), int(self.config["pwm_right_pin"])]
                )
            except Exception:  # pragma: no cover
                pass
        _log.info("Motor controller closed")

    # ------------------------------------------------------------------
    # Command execution
    # ------------------------------------------------------------------

    def execute(self, command: DriveCommand, current_heading: float) -> None:
        """
        Apply a drive command given the robot's *current_heading* (degrees).
        """
        self._current_heading = current_heading

        if command.speed == 0.0:
            self.stop()
            return

        heading_error = _angle_diff(command.heading, current_heading)
        turn_rate = self._kp * heading_error / 180.0  # normalised [-1, 1]

        left_v = command.speed * (1.0 - turn_rate)
        right_v = command.speed * (1.0 + turn_rate)

        left_v = max(-self._max_speed, min(self._max_speed, left_v))
        right_v = max(-self._max_speed, min(self._max_speed, right_v))

        left_duty = _speed_to_duty(left_v, self._max_speed)
        right_duty = _speed_to_duty(right_v, self._max_speed)

        self._set_motors(left_duty, right_duty)

    def stop(self) -> None:
        """Immediately stop both motors."""
        self._set_motors(0.0, 0.0)

    def _set_motors(self, left_duty: float, right_duty: float) -> None:
        self.last_left_duty = left_duty
        self.last_right_duty = right_duty
        if self._sim_mode:
            _log.debug("Motors – left=%.1f%% right=%.1f%%", left_duty, right_duty)
            return
        self._pwm_left.ChangeDutyCycle(abs(left_duty))   # pragma: no cover
        self._pwm_right.ChangeDutyCycle(abs(right_duty)) # pragma: no cover


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _angle_diff(target: float, current: float) -> float:
    """Return the signed angular difference in degrees (−180 … +180)."""
    diff = (target - current + 180) % 360 - 180
    return diff


def _speed_to_duty(speed: float, max_speed: float) -> float:
    """Map speed (m/s, possibly negative) to PWM duty cycle 0–100 %."""
    if max_speed <= 0:
        return 0.0
    return max(0.0, min(100.0, abs(speed) / max_speed * 100.0))
