"""
Ultrasonic sensor interface.

Supports HC-SR04 (and compatible) sensors via GPIO on embedded Linux boards
such as Raspberry Pi.  Simulation mode returns a configurable fixed distance.
Multiple sensors can be instantiated for different directions.
"""

import time
from dataclasses import dataclass
from typing import Optional

from .base import BaseSensor, SensorReading
from ..utils.logger import get_logger

_log = get_logger("sensors.ultrasonic")

_SOUND_SPEED_MPS = 343.0  # m/s at ~20 °C
_MAX_RANGE_M = 4.0         # HC-SR04 practical maximum


@dataclass
class UltrasonicReading(SensorReading):
    """Single distance measurement from an ultrasonic sensor."""

    distance: float = float("inf")  # metres; inf = no echo / out of range
    direction: str = "front"         # descriptive label


class UltrasonicSensor(BaseSensor):
    """
    Ultrasonic proximity sensor interface.

    Config keys
    -----------
    direction : str
        Sensor direction label (default ``"front"``).
    trigger_pin : int
        GPIO BCM pin number for the TRIG line.  Omit for simulation mode.
    echo_pin : int
        GPIO BCM pin number for the ECHO line.  Omit for simulation mode.
    max_range : float
        Maximum measurement range in metres (default ``4.0``).
    sim_distance : float
        Fixed distance returned in simulation mode (default ``3.0``).
    """

    def __init__(self, config: Optional[dict] = None) -> None:
        direction = (config or {}).get("direction", "front")
        super().__init__(f"ultrasonic_{direction}", config)
        self._gpio = None
        self._sim_mode = "trigger_pin" not in self.config
        self._direction = direction
        self._trig = self.config.get("trigger_pin")
        self._echo = self.config.get("echo_pin")
        self._max_range = float(self.config.get("max_range", _MAX_RANGE_M))
        self._sim_dist = float(self.config.get("sim_distance", 3.0))

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        if self._sim_mode:
            _log.info("Ultrasonic (%s) opened in simulation mode", self._direction)
            self._open = True
            return
        try:
            import RPi.GPIO as GPIO  # type: ignore

            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self._trig, GPIO.OUT)
            GPIO.setup(self._echo, GPIO.IN)
            GPIO.output(self._trig, False)
            time.sleep(0.05)  # sensor settle time
            self._gpio = GPIO
            _log.info(
                "Ultrasonic (%s) opened – TRIG=%s ECHO=%s",
                self._direction, self._trig, self._echo,
            )
            self._open = True
        except Exception as exc:  # pragma: no cover
            _log.error("Failed to open ultrasonic GPIO: %s", exc)
            raise

    def close(self) -> None:
        if self._gpio is not None:
            try:
                self._gpio.cleanup([self._trig, self._echo])
            except Exception:  # pragma: no cover
                pass
            self._gpio = None
        self._open = False
        _log.info("Ultrasonic (%s) sensor closed", self._direction)

    # ------------------------------------------------------------------
    # Data acquisition
    # ------------------------------------------------------------------

    def read(self) -> UltrasonicReading:
        if self._sim_mode:
            return UltrasonicReading(
                distance=self._sim_dist,
                direction=self._direction,
                timestamp=time.time(),
                valid=True,
            )
        return self._gpio_read()

    def _gpio_read(self) -> UltrasonicReading:  # pragma: no cover
        GPIO = self._gpio
        try:
            # Send 10 µs trigger pulse
            GPIO.output(self._trig, True)
            time.sleep(1e-5)
            GPIO.output(self._trig, False)

            # Wait for echo to go high
            timeout = time.time() + 0.1
            while not GPIO.input(self._echo):
                if time.time() > timeout:
                    return UltrasonicReading(distance=float("inf"), direction=self._direction, valid=False)
            pulse_start = time.time()

            # Wait for echo to go low
            timeout = time.time() + 0.1
            while GPIO.input(self._echo):
                if time.time() > timeout:
                    return UltrasonicReading(distance=float("inf"), direction=self._direction, valid=False)
            pulse_end = time.time()

            duration = pulse_end - pulse_start
            distance = (duration * _SOUND_SPEED_MPS) / 2.0

            if distance > self._max_range:
                distance = float("inf")

            return UltrasonicReading(
                distance=distance,
                direction=self._direction,
                timestamp=time.time(),
                valid=True,
            )
        except Exception as exc:
            _log.warning("Ultrasonic read error: %s", exc)
            return UltrasonicReading(distance=float("inf"), direction=self._direction, valid=False)
