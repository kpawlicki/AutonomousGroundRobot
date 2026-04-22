"""
GPS sensor interface.

Reads NMEA sentences from a serial GPS receiver (e.g. u-blox NEO series).
Falls back to a *simulated* mode when no serial port is configured, which is
useful for unit-testing and software-in-the-loop development.
"""

import math
import time
from dataclasses import dataclass, field
from typing import Optional

from .base import BaseSensor, SensorReading
from ..utils.logger import get_logger

_log = get_logger("sensors.gps")


@dataclass
class GPSReading(SensorReading):
    """One fix from the GPS receiver."""

    latitude: float = 0.0     # degrees, WGS-84
    longitude: float = 0.0    # degrees, WGS-84
    altitude: float = 0.0     # metres above ellipsoid
    heading: float = 0.0      # degrees, 0 = North, clockwise
    speed: float = 0.0        # m/s
    fix_quality: int = 0       # 0=no fix, 1=GPS, 2=DGPS, 4=RTK
    satellites: int = 0


class GPSSensor(BaseSensor):
    """
    GPS receiver interface.

    Config keys
    -----------
    port : str
        Serial port path (e.g. ``/dev/ttyUSB0``).  When omitted the sensor
        runs in *simulation* mode and returns a static or incrementing fix.
    baud : int
        Baud rate (default ``9600``).
    timeout : float
        Read timeout in seconds (default ``1.0``).
    sim_latitude : float
        Starting latitude for simulation mode (default ``0.0``).
    sim_longitude : float
        Starting longitude for simulation mode (default ``0.0``).
    """

    def __init__(self, config: Optional[dict] = None) -> None:
        super().__init__("gps", config)
        self._serial = None
        self._sim_mode = "port" not in self.config
        self._sim_lat = float(self.config.get("sim_latitude", 0.0))
        self._sim_lon = float(self.config.get("sim_longitude", 0.0))
        self._sim_tick = 0

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def open(self) -> None:
        if self._sim_mode:
            _log.info("GPS sensor opened in simulation mode")
            self._open = True
            return
        try:
            import serial  # type: ignore

            self._serial = serial.Serial(
                port=self.config["port"],
                baudrate=int(self.config.get("baud", 9600)),
                timeout=float(self.config.get("timeout", 1.0)),
            )
            _log.info("GPS serial port %s opened", self.config["port"])
            self._open = True
        except Exception as exc:  # pragma: no cover
            _log.error("Failed to open GPS serial port: %s", exc)
            raise

    def close(self) -> None:
        if self._serial is not None:
            self._serial.close()
            self._serial = None
        self._open = False
        _log.info("GPS sensor closed")

    # ------------------------------------------------------------------
    # Data acquisition
    # ------------------------------------------------------------------

    def read(self) -> GPSReading:
        if self._sim_mode:
            return self._sim_read()
        return self._serial_read()

    def _sim_read(self) -> GPSReading:
        self._sim_tick += 1
        return GPSReading(
            latitude=self._sim_lat + self._sim_tick * 1e-5,
            longitude=self._sim_lon + self._sim_tick * 1e-5,
            altitude=10.0,
            heading=float((self._sim_tick * 5) % 360),
            speed=1.0,
            fix_quality=1,
            satellites=8,
            timestamp=time.time(),
            valid=True,
        )

    def _serial_read(self) -> GPSReading:  # pragma: no cover
        """Read and parse NMEA sentences from the serial port."""
        reading = GPSReading(valid=False)
        try:
            line = self._serial.readline().decode("ascii", errors="replace").strip()
            if line.startswith("$GNGGA") or line.startswith("$GPGGA"):
                reading = _parse_gga(line)
            elif line.startswith("$GNRMC") or line.startswith("$GPRMC"):
                reading = _parse_rmc(line)
        except Exception as exc:
            _log.warning("GPS read error: %s", exc)
        return reading


# ---------------------------------------------------------------------------
# NMEA parsers (minimal, no external library required)
# ---------------------------------------------------------------------------

def _nmea_degrees(raw: str, direction: str) -> float:
    """Convert NMEA ddmm.mmmm format to decimal degrees."""
    if not raw:
        return 0.0
    dot = raw.index(".")
    deg = float(raw[: dot - 2])
    minutes = float(raw[dot - 2:])
    value = deg + minutes / 60.0
    if direction in ("S", "W"):
        value = -value
    return value


def _parse_gga(sentence: str) -> GPSReading:
    """Parse a GGA NMEA sentence."""
    parts = sentence.split(",")
    try:
        lat = _nmea_degrees(parts[2], parts[3])
        lon = _nmea_degrees(parts[4], parts[5])
        quality = int(parts[6]) if parts[6] else 0
        sats = int(parts[7]) if parts[7] else 0
        alt = float(parts[9]) if parts[9] else 0.0
        return GPSReading(
            latitude=lat, longitude=lon, altitude=alt,
            fix_quality=quality, satellites=sats,
            valid=quality > 0,
        )
    except (IndexError, ValueError):
        return GPSReading(valid=False)


def _parse_rmc(sentence: str) -> GPSReading:
    """Parse an RMC NMEA sentence (speed and heading)."""
    parts = sentence.split(",")
    try:
        status = parts[2]  # A=active, V=void
        lat = _nmea_degrees(parts[3], parts[4])
        lon = _nmea_degrees(parts[5], parts[6])
        speed_knots = float(parts[7]) if parts[7] else 0.0
        heading = float(parts[8]) if parts[8] else 0.0
        return GPSReading(
            latitude=lat, longitude=lon,
            speed=speed_knots * 0.5144,  # knots → m/s
            heading=heading,
            valid=(status == "A"),
        )
    except (IndexError, ValueError):
        return GPSReading(valid=False)
