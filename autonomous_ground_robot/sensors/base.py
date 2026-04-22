"""Abstract base classes for all robot sensors."""

import abc
import time
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class SensorReading:
    """Base class for all sensor readings."""

    timestamp: float = field(default_factory=time.time)
    valid: bool = True


class BaseSensor(abc.ABC):
    """
    Abstract sensor interface.

    All hardware-specific sensors must subclass this and implement
    :meth:`open`, :meth:`close`, and :meth:`read`.
    """

    def __init__(self, name: str, config: Optional[dict] = None) -> None:
        self.name = name
        self.config = config or {}
        self._open = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    @abc.abstractmethod
    def open(self) -> None:
        """Open/initialise the sensor hardware."""

    @abc.abstractmethod
    def close(self) -> None:
        """Release the sensor hardware."""

    # ------------------------------------------------------------------
    # Data acquisition
    # ------------------------------------------------------------------

    @abc.abstractmethod
    def read(self) -> SensorReading:
        """
        Acquire one reading from the sensor.

        Returns a :class:`SensorReading` (or subclass) with *valid=False*
        when data cannot be obtained.
        """

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "BaseSensor":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    def __repr__(self) -> str:
        state = "open" if self._open else "closed"
        return f"<{self.__class__.__name__} name={self.name!r} [{state}]>"
