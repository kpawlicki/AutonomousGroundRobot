"""
Radio (serial) communication carrier.

Targets radio modems that expose a transparent serial interface (e.g. SiK
telemetry radios, XBee modules, RFD900x).  The carrier simply reads/writes
raw bytes; framing and addressing are handled at the protocol layer.

Simulation mode buffers sent bytes locally (useful for unit testing).
"""

import queue
import time
from typing import Callable, Optional

from .base import BaseComm, CommStatus
from ..utils.logger import get_logger

_log = get_logger("comm.radio")


class RadioComm(BaseComm):
    """
    Serial radio modem communication carrier.

    Config keys
    -----------
    port : str
        Serial port (e.g. ``"/dev/ttyUSB1"``).  Omit for simulation mode.
    baud : int
        Baud rate (default ``57600`` – SiK radio default).
    timeout : float
        Read timeout in seconds (default ``1.0``).
    """

    def __init__(
        self,
        config: Optional[dict] = None,
        on_receive: Optional[Callable[[bytes], None]] = None,
    ) -> None:
        super().__init__("radio", config, on_receive)
        self._sim_mode = "port" not in self.config
        self._serial = None
        # Separate queues for outbound (sent by robot) and inbound (received from remote)
        self._out_queue: "queue.Queue[bytes]" = queue.Queue()
        self._in_queue: "queue.Queue[bytes]" = queue.Queue()
        self._baud = int(self.config.get("baud", 57600))
        self._timeout = float(self.config.get("timeout", 1.0))

    def connect(self) -> bool:
        if self._sim_mode:
            self.status = CommStatus.CONNECTED
            _log.info("Radio carrier in simulation mode")
            return True
        try:
            import serial  # type: ignore

            self._serial = serial.Serial(
                port=self.config["port"],
                baudrate=self._baud,
                timeout=self._timeout,
            )
            self.status = CommStatus.CONNECTED
            _log.info("Radio connected on %s @ %d baud", self.config["port"], self._baud)
            return True
        except Exception as exc:
            _log.warning("Radio connect failed: %s", exc)
            self.status = CommStatus.ERROR
            return False

    def disconnect(self) -> None:
        self.stop_receive_loop()
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self.status = CommStatus.DISCONNECTED
        _log.info("Radio disconnected")

    def send_bytes(self, data: bytes) -> bool:
        if self.status != CommStatus.CONNECTED:
            return False
        if self._sim_mode:
            self._out_queue.put(data)
            return True
        try:
            self._serial.write(data)
            return True
        except Exception as exc:
            _log.warning("Radio send error: %s", exc)
            self.status = CommStatus.ERROR
            return False

    def receive_bytes(self, timeout: float = 1.0) -> Optional[bytes]:
        if self.status != CommStatus.CONNECTED:
            return None
        if self._sim_mode:
            try:
                return self._in_queue.get(timeout=timeout)
            except queue.Empty:
                return None
        try:
            data = self._serial.read(self._serial.in_waiting or 1)
            return data if data else None
        except Exception as exc:
            _log.warning("Radio receive error: %s", exc)
            return None

    def inject_receive(self, data: bytes) -> None:
        """
        Inject bytes into the simulation inbound queue.

        Useful for unit tests to simulate inbound radio messages.
        """
        self._in_queue.put(data)

    def get_sent(self, timeout: float = 0.1) -> Optional[bytes]:
        """
        Retrieve the next bytes that the robot sent (outbound queue).

        Useful for unit tests to verify outbound messages.
        """
        try:
            return self._out_queue.get(timeout=timeout)
        except queue.Empty:
            return None
