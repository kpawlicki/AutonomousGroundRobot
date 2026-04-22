"""
LoRa communication carrier.

Supports LoRa modules that expose a UART interface (e.g. Dragino LoRa/GPS
HAT, RAK811, Ebyte E32/E22 series).  The module is expected to operate in
transparent/UART mode so that data written to the serial port is broadcast
over the LoRa air interface.

Simulation mode buffers sent bytes locally (useful for unit testing).
"""

import queue
import time
from typing import Callable, Optional

from .base import BaseComm, CommStatus
from ..utils.logger import get_logger

_log = get_logger("comm.lora")


class LoraComm(BaseComm):
    """
    LoRa UART communication carrier.

    Config keys
    -----------
    port : str
        Serial port (e.g. ``"/dev/ttyS0"``).  Omit for simulation mode.
    baud : int
        Baud rate (default ``9600`` – common for E32 modules in air-rate mode).
    timeout : float
        Read timeout in seconds (default ``2.0``).
    air_data_rate : str
        Informational label (default ``"2.4kbps"``).
    """

    def __init__(
        self,
        config: Optional[dict] = None,
        on_receive: Optional[Callable[[bytes], None]] = None,
    ) -> None:
        super().__init__("lora", config, on_receive)
        self._sim_mode = "port" not in self.config
        self._serial = None
        # Separate queues for outbound (sent by robot) and inbound (received from remote)
        self._out_queue: "queue.Queue[bytes]" = queue.Queue()
        self._in_queue: "queue.Queue[bytes]" = queue.Queue()
        self._baud = int(self.config.get("baud", 9600))
        self._timeout = float(self.config.get("timeout", 2.0))

    def connect(self) -> bool:
        if self._sim_mode:
            self.status = CommStatus.CONNECTED
            _log.info("LoRa carrier in simulation mode")
            return True
        try:
            import serial  # type: ignore

            self._serial = serial.Serial(
                port=self.config["port"],
                baudrate=self._baud,
                timeout=self._timeout,
            )
            self.status = CommStatus.CONNECTED
            _log.info("LoRa connected on %s @ %d baud", self.config["port"], self._baud)
            return True
        except Exception as exc:
            _log.warning("LoRa connect failed: %s", exc)
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
        _log.info("LoRa disconnected")

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
            _log.warning("LoRa send error: %s", exc)
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
            _log.warning("LoRa receive error: %s", exc)
            return None

    def inject_receive(self, data: bytes) -> None:
        """
        Inject bytes into the simulation inbound queue.

        Useful for unit tests to simulate inbound LoRa messages.
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
