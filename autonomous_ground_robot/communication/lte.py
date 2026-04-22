"""
LTE communication carrier.

Uses an LTE modem (e.g. Waveshare SIM7600, Quectel EC21) that exposes a
TCP/IP socket via AT commands or a built-in network interface.

Simulation mode opens a plain TCP socket to a local test server.
"""

import socket
import time
from typing import Callable, Optional

from .base import BaseComm, CommStatus
from ..utils.logger import get_logger

_log = get_logger("comm.lte")


class LteComm(BaseComm):
    """
    LTE TCP socket communication carrier.

    Config keys
    -----------
    host : str
        Remote ground station hostname or IP (default ``"127.0.0.1"``).
    port : int
        TCP port (default ``5760``).
    timeout : float
        Socket timeout in seconds (default ``5.0``).
    reconnect_interval : float
        Seconds between reconnect attempts (default ``10.0``).
    """

    def __init__(
        self,
        config: Optional[dict] = None,
        on_receive: Optional[Callable[[bytes], None]] = None,
    ) -> None:
        super().__init__("lte", config, on_receive)
        self._host = self.config.get("host", "127.0.0.1")
        self._port = int(self.config.get("port", 5760))
        self._timeout = float(self.config.get("timeout", 5.0))
        self._sock: Optional[socket.socket] = None

    def connect(self) -> bool:
        self.status = CommStatus.CONNECTING
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(self._timeout)
            self._sock.connect((self._host, self._port))
            self.status = CommStatus.CONNECTED
            _log.info("LTE connected to %s:%d", self._host, self._port)
            return True
        except Exception as exc:
            _log.warning("LTE connection failed: %s", exc)
            self.status = CommStatus.ERROR
            self._sock = None
            return False

    def disconnect(self) -> None:
        self.stop_receive_loop()
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self.status = CommStatus.DISCONNECTED
        _log.info("LTE disconnected")

    def send_bytes(self, data: bytes) -> bool:
        if self._sock is None or self.status != CommStatus.CONNECTED:
            return False
        try:
            self._sock.sendall(data)
            return True
        except Exception as exc:
            _log.warning("LTE send error: %s", exc)
            self.status = CommStatus.ERROR
            return False

    def receive_bytes(self, timeout: float = 1.0) -> Optional[bytes]:
        if self._sock is None or self.status != CommStatus.CONNECTED:
            return None
        try:
            self._sock.settimeout(timeout)
            data = self._sock.recv(4096)
            return data if data else None
        except socket.timeout:
            return None
        except Exception as exc:
            _log.warning("LTE receive error: %s", exc)
            self.status = CommStatus.ERROR
            return None
