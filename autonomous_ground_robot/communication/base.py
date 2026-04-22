"""
Abstract base class for all communication carriers.

All carriers (LTE, LoRa, Radio …) implement the same interface so that the
ground station protocol layer is completely carrier-agnostic.
"""

import abc
import threading
from enum import Enum, auto
from typing import Callable, Optional

from ..utils.logger import get_logger

_log = get_logger("comm.base")


class CommStatus(Enum):
    DISCONNECTED = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    ERROR = auto()


class BaseComm(abc.ABC):
    """
    Abstract communication carrier.

    Subclasses must implement :meth:`connect`, :meth:`disconnect`,
    :meth:`send_bytes`, and :meth:`receive_bytes`.

    An optional *on_receive* callback is called from a background receive
    thread whenever new data arrives.
    """

    def __init__(
        self,
        name: str,
        config: Optional[dict] = None,
        on_receive: Optional[Callable[[bytes], None]] = None,
    ) -> None:
        self.name = name
        self.config = config or {}
        self.on_receive = on_receive
        self.status = CommStatus.DISCONNECTED
        self._recv_thread: Optional[threading.Thread] = None
        self._running = False

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abc.abstractmethod
    def connect(self) -> bool:
        """
        Establish a connection.

        Returns ``True`` on success, ``False`` otherwise.
        Updates :attr:`status` accordingly.
        """

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Tear down the connection and release resources."""

    @abc.abstractmethod
    def send_bytes(self, data: bytes) -> bool:
        """
        Send raw bytes.

        Returns ``True`` if the data was handed off to the carrier
        successfully (not necessarily acknowledged by the remote end).
        """

    def receive_bytes(self, timeout: float = 1.0) -> Optional[bytes]:
        """
        Blocking receive call (optional override).

        Returns raw bytes or ``None`` if nothing was received within
        *timeout* seconds.  The default implementation returns ``None``
        (poll-only carriers should override this).
        """
        return None

    # ------------------------------------------------------------------
    # Background receive loop
    # ------------------------------------------------------------------

    def start_receive_loop(self) -> None:
        """
        Start a daemon thread that continuously calls :meth:`receive_bytes`
        and dispatches received data to :attr:`on_receive`.
        """
        if self._recv_thread and self._recv_thread.is_alive():
            return
        self._running = True
        self._recv_thread = threading.Thread(
            target=self._recv_loop,
            daemon=True,
            name=f"comm-recv-{self.name}",
        )
        self._recv_thread.start()

    def stop_receive_loop(self) -> None:
        self._running = False
        if self._recv_thread:
            self._recv_thread.join(timeout=2.0)

    def _recv_loop(self) -> None:
        while self._running and self.status == CommStatus.CONNECTED:
            try:
                data = self.receive_bytes(timeout=1.0)
                if data and self.on_receive:
                    self.on_receive(data)
            except Exception as exc:  # pragma: no cover
                _log.warning("[%s] receive error: %s", self.name, exc)

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "BaseComm":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()

    def __repr__(self) -> str:
        return f"<{self.__class__.__name__} name={self.name!r} status={self.status.name}>"
