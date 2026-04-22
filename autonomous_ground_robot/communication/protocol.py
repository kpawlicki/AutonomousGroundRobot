"""
Ground station communication protocol.

Defines the message format and :class:`GroundStationProtocol` which sits
on top of any :class:`BaseComm` carrier and provides structured telemetry
and command exchange.

Wire format (little-endian)
---------------------------
Byte 0      : STX (0xAA)
Byte 1      : message type (uint8)
Bytes 2-3   : payload length (uint16)
Bytes 4-N   : JSON payload (UTF-8)
Byte N+1    : CRC-8 of bytes 1..N
Byte N+2    : ETX (0x55)
"""

import json
import struct
import threading
import time
from dataclasses import asdict, dataclass, field
from enum import IntEnum
from typing import Any, Callable, Dict, Optional

from .base import BaseComm, CommStatus
from ..utils.logger import get_logger

_log = get_logger("comm.protocol")

STX = 0xAA
ETX = 0x55


class MessageType(IntEnum):
    HEARTBEAT = 0x01
    TELEMETRY = 0x02
    COMMAND = 0x03
    STATUS = 0x04
    ACK = 0x05
    NACK = 0x06


@dataclass
class Message:
    """A protocol-level message."""

    msg_type: int
    payload: Dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)

    def to_bytes(self) -> bytes:
        """Serialise the message to the wire format."""
        body = json.dumps(self.payload).encode("utf-8")
        length = len(body)
        header = struct.pack("<BH", self.msg_type, length)
        crc = _crc8(header + body)
        return bytes([STX]) + header + body + bytes([crc, ETX])

    @staticmethod
    def from_bytes(data: bytes) -> Optional["Message"]:
        """
        Parse a message from raw bytes.

        Returns ``None`` if parsing fails or the CRC is wrong.
        """
        try:
            if len(data) < 6:
                return None
            if data[0] != STX or data[-1] != ETX:
                return None
            msg_type, length = struct.unpack_from("<BH", data, 1)
            if len(data) != 1 + 3 + length + 2:
                return None
            header_and_body = data[1: 4 + length]
            body = data[4: 4 + length]
            expected_crc = _crc8(header_and_body)
            if data[-2] != expected_crc:
                _log.debug("CRC mismatch: got %02x expected %02x", data[-2], expected_crc)
                return None
            payload = json.loads(body.decode("utf-8"))
            return Message(msg_type=msg_type, payload=payload)
        except Exception as exc:
            _log.debug("Message parse error: %s", exc)
            return None


def _crc8(data: bytes) -> int:
    """Simple CRC-8 (polynomial 0x07)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


class GroundStationProtocol:
    """
    Application-layer protocol for robot ↔ ground station communication.

    Parameters
    ----------
    carrier : BaseComm
        The underlying communication carrier (LTE, LoRa, Radio …).
    telemetry_interval : float
        How often to send a TELEMETRY message (seconds, default ``1.0``).
    heartbeat_interval : float
        How often to send a HEARTBEAT message (seconds, default ``5.0``).
    on_command : callable
        Called with the decoded command ``dict`` when a COMMAND message
        is received from the ground station.
    """

    def __init__(
        self,
        carrier: BaseComm,
        telemetry_interval: float = 1.0,
        heartbeat_interval: float = 5.0,
        on_command: Optional[Callable[[dict], None]] = None,
    ) -> None:
        self.carrier = carrier
        self.telemetry_interval = telemetry_interval
        self.heartbeat_interval = heartbeat_interval
        self.on_command = on_command

        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._last_hb = 0.0
        self._last_telem = 0.0
        self._seq = 0

        # Register receive handler on the carrier
        self.carrier.on_receive = self._handle_raw

        # Latest telemetry data provided by the robot (updated externally)
        self._telemetry: dict = {}

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> bool:
        """Connect the carrier and start the protocol loop."""
        if not self.carrier.connect():
            return False
        self.carrier.start_receive_loop()
        self._running = True
        self._thread = threading.Thread(
            target=self._loop,
            daemon=True,
            name="gs-protocol",
        )
        self._thread.start()
        _log.info("Ground station protocol started on carrier %s", self.carrier.name)
        return True

    def stop(self) -> None:
        """Stop the protocol loop and disconnect the carrier."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        self.carrier.stop_receive_loop()
        self.carrier.disconnect()
        _log.info("Ground station protocol stopped")

    # ------------------------------------------------------------------
    # Telemetry update
    # ------------------------------------------------------------------

    def update_telemetry(self, data: dict) -> None:
        """
        Update the telemetry data that will be sent to the ground station.

        Typical fields: position (lat, lon, alt), heading, speed, battery,
        navigation_state, obstacle_detected.
        """
        self._telemetry = data

    # ------------------------------------------------------------------
    # Sending helpers
    # ------------------------------------------------------------------

    def send_heartbeat(self) -> bool:
        self._seq += 1
        msg = Message(
            msg_type=MessageType.HEARTBEAT,
            payload={"seq": self._seq, "ts": time.time()},
        )
        return self.carrier.send_bytes(msg.to_bytes())

    def send_telemetry(self) -> bool:
        msg = Message(
            msg_type=MessageType.TELEMETRY,
            payload=self._telemetry,
        )
        return self.carrier.send_bytes(msg.to_bytes())

    def send_status(self, status: str, detail: str = "") -> bool:
        msg = Message(
            msg_type=MessageType.STATUS,
            payload={"status": status, "detail": detail, "ts": time.time()},
        )
        return self.carrier.send_bytes(msg.to_bytes())

    def send_ack(self, ref_type: int) -> bool:
        msg = Message(
            msg_type=MessageType.ACK,
            payload={"ref_type": ref_type},
        )
        return self.carrier.send_bytes(msg.to_bytes())

    # ------------------------------------------------------------------
    # Private
    # ------------------------------------------------------------------

    def _loop(self) -> None:
        while self._running:
            now = time.time()
            if now - self._last_hb >= self.heartbeat_interval:
                self.send_heartbeat()
                self._last_hb = now
            if now - self._last_telem >= self.telemetry_interval and self._telemetry:
                self.send_telemetry()
                self._last_telem = now
            time.sleep(0.05)

    def _handle_raw(self, data: bytes) -> None:
        msg = Message.from_bytes(data)
        if msg is None:
            _log.debug("Received unparseable data (%d bytes)", len(data))
            return
        _log.debug("Received message type=0x%02x", msg.msg_type)
        if msg.msg_type == MessageType.COMMAND:
            self.send_ack(MessageType.COMMAND)
            if self.on_command:
                self.on_command(msg.payload)
        elif msg.msg_type == MessageType.HEARTBEAT:
            # Echo back an ACK for ground station heartbeats
            self.send_ack(MessageType.HEARTBEAT)
