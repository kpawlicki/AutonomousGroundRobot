"""Tests for communication protocol and carriers."""

import struct
import time
import pytest

from autonomous_ground_robot.communication.protocol import (
    Message,
    MessageType,
    GroundStationProtocol,
    _crc8,
    STX,
    ETX,
)
from autonomous_ground_robot.communication.base import CommStatus
from autonomous_ground_robot.communication.lora import LoraComm
from autonomous_ground_robot.communication.radio import RadioComm
from autonomous_ground_robot.communication.lte import LteComm


# ---------------------------------------------------------------------------
# CRC-8
# ---------------------------------------------------------------------------

class TestCrc8:
    def test_deterministic(self):
        assert _crc8(b"hello") == _crc8(b"hello")

    def test_different_data(self):
        assert _crc8(b"hello") != _crc8(b"world")

    def test_empty(self):
        assert _crc8(b"") == 0

    def test_range(self):
        for b in range(256):
            crc = _crc8(bytes([b]))
            assert 0 <= crc <= 255


# ---------------------------------------------------------------------------
# Message encoding / decoding
# ---------------------------------------------------------------------------

class TestMessage:
    def test_roundtrip_heartbeat(self):
        msg = Message(msg_type=MessageType.HEARTBEAT, payload={"seq": 1, "ts": 1234.5})
        raw = msg.to_bytes()
        parsed = Message.from_bytes(raw)
        assert parsed is not None
        assert parsed.msg_type == MessageType.HEARTBEAT
        assert parsed.payload["seq"] == 1
        assert abs(parsed.payload["ts"] - 1234.5) < 1e-6

    def test_roundtrip_telemetry(self):
        payload = {"x": 1.23, "y": 4.56, "heading": 90.0, "state": "NAVIGATING"}
        msg = Message(msg_type=MessageType.TELEMETRY, payload=payload)
        raw = msg.to_bytes()
        parsed = Message.from_bytes(raw)
        assert parsed is not None
        assert parsed.payload["x"] == pytest.approx(1.23)
        assert parsed.payload["state"] == "NAVIGATING"

    def test_roundtrip_command(self):
        payload = {"type": "goto_gps", "latitude": 48.86, "longitude": 2.35}
        msg = Message(msg_type=MessageType.COMMAND, payload=payload)
        raw = msg.to_bytes()
        parsed = Message.from_bytes(raw)
        assert parsed is not None
        assert parsed.payload["latitude"] == pytest.approx(48.86)

    def test_stx_etx_markers(self):
        msg = Message(msg_type=MessageType.STATUS, payload={"status": "ready"})
        raw = msg.to_bytes()
        assert raw[0] == STX
        assert raw[-1] == ETX

    def test_corrupted_crc_rejected(self):
        msg = Message(msg_type=MessageType.HEARTBEAT, payload={"seq": 1})
        raw = bytearray(msg.to_bytes())
        raw[-2] ^= 0xFF  # flip CRC byte
        parsed = Message.from_bytes(bytes(raw))
        assert parsed is None

    def test_wrong_stx_rejected(self):
        msg = Message(msg_type=MessageType.HEARTBEAT, payload={})
        raw = bytearray(msg.to_bytes())
        raw[0] = 0x00
        assert Message.from_bytes(bytes(raw)) is None

    def test_wrong_etx_rejected(self):
        msg = Message(msg_type=MessageType.HEARTBEAT, payload={})
        raw = bytearray(msg.to_bytes())
        raw[-1] = 0x00
        assert Message.from_bytes(bytes(raw)) is None

    def test_too_short_rejected(self):
        assert Message.from_bytes(b"\xAA\x01") is None

    def test_empty_payload(self):
        msg = Message(msg_type=MessageType.ACK, payload={})
        raw = msg.to_bytes()
        parsed = Message.from_bytes(raw)
        assert parsed is not None
        assert parsed.payload == {}

    def test_nested_payload(self):
        payload = {"nested": {"a": 1, "b": [1, 2, 3]}}
        msg = Message(msg_type=MessageType.TELEMETRY, payload=payload)
        raw = msg.to_bytes()
        parsed = Message.from_bytes(raw)
        assert parsed is not None
        assert parsed.payload["nested"]["b"] == [1, 2, 3]


# ---------------------------------------------------------------------------
# LoRa carrier (simulation mode)
# ---------------------------------------------------------------------------

class TestLoraComm:
    def test_connect_disconnect(self):
        c = LoraComm()
        assert c.connect()
        assert c.status == CommStatus.CONNECTED
        c.disconnect()
        assert c.status == CommStatus.DISCONNECTED

    def test_send_and_receive(self):
        c = LoraComm()
        c.connect()
        data = b"hello lora"
        assert c.send_bytes(data)
        received = c.get_sent(timeout=0.1)
        assert received == data
        c.disconnect()

    def test_inject_receive(self):
        c = LoraComm()
        c.connect()
        c.inject_receive(b"injected")
        data = c.receive_bytes(timeout=0.1)
        assert data == b"injected"
        c.disconnect()

    def test_send_while_disconnected(self):
        c = LoraComm()
        assert not c.send_bytes(b"nope")

    def test_context_manager(self):
        with LoraComm() as c:
            assert c.status == CommStatus.CONNECTED


# ---------------------------------------------------------------------------
# Radio carrier (simulation mode)
# ---------------------------------------------------------------------------

class TestRadioComm:
    def test_connect_disconnect(self):
        c = RadioComm()
        assert c.connect()
        assert c.status == CommStatus.CONNECTED
        c.disconnect()
        assert c.status == CommStatus.DISCONNECTED

    def test_send_and_receive(self):
        c = RadioComm()
        c.connect()
        data = b"hello radio"
        assert c.send_bytes(data)
        received = c.get_sent(timeout=0.1)
        assert received == data
        c.disconnect()

    def test_inject_receive(self):
        c = RadioComm()
        c.connect()
        c.inject_receive(b"over the air")
        data = c.receive_bytes(timeout=0.1)
        assert data == b"over the air"
        c.disconnect()


# ---------------------------------------------------------------------------
# LTE carrier (simulation/connection refused)
# ---------------------------------------------------------------------------

class TestLteComm:
    def test_connect_refused(self):
        # Port 1 is reserved and should always refuse connections
        c = LteComm({"host": "127.0.0.1", "port": 1, "timeout": 0.5})
        connected = c.connect()
        assert not connected
        assert c.status == CommStatus.ERROR

    def test_send_while_disconnected(self):
        c = LteComm({"host": "127.0.0.1", "port": 1, "timeout": 0.5})
        assert not c.send_bytes(b"data")


# ---------------------------------------------------------------------------
# Ground Station Protocol (with LoRa in simulation)
# ---------------------------------------------------------------------------

class TestGroundStationProtocol:
    def _make_protocol(self, on_command=None):
        carrier = LoraComm()
        return GroundStationProtocol(
            carrier=carrier,
            telemetry_interval=0.05,
            heartbeat_interval=0.1,
            on_command=on_command,
        )

    def test_start_stop(self):
        proto = self._make_protocol()
        assert proto.start()
        proto.stop()

    def test_heartbeat_is_sent(self):
        carrier = LoraComm()
        proto = GroundStationProtocol(
            carrier=carrier,
            heartbeat_interval=0.05,
            telemetry_interval=10.0,
        )
        proto.start()
        time.sleep(0.15)  # allow at least one heartbeat
        proto.stop()
        # At least one message should be in the outbound queue
        data = carrier.get_sent(timeout=0.1)
        assert data is not None
        msg = Message.from_bytes(data)
        assert msg is not None
        assert msg.msg_type == MessageType.HEARTBEAT

    def test_telemetry_sent_when_data_available(self):
        carrier = LoraComm()
        proto = GroundStationProtocol(
            carrier=carrier,
            telemetry_interval=0.05,
            heartbeat_interval=100.0,
        )
        proto.update_telemetry({"x": 1.0, "y": 2.0})
        proto.start()
        time.sleep(0.15)
        proto.stop()
        # Drain the outbound queue looking for a TELEMETRY message
        found = False
        while True:
            data = carrier.get_sent(timeout=0.05)
            if data is None:
                break
            msg = Message.from_bytes(data)
            if msg and msg.msg_type == MessageType.TELEMETRY:
                assert msg.payload["x"] == pytest.approx(1.0)
                found = True
                break
        assert found

    def test_command_dispatched(self):
        received_cmds = []
        carrier = LoraComm()
        proto = GroundStationProtocol(
            carrier=carrier,
            heartbeat_interval=100.0,
            telemetry_interval=100.0,
            on_command=received_cmds.append,
        )
        proto.start()

        # Simulate incoming COMMAND from ground station
        cmd_msg = Message(
            msg_type=MessageType.COMMAND,
            payload={"type": "stop"},
        )
        carrier.inject_receive(cmd_msg.to_bytes())
        time.sleep(0.2)
        proto.stop()

        assert len(received_cmds) == 1
        assert received_cmds[0]["type"] == "stop"
