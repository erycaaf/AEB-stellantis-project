"""
Unit tests for sil/docker/api/uds_client.py

Drives the client against a local fake TCP relay that mimics
sil/docker/canbus/can_tcp_server.py (16-byte frames) without needing
the full SIL stack.

Run from the repo root:
    cd sil/docker/api
    python -m pytest tests/test_uds_client.py -v
"""

import socket
import struct
import sys
import threading
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from uds_client import (    # noqa: E402  (import after sys.path tweak)
    UDSClient, UDSError, UDSTimeout,
    CAN_ID_REQUEST, CAN_ID_RESPONSE,
    SID_READ_DID, SID_READ_DID_RESP,
    SID_CLEAR_DTC, SID_CLEAR_DTC_RESP,
    SID_ROUTINE_CTRL, SID_ROUTINE_RESP,
    SID_NEGATIVE, NRC_REQUEST_OOR,
    DID_TTC, DID_FSM_STATE, DID_BRAKE_PRESS,
    RID_AEB_CONTROL,
    FRAME_SIZE,
)


# ── Fake TCP CAN relay ────────────────────────────────────────────────────

def _pack_frame(arb_id: int, payload: bytes) -> bytes:
    return struct.pack("<IBxxx", arb_id, len(payload)) \
         + payload.ljust(8, b"\x00")


def _unpack_frame(frame: bytes):
    arb_id = struct.unpack("<I", frame[0:4])[0]
    dlc    = frame[4]
    payload = frame[8:8 + min(dlc, 8)]
    return arb_id, dlc, payload


class FakeRelay:
    """Single-client TCP server that captures sent frames and replies on demand."""

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(1)
        self.port = self.sock.getsockname()[1]
        self.conn = None
        self.received = []   # list of (arb_id, dlc, payload)
        self._reply_payload = None
        self._reply_after = 0.0
        self._auto_reply_fn = None
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def set_reply(self, payload: bytes, *, after_seconds: float = 0.0):
        """Queue a single reply to the next request seen."""
        self._reply_payload = payload
        self._reply_after = after_seconds

    def set_auto_reply(self, fn):
        """Use a function (request_payload -> response_payload) to reply to
        every request automatically. Returning None drops the request."""
        self._auto_reply_fn = fn

    def expect_no_reply(self):
        self._reply_payload = None
        self._auto_reply_fn = None

    def _run(self):
        self.sock.settimeout(2.0)
        try:
            self.conn, _ = self.sock.accept()
        except socket.timeout:
            return
        self.conn.settimeout(0.2)
        while not self._stop.is_set():
            try:
                data = b""
                while len(data) < FRAME_SIZE:
                    chunk = self.conn.recv(FRAME_SIZE - len(data))
                    if not chunk:
                        return
                    data += chunk
                arb_id, dlc, payload = _unpack_frame(data)
                self.received.append((arb_id, dlc, payload))
                if arb_id != CAN_ID_REQUEST:
                    continue
                reply = None
                if self._auto_reply_fn is not None:
                    reply = self._auto_reply_fn(payload)
                elif self._reply_payload is not None:
                    reply = self._reply_payload
                    self._reply_payload = None
                if reply is not None:
                    if self._reply_after > 0:
                        time.sleep(self._reply_after)
                    self.conn.sendall(_pack_frame(CAN_ID_RESPONSE, reply))
            except socket.timeout:
                continue
            except OSError:
                return

    def close(self):
        self._stop.set()
        try:
            if self.conn:
                self.conn.close()
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass


@pytest.fixture()
def relay_and_client():
    """Spin up a FakeRelay and a UDSClient pointed at it."""
    relay = FakeRelay()
    client = UDSClient(host="127.0.0.1", port=relay.port, response_timeout=0.4)
    yield relay, client
    client.close()
    relay.close()


# ── Request encoding ──────────────────────────────────────────────────────

def test_read_did_request_wire_format(relay_and_client):
    relay, client = relay_and_client
    relay.set_reply(bytes([SID_READ_DID_RESP, 0xF1, 0x01, 1, 0, 0, 0, 0]))
    client.read_did(DID_FSM_STATE)
    assert len(relay.received) == 1
    arb_id, dlc, payload = relay.received[0]
    assert arb_id == CAN_ID_REQUEST
    assert dlc == 4
    assert payload == bytes([SID_READ_DID, 0xF1, 0x01, 0x00])


def test_clear_dtc_request_wire_format(relay_and_client):
    relay, client = relay_and_client
    relay.set_reply(bytes([SID_CLEAR_DTC_RESP, 0, 0, 0, 0, 0, 0, 0]))
    client.clear_dtc()
    arb_id, dlc, payload = relay.received[0]
    assert arb_id == CAN_ID_REQUEST
    assert dlc == 4
    assert payload == bytes([SID_CLEAR_DTC, 0xFF, 0xFF, 0xFF])


def test_routine_control_request_wire_format(relay_and_client):
    relay, client = relay_and_client
    relay.set_reply(bytes([SID_ROUTINE_RESP, 0x03, 0x01, 0, 0, 0, 0, 0]))
    client.routine_control(RID_AEB_CONTROL, value=0)
    arb_id, dlc, payload = relay.received[0]
    assert arb_id == CAN_ID_REQUEST
    assert dlc == 4
    assert payload == bytes([SID_ROUTINE_CTRL, 0x03, 0x01, 0x00])


# ── Response decoding (positive cases) ────────────────────────────────────

def test_read_did_fsm_state_decodes(relay_and_client):
    relay, client = relay_and_client
    relay.set_reply(bytes([SID_READ_DID_RESP, 0xF1, 0x01, 3, 0, 0, 0, 0]))
    result = client.read_did(DID_FSM_STATE)
    assert result["did"] == DID_FSM_STATE
    assert result["fsm_state"] == 3
    assert result["fsm_state_name"] == "BRAKE_L1"
    assert result["raw"] == [3, 0, 0, 0, 0]


def test_read_did_ttc_decodes(relay_and_client):
    relay, client = relay_and_client
    # raw uint16 = 350 (0x015E) → ttc_s = 3.50
    relay.set_reply(bytes([
        SID_READ_DID_RESP, 0xF1, 0x00, 0x5E, 0x01, 0, 0, 0
    ]))
    result = client.read_did(DID_TTC)
    assert result["ttc_s"] == pytest.approx(3.50)


def test_read_did_brake_press_decodes(relay_and_client):
    relay, client = relay_and_client
    # raw uint16 = 750 (0x02EE) → brake_pct = 75.0
    relay.set_reply(bytes([
        SID_READ_DID_RESP, 0xF1, 0x02, 0xEE, 0x02, 0, 0, 0
    ]))
    result = client.read_did(DID_BRAKE_PRESS)
    assert result["brake_pct"] == pytest.approx(75.0)


def test_clear_dtc_positive_response(relay_and_client):
    relay, client = relay_and_client
    relay.set_reply(bytes([SID_CLEAR_DTC_RESP, 0, 0, 0, 0, 0, 0, 0]))
    result = client.clear_dtc()
    assert result["ok"] is True


def test_routine_control_positive_response(relay_and_client):
    relay, client = relay_and_client
    relay.set_reply(bytes([SID_ROUTINE_RESP, 0x03, 0x01, 1, 0, 0, 0, 0]))
    result = client.routine_control(RID_AEB_CONTROL, value=1)
    assert result["ok"] is True
    assert result["rid"] == RID_AEB_CONTROL
    assert result["value"] == 1


# ── Error paths ───────────────────────────────────────────────────────────

def test_negative_response_raises_uds_error(relay_and_client):
    relay, client = relay_and_client
    relay.set_reply(bytes([
        SID_NEGATIVE, SID_READ_DID, NRC_REQUEST_OOR, 0, 0, 0, 0, 0
    ]))
    with pytest.raises(UDSError) as exc_info:
        client.read_did(0xF1FF)
    assert exc_info.value.sid == SID_READ_DID
    assert exc_info.value.nrc == NRC_REQUEST_OOR


def test_response_timeout_raises(relay_and_client):
    relay, client = relay_and_client
    relay.expect_no_reply()
    with pytest.raises(UDSTimeout):
        client.read_did(DID_FSM_STATE)


def test_did_echo_mismatch_raises(relay_and_client):
    relay, client = relay_and_client
    # Client asks for 0xF101 but the response echoes 0xF100
    relay.set_reply(bytes([SID_READ_DID_RESP, 0xF1, 0x00, 1, 0, 0, 0, 0]))
    with pytest.raises(UDSError, match="echoed DID"):
        client.read_did(DID_FSM_STATE)


def test_unexpected_response_sid_raises(relay_and_client):
    relay, client = relay_and_client
    # Client asks for ReadDID but server sends a RoutineControl positive response
    relay.set_reply(bytes([SID_ROUTINE_RESP, 0xF1, 0x01, 0, 0, 0, 0, 0]))
    with pytest.raises(UDSError, match="unexpected response SID"):
        client.read_did(DID_FSM_STATE)


def test_did_out_of_range_raises_value_error(relay_and_client):
    _, client = relay_and_client
    with pytest.raises(ValueError):
        client.read_did(0x10000)


def test_routine_value_out_of_range_raises(relay_and_client):
    _, client = relay_and_client
    with pytest.raises(ValueError):
        client.routine_control(RID_AEB_CONTROL, value=256)


# ── Concurrency ───────────────────────────────────────────────────────────

def test_concurrent_requests_serialise(relay_and_client):
    """Two threads issuing read_did simultaneously must each get a
    well-formed response (the request lock serialises them so neither
    thread reads the other's response)."""
    relay, client = relay_and_client

    # Auto-reply: echo the request DID with a STANDBY (state=1) value.
    # That way each request gets its own reply with the correct echo,
    # regardless of arrival order.
    def echo_reply(req_payload):
        # req_payload[0] = SID_READ_DID, [1:3] = DID, [3] = value byte
        return bytes([
            SID_READ_DID_RESP, req_payload[1], req_payload[2],
            1, 0, 0, 0, 0,
        ])
    relay.set_auto_reply(echo_reply)

    results = []
    errors  = []

    def hammer():
        try:
            for _ in range(3):
                results.append(client.read_did(DID_FSM_STATE))
        except Exception as e:
            errors.append(e)

    threads = [threading.Thread(target=hammer) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=4.0)
    assert not errors, f"unexpected errors: {errors}"
    # 2 threads x 3 requests each = 6 successful round-trips.
    # If the request lock weren't there, the second thread could read
    # the first's response and raise a DID echo mismatch.
    assert len(results) == 6
    assert all(r["fsm_state_name"] == "STANDBY" for r in results)
