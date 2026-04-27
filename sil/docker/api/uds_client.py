"""
UDS Client over TCP-CAN
=======================
Synchronous UDS (ISO 14229) client that talks to the AEB ECU via the
SIL's TCP CAN relay (`canbus`).

Wire format on the relay matches `sil/docker/canbus/can_tcp_server.py`:
    16 bytes per frame  =  4B ID  |  1B DLC  |  3B pad  |  8B data

CAN IDs (production code, see include/aeb_can.h):
    0x7DF  UDS request   TX from this client
    0x7E8  UDS response  RX

Services exposed (subset of ISO 14229):
    0x22  ReadDataByIdentifier
    0x14  ClearDiagnosticInformation
    0x31  RoutineControl

DIDs / RIDs (production code, see include/aeb_uds.h):
    0xF100  TTC value          (data1+data2: uint16 little-endian, x100)
    0xF101  FSM state          (data1: raw uint8)
    0xF102  Brake pressure     (data1+data2: uint16 little-endian, x10)
    0x0301  AEB enable/disable (RoutineControl, value byte: 0=disable, 1=enable)
"""

import socket
import struct
import threading
import time
from queue import Queue, Empty


# ── Wire constants ────────────────────────────────────────────────────────

FRAME_SIZE       = 16
CAN_ID_REQUEST   = 0x7DF
CAN_ID_RESPONSE  = 0x7E8

SID_READ_DID         = 0x22
SID_READ_DID_RESP    = 0x62
SID_CLEAR_DTC        = 0x14
SID_CLEAR_DTC_RESP   = 0x54
SID_ROUTINE_CTRL     = 0x31
SID_ROUTINE_RESP     = 0x71
SID_NEGATIVE         = 0x7F

NRC_SERVICE_NOT_SUPP = 0x11
NRC_REQUEST_OOR      = 0x31

DID_TTC          = 0xF100
DID_FSM_STATE    = 0xF101
DID_BRAKE_PRESS  = 0xF102
RID_AEB_CONTROL  = 0x0301

FSM_STATE_NAMES = {
    0: "OFF",
    1: "STANDBY",
    2: "WARNING",
    3: "BRAKE_L1",
    4: "BRAKE_L2",
    5: "BRAKE_L3",
    6: "POST_BRAKE",
}


# ── Exceptions ────────────────────────────────────────────────────────────

class UDSError(Exception):
    """Raised on a negative response (NRC) or timeout."""

    def __init__(self, message, *, sid=None, nrc=None):
        super().__init__(message)
        self.sid = sid
        self.nrc = nrc


class UDSTimeout(UDSError):
    """Raised when the response does not arrive in time."""


# ── Client ────────────────────────────────────────────────────────────────

class UDSClient:
    """Synchronous request/response UDS client.

    A background thread reads frames from the TCP relay and pushes any
    that match `CAN_ID_RESPONSE` onto a queue. Public methods are
    thread-safe via a request lock so concurrent FastAPI handlers
    serialize cleanly.
    """

    def __init__(self, host="canbus", port=29536, *, response_timeout=0.5):
        self.host = host
        self.port = port
        self.response_timeout = response_timeout
        self._sock = None
        self._send_lock = threading.Lock()
        self._req_lock  = threading.Lock()
        self._rx_queue  = Queue()
        self._rx_thread = None
        self._stop_evt  = threading.Event()
        self._connect()

    # ── Connection management ─────────────────────────────────────────

    def _connect(self, retries=30):
        last_err = None
        for _ in range(retries):
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.connect((self.host, self.port))
                self._sock = s
                self._stop_evt.clear()
                self._rx_thread = threading.Thread(
                    target=self._rx_loop, daemon=True
                )
                self._rx_thread.start()
                return
            except OSError as e:
                last_err = e
                time.sleep(1)
        raise ConnectionError(
            f"UDSClient: cannot connect to {self.host}:{self.port}: {last_err}"
        )

    def close(self):
        self._stop_evt.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._sock.close()
            self._sock = None

    # ── Background RX thread ──────────────────────────────────────────

    def _rx_loop(self):
        while not self._stop_evt.is_set():
            try:
                data = b""
                while len(data) < FRAME_SIZE:
                    chunk = self._sock.recv(FRAME_SIZE - len(data))
                    if not chunk:
                        return
                    data += chunk
                arb_id = struct.unpack("<I", data[0:4])[0]
                dlc    = data[4]
                payload = data[8:8 + min(dlc, 8)]
                if arb_id == CAN_ID_RESPONSE:
                    self._rx_queue.put(payload)
            except OSError:
                return

    # ── Frame I/O primitives ──────────────────────────────────────────

    def _send_request(self, payload: bytes):
        if len(payload) > 8:
            raise ValueError("UDS request payload must fit in 8 bytes")
        dlc = len(payload)
        frame = struct.pack("<IBxxx", CAN_ID_REQUEST, dlc) \
              + payload.ljust(8, b"\x00")
        with self._send_lock:
            self._sock.sendall(frame)

    def _wait_response(self) -> bytes:
        try:
            return self._rx_queue.get(timeout=self.response_timeout)
        except Empty:
            raise UDSTimeout(
                f"no UDS response within {self.response_timeout}s"
            )

    def _drain_queue(self):
        while not self._rx_queue.empty():
            try:
                self._rx_queue.get_nowait()
            except Empty:
                break

    @staticmethod
    def _check_negative(payload: bytes, expected_resp_sid: int):
        """Raise UDSError if the response is a negative (0x7F) reply."""
        if len(payload) >= 3 and payload[0] == SID_NEGATIVE:
            raise UDSError(
                f"negative response: SID=0x{payload[1]:02X} "
                f"NRC=0x{payload[2]:02X}",
                sid=payload[1], nrc=payload[2],
            )
        if not payload or payload[0] != expected_resp_sid:
            raise UDSError(
                f"unexpected response SID 0x{payload[0]:02X} "
                f"(expected 0x{expected_resp_sid:02X})"
            )

    # ── ReadDataByIdentifier (0x22) ───────────────────────────────────

    def read_did(self, did: int) -> dict:
        """Issue ReadDID(did) and return decoded fields.

        Returns a dict shaped as
            {
              "did":     0xF101,
              "did_hex": "0xF101",
              "raw":     [data1..data5],
              ... per-DID decoded fields ...
            }
        """
        if not (0 <= did <= 0xFFFF):
            raise ValueError(f"DID out of range: 0x{did:X}")
        with self._req_lock:
            self._drain_queue()
            self._send_request(bytes([
                SID_READ_DID,
                (did >> 8) & 0xFF,
                did & 0xFF,
                0x00,
            ]))
            payload = self._wait_response()
        self._check_negative(payload, SID_READ_DID_RESP)
        # payload[1:3] echoes the DID
        echoed_did = (payload[1] << 8) | payload[2]
        if echoed_did != did:
            raise UDSError(
                f"response echoed DID 0x{echoed_did:04X} "
                f"(requested 0x{did:04X})"
            )
        raw = list(payload[3:8])
        result = {
            "did":     did,
            "did_hex": f"0x{did:04X}",
            "raw":     raw,
        }
        result.update(_decode_did(did, raw))
        return result

    # ── ClearDiagnosticInformation (0x14) ─────────────────────────────

    def clear_dtc(self) -> dict:
        """Clear all DTCs (group 0xFFFFFF — all groups)."""
        with self._req_lock:
            self._drain_queue()
            self._send_request(bytes([SID_CLEAR_DTC, 0xFF, 0xFF, 0xFF]))
            payload = self._wait_response()
        self._check_negative(payload, SID_CLEAR_DTC_RESP)
        return {"ok": True, "raw": list(payload)}

    # ── RoutineControl (0x31) ─────────────────────────────────────────

    def routine_control(self, rid: int, value: int) -> dict:
        """Send RoutineControl(rid, value).

        For RID 0x0301 (AEB enable):
            value=0 -> disable AEB
            value=1 -> enable AEB
        """
        if not (0 <= rid <= 0xFFFF):
            raise ValueError(f"RID out of range: 0x{rid:X}")
        if not (0 <= value <= 0xFF):
            raise ValueError(f"value out of range: {value}")
        with self._req_lock:
            self._drain_queue()
            self._send_request(bytes([
                SID_ROUTINE_CTRL,
                (rid >> 8) & 0xFF,
                rid & 0xFF,
                value,
            ]))
            payload = self._wait_response()
        self._check_negative(payload, SID_ROUTINE_RESP)
        return {"ok": True, "rid": rid, "value": value, "raw": list(payload)}


# ── DID-specific decoding helpers ─────────────────────────────────────────

def _decode_did(did: int, raw: list) -> dict:
    """Apply DID-specific physical decoding on top of raw bytes."""
    if did == DID_TTC:
        scaled = raw[0] | (raw[1] << 8)
        return {"ttc_s": scaled / 100.0}
    if did == DID_FSM_STATE:
        state = raw[0]
        return {
            "fsm_state":      state,
            "fsm_state_name": FSM_STATE_NAMES.get(state, "UNKNOWN"),
        }
    if did == DID_BRAKE_PRESS:
        scaled = raw[0] | (raw[1] << 8)
        return {"brake_pct": scaled / 10.0}
    return {}
