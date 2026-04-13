"""
TCP CAN Bus Client
==================
Connects to the TCP CAN bus server and provides send/recv for CAN frames.
Wire format matches the server: 16 bytes [4B ID | 1B DLC | 3B pad | 8B data]

Drop-in replacement for python-can's SocketCAN interface in Docker.
"""

import socket
import struct
import threading
import time

FRAME_SIZE = 16


class TcpCanBus:
    def __init__(self, host='canbus', port=29536, timeout=30):
        self.sock = None
        self.host = host
        self.port = port
        self.lock = threading.Lock()
        self.rx_callbacks = []

        # Retry connection
        for attempt in range(timeout):
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect((host, port))
                print(f"[CAN TCP] Connected to {host}:{port}")

                # Start RX thread
                self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
                self.rx_thread.start()
                return
            except Exception as e:
                if self.sock:
                    self.sock.close()
                self.sock = None
                if attempt % 5 == 0:
                    print(f"[CAN TCP] Connecting to {host}:{port}... (attempt {attempt+1})")
                time.sleep(1)

        print(f"[CAN TCP] ERROR: Could not connect to {host}:{port}")

    def send(self, arb_id, data):
        """Send a CAN frame."""
        if self.sock is None:
            return

        dlc = len(data)
        frame = struct.pack('<IBxxx', arb_id, dlc) + bytes(data).ljust(8, b'\x00')[:8]

        with self.lock:
            try:
                self.sock.sendall(frame)
            except Exception:
                pass

    def on_receive(self, callback):
        """Register a callback for received frames: callback(arb_id, data, dlc)"""
        self.rx_callbacks.append(callback)

    def _rx_loop(self):
        """Background thread reading frames from the bus."""
        while True:
            try:
                data = b''
                while len(data) < FRAME_SIZE:
                    chunk = self.sock.recv(FRAME_SIZE - len(data))
                    if not chunk:
                        time.sleep(0.5)
                        continue
                    data += chunk

                arb_id, dlc = struct.unpack('<IB', data[0:5])
                frame_data = data[8:8+min(dlc, 8)]

                for cb in self.rx_callbacks:
                    try:
                        cb(arb_id, frame_data, dlc)
                    except Exception:
                        pass

            except Exception:
                time.sleep(0.5)

    def shutdown(self):
        if self.sock:
            self.sock.close()
