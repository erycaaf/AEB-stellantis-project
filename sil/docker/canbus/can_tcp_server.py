#!/usr/bin/env python3
"""
TCP CAN Bus Server
==================
Simple relay: any frame sent by one client is broadcast to all others.
Wire format: 16 bytes per frame [4B ID | 1B DLC | 3B pad | 8B data]

All containers connect to this server to exchange CAN frames.
Replaces vcan0/SocketCAN in Docker environments.

Usage: python3 can_tcp_server.py [port]
Default port: 29536
"""

import socket
import threading
import sys
import struct

FRAME_SIZE = 16
DEFAULT_PORT = 29536

clients = []
lock = threading.Lock()
frame_count = 0


def handle_client(conn, addr):
    global frame_count
    print(f"[CAN BUS] Client connected: {addr}", flush=True)

    with lock:
        clients.append(conn)

    try:
        while True:
            data = b''
            while len(data) < FRAME_SIZE:
                chunk = conn.recv(FRAME_SIZE - len(data))
                if not chunk:
                    return
                data += chunk

            frame_count += 1

            # Broadcast to all other clients
            with lock:
                for c in clients:
                    if c is not conn:
                        try:
                            c.sendall(data)
                        except Exception:
                            pass

            # Log every 100th frame
            if frame_count % 100 == 0:
                can_id = struct.unpack('<I', data[0:4])[0]
                dlc = data[4]
                print(f"[CAN BUS] {frame_count} frames | last: 0x{can_id:03X} DLC={dlc}",
                      flush=True)

    except Exception as e:
        print(f"[CAN BUS] Client {addr} error: {e}", flush=True)
    finally:
        with lock:
            if conn in clients:
                clients.remove(conn)
        conn.close()
        print(f"[CAN BUS] Client disconnected: {addr}", flush=True)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_PORT

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('0.0.0.0', port))
    server.listen(10)

    print(f"[CAN BUS] TCP CAN bus server on port {port}", flush=True)
    print("[CAN BUS] Frame format: 16 bytes [4B ID | 1B DLC | 3B pad | 8B data]",
          flush=True)

    while True:
        conn, addr = server.accept()
        t = threading.Thread(target=handle_client, args=(conn, addr), daemon=True)
        t.start()


if __name__ == '__main__':
    main()
