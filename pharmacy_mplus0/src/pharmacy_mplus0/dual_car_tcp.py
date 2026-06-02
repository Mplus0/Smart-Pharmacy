# -*- coding: utf-8 -*-
"""Dual-car TCP communication bridge.

Replaces ROS /dual_car_signal and /current_qr_task topics with direct TCP
communication between two cars running independent ROS Masters.

Each car runs both:
  - A TCP server (to receive messages from the peer)
  - A TCP client (to send messages to the peer)

Protocol: JSON object + newline (\\n) per message, easy to debug with nc / telnet.

Python 2.7 / ROS Melodic compatible. Standard library only.
"""

from __future__ import print_function

import json
import socket
import threading
import time


class DualCarTcpBridge(object):
    """Bidirectional TCP bridge for dual-car communication.

    Usage::

        def on_message(payload):
            msg_type = payload.get("type")
            if msg_type == "allow_start":
                handle_allow_start(payload)
            ...

        bridge = DualCarTcpBridge(
            car_id="1",
            peer_id="2",
            listen_ip="0.0.0.0",
            listen_port=9001,
            peer_ip="192.168.124.9",
            peer_port=9001,
            on_message=on_message,
        )
        bridge.start()
        ...
        bridge.send({"type": "allow_start", "from_car": "1", ...})
        ...
        bridge.stop()
    """

    def __init__(self, car_id, peer_id, listen_ip, listen_port,
                 peer_ip, peer_port, on_message,
                 connect_timeout=1.0, reconnect_interval=2.0,
                 logger=None):
        """Initialise but do not connect — call start() to begin.

        Args:
            car_id:            This car's ID ("1" or "2").
            peer_id:           The other car's ID.
            listen_ip:         IP address to bind the server socket to.
            listen_port:       Port to bind the server socket to.
            peer_ip:           Peer's IP address to connect to for sending.
            peer_port:         Peer's port to connect to for sending.
            on_message:        Callback called with a single dict argument
                               when a complete JSON message is received.
            connect_timeout:   Seconds before a connect attempt times out.
            reconnect_interval: Seconds to wait between reconnect attempts.
            logger:            Optional object with .info(), .warn(),
                               .error() methods. If None, logs are silent.
        """
        self._car_id = str(car_id)
        self._peer_id = str(peer_id)
        self._listen_ip = str(listen_ip)
        self._listen_port = int(listen_port)
        self._peer_ip = str(peer_ip)
        self._peer_port = int(peer_port)
        self._on_message = on_message
        self._connect_timeout = float(connect_timeout)
        self._reconnect_interval = float(reconnect_interval)

        # ---- logging ----------------------------------------------------
        if logger is not None:
            self._log_info = getattr(logger, "info", lambda *a: None)
            self._log_warn = getattr(logger, "warn",
                                     getattr(logger, "warning",
                                             lambda *a: None))
            self._log_error = getattr(logger, "error", lambda *a: None)
        else:
            self._log_info = lambda *a: None
            self._log_warn = lambda *a: None
            self._log_error = lambda *a: None

        # ---- runtime state ----------------------------------------------
        self._stop_event = threading.Event()
        self._server_sock = None
        self._client_sock = None
        self._client_lock = threading.Lock()
        self._server_thread = None
        self._client_thread = None
        self._started = False

    # ---- public API -----------------------------------------------------

    def start(self):
        """Start the server and client background threads."""
        if self._started:
            return
        self._started = True
        self._stop_event.clear()

        self._server_thread = threading.Thread(
            target=self._server_loop, name="dual_car_server",
        )
        self._server_thread.daemon = True
        self._server_thread.start()

        self._client_thread = threading.Thread(
            target=self._client_loop, name="dual_car_client",
        )
        self._client_thread.daemon = True
        self._client_thread.start()

        self._log_info(
            "[DualCarTcp] bridge started "
            "(car=%s peer=%s listen=%s:%d peer=%s:%d)",
            self._car_id, self._peer_id,
            self._listen_ip, self._listen_port,
            self._peer_ip, self._peer_port,
        )

    def stop(self):
        """Shut down all threads and close sockets.

        Safe to call multiple times; safe to call without a prior start().
        """
        self._stop_event.set()
        self._close_server_sock()
        self._close_client_sock()
        self._started = False
        self._log_info("[DualCarTcp] bridge stopped")

    def send(self, payload):
        """Encode *payload* (a dict) as a JSON line and try to send it.

        Non-blocking.  Returns True if the message was handed to the OS
        (sendall completed), False if it was dropped (bridge stopped,
        not connected, or serialisation/socket error).
        """
        if not self._started:
            return False

        # --- serialise ---------------------------------------------------
        try:
            json_str = json.dumps(payload, ensure_ascii=False)
            data = (json_str + "\n").encode("utf-8")
        except (TypeError, ValueError, UnicodeError) as exc:
            self._log_warn("[DualCarTcp] JSON serialise failed: %s", exc)
            return False

        # --- send --------------------------------------------------------
        with self._client_lock:
            sock = self._client_sock
        if sock is None:
            return False

        try:
            sock.sendall(data)
            return True
        except (socket.error, IOError, OSError) as exc:
            self._log_warn(
                "[DualCarTcp] send failed, closing client socket: %s", exc,
            )
            self._close_client_sock()
            return False

    def is_connected(self):
        """Return True if the client socket is currently connected to peer."""
        if not self._started:
            return False
        with self._client_lock:
            return self._client_sock is not None

    def is_started(self):
        """Return True if start() has been called and stop() has not."""
        return self._started

    # ---- server (receive) -----------------------------------------------

    def _server_loop(self):
        """Background thread: accept connections and read JSON lines.

        When the server socket breaks (e.g. port in use) the thread waits
        *reconnect_interval* seconds and retries, so the bridge tolerates
        temporary network issues without manual intervention.
        """
        while not self._stop_event.is_set():
            # --- create server socket ------------------------------------
            try:
                self._server_sock = socket.socket(
                    socket.AF_INET, socket.SOCK_STREAM,
                )
                self._server_sock.setsockopt(
                    socket.SOL_SOCKET, socket.SO_REUSEADDR, 1,
                )
                self._server_sock.bind(
                    (self._listen_ip, self._listen_port),
                )
                self._server_sock.listen(1)
                self._server_sock.settimeout(1.0)  # wake every second
                self._log_info(
                    "[DualCarTcp] server listening on %s:%d",
                    self._listen_ip, self._listen_port,
                )
            except (socket.error, IOError, OSError) as exc:
                self._log_warn(
                    "[DualCarTcp] server bind failed: %s, retrying in %.1fs",
                    exc, self._reconnect_interval,
                )
                self._close_server_sock()
                self._stop_event.wait(self._reconnect_interval)
                continue

            # --- accept loop ---------------------------------------------
            while not self._stop_event.is_set():
                try:
                    conn, addr = self._server_sock.accept()
                except socket.timeout:
                    continue
                except (socket.error, IOError, OSError) as exc:
                    if not self._stop_event.is_set():
                        self._log_warn(
                            "[DualCarTcp] server accept error: %s", exc,
                        )
                    break  # recreate server socket

                self._log_info(
                    "[DualCarTcp] accepted connection from %s:%d",
                    addr[0], addr[1],
                )
                self._handle_connection(conn)

            self._close_server_sock()

    def _handle_connection(self, conn):
        """Read newline-delimited JSON from *conn* until it closes."""
        buf = b""
        try:
            conn.settimeout(1.0)
        except Exception:
            pass

        while not self._stop_event.is_set():
            try:
                data = conn.recv(4096)
            except socket.timeout:
                continue
            except (socket.error, IOError, OSError) as exc:
                if not self._stop_event.is_set():
                    self._log_warn(
                        "[DualCarTcp] recv error: %s", exc,
                    )
                break

            if not data:
                # Peer closed the connection gracefully.
                break

            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._process_line(line)

        try:
            conn.close()
        except Exception:
            pass

    def _process_line(self, raw_line):
        """Decode one received line and dispatch to the callback."""
        line = raw_line.strip()
        if not line:
            return

        # --- decode ------------------------------------------------------
        try:
            text = line.decode("utf-8")
        except UnicodeDecodeError as exc:
            self._log_warn("[DualCarTcp] decode error: %s", exc)
            return

        # --- parse JSON -------------------------------------------------
        try:
            payload = json.loads(text)
        except ValueError as exc:
            self._log_warn(
                "[DualCarTcp] JSON parse error: %s  raw=%s", exc,
                text[:200],
            )
            return

        if not isinstance(payload, dict):
            self._log_warn(
                "[DualCarTcp] message is not a dict: %s",
                type(payload).__name__,
            )
            return

        # --- dispatch ----------------------------------------------------
        try:
            self._on_message(payload)
        except Exception as exc:
            self._log_warn(
                "[DualCarTcp] on_message callback error: %s", exc,
            )

    # ---- client (send) --------------------------------------------------

    def _client_loop(self):
        """Background thread: keep a connection to the peer for sending.

        When the socket breaks, the thread reconnects automatically.
        """
        while not self._stop_event.is_set():
            with self._client_lock:
                if self._client_sock is not None:
                    # Already connected — sleep and check again.
                    self._stop_event.wait(2.0)
                    continue

            # --- try to connect ------------------------------------------
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(self._connect_timeout)
                s.connect((self._peer_ip, self._peer_port))
                s.settimeout(None)  # sendall is blocking, which is fine
                with self._client_lock:
                    self._client_sock = s
                self._log_info(
                    "[DualCarTcp] connected to peer %s:%d",
                    self._peer_ip, self._peer_port,
                )
            except (socket.error, IOError, OSError) as exc:
                self._log_warn(
                    "[DualCarTcp] connect to %s:%d failed: %s, "
                    "retrying in %.1fs",
                    self._peer_ip, self._peer_port, exc,
                    self._reconnect_interval,
                )
                self._stop_event.wait(self._reconnect_interval)

    # ---- internal helpers -----------------------------------------------

    def _close_server_sock(self):
        if self._server_sock is None:
            return
        try:
            self._server_sock.close()
        except Exception:
            pass
        self._server_sock = None

    def _close_client_sock(self):
        with self._client_lock:
            if self._client_sock is None:
                return
            try:
                self._client_sock.close()
            except Exception:
                pass
            self._client_sock = None


# ---- lightweight self-test (run directly with python) --------------------


def _self_test_on_message(payload):
    """Simple callback that prints every received message."""
    import sys
    sys.stdout.write("[TEST] received: %s\n" % json.dumps(payload))
    sys.stdout.flush()


def _self_test():
    """Minimal self-test — starts a bridge in server-only mode.

    Usage::

        # Terminal 1 — run the test bridge (listens, echoes nothing)
        python -m pharmacy_mplus0.dual_car_tcp

        # Terminal 2 — send a test message via netcat
        echo '{"type":"allow_start","from_car":"1","target_car":"2"}' \\
          | nc 127.0.0.1 9001

    The bridge should print the received message and wait for more.
    Press Ctrl-C to stop.
    """
    import sys

    bridge = DualCarTcpBridge(
        car_id="1",
        peer_id="2",
        listen_ip="127.0.0.1",
        listen_port=9001,
        peer_ip="127.0.0.1",
        peer_port=9002,
        on_message=_self_test_on_message,
        logger=sys.modules[__name__],
    )

    # Give the module itself logger-like methods so the test is visible.
    def _info(msg, *args):
        sys.stdout.write(
            ("[INFO] " + msg % args + "\n") if args else ("[INFO] " + msg + "\n")
        )
        sys.stdout.flush()

    def _warn(msg, *args):
        sys.stderr.write(
            ("[WARN] " + msg % args + "\n") if args else ("[WARN] " + msg + "\n")
        )
        sys.stderr.flush()

    def _error(msg, *args):
        sys.stderr.write(
            ("[ERROR] " + msg % args + "\n") if args else ("[ERROR] " + msg + "\n")
        )
        sys.stderr.flush()

    bridge._log_info = _info
    bridge._log_warn = _warn
    bridge._log_error = _error

    bridge.start()
    sys.stdout.write("[TEST] bridge started, press Ctrl-C to stop\n")
    sys.stdout.flush()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sys.stdout.write("\n[TEST] stopping...\n")
        sys.stdout.flush()
    finally:
        bridge.stop()


if __name__ == "__main__":
    _self_test()
