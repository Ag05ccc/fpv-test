"""
gcs - Ground Control Station communication over UDP.

Sends telemetry packets to a GCS and receives commands back.
"""
import json
import socket
import threading
import time
import logging
from dataclasses import dataclass, field, asdict

logger = logging.getLogger(__name__)


# ── Telemetry ────────────────────────────────────────────────────────

@dataclass
class TelemetryPacket:
    """Snapshot of drone state sent to GCS each cycle."""
    timestamp: float = 0.0

    # Tracking
    target_found: bool = False
    target_bbox: tuple = (0, 0, 0, 0)
    target_center: tuple = (0.0, 0.0)

    # Control
    channels: list = field(default_factory=list)
    yaw_error: float = 0.0
    forward_error: float = 0.0
    yaw_output: float = 0.0
    forward_output: float = 0.0

    # Attitude (from FC, if available)
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    attitude_valid: bool = False

    # Performance
    loop_fps: float = 0.0

    def to_json(self):
        return json.dumps(asdict(self)).encode("utf-8")


# ── GCS Commands ─────────────────────────────────────────────────────

@dataclass
class GCSCommand:
    """Command received from the GCS."""
    command: str = ""         # "set_pid", "set_target_width", "stop", "ping"
    params: dict = field(default_factory=dict)


# ── GCS Link ─────────────────────────────────────────────────────────

class GCSLink:
    """Bidirectional UDP link to a Ground Control Station.

    Telemetry (drone → GCS):
        Call ``send_telemetry(packet)`` from the control loop.

    Commands (GCS → drone):
        Call ``recv_command()`` to pop the next command, or ``None``.
        Commands are received on a background thread.

    Wire format: UTF-8 JSON, one object per datagram.
    """

    def __init__(self, gcs_host="192.168.1.100", gcs_port=14550,
                 listen_port=14551, send_hz=10.0):
        self.gcs_addr = (gcs_host, gcs_port)
        self.listen_port = listen_port
        self.send_hz = send_hz

        self._tx_sock = None
        self._rx_sock = None
        self._cmd_queue = []
        self._cmd_lock = threading.Lock()
        self._running = False
        self._rx_thread = None

        # Rate-limit telemetry sends
        self._min_send_interval = 1.0 / send_hz
        self._last_send_time = 0.0

    def connect(self):
        """Open TX and RX sockets and start the command listener."""
        self._tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._rx_sock.bind(("0.0.0.0", self.listen_port))
        self._rx_sock.settimeout(0.5)

        self._running = True
        self._rx_thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._rx_thread.start()
        logger.info("GCS link up: send→%s:%d  listen←:%d",
                     self.gcs_addr[0], self.gcs_addr[1], self.listen_port)

    def disconnect(self):
        self._running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        if self._tx_sock:
            self._tx_sock.close()
        if self._rx_sock:
            self._rx_sock.close()
        logger.info("GCS link closed")

    # ── Telemetry (outbound) ──────────────────────────────────────

    def send_telemetry(self, packet):
        """Send a telemetry packet if enough time has elapsed since the last send."""
        now = time.monotonic()
        if now - self._last_send_time < self._min_send_interval:
            return
        self._last_send_time = now

        if not self._tx_sock:
            return
        try:
            self._tx_sock.sendto(packet.to_json(), self.gcs_addr)
        except OSError as e:
            logger.debug("GCS send failed: %s", e)

    # ── Commands (inbound) ────────────────────────────────────────

    def recv_command(self):
        """Pop the next command from the queue, or None."""
        with self._cmd_lock:
            return self._cmd_queue.pop(0) if self._cmd_queue else None

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self._rx_sock.recvfrom(4096)
                obj = json.loads(data.decode("utf-8"))
                cmd = GCSCommand(
                    command=obj.get("command", ""),
                    params=obj.get("params", {}),
                )
                with self._cmd_lock:
                    self._cmd_queue.append(cmd)
                logger.debug("GCS command from %s: %s", addr, cmd.command)
            except socket.timeout:
                continue
            except (json.JSONDecodeError, OSError) as e:
                logger.debug("GCS recv error: %s", e)
