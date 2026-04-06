"""
pipeline - Ties all modules together into the main tracking loop.
"""

import time
import logging
from dataclasses import dataclass, field

import cv2
import serial

from .camera import CameraCapture
from .tracker import ObjectTracker, TrackerType
from .controller import FlightController, PIDGains
from .msp import MSPConnection
from .gcs import GCSLink, TelemetryPacket

logger = logging.getLogger(__name__)


@dataclass
class PipelineConfig:
    # Camera
    camera_source: int = 0
    frame_width: int = 640
    frame_height: int = 480
    camera_fps: int = 30

    # Tracker
    tracker_type: str = TrackerType.CSRT
    hsv_lower: tuple = (0, 100, 100)
    hsv_upper: tuple = (10, 255, 255)

    # PID — yaw (direction) and forward (pitch for approach/retreat)
    yaw_pid: PIDGains = field(default_factory=lambda: PIDGains(
        kp=0.8, ki=0.05, kd=0.15, output_min=-300, output_max=300))
    forward_pid: PIDGains = field(default_factory=lambda: PIDGains(
        kp=0.4, ki=0.02, kd=0.1, output_min=-250, output_max=250))

    # Forward control: desired target bbox width (pixels) at ideal distance
    desired_target_width: float = 120.0

    # MSP
    serial_port: str = "/dev/ttyAMA0"
    baudrate: int = 115200

    # RC channel mapping (0-indexed)
    roll_ch: int = 0
    pitch_ch: int = 1
    throttle_ch: int = 2
    yaw_ch: int = 3
    num_channels: int = 8

    # Neutral RC values
    rc_center: int = 1500
    throttle_neutral: int = 1500

    # Roll limit (Betaflight angle mode)
    roll_limit_deg: float = 20.0
    max_angle_deg: float = 35.0

    # Rate limiting — max RC units change per second
    max_rc_rate: float = 200.0

    # GCS
    gcs_host: str = "192.168.1.100"
    gcs_port: int = 14550
    gcs_listen_port: int = 14551
    gcs_send_hz: float = 10.0
    gcs_enabled: bool = True

    # Control loop
    loop_hz: int = 30
    show_preview: bool = False

    # Deadbands (pixels)
    deadband: float = 10.0
    size_deadband: float = 15.0


class TrackingPipeline:
    """Full pipeline: Camera -> Tracker -> Controller -> MSP + GCS.

    Quick start::

        pipeline = TrackingPipeline(PipelineConfig())
        pipeline.start()
        pipeline.init_target()
        pipeline.run()
    """

    def __init__(self, config):
        self.cfg = config

        # Modules
        self.camera = CameraCapture(config.camera_source, config.frame_width,
                                    config.frame_height, config.camera_fps)
        self.tracker = ObjectTracker(config.tracker_type, config.hsv_lower,
                                     config.hsv_upper)
        self.controller = FlightController(config)
        self.msp = MSPConnection(config.serial_port, config.baudrate)
        self.gcs = None
        if config.gcs_enabled:
            self.gcs = GCSLink(config.gcs_host, config.gcs_port,
                               config.gcs_listen_port, config.gcs_send_hz)

        self._running = False
        self._loop_fps = 0.0

    # ── lifecycle ─────────────────────────────────────────────────

    def start(self):
        self.camera.start()
        try:
            self.msp.connect()
        except serial.SerialException as e:
            logger.warning("MSP serial failed (%s); running in dry-run mode", e)
        if self.gcs:
            try:
                self.gcs.connect()
            except OSError as e:
                logger.warning("GCS link failed (%s); telemetry disabled", e)
                self.gcs = None

    def stop(self):
        self._running = False
        self.controller.reset()
        self.msp.send_rc(self.controller.channels)
        time.sleep(0.05)
        self.camera.stop()
        self.msp.disconnect()
        if self.gcs:
            self.gcs.disconnect()
        if self.cfg.show_preview:
            cv2.destroyAllWindows()
        logger.info("Pipeline stopped")

    # ── target initialization ─────────────────────────────────────

    def init_target(self, bbox=None):
        """Initialize the tracker. If bbox is None, opens interactive ROI."""
        frame = None
        for _ in range(30):
            frame = self.camera.read()
            if frame is not None:
                break
            time.sleep(0.05)
        if frame is None:
            raise RuntimeError("No frame available from camera")

        if bbox is None:
            bbox = self.tracker.select_roi(frame)
        self.tracker.init(frame, bbox)

    # ── main loop ─────────────────────────────────────────────────

    def run(self):
        """Blocking control loop. Ctrl-C or stop() to exit."""
        self._running = True
        period = 1.0 / self.cfg.loop_hz
        logger.info("Control loop running @ %d Hz", self.cfg.loop_hz)

        try:
            while self._running:
                t_start = time.monotonic()

                frame = self.camera.read()
                if frame is None:
                    time.sleep(0.001)
                    continue

                # Track
                result = self.tracker.update(frame)

                # Control
                self.controller.update(result)
                self.msp.send_rc(self.controller.channels)

                # GCS telemetry
                if self.gcs:
                    self._send_telemetry(result)
                    self._handle_gcs_commands()

                # Preview
                if self.cfg.show_preview:
                    self._draw_preview(frame, result)

                # Timing
                elapsed = time.monotonic() - t_start
                self._loop_fps = 1.0 / elapsed if elapsed > 0 else 0.0
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            logger.info("Interrupted")
        finally:
            self.stop()

    # ── GCS helpers ───────────────────────────────────────────────

    def _send_telemetry(self, result):
        ctrl = self.controller
        attitude = self.msp.get_attitude()
        packet = TelemetryPacket(
            timestamp=time.time(),
            target_found=result.found,
            target_bbox=result.bbox,
            target_center=result.center,
            channels=ctrl.channels,
            yaw_error=ctrl.yaw_error,
            forward_error=ctrl.forward_error,
            yaw_output=ctrl.yaw_output,
            forward_output=ctrl.forward_output,
            roll=attitude["roll"] if attitude else 0.0,
            pitch=attitude["pitch"] if attitude else 0.0,
            yaw=attitude["yaw"] if attitude else 0.0,
            attitude_valid=attitude is not None,
            loop_fps=self._loop_fps,
        )
        self.gcs.send_telemetry(packet)

    def _handle_gcs_commands(self):
        while True:
            cmd = self.gcs.recv_command()
            if cmd is None:
                break
            if cmd.command == "stop":
                logger.info("GCS commanded stop")
                self._running = False
            elif cmd.command == "set_target_width":
                w = cmd.params.get("width")
                if w is not None:
                    self.controller.cfg.desired_target_width = float(w)
                    logger.info("GCS set desired_target_width=%.1f", w)
            elif cmd.command == "ping":
                logger.debug("GCS ping received")

    # ── preview ───────────────────────────────────────────────────

    def _draw_preview(self, frame, result):
        cx = int(self.cfg.frame_width / 2)
        cy = int(self.cfg.frame_height / 2)
        cv2.drawMarker(frame, (cx, cy), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)
        if result.found:
            x, y, w, h = result.bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (int(result.center[0]), int(result.center[1])),
                       5, (0, 0, 255), -1)
        cv2.imshow("Drone Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self._running = False
