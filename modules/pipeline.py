"""
pipeline - Ties all modules together into the main tracking loop.

Two-stage AUX arming:
  AUX1 (arm)   -> starts camera, systems ready, pilot keeps full control
  AUX2 (track) -> starts tracker, Pi sends MSP_SET_RAW_RC (overrides control)

When either AUX goes low, Pi stops sending RC -> real receiver takes over.
Requires Betaflight MSP Override feature + a real RC receiver on the FC.
"""

import time
import logging
from dataclasses import dataclass, field

import cv2
import serial

from .camera import CameraCapture
from .tracker import ObjectTracker, TrackResult, TrackerType
from .controller import FlightController, PIDGains
from .msp import MSPConnection
from .gcs import GCSLink, TelemetryPacket

logger = logging.getLogger(__name__)


# Pipeline states
IDLE = 0       # monitoring AUX, not sending RC, pilot has full control
ARMED = 1      # camera running, systems ready, not sending RC
TRACKING = 2   # tracker active, sending MSP_SET_RAW_RC


@dataclass
class PipelineConfig:
    # Camera
    camera_source: int = 0
    frame_width: int = 640
    frame_height: int = 480
    camera_fps: int = 30

    # Tracker
    tracker_type: str = TrackerType.CSRT

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

    # Two-stage AUX arming (0-indexed channel numbers)
    arm_aux_ch: int = 4          # AUX1 — arm pipeline
    track_aux_ch: int = 5        # AUX2 — start tracking
    aux_threshold: int = 1500    # above this = switch active
    track_bbox_size: int = 100   # fixed bbox size in pixels

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
    """Two-stage armed tracking pipeline.

    States:
        IDLE     — monitoring AUX channels, not sending RC, pilot flies normally
        ARMED    — camera running, systems ready, still not sending RC
        TRACKING — tracker active, Pi sends MSP_SET_RAW_RC via MSP Override

    When Pi stops sending RC (TRACKING -> ARMED or IDLE), the real RC
    receiver takes over immediately. No failsafe.
    """

    def __init__(self, config):
        self.cfg = config

        # Modules (created but not started yet)
        self.camera = CameraCapture(config.camera_source, config.frame_width,
                                    config.frame_height, config.camera_fps)
        self.tracker = ObjectTracker(config.tracker_type)
        self.controller = FlightController(config)
        self.msp = MSPConnection(config.serial_port, config.baudrate)
        self.gcs = None
        if config.gcs_enabled:
            self.gcs = GCSLink(config.gcs_host, config.gcs_port,
                               config.gcs_listen_port, config.gcs_send_hz)

        self._state = IDLE
        self._running = False
        self._camera_started = False
        self._loop_fps = 0.0

    @property
    def state_name(self):
        return {IDLE: "IDLE", ARMED: "ARMED", TRACKING: "TRACKING"}[self._state]

    # ── lifecycle ─────────────────────────────────────────────────

    def start(self):
        """Connect MSP (and GCS). Camera starts when pipeline is armed."""
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
        self._transition_to(IDLE)
        self.msp.disconnect()
        if self.gcs:
            self.gcs.disconnect()
        if self.cfg.show_preview:
            cv2.destroyAllWindows()
        logger.info("Pipeline stopped")

    # ── state transitions ─────────────────────────────────────────

    def _transition_to(self, new_state):
        old = self._state
        if new_state == old:
            return

        # Leaving TRACKING -> stop sending RC, real receiver takes over
        if old == TRACKING:
            self.tracker.reset()
            self.controller.reset()
            logger.info("TRACKING -> %s: RC override stopped, pilot has control",
                        {ARMED: "ARMED", IDLE: "IDLE"}[new_state])

        # Leaving ARMED -> stop camera
        if old >= ARMED and new_state == IDLE:
            if self._camera_started:
                self.camera.stop()
                self._camera_started = False
            logger.info("ARMED -> IDLE: camera stopped")

        # Entering ARMED -> start camera
        if new_state >= ARMED and not self._camera_started:
            self.camera.start()
            self._camera_started = True
            logger.info("IDLE -> ARMED: camera started, ready to track")

        # Entering TRACKING -> init tracker with centered bbox
        if new_state == TRACKING:
            frame = self.camera.read()
            if frame is not None:
                s = self.cfg.track_bbox_size
                x = self.cfg.frame_width // 2 - s // 2
                y = self.cfg.frame_height // 2 - s // 2
                self.tracker.init(frame, (x, y, s, s))
                logger.info("ARMED -> TRACKING: tracker started (%dx%d at center)", s, s)
            else:
                logger.warning("No frame available, cannot start tracking")
                return  # stay in ARMED

        self._state = new_state

    def _poll_aux_state(self):
        """Read AUX channels from FC and update pipeline state."""
        rc = self.msp.get_rc_channels()
        if rc is None:
            return

        min_ch = max(self.cfg.arm_aux_ch, self.cfg.track_aux_ch)
        if len(rc) <= min_ch:
            return

        arm_active = rc[self.cfg.arm_aux_ch] > self.cfg.aux_threshold
        track_active = rc[self.cfg.track_aux_ch] > self.cfg.aux_threshold

        if not arm_active:
            # AUX1 off -> everything off
            self._transition_to(IDLE)
        elif arm_active and track_active:
            # Both on -> tracking (arm first if needed)
            if self._state == IDLE:
                self._transition_to(ARMED)
            self._transition_to(TRACKING)
        elif arm_active and not track_active:
            # Only AUX1 -> armed but not tracking
            self._transition_to(ARMED)

    # ── main loop ─────────────────────────────────────────────────

    def run(self):
        """Blocking control loop. Ctrl-C or stop() to exit."""
        self._running = True
        period = 1.0 / self.cfg.loop_hz
        logger.info("Pipeline running, waiting for AUX arm signal...")

        try:
            while self._running:
                t_start = time.monotonic()

                # Always poll AUX state
                self._poll_aux_state()

                if self._state == IDLE:
                    # Nothing to do, just wait
                    time.sleep(period)
                    continue

                # ARMED or TRACKING — camera is running
                frame = self.camera.read()
                if frame is None:
                    time.sleep(0.001)
                    continue

                if self._state == TRACKING:
                    # Track + control + send RC
                    result = self.tracker.update(frame)
                    self.controller.update(result)
                    self.msp.send_rc(self.controller.channels)
                else:
                    # ARMED — camera running but not sending RC
                    result = TrackResult()

                # GCS telemetry (send in both ARMED and TRACKING)
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
        # Show state
        cv2.putText(frame, self.state_name, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
        cv2.imshow("Drone Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self._running = False
