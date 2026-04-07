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
AI_ARMED = 1      # camera running, systems ready, not sending RC
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

    # AUX arming — single 3-position switch (0-indexed channel)
    aux_ch: int = 7              # AUX4 channel (3-position switch)
    aux_arm_threshold: int = 1300    # above this = AI_ARMED
    aux_track_threshold: int = 1700  # above this = TRACKING
    track_bbox_size: int = 100       # fixed bbox size in pixels

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
        AI_ARMED    — camera running, systems ready, still not sending RC
        TRACKING — tracker active, Pi sends MSP_SET_RAW_RC via MSP Override

    When Pi stops sending RC (TRACKING -> AI_ARMED or IDLE), the real RC
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
        self._msp_connected = False
        self._loop_fps = 0.0
        self._last_rc = None
        self._debug_counter = 0

    @property
    def state_name(self):
        return {IDLE: "IDLE", AI_ARMED: "AI-ARMED", TRACKING: "TRACKING"}[self._state]

    # ── lifecycle ─────────────────────────────────────────────────

    def start(self):
        """Connect MSP (and GCS). Camera starts when pipeline is armed."""
        try:
            self.msp.connect()
            self._msp_connected = True
        except serial.SerialException as e:
            logger.warning("MSP serial failed (%s); running in preview-only mode", e)
            self._msp_connected = False
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
                        {AI_ARMED: "AI-ARMED", IDLE: "IDLE"}[new_state])

        # Leaving AI_ARMED -> stop camera
        if old >= AI_ARMED and new_state == IDLE:
            if self._camera_started:
                self.camera.stop()
                self._camera_started = False
            logger.info("AI_ARMED -> IDLE: camera stopped")

        # Entering AI_ARMED -> start camera
        if new_state >= AI_ARMED and not self._camera_started:
            self.camera.start()
            self._camera_started = True
            logger.info("IDLE -> AI_ARMED: camera started, ready to track")

        # Entering TRACKING -> init tracker with centered bbox
        if new_state == TRACKING:
            frame = self.camera.read()
            if frame is not None:
                s = self.cfg.track_bbox_size
                x = self.cfg.frame_width // 2 - s // 2
                y = self.cfg.frame_height // 2 - s // 2
                self.tracker.init(frame, (x, y, s, s))
                logger.info("AI_ARMED -> TRACKING: tracker started (%dx%d at center)", s, s)
            else:
                logger.warning("No frame available, cannot start tracking")
                return  # stay in AI_ARMED

        self._state = new_state

    def _poll_aux_state(self):
        """Read 3-position AUX switch from FC and update pipeline state.

        Low  (~1000) = IDLE
        Mid  (~1500) = AI_ARMED
        High (~2000) = TRACKING
        """
        rc = self.msp.get_rc_channels()
        if rc is None:
            return

        self._last_rc = rc

        if len(rc) <= self.cfg.aux_ch:
            return

        val = rc[self.cfg.aux_ch]

        if val > self.cfg.aux_track_threshold:
            if self._state == IDLE:
                self._transition_to(AI_ARMED)
            self._transition_to(TRACKING)
        elif val > self.cfg.aux_arm_threshold:
            self._transition_to(AI_ARMED)
        else:
            self._transition_to(IDLE)

    # ── debug output ──────────────────────────────────────────────

    def _debug_print(self, result):
        """Print a status line every ~1 second (every loop_hz frames)."""
        self._debug_counter += 1
        if self._debug_counter < self.cfg.loop_hz:
            return
        self._debug_counter = 0

        rc = self._last_rc
        aux_val = rc[self.cfg.aux_ch] if rc and len(rc) > self.cfg.aux_ch else 0

        # RC sticks: roll, pitch, throttle, yaw
        if rc and len(rc) >= 4:
            sticks = "R:%d P:%d T:%d Y:%d" % (rc[0], rc[1], rc[2], rc[3])
        else:
            sticks = "no RC data"

        ctrl = self.controller
        line = "[%s] AUX4:%d | %s" % (self.state_name, aux_val, sticks)

        if self._state == TRACKING:
            line += " | found=%s yaw_err=%.1f fwd_err=%.1f ch=[%s]" % (
                result.found,
                ctrl.yaw_error,
                ctrl.forward_error,
                ",".join(str(c) for c in ctrl.channels),
            )

        line += " | %.1f fps" % self._loop_fps
        print(line)

    # ── main loop ─────────────────────────────────────────────────

    def run(self):
        """Blocking control loop. Ctrl-C or stop() to exit."""
        self._running = True
        period = 1.0 / self.cfg.loop_hz

        # No MSP -> preview-only mode (for testing with video files)
        if not self._msp_connected:
            if not self._camera_started:
                self.camera.start()
                self._camera_started = True
            self.cfg.show_preview = True
            logger.info("Preview-only mode (no MSP). Press 'q' to quit.")

        else:
            logger.info("Pipeline running, waiting for AUX arm signal...")

        try:
            while self._running:
                t_start = time.monotonic()
                result = TrackResult()
                frame = None

                # Poll AUX state (only if MSP connected)
                if self._msp_connected:
                    self._poll_aux_state()

                # Get frame if camera is running
                if self._camera_started:
                    frame = self.camera.read()

                if self._state == TRACKING and frame is not None:
                    # Track + control + send RC
                    result = self.tracker.update(frame)
                    self.controller.update(result)
                    self.msp.send_rc(self.controller.channels)

                # GCS telemetry
                if self.gcs and self._state >= AI_ARMED:
                    self._send_telemetry(result)
                    self._handle_gcs_commands()

                # Preview — always show when enabled and we have a frame
                if self.cfg.show_preview and frame is not None:
                    self._draw_preview(frame, result)

                # Debug
                self._debug_print(result)

                # Timing — enforce loop_hz
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

        # Frame center crosshair
        cv2.drawMarker(frame, (cx, cy), (0, 255, 0),
                       cv2.MARKER_CROSS, 20, 2)

        # Init bbox region (where tracker would start)
        s = self.cfg.track_bbox_size
        ix = cx - s // 2
        iy = cy - s // 2
        if self._state == AI_ARMED:
            cv2.rectangle(frame, (ix, iy), (ix + s, iy + s), (0, 255, 255), 1)
            cv2.putText(frame, "init zone", (ix, iy - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

        # Tracked target
        if result.found:
            x, y, w, h = result.bbox
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (int(result.center[0]), int(result.center[1])),
                       5, (0, 0, 255), -1)
            # Line from center to target
            cv2.line(frame, (cx, cy),
                     (int(result.center[0]), int(result.center[1])),
                     (0, 0, 255), 1)
            # Target width vs desired
            cv2.putText(frame, "w:%d / %d" % (w, int(self.cfg.desired_target_width)),
                        (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        elif self._state == TRACKING:
            cv2.putText(frame, "TARGET LOST", (cx - 60, cy + 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # State + info overlay
        ctrl = self.controller
        color = {IDLE: (128, 128, 128), AI_ARMED: (0, 255, 255),
                 TRACKING: (0, 255, 0)}[self._state]
        cv2.putText(frame, self.state_name, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(frame, "%.0f fps" % self._loop_fps, (10, 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        if self._state == TRACKING:
            cv2.putText(frame, "yaw: %.1f  fwd: %.1f" % (ctrl.yaw_error, ctrl.forward_error),
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            cv2.putText(frame, "yaw_out: %.0f  fwd_out: %.0f" % (ctrl.yaw_output, ctrl.forward_output),
                        (10, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        cv2.imshow("Drone Tracker", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self._running = False
