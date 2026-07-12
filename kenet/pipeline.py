"""
pipeline - Ties all modules together into the main tracking loop.

Single-switch AUX arming:
  Low  -> IDLE, pilot keeps full control
  Mid  -> AI_ARMED, camera/pipeline ready, pilot keeps full control
  High -> TRACKING, tracker starts, Pi sends MSP_SET_RAW_RC

When AUX drops below tracking, Pi stops sending RC -> real receiver takes over.
Requires Betaflight MSP Override feature + a real RC receiver on the FC.
"""

import time
import logging
from dataclasses import dataclass, field

import cv2
import serial

from .camera import create_camera_capture
from .tracker import ObjectTracker, TrackResult, TrackerType
from .controller import FlightController, PIDGains
from .msp import MSPConnection
from .rc_channels import msp_rc_to_pilot_channels
from .gcs import GCSLink, TelemetryPacket
from .state_machine import (
    AI_ARMED,
    DEFAULT_AUX_ARM_THRESHOLD,
    DEFAULT_AUX_TRACK_THRESHOLD,
    DEFAULT_KENET_AUX_CH,
    IDLE,
    STATE_NAMES,
    TRACKING,
    state_from_aux,
)

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
    aux_ch: int = DEFAULT_KENET_AUX_CH  # CH6 / AUX2 3-position switch
    aux_arm_threshold: int = DEFAULT_AUX_ARM_THRESHOLD
    aux_track_threshold: int = DEFAULT_AUX_TRACK_THRESHOLD
    aux_poll_hz: float = 15.0        # MSP_RC polling rate
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
    """Tracking pipeline. Camera and preview always run.
    AUX switch only controls the tracker:

        IDLE     — monitoring AUX, not sending RC, pilot flies normally
        AI-ARMED — ready to track, still not sending RC
        TRACKING — tracker active, Pi sends MSP_SET_RAW_RC via MSP Override

    When Pi stops sending RC, the real receiver takes over without relying on a
    failsafe handoff.
    """

    def __init__(self, config):
        self.cfg = config

        # Modules (created but not started yet)
        self.camera = create_camera_capture(
            config.camera_source, config.frame_width,
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
        self._msp_connected = False
        self._loop_fps = 0.0
        self._last_rc = None
        self._last_override_channels = None
        self._debug_counter = 0
        self._lost_count = 0
        self._tracking_reentry_blocked = False
        self._last_aux_poll_time = 0.0
        self._preview_available = True
        self._warned_missing_pilot_rc_for_override = False

    @property
    def state_name(self):
        return STATE_NAMES[self._state]

    # ── lifecycle ─────────────────────────────────────────────────

    def start(self):
        """Start camera, connect MSP and GCS."""
        self.camera.start()
        try:
            self.msp.connect()
            self._msp_connected = True
        except (serial.SerialException, OSError) as e:
            logger.warning("MSP connection failed (%s); running in preview-only mode", e)
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
        self.camera.stop()
        self.msp.disconnect()
        if self.gcs:
            self.gcs.disconnect()
        if self.cfg.show_preview:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        logger.info("Pipeline stopped")

    # ── state transitions ─────────────────────────────────────────

    def _transition_to(self, new_state, force=False):
        old = self._state
        if new_state == old and not force:
            return

        # Leaving TRACKING -> stop sending RC, real receiver takes over
        if old == TRACKING:
            self.tracker.reset()
            self.controller.reset()
            self._last_override_channels = None
            logger.info("TRACKING -> %s: RC override stopped, pilot has control",
                        {AI_ARMED: "AI-ARMED", IDLE: "IDLE"}.get(new_state, "?"))

        if old == IDLE and new_state == AI_ARMED:
            logger.info("IDLE -> AI-ARMED: ready to track")

        if old == AI_ARMED and new_state == IDLE:
            logger.info("AI-ARMED -> IDLE")

        # Entering TRACKING -> init tracker with centered bbox
        if new_state == TRACKING:
            frame = None
            for _ in range(30):
                frame = self.camera.read()
                if frame is not None:
                    break
                time.sleep(0.05)
            if frame is not None:
                h_frame, w_frame = frame.shape[:2]
                s = self.cfg.track_bbox_size
                x = w_frame // 2 - s // 2
                y = h_frame // 2 - s // 2
                self.tracker.init(frame, (x, y, s, s))
                # Update controller center to match actual frame
                self.controller.set_frame_center(w_frame, h_frame)
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
        msp_rc = self.msp.get_rc_channels()
        if msp_rc is None:
            return

        rc = msp_rc_to_pilot_channels(msp_rc)
        self._last_rc = rc

        if len(rc) <= self.cfg.aux_ch:
            return

        val = rc[self.cfg.aux_ch]

        if val <= self.cfg.aux_track_threshold:
            self._tracking_reentry_blocked = False

        new_state = state_from_aux(
            val,
            self.cfg.aux_arm_threshold,
            self.cfg.aux_track_threshold,
            tracking_inhibited=self._tracking_reentry_blocked,
        )

        if new_state == TRACKING:
            if self._state == IDLE:
                self._transition_to(AI_ARMED)
            self._transition_to(TRACKING)
        elif new_state == AI_ARMED:
            self._transition_to(AI_ARMED)
        else:
            self._transition_to(IDLE)

    def _aux_poll_due(self, now=None):
        if self.cfg.aux_poll_hz <= 0:
            return True
        now = time.monotonic() if now is None else now
        if now - self._last_aux_poll_time < 1.0 / self.cfg.aux_poll_hz:
            return False
        self._last_aux_poll_time = now
        return True

    def _build_override_channels(self):
        """Merge Kenet pitch/yaw into the latest pilot RC frame.

        MSP_SET_RAW_RC carries a complete channel frame. The production safety
        contract is still pitch/yaw only, so roll, throttle, and AUX channels
        must come from the current pilot/receiver frame instead of the
        controller's neutral defaults.
        """
        if not self._last_rc or len(self._last_rc) < self.cfg.num_channels:
            if not self._warned_missing_pilot_rc_for_override:
                logger.warning("Skipping MSP override: no complete pilot RC frame")
                self._warned_missing_pilot_rc_for_override = True
            return None

        channels = list(self._last_rc[:self.cfg.num_channels])
        ai_channels = self.controller.channels
        for index in (self.cfg.pitch_ch, self.cfg.yaw_ch):
            channels[index] = ai_channels[index]
        self._last_override_channels = channels
        self._warned_missing_pilot_rc_for_override = False
        return channels

    def _track_control_step(self, frame):
        """Run one TRACKING control step and send MSP only with a found target."""
        result = self.tracker.update(frame)

        if not result.found:
            self.controller.reset()
            self._last_override_channels = None
            self._lost_count += 1
            if self._lost_count > self.cfg.loop_hz * 2:  # ~2 seconds
                logger.info("Target lost for 2s, dropping to AI-ARMED")
                self._tracking_reentry_blocked = True
                self._transition_to(AI_ARMED)
                self._lost_count = 0
            return result

        self._lost_count = 0
        self.controller.update(result)
        override_channels = self._build_override_channels()
        if override_channels is not None:
            self.msp.send_rc(override_channels)
        return result

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
        line = "[%s] CH%d:%d | %s" % (
            self.state_name,
            self.cfg.aux_ch + 1,
            aux_val,
            sticks,
        )

        if self._state == TRACKING:
            line += " | found=%s yaw_err=%.1f fwd_err=%.1f ch=[%s]" % (
                result.found,
                ctrl.yaw_error,
                ctrl.forward_error,
                ",".join(str(c) for c in ctrl.channels),
            )
            if self._last_override_channels is not None:
                line += " send=[%s]" % ",".join(
                    str(c) for c in self._last_override_channels
                )

        line += " | %.1f fps" % self._loop_fps
        print(line)

    # ── main loop ─────────────────────────────────────────────────

    def run(self):
        """Blocking control loop. Ctrl-C or stop() to exit."""
        self._running = True
        period = 1.0 / self.cfg.loop_hz

        if not self._msp_connected:
            if self.cfg.show_preview:
                logger.info("Preview-only mode (no MSP). Press 'q' to quit.")
            else:
                logger.info("Headless mode without MSP. Press Ctrl-C to quit.")
        else:
            logger.info("Pipeline running, waiting for AUX signal...")

        try:
            while self._running:
                t_start = time.monotonic()
                result = TrackResult()
                frame = None

                if self.gcs:
                    self._handle_gcs_commands()
                    if not self._running:
                        break

                # Poll AUX state (only if MSP connected)
                if self._msp_connected and self._aux_poll_due(t_start):
                    self._poll_aux_state()

                # Always grab frames
                frame = self.camera.read()

                if self._state == TRACKING and frame is not None:
                    # Track + control + send RC
                    result = self._track_control_step(frame)

                # GCS telemetry
                if self.gcs and self._state >= AI_ARMED:
                    self._send_telemetry(result)

                # Preview
                if self.cfg.show_preview and self._preview_available:
                    if frame is not None:
                        self._draw_preview(frame, result)
                    else:
                        try:
                            cv2.waitKey(1)
                        except cv2.error as e:
                            self._disable_preview(e)

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
        if not self.gcs.ready_to_send():
            return False

        ctrl = self.controller
        attitude = self.msp.get_attitude() if self._msp_connected else None
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
        return self.gcs.send_telemetry(packet)

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
            elif cmd.command == "set_pid":
                axis = cmd.params.get("axis", "yaw")
                pid = {k: cmd.params.get(k) for k in ("kp", "ki", "kd")}
                controller = (self.controller.yaw_pid if axis == "yaw"
                              else self.controller.forward_pid)
                controller.set_gains(**pid)
                logger.info("GCS set_pid axis=%s %s", axis,
                            {k: v for k, v in pid.items() if v is not None})
            elif cmd.command == "ping":
                logger.debug("GCS ping received")

    # ── preview ───────────────────────────────────────────────────

    def _draw_preview(self, frame, result):
        h_frame, w_frame = frame.shape[:2]
        cx = w_frame // 2
        cy = h_frame // 2

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

        try:
            cv2.imshow("Drone Tracker", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self._running = False
        except cv2.error as e:
            self._disable_preview(e)

    def _disable_preview(self, error):
        self._preview_available = False
        logger.warning(
            "OpenCV preview disabled (%s). Continue in headless mode. "
            "Use --headless to suppress preview, or run with GUI-capable OpenCV.",
            error,
        )
