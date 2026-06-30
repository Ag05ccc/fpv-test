#!/usr/bin/env python3
"""
Pilot RC + Kenet visual tracking mixer for Betaflight SITL.

The mixer reads pilot RC from a Linux joystick, optionally runs Kenet tracking
on a camera/video source, and sends one merged rc_packet to Betaflight SITL:

  - IDLE / AI-ARMED: all channels come from the pilot
  - TRACKING: roll/throttle/AUX stay pilot, pitch/yaw can come from Kenet

This is a SITL test tool. It does not use MSP override.
"""

import argparse
import logging
import os
import socket
import sys
import time
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from sitl_rc_bridge import CHANNEL_MAP, LinuxJoystick, apply_forced_mode_pwm, make_channels, pack_rc_packet
from sitl_log import JsonlLogger, make_log_path, resolve_log_dir
from sitl_virtual_rc import make_virtual_channels

from kenet.camera import CameraCapture
from kenet.controller import FlightController, PIDGains
from kenet.pipeline import AI_ARMED, IDLE, TRACKING, PipelineConfig
from kenet.state_machine import STATE_NAMES, state_from_aux
from kenet.tracker import ObjectTracker, TrackerUnavailableError, TrackResult


logger = logging.getLogger("kenet_sitl_mixer")

ROLL_CH = 0
PITCH_CH = 1
THROTTLE_CH = 2
YAW_CH = 3
ARM_CH = 4
SAFE_CENTER_PWM = 1500
SAFE_LOW_PWM = 1000


def parse_camera_source(value):
    return int(value) if str(value).isdigit() else value


def centered_bbox(frame, size):
    h_frame, w_frame = frame.shape[:2]
    size = int(size)
    x = max(0, w_frame // 2 - size // 2)
    y = max(0, h_frame // 2 - size // 2)
    return (x, y, size, size)


def synthetic_track_result(args):
    loss_after = getattr(args, "synthetic_target_loss_after_seconds", None)
    if loss_after is not None and args.synthetic_elapsed_seconds >= loss_after:
        return TrackResult(found=False)
    if args.synthetic_target_delay_seconds > 0 and args.synthetic_target_delay_seconds > args.synthetic_elapsed_seconds:
        w = int(round(args.desired_target_width))
        h = int(round(args.desired_target_width))
        cx = float(args.frame_width) / 2.0
        cy = float(args.frame_height) / 2.0
        x = int(round(cx - w / 2.0))
        y = int(round(cy - h / 2.0))
        return TrackResult(found=True, bbox=(x, y, w, h), center=(cx, cy))
    w = int(args.synthetic_target_width)
    h = int(args.synthetic_target_height)
    cx = float(args.synthetic_target_x)
    cy = float(args.synthetic_target_y)
    x = int(round(cx - w / 2.0))
    y = int(round(cy - h / 2.0))
    return TrackResult(found=True, bbox=(x, y, w, h), center=(cx, cy))


class KenetSitlMixer:
    def __init__(self, args):
        self.args = args
        self.cfg = PipelineConfig(
            camera_source=parse_camera_source(args.camera),
            frame_width=args.frame_width,
            frame_height=args.frame_height,
            camera_fps=args.camera_fps,
            tracker_type=args.tracker,
            loop_hz=args.loop_hz,
            show_preview=args.preview,
            aux_ch=args.aux_ch,
            track_bbox_size=args.track_size,
            desired_target_width=args.desired_target_width,
            gcs_enabled=False,
            yaw_pid=PIDGains(
                kp=args.yaw_kp,
                ki=args.yaw_ki,
                kd=args.yaw_kd,
                output_min=-args.yaw_limit,
                output_max=args.yaw_limit,
            ),
            forward_pid=PIDGains(
                kp=args.forward_kp,
                ki=args.forward_ki,
                kd=args.forward_kd,
                output_min=-args.forward_limit,
                output_max=args.forward_limit,
            ),
        )

        self.joystick = LinuxJoystick(args.device) if args.pilot_source == "joystick" else None
        self.camera = None
        self.tracker = None
        self.controller = FlightController(self.cfg)
        self.sock = None

        self.state = IDLE
        self.prev_state = None
        self.virtual_started = None
        self.lost_count = 0
        self.loop_fps = 0.0
        self.last_print = 0.0
        self.preview_available = True
        self.last_result = TrackResult()
        self.last_source = "pilot"
        self.tracker_error = None
        self.flight_logger = None
        self.last_flight_log = 0.0
        self.last_send_time = None
        self.tx_warning_count = 0
        self.tracking_reentry_blocked = False

    def start(self):
        self.virtual_started = time.monotonic()
        if self.joystick is not None:
            self.joystick.open()
        if self.args.synthetic_target:
            self.controller.set_frame_center(self.args.frame_width, self.args.frame_height)
        elif not self.args.no_vision:
            self.camera = CameraCapture(
                self.cfg.camera_source,
                self.cfg.frame_width,
                self.cfg.frame_height,
                self.cfg.camera_fps,
            )
            self.camera.start()
            self.tracker = ObjectTracker(self.cfg.tracker_type)
        if self.args.send:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._start_flight_log()

    def stop(self):
        self._send_safe_exit_frame()
        if self.flight_logger:
            self.flight_logger.close()
            self.flight_logger = None
        if self.sock:
            self.sock.close()
            self.sock = None
        if self.camera:
            self.camera.stop()
            self.camera = None
        if self.joystick is not None:
            self.joystick.close()
        if self.preview_available:
            try:
                import cv2
                cv2.destroyAllWindows()
            except Exception:
                pass

    def run(self):
        self.start()
        period = 1.0 / self.args.loop_hz
        deadline = None if self.args.duration <= 0 else time.monotonic() + self.args.duration
        logger.info(
            "pilot_source=%s device=%s camera=%s send=%s",
            self.args.pilot_source,
            self.args.device,
            self.args.camera,
            self.args.send,
        )
        if not self.args.send:
            logger.info("dry-run mode; add --send to transmit UDP RC packets to SITL")

        try:
            while True:
                now = time.monotonic()
                if deadline is not None and now >= deadline:
                    break

                t_start = now
                pilot_channels = self._read_pilot_channels()
                frame = self.camera.read() if self.camera else None
                result = self._update_vision_state(frame, pilot_channels)
                final_channels = self._mix_channels(pilot_channels, result)

                if self.sock:
                    self._send_rc_frame(final_channels, now=time.monotonic(), reason="loop")

                if self.args.preview and frame is not None and self.preview_available:
                    self._draw_preview(frame, result, pilot_channels, final_channels)

                self._print_status(pilot_channels, final_channels, result)
                self._log_flight_sample(frame, pilot_channels, final_channels, result)

                elapsed = time.monotonic() - t_start
                self.loop_fps = 1.0 / elapsed if elapsed > 0 else 0.0
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            logger.info("interrupted")
        finally:
            self.stop()

    def _safe_exit_channels(self):
        channels = [SAFE_CENTER_PWM] * 16
        channels[ROLL_CH] = SAFE_CENTER_PWM
        channels[PITCH_CH] = SAFE_CENTER_PWM
        channels[THROTTLE_CH] = SAFE_LOW_PWM
        channels[YAW_CH] = SAFE_CENTER_PWM
        channels[ARM_CH] = SAFE_LOW_PWM
        return apply_forced_mode_pwm(channels, self.args.force_mode_pwm)

    def _send_safe_exit_frame(self):
        if not self.sock:
            return False
        channels = self._safe_exit_channels()
        self._send_rc_frame(channels, now=time.monotonic(), reason="safe-exit", check_watchdog=False)
        logger.info("sent safe-exit RC frame throttle-low centered")
        return True

    def _send_rc_frame(self, channels, *, now, reason, check_watchdog=True):
        if self.sock is None:
            return 0
        if check_watchdog and self.last_send_time is not None:
            dt = now - self.last_send_time
            threshold = 2.0 / self.args.loop_hz
            if dt > threshold:
                self.tx_warning_count += 1
                logger.warning(
                    "RC send interval %.3fs exceeded watchdog %.3fs",
                    dt,
                    threshold,
                )
                if self.flight_logger:
                    self.flight_logger.write(
                        "tx_warning",
                        dt=dt,
                        threshold=threshold,
                        reason=reason,
                    )
        sent = self.sock.sendto(pack_rc_packet(channels), (self.args.host, self.args.port))
        self.last_send_time = now
        return sent

    def _read_pilot_channels(self):
        if self.joystick is None:
            channels = self._virtual_pilot_channels()
        else:
            self.joystick.poll(timeout=0.01)
            channels = make_channels(self.joystick, CHANNEL_MAP)
        return apply_forced_mode_pwm(channels, self.args.force_mode_pwm)

    def _virtual_pilot_channels(self):
        elapsed = None
        if self.args.virtual_script == "manual":
            throttle = self.args.virtual_throttle
            arm_pwm = self.args.virtual_arm_pwm
        else:
            elapsed = self._virtual_elapsed()
            throttle, arm_pwm = self._virtual_takeoff_throttle_arm(elapsed)
        if elapsed is None:
            elapsed = self._virtual_elapsed()
        kenet_pwm = self._virtual_kenet_pwm(elapsed)
        return make_virtual_channels(
            roll=self.args.virtual_roll,
            pitch=self.args.virtual_pitch,
            throttle=throttle,
            yaw=self.args.virtual_yaw,
            arm_pwm=arm_pwm,
            kenet_pwm=kenet_pwm,
            mode_pwm=self.args.virtual_mode_pwm,
        )

    def _virtual_kenet_pwm(self, elapsed):
        if self.args.virtual_kenet_delay_seconds <= 0:
            return self.args.virtual_kenet_pwm
        if elapsed < self.args.virtual_kenet_delay_seconds:
            return self.args.virtual_kenet_pre_pwm
        return self.args.virtual_kenet_pwm

    def _virtual_elapsed(self):
        if self.virtual_started is None:
            self.virtual_started = time.monotonic()
        return max(0.0, time.monotonic() - self.virtual_started)

    def _virtual_takeoff_throttle_arm(self, elapsed):
        low_end = self.args.virtual_low_seconds
        arm_end = low_end + self.args.virtual_arm_seconds
        ramp_end = arm_end + self.args.virtual_ramp_seconds
        hold_end = ramp_end + self.args.virtual_hold_seconds
        disarm_end = hold_end + self.args.virtual_disarm_seconds

        if elapsed < low_end:
            return 1000, 1000
        if elapsed < arm_end:
            return 1000, 2000
        if elapsed < ramp_end:
            if self.args.virtual_ramp_seconds <= 0:
                throttle = self.args.virtual_throttle
            else:
                fraction = (elapsed - arm_end) / self.args.virtual_ramp_seconds
                throttle = 1000 + (self.args.virtual_throttle - 1000) * max(0.0, min(1.0, fraction))
            return int(round(throttle)), 2000
        if elapsed < hold_end:
            return self.args.virtual_throttle, 2000
        if elapsed < disarm_end:
            return 1000, 1000
        return 1000, 1000

    def _update_vision_state(self, frame, pilot_channels):
        aux_value = pilot_channels[self.args.aux_ch]
        self.state = state_from_aux(
            aux_value,
            self.args.aux_arm_threshold,
            self.args.aux_track_threshold,
            tracking_inhibited=self.tracking_reentry_blocked,
        )
        if aux_value <= self.args.aux_track_threshold:
            self.tracking_reentry_blocked = False

        if self.state != self.prev_state:
            logger.info("state %s -> %s (AUX CH%d=%d)",
                        STATE_NAMES.get(self.prev_state, "START"),
                        STATE_NAMES[self.state],
                        self.args.aux_ch + 1,
                        aux_value)
            if self.state != TRACKING:
                self._reset_tracking()
            elif frame is not None:
                self._init_tracking(frame)
            self.prev_state = self.state

        if self.args.synthetic_target:
            if self.state == TRACKING:
                result = self._synthetic_track_result()
                self.last_result = result
                if result.found:
                    self.lost_count = 0
                    self.controller.update(result)
                else:
                    self.lost_count += 1
                    self.controller.reset()
                    if self.lost_count >= int(self.args.lost_seconds * self.args.loop_hz):
                        logger.warning(
                            "synthetic target lost for %.1fs; dropping to AI-ARMED",
                            self.args.lost_seconds,
                        )
                        self.tracking_reentry_blocked = True
                        self.state = AI_ARMED
                        self.prev_state = AI_ARMED
                return result
            self.controller.reset()
            self.last_result = TrackResult()
            return self.last_result

        if self.args.no_vision or frame is None or self.tracker is None:
            self.last_result = TrackResult()
            return self.last_result

        if self.state == TRACKING:
            if not self.tracker.is_initialized:
                self._init_tracking(frame)
                if self.tracker is None:
                    self.last_result = TrackResult()
                    return self.last_result
            result = self.tracker.update(frame)
            self.last_result = result
            if result.found:
                self.lost_count = 0
                self.controller.update(result)
            else:
                self.lost_count += 1
                self.controller.reset()
                if self.lost_count >= int(self.args.lost_seconds * self.args.loop_hz):
                    logger.warning(
                        "target lost for %.1fs; dropping to AI-ARMED",
                        self.args.lost_seconds,
                    )
                    self._reset_tracking()
                    self.tracking_reentry_blocked = True
                    self.state = AI_ARMED
                    self.prev_state = AI_ARMED
            return result

        self.controller.reset()
        self.last_result = TrackResult()
        return self.last_result

    def _synthetic_track_result(self):
        self.args.synthetic_elapsed_seconds = self._virtual_elapsed()
        return synthetic_track_result(self.args)

    def _init_tracking(self, frame):
        if self.tracker is None:
            return
        bbox = centered_bbox(frame, self.cfg.track_bbox_size)
        try:
            self.tracker.init(frame, bbox)
        except TrackerUnavailableError as exc:
            self.tracker_error = str(exc)
            logger.error("tracker unavailable: %s", exc)
            logger.error("activate the project venv first: source fpv_env/bin/activate")
            self.tracker = None
            self.controller.reset()
            return
        h_frame, w_frame = frame.shape[:2]
        self.controller.set_frame_center(w_frame, h_frame)
        self.lost_count = 0
        self.tracker_error = None
        logger.info("tracker initialized at center bbox=%s", bbox)

    def _reset_tracking(self):
        if self.tracker:
            self.tracker.reset()
        self.controller.reset()
        self.lost_count = 0

    def _mix_channels(self, pilot_channels, result):
        final_channels = list(pilot_channels)
        self.last_source = "pilot"

        if self.state == TRACKING and result.found:
            ai_channels = self.controller.channels
            final_channels[self.cfg.pitch_ch] = ai_channels[self.cfg.pitch_ch]
            final_channels[self.cfg.yaw_ch] = ai_channels[self.cfg.yaw_ch]
            self.last_source = "kenet"
        elif self.state == TRACKING and self.tracker_error:
            self.last_source = "pilot-tracker-unavailable"
        elif self.state == TRACKING:
            self.last_source = "pilot-target-lost"

        return final_channels

    def _start_flight_log(self):
        if self.args.no_flight_log:
            return
        if self.args.flight_log:
            path = Path(self.args.flight_log)
        else:
            log_dir = resolve_log_dir(self.args.log_dir, REPO_ROOT)
            path = make_log_path(log_dir, "kenet-mixer")
        max_bytes = None if self.args.flight_log_max_mb <= 0 else int(self.args.flight_log_max_mb * 1024 * 1024)
        self.flight_logger = JsonlLogger(path, metadata={
            "tool": "kenet_sitl_mixer",
            "pilot_source": self.args.pilot_source,
            "device": self.args.device,
            "camera": str(self.args.camera),
            "send": self.args.send,
            "force_mode_pwm": self.args.force_mode_pwm,
            "virtual_rc": {
                "script": self.args.virtual_script,
                "roll": self.args.virtual_roll,
                "pitch": self.args.virtual_pitch,
                "throttle": self.args.virtual_throttle,
                "yaw": self.args.virtual_yaw,
                "arm_pwm": self.args.virtual_arm_pwm,
                "kenet_pwm": self.args.virtual_kenet_pwm,
                "kenet_pre_pwm": self.args.virtual_kenet_pre_pwm,
                "kenet_delay_seconds": self.args.virtual_kenet_delay_seconds,
                "mode_pwm": self.args.virtual_mode_pwm,
            },
            "loop_hz": self.args.loop_hz,
            "pid": {
                "yaw": {
                    "kp": self.args.yaw_kp,
                    "ki": self.args.yaw_ki,
                    "kd": self.args.yaw_kd,
                    "limit": self.args.yaw_limit,
                },
                "forward": {
                    "kp": self.args.forward_kp,
                    "ki": self.args.forward_ki,
                    "kd": self.args.forward_kd,
                    "limit": self.args.forward_limit,
                },
            },
        }, flush_every=self.args.flight_log_flush_every,
            flush_interval=self.args.flight_log_flush_seconds,
            max_bytes=max_bytes)
        logger.info("flight log: %s", self.flight_logger.path)

    def _log_flight_sample(self, frame, pilot_channels, final_channels, result):
        if not self.flight_logger:
            return
        now = time.monotonic()
        if now - self.last_flight_log < 1.0 / self.args.flight_log_hz:
            return
        self.last_flight_log = now
        ctrl = self.controller
        frame_shape = None
        if frame is not None:
            h_frame, w_frame = frame.shape[:2]
            frame_shape = [int(w_frame), int(h_frame)]
        self.flight_logger.write(
            "kenet_mixer_sample",
            state=STATE_NAMES[self.state],
            source=self.last_source,
            target_found=bool(result.found),
            target_bbox=list(result.bbox) if result.bbox else None,
            target_center=list(result.center) if result.center else None,
            pilot_channels=list(pilot_channels),
            final_channels=list(final_channels),
            first8={
                "pilot": list(pilot_channels[:8]),
                "final": list(final_channels[:8]),
                "delta": [int(final_channels[i] - pilot_channels[i]) for i in range(8)],
            },
            controller={
                "yaw_error": ctrl.yaw_error,
                "forward_error": ctrl.forward_error,
                "yaw_output": ctrl.yaw_output,
                "forward_output": ctrl.forward_output,
                "channels": ctrl.channels,
            },
            frame_shape=frame_shape,
            loop_fps=self.loop_fps,
            send=bool(self.args.send),
            tx_warning_count=self.tx_warning_count,
            tracker_error=self.tracker_error,
        )

    def _print_status(self, pilot_channels, final_channels, result):
        now = time.monotonic()
        if now - self.last_print < 1.0 / self.args.print_hz:
            return
        self.last_print = now
        target = "found" if result.found else "none"
        print(
            "state=%s source=%s target=%s pilot=%s final=%s fps=%.1f" %
            (
                STATE_NAMES[self.state],
                self.last_source,
                target,
                ",".join(str(ch) for ch in pilot_channels[:8]),
                ",".join(str(ch) for ch in final_channels[:8]),
                self.loop_fps,
            ),
            flush=True,
        )

    def _draw_preview(self, frame, result, pilot_channels, final_channels):
        try:
            import cv2
            h_frame, w_frame = frame.shape[:2]
            cx = w_frame // 2
            cy = h_frame // 2
            cv2.drawMarker(frame, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)

            if self.state == AI_ARMED:
                x, y, w, h = centered_bbox(frame, self.cfg.track_bbox_size)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 1)
                cv2.putText(frame, "init zone", (x, y - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            if result.found:
                x, y, w, h = result.bbox
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (int(result.center[0]), int(result.center[1])), 5, (0, 0, 255), -1)

            cv2.putText(frame, STATE_NAMES[self.state], (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.putText(frame, "src:%s p/y:%d/%d" % (
                self.last_source,
                final_channels[self.cfg.pitch_ch],
                final_channels[self.cfg.yaw_ch],
            ), (10, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (220, 220, 220), 1)
            cv2.imshow("Kenet SITL Mixer", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise KeyboardInterrupt
        except KeyboardInterrupt:
            raise
        except Exception as exc:
            self.preview_available = False
            logger.warning("preview disabled: %s", exc)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pilot-source", choices=["joystick", "virtual"], default="joystick",
                        help="Read pilot RC from a Linux joystick or synthesize it from --virtual-* values")
    parser.add_argument("--device", default="/dev/input/js0")
    parser.add_argument("--camera", default="0")
    parser.add_argument("--tracker", default="CSRT", choices=["CSRT", "KCF"])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9004)
    parser.add_argument("--send", action="store_true",
                        help="Send merged RC packets to Betaflight SITL")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Stop after N seconds; 0 means run until Ctrl-C")
    parser.add_argument("--loop-hz", type=float, default=30.0)
    parser.add_argument("--print-hz", type=float, default=5.0)
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--no-vision", action="store_true",
                        help="Disable camera/tracker and pass pilot RC through")
    parser.add_argument("--synthetic-target", action="store_true",
                        help="Do not open a camera; feed a deterministic found target into the controller")
    parser.add_argument("--synthetic-target-x", type=float, default=340.0,
                        help="Synthetic target center x in pixels")
    parser.add_argument("--synthetic-target-y", type=float, default=240.0,
                        help="Synthetic target center y in pixels")
    parser.add_argument("--synthetic-target-width", type=float, default=110.0,
                        help="Synthetic target bbox width in pixels")
    parser.add_argument("--synthetic-target-height", type=float, default=110.0,
                        help="Synthetic target bbox height in pixels")
    parser.add_argument("--synthetic-target-delay-seconds", type=float, default=0.0,
                        help="Keep the synthetic target centered for this long before applying the configured offset")
    parser.add_argument("--synthetic-target-loss-after-seconds", type=float, default=None,
                        help="After this elapsed time, make the synthetic target disappear")

    parser.add_argument("--frame-width", type=int, default=640)
    parser.add_argument("--frame-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=int, default=30)
    parser.add_argument("--track-size", type=int, default=100)
    parser.add_argument("--desired-target-width", type=float, default=120.0)
    parser.add_argument("--lost-seconds", type=float, default=2.0)

    parser.add_argument("--aux-ch", type=int, default=5,
                        help="0-indexed Kenet state AUX channel; default CH6/AUX2")
    parser.add_argument("--aux-arm-threshold", type=int, default=1300)
    parser.add_argument("--aux-track-threshold", type=int, default=1700)
    parser.add_argument("--force-mode-pwm", type=int, default=None,
                        help="Force CH7/AUX3 to this PWM value, e.g. 1500 for ANGLE mode tests")
    parser.add_argument("--virtual-roll", type=int, default=1500)
    parser.add_argument("--virtual-script", choices=["manual", "takeoff"], default="manual",
                        help="Virtual pilot mode: fixed manual frame or boot/arm/ramp/hold/disarm takeoff script")
    parser.add_argument("--virtual-pitch", type=int, default=1500)
    parser.add_argument("--virtual-throttle", type=int, default=1000)
    parser.add_argument("--virtual-yaw", type=int, default=1500)
    parser.add_argument("--virtual-arm-pwm", type=int, default=1000)
    parser.add_argument("--virtual-kenet-pwm", type=int, default=1000)
    parser.add_argument("--virtual-kenet-pre-pwm", type=int, default=1000,
                        help="Kenet state PWM before --virtual-kenet-delay-seconds elapses")
    parser.add_argument("--virtual-kenet-delay-seconds", type=float, default=0.0,
                        help="Delay switching virtual Kenet state to --virtual-kenet-pwm")
    parser.add_argument("--virtual-mode-pwm", type=int, default=1500)
    parser.add_argument("--virtual-low-seconds", type=float, default=5.0)
    parser.add_argument("--virtual-arm-seconds", type=float, default=3.0)
    parser.add_argument("--virtual-ramp-seconds", type=float, default=3.0)
    parser.add_argument("--virtual-hold-seconds", type=float, default=8.0)
    parser.add_argument("--virtual-disarm-seconds", type=float, default=1.0)

    parser.add_argument("--yaw-kp", type=float, default=0.8)
    parser.add_argument("--yaw-ki", type=float, default=0.05)
    parser.add_argument("--yaw-kd", type=float, default=0.15)
    parser.add_argument("--yaw-limit", type=float, default=300.0)
    parser.add_argument("--forward-kp", type=float, default=0.4)
    parser.add_argument("--forward-ki", type=float, default=0.02)
    parser.add_argument("--forward-kd", type=float, default=0.1)
    parser.add_argument("--forward-limit", type=float, default=250.0)
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    parser.add_argument("--log-dir", default=os.environ.get("KENET_SITL_LOG_DIR"),
                        help="Directory for SITL JSONL logs; default logs/sitl")
    parser.add_argument("--flight-log", default=None,
                        help="Exact JSONL path for detailed mixer samples")
    parser.add_argument("--no-flight-log", action="store_true",
                        help="Disable detailed JSONL flight logging")
    parser.add_argument("--flight-log-hz", type=float, default=10.0,
                        help="Detailed JSONL sample rate")
    parser.add_argument("--flight-log-flush-every", type=int, default=10,
                        help="Flush detailed JSONL logs every N records")
    parser.add_argument("--flight-log-flush-seconds", type=float, default=1.0,
                        help="Flush detailed JSONL logs at least this often; 0 disables time-based flushing")
    parser.add_argument("--flight-log-max-mb", type=float, default=50.0,
                        help="Rotate detailed JSONL logs after this many MiB; 0 disables rotation")

    args = parser.parse_args()
    if args.loop_hz <= 0:
        parser.error("--loop-hz must be positive")
    if args.print_hz <= 0:
        parser.error("--print-hz must be positive")
    if args.synthetic_target and args.no_vision:
        parser.error("--synthetic-target cannot be combined with --no-vision")
    args.synthetic_elapsed_seconds = 0.0
    if not 0 <= args.aux_ch < 8:
        parser.error("--aux-ch must be between 0 and 7")
    if args.lost_seconds <= 0:
        parser.error("--lost-seconds must be positive")
    if args.flight_log_hz <= 0:
        parser.error("--flight-log-hz must be positive")
    if args.flight_log_flush_every <= 0:
        parser.error("--flight-log-flush-every must be positive")
    if args.flight_log_flush_seconds < 0:
        parser.error("--flight-log-flush-seconds must be non-negative")
    if args.flight_log_max_mb < 0:
        parser.error("--flight-log-max-mb must be non-negative")
    if args.force_mode_pwm is not None and not 1000 <= args.force_mode_pwm <= 2000:
        parser.error("--force-mode-pwm must be between 1000 and 2000")
    for name in (
        "virtual_roll",
        "virtual_pitch",
        "virtual_throttle",
        "virtual_yaw",
        "virtual_arm_pwm",
        "virtual_kenet_pwm",
        "virtual_kenet_pre_pwm",
        "virtual_mode_pwm",
    ):
        if not 1000 <= getattr(args, name) <= 2000:
            parser.error("--%s must be between 1000 and 2000" % name.replace("_", "-"))
    for name in (
        "virtual_low_seconds",
        "virtual_arm_seconds",
        "virtual_ramp_seconds",
        "virtual_hold_seconds",
        "virtual_disarm_seconds",
        "virtual_kenet_delay_seconds",
    ):
        if getattr(args, name) < 0:
            parser.error("--%s must be non-negative" % name.replace("_", "-"))
    for name in ("synthetic_target_width", "synthetic_target_height"):
        if getattr(args, name) <= 0:
            parser.error("--%s must be positive" % name.replace("_", "-"))
    if args.synthetic_target_delay_seconds < 0:
        parser.error("--synthetic-target-delay-seconds must be non-negative")
    if args.synthetic_target_loss_after_seconds is not None and args.synthetic_target_loss_after_seconds < 0:
        parser.error("--synthetic-target-loss-after-seconds must be non-negative")
    return args


def main():
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    mixer = KenetSitlMixer(args)
    mixer.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
