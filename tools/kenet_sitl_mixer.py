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

from sitl_rc_bridge import CHANNEL_MAP, LinuxJoystick, make_channels, pack_rc_packet

from kenet.camera import CameraCapture
from kenet.controller import FlightController, PIDGains
from kenet.pipeline import AI_ARMED, IDLE, TRACKING, PipelineConfig
from kenet.tracker import ObjectTracker, TrackerUnavailableError, TrackResult


logger = logging.getLogger("kenet_sitl_mixer")


STATE_NAMES = {
    IDLE: "IDLE",
    AI_ARMED: "AI-ARMED",
    TRACKING: "TRACKING",
}


def parse_camera_source(value):
    return int(value) if str(value).isdigit() else value


def state_from_aux(value, arm_threshold=1300, track_threshold=1700):
    if value >= track_threshold:
        return TRACKING
    if value >= arm_threshold:
        return AI_ARMED
    return IDLE


def centered_bbox(frame, size):
    h_frame, w_frame = frame.shape[:2]
    size = int(size)
    x = max(0, w_frame // 2 - size // 2)
    y = max(0, h_frame // 2 - size // 2)
    return (x, y, size, size)


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

        self.joystick = LinuxJoystick(args.device)
        self.camera = None
        self.tracker = None
        self.controller = FlightController(self.cfg)
        self.sock = None

        self.state = IDLE
        self.prev_state = None
        self.lost_count = 0
        self.loop_fps = 0.0
        self.last_print = 0.0
        self.preview_available = True
        self.last_result = TrackResult()
        self.last_source = "pilot"
        self.tracker_error = None

    def start(self):
        self.joystick.open()
        if not self.args.no_vision:
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

    def stop(self):
        if self.sock:
            self.sock.close()
            self.sock = None
        if self.camera:
            self.camera.stop()
            self.camera = None
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
        logger.info("device=%s camera=%s send=%s", self.args.device, self.args.camera, self.args.send)
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
                    self.sock.sendto(pack_rc_packet(final_channels), (self.args.host, self.args.port))

                if self.args.preview and frame is not None and self.preview_available:
                    self._draw_preview(frame, result, pilot_channels, final_channels)

                self._print_status(pilot_channels, final_channels, result)

                elapsed = time.monotonic() - t_start
                self.loop_fps = 1.0 / elapsed if elapsed > 0 else 0.0
                sleep_time = period - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            logger.info("interrupted")
        finally:
            self.stop()

    def _read_pilot_channels(self):
        self.joystick.poll(timeout=0.01)
        return make_channels(self.joystick, CHANNEL_MAP)

    def _update_vision_state(self, frame, pilot_channels):
        aux_value = pilot_channels[self.args.aux_ch]
        self.state = state_from_aux(
            aux_value,
            self.args.aux_arm_threshold,
            self.args.aux_track_threshold,
        )

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

        if self.args.no_vision or frame is None or self.tracker is None:
            self.last_result = TrackResult()
            return self.last_result

        if self.state == TRACKING:
            if not self.tracker._initialized:
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
                    logger.warning("target lost for %.1fs; reinitializing tracker", self.args.lost_seconds)
                    self._reset_tracking()
            return result

        self.controller.reset()
        self.last_result = TrackResult()
        return self.last_result

    def _init_tracking(self, frame):
        if self.tracker is None:
            return
        bbox = centered_bbox(frame, self.cfg.track_bbox_size)
        try:
            self.tracker.init(frame, bbox)
        except TrackerUnavailableError as exc:
            self.tracker_error = str(exc)
            logger.error("tracker unavailable: %s", exc)
            logger.error("activate the project venv first: source /home/gz/fpv-test/fpv_env/bin/activate")
            self.tracker = None
            self.controller.reset()
            return
        h_frame, w_frame = frame.shape[:2]
        self.controller._cx = w_frame / 2.0
        self.controller._cy = h_frame / 2.0
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

    parser.add_argument("--yaw-kp", type=float, default=0.8)
    parser.add_argument("--yaw-ki", type=float, default=0.05)
    parser.add_argument("--yaw-kd", type=float, default=0.15)
    parser.add_argument("--yaw-limit", type=float, default=300.0)
    parser.add_argument("--forward-kp", type=float, default=0.4)
    parser.add_argument("--forward-ki", type=float, default=0.02)
    parser.add_argument("--forward-kd", type=float, default=0.1)
    parser.add_argument("--forward-limit", type=float, default=250.0)
    parser.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])

    args = parser.parse_args()
    if args.loop_hz <= 0:
        parser.error("--loop-hz must be positive")
    if args.print_hz <= 0:
        parser.error("--print-hz must be positive")
    if not 0 <= args.aux_ch < 8:
        parser.error("--aux-ch must be between 0 and 7")
    if args.lost_seconds <= 0:
        parser.error("--lost-seconds must be positive")
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
