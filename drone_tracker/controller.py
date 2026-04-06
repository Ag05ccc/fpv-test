"""
controller - PID controllers + flight control logic.

Computes RC channel outputs from tracking results.
"""

import time
import logging
from dataclasses import dataclass

logger = logging.getLogger(__name__)


# ── PID ──────────────────────────────────────────────────────────────

@dataclass
class PIDGains:
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    output_min: float = -500.0
    output_max: float = 500.0
    integral_limit: float = 200.0  # anti-windup clamp


class PIDController:
    """Discrete PID with anti-windup and derivative-on-error."""

    def __init__(self, gains):
        self.gains = gains
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def update(self, error, t=None):
        now = t if t is not None else time.monotonic()
        if self._prev_time is None:
            self._prev_time = now
            self._prev_error = error
            return 0.0

        dt = now - self._prev_time
        if dt <= 0:
            return 0.0

        p = self.gains.kp * error

        self._integral += error * dt
        self._integral = max(-self.gains.integral_limit,
                             min(self.gains.integral_limit, self._integral))
        i = self.gains.ki * self._integral

        d = self.gains.kd * (error - self._prev_error) / dt

        self._prev_error = error
        self._prev_time = now

        output = p + i + d
        return max(self.gains.output_min, min(self.gains.output_max, output))


# ── Flight Controller ────────────────────────────────────────────────

class FlightController:
    """Converts TrackResult into rate-limited RC channel commands.

    Yaw  -> direction (horizontal pixel error).
    Pitch -> forward/backward (target apparent size vs desired width).
    Roll  -> clamped within +/-roll_limit_deg.

    Takes a PipelineConfig (or any object with the same attributes).
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.yaw_pid = PIDController(cfg.yaw_pid)
        self.forward_pid = PIDController(cfg.forward_pid)

        self._channels = [cfg.rc_center] * cfg.num_channels
        self._channels[cfg.throttle_ch] = cfg.throttle_neutral

        self._cx = cfg.frame_width / 2.0
        self._cy = cfg.frame_height / 2.0
        self._roll_max_offset = int(500 * cfg.roll_limit_deg / cfg.max_angle_deg)
        self._prev_time = None

        # Exposed state (read by pipeline / GCS)
        self.yaw_error = 0.0
        self.forward_error = 0.0
        self.yaw_output = 0.0
        self.forward_output = 0.0
        self.target_found = False

    @property
    def channels(self):
        return list(self._channels)

    def reset(self):
        """Return all channels to neutral."""
        self.yaw_pid.reset()
        self.forward_pid.reset()
        self._channels = [self.cfg.rc_center] * self.cfg.num_channels
        self._channels[self.cfg.throttle_ch] = self.cfg.throttle_neutral
        self._prev_time = None

    def update(self, result):
        """Compute one control cycle. Updates self.channels and error state."""
        now = time.monotonic()
        dt = (now - self._prev_time) if self._prev_time else 0.0
        self._prev_time = now

        self.target_found = result.found
        self.yaw_error = 0.0
        self.forward_error = 0.0
        self.yaw_output = 0.0
        self.forward_output = 0.0

        if not result.found:
            self.yaw_pid.reset()
            self.forward_pid.reset()
            yaw_target = float(self.cfg.rc_center)
            pitch_target = float(self.cfg.rc_center)
        else:
            # Yaw: horizontal error
            self.yaw_error = result.center[0] - self._cx
            if abs(self.yaw_error) < self.cfg.deadband:
                self.yaw_error = 0.0
            self.yaw_output = self.yaw_pid.update(self.yaw_error)
            yaw_target = self.cfg.rc_center + self.yaw_output

            # Pitch: forward/backward by target apparent size
            target_w = float(result.bbox[2])
            self.forward_error = self.cfg.desired_target_width - target_w
            if abs(self.forward_error) < self.cfg.size_deadband:
                self.forward_error = 0.0
            self.forward_output = self.forward_pid.update(self.forward_error)
            pitch_target = self.cfg.rc_center + self.forward_output

        # Rate limiting
        if dt > 0:
            max_delta = self.cfg.max_rc_rate * dt
            self._channels[self.cfg.yaw_ch] = _rate_limit(
                self._channels[self.cfg.yaw_ch], yaw_target, max_delta)
            self._channels[self.cfg.pitch_ch] = _rate_limit(
                self._channels[self.cfg.pitch_ch], pitch_target, max_delta)
        else:
            self._channels[self.cfg.yaw_ch] = int(yaw_target)
            self._channels[self.cfg.pitch_ch] = int(pitch_target)

        # Roll clamp
        roll_min = self.cfg.rc_center - self._roll_max_offset
        roll_max = self.cfg.rc_center + self._roll_max_offset
        self._channels[self.cfg.roll_ch] = max(
            roll_min, min(roll_max, self._channels[self.cfg.roll_ch]))


def _rate_limit(current, target, max_delta):
    delta = target - current
    if abs(delta) > max_delta:
        delta = max_delta if delta > 0 else -max_delta
    return int(current + delta)
